import cv2
import mediapipe as mp
import numpy as np
import math
from pythonosc import udp_client


# ================= 配置区域 =================
UE_IP = "127.0.0.1"
UE_PORT = 8000

# 状态常量
MODE_HOVER = 0
MODE_CRUISE = 1
MODE_SPEED_SET = 2
MODE_SINGLE_SELECT = 3

# 防抖阈值
GESTURE_CONFIRM_FRAMES = 15

class FinalDroneController:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            model_complexity=1,
            max_num_hands=2, 
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.osc_client = udp_client.SimpleUDPClient(UE_IP, UE_PORT)

        # 核心状态
        self.flight_mode = MODE_HOVER 
        self.last_flight_mode = MODE_HOVER 
        
        self.right_hand_active = False 
        self.speed_level = 1     
        self.drone_id = 0        
        self.formation = "TRIANGLE" 

        # 计数器
        self.left_history = None
        self.left_count = 0
        self.right_history = None
        self.right_count = 0
        
        # 瞬时反馈计时
        self.feedback_timer = 0
        self.feedback_msg = ""

    def _get_fingers(self, lm, label):
        """获取5指状态 [Thumb, Index, Middle, Ring, Pinky]"""
        fingers = []
        tips = [8, 12, 16, 20]
        bases = [6, 10, 14, 18]
        
        if label == "Right":
            fingers.append(lm[4].x < lm[3].x)
        else: # Left
            fingers.append(lm[4].x > lm[3].x)

        for t, b in zip(tips, bases):
            fingers.append(lm[t].y < lm[b].y)
        return fingers

    def classify_gesture(self, lm, label):
        """手势分类器"""
        fingers = self._get_fingers(lm, label)
        count = fingers.count(True)
        
        # 1. 握拳
        if not any(fingers[1:]): 
            if fingers[0]: return "THUMB_ONLY", 1 # 左手悬停
            return "FIST", 0
            
        # 2. 全掌
        if count == 5: return "PALM", 5

        # 3. 右手数字
        if label == "Right":
            if count == 1 and fingers[1]: return "ONE", 1
            if count == 2 and fingers[1] and fingers[2]: return "TWO", 2
            if count == 3 and fingers[1] and fingers[2] and fingers[3]: return "THREE", 3
            if count == 4: return "FOUR", 4

        # 4. 左手功能
        if label == "Left":
            # A. 长效飞行模式
            # 悬停: 仅拇指
            if fingers[0] and not any(fingers[1:]): return "THUMB_ONLY", 0
            # 巡航: 拇指+食指 (L型)
            if fingers[0] and fingers[1] and not fingers[2] and not fingers[3] and not fingers[4]: return "GUN_L", 0
            
            # B. 瞬时调节模式
            # 调速: 蜘蛛侠 (食指+小指)
            if fingers[1] and fingers[4] and not fingers[2] and not fingers[3]: return "SPIDERMAN", 0
            # 选机: 拇指+食指+中指 (无名指弯曲)
            if fingers[0] and fingers[1] and fingers[2] and not fingers[3]: return "THUMB_3", 0
            
            # C. 队形变换 (区分三指和两指)
            # 纵向 (Vertical): 食+中+无名 (三指, 拇指弯)
            if fingers[1] and fingers[2] and fingers[3] and not fingers[0] and not fingers[4]: 
                return "THREE_ROW_V", 0
            
            # [修改点] 横向 (Horizontal): 食+中 (两指/耶, 拇指弯, 无名指弯)
            if fingers[1] and fingers[2] and not fingers[0] and not fingers[3] and not fingers[4]: 
                return "TWO_ROW_H", 0

        return "UNKNOWN", count

    def calculate_pose(self, lm):
        """右手姿态解算"""
        p0 = np.array([lm[0].x, lm[0].y, lm[0].z])
        p5 = np.array([lm[5].x, lm[5].y, lm[5].z])
        p17 = np.array([lm[17].x, lm[17].y, lm[17].z])
        
        vec_u = p5 - p0
        vec_v = p17 - p0
        normal = np.cross(vec_u, vec_v)
        normal /= np.linalg.norm(normal)
        
        pitch = math.asin(max(-1.0, min(1.0, normal[1])))
        roll = math.atan2(normal[0], normal[2])
        yaw = math.atan2(vec_u[1], vec_u[0])
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def draw_hud(self, frame, roll, pitch, yaw, hand_center):
        """绘制 HUD"""
        h, w, c = frame.shape
        cx, cy = hand_center
        
        # 坐标轴
        axis_len = 80
        cv2.line(frame, (cx - axis_len, cy), (cx + axis_len, cy), (100, 100, 100), 2)
        cv2.line(frame, (cx, cy - axis_len), (cx, cy + axis_len), (100, 100, 100), 2)
        
        # 动态点
        dx = int(np.clip(roll * 2.0, -axis_len, axis_len))
        dy = int(np.clip(pitch * 2.0, -axis_len, axis_len))
        cv2.line(frame, (cx, cy), (cx + dx, cy + dy), (0, 255, 255), 2)
        cv2.circle(frame, (cx + dx, cy + dy), 8, (0, 255, 0), -1)
        
        # 数值
        text_lines = [f"R: {int(roll):+3d}", f"P: {int(pitch):+3d}", f"Y: {int(yaw):+3d}"]
        for i, line in enumerate(text_lines):
            cv2.putText(frame, line, (cx + axis_len + 10, cy - 40 + i * 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def process_frame(self, frame):
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        hands_data = {'Left': {'gesture': None, 'lm': None}, 
                      'Right': {'gesture': None, 'val': 0, 'lm': None}}

        if results.multi_hand_landmarks:
            for idx, handedness in enumerate(results.multi_handedness):
                label = handedness.classification[0].label
                lm = results.multi_hand_landmarks[idx].landmark
                self.mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[idx], self.mp_hands.HAND_CONNECTIONS)
                
                g_name, g_val = self.classify_gesture(lm, label)
                hands_data[label]['gesture'] = g_name
                hands_data[label]['val'] = g_val
                hands_data[label]['lm'] = lm

        # ================== 逻辑核心 ==================
        g_left = hands_data['Left']['gesture']
        g_right = hands_data['Right']['gesture']
        r_lm = hands_data['Right']['lm']

        # -------------------------------------------------
        # 1. 队形变换
        # -------------------------------------------------
        # 双手三角
        if g_left == "PALM" and g_right == "PALM":
            self.formation = "TRIANGLE"
        
        # 左手单手线性
        elif g_left == "THREE_ROW_V":
            self.formation = "LINE_VERTICAL"
        elif g_left == "TWO_ROW_H": # [修改点] 横向改为两指
            self.formation = "LINE_HORIZONTAL"

        # -------------------------------------------------
        # 2. 左手：模式控制
        # -------------------------------------------------
        stable_left = None
        if g_left is not None and g_left != "UNKNOWN":
            if g_left == self.left_history:
                self.left_count += 1
            else:
                self.left_history = g_left
                self.left_count = 0
            
            if self.left_count > GESTURE_CONFIRM_FRAMES:
                stable_left = g_left
        else:
            self.left_count = 0
            self.left_history = None

        # --- 模式状态机 ---
        if stable_left:
            # A. 长效模式
            if stable_left == "THUMB_ONLY":
                self.flight_mode = MODE_HOVER
                self.last_flight_mode = MODE_HOVER
            elif stable_left == "GUN_L":
                self.flight_mode = MODE_CRUISE
                self.last_flight_mode = MODE_CRUISE
            
            # B. 瞬时调节模式
            elif stable_left == "SPIDERMAN":
                self.flight_mode = MODE_SPEED_SET
            elif stable_left == "THUMB_3":
                self.flight_mode = MODE_SINGLE_SELECT
        
        else:
            # C. 放手回退
            if self.flight_mode in [MODE_SPEED_SET, MODE_SINGLE_SELECT]:
                self.flight_mode = self.last_flight_mode

        # -------------------------------------------------
        # 3. 右手：安全锁
        # -------------------------------------------------
        if g_right in ["FIST", "PALM"]:
            if g_right == self.right_history:
                self.right_count += 1
            else:
                self.right_history = g_right
                self.right_count = 0
            
            if self.right_count > GESTURE_CONFIRM_FRAMES:
                color = (0, 0, 255) if g_right == "FIST" else (255, 255, 0)
                cv2.rectangle(frame, (w-150, 130), (w-150 + self.right_count * 2, 140), color, -1)
                
                if g_right == "FIST": self.right_hand_active = False 
                elif g_right == "PALM": self.right_hand_active = True 
        else:
            self.right_count = 0

        # -------------------------------------------------
        # 4. 执行控制
        # -------------------------------------------------
        control_data = {"roll": 0, "pitch": 0, "yaw": 0, "throttle": 0}
        msg_action = "WAITING"

        if self.right_hand_active and g_right:
            # A. 调速
            if self.flight_mode == MODE_SPEED_SET:
                if 1 <= hands_data['Right']['val'] <= 5:
                    self.speed_level = hands_data['Right']['val']
                msg_action = f"SET SPEED: {self.speed_level}"
            
            # B. 选机 (瞬时反馈 + 自动回退)
            elif self.flight_mode == MODE_SINGLE_SELECT:
                val = hands_data['Right']['val']
                if 1 <= val <= 3:
                    self.drone_id = val
                    self.feedback_msg = f"ID {val} OUT!"
                    self.feedback_timer = 40 
                    self.flight_mode = self.last_flight_mode 
                    
                elif val == 5:
                    self.drone_id = 0
                    self.feedback_msg = "ALL REGROUP!"
                    self.feedback_timer = 40
                    self.flight_mode = self.last_flight_mode
                
                msg_action = "SELECTING..."
            
            # C. 飞行
            else:
                if g_right != "FIST":
                    r, p, y = self.calculate_pose(r_lm)
                    control_data = {"roll": r, "pitch": p, "yaw": y}
                    
                    if self.flight_mode == MODE_CRUISE:
                        control_data["throttle"] = self.speed_level * 0.2
                        msg_action = "FLYING (CRUISE)"
                    else:
                        control_data["throttle"] = 0
                        msg_action = "HOVERING"
                    
                    if r_lm:
                        center = (int(r_lm[9].x * w), int(r_lm[9].y * h))
                        self.draw_hud(frame, r, p, y, center)

        if not self.right_hand_active:
            msg_action = "LOCKED"

        # -------------------------------------------------
        # 5. UI & OSC
        # -------------------------------------------------
        cv2.rectangle(frame, (0, 0), (w, 100), (30, 30, 30), -1)
        
        mode_names = {MODE_HOVER: "HOVER", MODE_CRUISE: "CRUISE", 
                      MODE_SPEED_SET: "SET SPEED", MODE_SINGLE_SELECT: "SELECT"}
        cv2.putText(frame, f"MODE: {mode_names[self.flight_mode]}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        lock_str = "UNLOCKED" if self.right_hand_active else "LOCKED"
        lock_col = (0, 255, 255) if self.right_hand_active else (0, 0, 255)
        cv2.putText(frame, lock_str, (w-200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, lock_col, 2)

        status_str = f"Act: {msg_action} | SPD: {self.speed_level} | FORM: {self.formation} | ID: {self.drone_id}"
        cv2.putText(frame, status_str, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # 反馈消息
        if self.feedback_timer > 0:
            cv2.putText(frame, self.feedback_msg, (w//2 - 150, h//2 + 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            self.feedback_timer -= 1

        # 队形反馈
        if g_left == "THREE_ROW_V" or g_left == "TWO_ROW_H":
             cv2.putText(frame, f"ALIGN: {self.formation}", (w//2 - 100, h//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        # OSC
        form_map = {"TRIANGLE": 1.0, "LINE_VERTICAL": 2.0, "LINE_HORIZONTAL": 3.0}
        osc_msg = [
            float(self.flight_mode),
            float(self.speed_level),
            float(form_map.get(self.formation, 1.0)),
            float(self.drone_id),
            float(control_data["roll"]),
            float(control_data["pitch"]),
            float(control_data["yaw"]),
            float(control_data["throttle"])
        ]
        self.osc_client.send_message("/drone/control", osc_msg)

        return frame


# 1. 修改摄像头打开方式（在文件末尾）
if __name__ == "__main__":
    controller = FinalDroneController()
    cap = cv2.VideoCapture(0)

    # 设置窗口
    cv2.namedWindow('Drone HUD Control', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drone HUD Control', 800, 600)

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("读取帧失败")
            break

        frame = controller.process_frame(frame)
        if frame is not None:
            cv2.imshow('Drone HUD Control', frame)

        key = cv2.waitKey(5) & 0xFF
        if key == 27 or key == ord('q'):  # ESC 或 q
            break

    cap.release()
    cv2.destroyAllWindows()