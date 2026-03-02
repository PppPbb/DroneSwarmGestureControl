"""
File Name: HandGestureRecog.py
Runs on: Mac
Function: Hand gesture recognition, and sending commands to Windows
"""

import cv2
import mediapipe as mp
import numpy as np
import math
from pythonosc import udp_client

# ================= Configuration Area =================
# [IMPORTANT] Please confirm the Windows PC IP
UE_IP = "192.168.59.143"
UE_PORT = 8000

# State Constants
MODE_HOVER = 0
MODE_CRUISE = 1
MODE_SPEED_SET = 2
MODE_SINGLE_SELECT = 3

# Debounce Threshold
GESTURE_CONFIRM_FRAMES = 10  # Lower frame count to improve formation recognition response speed

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

        # Core States
        self.flight_mode = MODE_HOVER
        self.last_flight_mode = MODE_HOVER
        
        self.right_hand_active = False
        self.speed_level = 1
        self.drone_id = 0
        self.formation = "TRIANGLE"

        # Counters
        self.left_history = None
        self.left_count = 0
        self.right_history = None
        self.right_count = 0
        
        # Instant feedback timer (used to display successful formation switch)
        self.feedback_timer = 0
        self.feedback_msg = ""

        # ================= Tactical Control Parameters =================
        # Define Deadzone
        self.CALIB_PITCH_MIN = -10.0
        self.CALIB_PITCH_MAX = 5.0
        self.CALIB_YAW_MIN = -10.0
        self.CALIB_YAW_MAX = 5.0

        # Sensitivity
        self.SENS_PITCH = 0.05 # Ascent/Descent Sensitivity
        self.SENS_YAW = 0.05   # Yaw/Turning Sensitivity

    def _get_fingers(self, lm, label):
        """Get 5-finger state"""
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
        """Gesture Classifier"""
        fingers = self._get_fingers(lm, label)
        count = fingers.count(True)
        
        # 1. Fist (FIST)
        if not any(fingers[1:]):
            if fingers[0]: return "THUMB_ONLY", 1
            return "FIST", 0
            
        # 2. Full Palm (PALM)
        if count == 5: return "PALM", 5

        # 3. Numbers/Common fingers (1-4)
        # Recognize basic number gestures for both hands, used for combination logic
        if count == 1 and fingers[1]: return "ONE", 1
        if count == 2 and fingers[1] and fingers[2]: return "TWO", 2
        if count == 3 and fingers[1] and fingers[2] and fingers[3]: return "THREE", 3
        if count == 4: return "FOUR", 4

        # 4. Left hand special functions (retain original)
        if label == "Left":
            if fingers[0] and fingers[1] and not fingers[2]: return "GUN_L", 0
            if fingers[1] and fingers[4] and not fingers[2]: return "SPIDERMAN", 0
            if fingers[0] and fingers[1] and fingers[2] and not fingers[3]: return "THUMB_3", 0

        return "UNKNOWN", count

    def calculate_pose(self, lm):
        """Right hand pose calculation"""
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
        h, w, c = frame.shape
        cx, cy = hand_center
        
        axis_len = 80
        cv2.line(frame, (cx, cy - axis_len), (cx, cy + axis_len), (100, 100, 100), 2)
        cv2.line(frame, (cx - axis_len, cy), (cx + axis_len, cy), (60, 60, 60), 1)
        
        # Draw deadzone box
        y_min = int(self.CALIB_PITCH_MIN * 2.0)
        y_max = int(self.CALIB_PITCH_MAX * 2.0)
        x_min = int(self.CALIB_YAW_MIN * 2.0)
        x_max = int(self.CALIB_YAW_MAX * 2.0)
        cv2.rectangle(frame, (cx + x_min, cy + y_min), (cx + x_max, cy + y_max), (0, 100, 0), 1)

        dx = int(np.clip(yaw * 2.0, -axis_len, axis_len))
        dy = int(np.clip(pitch * 2.0, -axis_len, axis_len))
        
        color = (0, 255, 0)
        if pitch < self.CALIB_PITCH_MIN or pitch > self.CALIB_PITCH_MAX or \
           yaw < self.CALIB_YAW_MIN or yaw > self.CALIB_YAW_MAX:
            color = (0, 0, 255)

        cv2.line(frame, (cx, cy), (cx + dx, cy + dy), (0, 255, 255), 2)
        cv2.circle(frame, (cx + dx, cy + dy), 8, color, -1)
        
        text_lines = [f"P: {int(pitch):+3d}", f"Y: {int(yaw):+3d}"]
        for i, line in enumerate(text_lines):
            cv2.putText(frame, line, (cx + axis_len + 10, cy - 20 + i * 25),
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

        g_left = hands_data['Left']['gesture']
        g_right = hands_data['Right']['gesture']
        r_lm = hands_data['Right']['lm']

        # =========================================================
        # 1. Formation Change (Highest priority | Fast recognition | State retention)
        # =========================================================
        # Switch state immediately upon detecting a single valid frame, no need to hold the gesture
        
        # A. Triangle (△): Both hands full palm
        if g_left == "PALM" and g_right == "PALM":
            self.formation = "TRIANGLE"
            self.feedback_msg = "FORM: TRIANGLE"
            self.feedback_timer = 30 # Display for 1.5 seconds

        # B. Horizontal Line (—): Both hands index fingers
        elif g_left == "ONE" and g_right == "ONE":
            self.formation = "LINE_HORIZONTAL"
            self.feedback_msg = "FORM: HORIZONTAL"
            self.feedback_timer = 30
            
        # C. Vertical Line (｜): Left hand two fingers (Index + Middle)
        elif g_left == "TWO":
            self.formation = "LINE_VERTICAL"
            self.feedback_msg = "FORM: VERTICAL"
            self.feedback_timer = 30

        # =========================================================
        # 2. Left hand mode switch (Debouncing)
        # =========================================================
        stable_left = None
        if g_left is not None and g_left != "UNKNOWN":
            if g_left == self.left_history: self.left_count += 1
            else:
                self.left_history = g_left
                self.left_count = 0
            if self.left_count > GESTURE_CONFIRM_FRAMES: stable_left = g_left
        else:
            self.left_count = 0
            self.left_history = None

        if stable_left:
            if stable_left == "THUMB_ONLY":
                self.flight_mode = MODE_HOVER
                self.last_flight_mode = MODE_HOVER
            elif stable_left == "GUN_L":
                self.flight_mode = MODE_CRUISE
                self.last_flight_mode = MODE_CRUISE
            elif stable_left == "SPIDERMAN": self.flight_mode = MODE_SPEED_SET
            elif stable_left == "THUMB_3": self.flight_mode = MODE_SINGLE_SELECT
        else:
            if self.flight_mode in [MODE_SPEED_SET, MODE_SINGLE_SELECT]:
                self.flight_mode = self.last_flight_mode

        # =========================================================
        # 3. Right hand activation and real-time control
        # =========================================================
        # Right hand debounce
        if g_right in ["FIST", "PALM"]:
            if g_right == self.right_history: self.right_count += 1
            else:
                self.right_history = g_right
                self.right_count = 0
            
            if self.right_count > GESTURE_CONFIRM_FRAMES:
                if g_right == "FIST": self.right_hand_active = False
                elif g_right == "PALM": self.right_hand_active = True
        else:
            self.right_count = 0

        # Construct data packet
        control_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        
        if self.right_hand_active and g_right:
            # Instant adjustment mode
            if self.flight_mode == MODE_SPEED_SET:
                if 1 <= hands_data['Right']['val'] <= 5: self.speed_level = hands_data['Right']['val']
            
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
            
            # === C. Core Flight Logic (Cruise control + Vertical ascent/descent) ===
            else:
                if g_right == "FIST":
                    self.feedback_msg = "!!! EMERGENCY !!!"
                    self.feedback_timer = 5
                    cv2.rectangle(frame, (0, 0), (w, h), (0, 0, 255), 5)
                else:
                    r, p, y = self.calculate_pose(r_lm)
                    
                    # --- 1. Pitch -> Controls vertical ascent/descent only (VZ) ---
                    # Completely decoupled: Pitch no longer affects forward speed
                    val_pitch = 0.0
                    if p < self.CALIB_PITCH_MIN:   # Hand tilts up -> Ascend
                        val_pitch = (p - self.CALIB_PITCH_MIN) * self.SENS_PITCH
                    elif p > self.CALIB_PITCH_MAX: # Hand tilts down -> Descend
                        val_pitch = (p - self.CALIB_PITCH_MAX) * self.SENS_PITCH
                    
                    # --- 2. Yaw -> Left/Right turning ---
                    val_yaw = 0.0
                    if y < self.CALIB_YAW_MIN: val_yaw = (y - self.CALIB_YAW_MIN) * self.SENS_YAW
                    elif y > self.CALIB_YAW_MAX: val_yaw = (y - self.CALIB_YAW_MAX) * self.SENS_YAW

                    control_data = {
                        "roll": 0.0,
                        "pitch": val_pitch, # Windows side treats this as vertical speed
                        "yaw": val_yaw,
                        "throttle": 0.0
                    }
                    
                    # State feedback
                    if val_pitch < -0.1: action_str = "ASCEND"
                    elif val_pitch > 0.1: action_str = "DESCEND"
                    else: action_str = "ALT HOLD"
                    
                    # Hint: Forward speed depends only on SPEED
                    if self.flight_mode == MODE_CRUISE:
                        action_str += " + MOVING"

                    if r_lm:
                        center = (int(r_lm[9].x * w), int(r_lm[9].y * h))
                        self.draw_hud(frame, r, p, y, center)

        # =========================================================
        # 5. UI & OSC
        # =========================================================
        cv2.rectangle(frame, (0, 0), (300, 220), (0, 0, 0), -1)
        
        mode_names = {MODE_HOVER: "HOVER", MODE_CRUISE: "CRUISE",
                      MODE_SPEED_SET: "SET SPEED", MODE_SINGLE_SELECT: "SELECT"}

        cv2.putText(frame, f"MODE: {mode_names[self.flight_mode]}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Speed is only active in cruise mode, highlight accordingly
        spd_col = (0, 255, 255) if self.flight_mode == MODE_CRUISE else (100, 100, 100)
        cv2.putText(frame, f"SPEED: {self.speed_level}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, spd_col, 2)
        
        cv2.putText(frame, f"FORM: {self.formation}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        unit_str = "ALL" if self.drone_id == 0 else f"UAV{self.drone_id}"
        cv2.putText(frame, f"UNIT: {unit_str}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        lock_str = "UNLOCKED" if self.right_hand_active else "LOCKED"
        lock_col = (0, 255, 0) if self.right_hand_active else (0, 0, 255)
        cv2.putText(frame, f"STATUS: {lock_str}", (20, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, lock_col, 2)

        if self.feedback_timer > 0:
            cv2.putText(frame, self.feedback_msg, (w//2 - 150, h//2 + 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            self.feedback_timer -= 1
        
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

if __name__ == "__main__":
    controller = FinalDroneController()
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        success, frame = cap.read()
        if not success: break
        frame = controller.process_frame(frame)
        cv2.imshow('Drone HUD Control', frame)
        if cv2.waitKey(5) & 0xFF == 27: break
    cap.release()
    cv2.destroyAllWindows()
