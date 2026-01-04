"""
文件名: swarm_bridge.py
运行在: Windows (连接 AirSim 的电脑)
功能: 接收 Mac 指令，控制 AirSim 集群，防止心跳断联
"""
import airsim
import time
import numpy as np
import threading
import math
from pythonosc import dispatcher
from pythonosc import osc_server

# ================= 配置区域 =================
LISTEN_IP = "0.0.0.0"  # 允许局域网连接
LISTEN_PORT = 8000  # 必须与 Mac 端设置的一致


# ================= 全局控制变量 =================
class ControlState:
    def __init__(self):
        self.vx = 0.0  # 前后速度
        self.vz = 0.0  # 垂直速度 (新: 由 Pitch 控制)
        self.yaw_rate = 0.0  # 转向速度
        self.formation = "triangle"
        self.flight_mode = 0  # 0=HOVER, 1=CRUISE
        self.speed_mult = 1  # 速度等级 1-5


state = ControlState()


# ================= 1. OSC 消息处理 (适配新战术逻辑) =================
def handle_osc_command(unused_addr, *args):
    """
    接收 Mac 发来的数据:
    [mode, speed, formation, drone_id, roll, pitch, yaw, throttle]
    """
    if len(args) < 8: return

    # 1. 解包
    mode = int(args[0])
    speed_level = args[1]
    form_id = args[2]
    # args[3] drone_id (暂时忽略，默认控制集群)

    val_pitch = args[5]  # 现在代表【升降指令】
    val_yaw = args[6]  # 代表【转向指令】

    # 2. 更新状态
    state.flight_mode = mode
    state.speed_mult = speed_level

    # --- 动力映射 (Mixer) ---

    # A. 前进速度 (VX) - 仅由【模式】决定 (定速巡航)
    if state.flight_mode == 1:  # CRUISE
        # 速度公式：基础速度 0.8m/s * 等级
        state.vx = 1 * state.speed_mult
    else:
        state.vx = 0.0

    # B. 垂直速度 (VZ) - 由【右手 Pitch】决定
    # Mac发送: 负数=翘起(上), 正数=压下(下)
    # AirSim: 负数=向上(Up), 正数=向下(Down)
    # 直接映射，系数 0.08 用于调整升降手感
    state.vz = val_pitch * -1

    # C. 转向速度 (YawRate)
    state.yaw_rate = val_yaw * 1.5  # 放大系数，让转向更灵敏

    # D. 队形
    if form_id == 1.0:
        state.formation = "triangle"
    elif form_id == 2.0:
        state.formation = "line"  # 纵向
    elif form_id == 3.0:
        state.formation = "row"  # 横向


# ================= 2. 启动 OSC 服务器 =================
def start_osc_server():
    disp = dispatcher.Dispatcher()
    disp.map("/drone/control", handle_osc_command)
    server = osc_server.ThreadingOSCUDPServer((LISTEN_IP, LISTEN_PORT), disp)
    print(f"🎧 OSC 监听中: {LISTEN_IP}:{LISTEN_PORT}")
    server.serve_forever()


# 后台线程启动接收
t = threading.Thread(target=start_osc_server, daemon=True)
t.start()

# ================= 3. AirSim 初始化 =================
print("正在连接 AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()

vehicles = ["UAV1", "UAV2", "UAV3"]
for name in vehicles:
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

print("正在起飞...")
takeoff_tasks = [client.takeoffAsync(vehicle_name=n) for n in vehicles]
for task in takeoff_tasks: task.join()

# 初始升高一点
for name in vehicles:
    client.moveToZAsync(-1.5, 1.0, vehicle_name=name)
time.sleep(2.0)

print("🚀 系统就绪！Python 正在向 AirSim 发送心跳指令...")

# ================= 4. 主控制循环 (心跳包) =================
# 队形偏移参数
triangle_offsets = {"UAV1": [-2, -2, 0], "UAV3": [-2, 2, 0]}
line_offsets = {"UAV1": [-2, 0, 0], "UAV3": [-4, 0, 0]}  # 纵向
row_offsets = {"UAV1": [0, -3, 0], "UAV3": [0, 3, 0]}  # 横向


def rotate_vec(x, y, yaw):
    nx = x * math.cos(yaw) - y * math.sin(yaw)
    ny = x * math.sin(yaw) + y * math.cos(yaw)
    return nx, ny


leader = "UAV2"
followers = ["UAV1", "UAV3"]

try:
    while True:
        # A. 驱动领航机 (UAV2)
        # duration=0.5 表示这条指令管 0.5秒
        # 我们每 0.05秒 发一次，这样就永远不会断连
        client.moveByVelocityBodyFrameAsync(
            vx=state.vx,
            vy=0,
            vz=state.vz,
            duration=0.5,  # 【关键修改】加长持续时间，防止报错
            yaw_mode=airsim.YawMode(True, state.yaw_rate),
            vehicle_name=leader
        )

        # B. 驱动僚机 (跟随)
        leader_state = client.getMultirotorState(vehicle_name=leader)
        pos_L = leader_state.kinematics_estimated.position
        # 获取四元数转欧拉角拿 Yaw
        q = leader_state.kinematics_estimated.orientation
        yaw_rad = airsim.to_eularian_angles(q)[2]

        # 选队形
        if state.formation == "triangle":
            offsets = triangle_offsets
        elif state.formation == "row":
            offsets = row_offsets
        else:
            offsets = line_offsets

        for uav in followers:
            ox, oy, oz = offsets[uav]
            rot_x, rot_y = rotate_vec(ox, oy, yaw_rad)

            target_x = pos_L.x_val + rot_x
            target_y = pos_L.y_val + rot_y
            target_z = pos_L.z_val + oz

            client.moveToPositionAsync(
                target_x, target_y, target_z,
                2.0,
                yaw_mode=airsim.YawMode(
                    False,
                    math.degrees(yaw_rad)
                ),
                vehicle_name=uav
            )

        # 循环频率 20Hz
        time.sleep(0.05)

except KeyboardInterrupt:
    print("停止控制，正在降落...")
    for name in vehicles: client.landAsync(vehicle_name=name)
