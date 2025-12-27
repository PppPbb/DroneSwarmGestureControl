"""
文件名: swarm_bridge.py
运行在: Windows (连接 AirSim 的电脑)
功能: 接收 Mac 的 OSC 手势指令，控制 AirSim 无人机集群
"""
import airsim
import time
import numpy as np
import threading
from pythonosc import dispatcher
from pythonosc import osc_server
import math

# ================= 配置区域 =================
LISTEN_IP = "0.0.0.0"  # 允许局域网连接
LISTEN_PORT = 8000     # 必须与 Mac 端设置的一致

# ================= 全局控制变量 (由 OSC 更新) =================
class ControlState:
    def __init__(self):
        self.vx = 0.0          # 前后速度 (m/s)
        self.yaw_rate = 0.0    # 旋转速度 (rad/s)
        self.formation = "triangle" # 队形
        self.flight_mode = 0   # 0=悬停, 1=巡航...
        self.speed_mult = 1.0  # 速度倍率

state = ControlState()

# ================= 1. OSC 消息处理函数 =================
def handle_osc_command(unused_addr, *args):
    """
    接收格式: [mode, speed_level, formation_id, drone_id, roll, pitch, yaw, throttle]
    对应 main.py 发送的数据
    """
    if len(args) < 8: return

    # 1. 解包数据
    mode = int(args[0])
    speed_level = args[1]
    form_id = args[2]
    # drone_id = args[3] (暂时默认控制集群)
    # roll = args[4] (已禁用)
    pitch_val = args[5] # 手势前后倾斜量
    yaw_val = args[6]   # 手势左右旋转量

    # 2. 更新状态
    state.flight_mode = mode
    state.speed_mult = speed_level # 1~5

    # 3. 速度映射 (核心控制逻辑)
    # Pitch: 负数向前(手翘起), 正数向后(手下压) -> 你的逻辑里Pitch<0是手翘起
    # 假设：手翘起(负) = 向前飞; 手下压(正) = 向后飞
    # 映射系数：需要根据手感微调，这里 0.2 是个保守值
    state.vx = -pitch_val * 0.2 * (state.speed_mult * 0.5)

    # Yaw: 映射系数
    state.yaw_rate = yaw_val * 0.1

    # 4. 队形映射
    # main.py 定义: TRIANGLE=1.0, LINE_VERTICAL=2.0, LINE_HORIZONTAL=3.0
    if form_id == 1.0: state.formation = "triangle"
    elif form_id == 2.0: state.formation = "line" # 对应纵向
    elif form_id == 3.0: state.formation = "row"  # 对应横向 (原代码只有line和triangle，你可以自己加row)

    # 打印调试信息 (可选)
    # print(f"CMD | VX:{state.vx:.2f} | Yaw:{state.yaw_rate:.2f} | Form:{state.formation}")

# ================= 2. 启动 OSC 服务器 (子线程) =================
def start_osc_server():
    disp = dispatcher.Dispatcher()
    disp.map("/drone/control", handle_osc_command)

    server = osc_server.ThreadingOSCUDPServer((LISTEN_IP, LISTEN_PORT), disp)
    print(f"🎧 OSC 接收端已启动: {LISTEN_IP}:{LISTEN_PORT}")
    server.serve_forever()

# 开启后台线程接收网络数据，不阻塞主循环
t = threading.Thread(target=start_osc_server, daemon=True)
t.start()

# ================= 3. AirSim 初始化 (保留原逻辑) =================
print("正在连接 AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()
vehicles = ["UAV1", "UAV2", "UAV3"]

for name in vehicles:
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

print("正在起飞...")
takeoff_tasks = []
for name in vehicles:
    takeoff_tasks.append(client.takeoffAsync(vehicle_name=name))
for task in takeoff_tasks: task.join()

time.sleep(1.0)
print("调整高度...")
target_z = -3.0  # 稍微飞高一点
for name in vehicles:
    client.moveToZAsync(target_z, 1.0, vehicle_name=name)
time.sleep(2.0)

# 定义集群角色
leader = "UAV2"
followers = ["UAV1", "UAV3"]

# 队形偏移量定义
triangle_offsets = {
    "UAV1": np.array([-2.0, -2.0, 0.0]),   # 左后
    "UAV3": np.array([-2.0,  2.0, 0.0])    # 右后
}
line_offsets = { # 纵向一字
    "UAV1": np.array([-2.0, 0.0, 0.0]),
    "UAV3": np.array([-4.0, 0.0, 0.0])
}
# (可选) 横向一字
row_offsets = {
    "UAV1": np.array([0.0, -3.0, 0.0]),
    "UAV3": np.array([0.0, 3.0, 0.0])
}

def rotate_offset(offset, yaw_rad):
    R = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
        [np.sin(yaw_rad),  np.cos(yaw_rad), 0.0],
        [0.0,              0.0,             1.0]
    ])
    return R @ offset

print("🚀 系统就绪！请在 Mac 上比划手势。")

# ================= 4. 主控制循环 =================
try:
    while True:
        # 控制频率
        time.sleep(0.05)

        # A. 控制领航机 (UAV2)
        # 直接使用 state 全局变量里的数值
        # 注意 AirSim 的 YawMode 参数单位是 角度(degrees) 还是 弧度?
        # API 通常是 degrees/sec for yaw_rate if is_rate=True.
        # 我们的计算结果 yaw_rate 比较小，假设是度。

        # 紧急制动逻辑 (握拳时 flight_mode通常会变，或者速度为0)
        # 这里直接根据 vx 执行

        client.moveByVelocityBodyFrameAsync(
            vx=state.vx,
            vy=0, # 禁用了横移
            vz=0, # 禁用了升降 (保持定高)
            duration=0.1,
            yaw_mode=airsim.YawMode(True, state.yaw_rate * 5.0), # 乘系数放大一点转向
            vehicle_name=leader
        )

        # B. 编队跟随逻辑 (保持原样)
        leader_state = client.getMultirotorState(vehicle_name=leader)
        pos_L = leader_state.kinematics_estimated.position
        # 获取领航机当前的 Yaw
        orientation = leader_state.kinematics_estimated.orientation
        yaw_rad = airsim.to_eularian_angles(orientation)[2]

        # 选择当前队形
        if state.formation == "triangle":
            offsets = triangle_offsets
        elif state.formation == "row":
            offsets = row_offsets
        else:
            offsets = line_offsets

        # 计算并驱动僚机
        for uav in followers:
            if uav in offsets:
                # 1. 计算目标位置
                local_offset = offsets[uav]
                global_offset = rotate_offset(local_offset, yaw_rad)
                target_pos = pos_L + airsim.Vector3r(*global_offset)

                # 2. 计算跟随速度 (P控制)
                uav_state = client.getMultirotorState(vehicle_name=uav)
                pos = uav_state.kinematics_estimated.position

                # 简单的 P 控制器系数
                k_p = 1.0
                v_x = (target_pos.x_val - pos.x_val) * k_p
                v_y = (target_pos.y_val - pos.y_val) * k_p
                v_z = (target_pos.z_val - pos.z_val) * k_p # 保持高度一致

                # 3. 执行
                client.moveByVelocityAsync(
                    v_x, v_y, v_z, 0.1,
                    yaw_mode=airsim.YawMode(False, math.degrees(yaw_rad)), # 僚机朝向与领航机一致
                    vehicle_name=uav
                )

except KeyboardInterrupt:
    print("停止控制，正在降落...")
    for name in vehicles:
        client.landAsync(vehicle_name=name)
