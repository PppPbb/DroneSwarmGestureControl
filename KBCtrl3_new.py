import pygame
import sys
import airsim
import time
import numpy as np

# ========== pygame 初始化 ==========
pygame.init()

# 小窗口即可，只用于获取键盘输入
screen = pygame.display.set_mode((320, 240))
pygame.display.set_caption("Multi-UAV Keyboard Control")

# 防止窗口未响应
screen.fill((0, 0, 0))
pygame.display.flip()

# 设定键盘重复（可选）
pygame.key.set_repeat(1, 20)


# ========== AirSim 客户端 ==========
client = airsim.MultirotorClient()
client.confirmConnection()
vehicles = ["UAV1", "UAV2", "UAV3"]

for name in vehicles:
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

# 同步起飞
takeoff_tasks = []
for name in vehicles:
    takeoff_tasks.append(
        client.takeoffAsync(vehicle_name=name)
    )

# 等待全部起飞完成
for task in takeoff_tasks:
    task.join()

# 给一点时间稳定悬停
time.sleep(1.0)

target_z = -5.0  # NED 坐标系，负数表示向上

for name in vehicles:
    client.moveToZAsync(target_z, 1.0, vehicle_name=name)

time.sleep(2.0)



vehicles = ["UAV1", "UAV2", "UAV3"]
leader = "UAV2"
followers = ["UAV1", "UAV3"]

forward_mode = False
formation_mode = "triangle"

forward_speed = 2.0
yaw_rate = 5.0
vertical_speed = 1.0

triangle_offsets = {
    "UAV1": np.array([-2.0, -2.0, 0.0]),   # 左后
    "UAV3": np.array([-2.0,  2.0, 0.0])    # 右后
}

line_offsets = {
    "UAV1": np.array([-2.0, 0.0, 0.0]),    # UAV2 后方
    "UAV3": np.array([-4.0, 0.0, 0.0])     # 再后
}


def rotate_offset(offset, yaw):
    """
    offset: np.array([x, y, z])  (机体系偏移)
    yaw:    领航机 yaw 角（弧度）
    """
    R = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw),  np.cos(yaw), 0.0],
        [0.0,          0.0,         1.0]
    ])
    return R @ offset


while True:
    time.sleep(0.02)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                forward_mode = True
            if event.key == pygame.K_DOWN:
                forward_mode = False
            if event.key == pygame.K_f:
                formation_mode = "line"
            if event.key == pygame.K_g:
                formation_mode = "triangle"

    keys = pygame.key.get_pressed()

    # ===== UAV2 键盘控制 =====
    vz = 0
    yaw_cmd = 0
    vx = forward_speed if forward_mode else 0

    if keys[pygame.K_w]:
        vz = -vertical_speed
    if keys[pygame.K_s]:
        vz = vertical_speed
    if keys[pygame.K_a]:
        yaw_cmd = -yaw_rate
    if keys[pygame.K_d]:
        yaw_cmd = yaw_rate

    client.moveByVelocityBodyFrameAsync(
        vx=vx, vy=0, vz=vz, duration=0.02,
        yaw_mode=airsim.YawMode(True, yaw_cmd),
        vehicle_name=leader
    )

    # ===== 编队控制 =====
    leader_state = client.getMultirotorState(vehicle_name=leader)
    pos_L = leader_state.kinematics_estimated.position
    yaw = airsim.to_eularian_angles(
        leader_state.kinematics_estimated.orientation
    )[2]

    offsets = triangle_offsets if formation_mode == "triangle" else line_offsets

    for uav in followers:
        offset = rotate_offset(offsets[uav], yaw)
        target = pos_L + airsim.Vector3r(*offset)

        state = client.getMultirotorState(vehicle_name=uav)
        pos = state.kinematics_estimated.position

        err = target - pos
        vx, vy, vz = err.x_val, err.y_val, err.z_val

        client.moveByVelocityAsync(
            vx, vy, vz, 0.02, vehicle_name=uav
        )
