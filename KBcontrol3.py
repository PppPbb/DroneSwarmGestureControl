import sys
import time
import airsim
import pygame
import numpy as np
from math import cos, sin

# 无人机队形参数
FORMATION_RADIUS = 3.0  # 队形半径（米）
LEADER_INDEX = 1  # 领队无人机索引（UAV2在中间）

# 三架无人机的初始位置偏移（相对领队位置）
FORMATION_OFFSETS = {
    "UAV1": np.array([-FORMATION_RADIUS, -FORMATION_RADIUS, 0]),  # 左后方
    "UAV2": np.array([0, 0, 0]),  # 中间（领队）
    "UAV3": np.array([-FORMATION_RADIUS, FORMATION_RADIUS, 0])  # 右后方
}

# 无人机列表
DRONE_NAMES = ["UAV1", "UAV2", "UAV3"]

# 控制参数
vehicle_velocity = 2.0
speedup_ratio = 10.0
vehicle_yaw_rate = 5.0

# 初始化Pygame
pygame.init()
screen = pygame.display.set_mode((320, 240))
pygame.display.set_caption('Drone Formation Control')
screen.fill((0, 0, 0))


class FormationController:
    def __init__(self):
        self.clients = {}  # 为每架无人机创建独立的客户端

        # 创建并初始化每架无人机的客户端
        for drone_name in DRONE_NAMES:
            client = airsim.MultirotorClient()
            client.confirmConnection()
            client.enableApiControl(True, vehicle_name=drone_name)
            client.armDisarm(True, vehicle_name=drone_name)
            self.clients[drone_name] = client

        # 设置初始位置
        self.setup_initial_formation()

        # 顺序起飞所有无人机
        print("Taking off drones...")
        for drone_name in DRONE_NAMES:
            print(f"Taking off {drone_name}...")
            self.clients[drone_name].takeoffAsync(vehicle_name=drone_name).join()
            time.sleep(1)  # 给每架无人机起飞间隔时间

        # 获取领队初始姿态
        leader_state = self.clients[DRONE_NAMES[LEADER_INDEX]].getMultirotorState(
            vehicle_name=DRONE_NAMES[LEADER_INDEX]
        )
        self.leader_yaw = airsim.to_eularian_angles(leader_state.kinematics_estimated.orientation)[2]

    def setup_initial_formation(self):
        """设置初始三角形队形"""
        # 获取UAV2（领队）的初始位置
        leader_client = self.clients[DRONE_NAMES[LEADER_INDEX]]
        leader_start_pos = leader_client.simGetVehiclePose(vehicle_name=DRONE_NAMES[LEADER_INDEX]).position

        print("Setting up initial formation...")
        # 设置每架无人机的位置
        for drone_name in DRONE_NAMES:
            if drone_name == DRONE_NAMES[LEADER_INDEX]:
                continue

            print(f"Positioning {drone_name}...")
            # 计算相对于领队的位置
            offset = FORMATION_OFFSETS[drone_name]
            target_pos = airsim.Vector3r(
                leader_start_pos.x_val + offset[0],
                leader_start_pos.y_val + offset[1],
                leader_start_pos.z_val + offset[2]
            )

            # 设置无人机位置
            pose = self.clients[drone_name].simGetVehiclePose(vehicle_name=drone_name)
            pose.position = target_pos
            self.clients[drone_name].simSetVehiclePose(pose, True, vehicle_name=drone_name)
            time.sleep(0.5)

    def calculate_formation_positions(self, leader_velocity, yaw_rate, dt=0.02):
        """根据领队速度和偏航率计算队形位置"""
        # 获取领队当前状态
        leader_client = self.clients[DRONE_NAMES[LEADER_INDEX]]
        leader_state = leader_client.getMultirotorState(vehicle_name=DRONE_NAMES[LEADER_INDEX])
        leader_pos = leader_state.kinematics_estimated.position

        # 更新领队偏航角
        self.leader_yaw += yaw_rate * dt

        # 旋转矩阵
        cos_yaw = cos(self.leader_yaw)
        sin_yaw = sin(self.leader_yaw)

        formation_commands = {}

        for drone_name in DRONE_NAMES:
            if drone_name == DRONE_NAMES[LEADER_INDEX]:
                # 领队直接跟随控制输入
                formation_commands[drone_name] = {
                    'vx': leader_velocity[0],
                    'vy': leader_velocity[1],
                    'vz': leader_velocity[2],
                    'yaw_rate': yaw_rate
                }
            else:
                # 计算僚机在旋转后的队形位置
                offset = FORMATION_OFFSETS[drone_name]

                # 应用旋转
                rotated_offset = np.array([
                    offset[0] * cos_yaw - offset[1] * sin_yaw,
                    offset[0] * sin_yaw + offset[1] * cos_yaw,
                    offset[2]
                ])

                # 计算僚机的目标位置
                target_pos = np.array([
                    leader_pos.x_val + rotated_offset[0],
                    leader_pos.y_val + rotated_offset[1],
                    leader_pos.z_val + rotated_offset[2]
                ])

                # 获取僚机当前位置
                drone_state = self.clients[drone_name].getMultirotorState(vehicle_name=drone_name)
                drone_pos = drone_state.kinematics_estimated.position
                current_pos = np.array([drone_pos.x_val, drone_pos.y_val, drone_pos.z_val])

                # 计算位置误差
                pos_error = target_pos - current_pos

                # 比例控制（可以调整增益）
                kp = 2.0
                velocity_command = pos_error * kp

                # 限制最大速度
                max_vel = 5.0
                vel_norm = np.linalg.norm(velocity_command)
                if vel_norm > max_vel:
                    velocity_command = velocity_command * (max_vel / vel_norm)

                # 僚机与领队保持相同的偏航角
                current_yaw = airsim.to_eularian_angles(drone_state.kinematics_estimated.orientation)[2]
                yaw_error = self.leader_yaw - current_yaw

                # 归一化角度差到[-pi, pi]
                while yaw_error > np.pi:
                    yaw_error -= 2 * np.pi
                while yaw_error < -np.pi:
                    yaw_error += 2 * np.pi

                # 比例控制偏航
                yaw_kp = 1.0
                follower_yaw_rate = yaw_error * yaw_kp

                formation_commands[drone_name] = {
                    'vx': float(velocity_command[0]),
                    'vy': float(velocity_command[1]),
                    'vz': float(velocity_command[2]),
                    'yaw_rate': float(follower_yaw_rate)
                }

        return formation_commands

    def move_formation(self, velocity_x, velocity_y, velocity_z, yaw_rate):
        """控制整个队形移动"""
        leader_velocity = np.array([velocity_x, velocity_y, velocity_z])

        # 计算队形中各无人机指令
        formation_commands = self.calculate_formation_positions(leader_velocity, yaw_rate)

        # 顺序发送控制指令（不使用线程）
        for drone_name in DRONE_NAMES:
            cmd = formation_commands[drone_name]

            # 发送控制指令
            self.clients[drone_name].moveByVelocityBodyFrameAsync(
                vx=cmd['vx'],
                vy=cmd['vy'],
                vz=cmd['vz'],
                duration=0.02,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=cmd['yaw_rate']),
                vehicle_name=drone_name
            )

    def get_drone_positions(self):
        """获取所有无人机的位置"""
        positions = {}
        for drone_name in DRONE_NAMES:
            state = self.clients[drone_name].getMultirotorState(vehicle_name=drone_name)
            pos = state.kinematics_estimated.position
            positions[drone_name] = (pos.x_val, pos.y_val, pos.z_val)
        return positions


# 初始化队形控制器
try:
    print("Initializing formation controller...")
    controller = FormationController()
    print("Formation controller initialized successfully!")

    # 主循环
    while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        time.sleep(0.02)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        scan_wrapper = pygame.key.get_pressed()

        # 空格键加速
        if scan_wrapper[pygame.K_SPACE]:
            scale_ratio = speedup_ratio
        else:
            scale_ratio = 1.0

        # 偏航控制
        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate

        # 前后控制
        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
            velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio * vehicle_velocity

        # 左右控制
        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
            velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio * vehicle_velocity

        # 上下控制
        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
            velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio * vehicle_velocity

        # 控制整个队形移动
        controller.move_formation(velocity_x, velocity_y, velocity_z, yaw_rate)

        # 可选：打印无人机位置（调试用）
        # positions = controller.get_drone_positions()
        # for drone_name, pos in positions.items():
        #     print(f"{drone_name}: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
        # print("---")

        if scan_wrapper[pygame.K_ESCAPE]:
            print("Landing all drones...")
            # 降落所有无人机
            for drone_name in DRONE_NAMES:
                print(f"Landing {drone_name}...")
                controller.clients[drone_name].landAsync(vehicle_name=drone_name)
                time.sleep(1)
            time.sleep(3)
            break

except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()

finally:
    # 清理
    pygame.quit()
    sys.exit()