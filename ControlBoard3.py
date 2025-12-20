import tkinter as tk
from tkinter import ttk
import threading
import time
import airsim
import numpy as np
import math


class DroneControlPanel:
    def __init__(self, master):
        self.master = master
        master.title("无人机集群控制面板 - 修复版")
        master.geometry("1000x750")

        # AirSim客户端
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # 无人机列表
        self.drone_names = ['UAV1', 'UAV2', 'UAV3']
        self.init_positions = {}  # 每架无人机的初始位置（在无人机自己的坐标系中）
        self.current_positions = {}  # 存储当前实际位置
        self.current_global_positions = {}  # 存储当前全局位置
        self.is_armed = False
        self.current_formation = 'triangle'

        # 队形中心（全局坐标系）
        self.formation_center = [0.0, 0.0, -5.0]  # [center_x, center_y, center_z]

        # 修正后的队形偏移（以米为单位）
        # 格式：[(dx, dy), ...] 其中dx是X方向偏移（前/后），dy是Y方向偏移（左/右）
        self.formation_offsets = {
            'triangle': [
                (0.0, 0.0),  # 中心点
                (-2.0, 3.5),  # 左后方
                (2.0, 3.5)  # 右后方
            ],
            'line_horizontal': [
                (0.0, 0.0),  # 中间
                (0.0, 4.0),  # 右边
                (0.0, -4.0)  # 左边
            ],
            'line_vertical': [
                (0.0, 0.0),  # 中间
                (4.0, 0.0),  # 前面
                (-4.0, 0.0)  # 后面
            ],
            'circle': [
                (0.0, 0.0),  # 中心
                (3.0, 0.0),  # 东
                (0.0, 3.0)  # 北
            ],
            'square': [
                (0.0, 0.0),  # 中心
                (3.0, 3.0),  # 东北
                (-3.0, 3.0)  # 西北
            ],
            'v_formation': [
                (0.0, 0.0),  # 领头
                (2.0, 2.0),  # 右后方
                (2.0, -2.0)  # 左后方
            ]
        }

        # 队形显示名称映射
        self.formation_names = {
            'triangle': '三角形',
            'line_horizontal': '水平一字',
            'line_vertical': '垂直一字',
            'circle': '圆形',
            'square': '方形',
            'v_formation': 'V字形'
        }

        # 性能优化
        self.position_update_active = True
        self.last_debug_time = 0
        self.debug_interval = 2.0  # 调试信息输出间隔

        # 无人机状态
        self.drone_states = {name: 'disconnected' for name in self.drone_names}

        self.setup_ui()
        self.connect_to_sim()

        # 启动位置更新线程
        self.start_position_updater()
        self.start_safety_checker()

    def setup_ui(self):
        # 使用网格布局管理器
        self.master.columnconfigure(0, weight=1)

        # 创建主框架
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.columnconfigure(0, weight=1)

        # 1. 状态显示区域
        status_frame = ttk.LabelFrame(main_frame, text="无人机状态", padding="10")
        status_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        status_frame.columnconfigure(1, weight=1)
        status_frame.columnconfigure(2, weight=1)

        # 表头
        headers = ["无人机", "状态", "位置 (全局坐标系)", "高度"]
        for col, header in enumerate(headers):
            ttk.Label(status_frame, text=header, font=('Arial', 10, 'bold')).grid(
                row=0, column=col, padx=5, pady=2, sticky="w")

        self.status_labels = {}
        self.position_labels = {}
        self.height_labels = {}

        for i, name in enumerate(self.drone_names):
            row = i + 1

            # 无人机名称
            ttk.Label(status_frame, text=name, font=('Arial', 10)).grid(
                row=row, column=0, padx=5, pady=2, sticky="w")

            # 状态标签
            status_label = ttk.Label(status_frame, text="未连接", font=('Arial', 10), foreground="red")
            status_label.grid(row=row, column=1, padx=5, pady=2, sticky="w")
            self.status_labels[name] = status_label

            # 位置标签
            pos_label = ttk.Label(status_frame, text="(0.00, 0.00, 0.00)",
                                  font=('Consolas', 9))
            pos_label.grid(row=row, column=2, padx=5, pady=2, sticky="w")
            self.position_labels[name] = pos_label

            # 高度标签
            height_label = ttk.Label(status_frame, text="0.00 m", font=('Arial', 10))
            height_label.grid(row=row, column=3, padx=5, pady=2, sticky="w")
            self.height_labels[name] = height_label

        # 2. 队形中心控制区域
        center_frame = ttk.LabelFrame(main_frame, text="队形中心控制 (全局坐标系)", padding="10")
        center_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # X坐标控制
        ttk.Label(center_frame, text="X (前/后):").grid(row=0, column=0, padx=(0, 5))
        self.center_x_var = tk.DoubleVar(value=0.0)
        center_x_scale = ttk.Scale(center_frame, from_=-30.0, to=30.0,
                                   variable=self.center_x_var, orient=tk.HORIZONTAL, length=200)
        center_x_scale.grid(row=0, column=1, padx=5)
        self.center_x_label = ttk.Label(center_frame, text="0.0 m")
        self.center_x_label.grid(row=0, column=2, padx=(0, 20))

        # Y坐标控制
        ttk.Label(center_frame, text="Y (左/右):").grid(row=0, column=3, padx=(0, 5))
        self.center_y_var = tk.DoubleVar(value=0.0)
        center_y_scale = ttk.Scale(center_frame, from_=-30.0, to=30.0,
                                   variable=self.center_y_var, orient=tk.HORIZONTAL, length=200)
        center_y_scale.grid(row=0, column=4, padx=5)
        self.center_y_label = ttk.Label(center_frame, text="0.0 m")
        self.center_y_label.grid(row=0, column=5, padx=(0, 20))

        # Z坐标（高度）控制
        ttk.Label(center_frame, text="高度:").grid(row=0, column=6, padx=(0, 5))
        self.center_z_var = tk.DoubleVar(value=-5.0)
        center_z_scale = ttk.Scale(center_frame, from_=-20.0, to=-2.0,
                                   variable=self.center_z_var, orient=tk.HORIZONTAL, length=150)
        center_z_scale.grid(row=0, column=7, padx=5)
        self.center_z_label = ttk.Label(center_frame, text="5.0 m")
        self.center_z_label.grid(row=0, column=8, padx=(0, 20))

        # 绑定变量更新
        self.center_x_var.trace('w', lambda *args: self.update_center_labels())
        self.center_y_var.trace('w', lambda *args: self.update_center_labels())
        self.center_z_var.trace('w', lambda *args: self.update_center_labels())

        # 更新中心位置按钮
        ttk.Button(center_frame, text="更新队形中心",
                   command=self.update_formation_center, width=15).grid(
            row=0, column=9, padx=(20, 0))

        # 3. 基本控制区域
        control_frame = ttk.LabelFrame(main_frame, text="基本控制", padding="10")
        control_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")

        buttons_row1 = [
            ("全部起飞", self.takeoff_all),
            ("全部降落", self.land_all),
            ("全部悬停", self.hover_all),
            ("返回原点", self.return_home)
        ]

        for i, (text, command) in enumerate(buttons_row1):
            btn = ttk.Button(control_frame, text=text, command=command, width=12)
            btn.grid(row=0, column=i, padx=5)

        # 单独控制按钮
        ttk.Label(control_frame, text="单独控制:").grid(row=1, column=0, padx=5, pady=(10, 5), sticky="w")

        for i, name in enumerate(self.drone_names):
            frame = ttk.Frame(control_frame)
            frame.grid(row=1, column=i + 1, padx=5, pady=(10, 5))

            ttk.Label(frame, text=name, font=('Arial', 9)).pack(side=tk.LEFT, padx=(0, 5))
            ttk.Button(frame, text="起飞", width=6,
                       command=lambda n=name: self.single_takeoff(n)).pack(side=tk.LEFT, padx=2)
            ttk.Button(frame, text="降落", width=6,
                       command=lambda n=name: self.single_land(n)).pack(side=tk.LEFT, padx=2)
            ttk.Button(frame, text="悬停", width=6,
                       command=lambda n=name: self.single_hover(n)).pack(side=tk.LEFT, padx=2)

        # 4. 队形控制区域
        formation_frame = ttk.LabelFrame(main_frame, text="队形控制", padding="10")
        formation_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")

        formations = [
            ("三角形", 'triangle'),
            ("水平一字", 'line_horizontal'),
            ("垂直一字", 'line_vertical'),
            ("V字形", 'v_formation'),
            ("方形", 'square'),
            ("圆形", 'circle')
        ]

        for i, (text, formation_type) in enumerate(formations):
            btn = ttk.Button(formation_frame, text=text, width=12,
                             command=lambda ft=formation_type: self.set_formation(ft))
            btn.grid(row=0, column=i, padx=5)

        # 5. 运动控制区域
        movement_frame = ttk.LabelFrame(main_frame, text="运动控制", padding="10")
        movement_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")

        # 创建控制网格
        grid_frame = ttk.Frame(movement_frame)
        grid_frame.grid(row=0, column=0, columnspan=4, pady=5)

        # 方向控制网格 (3x3)
        buttons_grid = [
            [None, ("前进", 'forward'), None],
            [("左移", 'left'), None, ("右移", 'right')],
            [None, ("后退", 'backward'), None]
        ]

        for r in range(3):
            for c in range(3):
                if buttons_grid[r][c]:
                    text, direction = buttons_grid[r][c]
                    btn = ttk.Button(grid_frame, text=text, width=8,
                                     command=lambda d=direction: self.move_formation(d))
                    btn.grid(row=r, column=c, padx=3, pady=3, ipadx=5, ipady=5)

        # 高度控制
        height_frame = ttk.Frame(movement_frame)
        height_frame.grid(row=0, column=4, padx=(30, 10), pady=5, sticky="n")

        ttk.Button(height_frame, text="上升", width=8,
                   command=self.move_up).grid(row=0, column=0, pady=2)
        ttk.Button(height_frame, text="下降", width=8,
                   command=self.move_down).grid(row=1, column=0, pady=2)

        # 速度控制
        speed_frame = ttk.LabelFrame(movement_frame, text="速度控制", padding="5")
        speed_frame.grid(row=1, column=0, columnspan=3, padx=5, pady=(10, 5), sticky="ew")

        ttk.Label(speed_frame, text="速度:").grid(row=0, column=0, padx=(0, 5))
        self.speed_var = tk.DoubleVar(value=2.0)
        speed_scale = ttk.Scale(speed_frame, from_=0.5, to=8.0,
                                variable=self.speed_var, orient=tk.HORIZONTAL, length=200)
        speed_scale.grid(row=0, column=1, padx=5)
        self.speed_label = ttk.Label(speed_frame, text="2.0 m/s")
        self.speed_label.grid(row=0, column=2, padx=5)
        self.speed_var.trace('w', self.update_speed_label)

        # 步长控制
        ttk.Label(speed_frame, text="步长:").grid(row=0, column=3, padx=(20, 5))
        self.step_var = tk.DoubleVar(value=2.0)
        step_scale = ttk.Scale(speed_frame, from_=0.5, to=5.0,
                               variable=self.step_var, orient=tk.HORIZONTAL, length=150)
        step_scale.grid(row=0, column=4, padx=5)
        self.step_label = ttk.Label(speed_frame, text="2.0 m")
        self.step_label.grid(row=0, column=5, padx=5)
        self.step_var.trace('w', self.update_step_label)

        # 高度步长控制
        ttk.Label(speed_frame, text="高度步长:").grid(row=0, column=6, padx=(20, 5))
        self.height_step_var = tk.DoubleVar(value=1.0)
        height_step_scale = ttk.Scale(speed_frame, from_=0.5, to=3.0,
                                      variable=self.height_step_var, orient=tk.HORIZONTAL, length=100)
        height_step_scale.grid(row=0, column=7, padx=5)
        self.height_step_label = ttk.Label(speed_frame, text="1.0 m")
        self.height_step_label.grid(row=0, column=8, padx=5)
        self.height_step_var.trace('w', self.update_height_step_label)

        # 6. 高级功能区域
        advanced_frame = ttk.LabelFrame(main_frame, text="高级功能", padding="10")
        advanced_frame.grid(row=5, column=0, padx=5, pady=5, sticky="ew")

        advanced_buttons = [
            ("绘制矩形轨迹", self.start_rectangle_trajectory),
            ("保持队形移动测试", self.start_formation_movement_test),
            ("紧急停止", self.emergency_stop),
            ("重置仿真", self.reset_simulation)
        ]

        for i, (text, command) in enumerate(advanced_buttons):
            if text == "紧急停止":
                style = "Emergency.TButton"
            else:
                style = "TButton"

            btn = ttk.Button(advanced_frame, text=text, command=command,
                             style=style, width=15)
            btn.grid(row=0, column=i, padx=5)

        # 7. 调试信息区域
        debug_frame = ttk.LabelFrame(main_frame, text="调试信息", padding="10")
        debug_frame.grid(row=6, column=0, padx=5, pady=5, sticky="ew")

        ttk.Label(debug_frame, text="当前队形:").grid(row=0, column=0, sticky="w")
        self.current_formation_label = ttk.Label(debug_frame, text="无", font=('Arial', 10, 'bold'))
        self.current_formation_label.grid(row=0, column=1, sticky="w", padx=(5, 20))

        ttk.Label(debug_frame, text="队形中心:").grid(row=0, column=2, sticky="w")
        self.current_center_label = ttk.Label(debug_frame, text="(0.00, 0.00, -5.00)", font=('Consolas', 9))
        self.current_center_label.grid(row=0, column=3, sticky="w", padx=(5, 20))

        ttk.Label(debug_frame, text="安全距离:").grid(row=0, column=4, sticky="w")
        self.safe_distance_label = ttk.Label(debug_frame, text="正常", font=('Arial', 10), foreground="green")
        self.safe_distance_label.grid(row=0, column=5, sticky="w")

        # 8. 日志区域
        log_frame = ttk.LabelFrame(main_frame, text="控制日志", padding="10")
        log_frame.grid(row=7, column=0, padx=5, pady=(5, 10), sticky="nsew")

        # 配置网格权重
        main_frame.rowconfigure(7, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        # 创建文本区域和滚动条
        self.log_text = tk.Text(log_frame, height=10, wrap=tk.WORD, font=('Consolas', 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=scrollbar.set)

        # 日志控制按钮
        log_control_frame = ttk.Frame(log_frame)
        log_control_frame.grid(row=1, column=0, columnspan=2, pady=(5, 0), sticky="w")

        ttk.Button(log_control_frame, text="清空日志",
                   command=self.clear_log, width=10).pack(side=tk.LEFT, padx=(0, 5))

        ttk.Button(log_control_frame, text="导出日志",
                   command=self.export_log, width=10).pack(side=tk.LEFT, padx=5)

        # 添加标签配置
        self.log_text.tag_config("error", foreground="red")
        self.log_text.tag_config("warning", foreground="orange")
        self.log_text.tag_config("success", foreground="green")
        self.log_text.tag_config("info", foreground="blue")

    def log(self, message, tag="info"):
        """记录日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.log_text.see(tk.END)

        # 限制日志行数
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 100:
            self.log_text.delete(1.0, f"{lines - 80}.0")

        self.log_text.update()

    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)
        self.log("日志已清空", "info")

    def export_log(self):
        """导出日志到文件"""
        try:
            filename = f"drone_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"日志已导出到 {filename}", "success")
        except Exception as e:
            self.log(f"导出日志失败: {str(e)}", "error")

    def connect_to_sim(self):
        """连接模拟器并初始化所有无人机"""

        def connect_task():
            try:
                self.log("正在连接AirSim模拟器...")

                # 不要重置仿真，保持无人机当前位置
                for name in self.drone_names:
                    try:
                        # 启用API控制
                        self.client.enableApiControl(True, name)
                        time.sleep(0.2)

                        # 启动电机
                        self.client.armDisarm(True, name)
                        time.sleep(0.2)

                        # 获取初始位置
                        state = self.client.getMultirotorState(vehicle_name=name)
                        self.init_positions[name] = state.kinematics_estimated.position

                        # 计算全局位置：在无人机自己的坐标系中，初始位置就是原点
                        # 所以当前全局位置 = 初始位置
                        self.current_global_positions[name] = (
                            self.init_positions[name].x_val,
                            self.init_positions[name].y_val,
                            self.init_positions[name].z_val
                        )

                        # 更新状态显示
                        self.update_drone_status(name, "就绪", "green")
                        self.update_position_display(name)

                        self.log(f"{name} 初始化完成 - 初始位置: ({self.init_positions[name].x_val:.2f}, "
                                 f"{self.init_positions[name].y_val:.2f}, {self.init_positions[name].z_val:.2f})",
                                 "success")

                    except Exception as e:
                        self.log(f"{name} 初始化失败: {str(e)[:50]}...", "error")
                        # 设置默认值
                        self.init_positions[name] = airsim.Vector3r(0, 0, 0)
                        self.update_drone_status(name, "失败", "red")

                self.is_armed = True

                # 计算初始队形中心
                positions = []
                for name in self.drone_names:
                    if name in self.current_global_positions:
                        positions.append(self.current_global_positions[name])

                if positions:
                    avg_x = sum(p[0] for p in positions) / len(positions)
                    avg_y = sum(p[1] for p in positions) / len(positions)
                    avg_z = sum(p[2] for p in positions) / len(positions)

                    self.formation_center = [avg_x, avg_y, avg_z]
                    self.center_x_var.set(avg_x)
                    self.center_y_var.set(avg_y)
                    self.center_z_var.set(avg_z)

                    self.log(f"初始队形中心: ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})", "info")
                    self.update_center_labels()
                    self.update_current_formation_display()

                self.log("所有无人机初始化完成", "success")

            except Exception as e:
                self.log(f"连接失败: {str(e)}", "error")

        threading.Thread(target=connect_task, daemon=True).start()

    def start_position_updater(self):
        """启动位置更新线程"""

        def update_task():
            while self.position_update_active:
                try:
                    for name in self.drone_names:
                        try:
                            state = self.client.getMultirotorState(vehicle_name=name)
                            pos = state.kinematics_estimated.position
                            self.current_positions[name] = pos

                            # 更新全局位置
                            if name in self.init_positions:
                                self.current_global_positions[name] = (
                                    pos.x_val + self.init_positions[name].x_val,
                                    pos.y_val + self.init_positions[name].y_val,
                                    pos.z_val + self.init_positions[name].z_val
                                )

                            # 更新UI显示
                            self.master.after(0, self.update_position_display, name)

                        except Exception as e:
                            # 忽略单个无人机的位置获取失败
                            pass

                    # 定期输出调试信息
                    current_time = time.time()
                    if current_time - self.last_debug_time > self.debug_interval:
                        self.last_debug_time = current_time
                        self.output_debug_info()

                    time.sleep(0.3)  # 0.3秒更新一次

                except Exception as e:
                    time.sleep(1)

        thread = threading.Thread(target=update_task, daemon=True)
        thread.start()

    def start_safety_checker(self):
        """启动安全距离检查线程"""

        def safety_check_task():
            while self.position_update_active:
                try:
                    # 获取所有无人机位置
                    positions = {}
                    for name in self.drone_names:
                        if name in self.current_global_positions:
                            positions[name] = self.current_global_positions[name]

                    # 检查任意两架无人机之间的距离
                    drone_names = list(positions.keys())
                    min_distance = float('inf')
                    closest_pair = None

                    for i in range(len(drone_names)):
                        for j in range(i + 1, len(drone_names)):
                            name1, name2 = drone_names[i], drone_names[j]
                            if name1 in positions and name2 in positions:
                                pos1 = positions[name1]
                                pos2 = positions[name2]

                                # 计算水平距离
                                dx = pos1[0] - pos2[0]
                                dy = pos1[1] - pos2[1]
                                distance = math.sqrt(dx * dx + dy * dy)

                                if distance < min_distance:
                                    min_distance = distance
                                    closest_pair = (name1, name2)

                    # 更新UI显示
                    if closest_pair:
                        if min_distance < 1.0:
                            self.safe_distance_label.config(text=f"危险! {min_distance:.1f}m", foreground="red")
                        elif min_distance < 2.0:
                            self.safe_distance_label.config(text=f"警告 {min_distance:.1f}m", foreground="orange")
                        else:
                            self.safe_distance_label.config(text=f"安全 {min_distance:.1f}m", foreground="green")

                    time.sleep(1.0)  # 1秒检查一次

                except Exception as e:
                    time.sleep(2)

        thread = threading.Thread(target=safety_check_task, daemon=True)
        thread.start()

    def output_debug_info(self):
        """输出调试信息"""
        if self.current_global_positions:
            self.log("=== 调试信息 ===", "info")
            for name in self.drone_names:
                if name in self.current_global_positions:
                    x, y, z = self.current_global_positions[name]
                    self.log(f"{name}: ({x:.2f}, {y:.2f}, {z:.2f})", "info")
            self.log("================", "info")

    def update_drone_status(self, name, status, color="black"):
        """更新无人机状态显示"""
        self.status_labels[name].config(text=status, foreground=color)
        self.drone_states[name] = status

    def update_position_display(self, name):
        """更新位置显示"""
        if name in self.current_global_positions:
            x, y, z = self.current_global_positions[name]
            self.position_labels[name].config(
                text=f"({x:6.2f}, {y:6.2f}, {z:6.2f})"
            )
            self.height_labels[name].config(
                text=f"{abs(z):5.2f} m"
            )

    def update_center_labels(self):
        """更新中心坐标显示"""
        self.center_x_label.config(text=f"{self.center_x_var.get():.1f} m")
        self.center_y_label.config(text=f"{self.center_y_var.get():.1f} m")
        self.center_z_label.config(text=f"{abs(self.center_z_var.get()):.1f} m")

    def update_current_formation_display(self):
        """更新当前队形显示"""
        if self.current_formation in self.formation_names:
            formation_name = self.formation_names[self.current_formation]
        else:
            formation_name = self.current_formation

        self.current_formation_label.config(text=formation_name)
        self.current_center_label.config(
            text=f"({self.formation_center[0]:.2f}, {self.formation_center[1]:.2f}, {self.formation_center[2]:.2f})"
        )

    def takeoff_all(self):
        """所有无人机同时起飞"""

        def do_takeoff():
            try:
                self.log("开始所有无人机起飞...", "info")

                # 第一阶段：同时起飞
                takeoff_futures = []
                for name in self.drone_names:
                    self.log(f"发送 {name} 起飞命令", "info")
                    self.update_drone_status(name, "起飞中", "orange")
                    future = self.client.takeoffAsync(vehicle_name=name)
                    takeoff_futures.append((name, future))

                # 等待所有起飞完成
                for name, future in takeoff_futures:
                    try:
                        future.join()
                        self.log(f"{name} 起飞完成", "success")
                        self.update_drone_status(name, "空中", "green")
                    except Exception as e:
                        self.log(f"{name} 起飞异常: {str(e)[:50]}...", "error")
                        self.update_drone_status(name, "起飞失败", "red")

                time.sleep(1.0)  # 等待稳定

                # 第二阶段：调整到安全高度
                target_z = self.center_z_var.get()
                altitude_futures = []
                for name in self.drone_names:
                    if self.drone_states.get(name) == "空中":
                        self.log(f"{name} 调整到高度 {abs(target_z):.1f} 米", "info")
                        future = self.client.moveToZAsync(target_z, 1.5, vehicle_name=name)
                        altitude_futures.append((name, future))

                # 等待高度调整完成
                for name, future in altitude_futures:
                    try:
                        future.join()
                        self.log(f"{name} 高度调整完成", "success")
                    except Exception as e:
                        self.log(f"{name} 高度调整异常: {str(e)[:50]}...", "warning")

                # 更新队形中心高度
                self.formation_center[2] = target_z
                self.update_current_formation_display()

                self.log(f"所有无人机已起飞到高度 {abs(target_z):.1f} 米", "success")

            except Exception as e:
                self.log(f"起飞过程失败: {str(e)}", "error")

        threading.Thread(target=do_takeoff, daemon=True).start()

    def single_takeoff(self, name):
        """单架无人机起飞"""

        def do_single_takeoff():
            try:
                self.log(f"{name} 开始起飞...", "info")
                self.update_drone_status(name, "起飞中", "orange")

                # 起飞
                self.client.takeoffAsync(vehicle_name=name).join()
                self.log(f"{name} 起飞完成", "success")

                # 调整高度
                target_z = self.center_z_var.get()
                self.client.moveToZAsync(target_z, 1.5, vehicle_name=name).join()

                self.update_drone_status(name, "空中", "green")
                self.log(f"{name} 已起飞到高度 {abs(target_z):.1f} 米", "success")

            except Exception as e:
                self.log(f"{name} 起飞失败: {str(e)}", "error")
                self.update_drone_status(name, "起飞失败", "red")

        threading.Thread(target=do_single_takeoff, daemon=True).start()

    def land_all(self):
        """所有无人机降落"""

        def do_land():
            try:
                self.log("开始所有无人机降落...", "info")

                land_futures = []
                for name in self.drone_names:
                    if self.drone_states.get(name) in ["空中", "悬停"]:
                        self.log(f"{name} 开始降落", "info")
                        self.update_drone_status(name, "降落中", "orange")
                        future = self.client.landAsync(vehicle_name=name)
                        land_futures.append((name, future))

                for name, future in land_futures:
                    try:
                        future.join()
                        self.log(f"{name} 降落完成", "success")
                        self.update_drone_status(name, "地面", "blue")
                    except Exception as e:
                        self.log(f"{name} 降落异常: {str(e)[:50]}...", "warning")

                self.log("所有无人机已降落", "success")

            except Exception as e:
                self.log(f"降落过程失败: {str(e)}", "error")

        threading.Thread(target=do_land, daemon=True).start()

    def single_land(self, name):
        """单架无人机降落"""

        def do_single_land():
            try:
                self.log(f"{name} 开始降落...", "info")
                self.update_drone_status(name, "降落中", "orange")

                self.client.landAsync(vehicle_name=name).join()

                self.update_drone_status(name, "地面", "blue")
                self.log(f"{name} 降落完成", "success")

            except Exception as e:
                self.log(f"{name} 降落失败: {str(e)}", "error")
                self.update_drone_status(name, "降落失败", "red")

        threading.Thread(target=do_single_land, daemon=True).start()

    def hover_all(self):
        """所有无人机悬停"""

        def do_hover():
            try:
                for name in self.drone_names:
                    if self.drone_states.get(name) in ["空中", "起飞中"]:
                        self.client.hoverAsync(vehicle_name=name)

                self.log("所有无人机悬停", "info")

            except Exception as e:
                self.log(f"悬停失败: {str(e)}", "error")

        threading.Thread(target=do_hover, daemon=True).start()

    def single_hover(self, name):
        """单架无人机悬停"""

        def do_single_hover():
            try:
                if self.drone_states.get(name) in ["空中", "起飞中"]:
                    self.client.hoverAsync(vehicle_name=name)
                    self.log(f"{name} 悬停", "info")
                else:
                    self.log(f"{name} 不在空中，无法悬停", "warning")

            except Exception as e:
                self.log(f"{name} 悬停失败: {str(e)}", "error")

        threading.Thread(target=do_single_hover, daemon=True).start()

    def calculate_formation_positions(self):
        """计算当前队形下各无人机在全局坐标系中的目标位置"""
        center_x, center_y, center_z = self.formation_center

        # 获取当前队形的偏移
        if self.current_formation not in self.formation_offsets:
            self.current_formation = 'triangle'

        offsets = self.formation_offsets[self.current_formation]

        # 计算每架无人机的全局目标位置
        positions = []
        for i in range(len(self.drone_names)):
            if i < len(offsets):
                dx, dy = offsets[i]
                target_x = center_x + dx
                target_y = center_y + dy
                target_z = center_z
                positions.append((target_x, target_y, target_z))
            else:
                # 如果没有对应的偏移，使用中心点
                positions.append((center_x, center_y, center_z))

        return positions

    def set_formation(self, formation_type):
        """设置队形"""
        self.current_formation = formation_type

        # 获取显示名称
        if formation_type in self.formation_names:
            display_name = self.formation_names[formation_type]
        else:
            display_name = formation_type

        self.log(f"切换到 {display_name} 队形...", "info")
        self.update_current_formation_display()

        def do_formation():
            try:
                # 计算当前所有无人机的平均高度，保持当前高度
                heights = []
                for name in self.drone_names:
                    if name in self.current_global_positions:
                        heights.append(self.current_global_positions[name][2])

                if heights:
                    avg_height = sum(heights) / len(heights)
                    self.formation_center[2] = avg_height
                    self.center_z_var.set(avg_height)
                    self.update_center_labels()

                # 计算目标位置
                target_positions = self.calculate_formation_positions()

                # 记录调试信息
                self.log(f"队形中心: {self.formation_center}", "info")
                self.log(f"目标位置计算完成，开始移动...", "info")

                # 移动到目标位置
                futures = []
                for i, name in enumerate(self.drone_names):
                    if i < len(target_positions):
                        target_x, target_y, target_z = target_positions[i]

                        if name in self.init_positions:
                            offset = self.init_positions[name]

                            # 计算相对于无人机自身坐标系的目标位置
                            relative_x = target_x - offset.x_val
                            relative_y = target_y - offset.y_val
                            relative_z = target_z - offset.z_val

                            # 发送移动命令
                            future = self.client.moveToPositionAsync(
                                relative_x,
                                relative_y,
                                relative_z,
                                self.speed_var.get(),
                                vehicle_name=name
                            )
                            futures.append((name, future))

                            self.log(f"{name} -> 全局({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) "
                                     f"相对({relative_x:.1f}, {relative_y:.1f}, {relative_z:.1f})", "info")

                # 等待移动完成
                for name, future in futures:
                    try:
                        future.join()
                        self.log(f"{name} 到达队形位置", "success")
                    except Exception as e:
                        self.log(f"{name} 移动异常: {str(e)[:50]}...", "warning")

                self.log(f"{display_name} 队形已形成", "success")

            except Exception as e:
                self.log(f"队形切换失败: {str(e)}", "error")

        threading.Thread(target=do_formation, daemon=True).start()

    def update_formation_center(self):
        """更新队形中心位置"""
        new_center = [
            self.center_x_var.get(),
            self.center_y_var.get(),
            self.center_z_var.get()
        ]

        # 只更新队形中心，不移动无人机
        self.formation_center = new_center
        self.update_current_formation_display()

        self.log(f"队形中心更新为: ({new_center[0]:.2f}, {new_center[1]:.2f}, {new_center[2]:.2f})", "info")

    def move_formation(self, direction):
        """移动整个队形"""
        step = self.step_var.get()
        speed = self.speed_var.get()

        # 记录移动前的队形中心
        old_center = self.formation_center.copy()

        # 更新队形中心
        if direction == 'forward':
            self.formation_center[0] += step
        elif direction == 'backward':
            self.formation_center[0] -= step
        elif direction == 'left':
            self.formation_center[1] += step
        elif direction == 'right':
            self.formation_center[1] -= step

        # 更新UI显示
        self.center_x_var.set(self.formation_center[0])
        self.center_y_var.set(self.formation_center[1])
        self.update_center_labels()
        self.update_current_formation_display()

        self.log(f"队形向 {direction} 移动 {step:.1f} 米", "info")
        self.log(f"队形中心: {old_center} -> {self.formation_center}", "info")

        # 计算新位置
        target_positions = self.calculate_formation_positions()

        def do_move():
            try:
                futures = []
                for i, name in enumerate(self.drone_names):
                    if i < len(target_positions):
                        target_x, target_y, target_z = target_positions[i]

                        if name in self.init_positions:
                            offset = self.init_positions[name]

                            # 计算相对位置
                            relative_x = target_x - offset.x_val
                            relative_y = target_y - offset.y_val
                            relative_z = target_z - offset.z_val

                            # 发送移动命令
                            future = self.client.moveToPositionAsync(
                                relative_x,
                                relative_y,
                                relative_z,
                                speed,
                                vehicle_name=name
                            )
                            futures.append((name, future))

                # 等待移动完成
                for name, future in futures:
                    try:
                        future.join()
                    except Exception as e:
                        self.log(f"{name} 移动异常: {str(e)[:50]}...", "warning")

                self.log(f"队形移动完成", "success")

            except Exception as e:
                self.log(f"移动失败: {str(e)}", "error")

        threading.Thread(target=do_move, daemon=True).start()

    def move_up(self):
        """上升整个队形"""
        step = self.height_step_var.get()
        speed = self.speed_var.get()

        # 更新高度（NED坐标系下负值上升）
        new_height = self.formation_center[2] - step

        # 限制最低高度
        if new_height < -20:
            new_height = -20
            self.log("已达到最低高度限制", "warning")

        self.formation_center[2] = new_height
        self.center_z_var.set(new_height)
        self.update_center_labels()
        self.update_current_formation_display()

        self.log(f"队形上升 {step:.1f} 米到高度 {abs(new_height):.1f} 米", "info")

        # 重新形成队形
        self.set_formation(self.current_formation)

    def move_down(self):
        """下降整个队形"""
        step = self.height_step_var.get()
        speed = self.speed_var.get()

        # 更新高度（NED坐标系下正值下降）
        new_height = self.formation_center[2] + step

        # 限制最高高度
        if new_height > -2:
            new_height = -2
            self.log("已达到最高高度限制", "warning")

        self.formation_center[2] = new_height
        self.center_z_var.set(new_height)
        self.update_center_labels()
        self.update_current_formation_display()

        self.log(f"队形下降 {step:.1f} 米到高度 {abs(new_height):.1f} 米", "info")

        # 重新形成队形
        self.set_formation(self.current_formation)

    def return_home(self):
        """返航到初始位置附近"""

        def do_return():
            try:
                # 重置队形中心到(0, 0, -5)
                self.formation_center = [0.0, 0.0, -5.0]
                self.center_x_var.set(0.0)
                self.center_y_var.set(0.0)
                self.center_z_var.set(-5.0)
                self.update_center_labels()
                self.update_current_formation_display()

                # 形成队形
                self.set_formation(self.current_formation)
                self.log("返航到原点", "info")

            except Exception as e:
                self.log(f"返航失败: {str(e)}", "error")

        threading.Thread(target=do_return, daemon=True).start()

    def start_rectangle_trajectory(self):
        """开始矩形轨迹飞行"""

        def do_trajectory():
            try:
                self.log("开始矩形轨迹飞行...", "info")

                # 记录起始点
                start_center = self.formation_center.copy()

                # 定义矩形四个点（相对于当前队形中心）
                rectangle_points = [
                    (start_center[0] + 5, start_center[1], start_center[2]),  # 向前5米
                    (start_center[0] + 5, start_center[1] + 5, start_center[2]),  # 向右5米
                    (start_center[0], start_center[1] + 5, start_center[2]),  # 向后5米
                    (start_center[0], start_center[1], start_center[2])  # 向左5米
                ]

                for i, point in enumerate(rectangle_points):
                    self.log(f"前往点 {i + 1}: ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})", "info")

                    # 更新队形中心
                    self.formation_center = list(point)
                    self.center_x_var.set(point[0])
                    self.center_y_var.set(point[1])
                    self.update_center_labels()
                    self.update_current_formation_display()

                    # 移动到新位置
                    target_positions = self.calculate_formation_positions()

                    futures = []
                    for j, name in enumerate(self.drone_names):
                        if j < len(target_positions):
                            target_x, target_y, target_z = target_positions[j]

                            if name in self.init_positions:
                                offset = self.init_positions[name]
                                relative_x = target_x - offset.x_val
                                relative_y = target_y - offset.y_val
                                relative_z = target_z - offset.z_val

                                future = self.client.moveToPositionAsync(
                                    relative_x,
                                    relative_y,
                                    relative_z,
                                    self.speed_var.get(),
                                    vehicle_name=name
                                )
                                futures.append((name, future))

                    # 等待到达
                    for name, future in futures:
                        try:
                            future.join()
                        except Exception as e:
                            self.log(f"{name} 移动异常: {str(e)[:50]}...", "warning")

                    time.sleep(1)  # 在每个点停留1秒

                self.log("矩形轨迹飞行完成", "success")

            except Exception as e:
                self.log(f"轨迹飞行失败: {str(e)}", "error")

        threading.Thread(target=do_trajectory, daemon=True).start()

    def start_formation_movement_test(self):
        """保持队形的移动测试"""

        def do_test():
            try:
                self.log("开始保持队形移动测试...", "info")

                # 定义移动序列
                movements = [
                    ('forward', 3),
                    ('right', 3),
                    ('backward', 3),
                    ('left', 3)
                ]

                for direction, distance in movements:
                    self.log(f"向 {direction} 移动 {distance} 米", "info")

                    # 临时设置步长
                    original_step = self.step_var.get()
                    self.step_var.set(distance)

                    # 执行移动
                    self.move_formation(direction)

                    # 等待移动完成
                    time.sleep(3)

                    # 恢复步长
                    self.step_var.set(original_step)

                self.log("保持队形移动测试完成", "success")

            except Exception as e:
                self.log(f"移动测试失败: {str(e)}", "error")

        threading.Thread(target=do_test, daemon=True).start()

    def emergency_stop(self):
        """紧急停止所有无人机"""

        def do_stop():
            try:
                for name in self.drone_names:
                    if self.drone_states.get(name) in ["空中", "起飞中", "降落中"]:
                        self.client.hoverAsync(vehicle_name=name)
                        self.log(f"{name} 紧急悬停", "warning")

                self.log("紧急停止！所有无人机已悬停", "warning")

            except Exception as e:
                self.log(f"紧急停止失败: {str(e)}", "error")

        threading.Thread(target=do_stop, daemon=True).start()

    def reset_simulation(self):
        """重置仿真"""

        def do_reset():
            try:
                self.log("正在重置仿真...", "warning")
                self.client.reset()
                time.sleep(2)

                # 重新连接
                self.connect_to_sim()
                self.log("仿真重置完成", "success")

            except Exception as e:
                self.log(f"重置失败: {str(e)}", "error")

        threading.Thread(target=do_reset, daemon=True).start()

    def update_speed_label(self, *args):
        """更新速度显示"""
        speed = self.speed_var.get()
        self.speed_label.config(text=f"{speed:.1f} m/s")

    def update_step_label(self, *args):
        """更新步长显示"""
        step = self.step_var.get()
        self.step_label.config(text=f"{step:.1f} m")

    def update_height_step_label(self, *args):
        """更新高度步长显示"""
        step = self.height_step_var.get()
        self.height_step_label.config(text=f"{step:.1f} m")

    def on_closing(self):
        """窗口关闭时的清理工作"""
        self.position_update_active = False

        # 尝试安全降落所有无人机
        try:
            for name in self.drone_names:
                if self.drone_states.get(name) in ["空中", "起飞中"]:
                    self.client.landAsync(vehicle_name=name)
        except:
            pass

        time.sleep(1)
        self.master.destroy()


def main():
    root = tk.Tk()

    # 设置样式
    style = ttk.Style()
    style.configure("Emergency.TButton", foreground="white", background="red")

    app = DroneControlPanel(root)

    # 绑定关闭事件
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.mainloop()


if __name__ == "__main__":
    main()