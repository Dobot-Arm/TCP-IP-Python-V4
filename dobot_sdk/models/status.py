# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Robot status data model
机器人状态数据模型
"""

from dataclasses import dataclass, field
from typing import List
from enum import IntEnum


class RobotMode(IntEnum):
    """Robot operating mode
    机器人运行模式

    Reference: TCP/IP protocol documentation
    参考TCP/IP 协议文档
    """
    UNKNOWN = 0         # 未知/未初始化（Feedback数据包可能返回） / Unknown/Uninitialized (may be returned by Feedback packets)
    INIT = 1           # 初始化状态 / Initialization state
    BRAKE_OPEN = 2     # 有任意关节抱闸松开 / Any joint brake released
    POWEROFF = 3       # 下电状态 / Power-off state
    DISABLED = 4       # 未使能（无抱闸松开） / Disabled (no brake released)
    ENABLE = 5         # 使能且空闲 / Enabled and idle
    BACKDRIVE = 6      # 拖拽模式 / Backdrive mode
    RUNNING = 7        # 运行状态（工程、TCP队列运动等） / Running state (project, TCP queue motion, etc.)
    SINGLE_MOVE = 8    # 单次运动状态（点动、RunTo等） / Single-motion state (jog, RunTo, etc.)
    ERROR = 9          # 报警状态 / Error state
    PAUSE = 10         # 暂停状态 / Pause state
    JOG = 11           # 点动状态 / Jog state


@dataclass
class JointState:
    """Joint state / 关节状态"""
    q_actual: List[float] = field(default_factory=lambda: [0.0] * 6)  # 实际角度 / Actual angle
    q_target: List[float] = field(default_factory=lambda: [0.0] * 6)  # 目标角度 / Target angle
    qd_actual: List[float] = field(default_factory=lambda: [0.0] * 6)  # 实际速度 / Actual velocity
    qd_target: List[float] = field(default_factory=lambda: [0.0] * 6)  # 目标速度 / Target velocity
    qdd_target: List[float] = field(default_factory=lambda: [0.0] * 6)  # 目标加速度 / Target acceleration
    i_actual: List[float] = field(default_factory=lambda: [0.0] * 6)  # 实际电流 / Actual current
    i_target: List[float] = field(default_factory=lambda: [0.0] * 6)  # 目标电流 / Target current
    m_actual: List[float] = field(default_factory=lambda: [0.0] * 6)  # 实际扭矩 / Actual torque
    m_target: List[float] = field(default_factory=lambda: [0.0] * 6)  # 目标扭矩 / Target torque
    temperatures: List[float] = field(default_factory=lambda: [0.0] * 6)  # 电机温度 / Motor temperature
    joint_modes: List[float] = field(default_factory=lambda: [0.0] * 6)  # 关节控制模式 / Joint control mode
    voltages: List[float] = field(default_factory=lambda: [0.0] * 6)  # 关节电压 / Joint voltage


@dataclass
class CartesianPose:
    """Cartesian pose / 笛卡尔位姿"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def to_list(self) -> List[float]:
        """Convert to a list / 转换为列表"""
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]


@dataclass
class Quaternion:
    """Quaternion [qw, qx, qy, qz] / 四元数 [qw, qx, qy, qz]"""
    qw: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    def to_list(self) -> List[float]:
        """Convert to a list / 转换为列表"""
        return [self.qw, self.qx, self.qy, self.qz]


@dataclass
class RobotStatus:
    """
    Complete robot status
    机器人完整状态

    Parsed from a Feedback packet (1440 bytes)
    从Feedback数据包解析得到（1440字节）
    """
    # 基本状态 / Basic status
    robot_mode: RobotMode = RobotMode.INIT
    speed_scaling: float = 0.0  # 速度比例 0-100 / Speed scaling 0-100
    enable_status: bool = False  # 使能状态 / Enable status
    brake_status: bool = False  # 抱闸状态 / Brake status
    error_status: bool = False  # 错误状态 / Error status
    running_status: int = 0  # 运行状态（字节位置: 1028，0表示空闲，非0表示运动中） / Running status (byte offset: 1028, 0=idle, non-zero=moving)
    drag_status: int = 0  # 拖拽状态（0:不在拖拽，1:关节拖拽，2:力控拖拽） / Drag status (0:not dragging, 1:joint drag, 2:force-control drag)
    jog_status: int = 0  # 点动状态 / Jog status
    robot_type: int = 0  # 机器人型号 / Robot type

    # IO状态 / IO status
    digital_inputs: int = 0  # 数字输入（64位） / Digital inputs (64-bit)
    digital_outputs: int = 0  # 数字输出（64位） / Digital outputs (64-bit)
    safety_io_in: int = 0  # 安全IO输入 / Safety IO input
    safety_io_out: int = 0  # 安全IO输出 / Safety IO output

    # 时间信息 / Time information
    timestamp: int = 0  # Unix时间戳（ms） / Unix timestamp (ms)
    run_time: int = 0  # 开机运行时间（ms） / Runtime since power-on (ms)

    # 电气信息 / Electrical information
    voltage: float = 0.0  # 机器人电压 / Robot voltage
    current: float = 0.0  # 机器人电流 / Robot current

    # 程序状态 / Program status
    program_state: float = 0.0  # 脚本运行状态 / Script running status

    # 当前指令ID / Current command ID
    current_command_id: int = 0

    # 关节状态 / Joint state
    joint_state: JointState = field(default_factory=JointState)

    # 笛卡尔位姿 / Cartesian pose
    tool_vector_actual: CartesianPose = field(default_factory=CartesianPose)
    tool_vector_target: CartesianPose = field(default_factory=CartesianPose)
    tcp_speed_actual: CartesianPose = field(default_factory=CartesianPose)  # TCP实际速度 / TCP actual speed
    tcp_speed_target: CartesianPose = field(default_factory=CartesianPose)  # TCP目标速度 / TCP target speed

    # 力信息 / Force information
    actual_tcp_force: List[float] = field(default_factory=lambda: [0.0] * 6)  # TCP各轴受力（六维力计算） / TCP axis forces (6-axis force calculation)
    tcp_force: List[float] = field(default_factory=lambda: [0.0] * 6)  # TCP力值（由关节电流计算出力值） / TCP force values (calculated from joint current)

    # 负载信息 / Load information
    load: float = 0.0  # 末端负载重量（kg） / End-effector load weight (kg)
    load_center_x: float = 0.0  # 负载X偏心（mm） / Load X eccentricity (mm)
    load_center_y: float = 0.0  # 负载Y偏心（mm） / Load Y eccentricity (mm)
    load_center_z: float = 0.0  # 负载Z偏心（mm） / Load Z eccentricity (mm)

    # 坐标系 / Coordinate system
    user_coordinate: int = 0  # 用户坐标系编号 / User coordinate system index
    tool_coordinate: int = 0  # 工具坐标系编号 / Tool coordinate system index
    user_value: List[float] = field(default_factory=lambda: [0.0] * 6)  # 用户坐标系坐标值 / User coordinate system values
    tool_value: List[float] = field(default_factory=lambda: [0.0] * 6)  # 工具坐标系坐标值 / Tool coordinate system values

    # 四元数 / Quaternion
    target_quaternion: Quaternion = field(default_factory=Quaternion)  # 目标四元数 / Target quaternion
    actual_quaternion: Quaternion = field(default_factory=Quaternion)  # 实际四元数 / Actual quaternion

    # 速度/加速度比例 / Velocity/acceleration ratio
    velocity_ratio: int = 0  # 关节速度比例（0-100） / Joint velocity ratio (0-100)
    acceleration_ratio: int = 0  # 关节加速度比例（0-100） / Joint acceleration ratio (0-100)
    xyz_velocity_ratio: int = 0  # 笛卡尔位置速度比例（0-100） / Cartesian position velocity ratio (0-100)
    r_velocity_ratio: int = 0  # 笛卡尔姿态速度比例（0-100） / Cartesian orientation velocity ratio (0-100)
    xyz_acceleration_ratio: int = 0  # 笛卡尔位置加速度比例（0-100） / Cartesian position acceleration ratio (0-100)
    r_acceleration_ratio: int = 0  # 笛卡尔姿态加速度比例（0-100） / Cartesian orientation acceleration ratio (0-100)

    # 队列状态 / Queue status
    run_queued_cmd: int = 0  # 算法队列运行标志 / Algorithm queue running flag
    pause_cmd_flag: int = 0  # 算法队列暂停标志 / Algorithm queue pause flag

    # 手系 / Hand type
    hand_type: List[int] = field(default_factory=lambda: [0] * 4)  # 手系（备用参数） / Hand type (reserved parameter)

    # 末端按钮信号 / End-effector button signals
    drag_button_signal: int = 0  # 拖拽按钮信号 / Drag button signal
    enable_button_signal: int = 0  # 使能按钮信号 / Enable button signal
    record_button_signal: int = 0  # 录制按钮信号 / Record button signal
    reappear_button_signal: int = 0  # 复现按钮信号 / Replay button signal
    jaw_button_signal: int = 0  # 夹爪控制信号 / Gripper control signal

    # 六维力传感器 / 6-axis force sensor
    six_force_online: int = 0  # 六维力在线状态（0:离线，1:在线，2:异常） / 6-axis force online status (0:offline, 1:online, 2:abnormal)
    six_force_value: List[float] = field(default_factory=lambda: [0.0] * 6)  # 六维力原始值 / 6-axis force raw values

    # 安全状态 / Safety status
    collision_state: int = 0  # 碰撞状态 / Collision state
    arm_approach_state: int = 0  # 小臂安全皮肤接近暂停 / Forearm safety skin proximity pause
    j4_approach_state: int = 0  # J4安全皮肤接近暂停 / J4 safety skin proximity pause
    j5_approach_state: int = 0  # J5安全皮肤接近暂停 / J5 safety skin proximity pause
    j6_approach_state: int = 0  # J6安全皮肤接近暂停 / J6 safety skin proximity pause
    safety_state: int = 0  # 安全状态（位掩码） / Safety state (bitmask)

    # 抖动检测 / Vibration detection
    vibration_dis_z: float = 0.0  # Z轴抖动位移 / Z-axis vibration displacement

    # 模式 / Mode
    auto_manual_mode: int = 0  # 手动/自动模式 / Manual/auto mode
    export_status: int = 0  # U盘导出状态 / USB export status

    def is_ready(self) -> bool:
        """Check if the robot is ready / 检查机器人是否就绪"""
        return (
            self.robot_mode == RobotMode.ENABLE and
            not self.error_status and
            self.enable_status
        )

    def has_error(self) -> bool:
        """Check if there is an error / 检查是否有错误"""
        return self.robot_mode == RobotMode.ERROR or self.error_status

    def is_moving(self) -> bool:
        """Check if the robot is moving / 检查机器人是否运动中

        RunningStatus byte offset: 1028
        RunningStatus 字节位置: 1028
        Status values:
        状态值说明:
        - 0: Idle / 空闲
        - Non-zero: Moving (see the protocol documentation for specific values) / 运动中（具体值需参考协议文档）
        """
        return self.running_status != 0

    def get_safety_state_desc(self) -> str:
        """Get a safety state description / 获取安全状态描述"""
        states = []
        if self.safety_state & 0x01: states.append("Emergency stop")  # 急停 / Emergency stop
        if self.safety_state & 0x02: states.append("Protective stop")  # 防护性停止 / Protective stop
        if self.safety_state & 0x04: states.append("Reduced mode")  # 缩减模式 / Reduced mode
        if self.safety_state & 0x08: states.append("Non-stop")  # 非停止 / Non-stop
        if self.safety_state & 0x10: states.append("Moving")  # 运动中 / Moving
        if self.safety_state & 0x20: states.append("System emergency stop")  # 系统急停 / System emergency stop
        if self.safety_state & 0x40: states.append("User emergency stop")  # 用户急停 / User emergency stop
        if self.safety_state & 0x80: states.append("Safety origin")  # 安全原点 / Safety origin
        return ", ".join(states) if states else "Normal"  # 正常 / Normal
