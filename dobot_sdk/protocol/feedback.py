# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Feedback protocol
反馈协议

Parse robot status feedback data
解析机器人状态反馈数据
"""

import numpy as np
from typing import Optional
from .data_types import MyType, TEST_VALUE_MAGIC
from ..models.status import RobotStatus, RobotMode, JointState, CartesianPose, Quaternion


class FeedbackParser:
    """Feedback data parser / Feedback数据解析器"""

    PACKET_SIZE = 1440

    def parse(self, data: bytes) -> Optional[RobotStatus]:
        """
        Parse the Feedback data packet
        解析Feedback数据包

        Args:
            data: 1440-byte raw data / 1440字节的原始数据

        Returns:
            RobotStatus or None / RobotStatus或None
        """
        if len(data) != self.PACKET_SIZE:
            return None

        try:
            parsed = np.frombuffer(data, dtype=MyType)

            # Verify the magic number / 验证魔数
            if hex(parsed['TestValue'][0]) != hex(TEST_VALUE_MAGIC):
                return None

            return self._build_status(parsed)

        except Exception as e:
            print(f"Failed to parse Feedback: {e}")  # 解析Feedback失败
            return None

    def _build_status(self, parsed: np.ndarray) -> RobotStatus:
        """Build a status object from the NumPy array / 从NumPy数组构建状态对象"""
        status = RobotStatus()

        # Basic status / 基本状态
        robot_mode_value = int(parsed['RobotMode'][0])
        try:
            status.robot_mode = RobotMode(robot_mode_value)
        except ValueError:
            status.robot_mode = RobotMode.INIT

        status.speed_scaling = float(parsed['SpeedScaling'][0])
        status.enable_status = bool(parsed['EnableStatus'][0])
        status.brake_status = bool(parsed['BrakeStatus'][0])
        status.error_status = bool(parsed['ErrorStatus'][0])
        status.running_status = int(parsed['RunningStatus'][0])
        status.drag_status = int(parsed['DragStatus'][0])
        status.jog_status = int(parsed['JogStatusCR'][0])
        status.robot_type = int(parsed['CRRobotType'][0])

        # IO status / IO状态
        status.digital_inputs = int(parsed['DigitalInputs'][0])
        status.digital_outputs = int(parsed['DigitalOutputs'][0])
        status.safety_io_in = int(parsed['SafetyIOIn'][0])
        status.safety_io_out = int(parsed['SafetyIOOut'][0])

        # Time information / 时间信息
        status.timestamp = int(parsed['TimeStamp'][0])
        status.run_time = int(parsed['RunTime'][0])

        # Electrical information / 电气信息
        status.voltage = float(parsed['VRobot'][0])
        status.current = float(parsed['IRobot'][0])

        # Program status / 程序状态
        status.program_state = float(parsed['ProgramState'][0])

        # Current command ID / 当前指令ID
        status.current_command_id = int(parsed['CurrentCommandId'][0])

        # Joint status / 关节状态
        status.joint_state = JointState(
            q_actual=parsed['QActual'][0].tolist(),
            q_target=parsed['QTarget'][0].tolist(),
            qd_actual=parsed['QDActual'][0].tolist(),
            qd_target=parsed['QDTarget'][0].tolist(),
            qdd_target=parsed['QDDTarget'][0].tolist(),
            i_actual=parsed['IActual'][0].tolist(),
            i_target=parsed['ITarget'][0].tolist(),
            m_actual=parsed['MActual'][0].tolist(),
            m_target=parsed['MTarget'][0].tolist(),
            temperatures=parsed['MotorTemperatures'][0].tolist(),
            joint_modes=parsed['JointModes'][0].tolist(),
            voltages=parsed['VActual'][0].tolist()
        )

        # Cartesian pose / 笛卡尔位姿
        actual = parsed['ToolVectorActual'][0]
        status.tool_vector_actual = CartesianPose(
            x=float(actual[0]), y=float(actual[1]), z=float(actual[2]),
            rx=float(actual[3]), ry=float(actual[4]), rz=float(actual[5])
        )

        target = parsed['ToolVectorTarget'][0]
        status.tool_vector_target = CartesianPose(
            x=float(target[0]), y=float(target[1]), z=float(target[2]),
            rx=float(target[3]), ry=float(target[4]), rz=float(target[5])
        )

        # TCP speed / TCP速度
        tcp_speed_actual = parsed['TCPSpeedActual'][0]
        status.tcp_speed_actual = CartesianPose(
            x=float(tcp_speed_actual[0]), y=float(tcp_speed_actual[1]), z=float(tcp_speed_actual[2]),
            rx=float(tcp_speed_actual[3]), ry=float(tcp_speed_actual[4]), rz=float(tcp_speed_actual[5])
        )

        tcp_speed_target = parsed['TCPSpeedTarget'][0]
        status.tcp_speed_target = CartesianPose(
            x=float(tcp_speed_target[0]), y=float(tcp_speed_target[1]), z=float(tcp_speed_target[2]),
            rx=float(tcp_speed_target[3]), ry=float(tcp_speed_target[4]), rz=float(tcp_speed_target[5])
        )

        # Force information / 力信息
        status.actual_tcp_force = parsed['ActualTCPForce'][0].tolist()
        status.tcp_force = parsed['TCPForce'][0].tolist()

        # Load information / 负载信息
        status.load = float(parsed['Load'][0])
        status.load_center_x = float(parsed['CenterX'][0])
        status.load_center_y = float(parsed['CenterY'][0])
        status.load_center_z = float(parsed['CenterZ'][0])

        # Coordinate systems / 坐标系
        status.user_coordinate = int(parsed['User'][0])
        status.tool_coordinate = int(parsed['Tool'][0])
        status.user_value = parsed['UserValue'][0].tolist()
        status.tool_value = parsed['ToolValue'][0].tolist()

        # Quaternion / 四元数
        target_quat = parsed['TargetQuaternion'][0]
        status.target_quaternion = Quaternion(
            qw=float(target_quat[0]), qx=float(target_quat[1]),
            qy=float(target_quat[2]), qz=float(target_quat[3])
        )

        actual_quat = parsed['ActualQuaternion'][0]
        status.actual_quaternion = Quaternion(
            qw=float(actual_quat[0]), qx=float(actual_quat[1]),
            qy=float(actual_quat[2]), qz=float(actual_quat[3])
        )

        # Velocity/acceleration ratios / 速度/加速度比例
        status.velocity_ratio = int(parsed['VelocityRatio'][0])
        status.acceleration_ratio = int(parsed['AccelerationRatio'][0])
        status.xyz_velocity_ratio = int(parsed['XYZVelocityRatio'][0])
        status.r_velocity_ratio = int(parsed['RVelocityRatio'][0])
        status.xyz_acceleration_ratio = int(parsed['XYZAccelerationRatio'][0])
        status.r_acceleration_ratio = int(parsed['RAccelerationRatio'][0])

        # Queue status / 队列状态
        status.run_queued_cmd = int(parsed['RunQueuedCmd'][0])
        status.pause_cmd_flag = int(parsed['PauseCmdFlag'][0])

        # Hand type / 手系
        status.hand_type = parsed['HandType'][0].tolist()

        # End-effector button signals / 末端按钮信号
        status.drag_button_signal = int(parsed['DragButtonSignal'][0])
        status.enable_button_signal = int(parsed['EnableButtonSignal'][0])
        status.record_button_signal = int(parsed['RecordButtonSignal'][0])
        status.reappear_button_signal = int(parsed['ReappearButtonSignal'][0])
        status.jaw_button_signal = int(parsed['JawButtonSignal'][0])

        # Six-axis force sensor / 六维力传感器
        status.six_force_online = int(parsed['SixForceOnline'][0])
        status.six_force_value = parsed['SixForceValue'][0].tolist()

        # Safety status / 安全状态
        status.collision_state = int(parsed['CollisionState'][0])
        status.arm_approach_state = int(parsed['ArmApproachState'][0])
        status.j4_approach_state = int(parsed['J4ApproachState'][0])
        status.j5_approach_state = int(parsed['J5ApproachState'][0])
        status.j6_approach_state = int(parsed['J6ApproachState'][0])
        status.safety_state = int(parsed['SafetyState'][0])
        status.safe_state = int(parsed['SafeState'][0])

        # Vibration detection / 抖动检测
        status.vibration_dis_z = float(parsed['VibrationDisZ'][0])

        # Mode / 模式
        status.auto_manual_mode = int(parsed['AutoManualMode'][0])
        status.export_status = int(parsed['ExportStatus'][0])

        return status
