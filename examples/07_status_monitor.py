# -*- coding: utf-8 -*-
"""
Example 7: Complete Status Monitoring
示例7: 完整状态监控

Displays all fields parsed from the Feedback data packet
展示从Feedback数据包解析的所有字段
"""

import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot, set_log_level


def print_status(status):
    """
    Print the full robot status information
    打印机器人完整状态信息

    Args:
        status: RobotStatus object
    """
    # 重点监听 RunningStatus / Focus on monitoring RunningStatus
    running_status = status.running_status

    # RunningStatus 值含义（根据协议文档） / Meaning of RunningStatus values (per the protocol document)
    # 0: 空闲 / Idle
    # 1028: 运动中 / Moving
    # 其他值根据实际情况定义 / Other values are defined according to the actual situation

    status_text = "Unknown"  # 未知
    status_icon = ""
    if running_status == 0:
        status_text = "Idle"  # 空闲
        status_icon = "[OK]"
    elif running_status == 1028:
        status_text = "Moving"  # 运动中
        status_icon = "[RUN]"
    else:
        status_text = f"Status code: {running_status}"  # 状态码: {running_status}
        status_icon = "[WARN]"

    print(f"\n{'='*60}")
    print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")  # 时间: {time.strftime(...)}
    print(f"{'='*60}")

    # 运行状态 / Running status
    print(f"\n[Running Status]")  # 【运行状态】
    print(f"  RunningStatus: {status_icon} {status_text}")
    print(f"  Robot mode: {status.robot_mode.name}")  # 机器人模式: {status.robot_mode.name}
    print(f"  Enable status: {'Enabled' if status.enable_status else 'Disabled'}")  # 使能状态: ...
    print(f"  Brake status: {'Released' if status.brake_status else 'Engaged'}")  # 抱闸状态: ...
    print(f"  Error status: {'Error' if status.error_status else 'Normal'}")  # 错误状态: ...
    print(f"  Speed scaling: {status.speed_scaling:.1f}%")  # 速度比例: ...
    print(f"  Drag status: {status.drag_status} (0: not dragging, 1: joint drag, 2: force drag)")  # 拖拽状态: ...
    print(f"  Robot type: {status.robot_type}")  # 机器人型号: ...

    # 笛卡尔坐标 / Cartesian coordinates
    print(f"\n[Cartesian Coordinates]")  # 【笛卡尔坐标】
    print(f"  X: {status.tool_vector_actual.x:.2f} mm")
    print(f"  Y: {status.tool_vector_actual.y:.2f} mm")
    print(f"  Z: {status.tool_vector_actual.z:.2f} mm")
    print(f"  Rx: {status.tool_vector_actual.rx:.2f} °")
    print(f"  Ry: {status.tool_vector_actual.ry:.2f} °")
    print(f"  Rz: {status.tool_vector_actual.rz:.2f} °")

    # TCP速度 / TCP speed
    print(f"\n[TCP Speed]")  # 【TCP速度】
    print(f"  Vx: {status.tcp_speed_actual.x:.2f} mm/s")
    print(f"  Vy: {status.tcp_speed_actual.y:.2f} mm/s")
    print(f"  Vz: {status.tcp_speed_actual.z:.2f} mm/s")

    # 关节角度 / Joint angles
    print(f"\n[Joint Angles]")  # 【关节角度】
    joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    for i, (name, angle) in enumerate(zip(joint_names, status.joint_state.q_actual)):
        print(f"  {name}: {angle:.2f} °")

    # 关节速度 / Joint speeds
    print(f"\n[Joint Speeds]")  # 【关节速度】
    for i, (name, speed) in enumerate(zip(joint_names, status.joint_state.qd_actual)):
        print(f"  {name}: {speed:.2f} °/s")

    # 电气信息 / Electrical information
    print(f"\n[Electrical Info]")  # 【电气信息】
    print(f"  Voltage: {status.voltage:.2f} V")  # 电压: ...
    print(f"  Current: {status.current:.2f} A")  # 电流: ...
    print(f"  Load: {status.load:.2f} kg")  # 负载: ...
    print(f"  Load center: ({status.load_center_x:.1f}, {status.load_center_y:.1f}, {status.load_center_z:.1f}) mm")  # 负载中心: ...

    # 电机温度 / Motor temperatures
    print(f"\n[Motor Temperatures]")  # 【电机温度】
    for i, (name, temp) in enumerate(zip(joint_names, status.joint_state.temperatures)):
        print(f"  {name}: {temp:.1f} °C")

    # IO状态 / IO status
    print(f"\n[IO Status]")  # 【IO状态】
    print(f"  Digital inputs: {bin(status.digital_inputs)}")  # 数字输入: ...
    print(f"  Digital outputs: {bin(status.digital_outputs)}")  # 数字输出: ...
    print(f"  Safety IO inputs: {status.safety_io_in}")  # 安全IO输入: ...
    print(f"  Safety IO outputs: {status.safety_io_out}")  # 安全IO输出: ...

    # 速度/加速度比例 / Speed/acceleration ratios
    print(f"\n[Speed/Acceleration Ratios]")  # 【速度/加速度比例】
    print(f"  Joint speed: {status.velocity_ratio}%")  # 关节速度: ...
    print(f"  Joint acceleration: {status.acceleration_ratio}%")  # 关节加速度: ...
    print(f"  Cartesian position speed: {status.xyz_velocity_ratio}%")  # 笛卡尔位置速度: ...
    print(f"  Cartesian posture speed: {status.r_velocity_ratio}%")  # 笛卡尔姿态速度: ...
    print(f"  Cartesian position acceleration: {status.xyz_acceleration_ratio}%")  # 笛卡尔位置加速度: ...
    print(f"  Cartesian posture acceleration: {status.r_acceleration_ratio}%")  # 笛卡尔姿态加速度: ...

    # 坐标系 / Coordinate systems
    print(f"\n[Coordinate Systems]")  # 【坐标系】
    print(f"  User coordinate: {status.user_coordinate}")  # 用户坐标系: ...
    print(f"  Tool coordinate: {status.tool_coordinate}")  # 工具坐标系: ...

    # 四元数 / Quaternion
    print(f"\n[Quaternion]")  # 【四元数】
    print(f"  Target: [{status.target_quaternion.qw:.4f}, {status.target_quaternion.qx:.4f}, {status.target_quaternion.qy:.4f}, {status.target_quaternion.qz:.4f}]")  # 目标: [...]
    print(f"  Actual: [{status.actual_quaternion.qw:.4f}, {status.actual_quaternion.qx:.4f}, {status.actual_quaternion.qy:.4f}, {status.actual_quaternion.qz:.4f}]")  # 实际: [...]

    # 安全状态 / Safety status
    print(f"\n[Safety Status]")  # 【安全状态】
    print(f"  Safety state: {status.get_safety_state_desc()}")  # 安全状态: ...
    print(f"  Collision state: {status.collision_state}")  # 碰撞状态: ...
    print(f"  Lower arm approach: {status.arm_approach_state}")  # 小臂接近: ...
    print(f"  J4 approach: {status.j4_approach_state}")  # J4接近: ...
    print(f"  J5 approach: {status.j5_approach_state}")  # J5接近: ...
    print(f"  J6 approach: {status.j6_approach_state}")  # J6接近: ...

    # 六维力传感器 / Six-axis force sensor
    print(f"\n[Six-axis Force Sensor]")  # 【六维力传感器】
    force_status = {0: 'Offline', 1: 'Online', 2: 'Abnormal'}.get(status.six_force_online, 'Unknown')  # 离线/在线/异常/未知
    print(f"  Online status: {force_status}")  # 在线状态: ...
    if status.six_force_online == 1:
        print(f"  Raw values: {[f'{v:.2f}' for v in status.six_force_value]}")  # 原始值: ...

    # 末端按钮信号 / End-effector button signals
    print(f"\n[End-effector Button Signals]")  # 【末端按钮信号】
    print(f"  Drag button: {status.drag_button_signal}")  # 拖拽按钮: ...
    print(f"  Enable button: {status.enable_button_signal}")  # 使能按钮: ...
    print(f"  Record button: {status.record_button_signal}")  # 录制按钮: ...
    print(f"  Replay button: {status.reappear_button_signal}")  # 复现按钮: ...
    print(f"  Jaw button: {status.jaw_button_signal}")  # 夹爪按钮: ...

    # 指令信息 / Command information
    print(f"\n[Command Info]")  # 【指令信息】
    print(f"  Current command ID: {status.current_command_id}")  # 当前指令ID: ...
    print(f"  Runtime: {status.run_time // 1000} seconds")  # 运行时间: ... 秒
    print(f"  Queued running: {status.run_queued_cmd}")  # 队列运行: ...
    print(f"  Queue paused: {status.pause_cmd_flag}")  # 队列暂停: ...

    # 模式 / Modes
    print(f"\n[Modes]")  # 【模式】
    print(f"  Manual/Auto: {status.auto_manual_mode}")  # 手动/自动: ...
    print(f"  USB export: {status.export_status}")  # U盘导出: ...

    # 抖动检测 / Vibration detection
    print(f"\n[Vibration Detection]")  # 【抖动检测】
    print(f"  Z-axis vibration displacement: {status.vibration_dis_z:.4f} mm")  # Z轴抖动位移: ...

    print(f"\n{'='*60}")
    print("Press Ctrl+C to stop monitoring")  # 按 Ctrl+C 停止监听


def main():
    # 设置日志级别（可选） / Set the log level (optional)
    set_log_level("INFO")

    # 机器人IP地址 / Robot IP address
    ROBOT_IP = "192.168.1.100"

    print(f"Connecting to robot {ROBOT_IP}...")  # 正在连接机器人 {ROBOT_IP}...

    # 创建机器人对象，设置超时时间 / Create the robot object and set the timeout
    robot = DobotRobot(
        ROBOT_IP,
        connect_timeout=10.0,
        receive_timeout=15.0
    )

    try:
        # 连接机器人 / Connect to the robot
        robot.Connect()
        print("Connection successful!")  # 连接成功！

        # 请求控制权 / Request control
        robot.robot_control.RequestControl()

        # 清除错误 / Clear errors
        robot.robot_control.ClearError()

        # 启动反馈监控 / Start feedback monitoring
        robot.StartFeedbackMonitor(callback=print_status)

        print("\nStart monitoring robot status...")  # 开始监听机器人状态...
        print("Press Ctrl+C to stop monitoring\n")  # 按 Ctrl+C 停止监听

        # 持续监听 / Keep monitoring
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nUser stopped monitoring")  # 用户停止监听
    except Exception as e:
        print(f"\nError: {e}")  # 错误: {e}
        import traceback
        traceback.print_exc()
    finally:
        # 停止监控并断开连接 / Stop monitoring and disconnect
        robot.StopFeedbackMonitor()
        robot.Disconnect()
        print("Disconnected")  # 已断开连接


if __name__ == "__main__":
    main()
