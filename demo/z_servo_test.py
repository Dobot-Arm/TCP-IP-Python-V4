"""
Servo control test
# 伺服控制测试

Demonstrates ServoP trajectory following along a circular path. / 演示使用 ServoP 跟随圆形轨迹。
"""

import sys
import os

# Add parent directory to path for importing dobot_sdk / 添加父目录到路径，以便导入dobot_sdk
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import time
from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType


def parse_dobot_response(response_str: str) -> tuple:
    """Parse the string-format data returned by Dobot and extract each part
    # 解析Dobot返回的字符串格式数据，提取各部分内容

    Format example: 'part1,{1,2,3},part3'
    # 格式示例: 'part1,{1,2,3},part3'

    Args:
    # 参数:
        response_str: String to be parsed
        # response_str: 待解析的字符串

    Returns:
    # 返回:
        tuple: (prefix part, number list, suffix part), returns None on parse failure
        # tuple: (前缀部分, 数字列表, 后缀部分)，解析失败返回None
    """
    first_comma_idx = response_str.find(',')
    if first_comma_idx == -1:
        return None

    prefix = response_str[:first_comma_idx]
    remaining = response_str[first_comma_idx + 1:]

    if not remaining.startswith('{'):
        return None

    brace_depth = 1
    i = 1
    while i < len(remaining) and brace_depth > 0:
        if remaining[i] == '{':
            brace_depth += 1
        elif remaining[i] == '}':
            brace_depth -= 1
        i += 1

    if brace_depth != 0:
        return None

    middle_with_braces = remaining[:i]
    suffix = remaining[i + 1:] if i < len(remaining) else ""

    numbers_str = middle_with_braces[1:-1]
    number_list = [float(x.strip()) for x in numbers_str.split(',')]

    return prefix, number_list, suffix


def generate_circular_trajectory(radius: float, num_points: int) -> list:
    """Generate circular motion trajectory points
    # 生成圆周运动轨迹点

    Args:
    # 参数:
        radius: Circle radius (mm)
        # radius: 圆周半径(mm)
        num_points: Number of trajectory points
        # num_points: 轨迹点数

    Returns:
    # 返回:
        list: A list of trajectory points, each in the form [x, y, z, rx, ry, rz].
        # list: 轨迹点列表，每个点为[x, y, z, rx, ry, rz]
    """
    trajectory_points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        trajectory_points.append([x, y, 0, 0, 0, 0])
    return trajectory_points

def generate_circle_smooth(radius=50, total_points=300):
    points = []
    for i in range(total_points):
        s = i / (total_points - 1)
        angle = 2 * math.pi * (0.5 - 0.5 * math.cos(math.pi * s))
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        points.append([x, y, 0, 0, 0, 0])
    return points

def execute_trajectory_at_frequency(robot, trajectory_points, frequency_hz: float):
    """Execute the trajectory point sequence at a specified frequency.
    # 以指定频率执行轨迹点序列

    Args:
    # 参数:
        robot: DobotRobot instance
        # robot: DobotRobot实例
        trajectory_points: List of trajectory points
        # trajectory_points: 轨迹点列表
        frequency_hz: Execution frequency (Hz)
        # frequency_hz: 执行频率(Hz)
    """
    start_time = time.time()
    interval = 1.0 / frequency_hz

    for i, target_point in enumerate(trajectory_points):
        expected_time = start_time + (i + 1) * interval
        cycle_start = time.time()

        # Use the new SDK ServoP interface / 使用新SDK的ServoP接口
        robot.motion.ServoP(target_point)

        cycle_duration = time.time() - cycle_start
        current_time = time.time()
        delay_needed = expected_time - current_time

        if delay_needed > 0:
            time.sleep(delay_needed)
        else:
            print(f"Cycle {i + 1} timed out by {-delay_needed:.3f} seconds")  # 周期超时 / Cycle timed out

def wait_for_robot_ready(robot, target_state=5.0, polling_interval=0.1):
    """Wait for the robot to enter the ready state
    # 等待机器人进入就绪状态

    Args:
    # 参数:
        robot: DobotRobot instance
        # robot: DobotRobot实例
        target_state: Target state value
        # target_state: 目标状态值
        polling_interval: Polling interval (seconds)
        # polling_interval: 轮询间隔(秒)
    """
    while True:
        status_response = robot.robot_control.RobotMode()
        parsed = parse_dobot_response(status_response)

        if parsed is None:
            print("Robot status parse failed")  # 机器人状态解析失败 / Robot status parse failed
            time.sleep(polling_interval)
            continue

        _, state_data, _ = parsed

        if state_data and state_data[0] == target_state:
            print(f"Robot is ready, state: {state_data}")  # 机器人已就绪 / Robot is ready
            break

        time.sleep(polling_interval)


def main():
    # Set this value to the robot's actual IP address. / 修改为实际机器人IP
    ROBOT_IP = "192.168.5.1"

    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)  # 分隔线 / Separator
            print("Servo Control Test - Circular Trajectory Following")  # 伺服控制测试 - 圆周轨迹跟随 / Servo Control Test - Circular Trajectory Following
            print("=" * 50)  # 分隔线 / Separator

            # Initialization / 初始化
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)

            # Activate user coordinate system 0 / 激活用户坐标系0
            robot.robot_control.User(0)

            # Get current position / 获取当前位置
            pose_response = robot.robot_control.GetPose()
            parsed_pose = parse_dobot_response(pose_response)

            if parsed_pose is None:
                print("Error: failed to parse position information")  # 位置信息解析失败 / Failed to parse position information
                return

            _, current_position, _ = parsed_pose
            print(f"Current position: {current_position}")  # 当前位置 / Current position

            # Set user coordinate system 1 (using current position as origin) / 设置用户坐标系1（以当前位置为原点）
            set_user_result = robot.robot_control.SetUser(1, current_position)
            print(f"Set user coordinate system result: {set_user_result}")  # 设置用户坐标系结果 / Set user coordinate system result

            # Switch to user coordinate system 1 / 切换到用户坐标系1
            robot.robot_control.User(1)

            # Move to trajectory start point (relative position in user coordinate system) / 移动到轨迹起始点（用户坐标系中的相对位置）
            start_point = [50, 0, 0, 0, 0, 0]
            print(f"\nMoving to trajectory start point: {start_point}")  # 移动到轨迹起始点 / Moving to trajectory start point
            robot.motion.MovL(start_point, CoordinateType.CARTESIAN)
            time.sleep(3)

            # Wait for robot to be ready / 等待机器人就绪
            wait_for_robot_ready(robot)

            # Generate circular trajectory / 生成圆周轨迹
            # trajectory_points = generate_circular_trajectory(radius=50, num_points=300)
            trajectory_points = generate_circle_smooth(radius=50, total_points=300)
            print(f"\nGenerated {len(trajectory_points)} trajectory points")  # 生成轨迹点 / Generated trajectory points

            # Execute trajectory control / 执行轨迹控制
            print("Starting trajectory control...")  # 开始执行轨迹控制 / Starting trajectory control
            execute_trajectory_at_frequency(robot, trajectory_points, frequency_hz=33.0)

            print("\nTrajectory execution completed")  # 轨迹执行完成 / Trajectory execution completed

    except KeyboardInterrupt:
        print("\nUser interrupted")  # 用户中断 / User interrupted
    except Exception as e:
        print(f"\nError: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
