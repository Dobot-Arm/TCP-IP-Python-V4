"""
Motion control module tests
运动控制模块测试
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType


def test_motion_methods():
    """
    Test motion control methods
    测试运动控制方法
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试MovJ方法 / Test MovJ method
    assert hasattr(robot.motion, 'MovJ')
    print("MovJ method exists")  # MovJ方法存在 / MovJ method exists
    
    # 测试MovL方法 / Test MovL method
    assert hasattr(robot.motion, 'MovL')
    print("MovL method exists")  # MovL方法存在 / MovL method exists
    
    # 测试Arc方法 / Test Arc method
    assert hasattr(robot.motion, 'Arc')
    print("Arc method exists")  # Arc方法存在 / Arc method exists
    
    # 测试Circle方法 / Test Circle method
    assert hasattr(robot.motion, 'Circle')
    print("Circle method exists")  # Circle方法存在 / Circle method exists
    
    # 测试相对运动方法 / Test relative motion methods
    assert hasattr(robot.motion, 'RelMovL')
    assert hasattr(robot.motion, 'RelMovLTool')
    assert hasattr(robot.motion, 'RelMovLUser')
    print("Relative motion methods exist")  # 相对运动方法存在 / Relative motion methods exist
    
    # 检查 robot_control 模块的速度和加速度设置方法 / Check the methods for setting speed and acceleration in the robot_control module
    assert hasattr(robot.robot_control, 'SpeedFactor')
    assert hasattr(robot.robot_control, 'AccJ')
    assert hasattr(robot.robot_control, 'AccL')
    print("Speed/acceleration setting methods exist (robot_control module)")  # 速度/加速度设置方法存在 (robot_control 模块) / Speed/acceleration setting methods exist (robot_control module)
    
    # 注意：原 motion 模块下的 ToolVoltage 接口在手臂二开md文档中不存在，已移除 / Note: The ToolVoltage method was removed from the motion module because it is absent from the robot arm API documentation.
    print("Tool voltage check: interface not in docs, skipped")  # 工具电压相关检查：文档无此接口，已跳过 / Tool voltage check: interface not in docs, skipped
    
    # 测试伺服控制方法 / Test servo control methods
    assert hasattr(robot.motion, 'ServoJ')
    assert hasattr(robot.motion, 'ServoP')
    print("Servo control methods exist")  # 伺服控制方法存在 / Servo control methods exist
    
    # 测试Jog控制方法 / Test Jog control method
    assert hasattr(robot.motion, 'MoveJog')
    print("Jog control method exists")  # Jog控制方法存在 / Jog control method exists
    
    del robot
    print("Motion control methods test passed!")  # 运动控制方法测试通过 / Motion control methods test passed


def test_motion_command_generation():
    """
    Test motion command generation
    测试运动指令生成
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试MovJ指令生成 / Test MovJ command generation
    result = robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    assert result is not None
    print(f"MovJ command: {result}")  # MovJ指令 / MovJ command
    
    # 测试MovL指令生成 / Test MovL command generation
    result = robot.motion.MovL([400, 100, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    assert result is not None
    print(f"MovL command: {result}")  # MovL指令 / MovL command
    
    # 测试相对运动（展开6个独立offset参数，按文档规范） / Test relative motion (expand 6 independent offset parameters, per docs)
    result = robot.motion.RelMovLTool(50, 0, 0, 0, 0, 0)
    assert result is not None
    print(f"Relative motion command: {result}")  # 相对运动指令 / Relative motion command
    
    # 测试关节运动 / Test joint motion
    result = robot.motion.MovJ([0, -30, -60, 0, 90, 0], CoordinateType.JOINT)
    assert result is not None
    print(f"Joint motion command: {result}")  # 关节运动指令 / Joint motion command
    
    del robot
    print("Motion command generation test passed!")  # 运动指令生成测试通过 / Motion command generation test passed


def test_coordinate_type_enum():
    """
    Test coordinate type enum
    测试坐标系类型枚举
    """
    assert CoordinateType.JOINT == 0
    assert CoordinateType.CARTESIAN == 1
    assert CoordinateType.USER == 1
    assert CoordinateType.TOOL == 2
    assert CoordinateType.CARTESIAN.name == 'CARTESIAN'
    assert CoordinateType.JOINT.name == 'JOINT'
    assert CoordinateType.USER.name == 'USER'
    assert CoordinateType.TOOL.name == 'TOOL'
    print("Coordinate type enum test passed!")  # 坐标系类型枚举测试通过 / Coordinate type enum test passed


if __name__ == "__main__":
    print("Running motion control module tests...\n")  # 运行运动控制模块测试 / Running motion control module tests
    
    test_motion_methods()
    print()
    
    test_motion_command_generation()
    print()
    
    test_coordinate_type_enum()
    print()
    
    print("=" * 50)
    print("All motion control module tests passed!")  # 运动控制模块所有测试通过 / All motion control module tests passed
