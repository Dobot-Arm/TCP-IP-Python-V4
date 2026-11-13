"""
Basic functionality tests
基础功能测试
"""

import sys
import os

# 添加SDK路径 / Add SDK path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot
from dobot_sdk.models import RobotStatus, RobotMode
from dobot_sdk import CoordinateType


def test_import():
    """
    Test imports
    测试导入
    """
    print("Import successful")  # 导入成功 / Import successful
    assert DobotRobot is not None
    print("DobotRobot class available")  # DobotRobot类可用 / DobotRobot class available
    assert CoordinateType is not None
    print("CoordinateType enum available")  # CoordinateType枚举可用 / CoordinateType enum available


def test_models():
    """
    Test data models
    测试数据模型
    """
    # 测试RobotStatus / Test RobotStatus
    status = RobotStatus()
    assert status.robot_mode == RobotMode.INIT
    assert status.speed_scaling == 0.0
    print("RobotStatus model OK")  # RobotStatus模型正常 / RobotStatus model OK
    
    # 测试RobotMode枚举 / Test RobotMode enum
    assert RobotMode.ENABLE == 5
    assert RobotMode.ERROR == 9
    print("RobotMode enum OK")  # RobotMode枚举正常 / RobotMode enum OK


def test_robot_creation():
    """
    Test robot object creation
    测试机器人对象创建
    """
    robot = DobotRobot("192.168.1.100")
    assert robot.ip == "192.168.1.100"
    
    # 检查实际存在的模块 / Check actually existing modules
    assert robot.motion is not None
    print("motion module available")  # motion模块可用 / motion module available
    
    assert robot.io is not None
    print("io module available")  # io模块可用 / io module available
    
    assert robot.communication is not None
    print("communication module available")  # communication模块可用 / communication module available
    
    assert robot.plugins is not None
    print("plugins module available")  # plugins模块可用 / plugins module available
    
    assert robot.robot_control is not None
    print("robot_control module available")  # robot_control模块可用 / robot_control module available
    
    print("Robot object created successfully")  # 机器人对象创建成功 / Robot object created successfully
    
    # 清理 / Cleanup
    del robot


def test_coordinate_type():
    """
    Test CoordinateType enum
    测试CoordinateType枚举
    """
    assert CoordinateType.JOINT == 0
    assert CoordinateType.CARTESIAN == 1
    assert CoordinateType.USER == 1
    assert CoordinateType.TOOL == 2
    print("CoordinateType enum values correct")  # CoordinateType枚举值正确 / CoordinateType enum values correct


if __name__ == "__main__":
    print("Running basic tests...\n")  # 运行基础测试 / Running basic tests
    
    test_import()
    print()
    
    test_models()
    print()
    
    test_robot_creation()
    print()
    
    test_coordinate_type()
    print()
    
    print("=" * 50)
    print("All tests passed!")  # 所有测试通过 / All tests passed
