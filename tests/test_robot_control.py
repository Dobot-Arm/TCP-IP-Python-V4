"""
Robot control module tests
机器人控制模块测试
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_robot_control_methods():
    """
    Test robot control methods
    测试机器人控制方法
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试请求控制方法 / Test request control methods
    assert hasattr(robot.robot_control, 'RequestControl')
    assert hasattr(robot.robot_control, 'EnableRobot')
    print("Control request methods exist")  # 控制请求方法存在 / Control request methods exist
    
    # 检查使能和下使能方法 / Check the methods for enabling and disabling the robot
    assert hasattr(robot.robot_control, 'EnableRobot')
    assert hasattr(robot.robot_control, 'DisableRobot')
    print("Enable control methods exist")  # 使能控制方法存在 / Enable control methods exist
    
    # 测试错误处理方法 / Test error handling methods
    assert hasattr(robot.robot_control, 'ClearError')
    assert hasattr(robot.robot_control, 'GetErrorID')
    print("Error handling methods exist")  # 错误处理方法存在 / Error handling methods exist
    
    # 测试状态查询方法 / Test status query methods
    assert hasattr(robot.robot_control, 'GetPose')
    assert hasattr(robot.robot_control, 'GetAngle')
    assert hasattr(robot.robot_control, 'RobotMode')
    print("Status query methods exist")  # 状态查询方法存在 / Status query methods exist
    
    # 测试速度设置方法 / Test speed setting method
    assert hasattr(robot.robot_control, 'SpeedFactor')
    print("Speed setting method exists")  # 速度设置方法存在 / Speed setting method exists
    
    # 测试坐标系计算方法 / Test coordinate system calculation methods
    assert hasattr(robot.robot_control, 'CalcUser')
    assert hasattr(robot.robot_control, 'CalcTool')
    print("Coordinate system calculation methods exist")  # 坐标系计算方法存在 / Coordinate system calculation methods exist
    
    # 检查碰撞检测等级设置方法 / Check the method for setting the collision detection level
    assert hasattr(robot.robot_control, 'SetCollisionLevel')
    print("The SetCollisionLevel method exists")  # SetCollisionLevel 方法存在 / The SetCollisionLevel method exists
    
    del robot
    print("Robot control methods test passed!")  # 机器人控制方法测试通过 / Robot control methods test passed


def test_robot_control_parameter_validation():
    """
    Test robot control parameter validation
    测试机器人控制参数验证
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试EnableRobot方法 / Test EnableRobot method
    result = robot.robot_control.EnableRobot(load=1.0)
    assert result is not None
    print(f"EnableRobot(load=1.0): {result}")  # 使能机器人 / Enable robot
    
    # 测试SpeedFactor方法 / Test SpeedFactor method
    result = robot.robot_control.SpeedFactor(50)
    assert result is not None
    print(f"SpeedFactor(50): {result}")  # 速度系数 / Speed factor
    
    # 测试SetCollisionLevel方法（参数范围0-5） / Test SetCollisionLevel method (parameter range 0-5)
    result = robot.robot_control.SetCollisionLevel(3)
    assert result is not None
    print(f"SetCollisionLevel(3): {result}")  # 碰撞检测等级 / Collision detection level
    
    # 测试CalcUser方法 / Test CalcUser method
    result = robot.robot_control.CalcUser(0, 1, [0, 0, 0, 0, 0, 0])
    assert result is not None
    print(f"CalcUser(0, 1, [0,0,0,0,0,0]): {result}")  # 用户坐标系计算 / User coordinate calculation system
    
    del robot
    print("Robot control parameter validation test passed!")  # 机器人控制参数验证测试通过 / Robot control parameter validation test passed


if __name__ == "__main__":
    print("Running robot control module tests...\n")  # 运行机器人控制模块测试 / Running robot control module tests
    
    test_robot_control_methods()
    print()
    
    test_robot_control_parameter_validation()
    print()
    
    print("=" * 50)
    print("All robot control module tests passed!")  # 机器人控制模块所有测试通过 / All robot control module tests passed
