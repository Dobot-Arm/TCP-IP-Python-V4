"""
Plugin module tests (force control and conveyor)
插件模块测试（力控和传送带）
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_plugins_methods():
    """
    Test plugin module methods
    测试插件模块方法
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试力控方法 / Test force control methods
    assert hasattr(robot.plugins, 'FCForceMode')
    assert hasattr(robot.plugins, 'FCSetDeviation')
    assert hasattr(robot.plugins, 'FCSetForceLimit')
    assert hasattr(robot.plugins, 'FCSetMass')
    assert hasattr(robot.plugins, 'FCSetStiffness')
    assert hasattr(robot.plugins, 'FCSetDamping')
    assert hasattr(robot.plugins, 'FCOff')
    assert hasattr(robot.plugins, 'FCSetForce')
    assert hasattr(robot.plugins, 'SetFCCollision')
    assert hasattr(robot.plugins, 'FCCollisionSwitch')
    print("Force control methods exist")  # 力控方法存在 / Force control methods exist
    
    # 测试传送带方法 / Test conveyor methods
    assert hasattr(robot.plugins, 'CnvInit')
    assert hasattr(robot.plugins, 'GetCnvObject')
    assert hasattr(robot.plugins, 'StartSyncCnv')
    assert hasattr(robot.plugins, 'CnvMovL')
    assert hasattr(robot.plugins, 'CnvMovC')
    assert hasattr(robot.plugins, 'StopSyncCnv')
    assert hasattr(robot.plugins, 'SetCnvPointOffset')
    assert hasattr(robot.plugins, 'SetCnvTimeCompensation')
    print("Conveyor methods exist")  # 传送带方法存在 / Conveyor methods exist
    
    del robot
    print("Plugin module methods test passed!")  # 插件模块方法测试通过 / Plugin module methods test passed


def test_plugins_command_generation():
    """
    Test plugin command generation
    测试插件指令生成
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试力控模式设置 / Test force control mode setting
    result = robot.plugins.FCForceMode([1,1,1,0,0,0], [0,0,0,0,0,0])
    assert result is not None
    print(f"FCForceMode([1,1,1,0,0,0], [0,0,0,0,0,0]): {result}")  # 力控模式设置 / Force control mode setting
    
    # 测试力关闭 / Test Force off
    result = robot.plugins.FCOff()
    assert result is not None
    print(f"FCOff(): {result}")  # 力关闭 / Force off
    
    # 测试传送带初始化 / Test conveyor initialization
    result = robot.plugins.CnvInit(1)
    assert result is not None
    print(f"CnvInit(1): {result}")  # 传送带初始化 / Conveyor initialization
    
    del robot
    print("Plugin command generation test passed!")  # 插件指令生成测试通过 / Plugin command generation test passed


if __name__ == "__main__":
    print("Running plugin module tests...\n")  # 运行插件模块测试 / Running plugin module tests
    
    test_plugins_methods()
    print()
    
    test_plugins_command_generation()
    print()
    
    print("=" * 50)
    print("All plugin module tests passed!")  # 插件模块所有测试通过 / All plugin module tests passed
