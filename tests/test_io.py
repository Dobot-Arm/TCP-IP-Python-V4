"""
IO module tests
IO模块测试
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_io_methods():
    """
    Test IO module methods
    测试IO模块方法
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试数字输出方法 / Test digital output methods
    assert hasattr(robot.io, 'DO')
    assert hasattr(robot.io, 'DOInstant')
    print("Digital output methods exist")  # 数字输出方法存在 / Digital output methods exist
    
    # 检查 DI 方法 / Check the DI method
    assert hasattr(robot.io, 'DI')
    print("The DI method exists")  # DI 方法存在 / The DI method exists
    
    # 测试数字输出组方法 / Test digital output group methods
    assert hasattr(robot.io, 'DOGroup')
    assert hasattr(robot.io, 'DOGroupDEC')
    assert hasattr(robot.io, 'GetDOGroup')
    print("Digital output group methods exist")  # 数字输出组方法存在 / Digital output group methods exist
    
    # 测试数字输入组方法 / Test digital input group methods
    assert hasattr(robot.io, 'DIGroup')
    assert hasattr(robot.io, 'DIGroupDEC')
    print("Digital input group methods exist")  # 数字输入组方法存在 / Digital input group methods exist
    
    # 检查模拟输入和输出方法 / Check analog input and output methods
    assert hasattr(robot.io, 'AO')
    assert hasattr(robot.io, 'AOInstant')
    assert hasattr(robot.io, 'GetAO')
    assert hasattr(robot.io, 'AI')
    print("Analog input/output methods exist")  # 模拟输入输出方法存在 / Analog input/output methods exist
    
    # 测试工具IO方法 / Test tool IO methods
    assert hasattr(robot.io, 'ToolDO')
    assert hasattr(robot.io, 'ToolDOInstant')
    assert hasattr(robot.io, 'ToolDI')
    assert hasattr(robot.io, 'ToolAI')
    print("Tool IO methods exist")  # 工具IO方法存在 / Tool IO methods exist
    
    # 测试工具设置方法 / Test tool setting methods
    assert hasattr(robot.io, 'SetToolPower')
    assert hasattr(robot.io, 'SetToolMode')
    assert hasattr(robot.io, 'SetTool485')
    print("Tool setting methods exist")  # 工具设置方法存在 / Tool setting methods exist
    
    del robot
    print("IO module test passed!")  # IO模块测试通过 / IO module test passed


def test_io_parameter_validation():
    """
    Test IO parameter validation
    测试IO参数验证
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试 DO 方法 / Test DO method
    result = robot.io.DO(1, 1)
    assert result is not None
    print(f"DO(1, 1): {result}")  # 数字输出置位 / Digital output set
    
    # 测试 DO 方法（关闭） / Test DO method (off)
    result = robot.io.DO(1, 0)
    assert result is not None
    print(f"DO(1, 0): {result}")  # 数字输出复位 / Digital output reset
    
    # 测试 DI 方法 / Test DI method
    result = robot.io.DI(1)
    assert result is not None
    print(f"DI(1): {result}")  # 数字输入读取 / Digital input read
    
    del robot
    print("IO parameter validation test passed!")  # IO参数验证测试通过 / IO parameter validation test passed


if __name__ == "__main__":
    print("Running IO module tests...\n")  # 运行IO模块测试 / Running IO module tests
    
    test_io_methods()
    print()
    
    test_io_parameter_validation()
    print()
    
    print("=" * 50)
    print("All IO module tests passed!")  # IO模块所有测试通过 / All IO module tests passed
