"""
Communication module tests (Modbus)
通信模块测试（Modbus）
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_communication_methods():
    """
    Test communication module methods
    测试通信模块方法
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试Modbus方法 / Test Modbus methods
    assert hasattr(robot.communication, 'ModbusCreate')
    assert hasattr(robot.communication, 'ModbusRTUCreate')
    assert hasattr(robot.communication, 'ModbusClose')
    print("Modbus create/close methods exist")  # Modbus创建/关闭方法存在 / Modbus create/close methods exist
    
    # 检查离散输入和输入寄存器读取方法 / Check methods for reading discrete inputs and input registers
    assert hasattr(robot.communication, 'GetInBits')
    assert hasattr(robot.communication, 'GetInRegs')
    print("Discrete input and input register read methods exist")  # 离散输入和输入寄存器读取方法存在 / Discrete input and input register read methods exist
    
    # 测试线圈方法 / Test coil methods
    assert hasattr(robot.communication, 'GetCoils')
    assert hasattr(robot.communication, 'SetCoils')
    print("Coil methods exist")  # 线圈方法存在 / Coil methods exist
    
    # 测试保持寄存器方法 / Test holding register methods
    assert hasattr(robot.communication, 'GetHoldRegs')
    assert hasattr(robot.communication, 'SetHoldRegs')
    print("Holding register methods exist")  # 保持寄存器方法存在 / Holding register methods exist
    
    del robot
    print("Communication module methods test passed!")  # 通信模块方法测试通过 / Communication module methods test passed


def test_communication_command_generation():
    """
    Test communication command generation
    测试通信指令生成
    """
    robot = DobotRobot("192.168.1.100")
    
    # 测试创建 Modbus 主站 / Test Modbus master creation
    result = robot.communication.ModbusCreate("192.168.1.101", 502, 1)
    assert result is not None
    print(f"ModbusCreate('192.168.1.101', 502, 1): {result}")  # 创建 Modbus 主站 / Create a Modbus master
    
    # 测试读取输入寄存器 / Test reading input registers
    result = robot.communication.GetInRegs(1, 0, 10)
    assert result is not None
    print(f"GetInRegs(1, 0, 10): {result}")  # 读取输入寄存器 / Read input registers
    
    del robot
    print("Communication command generation test passed!")  # 通信指令生成测试通过 / Communication command generation test passed


if __name__ == "__main__":
    print("Running communication module tests...\n")  # 运行通信模块测试 / Running communication module tests
    
    test_communication_methods()
    print()
    
    test_communication_command_generation()
    print()
    
    print("=" * 50)
    print("All communication module tests passed!")  # 通信模块所有测试通过 / All communication module tests passed
