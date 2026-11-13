"""
Example 1: Connecting to and Enabling the Robot
示例1: 基础连接和使能

Demonstrate how to connect to the robot, enable it, set parameters, and shut down safely
演示如何连接机器人、使能、设置参数并安全关闭
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dobot_sdk import DobotRobot
import time


def main():
    # 机器人IP地址（根据实际情况修改） / Robot IP address (modify according to actual situation)
    ROBOT_IP = "192.168.5.1"
    
    # 使用上下文管理器（推荐方式） / Use context manager (recommended way)
    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("1. Connection successful")  # 连接成功 / Connection successful
            print("=" * 50)
            
            # 请求TCP控制模式 / Request TCP control mode
            print("Requesting TCP control mode...")  # 请求TCP控制模式 / Requesting TCP control mode
            response = robot.robot_control.RequestControl()
            print(f"Response: {response}")  # 响应 / Response
            
            # 清除报警（如果有） / Clear alarms (if any)
            print("Clearing alarms...")  # 清除报警 / Clearing alarms
            robot.robot_control.ClearError()
            
            # 使能机器人（设置1kg负载） / Enable the robot (set 1kg payload)
            print("Enabling robot...")  # 使能机器人 / Enabling robot
            response = robot.robot_control.EnableRobot(load=1.0)
            print(f"Response: {response}")  # 响应 / Response
            
            # 设置全局速度比例 / Set global speed factor
            speed = 50
            print(f"Setting the global speed factor to {speed}%...")  # 设置全局速度比例 / Setting the global speed factor
            robot.robot_control.SpeedFactor(speed)
            
            # 获取机器人状态 / Get robot status
            print("\nGetting robot status...")  # 获取机器人状态 / Getting robot status 
            mode = robot.robot_control.RobotMode()
            print(f"Robot mode: {mode}")  # 机器人模式 / Robot mode
            
            # 等待2秒 / Wait 2 seconds
            print(f"\nWaiting {2} seconds...")  # 等待 / Waiting
            time.sleep(2)
            
            # 下使能机器人 / Disable the robot
            print("\nDisabling robot...")  # 下使能机器人 / Disabling robot
            response = robot.robot_control.DisableRobot()
            print(f"Response: {response}")  # 响应 / Response
            
            print("\n" + "=" * 50)
            print("Basic connection example completed")  # 基础连接示例完成 / Basic connection example completed
            print("=" * 50)
            
    except Exception as e:
        print(f"\nError: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
