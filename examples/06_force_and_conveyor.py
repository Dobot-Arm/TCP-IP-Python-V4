"""
Example 6: Force Control and Conveyor
示例6: 力控和传送带

Demonstrates force sensor control and conveyor tracking features
演示力传感器控制和传送带跟踪功能
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
    ROBOT_IP = "120.79.211.106"

    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("6. Force Control and Conveyor Example")  # 6. 力控和传送带示例
            print("=" * 50)

            # 初始化 / Initialize
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)
            robot.robot_control.SpeedFactor(30)

            # ========== 力传感器控制 / Force Sensor Control ==========
            print("\n--- Force Sensor Control ---")  # --- 力传感器控制 ---

            # 请求启用力传感器 / Request to enable the force sensor
            print("Enabling force sensor...")  # 开启力传感器...
            response = robot.plugins.EnableFTSensor(1)
            print(f"Response: {response}")  # 响应: {response}
            time.sleep(1)

            # 力传感器归零 / Reset the force sensor to zero
            print("Zeroing force sensor...")  # 力传感器归零...
            response = robot.plugins.SixForceHome()
            print(f"Response: {response}")  # 响应: {response}
            time.sleep(1)

            # 获取力传感器数据 / Get force sensor data
            print("Reading force sensor data...")  # 读取力传感器数据...
            for i in range(3):
                force_data = robot.plugins.GetForce()
                print(f"Force data {i+1}: {force_data}")  # 力数据 {i+1}: {force_data}
                time.sleep(0.5)

            # 进入力控拖拽模式 / Enter force-control drag mode
            print("\nEntering force-control drag mode...")  # 进入力控拖拽模式...
            response = robot.plugins.ForceDriveMode(1)
            print(f"Response: {response}")  # 响应: {response}
            print("Wait 5 seconds, you can manually drag the robot...")  # 等待5秒，可手动拖拽机器人...
            time.sleep(5)

            # 退出力控拖拽模式 / Exit force-control drag mode
            print("Exiting force-control drag mode...")  # 退出力控拖拽模式...
            response = robot.plugins.ForceDriveMode(0)
            print(f"Response: {response}")  # 响应: {response}

            # 关闭力传感器 / Disable the force sensor
            print("Disabling force sensor...")  # 关闭力传感器...
            response = robot.plugins.EnableFTSensor(0)
            print(f"Response: {response}")  # 响应: {response}

            # ========== 传送带控制（演示） / Conveyor Control (Demo) ==========
            print("\n--- Conveyor Control ---")  # --- 传送带控制 ---

            # 注意：实际使用传送带需要先配置编码器 / Note: using the conveyor actually requires configuring the encoder first
            print("Starting conveyor...")  # 开启传送带...
            response = robot.plugins.CnvInit(1)
            print(f"Response: {response}")  # 响应: {response}

            # 开启传送带跟踪 / Start conveyor tracking
            print("Starting conveyor tracking...")  # 开启传送带跟踪...
            response = robot.plugins.StartSyncCnv()
            print(f"Response: {response}")  # 响应: {response}

            # 模拟传送带跟踪运动 / Simulate conveyor tracking motion
            print("Waiting for workpiece to enter the pick area...")  # 等待工件进入抓取区域.../Waiting for workpiece to enter the pick area...
            # response = robot.plugins.GetCnvObject(0)
            # print(f"Workpiece detection response: {response}")  # 工件检测响应: {response}

            print("Stopping conveyor tracking...")  # 停止传送带跟踪.../Stopping conveyor tracking...
            response = robot.plugins.StopSyncCnv()
            print(f"Response: {response}")  # 响应: {response}

            print("Stopping conveyor...")  # 关闭传送带...
            response = robot.plugins.CnvInit(0)
            print(f"Response: {response}")  # 响应: {response}

            # 返回安全位置 / Return to the safe position
            print("\nReturning to safe position...")  # 返回安全位置...
            safe_pose = [0, -30, -60, 0, 90, 0]
            robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
            time.sleep(3)

            # 下使能 / Disable the robot
            robot.robot_control.DisableRobot()

            print("\n" + "=" * 50)
            print("✅ Force Control and Conveyor Example Completed")  # ✅ 力控和传送带示例完成
            print("=" * 50)

    except Exception as e:
        print(f"\n❌ Error: {e}")  # ❌ 错误: {e}
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
