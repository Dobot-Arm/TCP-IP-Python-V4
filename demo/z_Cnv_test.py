"""
Conveyor test
# 传送带测试

Demonstrates the use of conveyor tracking functionality
# 演示传送带跟踪功能的使用
"""

import sys
import os

# Add parent directory to path for importing dobot_sdk / 添加父目录到路径，以便导入dobot_sdk
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
    # Set this value to the robot's actual IP address. / 修改为实际机器人IP
    ROBOT_IP = "192.168.5.1"

    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)  # 分隔线 / Separator
            print("Conveyor Test")  # 传送带测试 / Conveyor Test
            print("=" * 50)  # 分隔线 / Separator

            # Initialization / 初始化
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)

            # Query robot mode / 查询机器人模式
            mode_response = robot.robot_control.RobotMode()
            print(f"Robot mode: {mode_response}")  # 机器人模式 / Robot mode

            while True:
                # Initialize conveyor / 初始化传送带
                print("\nInitializing conveyor...")  # 初始化传送带 / Initializing conveyor
                robot.plugins.CnvInit(1)

                # Move to capture/wait position / 移动至拍照/等待位
                print("Moving to wait position...")  # 移动至等待位 / Moving to wait position
                robot.motion.MovJ(
                    [-29.3427, -386.0646, 248.1024, 180.0000, -0.0000, -154.3856],
                    CoordinateType.CARTESIAN,
                    user=0, tool=0
                )
                time.sleep(3)

                # Poll to detect whether an object has reached the conveyor / 轮询检测传送带是否有物体到位
                print("\nWaiting for object to enter pick area...")  # 等待物体进入抓取区域 / Waiting for object to enter pick area
                while True:
                    cnv_status = robot.plugins.GetCnvObject(0)
                    print(f"Conveyor status: {cnv_status}")  # 传送带状态 / Conveyor status

                    # Parse return status / 解析返回状态
                    try:
                        # Example response format: "GetCnvObject,{status},GetCnvObject();" / 响应格式类似: "GetCnvObject,{status},GetCnvObject();"
                        start = cnv_status.find("{") + 1
                        end = cnv_status.find("}")
                        if start > 0 and end > start:
                            status_values = cnv_status[start:end].split(",")
                            if len(status_values) > 3:
                                object_detected = int(status_values[3])
                                if object_detected == 1:
                                    print("Object detected!")  # 检测到物体! / Object detected!
                                    break
                    except Exception as e:
                        print(f"Failed to parse status: {e}")  # 解析状态失败 / Failed to parse status

                    time.sleep(0.2)

                # Start synchronous following / 启动同步跟随
                print("\nStarting conveyor sync...")  # 启动传送带同步 / Starting conveyor sync
                robot.plugins.StartSyncCnv()

                # Execute conveyor following motion / 执行传送带跟随运动
                print("Executing conveyor following motion...")  # 执行传送带跟随运动 / Executing conveyor following motion
                robot.plugins.CnvMovL([0, 0, 0, 0, 0, 153])
                time.sleep(2)

                # Trigger suction cup or gripper (DO6) / 触发吸盘或夹具 (DO6)
                print("Triggering suction cup...")  # 触发吸盘 / Triggering suction cup
                robot.io.DO(6, 1)

                # Descend to pick / 下降抓取
                print("Descending to pick...")  # 下降抓取 / Descending to pick
                robot.plugins.CnvMovL([0, 0, -50, 0, 0, 153])
                time.sleep(3)

                # Stop synchronization / 停止同步
                print("Stopping conveyor sync...")  # 停止传送带同步 / Stopping conveyor sync
                robot.plugins.StopSyncCnv()

                # Query robot mode / 查询机器人模式
                mode_response = robot.robot_control.RobotMode()
                print(f"Robot mode: {mode_response}")  # 机器人模式 / Robot mode

                # Stop motion / 停止运动
                robot.robot_control.Stop()

                # Move to place position / 移动至放置位置
                print("Moving to place position...")  # 移动至放置位置 / Moving to place position
                robot.motion.MovL(
                    [212.5693, -395.0977, 209.9998, 179.9999, -0.0001, -154.3857],
                    CoordinateType.CARTESIAN,
                    user=0, tool=0
                )
                time.sleep(3)

                # Release suction cup / 释放吸盘
                robot.io.DO(6, 0)

                # Return to standby position / 返回待机位
                print("Returning to standby position...")  # 返回待机位 / Returning to standby position
                robot.motion.MovJ(
                    [-29.3427, -386.0646, 248.1024, 180.0000, -0.0000, -154.3856],
                    CoordinateType.CARTESIAN,
                    user=0, tool=0
                )
                time.sleep(3)

    except KeyboardInterrupt:
        print("\nUser interrupted")  # 用户中断 / User interrupted
    except Exception as e:
        print(f"\nError: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
