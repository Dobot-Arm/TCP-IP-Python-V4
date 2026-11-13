"""
Example 5: Coordinate System Setup
示例5: 坐标系设置

Demonstrates how to set up and use user coordinate systems and tool coordinate systems
演示用户坐标系和工具坐标系的设置与使用
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
    ROBOT_IP = "192.168.1.100"
    
    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("5. Coordinate System Setup Example")  # 5. 坐标系设置示例 / 5. Coordinate System Setup Example
            print("=" * 50)
            
            # 初始化 / Initialization
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)
            robot.robot_control.SpeedFactor(30)
            
            # ========== 设置用户坐标系 ========== / ========== Set User Coordinate System ==========
            print("\n--- Set User Coordinate System ---")  # --- 设置用户坐标系 --- / --- Set User Coordinate System ---
            
            # 创建用户坐标系1（相对于世界坐标系偏移） / Create user coordinate system 1 (offset relative to world coordinate system)
            user_pose = [100, 50, 0, 0, 0, 0]  # X偏移100, Y偏移50 / X offset 100, Y offset 50
            print(f"Setting user coordinate system 1: {user_pose}")  # 设置用户坐标系1 / Setting user coordinate system 1
            robot.robot_control.SetUser(1, user_pose)
            
            # 切换到用户坐标系1 / Switch to user coordinate system 1
            print("Switching to user coordinate system 1...")  # 切换到用户坐标系1 / Switching to user coordinate system 1
            robot.robot_control.User(1)
            
            # 在用户坐标系下运动 / Move under user coordinate system
            print("Moving to (100, 0, 300) under user coordinate system 1...")  # 在用户坐标系1下移动到 (100, 0, 300) / Moving to (100, 0, 300) under user coordinate system 1
            pose_in_user = [100, 0, 300, 180, 0, 0]
            robot.motion.MovJ(pose_in_user, CoordinateType.CARTESIAN)
            time.sleep(3)
            
            # 切换回世界坐标系 / Switch back to world coordinate system
            print("Switching back to world coordinate system...")  # 切换回世界坐标系 / Switching back to world coordinate system
            robot.robot_control.User(0)
            
            # ========== 设置工具坐标系 ========== / ========== Set Tool Coordinate System ==========
            print("\n--- Set Tool Coordinate System ---")  # --- 设置工具坐标系 --- / --- Set Tool Coordinate System ---
            
            # 创建工具坐标系1（末端工具长度100mm） / Create tool coordinate system 1 (end tool length 100mm)
            tool_pose = [0, 0, 100, 0, 0, 0]  # Z方向偏移100mm / Z direction offset 100mm
            print(f"Setting tool coordinate system 1: {tool_pose}")  # 设置工具坐标系1 / Setting tool coordinate system 1
            robot.robot_control.SetTool(1, tool_pose)
            
            # 切换到工具坐标系1 / Switch to tool coordinate system 1
            print("Switching to tool coordinate system 1...")  # 切换到工具坐标系1 / Switching to tool coordinate system 1
            robot.robot_control.Tool(1)
            
            # 在工具坐标系下运动 / Move under tool coordinate system
            print("Relative movement under tool coordinate system 1...")  # 在工具坐标系1下相对移动 / Relative movement under tool coordinate system 1
            pose_in_tool = [400, 0, 300, 180, 0, 0]
            robot.motion.MovJ(pose_in_tool, CoordinateType.CARTESIAN)
            time.sleep(3)
            
            # 切换回默认工具坐标系 / Switch back to default tool coordinate system
            print("Switching back to default tool coordinate system...")  # 切换回默认工具坐标系 / Switching back to default tool coordinate system
            robot.robot_control.Tool(0)
            
            # ========== 设置负载参数 ========== / ========== Set Payload Parameters ==========
            print("\n--- Set Payload Parameters ---")  # --- 设置负载参数 --- / --- Set Payload Parameters ---
            print("Setting payload: weight 2kg, center of gravity (0, 0, 100)...")  # 设置负载: 重量2kg, 重心(0, 0, 100) / Setting payload: weight 2kg, center of gravity (0, 0, 100)
            robot.robot_control.SetPayload(2.0, 0, 0, 100)
            
            # ========== 计算坐标系（三点法） ========== / ========== Calculate Coordinate System (Three-Point Method) ==========
            print("\n--- Coordinate System Calculation Demo ---")  # --- 计算坐标系演示 --- / --- Coordinate System Calculation Demo ---
            print("Note: Actual use of CalcUser/CalcTool requires teaching points first")  # 注意: 实际使用CalcUser/CalcTool需要先示教点位 / Note: Actual use of CalcUser/CalcTool requires teaching points first
            print("This only demonstrates the API call method")  # 这里仅演示API调用方式 / This only demonstrates the API call method
            
            # 计算用户坐标系示例（需要先示教3个点） / Calculate user coordinate system example (requires teaching 3 points first)
            # robot.robot_control.CalcUser(1)  # 需要示教 / Requires teaching
            
            # 计算工具坐标系示例（需要先示教3个点） / Calculate tool coordinate system example (requires teaching 3 points first)
            # robot.robot_control.CalcTool(1)  # 需要示教 / Requires teaching
            
            # ========== 获取当前坐标系信息 ========== / ========== Get Current Coordinate System Info ==========
            print("\n--- Get Coordinate System Info ---")  # --- 获取坐标系信息 --- / --- Get Coordinate System Info ---
            pose = robot.robot_control.GetPose()
            print(f"Current pose: {pose}")  # 当前位姿 / Current pose
            
            # 返回安全位置 / Return to safe position
            print("\nReturning to safe position...")  # 返回安全位置 / Returning to safe position
            safe_pose = [0, -30, -60, 0, 90, 0]
            robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
            time.sleep(3)
            
            # 下使能 / Disable robot
            robot.robot_control.DisableRobot()
            
            print("\n" + "=" * 50)
            print("✅ Coordinate System Setup Example Complete")  # 坐标系设置示例完成 / Coordinate System Setup Example Complete
            print("=" * 50)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
