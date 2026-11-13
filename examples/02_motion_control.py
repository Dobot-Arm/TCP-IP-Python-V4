"""
Example 2: Motion Control
示例2: 运动控制

Demonstrate the usage of various motion commands
演示各种运动指令的使用方法
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time

def main():
    ROBOT_IP = "120.79.211.106"
    
    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("2. Motion Control Example")  # 运动控制示例 / Motion Control Example
            print("=" * 50)
            
            # 初始化 / Initialize
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)
            robot.robot_control.SpeedFactor(30)
            
            # 启动状态监控 / Start status monitoring
            robot.StartFeedbackMonitor()
            time.sleep(1)
            
            # ========== 关节运动 (MovJ) / Joint Motion (MovJ) ==========
            print("\n--- Joint Motion (MovJ) ---")  # 关节运动 / Joint Motion
            print("Moving to safe position...")  # 移动到安全位置 / Moving to safe position
            safe_pose = [0, -30, -60, 0, 90, 0]
            robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
            time.sleep(3)
            
            # ========== 关节运动（笛卡尔目标位姿） (MovJ) / Joint Motion with a Cartesian Target Pose (MovJ) ==========
            print("\n--- Joint Motion with a Cartesian Target Pose (MovJ) ---")  # 关节运动（笛卡尔目标位姿） / Joint Motion with a Cartesian Target Pose
            pose_a = [400, 0, 300, 180, 0, 0]
            print(f"Moving to point A: {pose_a}")  # 移动到点位A / Moving to point A
            robot.motion.MovJ(pose_a, CoordinateType.CARTESIAN)
            time.sleep(3)
            
            # ========== 直线运动 (MovL) / Linear Motion (MovL) ==========
            print("\n--- Linear Motion (MovL) ---")  # 直线运动 / Linear Motion
            pose_b = [400, 100, 300, 180, 0, 0]
            print(f"Moving linearly to point B: {pose_b}")  # 直线移动到点位B / Moving linearly to point B
            robot.motion.MovL(pose_b, CoordinateType.CARTESIAN)
            time.sleep(3)
            
            # ========== 圆弧运动 (Arc) / Arc Motion (Arc) ==========
            print("\n--- Arc Motion (Arc) ---")  # 圆弧运动 / Arc Motion
            pose_via = [450, 150, 300, 180, 0, 0]
            pose_c = [500, 100, 300, 180, 0, 0]
            print(f"Arc motion: B -> Via -> C")  # 圆弧运动 / Arc motion
            robot.motion.Arc(pose_via, pose_c, CoordinateType.CARTESIAN)
            time.sleep(3)
            
            # ========== 整圆运动 (Circle) / Circle Motion (Circle) ==========
            print("\n--- Circle Motion (Circle) ---")  # 整圆运动 / Circle Motion
            print(f"Circle motion: C -> Via -> B")  # 整圆运动 / Circle motion
            robot.motion.Circle(pose_via, pose_b, count=1, coord_type=CoordinateType.CARTESIAN)
            time.sleep(4)
            
            # ========== 相对运动 / Relative Motion ==========
            print("\n--- Relative Motion (RelMovLTool) ---")  # 相对运动 / Relative Motion
            offset = [0, -50, 0, 0, 0, 0]
            print(f"Relative offset: {offset}")  # 相对偏移 / Relative offset
            robot.motion.RelMovLTool(offset[0], offset[1], offset[2], offset[3], offset[4], offset[5])
            time.sleep(2)
            
            # ========== 获取当前状态 / Get Current Status ==========
            print("\n--- Getting Current Status ---")  # 获取当前状态 / Getting Current Status
            status = robot.GetStatus()
            if status:
                print(f"Current speed scaling: {status.speed_scaling:.1f}%")  # 当前速度比例 / Current speed scaling
                print(f"Current pose:")  # 当前位姿 / Current position
                print(f"  X: {status.tool_vector_actual.x:.2f}")
                print(f"  Y: {status.tool_vector_actual.y:.2f}")
                print(f"  Z: {status.tool_vector_actual.z:.2f}")
            
            # 返回安全位置 / Return to safe position
            print("\nReturning to safe position...")  # 返回安全位置 / Returning to safe position
            robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
            time.sleep(3)
            
            # 停止监控 / Stop monitoring
            robot.StopFeedbackMonitor()
            robot.robot_control.DisableRobot()
            
            print("\n" + "=" * 50)
            print("Motion control example completed")  # 运动控制示例完成 / Motion control example completed
            print("=" * 50)
            
    except Exception as e:
        print(f"\nError: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
