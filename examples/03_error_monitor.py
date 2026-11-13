"""
Example 3: Error Code Monitoring and Status Query
示例3: 错误码监控和状态查询

Demonstrates how to obtain robot status, error codes, and other information
演示如何获取机器人状态、错误码等信息
"""

from dobot_sdk import DobotRobot
import time


def parse_error_response(response):
    """Parse error code response
    解析错误码响应
    """
    # 响应格式: "ErrorID,{[id,...,id]},GetErrorID();" / Response format: "ErrorID,{[id,...,id]},GetErrorID();"
    if "ErrorID" in response:
        start = response.find("{") + 1
        end = response.find("}")
        if start > 0 and end > start:
            error_codes_str = response[start:end]
            # 去除内部的方括号 / Remove inner square brackets
            error_codes_str = error_codes_str.strip("[]")
            if error_codes_str.strip():
                return [int(x.strip()) for x in error_codes_str.split(",") if x.strip()]
    return []


def main():
    ROBOT_IP = "192.168.1.100"
    
    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("3. Error Code Monitoring and Status Query")  # 3. 错误码监控和状态查询 / 3. Error Code Monitoring and Status Query
            print("=" * 50)
            
            # ========== 获取错误码 ========== / ========== Get Error Codes ==========
            print("\n--- Get Error Codes ---")  # --- 获取错误码 --- / --- Get Error Codes ---
            response = robot.robot_control.GetErrorID()
            print(f"Raw response: {response}")  # 原始响应 / Raw response
            
            error_codes = parse_error_response(response)
            if error_codes:
                print(f"\nFound {len(error_codes)} error code(s):")  # 发现 N 个错误码 / Found N error code(s)
                for code in error_codes:
                    print(f"  - Error code: {code}")  # 错误码 / Error code
                    print(f"    Please refer to the official documentation for details")  # 请查阅官方文档获取详细说明 / Please refer to the official documentation for details
            else:
                print("✅ No error codes")  # 无错误码 / No error codes
            
            # ========== 获取机器人模式 ========== / ========== Get Robot Mode ==========
            print("\n--- Get Robot Mode ---")  # --- 获取机器人模式 --- / --- Get Robot Mode ---
            mode = robot.robot_control.RobotMode()
            print(f"Robot mode: {mode}")  # 机器人模式 / Robot mode
            
            # ========== 获取当前位姿 ========== / ========== Get Current Pose ==========
            print("\n--- Get Current Cartesian Coordinates ---")  # --- 获取当前笛卡尔坐标 --- / --- Get Current Cartesian Coordinates ---
            pose = robot.robot_control.GetPose()
            print(f"Current pose: {pose}")  # 当前位姿 / Current pose
            
            # ========== 获取关节角度 ========== / ========== Get Joint Angles ==========
            print("\n--- Get Current Joint Angles ---")  # --- 获取当前关节角度 --- / --- Get Current Joint Angles ---
            angles = robot.robot_control.GetAngle()
            print(f"Joint angles: {angles}")  # 关节角度 / Joint angles
            
            # ========== 清除报警演示 ========== / ========== Clear Alarm Demo ==========
            print("\n--- Clear Alarm Demo ---")  # --- 清除报警演示 --- / --- Clear Alarm Demo ---
            # 注意：只有在有错误时才需要清除 / Note: Clear only when there are errors
            if error_codes:
                print("Clearing alarms...")  # 清除报警 / Clearing alarms
                robot.robot_control.ClearError()
                # 再次检查 / Check again
                response = robot.robot_control.GetErrorID()
                error_codes = parse_error_response(response)
                if not error_codes:
                    print("✅ Alarms cleared")  # 报警已清除 / Alarms cleared
            
            # ========== 运动学计算演示 ========== / ========== Kinematics Calculation Demo ==========
            print("\n--- Kinematics Calculation ---")  # --- 运动学计算 --- / --- Kinematics Calculation ---
            # 正解：关节角度 -> 笛卡尔坐标 / Forward kinematics: joint angles -> Cartesian coordinates
            print("Forward kinematics (joint -> Cartesian)...")  # 正解运算 (关节->笛卡尔) / Forward kinematics (joint -> Cartesian)
            joints = [0, 0, 0, 0, 90, 0]
            forward_result = robot.robot_control.PositiveKin(joints)
            print(f"Input joints: {joints}")  # 输入关节角度 / Input joints angles
            print(f"Forward kinematics result: {forward_result}")  # 正解结果 / Forward kinematics result
            
            # 可达性检查 / Reachability check
            print("\n--- Linear Motion Reachability Check (CheckOddMovL) ---")  # --- 直线运动可达性检查 (CheckOddMovL) --- / --- Linear Motion Reachability Check (CheckOddMovL) ---
            # CheckOddMovL仅支持关节变量，需要提供起点和终点关节角度 / CheckOddMovL only supports joint variables, requires start and end joint angles
            joints_start = [0, 0, 90, 0, 0, 0]
            joints_end = [90, 30, 0, 0, 0, 0]
            result = robot.robot_control.CheckOddMovL(joints_start, joints_end)
            print(f"Check linear motion: {joints_start} -> {joints_end}")  # 检查直线运动 / Check linear motion
            print(f"Result: {result}")  # 结果 / Result
            
            print("\n--- Circular Motion Reachability Check (CheckOddMovC) ---")  # --- 圆弧运动可达性检查 (CheckOddMovC) --- / --- Circular Motion Reachability Check (CheckOddMovC) ---
            # CheckOddMovC仅支持关节变量，需要提供起点、中间点和终点关节角度 / CheckOddMovC only supports joint variables, requires start, via, and end joint angles
            joints_via = [60, 30, 0, 0, 0, 0]
            result = robot.robot_control.CheckOddMovC(joints_start, joints_via, joints_end)
            print(f"Check circular motion: {joints_start} -> {joints_via} -> {joints_end}")  # 检查圆弧运动 / Check circular motion
            print(f"Result: {result}")  # 结果 / Result
            
            print("\n" + "=" * 50)
            print("✅ Error Monitoring Example Complete")  # 错误监控示例完成 / Error Monitoring Example Complete
            print("=" * 50)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
