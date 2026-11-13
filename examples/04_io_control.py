"""
Example 4: IO Control
示例4: IO控制

Demonstrates how to use digital IO, analog IO, and tool IO
演示数字IO、模拟IO和末端IO的使用方法
"""

from dobot_sdk import DobotRobot
import time


def main():
    ROBOT_IP = "192.168.1.100"
    
    try:
        with DobotRobot(ROBOT_IP) as robot:
            print("=" * 50)
            print("4. IO Control Example")  # 4. IO控制示例 / 4. IO Control Example
            print("=" * 50)
            
            # 初始化 / Initialization
            robot.robot_control.RequestControl()
            robot.robot_control.ClearError()
            robot.robot_control.EnableRobot(load=1.0)
            
            # ========== 数字输出控制 ========== / ========== Digital Output Control ==========
            print("\n--- Digital Output Control (DO) ---")  # --- 数字输出控制 (DO) --- / --- Digital Output Control (DO) ---
            
            # 方式1: 使用 DO(index, status) / Method 1: Use DO(index, status)
            print("Turning ON DO1...")  # 打开DO1 / Turning ON DO1
            robot.io.DO(1, 1)
            time.sleep(1)
            
            print("Turning OFF DO1...")  # 关闭DO1 / Turning OFF DO1
            robot.io.DO(1, 0)
            time.sleep(1)
            
            # 方式2: 使用 Do（队列指令） / Method 2: Use do (queued command)
            print("Setting DO2 to ON...")  # 设置DO2为ON / Setting DO2 to ON
            robot.io.DO(2, 1)
            time.sleep(1)
            
            print("Setting DO2 to OFF...")  # 设置DO2为OFF / Setting DO2 to OFF
            robot.io.DO(2, 0)
            time.sleep(1)
            
            # 方式3: 使用 DOInstant（立即指令） / Method 3: Use DOInstant (immediate command)
            print("Using immediate command to set DO3...")  # 使用立即指令设置DO3 / Using immediate command to set DO3
            robot.io.DOInstant(3, 1)
            time.sleep(1)
            robot.io.DOInstant(3, 0)
            time.sleep(1)
            
            # ========== 数字输入读取 ========== / ========== Digital Input Read ==========
            print("\n--- Digital Input Read (DI) ---")  # --- 数字输入读取 (DI) --- / --- Digital Input Read (DI) ---
            for i in range(1, 5):
                di_status = robot.io.DI(i)
                print(f"DI{i} status: {di_status}")  # DI{i} 状态 / DI{i} status
            
            # ========== 数字输出状态读取 ========== / ========== Digital Output Status Read ==========
            print("\n--- Digital Output Status Read (DO) ---")  # --- 数字输出状态读取 (DO) --- / --- Digital Output Status Read (DO) ---
            for i in range(1, 4):
                do_status = robot.io.GetDO(i)
                print(f"DO{i} status: {do_status}")  # DO{i} 状态 / DO{i} status
            
            # ========== 模拟输出控制 ========== / ========== Analog Output Control ==========
            print("\n--- Analog Output Control (AO) ---")  # --- 模拟输出控制 (AO) --- / --- Analog Output Control (AO) ---
            print("Setting AO1 to 25%...")  # 设置AO1为25% / Setting AO1 to 25%
            robot.io.AO(1, 25)
            time.sleep(2)
            
            print("Setting AO1 to 75%...")  # 设置AO1为75% / Setting AO1 to 75%
            robot.io.AO(1, 75)
            time.sleep(2)
            
            print("Setting AO1 to 0%...")  # 设置AO1为0% / Setting AO1 to 0%
            robot.io.AO(1, 0)
            time.sleep(1)
            
            # ========== 模拟输入读取 ========== / ========== Analog Input Read ==========
            print("\n--- Analog Input Read (AI) ---")  # --- 模拟输入读取 (AI) --- / --- Analog Input Read (AI) ---
            for i in range(1, 3):
                ai_value = robot.io.AI(i)
                print(f"AI{i} value: {ai_value}")  # AI{i} 值 / AI{i} value
            
            # ========== 末端IO控制 ========== / ========== Tool IO Control ==========
            print("\n--- Tool IO Control ---")  # --- 末端IO控制 (Tool IO) --- / --- Tool IO Control ---
            
            # 设置末端DO / Set tool DO
            print("Setting tool DO1 to ON...")  # 设置末端DO1为ON / Setting tool DO1 to ON
            robot.io.ToolDO(1, 1)
            time.sleep(1)
            
            print("Setting tool DO1 to OFF...")  # 设置末端DO1为OFF / Setting tool DO1 to OFF
            robot.io.ToolDO(1, 0)
            time.sleep(1)
            
            # 读取末端DI / Read tool DI
            print("Reading tool DI...")  # 读取末端DI / Reading tool DI
            tool_di_status = robot.io.ToolDI(1)
            print(f"Tool DI status: {tool_di_status}")  # 末端DI 状态 / Tool DI status
            
            # 设置末端工具供电（开启） / Set tool power supply (ON)
            print("\nTurning ON tool power supply...")  # 开启末端工具供电 / Turning ON tool power supply
            robot.io.SetToolPower(1)  # 1 = 开启 / 1 = ON
            
            # ========== 批量IO操作 ========== / ========== Batch IO Operations ==========
            print("\n--- Batch IO Operations ---")  # --- 批量IO操作 --- / --- Batch IO Operations ---
            print("Setting DO1~DO4 all to ON...")  # 设置 DO1~DO4 全部为ON / Setting DO1~DO4 all to ON
            robot.io.DOGroup(1, 1, 2, 1, 3, 1, 4, 1)
            time.sleep(2)
            
            print("Setting DO1~DO4 all to OFF...")  # 设置 DO1~DO4 全部为OFF / Setting DO1~DO4 all to OFF
            robot.io.DOGroup(1, 0, 2, 0, 3, 0, 4, 0)
            time.sleep(1)
            
            # 下使能 / Disable robot
            robot.robot_control.DisableRobot()
            
            print("\n" + "=" * 50)
            print("✅ IO Control Example Complete")  # IO控制示例完成 / IO Control Example Complete
            print("=" * 50)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")  # 错误 / Error
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
