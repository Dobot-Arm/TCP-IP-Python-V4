"""
DobotDemo - Robot control class
# DobotDemo - 机器人控制类
"""

import sys
import os

# Add parent directory to path for importing dobot_sdk / 添加父目录到路径，以便导入dobot_sdk
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot
import threading
from time import sleep
import re


class DobotDemo:
    def __init__(self, ip):
        self.ip = ip
        self.robot = None
        self.feedData = None
        self.__globalLockValue = threading.Lock()

        # Initialize feedback data structure / 初始化反馈数据结构
        class item:
            def __init__(self):
                self.robotMode = -1
                self.robotCurrentCommandID = 0
                self.MessageSize = -1
                self.DigitalInputs = -1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1

        self.feedData = item()

    def start(self):
        """Start and enable the robot / 启动机器人并使能"""
        try:
            # Create the robot client and connect to the robot. / 创建机器人客户端并连接机器人。
            self.robot = DobotRobot(self.ip)
            self.robot.Connect()

            # Request TCP control mode / 请求TCP控制模式
            self.robot.robot_control.RequestControl()

            # Clear alarms / 清除报警
            self.robot.robot_control.ClearError()

            # Enable the robot / 使能机器人
            response = self.robot.robot_control.EnableRobot()
            if "Failed" in response:
                print("Enable failed: check if port 29999 is occupied")  # 使能失败: 检查29999端口是否被占用 / Enable failed: check if port 29999 is occupied
                return
            print("Robot enabled successfully")  # 使能成功 / Robot enabled successfully

            # Start status feedback thread / 启动状态反馈线程
            self.robot.StartFeedbackMonitor(callback=self._feed_callback)
            sleep(1)

            # Define two target points / 定义两个目标点
            point_a = [146.3759, -283.4321, 332.3956, 177.7879, -1.8540, 147.5821]
            point_b = [146.3759, -283.4321, 432.3956, 177.7879, -1.8540, 147.5821]

            # Point-move loop / 走点循环
            from dobot_sdk import CoordinateType
            while True:
                status = self.robot.GetStatus()
                if status:
                    self.feedData.DigitalInputs = status.digital_inputs
                    self.feedData.DigitalOutputs = status.digital_outputs
                    self.feedData.robotMode = status.robot_mode.value
                    self.feedData.robotCurrentCommandID = status.current_command_id

                print(f"DI: {self.feedData.DigitalInputs} 2DI: {bin(self.feedData.DigitalInputs)} --16: {hex(self.feedData.DigitalInputs)}")  # 数字输入 / Digital Inputs
                print(f"DO: {self.feedData.DigitalOutputs} 2DO: {bin(self.feedData.DigitalOutputs)} --16: {hex(self.feedData.DigitalOutputs)}")  # 数字输出 / Digital Outputs
                print(f"Robot Mode {self.feedData.robotMode}")  # 机器人模式 / Robot mode
                sleep(2)

        except Exception as e:
            print(f"Start failed: {e}")  # 启动失败 / Start failed
            import traceback
            traceback.print_exc()

    def _feed_callback(self, status):
        """Status feedback callback function / 状态反馈回调函数"""
        with self.__globalLockValue:
            self.feedData.robotMode = status.robot_mode.value
            self.feedData.DigitalInputs = status.digital_inputs
            self.feedData.DigitalOutputs = status.digital_outputs
            self.feedData.robotCurrentCommandID = status.current_command_id

    def RunPoint(self, point_list):
        """Point-move instruction (using new SDK) / 走点指令（使用新SDK）"""
        from dobot_sdk import CoordinateType

        # Execute joint motion / 执行关节运动
        response = self.robot.motion.MovJ(point_list, CoordinateType.CARTESIAN)
        print(f"MovJ: {response}")  # MovJ响应 / MovJ response

        # Parse command ID / 解析指令ID
        currentCommandID = self.parseResultId(response)[1]
        print(f"Command ID: {currentCommandID}")  # 指令ID / Command ID

        # Wait for motion to complete / 等待运动完成
        while True:
            print(f"Current mode: {self.feedData.robotMode}")  # 当前模式 / Current mode
            if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID == currentCommandID:
                print("Motion finished")  # 运动结束 / Motion finished
                break
            sleep(0.1)

    def parseResultId(self, valueRecv):
        """Parse return value / 解析返回值"""
        if "Not Tcp" in valueRecv:
            print("Control Mode Is Not TCP")  # 控制模式不是TCP / Control mode is not TCP
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]

    def __del__(self):
        """Destructor / 析构函数"""
        if self.robot:
            try:
                self.robot.StopFeedbackMonitor()
                self.robot.Disconnect()
            except Exception:
                pass
