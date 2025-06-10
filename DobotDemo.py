from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
from time import sleep
import re

class DobotDemo:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboard = None
        self.feedInfo = []
        self.__globalLockValue = threading.Lock()
        
        class item:
            def __init__(self):
                self.robotMode = -1     #
                self.robotCurrentCommandID = 0
                self.MessageSize = -1
                self.DigitalInputs =-1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1
                # 自定义添加所需反馈数据

        self.feedData = item()  # 定义结构对象

    def start(self):
        # 启动机器人并使能
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)
        if self.parseResultId(self.dashboard.EnableRobot())[0] != 0:
            print("使能失败: 检查29999端口是否被占用")
            return
        print("使能成功")

        # 启动状态反馈线程
        feed_thread = threading.Thread(
            target=self.GetFeed)  # 机器状态反馈线程
        feed_thread.daemon = True
        feed_thread.start()

        # 定义两个目标点
        point_a = [146.3759,-283.4321,332.3956,177.7879,-1.8540,147.5821]
        point_b = [146.3759,-283.4321,432.3956,177.7879,-1.8540,147.5821]

        # 走点循环
        while True:
            print("DI:", self.feedData.DigitalInputs,"2DI:", bin(self.feedData.DigitalInputs),"--16:",hex(self.feedData.DigitalInputs))
            print("DO:", self.feedData.DigitalOutputs,"2DO:" ,bin(self.feedData.DigitalOutputs),"--16:",hex(self.feedData.DigitalOutputs))
            print("robomode",self.feedData.robotMode)
            sleep(2)

    def GetFeed(self):
        # 获取机器人状态
        while True:
            feedInfo = self.feedFour.feedBackData()
            with self.__globalLockValue:
                if feedInfo is not None:   
                    if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':
                        # 基础字段
                        self.feedData.MessageSize = feedInfo['len'][0]
                        self.feedData.robotMode = feedInfo['RobotMode'][0]
                        self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0]
                        self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0]
                        self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]
                        # 自定义添加所需反馈数据
                        '''
                        self.feedData.DigitalOutputs = int(feedInfo['DigitalOutputs'][0])
                        self.feedData.RobotMode = int(feedInfo['RobotMode'][0])
                        self.feedData.TimeStamp = int(feedInfo['TimeStamp'][0])
                        '''

    def RunPoint(self, point_list):
        # 走点指令
        recvmovemess = self.dashboard.MovJ(*point_list, 0)
        print("MovJ:", recvmovemess)
        print(self.parseResultId(recvmovemess))
        currentCommandID = self.parseResultId(recvmovemess)[1]
        print("指令 ID:", currentCommandID)
        #sleep(0.02)
        while True:  #完成判断循环

            print(self.feedData.robotMode)
            if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID == currentCommandID:
                print("运动结束")
                break
            sleep(0.1)

    def parseResultId(self, valueRecv):
        # 解析返回值，确保机器人在 TCP 控制模式
        if "Not Tcp" in valueRecv:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]

    def __del__(self):
        del self.dashboard
        del self.feedFour
