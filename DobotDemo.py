from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
from time import sleep
import re
import sys

class DobotDemo:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboard = None
        self.feedInfo = []
        
        class item:
            def __init__(self):
                self.robotMode = 0     #
                self.robotCurrentCommandID = 0
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
        threading.Thread(target=self.GetFeed, daemon=True).start()

        # 定义两个目标点
        point_a = [0, 0, 202, -4, 9, -164]
        point_b = [-10, 0, 202, -4, 9, -164]

        # 走点循环
        while True:
            #self.RunPoint(point_a)
            #self.RunPoint(point_b)
            sleep(10000)

    def GetFeed(self):
        # 获取机器人状态
        while True:
            feedInfo = self.feedFour.feedBackData()
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                self.feedData.robotMode = feedInfo['robot_mode'][0]
                self.feedData.robotCurrentCommandID = feedInfo['currentcommandid'][0]
                # 自主添加所需机械臂反馈的数据
                '''
                self.feedData.robotErrorState = feedInfo['error_status'][0]
                self.feedData.robotEnableStatus = feedInfo['enable_status'][0]
                self.feedData.robotCurrentCommandID = feedInfo['currentcommandid'][0]
                '''

    def RunPoint(self, point_list):
        # 走点指令
        recvmovemess = self.dashboard.MovJ(*point_list, 1)
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
