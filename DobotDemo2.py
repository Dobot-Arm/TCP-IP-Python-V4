from dobot_api import DobotApiDashMove,  alarmAlarmJsonFile
import threading
from time import sleep
import re
import sys


class DobotDemo2:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboardmove = None
        self.feedInfo=[]
        self.__globalLockValue = threading.Lock()

    def start(self):
        self.dashboardmove = DobotApiDashMove(self.ip, self.dashboardPort,self.feedPortFour)
        enableState = self.parseResultId(self.dashboardmove.EnableRobot())
        if enableState[0] != 0:
            print("使能失败: 检查29999端口是否被占用)")
            return
        print("使能成功:)")

        feed_thread = threading.Thread(
            target=self.GetFeed)  # 机器状态反馈线程
        feed_thread.daemon = True
        feed_thread.start()

        point_a = [-90, 20, 0, 0, 0, 0]
        point_b = [90, 20, 0, 0, 0, 0]

        # point_a = [7.92, 0, 0, 0, 0, 0]
        # point_b = [-23, 0, 0, 0, 0, 0]

        while True:
            sleep(1)
            while True:
                p2Id = self.RunPoint(point_a)
                if p2Id[0] == 0:  # 运动指令返回值正确
                    self.dashboardmove.WaitArrive(p2Id[1])  # 传入运动指令commandID ,进入等待指令完成
                    break

            while True:
                p2Id = self.RunPoint(point_b)
                if p2Id[0] == 0:
                    self.dashboardmove.WaitArrive(p2Id[1])
                    break

    def GetFeed(self):
        while True:
            with self.__globalLockValue:
               self.feedInfo = self.dashboardmove.parseFeedData()
            sleep(0.01)

    def RunPoint(self, point_list: list):
        recvmovemess = self.dashboardmove.MovJ(
            point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], 1)
        print("Movj", recvmovemess)
        commandArrID = self.parseResultId(recvmovemess)  # 解析Movj指令的返回值
        return commandArrID


    def parseResultId(self, valueRecv):
        if valueRecv.find("Not Tcp") != -1:  # 通过返回值判断机器是否处于tcp模式
            print("Control Mode Is Not Tcp")
            return [1]
        recvData = re.findall(r'-?\d+', valueRecv)
        recvData = [int(num) for num in recvData]
        #  返回tcp指令返回值的所有数字数组
        if len(recvData) == 0:
            return [2]
        return recvData

    def ClearRobotError(self):
        dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
        while True:
          with self.__globalLockValue:
            if self.feedInfo.robotErrorState:
                geterrorID = self.parseResultId(self.dashboardmove.GetErrorID())
                if geterrorID[0] == 0:
                    for i in range(1, len(geterrorID)):
                        alarmState = False
                        for item in dataController:
                            if geterrorID[i] == item["id"]:
                                print("机器告警 Controller GetErrorID",
                                      i, item["zh_CN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue

                        for item in dataServo:
                            if geterrorID[i] == item["id"]:
                                print("机器告警 Servo GetErrorID", i,
                                      item["zh_CN"]["description"])
                                break

                    choose = input("输入1, 将清除错误, 机器继续运行: ")
                    if int(choose) == 1:
                        clearError = self.parseResultId(
                            self.dashboardmove.ClearError())
                        if clearError[0] == 0:
                            print("--机器清错成功--")
                            break
            else:
                if self.feedInfo.robotMode==11:
                    print("机器发生碰撞")
                    choose = input("输入1, 将清除碰撞, 机器继续运行: ")
                    self.dashboardmove.ClearError()
          sleep(5)

    def __del__(self):
        del self.dashboardmove
