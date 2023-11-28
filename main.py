import threading
from dobot_api import DobotApiDashboard, DobotApi, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import sys

# 全局变量(当前坐标)
robotErrorState = False
robotEnableStatus = False
globalLockValue = threading.Lock()

# 机器tcp连接函数


def ConnectRobot():
    try:
        ip = "192.168.5.1"
        dashboardPort = 29999
        feedPortFour = 30004
        feedPortFif = 30005
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        feedFour = DobotApi(ip, feedPortFour)
        feedFif = DobotApi(ip, feedPortFif)
        print(">.<连接成功>!<")
        return dashboard, feedFour, feedFif
    except Exception as e:
        print(":(连接失败:(")
        raise e

# 下发运动指令函数


def RunPoint(dashboard: DobotApiDashboard, point_list: list):
    recvmovemess = dashboard.MovJ(
        point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], 1)
    print("Movj", recvmovemess)
    commandArrID = parseResultId(recvmovemess)  # 解析Movj指令的返回值
    return commandArrID

# 反馈端口机器状态信息


def GetFeed(feed: DobotApi):
    global robotErrorState
    global robotEnableStatus
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            # Refresh Properties
            globalLockValue.acquire()  # 互斥量    robotErrorState robotEnableStatus加锁
            robotErrorState = feedInfo['error_status'][0]
            robotEnableStatus = feedInfo['enable_status'][0]
            globalLockValue.release()
        sleep(0.001)

# tcp指令返回值处理函数，返回值为字符串里包含数字的数组


def parseResultId(valueRecv):
    if valueRecv.find("Not Tcp") != -1:  # 通过返回值判断机器是否处于tcp模式
        print("Control Mode Is Not Tcp")
        return [-2]
    commandArrID = re.findall(r'-?\d+', valueRecv)
    # print("commandArrID", commandArrID)
    commandArrID = [int(num) for num in commandArrID]
    if commandArrID[0] == 0:
        if len(commandArrID) >= 2:
            return commandArrID  # 获取运动指令的commandArrID
        return [-1]
    else:
        # 根据返回值来判断机器处于什么状态
        if commandArrID[0] == -1:
            print("Queue command exceeds queue depth 64")
        elif commandArrID[0] == -2:
            print("The robot is in an error state")
        elif commandArrID[0] == -3:
            print("The robot is in emergency stop state")
        elif commandArrID[0] == -4:
            print("The robot is in power down state")
        else:
            print("The robot is in Uknown state", commandArrID[0])
    return [-2]

# 运动指令是否完成函数


def WaitArrive(dashboard: DobotApiDashboard, p2Id):
    global robotEnableStatus
    while True:
        globalLockValue.acquire()  # robotEnableStatus加锁
        if robotEnableStatus:
            currentId = parseResultId(
                dashboard.GetCurrentCommandID())  # 获取当前的commandID
            if currentId[0] == 0:
                # print("GetCurrentCommandID:", currentId[1])
                robotMode = parseResultId(dashboard.RobotMode())
                if robotMode[0] == 0:
                    if currentId[1] > p2Id:
                        globalLockValue.release()
                        break
                    else:
                        isFinsh = (robotMode[1] == 5)
                        if currentId[1] == p2Id and isFinsh:
                            globalLockValue.release()
                            break
        else:
           
            print("The robot is Disabletobot")
        globalLockValue.release()
        sleep(0.1)

# 清除机器错误状态函数


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
    while True:
        globalLockValue.acquire()  # robotErrorState加锁
        if robotErrorState:
            geterrorID = parseResultId(dashboard.GetErrorID())
            if geterrorID[0] != -2:
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
                    clearError = parseResultId(dashboard.ClearError())
                    if clearError[0] == 0:
                        globalLockValue.release()
                        break
        else:
            robotMode = parseResultId(dashboard.RobotMode())
            if robotMode[0] == 0 and robotMode[1] == 11:
                print("机器发生碰撞")
                choose = input("输入1, 将清除碰撞, 机器继续运行: ")
                dashboard.ClearError()
        globalLockValue.release()
        sleep(5)


if __name__ == '__main__':
    dashboard, feedFour, feedFif = ConnectRobot()
    print("开始使能...")
    enableState = 0
    enableRecvState = 0
    try:
        enableRecvState = dashboard.EnableRobot()           # enablerobot
        enableState = parseResultId(enableRecvState)
    except Exception as e:
        print(enableRecvState)  # 29999端口是否被占用
        sys.exit(0)
    if enableState[0] == -2:
        print("使能失败:)")
        sys.exit(0)
    print("使能成功:)")
    feed_thread = threading.Thread(
        target=GetFeed, args=(feedFour,))  # 机器状态反馈线程
    feed_thread.daemon = True
    feed_thread.start()
    feed_thread1 = threading.Thread(
        target=ClearRobotError, args=(dashboard,))  # 机器错误状态清错线程
    feed_thread1.daemon = True
    feed_thread1.start()
    point_a = [-90, 20, 0, 0, 0, 0]
    point_b = [90, 20, 0, 0, 0, 0]
    # 机器运动主线程
    while True:
        p2Id = RunPoint(dashboard, point_a)
        if p2Id[0] == 0:  # 运动指令返回值正确
            WaitArrive(dashboard, p2Id[1])  # 传入运动指令commandID ,进入等待指令完成
        else:
            sleep(5)  # 运动指令返回值错误(如非tcp模式等) 休眠5s后继续运行
        p2Id = RunPoint(dashboard, point_b)
        if p2Id[0] == 0:
            WaitArrive(dashboard, p2Id[1])
        else:
            sleep(5)
