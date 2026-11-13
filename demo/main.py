"""
Main entry file
# 主入口文件
"""

from DobotDemo import DobotDemo

if __name__ == '__main__':
    # Set this value to the robot's actual IP address. / 修改为实际机器人IP
    dobot = DobotDemo("192.168.5.1")
    dobot.start()
