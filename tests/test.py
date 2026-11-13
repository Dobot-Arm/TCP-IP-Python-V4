from dobot_sdk import DobotRobot,CoordinateType
# from dobot_sdk.api.motion import CoordinateType
import time

with DobotRobot("192.168.5.1") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.EnableRobot()

    # p1=[-200, -70, 400, -180, 0, -180]
    # p2=[-200, -100, 400, -180, 0, -180]
    p1=[-200, -70, 400, -180, 0, -180]
    p2=[-8.7346, -27.2151, 83.8642, 33.3090, -89.7513, -98.6253]   

    # 关节运动，分别使用笛卡尔目标和关节目标 / Joint motion using Cartesian and joint targets
    robot.motion.MovJ(
        pose=p1,
        coord_type=CoordinateType.CARTESIAN
    )
    robot.motion.MovJ(
        pose=p2,
        coord_type=CoordinateType.JOINT
    )    

    # 直线运动 / Linear motion
    robot.motion.MovL(
        pose=p1,
        coord_type=CoordinateType.CARTESIAN
    )    
    robot.motion.MovL(
        pose=p2,
        coord_type=CoordinateType.JOINT
    )

        # 数字输出 / Digital output
    robot.io.DO(4, 1)    # 打开DO4 / Turn DO4 on
    time.sleep(10)
    robot.io.DO(4, 0)   # 关闭DO4 / Turn DO4 off

    # 读取输入 / Read input
    di_status = robot.io.DI(6)  # 读取DI6 / Read DI6
    print(di_status)
