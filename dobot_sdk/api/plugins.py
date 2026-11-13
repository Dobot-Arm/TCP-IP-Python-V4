# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""插件相关模块 - 力控、传送带等扩展功能 / Plugin-related module - extended features such as force control and conveyor"""

from typing import Sequence
from ..core.connection import DobotConnection
from .motion import CoordinateType


class Plugins:
    """插件模块 - 处理力控、传送带等扩展功能 / Plugin module - handles extended features such as force control and conveyor"""

    def __init__(self, connection: DobotConnection):
        self.connection = connection

    def _send_cmd(self, command: str) -> str:
        """发送命令并接收响应 / Send a command and receive the response"""
        return self.connection.send_receive_text(command)

    # ==================== 力传感器基础指令 / Force sensor basic commands ====================

    def EnableFTSensor(self, status: int) -> str:
        """
        EnableFTSensor 开启或关闭力传感器（立即指令）
        EnableFTSensor turns the force sensor on or off (immediate command)

        Args:
            status: 0-关闭, 1-开启 / 0-off, 1-on        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"EnableFTSensor({status})")

    def SixForceHome(self) -> str:
        """SixForceHome力传感器回零（立即指令） / SixForceHome homes the force sensor (immediate command)"""
        return self._send_cmd("SixForceHome()")

    def GetForce(self, tool: int = -1) -> str:
        """
        GetForce获取力传感器数值（立即指令）
        GetForce gets the force sensor reading (immediate command)

        Args:
            tool: 工具坐标系编号，取值范围[0, 50]，-1表示使用当前工具坐标系
                  Tool coordinate system index, range [0, 50], -1 means use the current tool coordinate system
        """
        if tool == -1:
            return self._send_cmd("GetForce()")
        if not (0 <= tool <= 50):
            raise ValueError("tool parameter must be in [0, 50]")  # tool参数必须在[0, 50]之间 / tool parameter must be in [0, 50]
        return self._send_cmd(f"GetForce({tool})")

    # ==================== 力控拖拽模式 / Force control drag mode ====================

    def ForceDriveMode(self, direction: Sequence[int], user: int = -1) -> str:
        """
        ForceDriveMode指定可拖拽的方向并进入力控拖拽模式（立即指令）
        ForceDriveMode specifies the draggable directions and enters force control drag mode (immediate command)

        Args:
            direction: 6个方向的拖拽开关 [x,y,z,rx,ry,rz]，0表示该方向不能拖拽，1表示该方向可以拖拽
                       Drag enable switches for 6 directions [x,y,z,rx,ry,rz], 0 disables, 1 enables
            user: 用户坐标系编号，取值范围[0, 50]，不指定时表示不参考用户坐标系
                  User coordinate system index, range [0, 50]; if not specified, the user coordinate system is not referenced
        """
        if len(direction) != 6:
            raise ValueError("direction requires 6 parameters")  # direction需要6个参数 / direction requires 6 parameters
        for d in direction:
            if d not in [0, 1]:
                raise ValueError("direction value must be 0 or 1")  # direction参数值必须是0或1 / direction value must be 0 or 1
        if user != -1 and not (0 <= user <= 50):
            raise ValueError("user parameter must be in [0, 50]")  # user参数必须在[0, 50]之间 / user parameter must be in [0, 50]
        direction_str = "{" + ",".join([f"{v}" for v in direction]) + "}"
        if user == -1:
            return self._send_cmd(f"ForceDriveMode({direction_str})")
        return self._send_cmd(f"ForceDriveMode({direction_str},{user})")

    def ForceDriveSpeed(self, speed: int) -> str:
        """
        ForceDriveSpeed设置力控拖拽速度（立即指令）
        ForceDriveSpeed sets the force control drag speed (immediate command)

        Args:
            speed: 力控拖拽速度比例，取值范围[1, 100]
                   Force control drag speed ratio, range [1, 100]
        """
        if not 1 <= speed <= 100:
            raise ValueError("speed ratio must be between 1 and 100")  # 速度比例必须在1-100之间 / speed ratio must be between 1 and 100
        return self._send_cmd(f"ForceDriveSpeed({speed})")

    # ==================== 力控模式参数设置 / Force control mode parameter configuration ====================

    def FCForceMode(self, direction: Sequence[int], force: Sequence[float],
                      reference: int = 0, user: int = -1, tool: int = -1) -> str:
        """
        FCForceMode以用户指定的参数开启力控（队列指令）
        FCForceMode enables force control with user-specified parameters (queued command)

        Args:
            direction: 6个方向的力控开关 [x,y,z,rx,ry,rz]，1表示开启，0表示关闭
                       Force control switches for 6 directions [x,y,z,rx,ry,rz], 1 enables, 0 disables
            force: 目标力和力矩 [fx,fy,fz,frx,fry,frz]。平移方向的力为 [-200, 200] N，旋转方向的力矩为 [-12, 12] N·m。
                   Target forces and torques [fx,fy,fz,frx,fry,frz]. Translational forces range from -200 to 200 N, and rotational torques from -12 to 12 N·m.
            reference: 参考坐标系类型 (0-工具坐标系 1-用户坐标系)
                       Reference coordinate system type (0-tool, 1-user)
            user: 用户坐标系编号(0-50) / User coordinate system index (0-50)
            tool: 工具坐标系编号(0-50) / Tool coordinate system index (0-50)
        """
        if len(direction) != 6:
            raise ValueError("direction requires 6 parameters")  # direction需要6个参数 / direction requires 6 parameters
        if len(force) != 6:
            raise ValueError("force requires 6 parameters")  # force需要6个参数 / force requires 6 parameters
        for d in direction:
            if d not in [0, 1]:
                raise ValueError("direction value must be 0 or 1")  # direction参数值必须是0或1 / direction value must be 0 or 1
        for i, f in enumerate(force):
            if i < 3:
                if not (-200 <= f <= 200):
                    raise ValueError("translational target force must be in [-200, 200]")  # 位移方向目标力必须在[-200, 200]之间 / translational target force must be in [-200, 200]
            else:
                if not (-12 <= f <= 12):
                    raise ValueError("target torque must be between -12 and 12 N·m")  # 目标力矩必须在 -12 至 12 N·m 之间
        if user != -1 and not (0 <= user <= 50):
            raise ValueError("user parameter must be in [0, 50]")  # user参数必须在[0, 50]之间 / user parameter must be in [0, 50]
        if tool != -1 and not (0 <= tool <= 50):
            raise ValueError("tool parameter must be in [0, 50]")  # tool参数必须在[0, 50]之间 / tool parameter must be in [0, 50]
        if reference not in [0, 1]:
            raise ValueError("reference parameter must be 0 or 1")  # reference参数必须是0或1 / reference parameter must be 0 or 1
        direction_str = "{" + ",".join([f"{v}" for v in direction]) + "}"
        force_str = "{" + ",".join([f"{v}" for v in force]) + "}"
        params = [direction_str, force_str]
        if reference != 0:
            params.append(f"reference={reference}")
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        return self._send_cmd(f"FCForceMode({','.join(params)})")

    def FCSetDeviation(self, deviation: Sequence[float], control_type: int = -1) -> str:
        """
        FCSetDeviation设置力控模式下的位移和姿态偏差（立即指令）
        FCSetDeviation sets the translational and rotational deviation in force control mode (immediate command)

        Args:
            deviation: 6个方向的偏差 [x,y,z,rx,ry,rz]，位移单位mm，姿态单位度
                       位移偏差取值范围(0, 1000]，默认值100；姿态偏差取值范围(0, 360]，默认值36
                       Deviation for 6 directions [x,y,z,rx,ry,rz], translation in mm, rotation in deg
                       translational deviation range (0, 1000], default 100; rotational deviation range (0, 360], default 36
            control_type: 控制类型，0-超过阈值时报警，1-超过阈值时停止搜寻继续运动，-1表示默认
                          Control type, 0-alarm when threshold exceeded, 1-stop searching and continue motion when exceeded, -1-default
        """
        if len(deviation) != 6:
            raise ValueError("deviation requires 6 parameters")  # 偏差需要6个参数 / deviation requires 6 parameters
        for v in deviation[:3]:
            if not (0 < v <= 1000):
                raise ValueError("translational deviation must be in (0, 1000]")  # 位移偏差必须在(0, 1000]之间 / translational deviation must be in (0, 1000]
        for v in deviation[3:]:
            if not (0 < v <= 360):
                raise ValueError("rotational deviation must be in (0, 360]")  # 姿态偏差必须在(0, 360]之间 / rotational deviation must be in (0, 360]
        if control_type not in [-1, 0, 1]:
            raise ValueError("control_type must be -1, 0 or 1")  # control_type必须是-1、0或1 / control_type must be -1, 0 or 1
        dev_str = "{" + ",".join([f"{v}" for v in deviation]) + "}"
        if control_type == -1:
            return self._send_cmd(f"FCSetDeviation({dev_str})")
        return self._send_cmd(f"FCSetDeviation({dev_str},{control_type})")

    def FCSetForceLimit(self, x: float, y: float, z: float,
                           rx: float, ry: float, rz: float) -> str:
        """
        FCSetForceLimit设置最大力限制（立即指令）
        FCSetForceLimit sets the maximum force limit (immediate command)

        Args:
            x,y,z: 位移方向最大力限制 (N)，取值范围(0, 500]，默认值500
                   Maximum force limit in translational directions (N), range (0, 500], default 500
            rx,ry,rz: 旋转方向的力矩上限（N·m），取值范围为 (0, 50]，默认值为 50。
                      Torque limits for the rotational axes (N·m). Range: (0, 50]. Default: 50.
        """
        for v in [x, y, z]:
            if not (0 < v <= 500):
                raise ValueError("translational max force limit must be in (0, 500]")  # 位移方向最大力限制必须在(0, 500]之间 / translational max force limit must be in (0, 500]
        for v in [rx, ry, rz]:
            if not (0 < v <= 50):
                raise ValueError("torque limit must be greater than 0 and at most 50 N·m")  # 力矩上限必须大于 0 且不超过 50 N·m
        return self._send_cmd(f"FCSetForceLimit({x},{y},{z},{rx},{ry},{rz})")

    def FCSetMass(self, x: float, y: float, z: float,
                    rx: float, ry: float, rz: float) -> str:
        """
        FCSetMass设置力控模式下各方向的惯性系数（立即指令）
        FCSetMass sets the inertia coefficient of each direction in force control mode (immediate command)

        Args:
            x,y,z: 位移方向惯性系数(kg)，取值范围(0, 10000]，默认值20
                   Inertia coefficient in translational directions (kg), range (0, 10000], default 20
            rx,ry,rz: 姿态方向惯性系数(kg·m²)，取值范围(0, 10000]，默认值20
                      Inertia coefficient in rotational directions (kg·m²), range (0, 10000], default 20
        """
        for v in [x, y, z, rx, ry, rz]:
            if not (0 < v <= 10000):
                raise ValueError("inertia coefficient must be in (0, 10000]")  # 惯性系数必须在(0, 10000]之间 / inertia coefficient must be in (0, 10000]
        return self._send_cmd(f"FCSetMass({x},{y},{z},{rx},{ry},{rz})")

    def FCSetStiffness(self, x: float, y: float, z: float,
                         rx: float, ry: float, rz: float) -> str:
        """
        FCSetStiffness设置力控模式下各方向的弹性系数（立即指令）
        FCSetStiffness sets the stiffness coefficient of each direction in force control mode (immediate command)

        Args:
            x,y,z: 位移方向弹性系数(N/mm)，取值范围[0, 10000]，默认值30
                   Stiffness coefficient in translational directions (N/mm), range [0, 10000], default 30
            rx,ry,rz: 姿态方向弹性系数(N/m·deg)，取值范围[0, 10000]，默认值30
                      Stiffness coefficient in rotational directions (N/m·deg), range [0, 10000], default 30
        """
        for v in [x, y, z, rx, ry, rz]:
            if not (0 <= v <= 10000):
                raise ValueError("stiffness coefficient must be in [0, 10000]")  # 弹性系数必须在[0, 10000]之间 / stiffness coefficient must be in [0, 10000]
        return self._send_cmd(f"FCSetStiffness({x},{y},{z},{rx},{ry},{rz})")

    def FCSetDamping(self, x: float, y: float, z: float,
                       rx: float, ry: float, rz: float) -> str:
        """
        FCSetDamping设置力控模式下各方向的阻尼系数（立即指令）
        FCSetDamping sets the damping coefficient of each direction in force control mode (immediate command)

        Args:
            x,y,z: 位移方向阻尼系数 (N·s/mm)，取值范围[0, 1000]，默认值50
                   Damping coefficient in translational directions (N·s/mm), range [0, 1000], default 50
            rx,ry,rz: 姿态方向阻尼系数(N·s/m·deg)，取值范围[0, 1000]，默认值50
                      Damping coefficient in rotational directions (N·s/m·deg), range [0, 1000], default 50
        """
        for v in [x, y, z, rx, ry, rz]:
            if not (0 <= v <= 1000):
                raise ValueError("damping coefficient must be in [0, 1000]")  # 阻尼系数必须在[0, 1000]之间 / damping coefficient must be in [0, 1000]
        return self._send_cmd(f"FCSetDamping({x},{y},{z},{rx},{ry},{rz})")

    def FCOff(self) -> str:
        """FCOff退出力控模式（队列指令） / FCOff exits force control mode (queued command)"""
        return self._send_cmd("FCOff()")

    def FCSetForceSpeedLimit(self, x: float, y: float, z: float,
                                 rx: float, ry: float, rz: float) -> str:
        """
        FCSetForceSpeedLimit设置各方向的力控调节速度（立即指令）
        FCSetForceSpeedLimit sets the force control adjustment speed of each direction (immediate command)

        Args:
            x,y,z: 位移方向力控调节速度 (mm/s)
                   CRA/CRAF机型取值范围(0,安全限制TCP速度值]
                   其他机型取值范围(0, 300]
                   Force control adjustment speed in translational directions (mm/s)
                   CRA/CRAF models: range (0, safety-limited TCP speed]
                   Other models: range (0, 300]
            rx,ry,rz: 姿态方向力控调节速度 (deg/s)
                      CRA/CRAF机型取值范围(0,(4*安全限制TCP速度值*0.001)/(3.14*180)]
                      其他机型取值范围(0, 90]
                      Force control adjustment speed in rotational directions (deg/s)
                      CRA/CRAF models: range (0, (4*safety-limited TCP speed*0.001)/(3.14*180)]
                      Other models: range (0, 90]
        """
        for v in [x, y, z]:
            if not (0 < v <= 300):
                raise ValueError("translational force control speed must be in (0, 300]")  # 位移方向力控调节速度必须在(0, 300]之间 / translational force control speed must be in (0, 300]
        for v in [rx, ry, rz]:
            if not (0 < v <= 90):
                raise ValueError("rotational force control speed must be in (0, 90]")  # 姿态方向力控调节速度必须在(0, 90]之间 / rotational force control speed must be in (0, 90]
        return self._send_cmd(f"FCSetForceSpeedLimit({x},{y},{z},{rx},{ry},{rz})")

    def FCSetForce(self, x: float, y: float, z: float,
                     rx: float, ry: float, rz: float) -> str:
        """
        FCSetForce实时调整各方向的恒力设置（立即指令）
        FCSetForce adjusts the constant force settings of each direction in real time (immediate command)

        Args:
            x,y,z: 位移方向恒力值(N)，取值范围[-200, 200]
                   Constant force value in translational directions (N), range [-200, 200]
            rx,ry,rz: 旋转方向的恒力矩（N·m），取值范围为 [-12, 12]。
                      Constant torques for the rotational axes (N·m). Range: [-12, 12].
        """
        for v in [x, y, z]:
            if not (-200 <= v <= 200):
                raise ValueError("translational constant force must be in [-200, 200]")  # 位移方向恒力值必须在[-200, 200]之间 / translational constant force must be in [-200, 200]
        for v in [rx, ry, rz]:
            if not (-12 <= v <= 12):
                raise ValueError("constant torque must be between -12 and 12 N·m")  # 恒力矩必须在 -12 至 12 N·m 之间
        return self._send_cmd(f"FCSetForce({x},{y},{z},{rx},{ry},{rz})")

    # ==================== 力传感器碰撞检测（仅适用CRAF机型）/ Force sensor collision detection (CRAF models only) ====================

    def SetFCCollision(self, force: float, torque: float) -> str:
        """
        SetFCCollision设置力传感器碰撞检测的阈值参数（仅适用CRAF机型，立即指令）
        SetFCCollision sets the threshold parameters for force sensor collision detection (CRAF models only, immediate command)

        Args:
            force: 触发力传感器碰撞检测的力阈值，单位N
                   CR5AF取值范围：[5, 150]
                   CR10AF取值范围：[5, 300]
                   CR20AF取值范围：[5, 500]
                   Force threshold that triggers force sensor collision detection, in N
                   CR5AF range: [5, 150]
                   CR10AF range: [5, 300]
                   CR20AF range: [5, 500]
            torque: 触发力传感器碰撞检测的力矩阈值，单位为 N·m
                    CR5AF取值范围：[0.5, 15]
                    CR10AF取值范围：[0.5, 30]
                    CR20AF取值范围：[0.5, 50]
                    Torque threshold that triggers force sensor collision detection, in N·m
                    CR5AF range: [0.5, 15]
                    CR10AF range: [0.5, 30]
                    CR20AF range: [0.5, 50]
        """
        if not (5 <= force <= 500):
            raise ValueError("force threshold must be in [5, 500], exact range depends on model")  # 力阈值必须在[5, 500]之间，具体范围取决于机型 / force threshold must be in [5, 500], exact range depends on model
        if not (0.5 <= torque <= 50):
            raise ValueError("torque threshold must be in [0.5, 50], exact range depends on model")  # 力矩阈值必须在[0.5, 50]之间，具体范围取决于机型 / torque threshold must be in [0.5, 50], exact range depends on model
        return self._send_cmd(f"SetFCCollision({force},{torque})")

    def FCCollisionSwitch(self, status: int) -> str:
        """
        FCCollisionSwitch 开启或关闭力传感器碰撞检测（仅适用CRAF机型，立即指令）
        FCCollisionSwitch turns force sensor collision detection on or off (CRAF models only, immediate command)

        Args:
            status: 0-关闭碰撞检测 1-开启碰撞检测 / 0-disable collision detection 1-enable collision detection        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"FCCollisionSwitch({status})")

    # ==================== 传送带跟踪 / Conveyor tracking ====================

    def CnvInit(self, index: int) -> str:
        """
        CnvInit开启传送带（立即指令）
        CnvInit enables the conveyor (immediate command)

        Args:
            index: 传送带索引 (1-3) / Conveyor index (1-3)
        """
        if not 1 <= index <= 3:
            raise ValueError("conveyor index must be between 1 and 3")  # 传送带索引必须在1-3之间 / conveyor index must be between 1 and 3
        return self._send_cmd(f"CnvInit({index})")

    def GetCnvObject(self, obj_id: int) -> str:
        """
        GetCnvObject等待指定工件进入传送带的抓取区域（立即指令）
        GetCnvObject waits for the specified workpiece to enter the conveyor picking area (immediate command)

        Args:
            obj_id: 工件类型，取值范围[0, 15]
                    0：不指定工件类型，获取最先进入队列的工件信息
                    Workpiece type, range [0, 15]
                    0: Leave the workpiece type unspecified and retrieve information about the first workpiece to enter the queue.
        """
        if not 0 <= obj_id <= 15:
            raise ValueError("workpiece type must be between 0 and 15")  # 工件类型必须在0-15之间 / workpiece type must be between 0 and 15
        return self._send_cmd(f"GetCnvObject({obj_id})")

    def StartSyncCnv(self) -> str:
        """StartSyncCnv开启传送带跟踪功能（立即指令） / StartSyncCnv enables the conveyor tracking function (immediate command)"""
        return self._send_cmd("StartSyncCnv()")

    def CnvMovL(self, pose: Sequence[float],
                 coord_type: CoordinateType = CoordinateType.CARTESIAN,
                 user: int = -1, tool: int = -1,
                 a: int = -1, v: int = -1, speed: int = -1,
                 cp: int = -1, r: int = -1) -> str:
        """
        CnvMovL执行传送带跟随，采取直线轨迹插补（队列指令）
        CnvMovL performs conveyor following with linear trajectory interpolation (queued command)

        Args:
            pose: 6个坐标值 [x,y,z,rx,ry,rz] 或 [j1,j2,j3,j4,j5,j6]
                  6 coordinate values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
            coord_type: 点位坐标系类型 (CoordinateType.JOINT 或 CoordinateType.CARTESIAN)
                        Point coordinate system type (CoordinateType.JOINT or CoordinateType.CARTESIAN)
            user: 用户坐标系编号(0-50) / User coordinate system index (0-50)
            tool: 工具坐标系编号(0-50) / Tool coordinate system index (0-50)
            a: 加速度比例 (1-100) / Acceleration ratio (1-100)
            v: 速度比例 (1-100), 与speed互斥 / Speed ratio (1-100), mutually exclusive with speed
            speed: 目标速度 (mm/s), 与v互斥 / Target speed (mm/s), mutually exclusive with v
            cp: 平滑过渡比例 (0-100), 与r互斥 / Blending ratio (0-100), mutually exclusive with r
            r: 平滑过渡半径 (mm), 与cp互斥 / Blending radius (mm), mutually exclusive with cp
        """
        if len(pose) != 6:
            raise ValueError("pose requires 6 parameters")  # pose需要6个参数 / pose requires 6 parameters

        if coord_type == CoordinateType.JOINT:
            prefix = "joint"
        else:
            prefix = "pose"

        pose_str = prefix + "={" + ",".join([f"{v:.6f}" for v in pose]) + "}"
        params = [pose_str]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if speed != -1:
            params.append(f"speed={speed}")
        if cp != -1:
            params.append(f"cp={cp}")
        if r != -1:
            params.append(f"r={r}")
        return self._send_cmd(f"CnvMovL({','.join(params)})")

    def CnvMovC(self, via_point: Sequence[float], target_point: Sequence[float],
                 user: int = -1, tool: int = -1,
                 a: int = -1, v: int = -1, speed: int = -1,
                 cp: int = -1, r: int = -1, mode: int = 0,
                 coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
        """
        CnvMovC执行传送带跟随圆弧轨迹插补（队列指令）
        CnvMovC performs conveyor following with circular arc trajectory interpolation (queued command)

        文档原型：CnvMovC(P1,P2,user, tool, a, v, cp|r, mode)
        Document prototype: CnvMovC(P1,P2,user, tool, a, v, cp|r, mode)
        P1=中间点，P2=目标点
        P1=via point, P2=target point

        Args:
            via_point: 圆弧中间点 P1 [x,y,z,rx,ry,rz] 或 [j1..j6]
                       Arc via point P1 [x,y,z,rx,ry,rz] or [j1..j6]
            target_point: 圆弧终点 P2 [x,y,z,rx,ry,rz] 或 [j1..j6]
                          Arc target point P2 [x,y,z,rx,ry,rz] or [j1..j6]
            user: 用户坐标系编号(0-50)。-1表示当前
                  User coordinate system index (0-50). -1 means current
            tool: 工具坐标系编号(0-50)。-1表示当前
                  Tool coordinate system index (0-50). -1 means current
            a: 加速度比例 (1-100)。-1表示使用默认
               Acceleration ratio (1-100). -1 means use default
            v: 速度比例 (1-100), 与speed互斥。-1表示使用默认
               Speed ratio (1-100), mutually exclusive with speed. -1 means use default
            speed: 目标速度 (mm/s), 与v互斥（优先speed）
                   Target speed (mm/s), mutually exclusive with v (speed takes priority)
            cp: 平滑过渡比例 (0-100), 与r互斥。-1表示不设置
                Blending ratio (0-100), mutually exclusive with r. -1 means not set
            r: 平滑过渡半径 (mm), 与cp互斥。-1表示不设置
               Blending radius (mm), mutually exclusive with cp. -1 means not set
            mode: 插补模式。默认0 / Interpolation mode. Default 0
            coord_type: 点位坐标系类型（笛卡尔或关节）
                        Point coordinate system type (Cartesian or joint)
        """
        if len(via_point) != 6:
            raise ValueError("via_point (via point P1) requires 6 parameters")  # via_point(中间点P1)需要6个参数 / via_point (via point P1) requires 6 parameters
        if len(target_point) != 6:
            raise ValueError("target_point (target point P2) requires 6 parameters")  # target_point(目标点P2)需要6个参数 / target_point (target point P2) requires 6 parameters

        if coord_type == CoordinateType.JOINT:
            prefix = "joint"
        else:
            prefix = "pose"

        via_str = prefix + "={" + ",".join([f"{val:.6f}" for val in via_point]) + "}"
        target_str = prefix + "={" + ",".join([f"{val:.6f}" for val in target_point]) + "}"

        params = [via_str, target_str]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if speed != -1:
            params.append(f"speed={speed}")
        elif v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")
        if r != -1:
            params.append(f"r={r}")
        if mode != 0:
            params.append(f"mode={mode}")
        return self._send_cmd(f"CnvMovC({','.join(params)})")

    def StopSyncCnv(self) -> str:
        """StopSyncCnv停止传送带跟踪功能（立即指令） / StopSyncCnv stops the conveyor tracking function (immediate command)"""
        return self._send_cmd("StopSyncCnv()")

    def SetCnvPointOffset(self, x_offset: float, y_offset: float) -> str:
        """
        SetCnvPointOffset设置传送带用户坐标系下X、Y方向的偏移量（立即指令）
        SetCnvPointOffset sets the X and Y offsets in the conveyor user coordinate system (immediate command)

        Args:
            x_offset: X方向偏移量(mm) / X direction offset (mm)
            y_offset: Y方向偏移量(mm) / Y direction offset (mm)
        """
        return self._send_cmd(f"SetCnvPointOffset({x_offset},{y_offset})")

    def SetCnvTimeCompensation(self, compensation: float) -> str:
        """
        SetCnvTimeCompensation设置补偿时间（立即指令）
        SetCnvTimeCompensation sets the compensation time (immediate command)

        Args:
            compensation: 补偿时间 (ms) / Compensation time (ms)
        """
        return self._send_cmd(f"SetCnvTimeCompensation({compensation})")


