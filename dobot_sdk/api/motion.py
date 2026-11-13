# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Motion-related module - all motion commands and trajectory control"""
# 运动相关模块 - 所有运动指令、轨迹控制

from typing import Sequence
from enum import IntEnum
from ..core.connection import DobotConnection


class CoordinateType(IntEnum):
    """Coordinate system type enumeration (exactly matches the coordtype values in the documentation)

    Value mapping:
    0 = Joint coordinates (joint)
    1 = User coordinates (user / Cartesian)
    2 = Tool coordinates (tool / Cartesian)

    坐标系类型枚举（与文档coordtype值完全一致）

    值对应关系：
    0 = 关节坐标 (joint)
    1 = 用户坐标 (user / 笛卡尔)
    2 = 工具坐标 (tool / 笛卡尔)
    """
    JOINT = 0     # 关节角度 — 文档coordtype=0 / Joint angles - doc coordtype=0
    CARTESIAN = 1 # 用户坐标系下的笛卡尔位姿 — 文档coordtype=1 / Cartesian pose in user coordinate system - doc coordtype=1
    USER = 1      # 用户坐标别名，等价于CARTESIAN / User coordinate alias, equivalent to CARTESIAN
    TOOL = 2      # 工具坐标系下的笛卡尔位姿 — 文档coordtype=2 / Cartesian pose in tool coordinate system - doc coordtype=2


class Motion:
    """Motion module - handles all motion-related commands"""
    # 运动模块 - 处理所有运动相关指令

    def __init__(self, connection: DobotConnection):
        self.connection = connection

    def _fmt_pose(self, pose: Sequence[float], coord_type: CoordinateType) -> str:
        """Format a pose as joint={...} or pose={...}

        Note: USER(1) and TOOL(2) both use the pose= prefix in point formatting;
        the actual user/tool coordinate system is specified separately via the user= and tool= parameters.

        格式化位姿为 joint={...} 或 pose={...}

        注意：USER(1)和TOOL(2)在点位格式上都使用 pose= 前缀，
        实际的用户/工具坐标系通过 user= 和 tool= 参数单独指定。
        """
        if len(pose) != 6:
            raise ValueError(f"Pose requires 6 parameters, got {len(pose)}")  # 位姿需要6个参数，实际{len(pose)}个

        values = ",".join([f"{v:.6f}" for v in pose])
        if coord_type == CoordinateType.JOINT:
            return f"joint={{{values}}}"
        else:
            return f"pose={{{values}}}"

    def _send_cmd(self, command: str) -> str:
        """Send a command and receive the response"""
        # 发送命令并接收响应
        return self.connection.send_receive_text(command)

    # ==================== Basic motion ====================
    # ==================== 基础运动 ====================

    def MovJ(self, pose: Sequence[float],
             coord_type: CoordinateType,
             user: int = -1, tool: int = -1,
             a: int = -1, v: int = -1, cp: int = -1) -> str:
        """
        MovJ Joint motion (queued command)
        MovJ关节运动（队列指令）

        Args:
            pose: 6 coordinate values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
                  6个坐标值[x,y,z,rx,ry,rz] 或[j1,j2,j3,j4,j5,j6]
            coord_type: Coordinate system type (CoordinateType.CARTESIAN or CoordinateType.JOINT)
                        坐标系类型(CoordinateType.CARTESIAN 或 CoordinateType.JOINT)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            cp: Smooth transition ratio (0-100, -1 means not used)
                平滑过渡比例 (0-100, -1表示不使用)

        Returns:
            str: ErrorID,{ResultID},MovJ(...);
        """
        pose_str = self._fmt_pose(pose, coord_type)

        params = [pose_str]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")

        cmd = f"MovJ({','.join(params)})"
        return self._send_cmd(cmd)

    def MovL(self, pose: Sequence[float],
             coord_type: CoordinateType,
             user: int = -1, tool: int = -1,
             a: int = -1, v: int = -1, speed: int = -1,
             cp: int = -1, r: int = -1) -> str:
        """
        MovL Linear motion (queued command)
        MovL直线运动（队列指令）

        Args:
            pose: 6 Cartesian coordinates [x,y,z,rx,ry,rz]
                  6个笛卡尔坐标 [x,y,z,rx,ry,rz]
            coord_type: Coordinate system type (must be specified)
                        坐标系类型(必须指定)
            user: User coordinate system index (0-50)
                  用户坐标系编号(0-50)
            tool: Tool coordinate system index (0-50)
                  工具坐标系编号(0-50)
            a: Acceleration ratio (1-100)
               加速度比例 (1-100)
            v: Speed ratio (1-100), mutually exclusive with speed
               速度比例 (1-100), 与speed互斥
            speed: Target speed (mm/s), mutually exclusive with v
                   目标速度 (mm/s), 与v互斥
            cp: Smooth transition ratio (0-100), mutually exclusive with r
                平滑过渡比例 (0-100), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp
               平滑过渡半径 (mm), 与cp互斥

        Returns:
            str: ErrorID,{ResultID},MovL(...);
        """
        pose_str = self._fmt_pose(pose, coord_type)

        params = [pose_str]
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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")

        cmd = f"MovL({','.join(params)})"
        return self._send_cmd(cmd)

    def MovLIO(self, pose: Sequence[float], do_list: list,
                coord_type: CoordinateType,
                user: int = -1, tool: int = -1,
                a: int = -1, v: int = -1, speed: int = -1,
                cp: int = -1, r: int = -1) -> str:
        """
        MovLIO Linear motion with DO output (queued command)
        MovLIO直线运动并输出DO（队列指令）

        Args:
            pose: 6 Cartesian coordinates [x,y,z,rx,ry,rz]
                  6个笛卡尔坐标 [x,y,z,rx,ry,rz]
            do_list: List of DO outputs. Each entry contains [Mode, Distance, Index, Status].
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
                     DO输出列表，每个元素为[Mode, Distance, Index, Status]
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
            coord_type: Coordinate system type
                        坐标系类型
            user: User coordinate system index (0-50)
                  用户坐标系编号(0-50)
            tool: Tool coordinate system index (0-50)
                  工具坐标系编号(0-50)
            a: Acceleration ratio (1-100)
               加速度比例 (1-100)
            v: Speed ratio (1-100), mutually exclusive with speed
               速度比例 (1-100), 与speed互斥
            speed: Target speed (mm/s), mutually exclusive with v
                   目标速度 (mm/s), 与v互斥
            cp: Smooth transition ratio (0-100), mutually exclusive with r
                平滑过渡比例 (0-100), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp
               平滑过渡半径 (mm), 与cp互斥

        Returns:
            str: ErrorID,{ResultID},MovLIO(...);
        """
        pose_str = self._fmt_pose(pose, coord_type)

        params = [pose_str]
        for do_item in do_list:
            if len(do_item) != 4:
                raise ValueError("Each DO parameter requires 4 values [Mode, Distance, Index, Status]")  # 每个DO参数需要4个值[Mode, Distance, Index, Status]
            mode, distance, index, status = do_item
            params.append(f"{{{mode},{distance},{index},{status}}}")
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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")

        cmd = f"MovLIO({','.join(params)})"
        return self._send_cmd(cmd)

    def MovJIO(self, pose: Sequence[float], do_list: list,
                coord_type: CoordinateType,
                user: int = -1, tool: int = -1,
                a: int = -1, v: int = -1, cp: int = -1) -> str:
        """
        MovJIO Joint motion with DO output (queued command)
        MovJIO关节运动并输出DO（队列指令）

        Args:
            pose: 6 coordinate values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
                  6个坐标值[x,y,z,rx,ry,rz] 或[j1,j2,j3,j4,j5,j6]
            do_list: List of DO outputs. Each entry contains [Mode, Distance, Index, Status].
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
                     DO输出列表，每个元素为[Mode, Distance, Index, Status]
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
            coord_type: Coordinate system type
                        坐标系类型
            user: User coordinate system index (0-50)
                  用户坐标系编号(0-50)
            tool: Tool coordinate system index (0-50)
                  工具坐标系编号(0-50)
            a: Acceleration ratio (1-100)
               加速度比例 (1-100)
            v: Speed ratio (1-100)
               速度比例 (1-100)
            cp: Smooth transition ratio (0-100)
                平滑过渡比例 (0-100)

        Returns:
            str: ErrorID,{ResultID},MovJIO(...);
        """
        pose_str = self._fmt_pose(pose, coord_type)

        params = [pose_str]
        for do_item in do_list:
            if len(do_item) != 4:
                raise ValueError("Each DO parameter requires 4 values [Mode, Distance, Index, Status]")  # 每个DO参数需要4个值[Mode, Distance, Index, Status]
            mode, distance, index, status = do_item
            params.append(f"{{{mode},{distance},{index},{status}}}")
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")

        cmd = f"MovJIO({','.join(params)})"
        return self._send_cmd(cmd)

    def Arc(self, p1: Sequence[float], p2: Sequence[float],
            coord_type: CoordinateType,
            user: int = -1, tool: int = -1,
            a: int = -1, v: int = -1, speed: int = -1,
            cp: int = -1, r: int = -1, mode: int = 0) -> str:
        """
        Arc Arc interpolation motion (queued command)
        Arc圆弧插补运动（队列指令）

        Args:
            p1: Arc intermediate point
                圆弧中间点
            p2: Target point pose
                目标点位姿
            coord_type: Coordinate system type
                        坐标系类型
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            speed: Target speed (mm/s), mutually exclusive with v
                   目标速度 (mm/s), 与v互斥
            cp: Smooth transition ratio (0-100), mutually exclusive with r
                平滑过渡比例 (0-100), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp
               平滑过渡半径 (mm), 与cp互斥
            mode: Attitude control mode (0 = linear, 1 = pass through the intermediate point, 2 = fixed).
                  姿态控制模式（0 = 线性，1 = 经过中间点，2 = 固定）。

        Returns:
            str: ErrorID,{ResultID},Arc(...);
        """
        if len(p1) != 6 or len(p2) != 6:
            raise ValueError(f"Pose requires 6 parameters")  # 位姿需要6个参数

        if coord_type == CoordinateType.JOINT:
            p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
        else:
            p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"

        params = [p1_str, p2_str]

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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")
        if mode != 0:
            params.append(f"mode={mode}")

        cmd = f"Arc({','.join(params)})"
        return self._send_cmd(cmd)

    def ArcIO(self, p1: Sequence[float], p2: Sequence[float],
               do_list: list,
               coord_type: CoordinateType,
               user: int = -1, tool: int = -1,
               a: int = -1, v: int = -1, speed: int = -1,
               cp: int = -1, r: int = -1, mode: int = 0) -> str:
        """
        ArcIO Arc motion with DO output (queued command)
        ArcIO圆弧运动并输出DO（队列指令）

        Args:
            p1: Arc intermediate point
                圆弧中间点
            p2: Target point pose
                目标点位姿
            do_list: List of DO outputs. Each entry contains [Mode, Distance, Index, Status].
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
                     DO输出列表，每个元素为[Mode, Distance, Index, Status]
                     Mode=0/1, Distance=mm, Index=DO index, Status=0/1
            coord_type: Coordinate system type
                        坐标系类型
            user: User coordinate system index (0-50)
                  用户坐标系编号(0-50)
            tool: Tool coordinate system index (0-50)
                  工具坐标系编号(0-50)
            a: Acceleration ratio (1-100)
               加速度比例 (1-100)
            v: Speed ratio (1-100)
               速度比例 (1-100)
            speed: Target speed (mm/s), mutually exclusive with v
                   目标速度 (mm/s), 与v互斥
            cp: Smooth transition ratio (0-100), mutually exclusive with r
                平滑过渡比例 (0-100), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp
               平滑过渡半径 (mm), 与cp互斥
            mode: Attitude control mode (0-2)
                  姿态控制模式(0-2)

        Returns:
            str: ErrorID,{ResultID},ArcIO(...);
        """
        if len(p1) != 6 or len(p2) != 6:
            raise ValueError(f"Pose requires 6 parameters")  # 位姿需要6个参数

        if coord_type == CoordinateType.JOINT:
            p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
        else:
            p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"

        params = [p1_str, p2_str]
        for do_item in do_list:
            if len(do_item) != 4:
                raise ValueError("Each DO parameter requires 4 values [Mode, Distance, Index, Status]")  # 每个DO参数需要4个值[Mode, Distance, Index, Status]
            mode_do, distance, index, status = do_item
            params.append(f"{{{mode_do},{distance},{index},{status}}}")

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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")
        if mode != 0:
            params.append(f"mode={mode}")

        cmd = f"ArcIO({','.join(params)})"
        return self._send_cmd(cmd)

    def Circle(self, p1: Sequence[float], p2: Sequence[float],
               count: int, coord_type: CoordinateType,
               user: int = -1, tool: int = -1,
               a: int = -1, v: int = -1, speed: int = -1,
               cp: int = -1, r: int = -1, mode: int = 0) -> str:
        """
        Circle Full circle interpolation motion (queued command)
        Circle整圆插补运动（队列指令）

        Args:
            p1: Full circle intermediate point pose
                整圆中间点位姿
            p2: Full circle end point pose (should be the same as the start point)
                整圆结束点位姿（应与起点相同）
            count: Number of turns (1-999)
                   圈数 (1-999)
            coord_type: Coordinate system type
                        坐标系类型
            user: User coordinate system index (0-50)
                  用户坐标系编号(0-50)
            tool: Tool coordinate system index (0-50)
                  工具坐标系编号(0-50)
            a: Acceleration ratio (1-100)
               加速度比例 (1-100)
            v: Speed ratio (1-100)
               速度比例 (1-100)
            speed: Target speed (mm/s), mutually exclusive with v
                   目标速度 (mm/s), 与v互斥
            cp: Smooth transition ratio (0-100), mutually exclusive with r
                平滑过渡比例 (0-100), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp
               平滑过渡半径 (mm), 与cp互斥
            mode: Attitude control mode (0-2)
                  姿态控制模式(0-2)

        Returns:
            str: ErrorID,{ResultID},Circle(...);
        """
        if not 1 <= count <= 999:
            raise ValueError(f"Turn count must be between 1 and 999")  # 圈数必须在1-999之间

        if len(p1) != 6 or len(p2) != 6:
            raise ValueError(f"Pose requires 6 parameters")  # 位姿需要6个参数

        if coord_type == CoordinateType.JOINT:
            p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
        else:
            p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
            p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"

        params = [p1_str, p2_str, str(count)]

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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")
        if mode != 0:
            params.append(f"mode={mode}")

        cmd = f"Circle({','.join(params)})"
        return self._send_cmd(cmd)

    # ==================== Servo motion ====================
    # ==================== 伺服运动 ====================

    def ServoJ(self, joints: Sequence[float],
                t: float = 0.1, aheadtime: float = 50.0, gain: float = 500.0) -> str:
        """
        ServoJ Dynamic follow command based on joint space (queued command)
        ServoJ基于关节空间的动态跟随命令（队列指令）

        Args:
            joints: 6 joint angles [j1,j2,j3,j4,j5,j6]
                    6个关节角度 [j1,j2,j3,j4,j5,j6]
            t: Running time (seconds) 0.004-3600.0
               运行时间 (秒) 0.004-3600.0
            aheadtime: Lead time (20.0-100.0), similar to the D parameter of PID
                       提前量(20.0-100.0), 类似PID的D参数
            gain: Proportional gain (200.0-1000.0), similar to the P parameter of PID
                  比例增益 (200.0-1000.0), 类似PID的P参数

        Returns:
            str: ErrorID,{ResultID},ServoJ(...);
        """
        if len(joints) != 6:
            raise ValueError("joints requires 6 joint angles")  # joints需要6个关节角度
        if not 0.004 <= t <= 3600.0:
            raise ValueError(f"t must be between 0.004 and 3600.0")  # t必须在0.004-3600.0之间
        if not 20.0 <= aheadtime <= 100.0:
            raise ValueError(f"aheadtime must be between 20.0 and 100.0")  # aheadtime必须在20.0-100.0之间
        if not 200.0 <= gain <= 1000.0:
            raise ValueError(f"gain must be between 200.0 and 1000.0")  # gain必须在200.0-1000.0之间

        cmd = f"ServoJ({joints[0]:.6f},{joints[1]:.6f},{joints[2]:.6f},{joints[3]:.6f},{joints[4]:.6f},{joints[5]:.6f},t={t:.3f},aheadtime={aheadtime:.1f},gain={gain:.1f})"
        return self._send_cmd(cmd)

    def ServoP(self, pose: Sequence[float],
                t: float = 0.1, aheadtime: float = 50.0, gain: float = 500.0) -> str:
        """
        ServoP Dynamic follow command based on Cartesian space (queued command)
        ServoP基于笛卡尔空间的动态跟随命令（队列指令）

        Args:
            pose: Cartesian pose [x,y,z,rx,ry,rz]
                  笛卡尔位姿 [x,y,z,rx,ry,rz]
            t: Running time (seconds) 0.004-3600.0
               运行时间 (秒) 0.004-3600.0
            aheadtime: Lead time (20.0-100.0)
                       提前量(20.0-100.0)
            gain: Proportional gain (200.0-1000.0)
                  比例增益 (200.0-1000.0)

        Returns:
            str: ErrorID,{ResultID},ServoP(...);
        """
        if len(pose) != 6:
            raise ValueError("pose requires 6 Cartesian pose parameters")  # pose需要6个笛卡尔位姿参数
        if not 0.004 <= t <= 3600.0:
            raise ValueError(f"t must be between 0.004 and 3600.0")  # t必须在0.004-3600.0之间
        if not 20.0 <= aheadtime <= 100.0:
            raise ValueError(f"aheadtime must be between 20.0 and 100.0")  # aheadtime必须在20.0-100.0之间
        if not 200.0 <= gain <= 1000.0:
            raise ValueError(f"gain must be between 200.0 and 1000.0")  # gain必须在200.0-1000.0之间

        cmd = f"ServoP({pose[0]:.6f},{pose[1]:.6f},{pose[2]:.6f},{pose[3]:.6f},{pose[4]:.6f},{pose[5]:.6f},t={t:.3f},aheadtime={aheadtime:.1f},gain={gain:.1f})"
        return self._send_cmd(cmd)

    # ==================== Jog ====================
    # ==================== 点动 ====================

    def MoveJog(self, axis: str = "", coord_type: CoordinateType = None,
                 user: int = None, tool: int = None) -> str:
        """
        MoveJog Jog the robot arm (immediate command)
        MoveJog点动机械臂（立即指令）

        Description (exactly as in the documentation):
        说明（与文档完全一致）：
        - If coordtype is omitted, the SDK leaves it out of the command. The controller uses the value from the last successful call.
          省略 coordtype 时，SDK 不发送该参数，控制器沿用上次成功调用时的设置。
        - If user or tool is omitted, the SDK leaves that parameter out of the command. The controller uses its default value for that parameter.
          省略 user 或 tool 时，SDK 不发送对应参数，控制器使用该参数的默认值。
        - When axisID is a joint axis (J1~J6), coordtype can only be 0 (the passed value is ignored).
          当axisID为关节轴（J1~J6）时，coordtype只能取0（忽略传入值）
        - When axisID is a Cartesian axis (X/Y/Z/Rx/Ry/Rz), coordtype can only be 1 or 2; passing 0 returns error code -6.
          当axisID为笛卡尔轴（X/Y/Z/Rx/Ry/Rz）时，coordtype只能取1或2，取0会返回错误码-6

        Args:
            axis: "X+", "X-", "J1+", etc.; empty string stops
                  "X+", "X-", "J1+" 等，空字符串停止
            coord_type: Coordinate system type, optional. None = use the controller's last successful setting
                        JOINT=0 (joint jog), USER/CARTESIAN=1 (user coordinate system), TOOL=2 (tool coordinate system)
                        坐标系类型，可选。为 None 时，沿用控制器上次成功调用时的设置。
                        JOINT=0（关节点动）, USER/CARTESIAN=1（用户坐标系）, TOOL=2（工具坐标系）
            user: User coordinate system index, optional. None = use controller default
                  用户坐标系编号，可选。None=使用控制器默认
            tool: Tool coordinate system index, optional. None = use controller default
                  工具坐标系编号，可选。None=使用控制器默认

        Returns:
            str: ErrorID,{},MoveJog(...);
        """
        if not axis:
            return self._send_cmd("MoveJog()")

        params = [axis]
        if coord_type is not None:
            params.append(f"coordtype={int(coord_type)}")
        if user is not None:
            params.append(f"user={user}")
        if tool is not None:
            params.append(f"tool={tool}")

        cmd = f"MoveJog({','.join(params)})"
        return self._send_cmd(cmd)

    # ==================== Move to point ====================
    # ==================== 运动至点位====================

    def RunTo(self, point: Sequence[float], move_type: int,
              user: int = -1, tool: int = -1,
              a: int = -1, v: int = -1,
              coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
        """
        RunTo Move to a specified point (immediate command)
        RunTo运动至指定点位（立即指令）

        Args:
            point: 6 coordinate values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
                   6个坐标值[x,y,z,rx,ry,rz] 或[j1,j2,j3,j4,j5,j6]
            move_type: Motion type
                       0=Joint motion, 1=Linear motion
                       2=Joint motion to the specified offset angle (relative joint)
                       3=Relative linear motion along the tool coordinate system
                       4=Relative linear motion along the user coordinate system
                       运动类型
                       0=关节运动, 1=直线运动
                       2=关节运动至指定偏移角度（相对关节）
                       3=沿工具坐标系相对直线运动
                       4=沿用户坐标系相对直线运动
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            coord_type: Coordinate system type (default CARTESIAN)
                        坐标系类型(默认CARTESIAN)

        Returns:
            str: ErrorID,{ResultID},RunTo(...);
        """
        if move_type not in [0, 1, 2, 3, 4]:
            raise ValueError("move_type must be in the range [0, 4]: 0=joint, 1=linear, "  # move_type 必须在 [0, 4] 范围内: 0=关节, 1=直线,
                             "2=joint offset, 3=relative linear motion in the tool coordinate system, "
                             "4=relative linear motion in the user coordinate system")  # 2=关节偏移，3=工具坐标系下的相对直线运动，4=用户坐标系下的相对直线运动

        point_str = self._fmt_pose(point, coord_type)

        params = [point_str, str(move_type)]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")

        cmd = f"RunTo({','.join(params)})"
        return self._send_cmd(cmd)

    # ==================== Trajectory reproduction ====================
    # ==================== 轨迹复现 ====================

    def GetStartPose(self, trace_name: str, path_type: int = 1) -> str:
        """
        GetStartPose Get the first point of a specified trajectory (immediate command)
        GetStartPose获取指定轨迹的第一个点位（立即指令）

        Args:
            trace_name: Trajectory file name (including the .csv suffix)
                        轨迹文件名（含后缀.csv）
            path_type: Trajectory type (1=trajectory for reproduction, 2=trajectory for fitting, default is 1)
                       轨迹类型 (1-用于复现的轨迹 2-用于拟合的轨迹，默认为1)

        Returns:
            str: ErrorID,{X,Y,Z,Rx,Ry,Rz},GetStartPose(traceName,pathType);
        """
        if path_type not in [1, 2]:
            raise ValueError("path_type must be 1 or 2")  # path_type 必须是1 或 2

        cmd = f"GetStartPose(\"{trace_name}\",{path_type})"
        return self._send_cmd(cmd)

    def MovS(self,
             trace_or_points,
             coord_type: CoordinateType = CoordinateType.CARTESIAN,
             freq: float = -1,
             user: int = -1, tool: int = -1,
             a: int = -1, v: int = -1, speed: int = -1) -> str:
        """
        MovS Fitting motion (queued command)
        MovS拟合运动（队列指令）

        Two calling methods are supported:
        支持两种调用方式：
        1. Point list method: MovS([p1, p2, p3, ...], coord_type, freq, user, tool, a, v|speed, freq)
           点位列表方式：MovS([p1, p2, p3, ...], coord_type, freq, user, tool, a, v|speed, freq)
        2. File method: MovS("xxx.csv", coord_type, freq, user, tool, a, v|speed, freq)
           文件方式：MovS("xxx.csv", coord_type, freq, user, tool, a, v|speed, freq)

        Args:
            trace_or_points:
                - str: Trajectory file name (including suffix, e.g. "xxx.csv") -> file method
                       轨迹文件名（含后缀，如"xxx.csv"） -> 文件方式
                - Sequence[Sequence[float]]: Point list, each point is [x,y,z,rx,ry,rz] or [j1..j6]
                                             点位列表，每个点位为 [x,y,z,rx,ry,rz] 或 [j1..j6]
            coord_type: Point coordinate system type (only used in the point list method)
                        点位坐标系类型（仅点位列表方式时使用）
            freq: Filter coefficient (range 0-1, 1 means filtering disabled, -1 means not set)
                  滤波系数（范围 0-1, 1表示关闭滤波，-1表示不设置）
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系索引(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系索引(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global), mutually exclusive with speed
               速度比例 (1-100, -1表示使用全局), 与speed互斥
            speed: Target speed (mm/s), mutually exclusive with v (speed takes priority)
                   目标速度 (mm/s), 与v互斥（优先speed）

        Returns:
            str: ErrorID,{ResultID},MovS(...);
        """
        params = []

        if isinstance(trace_or_points, str):
            params.append(f"\"{trace_or_points}\"")
        else:
            points = list(trace_or_points)
            if len(points) < 4 or len(points) > 50:
                raise ValueError("Point list method requires 4 to 50 points")  # 点位列表方式需要提供 4~50 个点位
            for p in points:
                if len(p) != 6:
                    raise ValueError("Each point requires 6 values [x,y,z,rx,ry,rz] or [j1..j6]")  # 每个点位需要6个值 [x,y,z,rx,ry,rz] 或 [j1..j6]
                params.append(self._fmt_pose(p, coord_type))

        if freq != -1:
            params.append(f"freq={freq}")
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

        cmd = f"MovS({','.join(params)})"
        return self._send_cmd(cmd)

    def StartPath(self, trace_name: str, is_const: int = 0,
                   multi: float = 1.0, sample: int = 50,
                   freq: float = 0.2, user: int = -1, tool: int = -1) -> str:
        """
        StartPath Reproduce a recorded motion trajectory (queued command)
        StartPath复现录制的运动轨迹（队列指令）

        Args:
            trace_name: Trajectory file name (including suffix)
                        轨迹文件名（含后缀）
            is_const: Whether to reproduce at constant speed (0=original, 1=constant speed)
                      是否匀速复现(0-原始 1-匀速)
            multi: Speed multiplier (only meaningful when is_const=0, range 0.25-2)
                   速度倍数 (仅is_const=0时有意义 范围0.25-2)
            sample: Sampling interval (ms, range 8-1000)
                    采样间隔 (ms, 范围8-1000)
            freq: Filter coefficient (range 0-1, 1 means filtering disabled)
                  滤波系数 (范围0-1, 1表示关闭滤波)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系索引(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系索引(0-50, -1表示使用当前)

        Returns:
            str: ErrorID,{ResultID},StartPath(...);
        """
        params = [f"\"{trace_name}\""]
        params.append(f"isConst={is_const}")
        params.append(f"multi={multi}")
        params.append(f"sample={sample}")
        params.append(f"freq={freq}")
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")

        cmd = f"StartPath({','.join(params)})"
        return self._send_cmd(cmd)

    # ==================== Relative motion ====================
    # ==================== 相对运动 ====================

    def RelMovJTool(self, offsetX: float, offsetY: float, offsetZ: float,
                    offsetRx: float, offsetRy: float, offsetRz: float,
                    user: int = -1, tool: int = -1,
                    a: int = -1, v: int = -1, cp: int = -1) -> str:
        """
        RelMovJTool Relative joint motion along the tool coordinate system (queued command)
        RelMovJTool沿工具坐标系进行相对关节运动（队列指令）

        Args:
            offsetX: X direction offset (mm)
                     X方向偏移 (mm)
            offsetY: Y direction offset (mm)
                     Y方向偏移 (mm)
            offsetZ: Z direction offset (mm)
                     Z方向偏移 (mm)
            offsetRx: Rx direction offset (degrees)
                      Rx方向偏移 (度)
            offsetRy: Ry direction offset (degrees)
                      Ry方向偏移 (度)
            offsetRz: Rz direction offset (degrees)
                      Rz方向偏移 (度)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            cp: Smooth transition ratio (0-100, -1 means not used)
                平滑过渡比例 (0-100, -1表示不使用)

        Returns:
            str: ErrorID,{ResultID},RelMovJTool(...);
        """
        params = [
            f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
            f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
        ]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")

        cmd = f"RelMovJTool({','.join(params)})"
        return self._send_cmd(cmd)

    def RelMovLTool(self, offsetX: float, offsetY: float, offsetZ: float,
                    offsetRx: float, offsetRy: float, offsetRz: float,
                    user: int = -1, tool: int = -1,
                    a: int = -1, v: int = -1, speed: int = -1,
                    cp: int = -1, r: int = -1) -> str:
        """
        RelMovLTool Relative linear motion along the tool coordinate system (queued command)
        RelMovLTool沿工具坐标系进行相对直线运动（队列指令）

        Args:
            offsetX: X direction offset (mm)
                     X方向偏移 (mm)
            offsetY: Y direction offset (mm)
                     Y方向偏移 (mm)
            offsetZ: Z direction offset (mm)
                     Z方向偏移 (mm)
            offsetRx: Rx direction offset (degrees)
                      Rx方向偏移 (度)
            offsetRy: Ry direction offset (degrees)
                      Ry方向偏移 (度)
            offsetRz: Rz direction offset (degrees)
                      Rz方向偏移 (度)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global), mutually exclusive with speed
               速度比例 (1-100, -1表示使用全局), 与speed互斥
            speed: Target speed (mm/s), mutually exclusive with v (speed takes priority)
                   目标速度 (mm/s), 与v互斥 (speed优先)
            cp: Smooth transition ratio (0-100, -1 means not used), mutually exclusive with r
                平滑过渡比例 (0-100, -1表示不使用), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp (r takes priority)
               平滑过渡半径 (mm), 与cp互斥 (r优先)

        Returns:
            str: ErrorID,{ResultID},RelMovLTool(...);
        """
        params = [
            f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
            f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
        ]
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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")

        cmd = f"RelMovLTool({','.join(params)})"
        return self._send_cmd(cmd)

    def RelMovJUser(self, offsetX: float, offsetY: float, offsetZ: float,
                    offsetRx: float, offsetRy: float, offsetRz: float,
                    user: int = -1, tool: int = -1,
                    a: int = -1, v: int = -1, cp: int = -1) -> str:
        """
        RelMovJUser Relative joint motion along the user coordinate system (queued command)
        RelMovJUser沿用户坐标系进行相对关节运动（队列指令）

        Args:
            offsetX: X direction offset (mm)
                     X方向偏移 (mm)
            offsetY: Y direction offset (mm)
                     Y方向偏移 (mm)
            offsetZ: Z direction offset (mm)
                     Z方向偏移 (mm)
            offsetRx: Rx direction offset (degrees)
                      Rx方向偏移 (度)
            offsetRy: Ry direction offset (degrees)
                      Ry方向偏移 (度)
            offsetRz: Rz direction offset (degrees)
                      Rz方向偏移 (度)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            cp: Smooth transition ratio (0-100, -1 means not used)
                平滑过渡比例 (0-100, -1表示不使用)

        Returns:
            str: ErrorID,{ResultID},RelMovJUser(...);
        """
        params = [
            f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
            f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
        ]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")

        cmd = f"RelMovJUser({','.join(params)})"
        return self._send_cmd(cmd)

    def RelMovLUser(self, offsetX: float, offsetY: float, offsetZ: float,
                    offsetRx: float, offsetRy: float, offsetRz: float,
                    user: int = -1, tool: int = -1,
                    a: int = -1, v: int = -1, speed: int = -1,
                    cp: int = -1, r: int = -1) -> str:
        """
        RelMovLUser Relative linear motion along the user coordinate system (queued command)
        RelMovLUser沿用户坐标系进行相对直线运动（队列指令）

        Args:
            offsetX: X direction offset (mm)
                     X方向偏移 (mm)
            offsetY: Y direction offset (mm)
                     Y方向偏移 (mm)
            offsetZ: Z direction offset (mm)
                     Z方向偏移 (mm)
            offsetRx: Rx direction offset (degrees)
                      Rx方向偏移 (度)
            offsetRy: Ry direction offset (degrees)
                      Ry方向偏移 (度)
            offsetRz: Rz direction offset (degrees)
                      Rz方向偏移 (度)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global), mutually exclusive with speed
               速度比例 (1-100, -1表示使用全局), 与speed互斥
            speed: Target speed (mm/s), mutually exclusive with v (speed takes priority)
                   目标速度 (mm/s), 与v互斥 (speed优先)
            cp: Smooth transition ratio (0-100, -1 means not used), mutually exclusive with r
                平滑过渡比例 (0-100, -1表示不使用), 与r互斥
            r: Smooth transition radius (mm), mutually exclusive with cp (r takes priority)
               平滑过渡半径 (mm), 与cp互斥 (r优先)

        Returns:
            str: ErrorID,{ResultID},RelMovLUser(...);
        """
        params = [
            f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
            f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
        ]
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
        if r != -1:
            params.append(f"r={r}")
        elif cp != -1:
            params.append(f"cp={cp}")

        cmd = f"RelMovLUser({','.join(params)})"
        return self._send_cmd(cmd)

    def RelJointMovJ(self, offset1: float, offset2: float, offset3: float,
                     offset4: float, offset5: float, offset6: float,
                     user: int = -1, tool: int = -1,
                     a: int = -1, v: int = -1, cp: int = -1) -> str:
        """
        RelJointMovJ Relative joint motion along the joint coordinate system (queued command)
        RelJointMovJ沿关节坐标系进行相对关节运动（队列指令）

        Args:
            offset1: J1 joint offset (degrees)
                     J1关节偏移 (度)
            offset2: J2 joint offset (degrees)
                     J2关节偏移 (度)
            offset3: J3 joint offset (degrees)
                     J3关节偏移 (度)
            offset4: J4 joint offset (degrees)
                     J4关节偏移 (度)
            offset5: J5 joint offset (degrees)
                     J5关节偏移 (度)
            offset6: J6 joint offset (degrees)
                     J6关节偏移 (度)
            user: User coordinate system index (0-50, -1 means use current)
                  用户坐标系编号(0-50, -1表示使用当前)
            tool: Tool coordinate system index (0-50, -1 means use current)
                  工具坐标系编号(0-50, -1表示使用当前)
            a: Acceleration ratio (1-100, -1 means use global)
               加速度比例 (1-100, -1表示使用全局)
            v: Speed ratio (1-100, -1 means use global)
               速度比例 (1-100, -1表示使用全局)
            cp: Smooth transition ratio (0-100, -1 means not used)
                平滑过渡比例 (0-100, -1表示不使用)

        Returns:
            str: ErrorID,{ResultID},RelJointMovJ(...);
        """
        params = [
            f"{offset1:.6f}", f"{offset2:.6f}", f"{offset3:.6f}",
            f"{offset4:.6f}", f"{offset5:.6f}", f"{offset6:.6f}"
        ]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a}")
        if v != -1:
            params.append(f"v={v}")
        if cp != -1:
            params.append(f"cp={cp}")

        cmd = f"RelJointMovJ({','.join(params)})"
        return self._send_cmd(cmd)

    def RelPointTool(self, p: Sequence[float], offset: Sequence[float],
                     coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
        """
        RelPointTool Cartesian point offset along the tool coordinate system (immediate command)
        RelPointTool 在工具坐标系中对笛卡尔点位施加偏移（立即指令）

        Args:
            p: Point with 6 values [x,y,z,rx,ry,rz] or [j1..j6]
               6个值的点位[x,y,z,rx,ry,rz] 或 [j1..j6]
            offset: Offset with 6 values [offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
                    6个值的偏移[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
            coord_type: Point coordinate system type. The default is Cartesian (pose). Joint coordinates (joint) are also supported.
                        点位坐标系类型。默认为笛卡尔(pose)，也可选关节(joint)

        Returns:
            str: ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointTool(...);
        """
        if len(p) != 6:
            raise ValueError("p requires 6 point parameters")  # p需要6个点位参数
        if len(offset) != 6:
            raise ValueError("offset requires 6 offset parameters [offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]")  # offset需要6个偏移参数[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]

        p_str = self._fmt_pose(p, coord_type)
        offset_values = ",".join([f"{v:.6f}" for v in offset])
        cmd = f"RelPointTool({p_str},{{{offset_values}}})"
        return self._send_cmd(cmd)

    def RelPointUser(self, p: Sequence[float], offset: Sequence[float],
                     coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
        """
        RelPointUser Cartesian point offset along the user coordinate system (immediate command)
        RelPointUser 在用户坐标系中对笛卡尔点位施加偏移（立即指令）

        Args:
            p: Point with 6 values [x,y,z,rx,ry,rz] or [j1..j6]
               6个值的点位[x,y,z,rx,ry,rz] 或 [j1..j6]
            offset: Offset with 6 values [offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
                    6个值的偏移[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
            coord_type: Point coordinate system type. The default is Cartesian (pose). Joint coordinates (joint) are also supported.
                        点位坐标系类型。默认为笛卡尔(pose)，也可选关节(joint)

        Returns:
            str: ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointUser(...);
        """
        if len(p) != 6:
            raise ValueError("p requires 6 point parameters")  # p需要6个点位参数
        if len(offset) != 6:
            raise ValueError("offset requires 6 offset parameters [offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]")  # offset需要6个偏移参数[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]

        p_str = self._fmt_pose(p, coord_type)
        offset_values = ",".join([f"{v:.6f}" for v in offset])
        cmd = f"RelPointUser({p_str},{{{offset_values}}})"
        return self._send_cmd(cmd)

    def RelJoint(self, joints: Sequence[float], offset: Sequence[float]) -> str:
        """
        RelJoint Joint point offset (immediate command)
        RelJoint关节点位偏移（立即指令）

        Args:
            joints: 6 joint angles [J1,J2,J3,J4,J5,J6]
                    6个关节角度[J1,J2,J3,J4,J5,J6]
            offset: 6 offsets [offset1,offset2,offset3,offset4,offset5,offset6]
                    6个偏移[offset1,offset2,offset3,offset4,offset5,offset6]

        Returns:
            str: ErrorID,{J1,J2,J3,J4,J5,J6},RelJoint(...);
        """
        if len(joints) != 6:
            raise ValueError("joints requires 6 joint angles [J1,J2,J3,J4,J5,J6]")  # joints需要6个关节角度[J1,J2,J3,J4,J5,J6]
        if len(offset) != 6:
            raise ValueError("offset requires 6 offsets [offset1,offset2,offset3,offset4,offset5,offset6]")  # offset需要6个偏移[offset1,offset2,offset3,offset4,offset5,offset6]

        joints_str = ",".join([f"{v:.6f}" for v in joints])
        offset_values = ",".join([f"{v:.6f}" for v in offset])
        cmd = f"RelJoint({joints_str},{{{offset_values}}})"
        return self._send_cmd(cmd)

    # ==================== Command ID query ====================
    # ==================== 指令ID查询 ====================

    def GetCurrentCommandID(self) -> str:
        """
        GetCurrentCommandID Get the algorithm queue ID of the currently executing command (immediate command)
        GetCurrentCommandID获取当前执行指令的算法队列ID（立即指令）

        Returns:
            str: ErrorID,{ResultID},GetCurrentCommandID();
        """
        return self._send_cmd("GetCurrentCommandID()")

    # ==================== Coordinate system offset ====================
    # ==================== 坐标系偏移====================

    def StartRTOffset(self) -> str:
        """
        StartRTOffset Start coordinate system offset (queued command)
        StartRTOffset启动坐标系偏移（队列指令）

        Returns:
            str: ErrorID,{ResultID},StartRTOffset();
        """
        return self._send_cmd("StartRTOffset()")

    def EndRTOffset(self) -> str:
        """
        EndRTOffset End coordinate system offset (queued command)
        EndRTOffset结束坐标系偏移（队列指令）

        Returns:
            str: ErrorID,{ResultID},EndRTOffset();
        """
        return self._send_cmd("EndRTOffset()")

    def OffsetPara(self, x: float, y: float, z: float,
                   rx: float, ry: float, rz: float) -> str:
        """
        OffsetPara Set coordinate system offset values (immediate command)
        OffsetPara设置坐标系偏移值（立即指令）

        Args:
            x: X direction offset (mm)
               X方向偏移 (mm)
            y: Y direction offset (mm)
               Y方向偏移 (mm)
            z: Z direction offset (mm)
               Z方向偏移 (mm)
            rx: Rx direction offset (degrees)
                Rx方向偏移 (度)
            ry: Ry direction offset (degrees)
                Ry方向偏移 (度)
            rz: Rz direction offset (degrees)
                Rz方向偏移 (度)

        Returns:
            str: ErrorID,{},OffsetPara(...);
        """
        cmd = f"OffsetPara({x:.6f},{y:.6f},{z:.6f},{rx:.6f},{ry:.6f},{rz:.6f})"
        return self._send_cmd(cmd)

    # ==================== Trajectory recovery ====================
    # ==================== 轨迹恢复 ====================

    def SetResumeOffset(self, distance: float) -> str:
        """
        SetResumeOffset Set the fallback distance for trajectory recovery (immediate command)
        SetResumeOffset设置轨迹恢复的回退距离（立即指令）

        Args:
            distance: Fallback distance (mm)
                      回退距离 (mm)

        Returns:
            str: ErrorID,{},SetResumeOffset(distance);
        """
        return self._send_cmd(f"SetResumeOffset({distance:.6f})")

    def PathRecovery(self) -> str:
        """
        PathRecovery Start trajectory recovery (immediate command)
        PathRecovery开始轨迹恢复（立即指令）

        Returns:
            str: ErrorID,{},PathRecovery();
        """
        return self._send_cmd("PathRecovery()")

    def PathRecoveryStop(self) -> str:
        """
        PathRecoveryStop Stop the robot during trajectory recovery (immediate command)
        PathRecoveryStop轨迹恢复过程中停止机器人（立即指令）

        Returns:
            str: ErrorID,{},PathRecoveryStop();
        """
        return self._send_cmd("PathRecoveryStop()")

    def PathRecoveryStatus(self) -> str:
        """
        PathRecoveryStatus Query trajectory recovery status (immediate command)
        PathRecoveryStatus查询轨迹恢复状态（立即指令）

        Returns:
            str: ErrorID,{status},PathRecoveryStatus();
                 status: 0=returned to pause position, 1=small deviation, 2=large deviation
                 status: 0-已回到暂停位置 1-偏差较小, 2-偏差较大
        """
        return self._send_cmd("PathRecoveryStatus()")
