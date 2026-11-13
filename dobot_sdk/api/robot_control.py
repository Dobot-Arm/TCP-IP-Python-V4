# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Robot Control module - Status query, kinematics calculation, reachability check, robot control
机器人控制模块 - 状态查询、运动学计算、可达性检查、机器人控制"""

from typing import Tuple, Sequence
from ..core.connection import DobotConnection


class RobotControl:
    """Robot control module - Status query, kinematics calculation, reachability check, log, script control
    机器人控制模块 - 状态查询、运动学计算、可达性检查、日志、脚本控制"""

    def __init__(self, connection: DobotConnection):
        self.connection = connection

    def _send_cmd(self, command: str) -> str:
        """Send a command and receive the response
        发送命令并接收响应"""
        return self.connection.send_receive_text(command)

    # ==================== 控制模式与电源 / Control mode and power ====================

    def RequestControl(self) -> str:
        """
        Request a switch to TCP control mode (immediate command)
        请求切换至 TCP 控制模式（立即指令）

        说明 / Notes:
        - TCP control mode can be requested when the robot is powered off, or when it is disabled with the brakes engaged and no pause active.
          机器人未上电时，或处于下使能且未暂停、抱闸未松开时，可请求切换至 TCP 控制模式。
        - Use other TCP commands, such as EnableRobot and motion commands, after successfully switching to TCP control mode.
          成功切换至 TCP 控制模式后，才可执行 EnableRobot、运动指令等其他 TCP 指令。

        Returns:
            str: ErrorID,{},RequestControl();
        """
        return self._send_cmd("RequestControl()")

    def PowerOn(self) -> str:
        """PowerOn powers on the robot (immediate command)
        PowerOn机器人上电（立即指令）"""
        return self._send_cmd("PowerOn()")

    def EnableRobot(self, load: float = 0.0, **kwargs) -> str:
        """
        EnableRobot enables the robot (immediate command)
        EnableRobot使能机器人（立即指令）

        原型 / Prototype: EnableRobot(load,centerX,centerY,centerZ,isCheck)

        Args:
            load: Payload weight (kg) / 负载重量 (kg)
            centerX: Payload center of gravity X coordinate (mm) / 负载重心X坐标 (mm)
            centerY: Payload center of gravity Y coordinate (mm) / 负载重心Y坐标 (mm)
            centerZ: Payload center of gravity Z coordinate (mm) / 负载重心Z坐标 (mm)
            isCheck: Whether to check payload (0=do not check, 1=check) / 是否检查负载 (0=不检查, 1=检查)
        """
        if load == 0.0:
            cmd = "EnableRobot()"
        elif 'centerX' not in kwargs:
            cmd = f"EnableRobot({load:.6f})"
        else:
            cx = kwargs.get('centerX', 0.0)
            cy = kwargs.get('centerY', 0.0)
            cz = kwargs.get('centerZ', 0.0)
            check = kwargs.get('isCheck', 0)
            cmd = f"EnableRobot({load:.6f},{cx:.6f},{cy:.6f},{cz:.6f},{check})"
        return self._send_cmd(cmd)

    def DisableRobot(self) -> str:
        """DisableRobot disables the robot (immediate command)
        DisableRobot下使能机器人（立即指令）"""
        return self._send_cmd("DisableRobot()")

    def ClearError(self) -> str:
        """ClearError clears the robot alarm (immediate command)
        ClearError清除机器人报警（立即指令）"""
        return self._send_cmd("ClearError()")

    # ==================== 运动控制 / Motion control ====================

    def RunScript(self, script_name: str) -> str:
        """
        RunScript runs the specified project (immediate command)
        RunScript运行指定工程（立即指令）

        Args:
            script_name: Project file name / 工程文件名
        """
        return self._send_cmd(f"RunScript(\"{script_name}\")")

    def Stop(self) -> str:
        """Stop stops motion (or the running project) (immediate command)
        Stop停止运动（或正在运行的工程）（立即指令）"""
        return self._send_cmd("Stop()")

    def Pause(self) -> str:
        """Pause pauses motion (or the running project) (immediate command)
        Pause暂停运动（或正在运行的工程）（立即指令）"""
        return self._send_cmd("Pause()")

    def Continue(self) -> str:
        """Continue resumes motion (or the paused project) (immediate command)
        Continue继续运动（或已暂停的工程）（立即指令）"""
        return self._send_cmd("Continue()")

    def EmergencyStop(self, mode: int) -> str:
        """
        Trigger or release the robot's emergency stop (immediate command)
        触发或解除机器人急停（立即指令）

        Args:
            mode: Emergency stop operation. 1 triggers the emergency stop. 0 releases it.
                  急停操作。1 表示触发急停，0 表示解除急停。
        """
        if mode not in [0, 1]:
            raise ValueError("mode must be 0 or 1")  # mode必须是0或1 / mode must be 0 or 1
        return self._send_cmd(f"EmergencyStop({mode})")

    # ==================== 抱闸与拖拽 / Brake and drag ====================

    def BrakeControl(self, axis_id: int, value: int) -> str:
        """
        BrakeControl controls the brake of the specified joint (immediate command)
        BrakeControl控制指定关节的抱闸（立即指令）

        Args:
            axis_id: Joint number (1-6) / 关节编号 (1-6)
            value: 0 = engage the brake (joint locked), 1 = release the brake (joint can move) / 0-抱闸, 1-松闸
        """
        if not 1 <= axis_id <= 6:
            raise ValueError("joint number must be between 1 and 6")  # 关节编号必须在1-6之间 / joint number must be between 1 and 6
        if value not in [0, 1]:
            raise ValueError("value must be 0 or 1")  # value必须是0或1 / value must be 0 or 1
        return self._send_cmd(f"BrakeControl({axis_id},{value})")

    def StartDrag(self) -> str:
        """StartDrag puts the robot into joint drag mode (immediate command)
        StartDrag机器人进入关节拖拽模式（立即指令）"""
        return self._send_cmd("StartDrag()")

    def StopDrag(self) -> str:
        """StopDrag exits the robot drag mode (immediate command)
        StopDrag机器人退出拖拽模式（立即指令）"""
        return self._send_cmd("StopDrag()")

    def DragSensivity(self, index: int, value: int) -> str:
        """
        DragSensivity sets the drag sensitivity (immediate command)
        DragSensivity设置拖拽灵敏度（立即指令）

        Args:
            index: Axis index. Range: [0, 6]. 0 sets the same sensitivity for all axes. Values 1–6 correspond to axes J1–J6, respectively.
                   轴序号, 取值范围: [0,6]。0表示所有轴设置为相同的灵敏度。1~6分别表示设置J1~J6轴的灵敏度。
            value: Drag sensitivity. The smaller the value, the greater the resistance during dragging. Range: [1, 90].
                   拖拽灵敏度, 值越小, 拖拽时的阻力越大。取值范围: [1, 90]。
        """
        if not 0 <= index <= 6:
            raise ValueError("axis index must be between 0 and 6")  # 轴序号必须在0-6之间 / axis index must be between 0 and 6
        if not 1 <= value <= 90:
            raise ValueError("drag sensitivity must be between 1 and 90")  # 拖拽灵敏度必须在1-90之间 / drag sensitivity must be between 1 and 90
        return self._send_cmd(f"DragSensivity({index},{value})")

    # ==================== 速度与加速度设置 / Speed and acceleration settings ====================

    def SpeedFactor(self, factor: int) -> str:
        """
        SpeedFactor sets the global speed ratio (immediate command)
        SpeedFactor设置全局速度比例（立即指令）

        Args:
            factor: Speed ratio (1-100) / 速度比例 (1-100)
        """
        if not 1 <= factor <= 100:
            raise ValueError("speed ratio must be between 1 and 100")  # 速度比例必须在1-100之间 / speed ratio must be between 1 and 100
        return self._send_cmd(f"SpeedFactor({factor})")

    def AccJ(self, acc: int) -> str:
        """
        AccJ sets the acceleration ratio of joint motion (immediate command)
        AccJ设置关节运动方式的加速度比例（立即指令）

        Args:
            acc: Acceleration ratio (1-100) / 加速度比例 (1-100)
        """
        if not 1 <= acc <= 100:
            raise ValueError("acceleration ratio must be between 1 and 100")  # 加速度比例必须在1-100之间 / acceleration ratio must be between 1 and 100
        return self._send_cmd(f"AccJ({acc})")

    def AccL(self, acc: int) -> str:
        """
        AccL sets the acceleration ratio of linear and arc motion (immediate command)
        AccL设置直线和弧线运动方式的加速度比例（立即指令）

        Args:
            acc: Acceleration ratio (1-100) / 加速度比例 (1-100)
        """
        if not 1 <= acc <= 100:
            raise ValueError("acceleration ratio must be between 1 and 100")  # 加速度比例必须在 1–100 之间 / acceleration ratio must be between 1 and 100
        return self._send_cmd(f"AccL({acc})")

    def VelJ(self, vel: int) -> str:
        """
        VelJ sets the speed ratio of joint motion (immediate command)
        VelJ设置关节运动方式的速度比例（立即指令）

        Args:
            vel: Speed ratio (1-100) / 速度比例 (1-100)
        """
        if not 1 <= vel <= 100:
            raise ValueError("speed ratio must be between 1 and 100")  # 速度比例必须在 1–100 之间 / speed ratio must be between 1 and 100
        return self._send_cmd(f"VelJ({vel})")

    def VelL(self, vel: int) -> str:
        """
        VelL sets the speed ratio of linear and arc motion (immediate command)
        VelL设置直线和弧线运动方式的速度比例（立即指令）

        Args:
            vel: Speed ratio (1-100) / 速度比例 (1-100)
        """
        if not 1 <= vel <= 100:
            raise ValueError("speed ratio must be between 1 and 100")  # 速度比例必须在 1–100 之间 / speed ratio must be between 1 and 100
        return self._send_cmd(f"VelL({vel})")

    def CP(self, value: int) -> str:
        """
        CP sets the smooth transition ratio (immediate command)
        CP设置平滑过渡比例（立即指令）

        Args:
            value: Smooth transition ratio (0-100) / 平滑过渡比例 (0-100)
        """
        if not 0 <= value <= 100:
            raise ValueError("smooth transition ratio must be between 0 and 100")  # 平滑过渡比例必须在0-100之间 / smooth transition ratio must be between 0 and 100
        return self._send_cmd(f"CP({value})")

    # ==================== 坐标系设置 / Coordinate system settings ====================

    def User(self, index: int) -> str:
        """
        User sets the global user coordinate system (queued command)
        User设置全局用户坐标系（队列指令）

        Args:
            index: User coordinate system index (0-50) / 用户坐标系编号(0-50)
        """
        if not 0 <= index <= 50:
            raise ValueError("user coordinate system index must be between 0 and 50")  # 用户坐标系编号必须在0-50之间 / user coordinate system index must be between 0 and 50
        return self._send_cmd(f"User({index})")

    def SetUser(self, index: int, pose: Sequence[float], type: int = None) -> str:
        """
        SetUser modifies the specified user coordinate system (immediate command)
        SetUser修改指定的用户坐标系（立即指令）

        Args:
            index: User coordinate system index (1-50) / 用户坐标系编号(1-50)
            pose: 6 coordinate parameters [x,y,z,rx,ry,rz] / 6个坐标参数[x,y,z,rx,ry,rz]
            type: Whether to make the coordinate system change globally effective.
                  0: The coordinate system modified by this command is only effective during the current project runtime.
                  1: The coordinate system modified by this command will be saved by the controller.
                  是否使坐标系改动全局生效。0: 该命令修改的坐标系仅在当前工程运行中生效。1: 该命令修改的坐标系将会被控制器保存。
        """
        if not 1 <= index <= 50:
            raise ValueError("user coordinate system index must be between 1 and 50")  # 用户坐标系编号必须在1-50之间 / user coordinate system index must be between 1 and 50
        if len(pose) != 6:
            raise ValueError("pose requires 6 parameters")  # pose需要6个参数 / pose requires 6 parameters
        pose_str = "{" + ",".join([f"{v:.6f}" for v in pose]) + "}"
        if type is not None:
            if type not in [0, 1]:
                raise ValueError("type must be 0 or 1")  # type必须是0或1 / type must be 0 or 1
            return self._send_cmd(f"SetUser({index},{pose_str},{type})")
        return self._send_cmd(f"SetUser({index},{pose_str})")

    def CalcUser(self, index: int, matrix_direction: int, offset: Sequence[float]) -> str:
        """
        CalcUser calculates the user coordinate system (immediate command)
        CalcUser计算用户坐标系（立即指令）

        Args:
            index: User coordinate system index (0-50) / 用户坐标系编号(0-50)
            matrix_direction: Calculation direction (1-left multiply, coordinate system deflects along base coordinate system; 0-right multiply, coordinate system deflects along itself)
                              计算方向 (1-左乘，坐标系沿基坐标系偏转; 0-右乘，坐标系沿自身偏转)
            offset: Offset value [x,y,z,rx,ry,rz] / 偏移值 [x,y,z,rx,ry,rz]
        """
        if not 0 <= index <= 50:
            raise ValueError("user coordinate system index must be between 0 and 50")  # 用户坐标系编号必须在0-50之间 / user coordinate system index must be between 0 and 50
        if matrix_direction not in [0, 1]:
            raise ValueError("matrix_direction must be 0 or 1")  # matrix_direction 必须是0或1 / matrix_direction must be 0 or 1
        if len(offset) != 6:
            raise ValueError("offset requires 6 parameters")  # offset需要6个参数 / offset requires 6 parameters
        offset_str = "{" + ",".join([f"{v:.6f}" for v in offset]) + "}"
        return self._send_cmd(f"CalcUser({index},{matrix_direction},{offset_str})")

    def Tool(self, index: int) -> str:
        """
        Tool sets the global tool coordinate system (queued command)
        Tool设置全局工具坐标系（队列指令）

        Args:
            index: Tool coordinate system index (0-50) / 工具坐标系编号(0-50)
        """
        if not 0 <= index <= 50:
            raise ValueError("tool coordinate system index must be between 0 and 50")  # 工具坐标系编号必须在0-50之间 / tool coordinate system index must be between 0 and 50
        return self._send_cmd(f"Tool({index})")

    def SetTool(self, index: int, pose: Sequence[float], type: int = None) -> str:
        """
        SetTool modifies the specified tool coordinate system (immediate command)
        SetTool修改指定的工具坐标系（立即指令）

        Args:
            index: Tool coordinate system index (1-50) / 工具坐标系编号(1-50)
            pose: 6 coordinate parameters [x,y,z,rx,ry,rz] / 6个坐标参数[x,y,z,rx,ry,rz]
            type: Whether to make the coordinate system change globally effective.
                  0: The coordinate system modified by this command is only effective during the current project runtime.
                  1: The coordinate system modified by this command will be saved by the controller.
                  是否使坐标系改动全局生效。0: 该命令修改的坐标系仅在当前工程运行中生效。1: 该命令修改的坐标系将会被控制器保存。
        """
        if not 1 <= index <= 50:
            raise ValueError("tool coordinate system index must be between 1 and 50")  # 工具坐标系编号必须在1-50之间 / tool coordinate system index must be between 1 and 50
        if len(pose) != 6:
            raise ValueError("pose requires 6 parameters")  # pose需要6个参数 / pose requires 6 parameters
        pose_str = "{" + ",".join([f"{v:.6f}" for v in pose]) + "}"
        if type is not None:
            if type not in [0, 1]:
                raise ValueError("type must be 0 or 1")  # type必须是0或1 / type must be 0 or 1
            return self._send_cmd(f"SetTool({index},{pose_str},{type})")
        return self._send_cmd(f"SetTool({index},{pose_str})")

    def CalcTool(self, index: int, matrix_direction: int, offset: Sequence[float]) -> str:
        """
        CalcTool calculates the tool coordinate system (immediate command)
        CalcTool计算工具坐标系（立即指令）

        Args:
            index: Tool coordinate system index (0-50) / 工具坐标系编号(0-50)
            matrix_direction: Calculation direction (1-left multiply, coordinate system deflects along flange coordinate system; 0-right multiply, coordinate system deflects along itself)
                              计算方向 (1-左乘，坐标系沿法兰坐标系偏转; 0-右乘，坐标系沿自身偏转)
            offset: Offset value [x,y,z,rx,ry,rz] / 偏移值 [x,y,z,rx,ry,rz]
        """
        if not 0 <= index <= 50:
            raise ValueError("tool coordinate system index must be between 0 and 50")  # 工具坐标系编号必须在0-50之间 / tool coordinate system index must be between 0 and 50
        if matrix_direction not in [0, 1]:
            raise ValueError("matrix_direction must be 0 or 1")  # matrix_direction 必须是0或1 / matrix_direction must be 0 or 1
        if len(offset) != 6:
            raise ValueError("offset requires 6 parameters")  # offset需要6个参数 / offset requires 6 parameters
        offset_str = "{" + ",".join([f"{v:.6f}" for v in offset]) + "}"
        return self._send_cmd(f"CalcTool({index},{matrix_direction},{offset_str})")

    # ==================== 负载设置 / Payload settings ====================

    def SetPayload(self, load_or_name, *args, **kwargs) -> str:
        """
        SetPayload sets the end-effector payload of the arm (queued command)
        SetPayload设置机械臂末端负载（队列指令）

        Supports two calling methods (consistent with the documentation):
        支持两种调用方式（与文档完全一致）：
        Method 1: SetPayload(load, x, y, z) / 方式一：SetPayload(load, x, y, z)
        Method 2: SetPayload(name) / 方式二：SetPayload(name)

        Also maintains backward compatibility: SetPayload(load, center=[x,y,z])
        同时保持向后兼容：SetPayload(load, center=[x,y,z])

        Args:
            load_or_name:
                - float: Payload weight (kg) -> Method 1 / 负载重量 (kg) → 方式一
                - str: Preset payload parameter group name -> Method 2 / 预设负载参数组名称 → 方式二
            x (optional, float): End-effector payload X-axis eccentricity coordinate (mm) / 末端负载X轴偏心坐标 (mm)
            y (optional, float): End-effector payload Y-axis eccentricity coordinate (mm) / 末端负载Y轴偏心坐标 (mm)
            z (optional, float): End-effector payload Z-axis eccentricity coordinate (mm) / 末端负载Z轴偏心坐标 (mm)
            center (optional, Sequence[float]): Backward compatible, payload center of gravity [x,y,z]
                                                向后兼容，负载重心[x,y,z]
            preset_name (optional, str): Backward compatible, if provided it ignores the previous two, equivalent to Method 2
                                         向后兼容，若传入则忽略前两者，等效于方式二
        """
        center = kwargs.get('center', None)
        preset_name = kwargs.get('preset_name', None)

        if isinstance(load_or_name, str):
            return self._send_cmd(f'SetPayload("{load_or_name}")')

        if preset_name is not None:
            return self._send_cmd(f'SetPayload("{preset_name}")')

        load = float(load_or_name)

        x = y = z = None
        if len(args) == 3:
            x, y, z = args
        elif len(args) == 1 and isinstance(args[0], (list, tuple)):
            if len(args[0]) != 3:
                raise ValueError("center list requires 3 parameters [x,y,z]")  # center列表需要3个参数[x,y,z] / center list requires 3 parameters [x,y,z]
            x, y, z = args[0]
        elif center is not None:
            if len(center) != 3:
                raise ValueError("center requires 3 parameters [x,y,z]")  # center需要3个参数[x,y,z] / center requires 3 parameters [x,y,z]
            x, y, z = center
        elif len(args) != 0:
            raise ValueError("positional arguments only support SetPayload(load, x, y, z) three-parameter form or SetPayload(load, [x,y,z]) list form")  # 位置参数只支持 SetPayload(load, x, y, z) 三参数形式 或 SetPayload(load, [x,y,z]) 列表形式 / positional arguments only support SetPayload(load, x, y, z) three-parameter form or SetPayload(load, [x,y,z]) list form

        if x is not None and y is not None and z is not None:
            return self._send_cmd(f"SetPayload({load:.6f},{float(x):.6f},{float(y):.6f},{float(z):.6f})")
        return self._send_cmd(f"SetPayload({load:.6f})")

    # ==================== 碰撞检测设置 / Collision detection settings ====================

    def SetCollisionLevel(self, level: int) -> str:
        """
        SetCollisionLevel sets the collision detection level (queued command)
        SetCollisionLevel设置碰撞检测等级（队列指令）

        Args:
            level: Collision detection level (0–5). 0 disables collision detection. For levels 1–5, higher values mean greater sensitivity.
                   碰撞检测等级（0–5）。0 表示关闭碰撞检测，1–5 的数值越大，灵敏度越高。
        """
        if not 0 <= level <= 5:
            raise ValueError("collision detection level must be between 0 and 5")  # 碰撞检测等级必须在0-5之间 / collision detection level must be between 0 and 5
        return self._send_cmd(f"SetCollisionLevel({level})")

    def SetBackDistance(self, distance: float) -> str:
        """
        Set the distance to retract along the original path after a collision (queued command)
        设置碰撞后沿原路径回退的距离（队列指令）

        Args:
            distance: Distance to retract along the original path, in millimeters. Range: [0, 50].
                      沿原路径回退的距离，单位为毫米，取值范围为 [0, 50]。
        """
        if not 0 <= distance <= 50:
            raise ValueError("distance must be between 0 and 50 mm")  # distance 必须在 0–50 mm 之间 / distance must be between 0 and 50 mm
        return self._send_cmd(f"SetBackDistance({distance:.6f})")

    def SetPostCollisionMode(self, mode: int) -> str:
        """
        SetPostCollisionMode sets the post-collision handling method (queued command)
        SetPostCollisionMode设置碰撞后处理方式（队列指令）

        Args:
            mode: Post-collision handling mode / 碰撞后处理模式
                  0: Enter the stopped state after a collision. / 检测到碰撞后进入停止状态。
                  1: Enter the paused state after a collision. / 检测到碰撞后进入暂停状态。
                  2: Accepted by the SDK. Controller support and behavior require firmware-specific confirmation.
                     SDK 接受该值。控制器是否支持及具体行为需按固件版本确认。
        """
        if mode not in [0, 1, 2]:
            raise ValueError("post-collision handling mode must be 0, 1 or 2")  # 碰撞后处理模式必须是0、1或2 / post-collision handling mode must be 0, 1 or 2
        return self._send_cmd(f"SetPostCollisionMode({mode})")

    # ==================== 安全皮肤与安全区域 / Safety skin and safety zone ====================

    def EnableSafeSkin(self, status: int) -> str:
        """
        EnableSafeSkin enables or disables the safety skin function (queued command)
        EnableSafeSkin开启或关闭安全皮肤功能（队列指令）

        Args:
            status: 0-disable, 1-enable / 0-关闭, 1-开启
        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"EnableSafeSkin({status})")

    def SetSafeSkin(self, part: int, sensitivity: int) -> str:
        """
        SetSafeSkin sets the sensitivity of each part of the safety skin (queued command)
        SetSafeSkin设置安全皮肤各个部位的灵敏度（队列指令）

        Args:
            part: Safety skin part number. 3=forearm, 4~6=J4~J6 / 安全皮肤部位编号。3=小臂，4~6=J4~J6
            sensitivity: Sensitivity level [0, 3]. 0=off, 1=low, 2=medium, 3=high
                         灵敏度等级 [0, 3]。0=关闭，1=低，2=中，3=高
        """
        if not (part == 3 or 4 <= part <= 6):
            raise ValueError("part can only be 3 (forearm) or 4~6 (J4~J6)")  # part 只能是 3(小臂) 或 4~6 (J4~J6) / part can only be 3 (forearm) or 4~6 (J4~J6)
        if not 0 <= sensitivity <= 3:
            raise ValueError("sensitivity must be between 0 and 3")  # sensitivity 必须在0到3之间 / sensitivity must be between 0 and 3
        return self._send_cmd(f"SetSafeSkin({part},{sensitivity})")

    def SetSafeWallEnable(self, index: int, status: int) -> str:
        """
        SetSafeWallEnable enables or disables the specified safety wall (queued command)
        SetSafeWallEnable开启或关闭指定的安全墙（队列指令）

        Args:
            index: Safety wall number / 安全墙编号
            status: 0-disable, 1-enable / 0-关闭, 1-开启
        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"SetSafeWallEnable({index},{status})")

    def SetWorkZoneEnable(self, index: int, status: int) -> str:
        """
        SetWorkZoneEnable enables or disables the specified safety zone (queued command)
        SetWorkZoneEnable开启或关闭指定的安全区域（队列指令）

        Args:
            index: Safety zone number / 安全区域编号
            status: 0-disable, 1-enable / 0-关闭, 1-开启
        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"SetWorkZoneEnable({index},{status})")

    # ==================== 状态查询 / Status query ====================

    def RobotMode(self) -> str:
        """RobotMode gets the current robot status (immediate command)
        RobotMode获取机器人当前状态（立即指令）"""
        return self._send_cmd("RobotMode()")

    def GetPose(self, user: int = None, tool: int = None) -> str:
        """
        GetPose gets the Cartesian coordinates of the current robot pose in the specified coordinate system (immediate command)
        GetPose获取机器人当前位姿在指定坐标系下的笛卡尔坐标（立即指令）

        Args:
            user: User coordinate system index (0-50) / 用户坐标系索引(0-50)
            tool: Tool coordinate system index (0-50) / 工具坐标系索引(0-50)
        """
        has_coord = user is not None or tool is not None
        if has_coord and (user is None or tool is None):
            raise ValueError("user and tool parameters must be set together or not set at all")  # user 和 tool 参数必须同时设置或都不设置 / user and tool parameters must be set together or not set at all

        if has_coord:
            if not (0 <= user <= 50):
                raise ValueError("user must be between 0 and 50")  # user 必须在0-50 之间 / user must be between 0 and 50
            if not (0 <= tool <= 50):
                raise ValueError("tool must be between 0 and 50")  # tool 必须在0-50 之间 / tool must be between 0 and 50
            return self._send_cmd(f"GetPose(user={user},tool={tool})")
        else:
            return self._send_cmd("GetPose()")

    def GetAngle(self) -> str:
        """GetAngle gets the joint coordinates of the current robot pose (immediate command)
        GetAngle获取机器人当前位姿的关节坐标（立即指令）"""
        return self._send_cmd("GetAngle()")

    def GetErrorID(self) -> str:
        """GetErrorID gets the current robot error code (immediate command)
        GetErrorID获取机器人当前报错的错误码（立即指令）"""
        return self._send_cmd("GetErrorID()")

    def GetScrName(self) -> str:
        """GetScrName gets the name of the script currently running on the robot (immediate command)
        GetScrName获取当前机器人正在运行的脚本名称（立即指令）"""
        return self._send_cmd("GetScrName()")

    # ==================== 运动学计算 / Kinematics calculation ====================

    def PositiveKin(self, joints: Sequence[float], user: int = -1, tool: int = -1) -> str:
        """
        Compute forward kinematics (immediate command)
        PositiveKin进行正解运算（立即指令）

        Args:
            joints: 6 joint angles [j1,j2,j3,j4,j5,j6] (deg) / 6个关节角度[j1,j2,j3,j4,j5,j6] (°)
            user: User coordinate system index. Default is -1, meaning the current user coordinate system.
                  用户坐标系编号。默认为-1，表示当前用户坐标系。
            tool: Tool coordinate system index. Default is -1, meaning the current tool coordinate system.
                  工具坐标系编号。默认为-1，表示当前工具坐标系。
        """
        if len(joints) != 6:
            raise ValueError("joints must contain six joint angles")  # joints 必须包含 6 个关节角度
        joint_str = ",".join([f"{j:.6f}" for j in joints])
        if user != -1 and tool != -1:
            return self._send_cmd(f"PositiveKin({joint_str},user={user},tool={tool})")
        return self._send_cmd(f"PositiveKin({joint_str})")

    def InverseKin(self, pose: Sequence[float], use_joint_near: int = 0, joint_near: Sequence[float] = None, user: int = -1, tool: int = -1) -> str:
        """
        Compute inverse kinematics (immediate command)
        InverseKin进行逆解运算（立即指令）

        Args:
            pose: 6 Cartesian coordinates [x,y,z,rx,ry,rz] / 6个笛卡尔坐标 [x,y,z,rx,ry,rz]
            use_joint_near: Reference for selecting the nearest inverse kinematics solution.
                            0 uses the robot's current joint angles. 1 uses joint_near if supplied.
                            就近选解所用的参考关节角。0 使用机器人当前关节角，1 使用传入的 joint_near。
            joint_near: Reference joint angles [j1,j2,j3,j4,j5,j6] for selecting the nearest solution when use_joint_near is 1.
                        use_joint_near 为 1 时，用于就近选解的参考关节角 [j1,j2,j3,j4,j5,j6]。
            user: User coordinate system index. Default is -1, meaning the current user coordinate system.
                  用户坐标系编号。默认为-1，表示当前用户坐标系。
            tool: Tool coordinate system index. Default is -1, meaning the current tool coordinate system.
                  工具坐标系编号。默认为-1，表示当前工具坐标系。
        """
        if len(pose) != 6:
            raise ValueError("pose must contain six values")  # pose 必须包含 6 个位姿分量
        pose_str = ",".join([f"{p:.6f}" for p in pose])
        params = [pose_str]
        if use_joint_near != 0:
            params.append(f"useJointNear={use_joint_near}")
            if joint_near is not None:
                if len(joint_near) != 6:
                    raise ValueError("joint_near requires 6 joint angles")  # joint_near需要6个关节角度 / joint_near requires 6 joint angles
                joint_near_str = "jointNear={" + ",".join([f"{j:.6f}" for j in joint_near]) + "}"
                params.append(joint_near_str)
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        return self._send_cmd(f"InverseKin({','.join(params)})")

    # ==================== 可达性检测 / Reachability check ====================

    def CheckOddMovL(self, p1: Sequence[float], p2: Sequence[float],
                     point_type: str = "joint",
                     user: int = -1, tool: int = -1,
                     a: float = -1, v: float = -1,
                     cp: float = None, r: float = None) -> str:
        """
        CheckOddMovL checks the point reachability of linear motion (immediate command)
        CheckOddMovL检查直线运动的点位可达性（立即指令）

        Args:
            p1: Start point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 起点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            p2: End point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 终点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            point_type: Point type "joint" or "pose" (Cartesian) / 点位类型 "joint"（关节）或 "pose"（笛卡尔）
            user: User coordinate system index. -1 means unspecified / 用户坐标系编号。-1 表示不指定
            tool: Tool coordinate system index. -1 means unspecified / 工具坐标系编号。-1 表示不指定
            a: Acceleration. -1 means use default value / 加速度。-1 表示使用默认值
            v: Speed. -1 means use default value / 速度。-1 表示使用默认值
            cp: Continuity (mutually exclusive with r) / 连续度（与 r 二选一）
            r: Blend radius (mutually exclusive with cp, unit mm) / 融合半径（与 cp 二选一，单位mm）
        """
        if len(p1) != 6:
            raise ValueError("p1 requires 6 values")  # p1需要6个值 / p1 requires 6 values
        if len(p2) != 6:
            raise ValueError("p2 requires 6 values")  # p2需要6个值 / p2 requires 6 values
        if point_type not in ("joint", "pose"):
            raise ValueError("point_type can only be 'joint' or 'pose'")  # point_type 只能是 'joint' 或 'pose' / point_type can only be 'joint' or 'pose'
        p_values1 = ",".join([f"{j:.6f}" for j in p1])
        p_values2 = ",".join([f"{j:.6f}" for j in p2])
        params = [f"{point_type}={{{p_values1}}}", f"{point_type}={{{p_values2}}}"]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a:.6f}")
        if v != -1:
            params.append(f"v={v:.6f}")
        if cp is not None:
            params.append(f"cp={cp:.6f}")
        elif r is not None:
            params.append(f"r={r:.6f}")
        return self._send_cmd(f"CheckOddMovL({','.join(params)})")

    def CheckOddMovJ(self, p1: Sequence[float], p2: Sequence[float],
                     point_type: str = "joint",
                     a: float = -1, v: float = -1,
                     cp: float = None) -> str:
        """
        CheckOddMovJ checks the point reachability of joint motion (immediate command)
        CheckOddMovJ检查关节运动的点位可达性（立即指令）

        Args:
            p1: Start point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 起点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            p2: End point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 终点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            point_type: Point type "joint" or "pose" (Cartesian) / 点位类型 "joint"（关节）或 "pose"（笛卡尔）
            a: Acceleration. -1 means use default value / 加速度。-1 表示使用默认值
            v: Speed. -1 means use default value / 速度。-1 表示使用默认值
            cp: Continuity / 连续度
        """
        if len(p1) != 6:
            raise ValueError("p1 requires 6 values")  # p1需要6个值 / p1 requires 6 values
        if len(p2) != 6:
            raise ValueError("p2 requires 6 values")  # p2需要6个值 / p2 requires 6 values
        if point_type not in ("joint", "pose"):
            raise ValueError("point_type can only be 'joint' or 'pose'")  # point_type 只能是 'joint' 或 'pose' / point_type can only be 'joint' or 'pose'
        p_values1 = ",".join([f"{j:.6f}" for j in p1])
        p_values2 = ",".join([f"{j:.6f}" for j in p2])
        params = [f"{point_type}={{{p_values1}}}", f"{point_type}={{{p_values2}}}"]
        if a != -1:
            params.append(f"a={a:.6f}")
        if v != -1:
            params.append(f"v={v:.6f}")
        if cp is not None:
            params.append(f"cp={cp:.6f}")
        return self._send_cmd(f"CheckOddMovJ({','.join(params)})")

    def CheckOddMovC(self, p1: Sequence[float], p2: Sequence[float], p3: Sequence[float],
                     point_type: str = "joint",
                     user: int = -1, tool: int = -1,
                     a: float = -1, v: float = -1,
                     cp: float = None, r: float = None) -> str:
        """
        CheckOddMovC checks the point reachability of arc motion (immediate command)
        CheckOddMovC检查圆弧运动的点位可达性（立即指令）

        Args:
            p1: Start point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 起点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            p2: Intermediate point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 中间点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            p3: End point [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz] / 终点 [j1,j2,j3,j4,j5,j6] 或 [x,y,z,rx,ry,rz]
            point_type: Point type "joint" or "pose" (Cartesian) / 点位类型 "joint"（关节）或 "pose"（笛卡尔）
            user: User coordinate system index. -1 means unspecified / 用户坐标系编号。-1 表示不指定
            tool: Tool coordinate system index. -1 means unspecified / 工具坐标系编号。-1 表示不指定
            a: Acceleration. -1 means use default value / 加速度。-1 表示使用默认值
            v: Speed. -1 means use default value / 速度。-1 表示使用默认值
            cp: Continuity (mutually exclusive with r) / 连续度（与 r 二选一）
            r: Blend radius (mutually exclusive with cp, unit mm) / 融合半径（与 cp 二选一，单位mm）
        """
        if len(p1) != 6:
            raise ValueError("p1 requires 6 values")  # p1需要6个值 / p1 requires 6 values
        if len(p2) != 6:
            raise ValueError("p2 requires 6 values")  # p2需要6个值 / p2 requires 6 values
        if len(p3) != 6:
            raise ValueError("p3 requires 6 values")  # p3需要6个值 / p3 requires 6 values
        if point_type not in ("joint", "pose"):
            raise ValueError("point_type can only be 'joint' or 'pose'")  # point_type 只能是 'joint' 或 'pose' / point_type can only be 'joint' or 'pose'
        p_values1 = ",".join([f"{j:.6f}" for j in p1])
        p_values2 = ",".join([f"{j:.6f}" for j in p2])
        p_values3 = ",".join([f"{j:.6f}" for j in p3])
        params = [f"{point_type}={{{p_values1}}}",
                  f"{point_type}={{{p_values2}}}",
                  f"{point_type}={{{p_values3}}}"]
        if user != -1:
            params.append(f"user={user}")
        if tool != -1:
            params.append(f"tool={tool}")
        if a != -1:
            params.append(f"a={a:.6f}")
        if v != -1:
            params.append(f"v={v:.6f}")
        if cp is not None:
            params.append(f"cp={cp:.6f}")
        elif r is not None:
            params.append(f"r={r:.6f}")
        return self._send_cmd(f"CheckOddMovC({','.join(params)})")

    # ==================== 托盘相关 / Tray related ====================

    def CreateTray(self, name: str, dimensions: Sequence[int],
                    points: list) -> str:
        """
        CreateTray creates a tray (immediate command). Supports 1D, 2D, and 3D trays.
        CreateTray创建托盘（立即指令）。支持一维、二维、三维托盘。

        Prototype (consistent with documentation):
        原型（与文档完全一致）：
        CreateTray(Trayname, {Count}, {P1},{P2}) -- 1D tray / 一维托盘
        CreateTray(Trayname, {row,col}, {P1},{P2},{P3},{P4}) -- 2D tray / 二维托盘
        CreateTray(Trayname, {row,col,layer}, {P1},{P2},{P3},{P4},{P5},{P6},{P7},{P8}) -- 3D tray / 三维托盘

        Args:
            name: Tray name (string up to 32 bytes, names consisting only of digits or spaces not allowed)
                  托盘名称（最长32字节字符串，不允许纯数字或纯空格）
            dimensions: Dimension parameters / 维度参数
                - 1D: [count] number of points [2,50] / 一维: [count] 点位数量[2,50]
                - 2D: [row, col] number of rows and columns / 二维: [row, col] 行数和列数
                - 3D: [row, col, layer] number of rows, columns, and layers / 三维: [row, col, layer] 行数、列数、层数
            points: List of endpoints, each endpoint is in [x,y,z,rx,ry,rz] format, each point passed as an independent parameter
                    端点列表，每个端点为[x,y,z,rx,ry,rz]格式，每个点位作为独立参数
                - 1D: 2 endpoints [p1, p2] / 一维: 2个端点 [p1, p2]
                - 2D: 4 endpoints [p1, p2, p3, p4] / 二维: 4个端点 [p1, p2, p3, p4]
                - 3D: 8 endpoints [p1, p2, p3, p4, p5, p6, p7, p8] / 三维: 8个端点 [p1, p2, p3, p4, p5, p6, p7, p8]
        """
        dims = ",".join([str(d) for d in dimensions])
        point_strs = []
        for p in points:
            if len(p) != 6:
                raise ValueError(f"each point requires 6 values [x,y,z,rx,ry,rz], currently {len(p)} passed")  # 每个点位需要6个值[x,y,z,rx,ry,rz]，当前传入{len(p)}个 / each point requires 6 values [x,y,z,rx,ry,rz], currently {len(p)} passed
            point_strs.append("{pose = {" + ",".join([f"{v:.6f}" for v in p]) + "}}")
        cmd = f"CreateTray({name},{{{dims}}}," + ",".join(point_strs) + ")"
        return self._send_cmd(cmd)

    def GetTrayPoint(self, trayname: str, index: int) -> str:
        """
        GetTrayPoint gets a tray point (immediate command)
        GetTrayPoint获取托盘点（立即指令）

        Args:
            trayname: Tray name (string, corresponds to the name used when CreateTray was created)
                      托盘名称（字符串，与 CreateTray 创建时的 name 对应）
            index: Tray point index (starting from which one, 1-based)
                   托盘点位序号（从第几个开始，1-based）
        """
        if not trayname or not trayname.strip():
            raise ValueError("trayname cannot be empty")  # trayname 不能为空 / trayname cannot be empty
        if index < 1:
            raise ValueError("index must be greater than or equal to 1")  # index 必须大于等于 1 / index must be greater than or equal to 1
        return self._send_cmd(f"GetTrayPoint({trayname},{index})")

    # ==================== 日志导出 / Log export ====================

    def LogExportUSB(self, log_range: int = None) -> str:
        """
        LogExportUSB exports robot logs to a USB drive (immediate command)
        LogExportUSB将机器人日志导出至U盘（立即指令）

        Args:
            log_range: Log export range. / 日志导出范围。
                0: Export logs/all and logs/user / 导出 logs/all 和 logs/user
                1: Export all contents of the logs folder / 导出 logs 文件夹所有内容
        """
        if log_range is not None:
            if log_range not in [0, 1]:
                raise ValueError("log_range must be 0 or 1")  # log_range必须是0或1 / log_range must be 0 or 1
            return self._send_cmd(f"LogExportUSB({log_range})")
        return self._send_cmd("LogExportUSB()")

    def GetExportStatus(self) -> str:
        """GetExportStatus gets the log export status (immediate command)
        GetExportStatus获取日志导出状态（立即指令）"""
        return self._send_cmd("GetExportStatus()")
