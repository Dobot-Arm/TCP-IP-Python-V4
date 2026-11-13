# Dobot TCP-IP Python SDK API 文档（完整版）

## 目录

1. [概述](#概述)
2. [快速开始](#快速开始)
3. [DobotRobot 主控制类](#dobotrobot-主控制类)
4. [Motion 运动模块](#motion-运动模块)
5. [IO 数字IO模块](#io-数字io模块)
6. [RobotControl 机器人控制模块](#robotcontrol-机器人控制模块)
7. [Communication 通讯模块](#communication-通讯模块)
8. [Plugins 插件模块](#plugins-插件模块)
9. [状态反馈系统](#状态反馈系统)
10. [错误码处理](#错误码处理)
11. [完整示例](#完整示例)
12. [附录](#附录)

***

## 概述

### 关于本SDK

Dobot TCP-IP Python SDK 是越疆机器人官方提供的二次开发接口，基于TCP/IP协议实现与机器人的通信和控制。

### 支持的机器人型号

- CR3/CR5/CR10 系列协作机器人

### 协议版本

- 支持 Dobot TCP/IP 协议 V4.6.6

### 架构设计

```
DobotRobot (主类)
├── motion      - 运动控制模块
├── io          - 数字IO模块
├── robot_control - 机器人控制模块
├── communication - 通讯模块
└── plugins     - 插件模块
```

***

## 快速开始

### 环境要求

- Python 3.7+
- 机器人固件版本支持 TCP/IP 协议 V4.6.6

### 安装方式

```bash
# 源码安装
cd dobot_sdk
pip install -e .
```

### 基础示例

```python
from dobot_sdk import DobotRobot, CoordinateType

# 连接机器人
with DobotRobot("192.168.1.100") as robot:
    # 请求控制
    robot.robot_control.RequestControl()
    
    # 清除报警
    robot.robot_control.ClearError()
    
    # 使能机器人
    robot.robot_control.EnableRobot()
    
    # 关节运动到初始位置
    robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)
    
    # 笛卡尔运动
    robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    
    # 关闭DO1
    robot.io.DO(1, 0)
    
    # 获取当前位姿
    pose = robot.robot_control.GetPose()
    print(f"当前位姿: {pose}")
```

***

## DobotRobot 主控制类

### 初始化

```python
DobotRobot(ip: str, dashboard_port: int = 29999, feedback_port: int = 30004)
```

**参数说明**:

| 参数              | 类型  | 默认值   | 说明                |
| --------------- | --- | ----- | ----------------- |
| ip              | str | -     | 机器人IP地址           |
| dashboard\_port | int | 29999 | Dashboard端口（命令控制） |
| feedback\_port  | int | 30004 | Feedback端口（状态反馈）  |

**示例**:

```python
# 使用默认端口
robot = DobotRobot("192.168.1.100")

# 自定义端口
robot = DobotRobot("192.168.1.100", dashboard_port=29999, feedback_port=30004)
```

### 连接管理

#### Connect()

建立与机器人的连接。

```python
robot.Connect(timeout: float = 5.0) -> None
```

**参数**:

| 参数      | 类型    | 默认值 | 说明        |
| ------- | ----- | --- | --------- |
| timeout | float | 5.0 | 连接超时时间（秒） |

**示例**:

```python
robot.Connect()
```

#### Disconnect()

断开与机器人的连接。

```python
robot.Disconnect() -> None
```

**示例**:

```python
robot.Disconnect()
```

### 模块接口说明

> **注意**: 为保持接口清晰，DobotRobot主类不再提供快捷控制方法，请通过对应模块调用：
> - 机器人控制: `robot.robot_control.EnableRobot()` / `robot.robot_control.DisableRobot()` / `robot.robot_control.ClearError()`
> - 运动控制: `robot.motion.MovJ()` / `robot.motion.MovL()` / `robot.motion.Arc()`
> - IO控制: `robot.io.DO()` / `robot.io.DI()` / `robot.io.AO()`
> - 通讯控制: `robot.communication.ModbusCreate()`
> - 插件控制: `robot.plugins.FCForceMode()`

详细接口请参考各模块章节。

### 状态监控

#### StartFeedbackMonitor()

启动状态反馈监控线程。

```python
robot.StartFeedbackMonitor(callback: Callable = None) -> None
```

**参数**:

| 参数       | 类型       | 说明                         |
| -------- | -------- | -------------------------- |
| callback | Callable | 状态更新回调函数，接收 RobotStatus 参数 |

**示例**:

```python
def on_status_update(status):
    print(f"当前位置: X={status.tool_vector_actual.x}")

robot.StartFeedbackMonitor(callback=on_status_update)
```

#### StopFeedbackMonitor()

停止状态反馈监控。

```python
robot.StopFeedbackMonitor() -> None
```

**示例**:

```python
robot.StopFeedbackMonitor()
```

#### GetStatus()

获取当前状态。

```python
robot.GetStatus() -> Optional[RobotStatus]
```

**返回值**: RobotStatus 对象

**示例**:

```python
status = robot.GetStatus()
if status:
    print(f"速度比例: {status.speed_scaling}%")
    print(f"机器人模式: {status.robot_mode.value}")
```

### 上下文管理器支持

```python
with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    # 执行操作
# 自动断开连接
```

***

## Motion 运动模块

通过 `robot.motion` 访问。

### 坐标系类型

```python
from dobot_sdk import CoordinateType

CoordinateType.CARTESIAN  # 笛卡尔坐标 (x, y, z, rx, ry, rz)
CoordinateType.JOINT      # 关节角度 (j1, j2, j3, j4, j5, j6)
```

### 基础运动指令

#### MovJ() - 关节运动

```python
motion.MovJ(pose, coord_type, user=-1, tool=-1, a=-1, v=-1, cp=-1) -> str
```

**参数**:

| 参数          | 类型             | 默认值 | 说明            |
| ----------- | -------------- | --- | ------------- |
| pose        | list\[float]   | -   | 6个坐标值         |
| coord\_type | CoordinateType | -   | 坐标系类型         |
| user        | int            | -1  | 用户坐标系编号（0-50） |
| tool        | int            | -1  | 工具坐标系编号（0-50） |
| a           | int            | -1  | 加速度比例（1-100）  |
| v           | int            | -1  | 速度比例（1-100）   |
| cp          | int            | -1  | 平滑过渡比例（0-100） |

**示例**:

```python
# 关节运动
robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)

# 笛卡尔运动，带参数
robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN, v=50)
```

#### MovL() - 直线运动

```python
motion.MovL(pose, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1) -> str
```

**参数**:

| 参数    | 类型  | 默认值 | 说明               |
| ----- | --- | --- | ---------------- |
| speed | int | -1  | 目标速度（mm/s），与v互斥  |
| r     | int | -1  | 平滑过渡半径（mm），与cp互斥 |

**示例**:

```python
robot.motion.MovL([500, 100, 200, 180, 0, 0], CoordinateType.CARTESIAN, speed=100)
```

#### MovJIO() - 关节运动并输出DO

```python
motion.MovJIO(pose, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, cp=-1) -> str
```

**参数**:

| 参数     | 类型             | 说明                              |
| ------ | -------------- | ------------------------------- |
| do_list | list | DO输出列表，每个元素为[do_index, do_status]，支持多个DO同时输出 |

**示例**:

```python
# 运动到目标点，同时打开DO1和DO2
robot.motion.MovJIO([400, 0, 300, 180, 0, 0], [[1, 1], [2, 1]], CoordinateType.CARTESIAN)
```

#### MovLIO() - 直线运动并输出DO

```python
motion.MovLIO(pose, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1) -> str
```

**示例**:

```python
# 直线运动到目标点，关闭DO1
robot.motion.MovLIO([500, 0, 300, 180, 0, 0], [[1, 0]], CoordinateType.CARTESIAN)
```

#### Arc() - 圆弧插补运动

```python
motion.Arc(p1, p2, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**参数**:

| 参数   | 类型           | 默认值 | 说明                       |
| ---- | ------------ | --- | ------------------------ |
| p1   | list\[float] | -   | 圆弧中间点位姿                  |
| p2   | list\[float] | -   | 目标点位姿                    |
| mode | int          | 0   | 姿态控制模式（0-线性，1-过中间点，2-固定） |

**示例**:

```python
mid_point = [400, 100, 300, 180, 0, 0]
end_point = [400, 200, 300, 180, 0, 0]
robot.motion.Arc(mid_point, end_point, CoordinateType.CARTESIAN)
```

#### ArcIO() - 圆弧运动并输出DO

```python
motion.ArcIO(p1, p2, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**示例**:

```python
mid_point = [400, 100, 300, 180, 0, 0]
end_point = [400, 200, 300, 180, 0, 0]
# 圆弧运动到终点时打开DO1
robot.motion.ArcIO(mid_point, end_point, [[1, 1]], CoordinateType.CARTESIAN)
```

#### Circle() - 整圆插补运动

```python
motion.Circle(p1, p2, count, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**参数**:

| 参数    | 类型  | 说明        |
| ----- | --- | --------- |
| count | int | 圈数（1-999） |

**示例**:

```python
# 沿同一圆形轨迹运动两圈
robot.motion.Circle(mid_point, end_point, 2, CoordinateType.CARTESIAN)
```

### 伺服运动

#### ServoJ() - 关节空间动态跟随

```python
motion.ServoJ(joints, t=0.1, aheadtime=50.0, gain=500.0) -> str
```

**参数**:

| 参数        | 类型    | 默认值   | 说明                   |
| --------- | ----- | ----- | -------------------- |
| joints    | Sequence[float] | -     | 关节角度列表 [j1,j2,j3,j4,j5,j6]（度） |
| t         | float | 0.1   | 运行时间（秒，0.004-3600.0） |
| aheadtime | float | 50.0  | 提前量（20.0-100.0）      |
| gain      | float | 500.0 | 比例增益（200.0-1000.0）   |

**示例**:

```python
import time

# 实时跟随示例
for i in range(100):
    j1 = i * 0.5
    robot.motion.ServoJ([j1, 0, 90, 0, 90, 0], t=0.05)
    time.sleep(0.05)
```

#### ServoP() - 笛卡尔空间动态跟随

```python
motion.ServoP(pose, t=0.1, aheadtime=50.0, gain=500.0) -> str
```

**示例**:

```python
robot.motion.ServoP([400 + i*2, 0, 300, 180, 0, 0], t=0.05)
```

### 点动控制

#### MoveJog() - 点动机械臂

```python
motion.MoveJog(axis="", coord_type=CoordinateType.JOINT, user=0, tool=0) -> str
```

**参数**:

| 参数   | 类型  | 默认值 | 说明                                                        |
| ---- | --- | --- | --------------------------------------------------------- |
| axis | str | ""  | "X+", "X-", "Y+", "Y-", "Z+", "Z-", "J1+", "J1-" 等，空字符串停止 |

**示例**:

```python
# 启动点动
robot.motion.MoveJog("X+", CoordinateType.CARTESIAN)

# 停止点动
robot.motion.MoveJog()
```

### 相对运动

#### RelMovJTool() - 工具坐标系相对关节运动

```python
motion.RelMovJTool(offset, v=-1) -> str
```

**示例**:

```python
# 沿工具坐标系移动
robot.motion.RelMovJTool([0, 0, 0, 0, 0, 30], v=50)
```

#### RelMovLTool() - 工具坐标系相对直线运动

```python
motion.RelMovLTool(offset, v=-1, r=-1) -> str
```

**示例**:

```python
# 沿X方向移动50mm
robot.motion.RelMovLTool([50, 0, 0, 0, 0, 0], v=50)
```

#### RelMovJUser() - 用户坐标系相对关节运动

```python
motion.RelMovJUser(offset, v=-1) -> str
```

**示例**:

```python
# 在用户坐标系中相对移动
robot.motion.RelMovJUser([0, 50, 0, 0, 0, 0], v=50)
```

#### RelMovLUser() - 用户坐标系相对直线运动

```python
motion.RelMovLUser(offset, v=-1, r=-1) -> str
```

**示例**:

```python
# 在用户坐标系中相对直线移动
robot.motion.RelMovLUser([50, 0, 0, 0, 0, 0], v=50)
```

#### RelJointMovJ() - 关节坐标系相对运动

```python
motion.RelJointMovJ(offset, v=-1) -> str
```

**示例**:

```python
# J1轴相对旋转30度
robot.motion.RelJointMovJ([30, 0, 0, 0, 0, 0], v=30)
```

### 轨迹复现

#### MovS() - 拟合导入轨迹

```python
# 方式1：点位列表方式（4~50个点位）
motion.MovS([p1, p2, p3, ...], coord_type=CoordinateType.CARTESIAN, freq=-1, user=-1, tool=-1, a=-1, v=-1, speed=-1) -> str

# 方式2：文件方式
motion.MovS("trajectory.csv", coord_type=CoordinateType.CARTESIAN, freq=-1, user=-1, tool=-1, a=-1, v=-1, speed=-1) -> str
```

**参数**:

| 参数              | 类型                          | 默认值   | 说明                                                            |
| --------------- | --------------------------- | ----- | ------------------------------------------------------------- |
| trace\_or\_points | str \| Sequence[Sequence[float]] | -     | str=轨迹文件名（含后缀）；Sequence=4~50个点位，每个点位为 [x,y,z,rx,ry,rz] 或 [j1..j6] |
| coord\_type     | CoordinateType              | CARTESIAN | 点位坐标系类型。仅点位列表方式时使用：CARTESIAN(笛卡尔)、JOINT(关节)          |
| freq            | float                       | -1    | 滤波系数（0~1，1=关闭滤波；-1=不设置）                                       |
| user            | int                         | -1    | 用户坐标系编号（-1=当前）                                                 |
| tool            | int                         | -1    | 工具坐标系编号（-1=当前）                                                 |
| a               | int                         | -1    | 加速度比例（1~100；-1=全局）                                              |
| v               | int                         | -1    | 速度比例（1~100；-1=全局，与 speed 互斥，speed 优先）                              |
| speed           | int                         | -1    | 目标速度 (mm/s)，与 v 互斥（优先 speed）                                     |

> **说明**：`isConst/multi/sample` 等参数属于 `StartPath()` 轨迹复现指令，`MovS()` 拟合指令**不使用**这些参数。

**示例**:

```python
# 方式1：4个点位拟合（笛卡尔坐标）
points = [
    [100, 0, 100, 0, 0, 0],
    [100, 20, 100, 0, 0, 0],
    [100, 30, 100, 0, 0, 0],
    [100, 40, 100, 0, 0, 0],
]
robot.motion.MovS(points, v=50, freq=0.2)

# 方式2：文件拟合
robot.motion.MovS("trajectory.csv", v=30)
```

#### StartPath() - 复现录制轨迹

```python
motion.StartPath(trace_name, is_const=0, multi=1.0, sample=50, freq=0.2, user=-1, tool=-1) -> str
```

**示例**:

```python
robot.motion.StartPath("recorded_trace.csv", is_const=1, multi=0.8)
```

#### GetStartPose() - 获取轨迹起点

```python
motion.GetStartPose(trace_name, path_type=1) -> str
```

**参数**:

| 参数         | 类型  | 默认值 | 说明                  |
| ---------- | --- | --- | ------------------- |
| path\_type | int | 1   | 轨迹类型（1-复现轨迹，2-拟合轨迹） |

**示例**:

```python
# 获取拟合轨迹的起点
response = robot.motion.GetStartPose("trajectory.csv", path_type=2)
print(f"轨迹起点: {response}")
```

### 坐标系偏移

#### StartRTOffset() - 启动坐标系偏移

```python
motion.StartRTOffset(offset) -> str

**示例**:

```python
# 启动偏移
robot.motion.StartRTOffset([10, 10, 0, 0, 0, 0])

# 执行运动（带偏移）
robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)

# 结束偏移
robot.motion.EndRTOffset()
```

#### EndRTOffset() - 结束坐标系偏移

```python
motion.EndRTOffset() -> str
```

#### OffsetPara() - 设置偏移参数

```python
motion.OffsetPara(freq=0.2) -> str
```

**示例**:

```python
# 设置偏移滤波系数
robot.motion.OffsetPara(freq=0.5)
```

### 轨迹恢复

#### SetResumeOffset() - 设置恢复回退距离

```python
motion.SetResumeOffset(distance) -> str
```

**示例**:

```python
robot.motion.SetResumeOffset(50)  # 回退50mm
```

#### PathRecovery() - 开始轨迹恢复

```python
motion.PathRecovery() -> str
```

**示例**:

```python
# 设置回退距离后开始恢复
robot.motion.SetResumeOffset(50)
robot.motion.PathRecovery()
```

#### PathRecoveryStop() - 停止轨迹恢复

```python
motion.PathRecoveryStop() -> str
```

**示例**:

```python
robot.motion.PathRecoveryStop()
```

#### PathRecoveryStatus() - 查询恢复状态

```python
motion.PathRecoveryStatus() -> str
```

**返回值**:

- 0: 已回到暂停位姿
- 1: 偏差较小
- 2: 偏差较大

**示例**:

```python
status = robot.motion.PathRecoveryStatus()
print(f"恢复状态: {status}")
```

### 速度加速度设置（robot_control模块）

#### SpeedFactor() - 设置全局速度比例

```python
robot_control.SpeedFactor(speed) -> str
```

**参数**:

| 参数    | 类型  | 说明            |
| ----- | --- | ------------- |
| speed | int | 全局运动速度比例，取值范围：\[1, 100] |

**示例**:

```python
robot.robot_control.SpeedFactor(50)  # 设置50%全局速度
```

#### AccJ() - 设置关节运动加速度比例

```python
robot_control.AccJ(acc) -> str
```

**参数**:

| 参数  | 类型  | 说明                     |
| --- | --- | ---------------------- |
| acc | int | 关节加速度比例，取值范围：\[1, 100] |

**示例**:

```python
# 设置关节运动加速度比例为30%
robot.robot_control.AccJ(30)
```

#### AccL() - 设置直线和弧线运动加速度比例

```python
robot_control.AccL(acc) -> str
```

**参数**:

| 参数  | 类型  | 说明                     |
| --- | --- | ---------------------- |
| acc | int | 加速度比例，取值范围：\[1, 100] |

**示例**:

```python
# 设置直线和弧线运动加速度比例为40%
robot.robot_control.AccL(40)
```

### 坐标系设置

#### SetUser() - 设置用户坐标系

```python
robot_control.SetUser(index, pose, type=None) -> str

**参数**:

| 参数    | 类型  | 说明                                                                 |
| ----- | --- | ------------------------------------------------------------------ |
| index | int | 用户坐标系编号（1-50）                                                      |
| pose  | Sequence[float] | 6个坐标 [x,y,z,rx,ry,rz]                                                  |
| type  | int | 是否使坐标系改动全局生效。0: 该命令修改的坐标系仅在当前工程运行中生效。1: 该命令修改的坐标系将会被控制器保存（可选） |

**示例**:

```python
# 设置用户坐标系1
robot.robot_control.SetUser(1, [100, 0, 0, 0, 0, 0])

# 设置用户坐标系1并保存到控制器
robot.robot_control.SetUser(1, [100, 0, 0, 0, 0, 0], type=1)
```

#### SetTool() - 设置工具坐标系

```python
robot_control.SetTool(index, pose, type=None) -> str

**参数**:

| 参数    | 类型  | 说明                                                                 |
| ----- | --- | ------------------------------------------------------------------ |
| index | int | 工具坐标系编号（1-50）                                                      |
| pose  | Sequence[float] | 6个坐标 [x,y,z,rx,ry,rz]                                                  |
| type  | int | 是否使坐标系改动全局生效。0: 该命令修改的坐标系仅在当前工程运行中生效。1: 该命令修改的坐标系将会被控制器保存（可选） |

**示例**:

```python
# 设置工具坐标系1（末端工具偏移）
robot.robot_control.SetTool(1, [0, 0, 100, 0, 0, 0])

# 设置工具坐标系1并保存到控制器
robot.robot_control.SetTool(1, [0, 0, 100, 0, 0, 0], type=1)
```

#### SetPayload() - 设置末端负载

```python
robot_control.SetPayload(load, center=None, preset_name=None) -> str

**参数**:

| 参数         | 类型    | 说明                      |
| ---------- | ----- | ----------------------- |
| load       | float | 负载重量（kg）               |
| center     | Sequence[float] | 负载重心坐标 [x,y,z]（mm）（可选） |
| preset_name | str   | 负载预设名称，用于保存当前配置供后续调用（可选） |

**示例**:

```python
# 方式一：直接设置参数
robot.robot_control.SetPayload(1.5, [0, 0, 50])

# 方式二：只设置重量
robot.robot_control.SetPayload(1.5)

# 方式三：设置负载并保存为预设
robot.robot_control.SetPayload(1.5, [0, 0, 50], "my_payload")
```

### 抱闸控制（robot_control模块）

#### BrakeControl() - 抱闸控制

```python
robot_control.BrakeControl(axis, status) -> str
```

**参数**:

| 参数     | 类型  | 说明                                            |
| ------ | --- | --------------------------------------------- |
| axis   | int | 关节轴序号，取值范围：\[1, 6]。1表示J1轴，2表示J2轴，以此类推      |
| status | int | 抱闸状态。0表示抱闸锁死（关节不可移动）。1表示松开抱闸（关节可移动）。 |

**示例**:

```python
robot.robot_control.BrakeControl(1, 1)  # 松开J1轴抱闸
```

### 运动控制（robot_control模块）

#### Stop() - 停止运动

```python
robot_control.Stop() -> str
```

**说明**:
该指令用于停止机器人当前已下发的运动指令队列，同时也可用于停止正在运行的脚本工程（兼容替代RunScript停止）。

**示例**:

```python
robot.robot_control.Stop()
```

#### Pause() - 暂停运动

```python
robot_control.Pause() -> str
```

**说明**:
暂停已下发的运动指令队列或者RunScript指令运行的工程。

**示例**:

```python
robot.robot_control.Pause()
```

#### Continue() - 继续运动

```python
robot_control.Continue() -> str
```

**说明**:
继续已暂停的运动指令队列或者RunScript指令运行的工程。

**示例**:

```python
robot.robot_control.Continue()
```

#### EmergencyStop() - 紧急停止

```python
robot_control.EmergencyStop(mode) -> str
```

**参数**:

| 参数    | 类型  | 说明                              |
| ----- | --- | ------------------------------- |
| mode  | int | 急停操作模式。1表示按下急停，0表示松开急停 |

**示例**:

```python
# 按下急停
robot.robot_control.EmergencyStop(1)

# 松开急停
robot.robot_control.EmergencyStop(0)
```

***

## IO 数字IO模块

通过 `robot.io` 访问。

### 数字输出

#### DO() - 设置数字输出（队列指令）

```python
io.DO(index, status, time=None) -> str
```

**参数**:

| 参数     | 类型    | 说明                                        |
| ------ | ----- | ----------------------------------------- |
| index  | int   | DO端口索引（1-64）                               |
| status | int   | 0-Off，1-On                                  |
| time   | float | 输出持续时间（秒，可选），仅在 `status` 为 1 时有效。时间到后，数字输出状态自动变为 0。     |

**示例**:

```python
robot.io.DO(1, 1)           # 打开DO1
robot.io.DO(1, 0)           # 关闭DO1
robot.io.DO(1, 1, 2.0)      # 打开DO1，持续2秒后自动关闭
```

#### DOInstant() - 设置数字输出（立即指令）

```python
io.DOInstant(index, status) -> str
```

**示例**:

```python
robot.io.DOInstant(1, 1)
```

#### GetDO() - 获取数字输出状态

```python
io.GetDO(index) -> str
```

**示例**:

```python
response = robot.io.GetDO(1)
print(response)  # ErrorID,{status},GetDO(1);
```

#### DOGroup() - 设置多个数字输出

```python
io.DOGroup(group_index, status) -> str
```

**参数**:

| 参数           | 类型       | 说明         |
| ------------ | -------- | ---------- |
| group\_index | int      | IO组索引（0-3） |
| status       | list/str | 端口状态列表或字符串 |

**示例**:

```python
# 设置第0组（DO1-DO4）
robot.io.DOGroup(0, [1, 0, 1, 0])
```

#### DOGroupDEC() - 十进制设置DO组

```python
io.DOGroupDEC(group_index, value) -> str
```

**示例**:

```python
robot.io.DOGroupDEC(0, 5)  # 二进制 0101
```

#### GetDOGroup() - 获取DO组状态

```python
io.GetDOGroup(group_index) -> str
```

**示例**:

```python
response = robot.io.GetDOGroup(0)
print(f"DO组状态: {response}")
```

### 数字输入

#### DI() - 获取数字输入状态

```python
io.DI(index) -> str
```

**参数**:

| 参数    | 类型  | 说明                                                                                                |
| ----- | --- | ------------------------------------------------------------------------------------------------- |
| index | int | DI端子的编号。取值范围：\[1, MAX]或\[100, 1000]。MAX代表当前控制柜的DI范围。当取值范围为\[100, 1000]时，需要有拓展IO模块的硬件支持。 |

**返回值格式**: `ErrorID,{value},DI(index);`
- value表示DI端子的状态，0为关闭，1为打开。

**示例**:

```python
response = robot.io.DI(1)
```

#### DIGroup() - 等待数字输入组

```python
io.DIGroup(group_index, status) -> str
```

**示例**:

```python
robot.io.DIGroup(0, [1, 1, 0, 0])
```

#### DIGroupDEC() - 获取DI组状态（十进制）

```python
io.DIGroupDEC(group_index) -> str
```

**示例**:

```python
response = robot.io.DIGroupDEC(0)
print(f"DI组状态(十进制): {response}")
```

### 模拟IO

#### AO() - 设置模拟输出

```python
io.AO(index, value) -> str
```

**参数**:

| 参数    | 类型    | 说明                  |
| ----- | ----- | ------------------- |
| index | int   | AO端口索引（1-2）         |
| value | float | 输出值（0-10V 对应 0-100） |

**示例**:

```python
robot.io.AO(1, 50)  # 输出5V
```

#### GetAO() - 获取模拟输出

```python
io.GetAO(index) -> str
```

**示例**:

```python
response = robot.io.GetAO(1)
print(f"AO1输出值: {response}")
```

#### AI() - 获取模拟输入

```python
io.AI(index) -> str
```

**参数**:

| 参数    | 类型  | 说明                                          |
| ----- | --- | ------------------------------------------- |
| index | int | AI端子的编号。取值范围：\[1, MAX]。不同控制柜的AI资源数量不一样。 |

**返回值格式**: `ErrorID,{value},AI(index);`

**示例**:

```python
response = robot.io.AI(1)
print(f"AI1输入值: {response}")
```

### 末端IO

#### ToolDO() - 设置末端数字输出

```python
io.ToolDO(index, status) -> str
```

**示例**:

```python
robot.io.ToolDO(1, 1)
```

#### ToolDI() - 获取末端数字输入

```python
io.ToolDI(index) -> str
```

**示例**:

```python
response = robot.io.ToolDI(1)
print(f"末端DI1状态: {response}")
```

#### ToolAI() - 获取末端模拟输入

```python
io.ToolAI(index) -> str
```

**示例**:

```python
response = robot.io.ToolAI(1)
print(f"末端AI1输入值: {response}")
```

***

## RobotControl 机器人控制模块

通过 `robot.robot_control` 访问。

### 控制模式

#### RequestControl() - 请求TCP控制模式

```python
robot_control.RequestControl() -> str
```

**示例**:

```python
robot.robot_control.RequestControl()
```

### 电源控制

#### PowerOn() - 机器人上电

```python
robot_control.PowerOn() -> str
```

**示例**:

```python
robot.robot_control.PowerOn()
```

#### DisableRobot() - 机器人下使能

```python
robot_control.DisableRobot() -> str
```

**示例**:

```python
robot.robot_control.DisableRobot()
```

### 状态查询

#### GetErrorID() - 获取错误码

```python
robot_control.GetErrorID() -> str
```

**返回值格式**: `ErrorID,{[error1,error2,...]},GetErrorID();`

**示例**:

```python
response = robot.robot_control.GetErrorID()
print(response)  # 0,{[1537,2048]},GetErrorID();
```

#### RobotMode() - 获取机器人模式

```python
robot_control.RobotMode() -> str
```

**返回值**:

| 值 | 模式  |
| - | --- |
| 1 | 初始化 |
| 2 | 手动  |
| 3 | 自动  |
| 4 | 远程  |
| 9 | 错误  |

**示例**:

```python
mode = robot.robot_control.RobotMode()
print(f"机器人模式: {mode}")
```

#### GetPose() - 获取当前位姿

```python
robot_control.GetPose(user=-1, tool=-1) -> str
```

**示例**:

```python
response = robot.robot_control.GetPose()
print(response)  # ErrorID,{x,y,z,rx,ry,rz},GetPose();
```

#### GetAngle() - 获取关节角度

```python
robot_control.GetAngle() -> str
```

**示例**:

```python
response = robot.robot_control.GetAngle()
print(response)  # ErrorID,{j1,j2,j3,j4,j5,j6},GetAngle();
```

#### 获取当前速度比例

当前速度比例以百分比表示，可通过 30004 端口实时反馈中的 `SpeedScaling` 字段获取。SDK 中可通过 `robot.GetStatus().speed_scaling` 读取该值。

**示例**:

```python
# 通过状态反馈获取速度比例
status = robot.GetStatus()
if status:
    print(f"当前速度比例: {status.speed_scaling}%")
```

### 运动学计算

#### InverseKin() - 逆解计算

```python
robot_control.InverseKin(pose, use_joint_near=0, joint_near=None, user=-1, tool=-1) -> str
```

**参数**:

| 参数          | 类型           | 说明                                               |
| ----------- | ------------ | ------------------------------------------------ |
| pose        | Sequence[float] | 6个笛卡尔坐标 [x,y,z,rx,ry,rz]                             |
| use_joint_near | int          | 是否使用关节接近度约束。0: 不使用。1: 使用                           |
| joint_near  | Sequence[float] | 关节接近度参考值 [j1,j2,j3,j4,j5,j6]，当use_joint_near为1时生效（可选） |
| user        | int          | 用户坐标系编号。默认为-1，表示当前用户坐标系（可选）                    |
| tool        | int          | 工具坐标系编号。默认为-1，表示当前工具坐标系（可选）                    |

**示例**:

```python
pose = [400, 0, 300, 180, 0, 0]
response = robot.robot_control.InverseKin(pose)

# 使用关节接近度约束
response = robot.robot_control.InverseKin(pose, use_joint_near=1, joint_near=[0, 0, 90, 0, 90, 0])
```

#### PositiveKin() - 正解计算

```python
robot_control.PositiveKin(joints, user=-1, tool=-1) -> str
```

**参数**:

| 参数    | 类型           | 说明                                   |
| ----- | ------------ | ------------------------------------ |
| joints | Sequence[float] | 6个关节角度 [j1,j2,j3,j4,j5,j6]                 |
| user  | int          | 用户坐标系编号。默认为-1，表示当前用户坐标系（可选）        |
| tool  | int          | 工具坐标系编号。默认为-1，表示当前工具坐标系（可选）        |

**示例**:

```python
joints = [0, 0, 90, 0, 90, 0]
response = robot.robot_control.PositiveKin(joints)
```

### 脚本控制

#### RunScript() - 运行脚本

```python
robot_control.RunScript(script_name) -> str
```

**示例**:

```python
robot.robot_control.RunScript("test.lua")
```

#### Stop() - 停止运动或停止脚本运行

```python
robot_control.Stop() -> str
```

**说明**:
该指令用于停止机器人当前的运动，同时也可用于停止正在运行的脚本工程（兼容替代RunScript停止）。

**示例**:

```python
# 停止当前运动或脚本
robot.robot_control.Stop()
```

### 日志导出

#### LogExportUSB() - 将机器人日志导出至U盘

```python
robot_control.LogExportUSB(range) -> str
```

**参数**:

| 参数    | 类型  | 说明                                                                  |
| ----- | --- | ------------------------------------------------------------------- |
| range | int | （必选）日志导出范围。 0：导出logs/all 和logs/user文件夹的内容。 1：导出logs文件夹所有内容。仅允许取值 0 或 1。 |

**返回值格式**: `ErrorID,{},LogExportUSB(range);`

**说明**:
- 导出日志时建议只插入一个U盘，避免导出失败。
- 如果U盘包含多个分区，日志会导出至第一个分区。
- 请勿在导出过程中拔出U盘，否则可能导致文件损坏。
- 该指令下发后立即返回，请通过 GetExportStatus 获取日志导出状态。

**示例**:

```python
# 导出logs/all 和 logs/user 文件夹的内容
robot.robot_control.LogExportUSB(0)

# 导出 logs 文件夹所有内容
robot.robot_control.LogExportUSB(1)
```

#### GetExportStatus() - 获取日志导出状态

```python
robot_control.GetExportStatus() -> str
```

**返回值格式**: `ErrorID,{status},GetExportStatus();`

**返回值 status 说明**:

| 值  | 状态                 |
| -- | ------------------ |
| 0  | 未开始导出              |
| 1  | 导出中                |
| 2  | 导出完成               |
| 3  | 导出失败，找不到U盘         |
| 4  | 导出失败，U盘空间不足        |
| 5  | 导出失败，导出过程中U盘被拔出     |

**说明**:
导出完成和导出失败的状态会保持到下次用户使用导出功能。

**示例**:

```python
import time

# 启动导出
robot.robot_control.LogExportUSB(0)

# 轮询导出状态
while True:
    status_resp = robot.robot_control.GetExportStatus()
    print(f"导出状态: {status_resp}")
    # 解析状态判断是否完成
    time.sleep(1)
```

***

## Communication 通讯模块

通过 `robot.communication` 访问。

### Modbus 主站

#### ModbusCreate() - 创建Modbus主站

```python
communication.ModbusCreate(ip, port, slave_id, is_rtu=0) -> str
```

**参数**:

| 参数        | 类型  | 默认值 | 说明              |
| --------- | --- | --- | --------------- |
| ip        | str | -   | 从站IP地址          |
| port      | int | -   | 从站端口            |
| slave\_id | int | -   | 从站ID            |
| is\_rtu   | int | 0   | 0-TCP模式，1-RTU模式 |

**示例**:

```python
response = robot.communication.ModbusCreate("192.168.1.10", 502, 1)
```

#### ModbusRTUCreate() - 创建RTU主站

```python
communication.ModbusRTUCreate(slave_id, baud, parity="E", data_bit=8, stop_bit=1) -> str
```

**参数**:

| 参数     | 类型  | 默认值 | 说明                      |
| ------ | --- | --- | ----------------------- |
| parity | str | "E" | "O"-奇校验，"E"-偶校验，"N"-无校验 |

**示例**:

```python
robot.communication.ModbusRTUCreate(1, 19200, "N", 8, 1)
```

#### ModbusClose() - 关闭Modbus连接

```python
communication.ModbusClose(index) -> str
```

**示例**:

```python
robot.communication.ModbusClose(0)
```

### 寄存器读写

#### GetInBits() - 读取触点寄存器

```python
communication.GetInBits(index, addr, count) -> str
```

**示例**:

```python
response = robot.communication.GetInBits(0, 0, 8)
print(f"触点寄存器: {response}")
```

#### GetCoils() - 读取线圈寄存器

```python
communication.GetCoils(index, addr, count) -> str
```

**示例**:

```python
response = robot.communication.GetCoils(0, 0, 8)
print(f"线圈寄存器: {response}")
```

#### SetCoils() - 写入线圈寄存器

```python
communication.SetCoils(index, addr, count, values) -> str
```

**示例**:

```python
robot.communication.SetCoils(0, 0, 4, [1, 0, 1, 0])
```

#### GetInRegs() - 读取输入寄存器

```python
communication.GetInRegs(index, addr, count) -> str
```

**示例**:

```python
response = robot.communication.GetInRegs(0, 0, 4)
print(f"输入寄存器: {response}")
```

#### GetHoldRegs() - 读取保持寄存器

```python
communication.GetHoldRegs(index, addr, count) -> str
```

**示例**:

```python
response = robot.communication.GetHoldRegs(0, 0, 4)
print(f"保持寄存器: {response}")
```

#### SetHoldRegs() - 写入保持寄存器

```python
communication.SetHoldRegs(index, addr, count, values) -> str
```

**示例**:

```python
robot.communication.SetHoldRegs(0, 0, 2, [100, 200])
```

#### SetSingleCoil() - 写入单个线圈寄存器（V4.6.6新增）

```python
communication.SetSingleCoil(index, addr, value) -> str
```

**参数**:

| 参数    | 类型  | 说明                     |
| ----- | --- | ---------------------- |
| index | int | Modbus主站索引（0-9）         |
| addr  | int | 线圈寄存器地址                 |
| value | int | 写入值，0-断开，1-接通            |

**示例**:

```python
# 接通地址0的单个线圈
robot.communication.SetSingleCoil(0, 0, 1)

# 断开地址5的单个线圈
robot.communication.SetSingleCoil(0, 5, 0)
```

#### SetSingleHoldReg() - 写入单个保持寄存器（V4.6.6新增）

```python
communication.SetSingleHoldReg(index, addr, val) -> str
```

**参数**:

| 参数    | 类型  | 说明                   |
| ----- | --- | -------------------- |
| index | int | Modbus主站索引（0-9）        |
| addr  | int | 保持寄存器地址              |
| val   | int | 写入的整数值（U16：16位无符号整数） |

**示例**:

```python
# 向地址1写入值200
robot.communication.SetSingleHoldReg(0, 1, 200)
```


***

## Plugins 插件模块

通过 `robot.plugins` 访问。

### 力传感器

#### EnableFTSensor() - 开启/关闭力传感器

```python
plugins.EnableFTSensor(status) -> str
```

**示例**:

```python
robot.plugins.EnableFTSensor(1)  # 开启
```

#### SixForceHome() - 力传感器回零

```python
plugins.SixForceHome() -> str
```

**示例**:

```python
robot.plugins.SixForceHome()
```

#### GetForce() - 获取力传感器数值

```python
plugins.GetForce(tool=-1) -> str
```

**返回值格式**: `ErrorID,{fx,fy,fz,frx,fry,frz},GetForce();`

**示例**:

```python
response = robot.plugins.GetForce()
```

### 力控拖拽模式

#### ForceDriveMode() - 进入力控拖拽模式

```python
plugins.ForceDriveMode(direction, user=-1) -> str
```

**参数**:

| 参数        | 类型            | 说明                                                                                                          |
| --------- | ------------- | ----------------------------------------------------------------------------------------------------------- |
| direction | Sequence[int] | 6个方向的拖拽开关 [x,y,z,rx,ry,rz]，0表示该方向不能拖拽，1表示该方向可以拖拽。例如：[1,1,1,1,1,1]表示所有方向均可拖拽 |
| user      | int           | （可选）用户坐标系编号，取值范围[0,50]，不指定时表示不参考用户坐标系                                                          |

**示例**:

```python
# 进入力控拖拽模式，所有方向均可拖拽，参考用户坐标系1
robot.plugins.ForceDriveMode([1, 1, 1, 1, 1, 1], user=1)

# 仅XYZ轴方向可拖拽
robot.plugins.ForceDriveMode([1, 1, 1, 0, 0, 0])
```

#### ForceDriveSpeed() - 设置拖拽速度

```python
plugins.ForceDriveSpeed(speed) -> str
```

**参数**:

| 参数    | 类型  | 说明          |
| ----- | --- | ----------- |
| speed | int | 速度比例（0-100） |

**示例**:

```python
robot.plugins.ForceDriveSpeed(50)
```

### 力控模式

#### FCForceMode() - 开启力控模式

```python
plugins.FCForceMode(direction, force, reference=0, user=-1, tool=-1) -> str
```

**参数**:

| 参数        | 类型            | 说明                                                                                                                                                                                   |
| --------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| direction | Sequence[int] | 6个方向的力控开关 [x,y,z,rx,ry,rz]，1表示开启，0表示关闭                                                                                                                                                      |
| force     | Sequence[float] | 目标力和目标力矩 `[fx, fy, fz, frx, fry, frz]`。平移方向的目标力范围为 [-200, 200] N，旋转方向的目标力矩范围为 [-12, 12] N·m。目标力为 0 时，机器人处于柔顺模式。                                                                                                                   |
| reference | int           | （可选）参考坐标系类型。0-工具坐标系（默认），1-用户坐标系                                                                                                                                                                        |
| user      | int           | （可选）用户坐标系编号(0-50)，默认为-1表示当前用户坐标系                                                                                                                                                                             |
| tool      | int           | （可选）工具坐标系编号(0-50)，默认为-1表示当前工具坐标系                                                                                                                                                                             |

**示例**:

```python
direction = [1, 1, 0, 0, 0, 0]  # X,Y方向开启力控
force = [100, 100, 0, 0, 0, 0]   # X,Y方向目标力100N
robot.plugins.FCForceMode(direction, force, reference=1, user=1)
```

#### FCSetDeviation() - 设置力控位移和姿态偏差

```python
plugins.FCSetDeviation(deviation, control_type=-1) -> str
```

**参数**:

| 参数           | 类型             | 说明                                                                                                                      |
| ------------ | -------------- | ----------------------------------------------------------------------------------------------------------------------- |
| deviation    | Sequence[float] | 6个方向的偏差 [x,y,z,rx,ry,rz]。位移偏差(mm)范围(0,1000]，默认100；姿态偏差(度)范围(0,360]，默认36                                                  |
| control_type | int            | （可选）控制类型。-1:默认值；0:超过阈值时报警；1:超过阈值时停止搜寻继续运动                                                                                   |

**示例**:

```python
# 设置位移偏差200mm，姿态偏差36度
robot.plugins.FCSetDeviation([200, 200, 200, 36, 36, 36])
```

#### FCSetForceLimit() - 设置最大力限制

```python
plugins.FCSetForceLimit(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                                   |
| -- | ------ | -------------------------------------------------------------------- |
| x  | float  | X方向最大力限制(N)，范围(0, 500]，默认500                                                      |
| y  | float  | Y方向最大力限制(N)，范围(0, 500]，默认500                                                      |
| z  | float  | Z方向最大力限制(N)，范围(0, 500]，默认500                                                      |
| rx | float  | Rx 轴力矩上限（N·m），范围(0, 50]，默认50                                                  |
| ry | float  | Ry 轴力矩上限（N·m），范围(0, 50]，默认50                                                  |
| rz | float  | Rz 轴力矩上限（N·m），范围(0, 50]，默认50                                                  |

**示例**:

```python
robot.plugins.FCSetForceLimit(500, 500, 500, 50, 50, 50)
```

#### FCSetMass() - 设置惯性系数

```python
plugins.FCSetMass(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                     |
| -- | ------ | ------------------------------------------------------ |
| x  | float  | X方向惯性系数(kg)，范围(0, 10000]，默认20                             |
| y  | float  | Y方向惯性系数(kg)，范围(0, 10000]，默认20                             |
| z  | float  | Z方向惯性系数(kg)，范围(0, 10000]，默认20                             |
| rx | float  | Rx方向惯性系数(kg·m²)，范围(0, 10000]，默认20                         |
| ry | float  | Ry方向惯性系数(kg·m²)，范围(0, 10000]，默认20                         |
| rz | float  | Rz方向惯性系数(kg·m²)，范围(0, 10000]，默认20                         |

**示例**:

```python
robot.plugins.FCSetMass(20, 20, 20, 20, 20, 20)
```

#### FCSetStiffness() - 设置弹性系数

```python
plugins.FCSetStiffness(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                         |
| -- | ------ | ---------------------------------------------------------- |
| x  | float  | X方向弹性系数(N/mm)，范围[0, 10000]，默认30                           |
| y  | float  | Y方向弹性系数(N/mm)，范围[0, 10000]，默认30                           |
| z  | float  | Z方向弹性系数(N/mm)，范围[0, 10000]，默认30                           |
| rx | float  | Rx方向弹性系数(N/m·deg)，范围[0, 10000]，默认30                      |
| ry | float  | Ry方向弹性系数(N/m·deg)，范围[0, 10000]，默认30                      |
| rz | float  | Rz方向弹性系数(N/m·deg)，范围[0, 10000]，默认30                      |

**示例**:

```python
robot.plugins.FCSetStiffness(30, 30, 30, 30, 30, 30)
```

#### FCSetDamping() - 设置阻尼系数

```python
plugins.FCSetDamping(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                           |
| -- | ------ | ------------------------------------------------------------ |
| x  | float  | X方向阻尼系数 (N·s/mm)，范围[0, 1000]，默认50                          |
| y  | float  | Y方向阻尼系数 (N·s/mm)，范围[0, 1000]，默认50                          |
| z  | float  | Z方向阻尼系数 (N·s/mm)，范围[0, 1000]，默认50                          |
| rx | float  | Rx方向阻尼系数(N·s/m·deg)，范围[0, 1000]，默认50                     |
| ry | float  | Ry方向阻尼系数(N·s/m·deg)，范围[0, 1000]，默认50                     |
| rz | float  | Rz方向阻尼系数(N·s/m·deg)，范围[0, 1000]，默认50                     |

**示例**:

```python
robot.plugins.FCSetDamping(50, 50, 50, 50, 50, 50)
```

#### FCSetForceSpeedLimit() - 设置力控调节速度

```python
plugins.FCSetForceSpeedLimit(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                        |
| -- | ------ | --------------------------------------------------------- |
| x  | float  | X方向力控调节速度(mm/s)，范围(0, 300]，默认20                              |
| y  | float  | Y方向力控调节速度(mm/s)，范围(0, 300]，默认20                              |
| z  | float  | Z方向力控调节速度(mm/s)，范围(0, 300]，默认20                              |
| rx | float  | Rx方向力控调节速度(deg/s)，范围(0, 90]，默认20                           |
| ry | float  | Ry方向力控调节速度(deg/s)，范围(0, 90]，默认20                           |
| rz | float  | Rz方向力控调节速度(deg/s)，范围(0, 90]，默认20                           |

**示例**:

```python
robot.plugins.FCSetForceSpeedLimit(20, 20, 20, 20, 20, 20)
```

#### FCSetForce() - 实时调整恒力和恒力矩

```python
plugins.FCSetForce(x, y, z, rx, ry, rz) -> str
```

**参数**:

| 参数 | 类型     | 说明                                                               |
| -- | ------ | ---------------------------------------------------------------- |
| x  | float  | X方向恒力值(N)，范围[-200, 200]                                              |
| y  | float  | Y方向恒力值(N)，范围[-200, 200]                                              |
| z  | float  | Z方向恒力值(N)，范围[-200, 200]                                              |
| rx | float  | Rx 轴恒力矩（N·m），范围[-12, 12]                                          |
| ry | float  | Ry 轴恒力矩（N·m），范围[-12, 12]                                          |
| rz | float  | Rz 轴恒力矩（N·m），范围[-12, 12]                                          |

**示例**:

```python
# 各平移轴的恒力设为 50 N，各旋转轴的恒力矩设为 10 N·m
robot.plugins.FCSetForce(50, 50, 50, 10, 10, 10)
```

#### FCOff() - 退出力控模式

```python
plugins.FCOff() -> str
```

**示例**:

```python
# 关闭力控模式
robot.plugins.FCOff()
```

### 传送带跟踪

#### CnvInit() - 开启传送带

```python
plugins.CnvInit(index) -> str
```

**参数**:

| 参数    | 类型  | 说明          |
| ----- | --- | ----------- |
| index | int | 传送带索引（1-3） |

**示例**:

```python
robot.plugins.CnvInit(1)
```

#### GetCnvObject() - 等待工件进入抓取区域

```python
plugins.GetCnvObject(obj_id) -> str
```

**参数**:

| 参数   | 类型  | 说明                          |
| ---- | --- | --------------------------- |
| obj_id | int | 工件类型，取值范围[0, 15]。0：不指定工件类型，获取最先进入队列的工件信息 |

**示例**:

```python
response = robot.plugins.GetCnvObject(0)
```

#### StartSyncCnv() - 开启传送带跟踪功能

```python
plugins.StartSyncCnv() -> str
```

**示例**:

```python
robot.plugins.StartSyncCnv()
```

#### CnvMovL() - 传送带跟随直线运动

```python
plugins.CnvMovL(index, pose, offset, mode) -> str
```

**参数**:

| 参数     | 类型           | 说明                                                                 |
| ------ | ------------ | ------------------------------------------------------------------ |
| index  | int          | 传送带编号。取值范围：0~7。                                                     |
| pose   | Sequence[float] | 世界坐标系下的目标位姿 `[x, y, z, rx, ry, rz]`                             |
| offset | Sequence[float] | 传送带跟踪偏移位置 \[x, y, z, rx, ry, rz]                                  |
| mode   | int          | 运行模式。 0：按位姿跟踪。 1：按路径跟踪。                                             |

**返回值格式**: `ErrorID,{},CnvMovL(index, pose, offset, mode);`

**示例**:

```python
robot.plugins.CnvMovL(0, [400, 0, 300, 180, 0, 0], [0, 0, 0, 0, 0, 0], 0)
```

#### CnvMovC() - 传送带跟随圆弧运动

```python
plugins.CnvMovC(via_point, target_point, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0, coord_type=CoordinateType.CARTESIAN) -> str
```

文档原型：`CnvMovC(P1, P2, user, tool, a, v, cp|r, mode)`
- **P1 = 中间点 via_point**（先经过）
- **P2 = 目标点 target_point**（后到达）

**参数**:

| 参数           | 类型              | 默认值   | 说明                                                            |
| ------------ | --------------- | ----- | ------------------------------------------------------------- |
| via\_point   | Sequence[float] | -     | 圆弧中间点 P1 [x,y,z,rx,ry,rz] 或 [j1..j6]                        |
| target\_point | Sequence[float] | -     | 圆弧终点 P2 [x,y,z,rx,ry,rz] 或 [j1..j6]                         |
| user         | int             | -1    | 用户坐标系编号（0~50；-1=当前）                                       |
| tool         | int             | -1    | 工具坐标系编号（0~50；-1=当前）                                       |
| a            | int             | -1    | 加速度比例（1~100；-1=默认）                                          |
| v            | int             | -1    | 速度比例（1~100；-1=默认，与 speed 互斥，speed 优先）                            |
| speed        | int             | -1    | 目标速度（mm/s），与 v 互斥（优先 speed）                                      |
| cp           | int             | -1    | 平滑过渡比例（0~100；-1=不设置，与 r 互斥）                                  |
| r            | int             | -1    | 平滑过渡半径（mm；-1=不设置，与 cp 互斥）                                    |
| mode         | int             | 0     | 插补模式，默认 0                                                     |
| coord\_type  | CoordinateType  | CARTESIAN | 点位坐标系类型：CARTESIAN（笛卡尔 pose=）或 JOINT（关节 joint=）                  |

**返回值格式**: `ErrorID,{flag},CnvMovC(P1,P2,user,tool,a,v,cp|r,mode);`

**示例**:

```python
# 文档示例：CnvMovC(joint={1,2,3,4,5,6},joint={7,8,9,10,11,12},user=1,tool=0,a=20,v=50,cp=100)
# 对应 SDK 调用：
via    = [1, 2, 3, 4, 5, 6]
target = [7, 8, 9, 10, 11, 12]
robot.plugins.CnvMovC(via, target, user=1, tool=0, a=20, v=50, cp=100, coord_type=CoordinateType.JOINT)
```

#### StopSyncCnv() - 停止传送带跟踪

```python
plugins.StopSyncCnv() -> str
```

**示例**:

```python
robot.plugins.StopSyncCnv()
```

#### SetCnvPointOffset() - 设置传送带用户坐标系偏移

```python
plugins.SetCnvPointOffset(x_offset, y_offset) -> str
```

**示例**:

```python
robot.plugins.SetCnvPointOffset(10, 5)
```

#### SetCnvTimeCompensation() - 设置补偿时间

```python
plugins.SetCnvTimeCompensation(compensation) -> str
```

**示例**:

```python
robot.plugins.SetCnvTimeCompensation(100)
```

***

## 状态反馈系统

### RobotStatus 对象

**属性说明**:

| 属性                   | 类型            | 说明          |
| -------------------- | ------------- | ----------- |
| robot\_mode          | RobotMode     | 机器人模式       |
| speed\_scaling       | float         | 速度比例（0-100） |
| digital\_inputs      | int           | 数字输入状态（64位） |
| digital\_outputs     | int           | 数字输出状态（64位） |
| joint\_state         | JointState    | 关节状态        |
| tool\_vector\_actual | CartesianPose | 当前工具位姿      |
| tool\_vector\_target | CartesianPose | 目标工具位姿      |
| current\_tool        | int           | 当前工具坐标系     |
| current\_user        | int           | 当前用户坐标系     |

### JointState 对象

| 属性         | 类型           | 说明     |
| ---------- | ------------ | ------ |
| q\_actual  | list\[float] | 实际关节角度 |
| q\_target  | list\[float] | 目标关节角度 |
| qd\_actual | list\[float] | 实际关节速度 |
| i\_actual  | list\[float] | 实际关节电流 |

### CartesianPose 对象

| 属性 | 类型    | 说明      |
| -- | ----- | ------- |
| x  | float | X坐标（mm） |
| y  | float | Y坐标（mm） |
| z  | float | Z坐标（mm） |
| rx | float | Rx角度（度） |
| ry | float | Ry角度（度） |
| rz | float | Rz角度（度） |

**方法**:

```python
pose.to_list()  # 转换为列表 [x, y, z, rx, ry, rz]
```

### 使用示例

```python
# 获取状态
status = robot.GetStatus()

if status:
    # 机器人模式
    print(f"模式: {status.robot_mode.value}")
    
    # 速度比例
    print(f"速度比例: {status.speed_scaling}%")
    
    # 数字输入输出
    print(f"DI: {bin(status.digital_inputs)}")
    print(f"DO: {bin(status.digital_outputs)}")
    
    # 关节角度
    if status.joint_state:
        print(f"J1: {status.joint_state.q_actual[0]:.2f}")
    
    # 笛卡尔坐标
    if status.tool_vector_actual:
        pose = status.tool_vector_actual.to_list()
        print(f"位置: {pose}")
```

***

## 错误码处理

### 错误码解析

```python
from dobot_sdk.api.error_code import parse_error_ids, format_error_messages

# 解析错误码响应
response = "0,{[1537,2048,2049]},GetErrorID();"
error_ids = parse_error_ids(response)
print(error_ids)  # [1537, 2048, 2049]

# 格式化错误信息
message = format_error_messages(error_ids, lang="zh_CN")
print(message)
```

### 支持的语言

- `zh_CN` - 简体中文
- `en` - English
- `ja` - 日本語

### 错误信息格式

```
错误码: 1537
描述: 急停触发
原因: 急停按钮被按下
解决方案: 检查急停按钮并复位

错误码: 2048
描述: J1轴伺服报警
原因: J1轴伺服驱动器报警
解决方案: 检查J1轴伺服电机
```

### 常用错误码

| 错误码  | 描述      | 原因         |
| ---- | ------- | ---------- |
| 0    | 无错误     | -          |
| 1537 | 急停触发    | 急停按钮被按下    |
| 2048 | J1轴伺服报警 | J1轴伺服驱动器报警 |
| 2049 | J2轴伺服报警 | J2轴伺服驱动器报警 |
| 2050 | J3轴伺服报警 | J3轴伺服驱动器报警 |
| 2051 | J4轴伺服报警 | J4轴伺服驱动器报警 |
| 2052 | J5轴伺服报警 | J5轴伺服驱动器报警 |
| 2053 | J6轴伺服报警 | J6轴伺服驱动器报警 |

***

## 完整示例

### 示例1：基础运动控制

```python
from dobot_sdk import DobotRobot, CoordinateType
import time

# 连接机器人
robot = DobotRobot("192.168.1.100")
robot.Connect()

try:
    # 请求控制
    robot.robot_control.RequestControl()
    
    # 清除报警
    robot.robot_control.ClearError()
    
    # 使能
    robot.robot_control.EnableRobot()
    print("机器人已使能")
    
    # 等待稳定
    time.sleep(2)
    
    # 关节运动
    print("执行关节运动...")
    robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)
    time.sleep(3)
    
    # 直线运动
    print("执行直线运动...")
    robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    time.sleep(3)
    
    # 圆弧运动
    print("执行圆弧运动...")
    mid_point = [400, 100, 300, 180, 0, 0]
    end_point = [400, 200, 300, 180, 0, 0]
    robot.motion.Arc(mid_point, end_point, CoordinateType.CARTESIAN)
    time.sleep(3)
    
    # 回到初始位置
    print("回到初始位置...")
    robot.motion.Home()
    time.sleep(3)
    
    # 下使能
    robot.robot_control.DisableRobot()
    print("机器人已下使能")
    
finally:
    # 断开连接
    robot.Disconnect()
    print("已断开连接")
```

### 示例2：IO控制

```python
from dobot_sdk import DobotRobot

with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    
    # 设置DO1
    robot.io.DO(1, 1)
    print("DO1已打开")
    
    # 读取DI1状态
    response = robot.io.DI(1)
    print(f"DI1状态: {response}")
    
    # 设置模拟输出
    robot.io.AO(1, 50)  # 输出5V
    print("AO1已设置为5V")
    
    # 等待输入信号
    print("等待DI1为高电平...")
    robot.io.DIGroup(0, [1, 0, 0, 0])
    print("DI1已触发")
    
    # 关闭DO1
    robot.io.DO(1, 0)
    print("DO1已关闭")
```

### 示例3：状态监控

```python
from dobot_sdk import DobotRobot
import time

def on_status(status):
    """状态回调函数"""
    if status.tool_vector_actual:
        x = status.tool_vector_actual.x
        y = status.tool_vector_actual.y
        z = status.tool_vector_actual.z
        print(f"\r位置: X={x:.2f} Y={y:.2f} Z={z:.2f}", end="")

robot = DobotRobot("192.168.1.100")
robot.Connect()
robot.robot_control.RequestControl()
robot.robot_control.ClearError()
robot.robot_control.EnableRobot()

# 启动状态监控
robot.StartFeedbackMonitor(callback=on_status)

try:
    # 执行运动
    robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    time.sleep(5)
    
    # 获取当前状态
    status = robot.GetStatus()
    if status:
        print(f"\n最终位置: {status.tool_vector_actual.to_list()}")
        
finally:
    robot.StopFeedbackMonitor()
    robot.Disconnect()
```

### 示例4：力控模式

```python
from dobot_sdk import DobotRobot
import time

with DobotRobot("192.168.1.100") as robot:
    # 请求控制并使能机器人
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    
    # 开启力传感器
    robot.plugins.EnableFTSensor(1)
    robot.plugins.SixForceHome()
    
    # 设置力控参数
    direction = [1, 1, 1, 0, 0, 0]   # 启用 X、Y、Z 轴的力控
    force = [0, 0, -5, 0, 0, 0]       # 沿 Z 轴负方向设置 5 N 的目标力
    
    # 进入力控模式
    robot.plugins.FCForceMode(direction, force)
    
    # 保持力控状态5秒
    time.sleep(5)
    
    # 停止力控
    robot.robot_control.Stop()
```

***

## 附录

### 机器人模式对照表

| 值 | 模式名称  | 说明      |
| - | ----- | ------- |
| 1 | 初始化模式 | 系统初始化中  |
| 2 | 手动模式  | 手动操作模式  |
| 3 | 自动模式  | 自动运行模式  |
| 4 | 远程模式  | 远程控制模式  |
| 9 | 错误模式  | 错误/报警状态 |

### 常用端口

| 端口    | 用途        | 说明     |
| ----- | --------- | ------ |
| 29999 | Dashboard | 命令控制端口 |
| 30004 | Feedback  | 状态反馈端口 |
| 30003 | Real-time | 实时控制端口 |

### 指令类型说明

| 类型   | 说明          | 示例                        |
| ---- | ----------- | ------------------------- |
| 队列指令 | 加入运动队列，顺序执行 | MovJ, MovL, Arc           |
| 立即指令 | 立即执行，不进入队列  | GetPose, ClearError, Stop |

### 坐标系说明

| 坐标系   | 范围  | 说明       |
| ----- | --- | -------- |
| 用户坐标系 | 0-9 | 用户自定义坐标系 |
| 工具坐标系 | 0-9 | 工具末端坐标系  |

### 速度加速度范围

| 参数    | 范围    | 说明   |
| ----- | ----- | ---- |
| 速度比例  | 0-100 | 百分比  |
| 加速度比例 | 0-100 | 百分比  |
| 平滑过渡  | 0-100 | cp参数 |

### 安全注意事项

1. **急停按钮**：确保急停按钮易于触及
2. **速度限制**：首次调试建议使用低速
3. **碰撞检测**：开启碰撞检测功能
4. **安全区域**：确保机器人工作范围内无人
5. **负载检查**：正确设置负载参数

***

**文档版本**: V1.0\
**生成日期**: 2026年5月\
**适用SDK**: Dobot TCP-IP Python SDK V4.0
