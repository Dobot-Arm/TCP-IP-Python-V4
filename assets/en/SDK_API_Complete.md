# Dobot TCP-IP Python SDK API Documentation (Complete Version)

## Table of Contents

1. [Overview](#overview)
2. [Quick Start](#quick-start)
3. [DobotRobot Main Control Class](#dobotrobot-main-control-class)
4. [Motion Module](#motion-module)
5. [IO Digital IO Module](#io-digital-io-module)
6. [RobotControl Module](#robotcontrol-module)
7. [Communication Module](#communication-module)
8. [Plugins Module](#plugins-module)
9. [Status Feedback System](#status-feedback-system)
10. [Error Code Handling](#error-code-handling)
11. [Complete Examples](#complete-examples)
12. [Appendix](#appendix)

***

## Overview

### About This SDK

The Dobot TCP-IP Python SDK is Dobot's official interface for developing robot applications. It lets you communicate with and control robots over TCP/IP.

### Supported Robot Models

- CR3/CR5/CR10 series collaborative robots

### Protocol Version

- Supports Dobot TCP/IP protocol V4.6.6

### Architecture Design

```
DobotRobot (Main Class)
├── motion      - Motion control module
├── io          - Digital IO module
├── robot_control - Robot control module
├── communication - Communication module
└── plugins     - Plugin module
```

***

## Quick Start

### Environment Requirements

- Python 3.7+
- Robot firmware version supports TCP/IP protocol V4.6.6

### Installation

```bash
# Source code installation
cd dobot_sdk
pip install -e .
```

### Basic Example

```python
from dobot_sdk import DobotRobot, CoordinateType

# Connect to robot
with DobotRobot("192.168.1.100") as robot:
    # Request control
    robot.robot_control.RequestControl()
    
    # Clear alarms
    robot.robot_control.ClearError()
    
    # Enable robot
    robot.robot_control.EnableRobot()
    
    # Joint motion to initial position
    robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)
    
    # Cartesian motion
    robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    
    # Turn off DO1
    robot.io.DO(1, 0)
    
    # Get current pose
    pose = robot.robot_control.GetPose()
    print(f"Current pose: {pose}")
```

***

## DobotRobot Main Control Class

### Initialization

```python
DobotRobot(ip: str, dashboard_port: int = 29999, feedback_port: int = 30004)
```

**Parameters**:

| Parameter       | Type | Default | Description                  |
| --------------- | ---- | ------- | ---------------------------- |
| ip              | str  | -       | Robot IP address             |
| dashboard\_port | int  | 29999   | Dashboard port (command control) |
| feedback\_port  | int  | 30004   | Feedback port (status feedback) |

**Example**:

```python
# Use default ports
robot = DobotRobot("192.168.1.100")

# Custom ports
robot = DobotRobot("192.168.1.100", dashboard_port=29999, feedback_port=30004)
```

### Connection Management

#### Connect()

Establish a connection with the robot.

```python
robot.Connect(timeout: float = 5.0) -> None
```

**Parameters**:

| Parameter | Type  | Default | Description        |
| --------- | ----- | ------- | ------------------ |
| timeout   | float | 5.0     | Connection timeout (seconds) |

**Example**:

```python
robot.Connect()
```

#### Disconnect()

Disconnect from the robot.

```python
robot.Disconnect() -> None
```

**Example**:

```python
robot.Disconnect()
```

### Module Interface Description

> **Note**: To keep the interface clean, the DobotRobot main class no longer provides shortcut control methods. Please call them through the corresponding modules:
> - Robot control: `robot.robot_control.EnableRobot()` / `robot.robot_control.DisableRobot()` / `robot.robot_control.ClearError()`
> - Motion control: `robot.motion.MovJ()` / `robot.motion.MovL()` / `robot.motion.Arc()`
> - IO control: `robot.io.DO()` / `robot.io.DI()` / `robot.io.AO()`
> - Communication control: `robot.communication.ModbusCreate()`
> - Plugin control: `robot.plugins.FCForceMode()`

Please refer to each module section for detailed interfaces.

### Status Monitoring

#### StartFeedbackMonitor()

Start the status feedback monitoring thread.

```python
robot.StartFeedbackMonitor(callback: Callable = None) -> None
```

**Parameters**:

| Parameter | Type       | Description                                      |
| --------- | ---------- | ------------------------------------------------ |
| callback  | Callable   | A status update callback that receives a `RobotStatus` object. |

**Example**:

```python
def on_status_update(status):
    print(f"Current position: X={status.tool_vector_actual.x}")

robot.StartFeedbackMonitor(callback=on_status_update)
```

#### StopFeedbackMonitor()

Stop status feedback monitoring.

```python
robot.StopFeedbackMonitor() -> None
```

**Example**:

```python
robot.StopFeedbackMonitor()
```

#### GetStatus()

Get the current status.

```python
robot.GetStatus() -> Optional[RobotStatus]
```

**Return value**: RobotStatus object

**Example**:

```python
status = robot.GetStatus()
if status:
    print(f"Speed scaling: {status.speed_scaling}%")
    print(f"Robot mode: {status.robot_mode.value}")
```

### Context Manager Support

```python
with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    # Perform operations
# Auto disconnect
```

***

## Motion Module

Accessed via `robot.motion`.

### Coordinate System Types

```python
from dobot_sdk import CoordinateType

CoordinateType.CARTESIAN  # Cartesian coordinates (x, y, z, rx, ry, rz)
CoordinateType.JOINT      # Joint angles (j1, j2, j3, j4, j5, j6)
```

### Basic Motion Commands

#### MovJ() - Joint Motion

```python
motion.MovJ(pose, coord_type, user=-1, tool=-1, a=-1, v=-1, cp=-1) -> str
```

**Parameters**:

| Parameter   | Type           | Default | Description                          |
| ----------- | -------------- | ------- | ------------------------------------ |
| pose        | list[float]    | -       | 6 coordinate values                  |
| coord\_type | CoordinateType | -       | Coordinate system type               |
| user        | int            | -1      | User coordinate system index (0-50)  |
| tool        | int            | -1      | Tool coordinate system index (0-50)  |
| a           | int            | -1      | Acceleration ratio (1-100)           |
| v           | int            | -1      | Speed ratio (1-100)                  |
| cp          | int            | -1      | Smooth transition ratio (0-100)      |

**Example**:

```python
# Joint motion
robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)

# Cartesian motion with parameters
robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN, v=50)
```

#### MovL() - Linear Motion

```python
motion.MovL(pose, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1) -> str
```

**Parameters**:

| Parameter | Type | Default | Description                                      |
| --------- | ---- | ------- | ------------------------------------------------ |
| speed     | int  | -1      | Target speed (mm/s), mutually exclusive with v   |
| r         | int  | -1      | Smooth transition radius (mm), mutually exclusive with cp |

**Example**:

```python
robot.motion.MovL([500, 100, 200, 180, 0, 0], CoordinateType.CARTESIAN, speed=100)
```

#### MovJIO() - Joint Motion with DO Output

```python
motion.MovJIO(pose, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, cp=-1) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                 |
| --------- | ---- | --------------------------------------------------------------------------- |
| do_list   | list | DO output list, each element is [do_index, do_status], supports multiple DO outputs simultaneously |

**Example**:

```python
# Move to target point while turning on DO1 and DO2
robot.motion.MovJIO([400, 0, 300, 180, 0, 0], [[1, 1], [2, 1]], CoordinateType.CARTESIAN)
```

#### MovLIO() - Linear Motion with DO Output

```python
motion.MovLIO(pose, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1) -> str
```

**Example**:

```python
# Linear motion to target point, turn off DO1
robot.motion.MovLIO([500, 0, 300, 180, 0, 0], [[1, 0]], CoordinateType.CARTESIAN)
```

#### Arc() - Arc Interpolation Motion

```python
motion.Arc(p1, p2, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**Parameters**:

| Parameter | Type         | Default | Description                                                |
| --------- | ------------ | ------- | ---------------------------------------------------------- |
| p1        | list[float]  | -       | Arc intermediate point pose                                |
| p2        | list[float]  | -       | Target point pose                                          |
| mode      | int          | 0       | Attitude control mode (0-linear, 1-via intermediate point, 2-fixed) |

**Example**:

```python
mid_point = [400, 100, 300, 180, 0, 0]
end_point = [400, 200, 300, 180, 0, 0]
robot.motion.Arc(mid_point, end_point, CoordinateType.CARTESIAN)
```

#### ArcIO() - Arc Motion with DO Output

```python
motion.ArcIO(p1, p2, do_list, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**Example**:

```python
mid_point = [400, 100, 300, 180, 0, 0]
end_point = [400, 200, 300, 180, 0, 0]
# Turn on DO1 when reaching the arc end point
robot.motion.ArcIO(mid_point, end_point, [[1, 1]], CoordinateType.CARTESIAN)
```

#### Circle() - Full Circle Interpolation Motion

```python
motion.Circle(p1, p2, count, coord_type, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0) -> str
```

**Parameters**:

| Parameter | Type | Description       |
| --------- | ---- | ----------------- |
| count     | int  | Number of turns (1-999) |

**Example**:

```python
# Complete two revolutions along the circular path
robot.motion.Circle(mid_point, end_point, 2, CoordinateType.CARTESIAN)
```

### Servo Motion

#### ServoJ() - Joint Space Dynamic Following

```python
motion.ServoJ(joints, t=0.1, aheadtime=50.0, gain=500.0) -> str
```

**Parameters**:

| Parameter | Type            | Default | Description                                      |
| --------- | --------------- | ------- | ------------------------------------------------ |
| joints    | Sequence[float] | -       | Joint angle list [j1,j2,j3,j4,j5,j6] (degrees)   |
| t         | float           | 0.1     | Run time (seconds, 0.004-3600.0)                 |
| aheadtime | float           | 50.0    | Look-ahead amount (20.0-100.0)                   |
| gain      | float           | 500.0   | Proportional gain (200.0-1000.0)                 |

**Example**:

```python
import time

# Real-time following example
for i in range(100):
    j1 = i * 0.5
    robot.motion.ServoJ([j1, 0, 90, 0, 90, 0], t=0.05)
    time.sleep(0.05)
```

#### ServoP() - Cartesian Space Dynamic Following

```python
motion.ServoP(pose, t=0.1, aheadtime=50.0, gain=500.0) -> str
```

**Example**:

```python
robot.motion.ServoP([400 + i*2, 0, 300, 180, 0, 0], t=0.05)
```

### Jog Control

#### MoveJog() - Jog the Robot Arm

```python
motion.MoveJog(axis="", coord_type=CoordinateType.JOINT, user=0, tool=0) -> str
```

**Parameters**:

| Parameter | Type | Default | Description                                                                       |
| --------- | ---- | ------- | --------------------------------------------------------------------------------- |
| axis      | str  | ""      | "X+", "X-", "Y+", "Y-", "Z+", "Z-", "J1+", "J1-", etc. Empty string stops jogging |

**Example**:

```python
# Start jogging
robot.motion.MoveJog("X+", CoordinateType.CARTESIAN)

# Stop jogging
robot.motion.MoveJog()
```

### Relative Motion

#### RelMovJTool() - Relative Joint Motion in Tool Coordinate System

```python
motion.RelMovJTool(offset, v=-1) -> str
```

**Example**:

```python
# Move along tool coordinate system
robot.motion.RelMovJTool([0, 0, 0, 0, 0, 30], v=50)
```

#### RelMovLTool() - Relative Linear Motion in Tool Coordinate System

```python
motion.RelMovLTool(offset, v=-1, r=-1) -> str
```

**Example**:

```python
# Move 50mm along X direction
robot.motion.RelMovLTool([50, 0, 0, 0, 0, 0], v=50)
```

#### RelMovJUser() - Relative Joint Motion in User Coordinate System

```python
motion.RelMovJUser(offset, v=-1) -> str
```

**Example**:

```python
# Relative motion in user coordinate system
robot.motion.RelMovJUser([0, 50, 0, 0, 0, 0], v=50)
```

#### RelMovLUser() - Relative Linear Motion in User Coordinate System

```python
motion.RelMovLUser(offset, v=-1, r=-1) -> str
```

**Example**:

```python
# Relative linear motion in user coordinate system
robot.motion.RelMovLUser([50, 0, 0, 0, 0, 0], v=50)
```

#### RelJointMovJ() - Relative Motion in Joint Coordinate System

```python
motion.RelJointMovJ(offset, v=-1) -> str
```

**Example**:

```python
# J1 axis relative rotation by 30 degrees
robot.motion.RelJointMovJ([30, 0, 0, 0, 0, 0], v=30)
```

### Trajectory Reproduction

#### MovS() - Fitting Imported Trajectory

```python
# Method 1: Point list method (4~50 points)
motion.MovS([p1, p2, p3, ...], coord_type=CoordinateType.CARTESIAN, freq=-1, user=-1, tool=-1, a=-1, v=-1, speed=-1) -> str

# Method 2: File method
motion.MovS("trajectory.csv", coord_type=CoordinateType.CARTESIAN, freq=-1, user=-1, tool=-1, a=-1, v=-1, speed=-1) -> str
```

**Parameters**:

| Parameter              | Type                          | Default   | Description                                                                                      |
| ---------------------- | ----------------------------- | --------- | ------------------------------------------------------------------------------------------------ |
| trace\_or\_points      | str \| Sequence[Sequence[float]] | -       | str=trajectory file name (with extension); Sequence=4~50 points, each point is [x,y,z,rx,ry,rz] or [j1..j6] |
| coord\_type            | CoordinateType                | CARTESIAN | Point coordinate system type. Only used for point list method: CARTESIAN or JOINT                |
| freq                   | float                         | -1        | Filter coefficient (0~1, 1=filter off; -1=not set)                                               |
| user                   | int                           | -1        | User coordinate system index (-1=current)                                                        |
| tool                   | int                           | -1        | Tool coordinate system index (-1=current)                                                        |
| a                      | int                           | -1        | Acceleration ratio (1~100; -1=global)                                                            |
| v                      | int                           | -1        | Speed ratio (1~100; -1=global, mutually exclusive with speed, speed takes priority)              |
| speed                  | int                           | -1        | Target speed (mm/s), mutually exclusive with v (speed takes priority)                            |

> **Note**: Parameters such as `isConst/multi/sample` belong to the `StartPath()` trajectory reproduction command. The `MovS()` fitting command **does not use** these parameters.

**Example**:

```python
# Method 1: 4-point fitting (Cartesian coordinates)
points = [
    [100, 0, 100, 0, 0, 0],
    [100, 20, 100, 0, 0, 0],
    [100, 30, 100, 0, 0, 0],
    [100, 40, 100, 0, 0, 0],
]
robot.motion.MovS(points, v=50, freq=0.2)

# Method 2: File fitting
robot.motion.MovS("trajectory.csv", v=30)
```

#### StartPath() - Reproduce Recorded Trajectory

```python
motion.StartPath(trace_name, is_const=0, multi=1.0, sample=50, freq=0.2, user=-1, tool=-1) -> str
```

**Example**:

```python
robot.motion.StartPath("recorded_trace.csv", is_const=1, multi=0.8)
```

#### GetStartPose() - Get Trajectory Start Point

```python
motion.GetStartPose(trace_name, path_type=1) -> str
```

**Parameters**:

| Parameter   | Type | Default | Description                                      |
| ----------- | ---- | ------- | ------------------------------------------------ |
| path\_type  | int  | 1       | Trajectory type (1-reproduction trajectory, 2-fitting trajectory) |

**Example**:

```python
# Get the start point of a fitting trajectory
response = robot.motion.GetStartPose("trajectory.csv", path_type=2)
print(f"Trajectory start point: {response}")
```

### Coordinate System Offset

#### StartRTOffset() - Start Coordinate System Offset

```python
motion.StartRTOffset(offset) -> str
```

**Example**:

```python
# Start offset
robot.motion.StartRTOffset([10, 10, 0, 0, 0, 0])

# Execute motion (with offset)
robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)

# End offset
robot.motion.EndRTOffset()
```

#### EndRTOffset() - End Coordinate System Offset

```python
motion.EndRTOffset() -> str
```

#### OffsetPara() - Set Offset Parameters

```python
motion.OffsetPara(freq=0.2) -> str
```

**Example**:

```python
# Set offset filter coefficient
robot.motion.OffsetPara(freq=0.5)
```

### Trajectory Recovery

#### SetResumeOffset() - Set Recovery Back-off Distance

```python
motion.SetResumeOffset(distance) -> str
```

**Example**:

```python
robot.motion.SetResumeOffset(50)  # Back off 50mm
```

#### PathRecovery() - Start Trajectory Recovery

```python
motion.PathRecovery() -> str
```

**Example**:

```python
# Set back-off distance then start recovery
robot.motion.SetResumeOffset(50)
robot.motion.PathRecovery()
```

#### PathRecoveryStop() - Stop Trajectory Recovery

```python
motion.PathRecoveryStop() -> str
```

**Example**:

```python
robot.motion.PathRecoveryStop()
```

#### PathRecoveryStatus() - Query Recovery Status

```python
motion.PathRecoveryStatus() -> str
```

**Return value**:

- 0: Returned to paused pose
- 1: Small deviation
- 2: Large deviation

**Example**:

```python
status = robot.motion.PathRecoveryStatus()
print(f"Recovery status: {status}")
```

### Speed and Acceleration Settings (robot_control module)

#### SpeedFactor() - Set Global Speed Ratio

```python
robot_control.SpeedFactor(speed) -> str
```

**Parameters**:

| Parameter | Type | Description                          |
| --------- | ---- | ------------------------------------ |
| speed     | int  | Global motion speed ratio, range: [1, 100] |

**Example**:

```python
robot.robot_control.SpeedFactor(50)  # Set 50% global speed
```

#### AccJ() - Set Joint Motion Acceleration Ratio

```python
robot_control.AccJ(acc) -> str
```

**Parameters**:

| Parameter | Type | Description                                      |
| --------- | ---- | ------------------------------------------------ |
| acc       | int  | Joint acceleration ratio, range: [1, 100]         |

**Example**:

```python
# Set joint motion acceleration ratio to 30%
robot.robot_control.AccJ(30)
```

#### AccL() - Set Linear and Arc Motion Acceleration Ratio

```python
robot_control.AccL(acc) -> str
```

**Parameters**:

| Parameter | Type | Description                          |
| --------- | ---- | ------------------------------------ |
| acc       | int  | Acceleration ratio, range: [1, 100]  |

**Example**:

```python
# Set linear and arc motion acceleration ratio to 40%
robot.robot_control.AccL(40)
```

### Coordinate System Settings

#### SetUser() - Set User Coordinate System

```python
robot_control.SetUser(index, pose, type=None) -> str
```

**Parameters**:

| Parameter | Type             | Description                                                                                      |
| --------- | ---------------- | ------------------------------------------------------------------------------------------------ |
| index     | int              | User coordinate system index (1-50)                                                              |
| pose      | Sequence[float]  | 6 coordinates [x,y,z,rx,ry,rz]                                                                   |
| type      | int              | Whether to make coordinate system changes globally effective. 0: changes only take effect during current project runtime. 1: changes will be saved by the controller (optional) |

**Example**:

```python
# Set user coordinate system 1
robot.robot_control.SetUser(1, [100, 0, 0, 0, 0, 0])

# Set user coordinate system 1 and save to controller
robot.robot_control.SetUser(1, [100, 0, 0, 0, 0, 0], type=1)
```

#### SetTool() - Set Tool Coordinate System

```python
robot_control.SetTool(index, pose, type=None) -> str
```

**Parameters**:

| Parameter | Type             | Description                                                                                      |
| --------- | ---------------- | ------------------------------------------------------------------------------------------------ |
| index     | int              | Tool coordinate system index (1-50)                                                              |
| pose      | Sequence[float]  | 6 coordinates [x,y,z,rx,ry,rz]                                                                   |
| type      | int              | Whether to make coordinate system changes globally effective. 0: changes only take effect during current project runtime. 1: changes will be saved by the controller (optional) |

**Example**:

```python
# Set tool coordinate system 1 (end-effector tool offset)
robot.robot_control.SetTool(1, [0, 0, 100, 0, 0, 0])

# Set tool coordinate system 1 and save to controller
robot.robot_control.SetTool(1, [0, 0, 100, 0, 0, 0], type=1)
```

#### SetPayload() - Set End-effector Payload

```python
robot_control.SetPayload(load, center=None, preset_name=None) -> str
```

**Parameters**:

| Parameter    | Type            | Description                                                  |
| ------------ | --------------- | ------------------------------------------------------------ |
| load         | float           | Payload weight (kg)                                          |
| center       | Sequence[float] | Payload center of gravity coordinates [x,y,z] (mm) (optional)|
| preset\_name | str             | Payload preset name, used to save current configuration for later use (optional) |

**Example**:

```python
# Method 1: Set parameters directly
robot.robot_control.SetPayload(1.5, [0, 0, 50])

# Method 2: Set weight only
robot.robot_control.SetPayload(1.5)

# Method 3: Set payload and save as preset
robot.robot_control.SetPayload(1.5, [0, 0, 50], "my_payload")
```

### Brake Control (robot_control module)

#### BrakeControl() - Brake Control

```python
robot_control.BrakeControl(axis, status) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                          |
| --------- | ---- | ------------------------------------------------------------------------------------ |
| axis      | int  | Joint axis index, range: [1, 6]. 1 represents J1 axis, 2 represents J2 axis, and so on |
| status    | int  | Brake status. 0 means brake locked (joint immovable). 1 means brake released (joint movable). |

**Example**:

```python
robot.robot_control.BrakeControl(1, 1)  # Release J1 axis brake
```

### Motion Control (robot_control module)

#### Stop() - Stop Motion

```python
robot_control.Stop() -> str
```

**Description**:
This command is used to stop the robot's currently issued motion command queue. It can also be used to stop a running script project (compatible alternative to RunScript stop).

**Example**:

```python
robot.robot_control.Stop()
```

#### Pause() - Pause Motion

```python
robot_control.Pause() -> str
```

**Description**:
Pause the issued motion command queue or the project running via the RunScript command.

**Example**:

```python
robot.robot_control.Pause()
```

#### Continue() - Resume Motion

```python
robot_control.Continue() -> str
```

**Description**:
Resume the paused motion command queue or the project running via the RunScript command.

**Example**:

```python
robot.robot_control.Continue()
```

#### EmergencyStop() - Emergency Stop

```python
robot_control.EmergencyStop(mode) -> str
```

**Parameters**:

| Parameter | Type | Description                                              |
| --------- | ---- | -------------------------------------------------------- |
| mode      | int  | Emergency stop operation mode. 1 means press emergency stop, 0 means release emergency stop |

**Example**:

```python
# Press emergency stop
robot.robot_control.EmergencyStop(1)

# Release emergency stop
robot.robot_control.EmergencyStop(0)
```

***

## IO Digital IO Module

Accessed via `robot.io`.

### Digital Output

#### DO() - Set Digital Output (Queue Command)

```python
io.DO(index, status, time=None) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                      |
| --------- | ----- | -------------------------------------------------------------------------------- |
| index     | int   | DO port index (1-64)                                                             |
| status    | int   | 0-Off, 1-On                                                                      |
| time      | float | Optional output duration in seconds. Applies when `status` is 1. The output automatically switches to 0 when the specified duration elapses. |

**Example**:

```python
robot.io.DO(1, 1)           # Turn on DO1
robot.io.DO(1, 0)           # Turn off DO1
robot.io.DO(1, 1, 2.0)      # Turn on DO1, auto turn off after 2 seconds
```

#### DOInstant() - Set Digital Output (Immediate Command)

```python
io.DOInstant(index, status) -> str
```

**Example**:

```python
robot.io.DOInstant(1, 1)
```

#### GetDO() - Get Digital Output Status

```python
io.GetDO(index) -> str
```

**Example**:

```python
response = robot.io.GetDO(1)
print(response)  # ErrorID,{status},GetDO(1);
```

#### DOGroup() - Set Multiple Digital Outputs

```python
io.DOGroup(group_index, status) -> str
```

**Parameters**:

| Parameter    | Type     | Description               |
| ------------ | -------- | ------------------------- |
| group\_index | int      | IO group index (0-3)      |
| status       | list/str | Port status list or string |

**Example**:

```python
# Set group 0 (DO1-DO4)
robot.io.DOGroup(0, [1, 0, 1, 0])
```

#### DOGroupDEC() - Set DO Group in Decimal

```python
io.DOGroupDEC(group_index, value) -> str
```

**Example**:

```python
robot.io.DOGroupDEC(0, 5)  # Binary 0101
```

#### GetDOGroup() - Get DO Group Status

```python
io.GetDOGroup(group_index) -> str
```

**Example**:

```python
response = robot.io.GetDOGroup(0)
print(f"DO group status: {response}")
```

### Digital Input

#### DI() - Get Digital Input Status

```python
io.DI(index) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                                                                                |
| --------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| index     | int  | DI terminal number. Range: [1, MAX] or [100, 1000]. MAX represents the DI range of the current control cabinet. When range is [100, 1000], extended IO module hardware support is required. |

**Return value format**: `ErrorID,{value},DI(index);`
- `value` indicates the DI terminal state. A value of 0 means off, and 1 means on.

**Example**:

```python
response = robot.io.DI(1)
```

#### DIGroup() - Wait for Digital Input Group

```python
io.DIGroup(group_index, status) -> str
```

**Example**:

```python
robot.io.DIGroup(0, [1, 1, 0, 0])
```

#### DIGroupDEC() - Get DI Group Status (Decimal)

```python
io.DIGroupDEC(group_index) -> str
```

**Example**:

```python
response = robot.io.DIGroupDEC(0)
print(f"DI group status (decimal): {response}")
```

### Analog IO

#### AO() - Set Analog Output

```python
io.AO(index, value) -> str
```

**Parameters**:

| Parameter | Type  | Description                          |
| --------- | ----- | ------------------------------------ |
| index     | int   | AO port index (1-2)                  |
| value     | float | Output value (0-10V corresponds to 0-100) |

**Example**:

```python
robot.io.AO(1, 50)  # Output 5V
```

#### GetAO() - Get Analog Output

```python
io.GetAO(index) -> str
```

**Example**:

```python
response = robot.io.GetAO(1)
print(f"AO1 output value: {response}")
```

#### AI() - Get Analog Input

```python
io.AI(index) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                      |
| --------- | ---- | -------------------------------------------------------------------------------- |
| index     | int  | AI terminal number. Range: [1, MAX]. Different control cabinets have different AI resource counts. |

**Return value format**: `ErrorID,{value},AI(index);`

**Example**:

```python
response = robot.io.AI(1)
print(f"AI1 input value: {response}")
```

### End-effector IO

#### ToolDO() - Set End-effector Digital Output

```python
io.ToolDO(index, status) -> str
```

**Example**:

```python
robot.io.ToolDO(1, 1)
```

#### ToolDI() - Get End-effector Digital Input

```python
io.ToolDI(index) -> str
```

**Example**:

```python
response = robot.io.ToolDI(1)
print(f"End-effector DI1 status: {response}")
```

#### ToolAI() - Get End-effector Analog Input

```python
io.ToolAI(index) -> str
```

**Example**:

```python
response = robot.io.ToolAI(1)
print(f"End-effector AI1 input value: {response}")
```

***

## RobotControl Module

Accessed via `robot.robot_control`.

### Control Mode

#### RequestControl() - Request TCP Control Mode

```python
robot_control.RequestControl() -> str
```

**Example**:

```python
robot.robot_control.RequestControl()
```

### Power Control

#### PowerOn() - Power On Robot

```python
robot_control.PowerOn() -> str
```

**Example**:

```python
robot.robot_control.PowerOn()
```

#### DisableRobot() - Disable the Robot

```python
robot_control.DisableRobot() -> str
```

**Example**:

```python
robot.robot_control.DisableRobot()
```

### Status Query

#### GetErrorID() - Get Error Code

```python
robot_control.GetErrorID() -> str
```

**Return value format**: `ErrorID,{[error1,error2,...]},GetErrorID();`

**Example**:

```python
response = robot.robot_control.GetErrorID()
print(response)  # 0,{[1537,2048]},GetErrorID();
```

#### RobotMode() - Get Robot Mode

```python
robot_control.RobotMode() -> str
```

**Return value**:

| Value | Mode       |
| ----- | ---------- |
| 1     | Initialization |
| 2     | Manual     |
| 3     | Automatic  |
| 4     | Remote     |
| 9     | Error      |

**Example**:

```python
mode = robot.robot_control.RobotMode()
print(f"Robot mode: {mode}")
```

#### GetPose() - Get Current Pose

```python
robot_control.GetPose(user=-1, tool=-1) -> str
```

**Example**:

```python
response = robot.robot_control.GetPose()
print(response)  # ErrorID,{x,y,z,rx,ry,rz},GetPose();
```

#### GetAngle() - Get Joint Angles

```python
robot_control.GetAngle() -> str
```

**Example**:

```python
response = robot.robot_control.GetAngle()
print(response)  # ErrorID,{j1,j2,j3,j4,j5,j6},GetAngle();
```

#### Get Current Speed Scaling

Read the current speed scaling percentage from the `SpeedScaling` field in the real-time feedback on port 30004. The SDK exposes this value as `robot.GetStatus().speed_scaling`.

**Example**:

```python
# Read speed scaling from status feedback
status = robot.GetStatus()
if status:
    print(f"Current speed scaling: {status.speed_scaling}%")
```

### Kinematics Calculation

#### InverseKin() - Inverse Kinematics Calculation

```python
robot_control.InverseKin(pose, use_joint_near=0, joint_near=None, user=-1, tool=-1) -> str
```

**Parameters**:

| Parameter       | Type            | Description                                                                       |
| --------------- | --------------- | --------------------------------------------------------------------------------- |
| pose            | Sequence[float] | 6 Cartesian coordinates [x,y,z,rx,ry,rz]                                          |
| use\_joint\_near| int             | Whether to use joint proximity constraint. 0: not used. 1: used                   |
| joint\_near     | Sequence[float] | Joint proximity reference values [j1,j2,j3,j4,j5,j6], effective when use_joint_near is 1 (optional) |
| user            | int             | User coordinate system index. Default -1, meaning current user coordinate system (optional) |
| tool            | int             | Tool coordinate system index. Default -1, meaning current tool coordinate system (optional) |

**Example**:

```python
pose = [400, 0, 300, 180, 0, 0]
response = robot.robot_control.InverseKin(pose)

# Use joint proximity constraint
response = robot.robot_control.InverseKin(pose, use_joint_near=1, joint_near=[0, 0, 90, 0, 90, 0])
```

#### PositiveKin() - Forward Kinematics Calculation

```python
robot_control.PositiveKin(joints, user=-1, tool=-1) -> str
```

**Parameters**:

| Parameter | Type            | Description                                                                       |
| --------- | --------------- | --------------------------------------------------------------------------------- |
| joints    | Sequence[float] | 6 joint angles [j1,j2,j3,j4,j5,j6]                                                |
| user      | int             | User coordinate system index. Default -1, meaning current user coordinate system (optional) |
| tool      | int             | Tool coordinate system index. Default -1, meaning current tool coordinate system (optional) |

**Example**:

```python
joints = [0, 0, 90, 0, 90, 0]
response = robot.robot_control.PositiveKin(joints)
```

### Script Control

#### RunScript() - Run Script

```python
robot_control.RunScript(script_name) -> str
```

**Example**:

```python
robot.robot_control.RunScript("test.lua")
```

#### Stop() - Stop Motion or Stop Script Execution

```python
robot_control.Stop() -> str
```

**Description**:
This command is used to stop the robot's current motion. It can also be used to stop a running script project (compatible alternative to RunScript stop).

**Example**:

```python
# Stop current motion or script
robot.robot_control.Stop()
```

### Log Export

#### LogExportUSB() - Export Robot Logs to USB Drive

```python
robot_control.LogExportUSB(range) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                                                  |
| --------- | ---- | ------------------------------------------------------------------------------------------------------------ |
| range     | int  | (Required) Log export range. 0: export contents of logs/all and logs/user folders. 1: export all contents of logs folder. Only values 0 or 1 are allowed. |

**Return value format**: `ErrorID,{},LogExportUSB(range);`

**Description**:
- When exporting logs, it is recommended to insert only one USB drive to avoid export failure.
- If the USB drive contains multiple partitions, logs will be exported to the first partition.
- Do not remove the USB drive during export, as this may cause file corruption.
- This command returns immediately after being issued. Please use GetExportStatus to check the log export status.

**Example**:

```python
# Export contents of logs/all and logs/user folders
robot.robot_control.LogExportUSB(0)

# Export all contents of logs folder
robot.robot_control.LogExportUSB(1)
```

#### GetExportStatus() - Get Log Export Status

```python
robot_control.GetExportStatus() -> str
```

**Return value format**: `ErrorID,{status},GetExportStatus();`

**Return value status description**:

| Value | Status                                  |
| ----- | --------------------------------------- |
| 0     | Export not started                      |
| 1     | Exporting                               |
| 2     | Export completed                        |
| 3     | Export failed, USB drive not found      |
| 4     | Export failed, insufficient USB space   |
| 5     | Export failed, USB drive removed during export |

**Description**:
The export completed and export failed statuses will persist until the user uses the export function next time.

**Example**:

```python
import time

# Start export
robot.robot_control.LogExportUSB(0)

# Poll export status
while True:
    status_resp = robot.robot_control.GetExportStatus()
    print(f"Export status: {status_resp}")
    # Parse status to determine if complete
    time.sleep(1)
```

***

## Communication Module

Accessed via `robot.communication`.

### Modbus Master

#### ModbusCreate() - Create Modbus Master

```python
communication.ModbusCreate(ip, port, slave_id, is_rtu=0) -> str
```

**Parameters**:

| Parameter   | Type | Default | Description                  |
| ----------- | ---- | ------- | ---------------------------- |
| ip          | str  | -       | Slave IP address             |
| port        | int  | -       | Slave port                   |
| slave\_id   | int  | -       | Slave ID                     |
| is\_rtu     | int  | 0       | 0-TCP mode, 1-RTU mode       |

**Example**:

```python
response = robot.communication.ModbusCreate("192.168.1.10", 502, 1)
```

#### ModbusRTUCreate() - Create RTU Master

```python
communication.ModbusRTUCreate(slave_id, baud, parity="E", data_bit=8, stop_bit=1) -> str
```

**Parameters**:

| Parameter | Type | Default | Description                                      |
| --------- | ---- | ------- | ------------------------------------------------ |
| parity    | str  | "E"     | "O"-odd parity, "E"-even parity, "N"-no parity   |

**Example**:

```python
robot.communication.ModbusRTUCreate(1, 19200, "N", 8, 1)
```

#### ModbusClose() - Close Modbus Connection

```python
communication.ModbusClose(index) -> str
```

**Example**:

```python
robot.communication.ModbusClose(0)
```

### Register Read/Write

#### GetInBits() - Read Contact Registers

```python
communication.GetInBits(index, addr, count) -> str
```

**Example**:

```python
response = robot.communication.GetInBits(0, 0, 8)
print(f"Contact registers: {response}")
```

#### GetCoils() - Read Coil Registers

```python
communication.GetCoils(index, addr, count) -> str
```

**Example**:

```python
response = robot.communication.GetCoils(0, 0, 8)
print(f"Coil registers: {response}")
```

#### SetCoils() - Write Coil Registers

```python
communication.SetCoils(index, addr, count, values) -> str
```

**Example**:

```python
robot.communication.SetCoils(0, 0, 4, [1, 0, 1, 0])
```

#### GetInRegs() - Read Input Registers

```python
communication.GetInRegs(index, addr, count) -> str
```

**Example**:

```python
response = robot.communication.GetInRegs(0, 0, 4)
print(f"Input registers: {response}")
```

#### GetHoldRegs() - Read Holding Registers

```python
communication.GetHoldRegs(index, addr, count) -> str
```

**Example**:

```python
response = robot.communication.GetHoldRegs(0, 0, 4)
print(f"Holding registers: {response}")
```

#### SetHoldRegs() - Write Holding Registers

```python
communication.SetHoldRegs(index, addr, count, values) -> str
```

**Example**:

```python
robot.communication.SetHoldRegs(0, 0, 2, [100, 200])
```

#### SetSingleCoil() - Write Single Coil Register (New in V4.6.6)

```python
communication.SetSingleCoil(index, addr, value) -> str
```

**Parameters**:

| Parameter | Type | Description                              |
| --------- | ---- | ---------------------------------------- |
| index     | int  | Modbus master index (0-9)                |
| addr      | int  | Coil register address                    |
| value     | int  | Write value, 0-open, 1-close             |

**Example**:

```python
# Close a single coil at address 0
robot.communication.SetSingleCoil(0, 0, 1)

# Open a single coil at address 5
robot.communication.SetSingleCoil(0, 5, 0)
```

#### SetSingleHoldReg() - Write Single Holding Register (New in V4.6.6)

```python
communication.SetSingleHoldReg(index, addr, val) -> str
```

**Parameters**:

| Parameter | Type | Description                                              |
| --------- | ---- | -------------------------------------------------------- |
| index     | int  | Modbus master index (0-9)                                |
| addr      | int  | Holding register address                                 |
| val       | int  | Integer value to write (U16: 16-bit unsigned integer)    |

**Example**:

```python
# Write value 200 to address 1
robot.communication.SetSingleHoldReg(0, 1, 200)
```


***

## Plugins Module

Accessed via `robot.plugins`.

### Force Sensor

#### EnableFTSensor() - Enable/Disable Force Sensor

```python
plugins.EnableFTSensor(status) -> str
```

**Example**:

```python
robot.plugins.EnableFTSensor(1)  # Enable
```

#### SixForceHome() - Force Sensor Zeroing

```python
plugins.SixForceHome() -> str
```

**Example**:

```python
robot.plugins.SixForceHome()
```

#### GetForce() - Get Force Sensor Values

```python
plugins.GetForce(tool=-1) -> str
```

**Return value format**: `ErrorID,{fx,fy,fz,frx,fry,frz},GetForce();`

**Example**:

```python
response = robot.plugins.GetForce()
```

### Force Control Drag Mode

#### ForceDriveMode() - Enter Force Control Drag Mode

```python
plugins.ForceDriveMode(direction, user=-1) -> str
```

**Parameters**:

| Parameter | Type          | Description                                                                                                                       |
| --------- | ------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| direction | Sequence[int] | Drag settings for the six axes `[x, y, z, rx, ry, rz]`. For each axis, `0` disables dragging and `1` enables it. For example, `[1, 1, 1, 1, 1, 1]` enables dragging on all six axes. |
| user      | int           | Optional user coordinate system index (0–50). If omitted, the command does not use a user coordinate system as a reference.             |

**Example**:

```python
# Enter force control drag mode, all directions draggable, reference user coordinate system 1
robot.plugins.ForceDriveMode([1, 1, 1, 1, 1, 1], user=1)

# Only XYZ axes draggable
robot.plugins.ForceDriveMode([1, 1, 1, 0, 0, 0])
```

#### ForceDriveSpeed() - Set Drag Speed

```python
plugins.ForceDriveSpeed(speed) -> str
```

**Parameters**:

| Parameter | Type | Description            |
| --------- | ---- | ---------------------- |
| speed     | int  | Speed ratio (0-100)    |

**Example**:

```python
robot.plugins.ForceDriveSpeed(50)
```

### Force Control Mode

#### FCForceMode() - Enable Force Control Mode

```python
plugins.FCForceMode(direction, force, reference=0, user=-1, tool=-1) -> str
```

**Parameters**:

| Parameter | Type            | Description                                                                                                                                                                                   |
| --------- | --------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| direction | Sequence[int]   | Force control switches for 6 directions [x,y,z,rx,ry,rz], 1 means enabled, 0 means disabled                                                                                                   |
| force     | Sequence[float] | Target forces and torques `[fx, fy, fz, frx, fry, frz]`. Target forces for the translational axes range from −200 to 200 N. Target torques for the rotational axes range from −12 to 12 N·m. When the target force is 0, the robot operates in compliance mode. |
| reference | int             | (Optional) Reference coordinate system type. 0-tool coordinate system (default), 1-user coordinate system                                                                                     |
| user      | int             | (Optional) User coordinate system index (0-50), default -1 means current user coordinate system                                                                                               |
| tool      | int             | (Optional) Tool coordinate system index (0-50), default -1 means current tool coordinate system                                                                                               |

**Example**:

```python
direction = [1, 1, 0, 0, 0, 0]  # X,Y directions force control enabled
force = [100, 100, 0, 0, 0, 0]   # X,Y direction target force 100N
robot.plugins.FCForceMode(direction, force, reference=1, user=1)
```

#### FCSetDeviation() - Set Force Control Displacement and Attitude Deviation

```python
plugins.FCSetDeviation(deviation, control_type=-1) -> str
```

**Parameters**:

| Parameter     | Type            | Description                                                                                                                       |
| --------------| --------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| deviation     | Sequence[float] | Deviation for 6 directions [x,y,z,rx,ry,rz]. Translation deviation (mm) range (0,1000], default 100; rotation deviation (degrees) range (0,360], default 36 |
| control\_type | int             | (Optional) Control type. -1: default; 0: alarm when threshold exceeded; 1: stop searching and continue motion when threshold exceeded |

**Example**:

```python
# Set translation deviation 200mm, rotation deviation 36 degrees
robot.plugins.FCSetDeviation([200, 200, 200, 36, 36, 36])
```

#### FCSetForceLimit() - Set Maximum Force Limit

```python
plugins.FCSetForceLimit(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                   |
| --------- | ----- | ----------------------------------------------------------------------------- |
| x         | float | X direction maximum force limit (N), range (0, 500], default 500              |
| y         | float | Y direction maximum force limit (N), range (0, 500], default 500              |
| z         | float | Z direction maximum force limit (N), range (0, 500], default 500              |
| rx        | float | Rx axis torque limit (N·m), range (0, 50], default 50            |
| ry        | float | Ry axis torque limit (N·m), range (0, 50], default 50            |
| rz        | float | Rz axis torque limit (N·m), range (0, 50], default 50            |

**Example**:

```python
robot.plugins.FCSetForceLimit(500, 500, 500, 50, 50, 50)
```

#### FCSetMass() - Set Inertia Coefficient

```python
plugins.FCSetMass(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                   |
| --------- | ----- | ----------------------------------------------------------------------------- |
| x         | float | X direction inertia coefficient (kg), range (0, 10000], default 20           |
| y         | float | Y direction inertia coefficient (kg), range (0, 10000], default 20           |
| z         | float | Z direction inertia coefficient (kg), range (0, 10000], default 20           |
| rx        | float | Rx direction inertia coefficient (kg·m²), range (0, 10000], default 20       |
| ry        | float | Ry direction inertia coefficient (kg·m²), range (0, 10000], default 20       |
| rz        | float | Rz direction inertia coefficient (kg·m²), range (0, 10000], default 20       |

**Example**:

```python
robot.plugins.FCSetMass(20, 20, 20, 20, 20, 20)
```

#### FCSetStiffness() - Set Stiffness Coefficient

```python
plugins.FCSetStiffness(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                       |
| --------- | ----- | --------------------------------------------------------------------------------- |
| x         | float | X direction stiffness coefficient (N/mm), range [0, 10000], default 30           |
| y         | float | Y direction stiffness coefficient (N/mm), range [0, 10000], default 30           |
| z         | float | Z direction stiffness coefficient (N/mm), range [0, 10000], default 30           |
| rx        | float | Rx direction stiffness coefficient (N/m·deg), range [0, 10000], default 30       |
| ry        | float | Ry direction stiffness coefficient (N/m·deg), range [0, 10000], default 30       |
| rz        | float | Rz direction stiffness coefficient (N/m·deg), range [0, 10000], default 30       |

**Example**:

```python
robot.plugins.FCSetStiffness(30, 30, 30, 30, 30, 30)
```

#### FCSetDamping() - Set Damping Coefficient

```python
plugins.FCSetDamping(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                         |
| --------- | ----- | ----------------------------------------------------------------------------------- |
| x         | float | X direction damping coefficient (N·s/mm), range [0, 1000], default 50              |
| y         | float | Y direction damping coefficient (N·s/mm), range [0, 1000], default 50              |
| z         | float | Z direction damping coefficient (N·s/mm), range [0, 1000], default 50              |
| rx        | float | Rx direction damping coefficient (N·s/m·deg), range [0, 1000], default 50          |
| ry        | float | Ry direction damping coefficient (N·s/m·deg), range [0, 1000], default 50          |
| rz        | float | Rz direction damping coefficient (N·s/m·deg), range [0, 1000], default 50          |

**Example**:

```python
robot.plugins.FCSetDamping(50, 50, 50, 50, 50, 50)
```

#### FCSetForceSpeedLimit() - Set Force Control Adjustment Speed

```python
plugins.FCSetForceSpeedLimit(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                                       |
| --------- | ----- | --------------------------------------------------------------------------------- |
| x         | float | X direction force control adjustment speed (mm/s), range (0, 300], default 20    |
| y         | float | Y direction force control adjustment speed (mm/s), range (0, 300], default 20    |
| z         | float | Z direction force control adjustment speed (mm/s), range (0, 300], default 20    |
| rx        | float | Rx direction force control adjustment speed (deg/s), range (0, 90], default 20    |
| ry        | float | Ry direction force control adjustment speed (deg/s), range (0, 90], default 20    |
| rz        | float | Rz direction force control adjustment speed (deg/s), range (0, 90], default 20    |

**Example**:

```python
robot.plugins.FCSetForceSpeedLimit(20, 20, 20, 20, 20, 20)
```

#### FCSetForce() - Adjust Constant Forces and Torques in Real Time

```python
plugins.FCSetForce(x, y, z, rx, ry, rz) -> str
```

**Parameters**:

| Parameter | Type  | Description                                                               |
| --------- | ----- | ------------------------------------------------------------------------- |
| x         | float | X direction constant force value (N), range [-200, 200]                   |
| y         | float | Y direction constant force value (N), range [-200, 200]                   |
| z         | float | Z direction constant force value (N), range [-200, 200]                   |
| rx        | float | Rx axis constant torque (N·m), range [-12, 12]                  |
| ry        | float | Ry axis constant torque (N·m), range [-12, 12]                  |
| rz        | float | Rz axis constant torque (N·m), range [-12, 12]                  |

**Example**:

```python
# Set 50 N of constant force on each translational axis and 10 N·m of constant torque on each rotational axis
robot.plugins.FCSetForce(50, 50, 50, 10, 10, 10)
```

#### FCOff() - Exit Force Control Mode

```python
plugins.FCOff() -> str
```

**Example**:

```python
# Turn off force control mode
robot.plugins.FCOff()
```

### Conveyor Tracking

#### CnvInit() - Start Conveyor

```python
plugins.CnvInit(index) -> str
```

**Parameters**:

| Parameter | Type | Description              |
| --------- | ---- | ------------------------ |
| index     | int  | Conveyor index (1-3)     |

**Example**:

```python
robot.plugins.CnvInit(1)
```

#### GetCnvObject() - Wait for Workpiece to Enter Picking Area

```python
plugins.GetCnvObject(obj_id) -> str
```

**Parameters**:

| Parameter | Type | Description                                                                              |
| --------- | ---- | ---------------------------------------------------------------------------------------- |
| obj\_id   | int  | Workpiece type (0–15). Use 0 to retrieve information about the first workpiece to enter the queue, regardless of type. |

**Example**:

```python
response = robot.plugins.GetCnvObject(0)
```

#### StartSyncCnv() - Start Conveyor Tracking Function

```python
plugins.StartSyncCnv() -> str
```

**Example**:

```python
robot.plugins.StartSyncCnv()
```

#### CnvMovL() - Conveyor Following Linear Motion

```python
plugins.CnvMovL(index, pose, offset, mode) -> str
```

**Parameters**:

| Parameter | Type            | Description                                                                       |
| --------- | --------------- | --------------------------------------------------------------------------------- |
| index     | int             | Conveyor number. Range: 0~7.                                                      |
| pose      | Sequence[float] | Target pose in the world coordinate system `[x, y, z, rx, ry, rz]`            |
| offset    | Sequence[float] | Conveyor tracking offset position [x, y, z, rx, ry, rz]                           |
| mode      | int             | Run mode. 0: track by pose. 1: track by path.                                     |

**Return value format**: `ErrorID,{},CnvMovL(index, pose, offset, mode);`

**Example**:

```python
robot.plugins.CnvMovL(0, [400, 0, 300, 180, 0, 0], [0, 0, 0, 0, 0, 0], 0)
```

#### CnvMovC() - Conveyor Following Arc Motion

```python
plugins.CnvMovC(via_point, target_point, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=0, coord_type=CoordinateType.CARTESIAN) -> str
```

Documentation prototype: `CnvMovC(P1, P2, user, tool, a, v, cp|r, mode)`
- **P1 = intermediate point via_point** (passed first)
- **P2 = target point target_point** (reached later)

**Parameters**:

| Parameter      | Type              | Default   | Description                                                            |
| -------------- | ----------------- | --------- | ---------------------------------------------------------------------- |
| via\_point     | Sequence[float]   | -         | Arc intermediate point P1 [x,y,z,rx,ry,rz] or [j1..j6]                 |
| target\_point  | Sequence[float]   | -         | Arc end point P2 [x,y,z,rx,ry,rz] or [j1..j6]                          |
| user           | int               | -1        | User coordinate system index (0~50; -1=current)                        |
| tool           | int               | -1        | Tool coordinate system index (0~50; -1=current)                        |
| a              | int               | -1        | Acceleration ratio (1~100; -1=default)                                 |
| v              | int               | -1        | Speed ratio (1~100; -1=default, mutually exclusive with speed, speed takes priority) |
| speed          | int               | -1        | Target speed (mm/s), mutually exclusive with v (speed takes priority)  |
| cp             | int               | -1        | Smooth transition ratio (0~100; -1=not set, mutually exclusive with r) |
| r              | int               | -1        | Smooth transition radius (mm; -1=not set, mutually exclusive with cp)  |
| mode           | int               | 0         | Interpolation mode, default 0                                          |
| coord\_type    | CoordinateType    | CARTESIAN | Point coordinate system type: CARTESIAN (pose=) or JOINT (joint=)      |

**Return value format**: `ErrorID,{flag},CnvMovC(P1,P2,user,tool,a,v,cp|r,mode);`

**Example**:

```python
# Documentation example: CnvMovC(joint={1,2,3,4,5,6},joint={7,8,9,10,11,12},user=1,tool=0,a=20,v=50,cp=100)
# Corresponding SDK call:
via    = [1, 2, 3, 4, 5, 6]
target = [7, 8, 9, 10, 11, 12]
robot.plugins.CnvMovC(via, target, user=1, tool=0, a=20, v=50, cp=100, coord_type=CoordinateType.JOINT)
```

#### StopSyncCnv() - Stop Conveyor Tracking

```python
plugins.StopSyncCnv() -> str
```

**Example**:

```python
robot.plugins.StopSyncCnv()
```

#### SetCnvPointOffset() - Set Conveyor User Coordinate System Offset

```python
plugins.SetCnvPointOffset(x_offset, y_offset) -> str
```

**Example**:

```python
robot.plugins.SetCnvPointOffset(10, 5)
```

#### SetCnvTimeCompensation() - Set Compensation Time

```python
plugins.SetCnvTimeCompensation(compensation) -> str
```

**Example**:

```python
robot.plugins.SetCnvTimeCompensation(100)
```

***

## Status Feedback System

### RobotStatus Object

**Attribute Description**:

| Attribute            | Type          | Description                  |
| -------------------- | ------------- | ---------------------------- |
| robot\_mode          | RobotMode     | Robot mode                   |
| speed\_scaling       | float         | Speed ratio (0-100)          |
| digital\_inputs      | int           | Digital input status (64-bit)|
| digital\_outputs     | int           | Digital output status (64-bit)|
| joint\_state         | JointState    | Joint state                  |
| tool\_vector\_actual | CartesianPose | Current tool pose            |
| tool\_vector\_target | CartesianPose | Target tool pose             |
| current\_tool        | int           | Current tool coordinate system |
| current\_user        | int           | Current user coordinate system |

### JointState Object

| Attribute   | Type         | Description        |
| ----------- | ------------ | ------------------ |
| q\_actual   | list[float]  | Actual joint angles|
| q\_target   | list[float]  | Target joint angles|
| qd\_actual  | list[float]  | Actual joint speeds|
| i\_actual   | list[float]  | Actual joint currents |

### CartesianPose Object

| Attribute | Type  | Description       |
| --------- | ----- | ----------------- |
| x         | float | X coordinate (mm) |
| y         | float | Y coordinate (mm) |
| z         | float | Z coordinate (mm) |
| rx        | float | Rx angle (degrees)|
| ry        | float | Ry angle (degrees)|
| rz        | float | Rz angle (degrees)|

**Methods**:

```python
pose.to_list()  # Convert to list [x, y, z, rx, ry, rz]
```

### Usage Example

```python
# Get status
status = robot.GetStatus()

if status:
    # Robot mode
    print(f"Mode: {status.robot_mode.value}")
    
    # Speed scaling
    print(f"Speed scaling: {status.speed_scaling}%")
    
    # Digital inputs/outputs
    print(f"DI: {bin(status.digital_inputs)}")
    print(f"DO: {bin(status.digital_outputs)}")
    
    # Joint angles
    if status.joint_state:
        print(f"J1: {status.joint_state.q_actual[0]:.2f}")
    
    # Cartesian coordinates
    if status.tool_vector_actual:
        pose = status.tool_vector_actual.to_list()
        print(f"Position: {pose}")
```

***

## Error Code Handling

### Error Code Parsing

```python
from dobot_sdk.api.error_code import parse_error_ids, format_error_messages

# Parse error code response
response = "0,{[1537,2048,2049]},GetErrorID();"
error_ids = parse_error_ids(response)
print(error_ids)  # [1537, 2048, 2049]

# Format error messages
message = format_error_messages(error_ids, lang="zh_CN")
print(message)
```

### Supported Languages

- `zh_CN` - Simplified Chinese
- `en` - English
- `ja` - Japanese

### Error Message Format

```
Error code: 1537
Description: Emergency stop triggered
Cause: Emergency stop button pressed
Solution: Check the emergency stop button and reset

Error code: 2048
Description: J1 axis servo alarm
Cause: J1 axis servo driver alarm
Solution: Check J1 axis servo motor
```

### Common Error Codes

| Error Code | Description         | Cause                     |
| ---------- | ------------------- | ------------------------- |
| 0          | No error            | -                         |
| 1537       | Emergency stop triggered | Emergency stop button pressed |
| 2048       | J1 axis servo alarm | J1 axis servo driver alarm |
| 2049       | J2 axis servo alarm | J2 axis servo driver alarm |
| 2050       | J3 axis servo alarm | J3 axis servo driver alarm |
| 2051       | J4 axis servo alarm | J4 axis servo driver alarm |
| 2052       | J5 axis servo alarm | J5 axis servo driver alarm |
| 2053       | J6 axis servo alarm | J6 axis servo driver alarm |

***

## Complete Examples

### Example 1: Basic Motion Control

```python
from dobot_sdk import DobotRobot, CoordinateType
import time

# Connect to robot
robot = DobotRobot("192.168.1.100")
robot.Connect()

try:
    # Request control
    robot.robot_control.RequestControl()
    
    # Clear alarms
    robot.robot_control.ClearError()
    
    # Enable
    robot.robot_control.EnableRobot()
    print("Robot enabled")
    
    # Wait for stabilization
    time.sleep(2)
    
    # Joint motion
    print("Executing joint motion...")
    robot.motion.MovJ([0, 0, 90, 0, 90, 0], CoordinateType.JOINT)
    time.sleep(3)
    
    # Linear motion
    print("Executing linear motion...")
    robot.motion.MovL([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    time.sleep(3)
    
    # Arc motion
    print("Executing arc motion...")
    mid_point = [400, 100, 300, 180, 0, 0]
    end_point = [400, 200, 300, 180, 0, 0]
    robot.motion.Arc(mid_point, end_point, CoordinateType.CARTESIAN)
    time.sleep(3)
    
    # Return to initial position
    print("Returning to initial position...")
    robot.motion.Home()
    time.sleep(3)
    
    # Disable
    robot.robot_control.DisableRobot()
    print("Robot disabled")
    
finally:
    # Disconnect
    robot.Disconnect()
    print("Disconnected")
```

### Example 2: IO Control

```python
from dobot_sdk import DobotRobot

with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    
    # Set DO1
    robot.io.DO(1, 1)
    print("DO1 turned on")
    
    # Read DI1 status
    response = robot.io.DI(1)
    print(f"DI1 status: {response}")
    
    # Set analog output
    robot.io.AO(1, 50)  # Output 5V
    print("AO1 set to 5V")
    
    # Wait for input signal
    print("Waiting for DI1 to be high...")
    robot.io.DIGroup(0, [1, 0, 0, 0])
    print("DI1 triggered")
    
    # Turn off DO1
    robot.io.DO(1, 0)
    print("DO1 turned off")
```

### Example 3: Status Monitoring

```python
from dobot_sdk import DobotRobot
import time

def on_status(status):
    """Status callback function"""
    if status.tool_vector_actual:
        x = status.tool_vector_actual.x
        y = status.tool_vector_actual.y
        z = status.tool_vector_actual.z
        print(f"\rPosition: X={x:.2f} Y={y:.2f} Z={z:.2f}", end="")

robot = DobotRobot("192.168.1.100")
robot.Connect()
robot.robot_control.RequestControl()
robot.robot_control.ClearError()
robot.robot_control.EnableRobot()

# Start status monitoring
robot.StartFeedbackMonitor(callback=on_status)

try:
    # Execute motion
    robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
    time.sleep(5)
    
    # Get current status
    status = robot.GetStatus()
    if status:
        print(f"\nFinal position: {status.tool_vector_actual.to_list()}")
        
finally:
    robot.StopFeedbackMonitor()
    robot.Disconnect()
```

### Example 4: Force Control Mode

```python
from dobot_sdk import DobotRobot
import time

with DobotRobot("192.168.1.100") as robot:
    # Request control and enable robot
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot()
    
    # Enable force sensor
    robot.plugins.EnableFTSensor(1)
    robot.plugins.SixForceHome()
    
    # Set force control parameters
    direction = [1, 1, 1, 0, 0, 0]   # Enable force control on the X, Y, and Z axes
    force = [0, 0, -5, 0, 0, 0]       # Set a target force of 5 N along the negative Z axis
    
    # Enter force control mode
    robot.plugins.FCForceMode(direction, force)
    
    # Maintain force control state for 5 seconds
    time.sleep(5)
    
    # Stop force control
    robot.robot_control.Stop()
```

***

## Appendix

### Robot Mode Reference Table

| Value | Mode Name        | Description          |
| ----- | ---------------- | -------------------- |
| 1     | Initialization mode | System initializing  |
| 2     | Manual mode      | Manual operation mode|
| 3     | Automatic mode   | Automatic run mode   |
| 4     | Remote mode      | Remote control mode  |
| 9     | Error mode       | Error/alarm status   |

### Common Ports

| Port  | Usage      | Description        |
| ----- | ---------- | ------------------ |
| 29999 | Dashboard  | Command control port |
| 30004 | Feedback   | Status feedback port |
| 30003 | Real-time  | Real-time control port |

### Command Type Description

| Type           | Description                              | Example                        |
| -------------- | ---------------------------------------- | ------------------------------ |
| Queue command  | Added to motion queue, executed sequentially | MovJ, MovL, Arc            |
| Immediate command | Executed immediately, not queued      | GetPose, ClearError, Stop      |

### Coordinate System Description

| Coordinate System | Range | Description            |
| ----------------- | ----- | ---------------------- |
| User coordinate system | 0-9 | User-defined coordinate system |
| Tool coordinate system | 0-9 | Tool end-effector coordinate system |

### Speed and Acceleration Range

| Parameter       | Range  | Description |
| --------------- | ------ | ----------- |
| Speed ratio     | 0-100  | Percentage  |
| Acceleration ratio | 0-100 | Percentage |
| Smooth transition | 0-100 | cp parameter |

### Safety Precautions

1. **Emergency stop button**: Ensure the emergency stop button is easily accessible
2. **Speed limit**: Use low speed during initial debugging
3. **Collision detection**: Enable collision detection function
4. **Safety area**: Ensure no one is within the robot's working range
5. **Payload check**: Set payload parameters correctly

***

**Document Version**: V1.0  
**Generation Date**: May 2026  
**Applicable SDK**: Dobot TCP-IP Python SDK V4.0
