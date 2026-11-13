# Example Code Guide

This directory contains example code for the DOBOT SDK, categorized by functionality:

## Example List

| No. | File | Function Description |
|:---:|------|----------------------|
| 00 | `00_logging_demo.py` | Logging system demo |
| 01 | `01_basic_connection.py` | Basic connection and enabling |
| 02 | `02_motion_control.py` | Motion control (joint/linear/arc/circle) |
| 03 | `03_error_monitor.py` | Error code monitoring and status query |
| 04 | `04_io_control.py` | IO control (digital/analog/end-effector IO) |
| 05 | `05_coordinate_system.py` | Coordinate system setup (user/tool coordinate systems) |
| 06 | `06_force_and_conveyor.py` | Force control and conveyor tracking |
| 07 | `07_status_monitor.py` | Status monitoring |
| 08 | `08_multi_thread_control.py` | Multi-thread control |

## Running Examples

```bash
# Run basic connection example
python examples/01_basic_connection.py

# Run motion control example
python examples/02_motion_control.py

# Run error monitoring example
python examples/03_error_monitor.py

# Run IO control example
python examples/04_io_control.py

# Run coordinate system setup example
python examples/05_coordinate_system.py

# Run force control and conveyor tracking example
python examples/06_force_and_conveyor.py

# Run status monitoring example
python examples/07_status_monitor.py

# Run multi-thread control example
python examples/08_multi_thread_control.py
```

## Usage Notes

1. **Modify IP address**: Each example file has a `ROBOT_IP` variable at the beginning. Please modify it according to the actual robot IP.
2. **Safety first**: Before running motion examples, ensure there is sufficient safe space around the robot.
3. **TCP mode**: Ensure the robot has been switched to TCP/IP control mode.
4. **Dependency installation**: Ensure necessary dependencies are installed.
   ```bash
   pip install numpy requests
   ```

## Example Structure Description

All examples follow a unified structural pattern:

```python
from dobot_sdk import DobotRobot

def main():
    ROBOT_IP = "192.168.1.100"
    
    try:
        with DobotRobot(ROBOT_IP) as robot:
            # 1. Request control mode
            robot.robot_control.RequestControl()
            
            # 2. Clear alarms
            robot.robot_control.ClearError()
            
            # 3. Enable robot
            robot.robot_control.EnableRobot(load=1.0)
            
            # 4. Perform operations...
            # ...
            
            # 5. Disable robot
            robot.robot_control.DisableRobot()
            
    except Exception as e:
        print(f"Error: {e}")
```

## Module Description

| Module | Description |
|--------|-------------|
| `robot.robot_control` | Basic control (enable, mode, coordinate system, status query) |
| `robot.motion` | Motion control (MovJ/MovL/Arc/Circle, etc.) |
| `robot.io` | IO control (digital/analog input/output) |
| `robot.communication` | Communication control (register operations) |
| `robot.plugins` | Plugin module (force control, conveyor tracking) |
