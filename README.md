<div align="center">
  
<img src="assets/CR3A.png" alt="Dobot SDK V4" style="max-width: 400px; margin-bottom: 20px;" />

<h1>Dobot TCP/IP Python SDK V4</h1>

**Dobot Robot Python SDK V4**  
High-Performance Robot Control Framework Based on TCP/IP  
Supports CRA, E6, CRAF, NovaLite and Other V4 Series Robots

[English](README.md) · [简体中文](README.zh-CN.md) · [📖 Full API Documentation](assets/en/SDK_API_Complete.md)

[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-blue?style=flat-square)](https://github.com/dobot-cn/TCP-IP-Python-V4-main)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green?style=flat-square)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)](LICENSE)
[![GitHub Issues](https://img.shields.io/github/issues/dobot-cn/TCP-IP-Python-V4-main?style=flat-square)](https://github.com/dobot-cn/TCP-IP-Python-V4-main/issues)

</div>

---

## Quick Start

### Requirements

| Requirement | Version |
|-------------|---------|
| **Python** | 3.8+ |
| **numpy** | ≥1.19.0 |
| **requests** | ≥2.25.0 |

### Installation

**Development Mode Installation:**

```bash
git clone -b feature/v4-optimization https://github.com/dobot-cn/TCP-IP-Python-V4-main.git
pip install -e .
```

**Direct Import (No Installation):**

Suitable for development/debugging scenarios or when you do not want to modify the Python environment.

```python
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot
```

**Directory Structure Requirement:**
```
TCP-IP-Python-V4-main/
├── dobot_sdk/                  ← SDK core package
├── examples/                   ← Examples directory (place your script here)
│   └── your_script.py
└── README.md
```

**Notes:**
- Assumes the script is located in the `examples/` directory, automatically getting the project root
- If the script is in a different location, adjust the number of `os.path.dirname()` calls

**Update or Reinstall:**

```bash
# Uninstall the package
pip uninstall dobot_sdk -y

# Reinstall with latest changes
pip install -e .

# Verify installation
pip show dobot_sdk
```

### Network Connection

| Config | Description |
|--------|-------------|
| **Robot IP** | 192.168.1.100 (default) |
| **Dashboard Port** | 29999 |
| **Feedback Port** | 30004/30005/30006 |
| **PC IP** | Must be on the same subnet as the robot. |

---

## Usage Examples

### 1. Basic Connection

```python
from dobot_sdk import DobotRobot

ROBOT_IP = "192.168.1.100"

# Use a context manager (recommended)
with DobotRobot(ROBOT_IP) as robot:
    # Initialization
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot(load=1.0)

    # Perform operations...
    robot.robot_control.SpeedFactor(50)

    # Disable robot
    robot.robot_control.DisableRobot()
```

### 2. Motion Control

```python
from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType

with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.EnableRobot()

    # Cartesian space motion
    robot.motion.MovJ(
        pose=[400, 0, 300, 180, 0, 0],
        coord_type=CoordinateType.CARTESIAN
    )

    # Linear motion
    robot.motion.MovL(
        pose=[400, 100, 300, 180, 0, 0],
        coord_type=CoordinateType.CARTESIAN
    )
```

### 3. IO Control

```python
from dobot_sdk import DobotRobot

with DobotRobot("192.168.1.100") as robot:
    robot.robot_control.EnableRobot()

    # Digital output (see the robot arm's TCP/IP interface guide for DO(index, status))
    robot.io.DO(1, 1)    # Turn on DO1
    robot.io.DO(1, 0)    # Turn off DO1

    # Read input
    di_status = robot.io.DI(1)  # Read DI1
```

---

## Running Examples

```bash
# Run example code
cd examples
python 01_basic_connection.py
python 02_motion_control.py
python 03_error_monitor.py

# Run GUI
cd demo
python main_UI.py
```

---

## Project Structure

```
TCP-IP-Python-V4-main/
├── dobot_sdk/              # SDK Core Package
│   ├── api/                # API Interface Layer
│   ├── core/               # Core Communication Layer
│   ├── protocol/           # Protocol Layer
│   └── models/             # Data Models
├── demo/                   # Demo Programs
├── examples/               # Example Code
├── tests/                  # Test Code
├── assets/                 # Resource Files
│   ├── CR3A.png            # Robot Image
│   ├── en/                 # English Documentation
│   │   ├── SDK_API_Complete.md        # Full API Documentation
│   │   ├── error_controller_README.md # HTTP Error Interface Documentation
│   │   └── DOBOT TCP_IP Remote Control Interface Guide V4.6.6_en.pdf
│   └── zh/                 # Chinese Documentation
│       ├── SDK_API文档_完整版.md       # 完整 API 文档
│       ├── error_controller_README.md  # HTTP 错误接口说明
│       └── DOBOT TCP_IP二次开发接口文档_V4.6.6_cn.pdf
├── pyproject.toml          # Project Config
├── requirements.txt        # Dependencies
├── README.md               # English README
└── README.zh-CN.md         # Chinese README
```

---

## API Reference

### Main Control Class

```python
from dobot_sdk import DobotRobot, CoordinateType
```

| Module | Description |
|--------|-------------|
| `robot.robot_control` | Basic control (enable, mode, coordinate system, status query, etc.) |
| `robot.motion` | Motion control (MovJ/MovL/Arc/Circle, etc.) |
| `robot.io` | IO control |
| `robot.communication` | Communication control |
| `robot.plugins` | Plugin module (force control, conveyor tracking, etc.) |

### Logging Control

```python
from dobot_sdk import get_logger, set_log_level, get_log_directory

# Set log level: DEBUG, INFO, WARNING, ERROR
set_log_level("DEBUG")

# Get logger instance
logger = get_logger()

# Get log file directory
log_dir = get_log_directory()  # Returns: dobot_sdk/logs/
```

| Function | Description |
|----------|-------------|
| `set_log_level(level)` | Set log level (DEBUG/INFO/WARNING/ERROR) |
| `get_logger()` | Get SDK logger instance |
| `get_log_directory()` | Get log file storage directory |

**Log Features:**
- Auto-rotate log files (max 5 files, 10MB each)
- Cross-platform compatible (Windows/Ubuntu)
- Structured log format with timestamp and module info
- Automatically logs API calls, commands, and responses

### Connection Management

```python
from dobot_sdk import DobotRobot

# Create robot with custom timeout settings
robot = DobotRobot(
    "192.168.1.100",
    connect_timeout=10.0,   # Connection timeout in seconds
    receive_timeout=15.0    # Receive timeout in seconds
)

# Enable auto-reconnect with connection status callback
def on_connection_status(is_connected):
    print(f"Connection status: {'Connected' if is_connected else 'Disconnected'}")

robot.EnableAutoReconnect(enable=True, callback=on_connection_status)

# Check connection status
if robot.IsConnected:
    print("Robot is connected")
```

| Function | Description |
|----------|-------------|
| `robot.SetTimeout(connect_timeout, receive_timeout)` | Set timeout settings |
| `robot.EnableAutoReconnect(enable, callback)` | Enable/disable auto-reconnect |
| `robot.IsConnected` | Check connection status (property) |

**Connection Features:**
- **Connection Timeout**: Default 5 seconds; **Receive Timeout**: Default 10 seconds, prevents blocking on receive operations
- **Auto-reconnect**: Automatically attempts to reconnect when connection is lost
- **Exponential Backoff**: Reconnect delay increases exponentially (1s, 2s, 4s, ..., max 30s)
- **Connection Callback**: Get notified when connection status changes

### Example Code

For more detailed examples, see:
- `examples/` - Functionally categorized example code
- `demo/` - Complete demo programs

---

## Supported Models

| Series | Models |
|--------|--------|
| **CRA Series** | CR3A, CR5, CR10, CR16, etc. |
| **E6 Series** | E6, E6 Pro, etc. |
| **CRAF Series** | CRAF5, etc. |
| **NovaLite Series** | NovaLite, etc. |
| **Other V4 Series** | Robots that support TCP/IP |

---

## Documentation

| Document | Description |
|----------|-------------|
| [SDK_API_Complete.md](assets/en/SDK_API_Complete.md) | Full API Documentation (English) |
| [SDK_API文档_完整版.md](assets/zh/SDK_API文档_完整版.md) | 完整 API 文档（中文） |
| [DOBOT TCP_IP Remote Control Interface Guide](assets/en/DOBOT%20TCP_IP%20Remote%20Control%20Interface%20Guide%20V4.6.6_en.pdf) | Official Interface Document (English PDF) |
| [DOBOT TCP_IP二次开发接口文档](assets/zh/DOBOT%20TCP_IP%E4%BA%8C%E6%AC%A1%E5%BC%80%E5%8F%91%E6%8E%A5%E5%8F%A3%E6%96%87%E6%A1%A3_V4.6.6_cn.pdf) | 官方接口文档（中文 PDF） |
| [error_controller_README.md](assets/en/error_controller_README.md) | HTTP Error Interface Documentation (English) |

---

## Important Notes

> ⚠️ **Safety First**: Make sure the robot is in a safe position before running

1. **Network Configuration**: Ensure IP addresses are in the same subnet
2. **Port Occupancy**: Ensure ports 29999 and 30004 are not occupied
3. **Robot Mode**: Ensure robot is in TCP/IP control mode
4. **Coordinate Type**: All motion instructions must explicitly specify CoordinateType

---

## Version Information

| Info | Content |
|------|---------|
| **Current Version** | 2.0.0 |
| **Python Requirement** | ≥3.8 |
| **Main Dependencies** | numpy≥1.19.0, requests≥2.25.0 |

---

## License

[MIT License](LICENSE)

<div align="center">

Built by Dobot-Arm

</div>
