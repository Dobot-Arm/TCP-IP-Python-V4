# TCP-IP-Python-V4 Project Documentation

## Project Overview

This project is the Dobot Robot TCP-IP-CR-Python-V4 secondary development API program, used to control Dobot robots through TCP/IP protocol. The project provides complete robot control interfaces, including motion control, status monitoring, alarm handling, and other functions.

## Environment Requirements

### Python Version

- Python 3.6 or higher

### Required Libraries

```bash
# Basic numerical computation library
pip install numpy

# JSON data processing (Python built-in, no installation required)
# import json

# Network communication (Python built-in, no installation required)
# import socket

# Multi-threading support (Python built-in, no installation required)
# import threading

# Time processing (Python built-in, no installation required)
# import time

# Regular expressions (Python built-in, no installation required)
# import re

# GUI interface library (if using ui.py)
pip install tkinter  # Usually comes with Python
```

### Network Configuration Requirements

- Local machine IP address needs to be set to 192.168.X.X network segment
- Robot needs to be switched to TCP/IP mode
- Ensure ports 29999 and 30004 are not occupied

## Main Program Files and Functions

### 1. main.py

**Function**: Project main entry file

- Demonstrates basic robot connection and control flow
- Contains complete robot operation examples
- Suitable for beginners to understand project structure

### 2. dobot_api.py

**Function**: Core API interface file

- **DobotApi**: Basic communication class, handles TCP connections
- **DobotApiDashboard**: Robot control interface class
  - Robot enable/disable
  - Motion control commands (MovJ, MovL, Arc, etc.)
  - Status query and setting
  - Alarm information acquisition (including newly added GetError interface)
- **DobotApiFeedBack**: Status feedback class
  - Real-time acquisition of robot status information
  - Monitor robot operation mode
  - Get current command ID
- **MyType**: Data type definitions
- **alarm_controller**: Controller alarm handling
- **alarm_servo**: Servo alarm handling

### 3. ui.py

**Function**: Graphical user interface program

- Provides visual robot control interface
- Integrates robot connection, motion control, status display and other functions
- Supports real-time display of robot status and alarm information
- Prioritizes using GetError interface to get alarm information, falls back to original method if failed

### 4. Test and Example Files

#### get_error_example.py

**Function**: GetError interface usage example

- Provides RobotErrorMonitor class for alarm monitoring
- Demonstrates how to get and process multi-language alarm information
- Contains functionality to save alarm information to files
- Comments in both Chinese and English

### 5. Documentation Files

#### GetError_README.md

**Function**: GetError interface Chinese documentation

- Detailed explanation of GetError interface usage
- Contains interface parameters, return values, example code, etc.
- Provides troubleshooting and precautions

#### GetError_README_EN.md

**Function**: GetError interface English documentation

- English version of GetError_README.md
- Convenient for international users to understand and use

## Project Directory Structure

TCP-IP-Python-V4/
├── main.py                    # Main program entry
├── dobot_api.py               # Core API interface
├── ui.py                      # Graphical interface program
├── PythonExample.py           # Python examples
├── get_error_example.py       # GetError usage example
├── GetError_README.md         # GetError Chinese documentation
├── GetError_README_EN.md      # GetError English documentation
├── README.md                  # Project documentation
└── files/                     # Other support files

## Quick Start

### 1. Environment Setup

```bash
# Clone the project
git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git

# Install dependencies
pip install numpy
```

### 2. Network Configuration

- Set local machine IP to 192.168.X.X network segment
- Ensure robot is in TCP/IP mode

### 3. Run Programs

# Run main program
python main.py

# Or run graphical interface
python main_UI.py


## Common Problem Solutions

### 1. ModuleNotFoundError: No module named 'numpy'

**Solution**: Install numpy library

```bash
pip install numpy
```

### 2. Connection refused, IP:Port has been occupied

**Solution**: Check if port 29999 is occupied, close the program occupying that port

### 3. Control Mode Is Not Tcp

**Solution**: Switch robot mode to TCP/IP mode in DobotStudio Pro

### 4. Robot Status Abnormal

| Output Message                        | Robot Status        | Solution                           |
| ------------------------------------- | ------------------- | ---------------------------------- |
| Command execution failed              | Command failed      | Check command parameters and robot status |
| The robot is in an error state       | Robot error state   | Clear alarms and retry             |
| The robot is in emergency stop state | Emergency stop state| Release emergency stop button      |
| The robot is in power down state     | Power down state    | Power on the robot                 |

## Precautions

1. **Safety First**: Ensure the robot is in a safe position before running examples to prevent collisions
2. **Network Configuration**: Ensure correct network configuration with IP addresses in the same network segment
3. **Port Occupation**: Ensure ports 29999 and 30004 are not occupied by other programs
4. **Robot Mode**: Ensure the robot is in TCP/IP control mode
5. **Permission Issues**: Some operations may require administrator privileges

## Technical Support

If you encounter problems, please refer to: Project README.md documentation

- GetError related documentation
- Example code and test programs
- Dobot official technical support

---

**Version**: V4
**Update Date**: 2025-9-5
**Maintainer**: dobot_futingxing
