# Demo Programs

This directory contains robot control demo programs adapted for the new SDK.

## File Structure

```
demo/
├── DobotDemo.py      # Robot control core class
├── main.py           # Basic demo entry point
├── ui.py             # Graphical control interface
├── main_UI.py        # UI interface entry point
├── z_Cnv_test.py     # Conveyor tracking test
├── z_servo_test.py   # Servo dynamic following test
└── README.md         # Documentation
```

## Function Description

| File | Function | Run Command |
|------|----------|-------------|
| `main.py` | Basic connection and status monitoring | `python main.py` |
| `main_UI.py` | Graphical robot control interface | `python main_UI.py` |
| `z_Cnv_test.py` | Conveyor tracking function demo | `python z_Cnv_test.py` |
| `z_servo_test.py` | ServoP dynamic trajectory following demo | `python z_servo_test.py` |

## Preparations Before Use

1. **Install dependencies**: Ensure project dependencies are installed
   ```bash
   pip install -r requirements.txt
   ```

2. **Configure robot IP**: Modify the `ROBOT_IP` variable in each file to the actual robot IP address

3. **Run permissions**: Ensure the robot has entered TCP control mode

## Quick Start

### Method 1: Command Line Basic Demo
```bash
cd demo
python main.py
```

### Method 2: Graphical Interface Control
```bash
cd demo
python main_UI.py
```

## Notes

1. Ensure the robot is properly connected to the network before running
2. Request TCP control mode for first-time use
3. It is recommended to test motion functions within a safe area
4. Conveyor testing requires hardware support
