# GetError Interface Usage Guide

## Overview

The `GetError` interface is a newly added feature in `dobot_api.py` for retrieving robot alarm information through HTTP requests. This interface supports multiple languages and returns alarm data in JSON format.

## Features

- ✅ Supports alarm information display in 10 languages
- ✅ Retrieves alarm data via HTTP GET requests
- ✅ Automatically sets language preferences
- ✅ Returns structured JSON data
- ✅ Comprehensive error handling mechanism

## Supported Languages

| Language Code | Language Name       |
| ------------- | ------------------- |
| `zh_cn`     | Simplified Chinese  |
| `zh_hant`   | Traditional Chinese |
| `en`        | English             |
| `ja`        | Japanese            |
| `de`        | German              |
| `vi`        | Vietnamese          |
| `es`        | Spanish             |
| `fr`        | French              |
| `ko`        | Korean              |
| `ru`        | Russian             |

## Interface Description

### Method Signature

```python
def GetError(self, language="zh_cn"):
    """
    Get robot alarm information
  
    Args:
        language (str): Language setting, default is "zh_cn"
  
    Returns:
        dict: Alarm information dictionary with the following format:
        {
            "errMsg": [
                {
                    "id": xxx,
                    "level": xxx,
                    "description": "xxx",
                    "solution": "xxx",
                    "mode": "xxx",
                    "date": "xxxx",
                    "time": "xxxx"
                }
            ]
        }
    """
```

### HTTP Interfaces Used

1. **Language Setting Interface**

   - **URL**: `http://ip:22000/interface/language`
   - **Method**: POST
   - **Data Format**: `{"type": "language_code"}`
2. **Get Alarm Information Interface**

   - **URL**: `http://ip:22000/protocol/getAlarm`
   - **Method**: GET
   - **Return Format**: JSON

## Basic Usage Examples

### 1. Simple Usage

```python
from dobot_api import DobotApiDashboard

# Create connection
dashboard = DobotApiDashboard("192.168.5.1", 29999)

# Get Chinese alarm information
error_info = dashboard.GetError("zh_cn")

# Check for alarms
if error_info and "errMsg" in error_info:
    errors = error_info["errMsg"]
    if errors:
        print(f"Found {len(errors)} alarm(s)")
        for error in errors:
            print(f"Alarm ID: {error['id']}")
            print(f"Description: {error['description']}")
            print(f"Solution: {error['solution']}")
    else:
        print("No alarm information")

# Close connection
dashboard.close()
```

### 2. Multi-language Support

```python
# Get English alarm information
error_info_en = dashboard.GetError("en")

# Get Japanese alarm information
error_info_ja = dashboard.GetError("ja")

# Get German alarm information
error_info_de = dashboard.GetError("de")
```

### 3. Error Handling

```python
try:
    error_info = dashboard.GetError("zh_cn")
    if error_info is None:
        print("Failed to get alarm information")
    elif "errMsg" not in error_info:
        print("Abnormal return data format")
    else:
        # Handle normal data
        pass
except Exception as e:
    print(f"Exception occurred: {e}")
```

## Complete Example Programs

The project provides two example programs:

1. **`test_get_error.py`** - Basic functionality test
2. **`get_error_example.py`** - Complete usage example with monitoring class

### Running Test Programs

```bash
# Basic test
python test_get_error.py

# Complete example
python get_error_example.py
```

## Return Data Format

### Successful Response

```json
{
    "errMsg": [
        {
            "id": 1537,
            "level": 1,
            "description": "E-Stop button pressed",
            "solution": "Release the E-Stop button and clear the error. If the error persists, check if the button is faulty and replace it if needed.",
            "mode": "Safety controller error",
            "date": "2025-01-09",
            "time": "10:30:15"
        }
    ]
}
```

### No Alarms

```json
{
    "errMsg": []
}
```

### Error Cases

- Network connection failure: Returns `None`
- HTTP request failure: Returns `None`
- JSON parsing failure: Returns `None`

## Important Notes

1. **Network Connection**: Ensure the robot network connection is normal and port 22000 is accessible
2. **IP Address**: Use the same IP address as when establishing TCP connection
3. **Language Setting**: Each call will first set the language, then retrieve alarm information
4. **Error Handling**: It's recommended to add appropriate error handling logic in actual usage
5. **Connection Management**: Remember to close the connection after use

## Troubleshooting

### Common Issues

1. **Connection Timeout**

   - Check if the robot IP address is correct
   - Confirm network connection is normal
   - Verify port 22000 is open
2. **Returns None**

   - Check if HTTP request is successful
   - Confirm robot firmware version supports this interface
   - Check network firewall settings
3. **Language Setting Invalid**

   - Confirm language code spelling is correct
   - Check if robot supports the language
   - Try using default language "zh_cn"

### Debugging Suggestions

1. First use `test_get_error.py` for basic functionality testing
2. Check if robot web interface is accessible normally
3. Use browser to directly access HTTP interface for verification
4. Check console output for error messages

## Update Log

- **v1.0** (2025-09-05)
  - Initial version release
  - Support for 10 languages
  - Complete error handling mechanism
  - Provided example programs and documentation
