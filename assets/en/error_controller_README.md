# HTTP Interface Guide - Error Controller

## Basic Information

- **Service Address**: `http://{RobotIP}:22000`
- **Protocol**: HTTP/JSON
- **Default Timeout**: 5 seconds

## Interface List

### 1. Set Language Interface

**Interface**: `POST /interface/language`

**Function**: Set the display language for robot error messages

**Request Example**:
```bash
POST http://192.168.1.100:22000/interface/language
Content-Type: application/json

{
    "type": "zh_cn"
}
```

**Supported Language Codes**:
- `zh_cn` - Simplified Chinese
- `zh_hant` - Traditional Chinese
- `en` - English
- `ja` - Japanese
- `de` - German
- `es` - Spanish
- `ru` - Russian
- `ko` - Korean
- `vi` - Vietnamese
- `fr` - French

**Response**: HTTP 200 indicates success

---

### 2. Get Alarm Information Interface

**Interface**: `GET /protocol/getAlarm`

**Function**: Retrieve the robot's current alarm information list

**Request Example**:
```bash
GET http://192.168.1.100:22000/protocol/getAlarm
```

**Response Format**:

**When alarms exist**:
```json
{
    "errMsg": [
        {
            "id": 1537,
            "level": 2,
            "description": "Emergency stop button pressed",
            "solution": "Please release the emergency stop button",
            "mode": "Auto",
            "date": "2026-06-15",
            "time": "14:30:25"
        }
    ]
}
```

**When no alarms exist**:
```json
{}
```
or
```json
{
    "errMsg": []
}
```

**Response Field Description**:

| Field | Type | Description |
|-------|------|-------------|
| errMsg | Array | Alarm information array |
| id | Integer | Error code ID |
| level | Integer | Error level |
| description | String | Error description |
| solution | String | Solution |
| mode | String | Robot mode |
| date | String | Alarm date (YYYY-MM-DD) |
| time | String | Alarm time (HH:MM:SS) |

---

## Call Flow

Recommended call flow:

1. First call the **Set Language Interface** to set the desired language
2. Then call the **Get Alarm Information Interface** to retrieve alarm information in the corresponding language

**Example Code**:
```python
import requests

# 1. Set language to Chinese
language_url = "http://192.168.1.100:22000/interface/language"
requests.post(language_url, json={"type": "zh_cn"}, timeout=5)

# 2. Get alarm information
alarm_url = "http://192.168.1.100:22000/protocol/getAlarm"
response = requests.get(alarm_url, timeout=5)

if response.status_code == 200:
    result = response.json()
    print(result)
```

---

## Error Handling

| HTTP Status Code | Description |
|------------------|-------------|
| 200 | Request succeeded |
| Other status codes | Request failed |

**Network Exceptions**:
- Connection timeout
- Network unreachable
- Port not open
