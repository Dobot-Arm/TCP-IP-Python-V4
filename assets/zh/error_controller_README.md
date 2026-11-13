# HTTP 接口说明 - 错误控制器

## 基础信息

- **服务地址**: `http://{机器人IP}:22000`
- **接口协议**: HTTP/JSON
- **默认超时**: 5秒

## 接口列表

### 1. 设置语言接口

**接口**: `POST /interface/language`

**功能**: 设置机器人错误信息的显示语言

**请求示例**:
```bash
POST http://192.168.1.100:22000/interface/language
Content-Type: application/json

{
    "type": "zh_cn"
}
```

**支持的语言代码**:
- `zh_cn` - 简体中文
- `zh_hant` - 繁体中文
- `en` - 英语
- `ja` - 日语
- `de` - 德语
- `es` - 西班牙语
- `ru` - 俄语
- `ko` - 韩语
- `vi` - 越南语
- `fr` - 法语

**响应**: HTTP 200 表示成功

---

### 2. 获取报警信息接口

**接口**: `GET /protocol/getAlarm`

**功能**: 获取机器人当前的报警信息列表

**请求示例**:
```bash
GET http://192.168.1.100:22000/protocol/getAlarm
```

**响应格式**:

**有报警时**:
```json
{
    "errMsg": [
        {
            "id": 1537,
            "level": 2,
            "description": "紧急停止按钮被按下",
            "solution": "请释放急停按钮",
            "mode": "自动",
            "date": "2026-06-15",
            "time": "14:30:25"
        }
    ]
}
```

**无报警时**:
```json
{}
```
或
```json
{
    "errMsg": []
}
```

**响应字段说明**:

| 字段 | 类型 | 说明 |
|------|------|------|
| errMsg | Array | 报警信息数组 |
| id | Integer | 错误码 ID |
| level | Integer | 错误级别 |
| description | String | 错误描述 |
| solution | String | 解决方案 |
| mode | String | 机器人模式 |
| date | String | 报警日期 (YYYY-MM-DD) |
| time | String | 报警时间 (HH:MM:SS) |

---

## 调用流程

推荐的调用流程：

1. 先调用 **设置语言接口** 设置需要的语言
2. 再调用 **获取报警信息接口** 获取对应语言的报警信息

**示例代码**:
```python
import requests

# 1. 设置语言为中文
language_url = "http://192.168.1.100:22000/interface/language"
requests.post(language_url, json={"type": "zh_cn"}, timeout=5)

# 2. 获取报警信息
alarm_url = "http://192.168.1.100:22000/protocol/getAlarm"
response = requests.get(alarm_url, timeout=5)

if response.status_code == 200:
    result = response.json()
    print(result)
```

---

## 错误处理

| HTTP 状态码 | 说明 |
|------------|------|
| 200 | 请求成功 |
| 其他状态码 | 请求失败 |

**网络异常**:
- 连接超时
- 网络不可达
- 端口未开放