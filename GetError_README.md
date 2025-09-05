# GetError接口使用说明

## 概述

`GetError`接口是在 `dobot_api.py`中新增的功能，用于通过HTTP请求获取机器人的报警信息。该接口支持多种语言，并返回JSON格式的报警数据。

## 功能特性

- ✅ 支持10种语言的报警信息显示
- ✅ 通过HTTP GET请求获取报警数据
- ✅ 自动设置语言偏好
- ✅ 返回结构化的JSON数据
- ✅ 完善的错误处理机制

## 支持的语言

| 语言代码    | 语言名称              |
| ----------- | --------------------- |
| `zh_cn`   | 简体中文              |
| `zh_hant` | 繁体中文              |
| `en`      | English (英语)        |
| `ja`      | 日本語 (日语)         |
| `de`      | Deutsch (德语)        |
| `vi`      | Tiếng Việt (越南语) |
| `es`      | Español (西班牙语)   |
| `fr`      | Français (法语)      |
| `ko`      | 한국어 (韩语)         |
| `ru`      | Русский (俄语) |

## 接口说明

### 方法签名

```python
def GetError(self, language="zh_cn"):
    """
    获取机器人报警信息
  
    Args:
        language (str): 语言设置，默认为"zh_cn"
  
    Returns:
        dict: 报警信息字典，格式如下：
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

### 使用的HTTP接口

1. **设置语言接口**

   - **URL**: `http://ip:22000/interface/language`
   - **方法**: POST
   - **数据格式**: `{"type": "语言代码"}`
2. **获取报警信息接口**

   - **URL**: `http://ip:22000/protocol/getAlarm`
   - **方法**: GET
   - **返回格式**: JSON

## 基本使用示例

### 1. 简单使用

```python
from dobot_api import DobotApiDashboard

# 创建连接
dashboard = DobotApiDashboard("192.168.5.1", 29999)

# 获取中文报警信息
error_info = dashboard.GetError("zh_cn")

# 检查是否有报警
if error_info and "errMsg" in error_info:
    errors = error_info["errMsg"]
    if errors:
        print(f"发现 {len(errors)} 个报警")
        for error in errors:
            print(f"报警ID: {error['id']}")
            print(f"描述: {error['description']}")
            print(f"解决方案: {error['solution']}")
    else:
        print("无报警信息")

# 关闭连接
dashboard.close()
```

### 2. 多语言支持

```python
# 获取英文报警信息
error_info_en = dashboard.GetError("en")

# 获取日文报警信息
error_info_ja = dashboard.GetError("ja")

# 获取德文报警信息
error_info_de = dashboard.GetError("de")
```

### 3. 错误处理

```python
try:
    error_info = dashboard.GetError("zh_cn")
    if error_info is None:
        print("获取报警信息失败")
    elif "errMsg" not in error_info:
        print("返回数据格式异常")
    else:
        # 处理正常数据
        pass
except Exception as e:
    print(f"发生异常: {e}")
```

## 完整示例程序

项目中提供了两个示例程序：

1. **`test_get_error.py`** - 基本功能测试
2. **`get_error_example.py`** - 完整的使用示例，包含监控类

### 运行测试程序

```bash
# 基本测试
python test_get_error.py

# 完整示例
python get_error_example.py
```

## 返回数据格式

### 成功响应

```json
{
    "errMsg": [
        {
            "id": 1537,
            "level": 1,
            "description": "急停按钮被拍下",
            "solution": "恢复急停按钮并清错。若仍然告警，则确认急停按钮是否损坏，可更换急停按钮",
            "mode": "安全控制器错误",
            "date": "2025-01-09",
            "time": "10:30:15"
        }
    ]
}
```

### 无报警时

```json
{
    "errMsg": []
}
```

### 错误情况

- 网络连接失败：返回 `None`
- HTTP请求失败：返回 `None`
- JSON解析失败：返回 `None`

## 注意事项

1. **网络连接**：确保机器人网络连接正常，端口22000可访问
2. **IP地址**：使用建立TCP连接时的相同IP地址
3. **语言设置**：每次调用都会先设置语言，然后获取报警信息
4. **错误处理**：建议在实际使用中添加适当的错误处理逻辑
5. **连接管理**：使用完毕后记得关闭连接

## 故障排除

### 常见问题

1. **连接超时**

   - 检查机器人IP地址是否正确
   - 确认网络连接是否正常
   - 验证端口22000是否开放
2. **返回None**

   - 检查HTTP请求是否成功
   - 确认机器人固件版本是否支持该接口
   - 查看网络防火墙设置
3. **语言设置无效**

   - 确认语言代码拼写正确
   - 检查机器人是否支持该语言
   - 尝试使用默认语言"zh_cn"

### 调试建议

1. 先使用 `test_get_error.py`进行基本功能测试
2. 检查机器人Web界面是否可以正常访问
3. 使用浏览器直接访问HTTP接口进行验证
4. 查看控制台输出的错误信息

## 更新日志

- **v1.0** (2025-09-05)
  - 初始版本发布
  - 支持10种语言
  - 完整的错误处理机制
  - 提供示例程序和文档
