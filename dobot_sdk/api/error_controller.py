# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Error code handling module / 错误码处理模块
Provides functionality to retrieve robot alarm information via the HTTP interface
提供通过HTTP接口获取机器人报警信息的功能
"""

import requests
import json
from typing import List, Dict, Optional, Any

# Supported language list / 支持的语言列表
SUPPORTED_LANGUAGES = [
    ("中文", "zh_cn"),
    ("English", "en"),
    ("日本語", "ja"),
    ("Deutsch", "de"),
    ("Español", "es"),
    ("Русский", "ru"),
    ("한국어", "ko"),
    ("繁体中文", "zh_hant"),
    ("Tiếng Việt", "vi"),
    ("Français", "fr")
]


class ErrorController:
    """
    Error controller class / 错误控制器类
    
    Retrieve robot alarm information via the HTTP interface.
    通过HTTP接口获取机器人报警信息
    Example / 示例:
        error_ctrl = ErrorController("192.168.1.100")
        error_info = error_ctrl.GetError("zh_cn")
        formatted_error = error_ctrl.GetErrorFormatted("zh_cn")
    """
    
    def __init__(self, ip: str):
        """
        Initialize error controller / 初始化错误控制器
        
        Args:
            ip: Robot IP address / 机器人IP地址
        """
        self.ip = ip
    
    def SetLanguage(self, language: str = "zh_cn") -> bool:
        """
        Set robot language / 设置机器人语言
        
        Args:
            language: Language code, supported languages / 语言代码，支持的语言
                     "zh_cn" - Simplified Chinese / 简体中文
                     "zh_hant" - Traditional Chinese / 繁体中文
                     "en" - English / 英语
                     "ja" - Japanese / 日语
                     "de" - German / 德语
                     "vi" - Vietnamese / 越南语
                     "es" - Spanish / 西班牙语
                     "fr" - French / 法语
                     "ko" - Korean / 韩语
                     "ru" - Russian / 俄语
        
        Returns:
            Whether the setting was successful / 是否设置成功
        """
        try:
            language_url = f"http://{self.ip}:22000/interface/language"
            language_data = {"type": language}
            
            response = requests.post(language_url, json=language_data, timeout=5)
            return response.status_code == 200
        except requests.exceptions.RequestException as e:
            print(f"Failed to set language: {e}")  # 设置语言失败 / Failed to set language
            return False
    
    def GetError(self, language: str = "zh_cn") -> Dict[str, Any]:
        """
        Get robot alarm information / 获取机器人报警信息
        
        Args:
            language: Language setting, default "zh_cn" / 语言设置，默认为"zh_cn"
        
        Returns:
            dict: Alarm information dictionary with the following structure / 报警信息字典，格式如下：
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
            If there are no alarms, the method returns {"errMsg": []}. / 如果没有报警，则返回 {"errMsg": []}。
        """
        try:
            # First set the language / 首先设置语言
            self.SetLanguage(language)
            
            # Get alarm information / 获取报警信息
            alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
            response = requests.get(alarm_url, timeout=5)
            
            if response.status_code == 200:
                try:
                    result = response.json()
                    # If an empty object is returned, convert it to the standard format. / 如果返回空对象，则将其转换为标准格式。
                    if result == {} or result is None:
                        return {"errMsg": []}
                    return result
                except json.JSONDecodeError:
                    # If the response is empty or is not in JSON format, treat it as having no alarms. / 如果响应为空或不是 JSON 格式，则视为无报警。
                    return {"errMsg": []}
            else:
                print(f"Failed to get alarm information: HTTP {response.status_code}")  # 获取报警信息失败 / Failed to get alarm info
                return {"errMsg": []}
                
        except requests.exceptions.RequestException as e:
            print(f"HTTP request exception: {e}")  # HTTP请求异常 / HTTP request exception
            return {"errMsg": []}
        except Exception as e:
            print(f"An unexpected error occurred while retrieving alarm information: {e}")  # 获取报警信息时发生意外错误 / Unexpected error while retrieving alarm information
            return {"errMsg": []}
    
    def GetErrorFormatted(self, language: str = "zh_cn") -> str:
        """
        Get formatted robot alarm information / 获取格式化的机器人报警信息
        
        Args:
            language: Language setting / 语言设置
        
        Returns:
            str: Formatted alarm information string / 格式化的报警信息字符串
        """
        error_info = self.GetError(language)
        return self._format_error_messages(error_info)
    
    def _format_error_messages(self, error_info: Dict[str, Any]) -> str:
        """
        Format error messages / 格式化错误信息
        
        Args:
            error_info: Alarm information dictionary returned by get_error() / get_error() 返回的报警信息字典
        Returns:
            str: Formatted error message string / 格式化的错误信息字符串
        """
        err_msg_list = error_info.get("errMsg", [])
        
        if not err_msg_list:
            return "No alarm information"  # 无报警信息 / No alarm information
        
        messages = []
        for err in err_msg_list:
            error_id = err.get("id", "Unknown")  # 未知 / Unknown
            level = err.get("level", "")
            description = err.get("description", "")
            solution = err.get("solution", "")
            mode = err.get("mode", "")
            date = err.get("date", "")
            time = err.get("time", "")
            
            msg = f"Error code {error_id}"  # 错误码 / Error code
            if level:
                msg += f"\nLevel: {level}"  # 级别 / Level
            if description:
                msg += f"\nDescription: {description}"  # 描述 / Description
            if solution:
                msg += f"\nSolution: {solution}"  # 解决方案 / Solution
            if mode:
                msg += f"\nMode: {mode}"  # 模式 / Mode
            if date and time:
                msg += f"\nTime: {date} {time}"  # 时间 / Time
            
            messages.append(msg)
        
        return "\n\n".join(messages)
    
    # Backward compatibility aliases (snake_case -> PascalCase) / 向后兼容别名
    set_language = SetLanguage
    get_error = GetError
    get_error_formatted = GetErrorFormatted


def parse_error_ids(error_response: Optional[str]) -> List[int]:
    """
    Parse the response string returned by the TCP/IP GetErrorID command.
    解析 TCP/IP GetErrorID 指令返回的响应字符串。
    
    Args:
        error_response: Error code response returned by robot, format "0,{[1537,2048,2049]},GetErrorID();"
                        机器人返回的错误码响应，格式为"0,{[1537,2048,2049]},GetErrorID();"
    
    Returns:
        List of alarm IDs / 解析出的错误码列表
    """
    if not error_response:
        return []
    
    # A successful GetErrorID response starts with '0,'.
    # GetErrorID 的成功应答以 '0,' 开头。
    if isinstance(error_response, str):
        if not error_response.strip().startswith('0,'):
            # GetErrorID 命令执行失败，或应答格式无法识别。
            print(f"GetErrorID returned a non-success or unrecognized response: {error_response}")
            return []
    
    try:
        # Remove trailing "GetErrorID();" / 移除末尾的"GetErrorID();"
        if error_response.endswith('GetErrorID();'):
            error_response = error_response[:-len('GetErrorID();')].strip()
        
        # Format is ,{[1537,2048,2049]} / 格式为,{[1537,2048,2049]}
        # Extract the list part inside braces / 提取大括号内的列表部分
        start = error_response.find('{[')
        end = error_response.find(']}')
        
        if start != -1 and end != -1:
            list_str = error_response[start+2:end]
            error_ids = [int(x.strip()) for x in list_str.split(',') if x.strip()]
            return error_ids
        else:
            # Try to parse it directly as an integer. / 尝试将其直接解析为整数。
            return [int(error_response.strip())]
    except ValueError as e:
        # Return an empty list if parsing fails. / 如果解析失败，则返回空列表。
        print(f"Failed to parse error code: {e}")  # 解析错误码失败 / Failed to parse error code
        return []
    except Exception as e:
        print(f"Failed to parse error code: {e}")  # 解析错误码失败 / Failed to parse error code
        return []


# Keep the original standalone function interface for backward compatibility
# 保留原有的独立函数接口，向后兼容
def set_language(ip: str, language: str = "zh_cn") -> bool:
    """
    Set robot language (compatible with the legacy API) / 设置机器人语言（兼容旧接口）
    
    Args:
        ip: Robot IP address / 机器人IP地址
        language: Language code / 语言代码
    
    Returns:
        Whether the setting was successful / 是否设置成功
    """
    return ErrorController(ip).SetLanguage(language)


def get_error(ip: str, language: str = "zh_cn") -> Dict[str, Any]:
    """
    Get robot alarm information (compatible with the legacy API) / 获取机器人报警信息（兼容旧接口）
    
    Args:
        ip: Robot IP address / 机器人IP地址
        language: Language setting / 语言设置
    
    Returns:
        Alarm information dictionary / 报警信息字典
    """
    return ErrorController(ip).GetError(language)


def format_error_messages_from_http(error_info: Dict[str, Any]) -> str:
    """
    Format error information returned by the HTTP interface (compatible with the legacy API).
    格式化HTTP接口返回的错误信息（兼容旧接口）
    
    Args:
        error_info: Alarm information dictionary returned by get_error() / get_error() 返回的报警信息字典
    Returns:
        Formatted error message string / 格式化的错误信息字符串
    """
    err_msg_list = error_info.get("errMsg", [])
    
    if not err_msg_list:
        return "No alarm information"  # 无报警信息 / No alarm information
    
    messages = []
    for err in err_msg_list:
        error_id = err.get("id", "Unknown")  # 未知 / Unknown
        level = err.get("level", "")
        description = err.get("description", "")
        solution = err.get("solution", "")
        mode = err.get("mode", "")
        date = err.get("date", "")
        time = err.get("time", "")
        
        msg = f"Error code {error_id}"  # 错误码 / Error code
        if level:
            msg += f"\nLevel: {level}"  # 级别 / Level
        if description:
            msg += f"\nDescription: {description}"  # 描述 / Description
        if solution:
            msg += f"\nSolution: {solution}"  # 解决方案 / Solution
        if mode:
            msg += f"\nMode: {mode}"  # 模式 / Mode
        if date and time:
            msg += f"\nTime: {date} {time}"  # 时间 / Time
        
        messages.append(msg)
    
    return "\n\n".join(messages)
