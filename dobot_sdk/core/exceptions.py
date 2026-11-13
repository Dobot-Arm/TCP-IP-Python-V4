# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Custom exception classes / 自定义异常类
"""


class DobotError(Exception):
    """Dobot SDK base exception / Dobot SDK基础异常"""
    pass


class ConnectionError(DobotError):
    """Connection exception / 连接异常"""
    def __init__(self, message="Connection failed", ip=None, port=None):  # 连接失败 / Connection failed
        self.ip = ip
        self.port = port
        super().__init__(f"{message} (IP: {ip}, Port: {port})")


class ProtocolError(DobotError):
    """Protocol exception / 协议异常"""
    def __init__(self, message="Protocol error", command=None):  # 协议错误 / Protocol error
        self.command = command
        super().__init__(f"{message} (Command: {command})")


class RobotError(DobotError):
    """Robot exception / 机器人异常"""
    def __init__(self, message="Robot error", error_code=None):  # 机器人错误 / Robot error
        self.error_code = error_code
        super().__init__(f"{message} (Code: {error_code})")


class TimeoutError(DobotError):
    """Timeout exception / 超时异常"""
    pass
