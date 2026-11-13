# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Alarm information data model / 报警信息数据模型
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import List


@dataclass
class ErrorInfo:
    """
    Single alarm information / 单条报警信息
    
    Parsed from data returned by the GetError interface
    从GetError接口返回的数据解析得到
    """
    id: int
    level: int
    description: str
    solution: str
    mode: str
    date: str
    time: str
    
    @property
    def timestamp_str(self) -> str:
        """Get full timestamp string / 获取完整时间戳字符串"""
        return f"{self.date} {self.time}"
    
    def __str__(self) -> str:
        return (
            f"Error ID: {self.id}\n"
            f"Level: {self.level}\n"
            f"Description: {self.description}\n"
            f"Solution: {self.solution}\n"
            f"Time: {self.timestamp_str}"
        )


@dataclass
class ErrorReport:
    """
    Alarm report / 报警报告
    
    Contains all current alarm information
    包含所有当前报警信息
    """
    errors: List[ErrorInfo]
    timestamp: datetime = field(default_factory=datetime.now)
    
    @property
    def has_errors(self) -> bool:
        """Whether there are alarms / 是否有报警"""
        return len(self.errors) > 0
    
    @property
    def error_count(self) -> int:
        """Number of alarms / 报警数量"""
        return len(self.errors)
    
    def get_critical_errors(self) -> List[ErrorInfo]:
        """Get critical-level alarms (level >= 5) / 获取严重级别的报警（level >= 5）"""
        return [e for e in self.errors if e.level >= 5]
    
    def __str__(self) -> str:
        if not self.has_errors:
            return "Robot status normal, no alarm information"  # 机器人状态正常，无报警信息 / The robot status is normal, and there is no alarm information.
        
        result = f"Found {self.error_count} alarm(s)\n"  # 发现X个报警 / Found X alarms
        result += "=" * 50 + "\n"
        
        for i, error in enumerate(self.errors, 1):
            result += f"Alarm {i}:\n"  # 报警 / Alarm
            result += f"  ID: {error.id}\n"
            result += f"  Level: {error.level}\n"  # 级别 / Level
            result += f"  Description: {error.description}\n"  # 描述 / Description
            result += f"  Solution: {error.solution}\n"  # 解决方案 / Solution
            result += f"  Time: {error.timestamp_str}\n"  # 时间 / Time
            result += "-" * 30 + "\n"
        
        return result
