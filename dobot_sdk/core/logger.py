# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Logging module / 日志记录模块
Provides unified logging functionality for the SDK, supports cross-platform operation (Windows/Ubuntu)
提供SDK统一的日志记录功能，支持跨平台运行（Windows/Ubuntu）
"""

import logging
import os
import sys
from logging.handlers import RotatingFileHandler
from typing import Optional
from datetime import datetime


class DobotLogger:
    """
    Dobot Robot SDK Logger / 越疆机器人SDK日志记录器
    
    Features / 特性:
        - Supports multiple log levels: DEBUG, INFO, WARNING, ERROR
        - 支持多日志级别: DEBUG, INFO, WARNING, ERROR
        - Outputs to both console and log file simultaneously
        - 同时输出到控制台和日志文件
        - Automatic log file rotation (keep up to 5 files, max 10MB each)
        - 日志文件自动轮转（最多保留5个文件，每个最大10MB）
        - Cross-platform compatible (Windows/Ubuntu)
        - 跨平台兼容（Windows/Ubuntu）
        - Structured log format
        - 结构化日志格式
    """
    
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, log_level: str = "INFO", log_dir: Optional[str] = None):
        """
        Initialize the logger / 初始化日志记录器
        
        Args:
            log_level: Log level, options: "DEBUG", "INFO", "WARNING", "ERROR"
                       日志级别，可选 "DEBUG", "INFO", "WARNING", "ERROR"
            log_dir: Log file directory, defaults to the logs folder under the SDK installation directory
                     日志文件目录，默认为SDK安装目录下的logs文件夹
        """
        if hasattr(self, '_initialized') and self._initialized:
            return
        
        self._initialized = True
        
        # Get log directory / 获取日志目录
        if log_dir is None:
            # Default log directory: SDK installation directory/logs / 默认日志目录: SDK安装目录/logs
            sdk_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.log_dir = os.path.join(sdk_dir, 'logs')
        else:
            self.log_dir = log_dir
        
        # Ensure log directory exists / 确保日志目录存在
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Log file name format: dobot_sdk_20260615.log / 日志文件名格式: dobot_sdk_20260615.log
        today = datetime.now().strftime("%Y%m%d")
        self.log_file = os.path.join(self.log_dir, f"dobot_sdk_{today}.log")
        
        # Create logger / 创建日志记录器
        self.logger = logging.getLogger("dobot_sdk")
        self.logger.setLevel(getattr(logging, log_level.upper()))
        self.logger.propagate = False
        
        # Remove existing handlers to avoid duplicates / 移除已存在的处理器，避免重复
        for handler in self.logger.handlers[:]:
            self.logger.removeHandler(handler)
        
        # Console handler / 控制台处理器
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.DEBUG)
        
        # File handler (with rotation) / 文件处理器（带轮转）
        file_handler = RotatingFileHandler(
            self.log_file,
            maxBytes=10 * 1024 * 1024,  # 10MB
            backupCount=5,              # Keep up to 5 backups / 最多保留5个备份
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        
        # Log format / 日志格式
        log_format = logging.Formatter(
            '%(asctime)s - %(levelname)s - %(module)s.%(funcName)s:%(lineno)d - %(message)s'
        )
        console_handler.setFormatter(log_format)
        file_handler.setFormatter(log_format)
        
        # Add handlers / 添加处理器
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)
    
    def get_logger(self) -> logging.Logger:
        """Get logger instance / 获取日志记录器实例"""
        return self.logger
    
    @property
    def log_directory(self) -> str:
        """Get log file directory / 获取日志文件目录"""
        return self.log_dir


# Global log decorator / 全局日志装饰器
def log_api_call(func):
    """
    Decorator: Record API call logs / 装饰器：记录API调用日志
    
    Automatically records / 自动记录：
        - Called function name / 调用的函数名
        - Passed parameters / 传入的参数
        - Return value / 返回值
        - Execution time / 执行时间
        - Exception information (if any) / 异常信息（如果发生）
    """
    import time
    
    def wrapper(*args, **kwargs):
        logger = logging.getLogger("dobot_sdk")
        start_time = time.time()
        
        # Get call information / 获取调用信息
        class_name = ""
        if len(args) > 0 and hasattr(args[0], '__class__'):
            class_name = args[0].__class__.__name__ + "."
        
        func_name = class_name + func.__name__
        
        # Format parameters / 格式化参数
        args_str = []
        for i, arg in enumerate(args[1:], 1):  # Skip self / 跳过self
            args_str.append(f"arg{i}={arg!r}")
        for k, v in kwargs.items():
            args_str.append(f"{k}={v!r}")
        
        args_str = ", ".join(args_str)
        
        # Record call information / 记录调用信息
        logger.info(f"API call: {func_name}({args_str})")  # API调用 / API call
        
        try:
            result = func(*args, **kwargs)
            elapsed = (time.time() - start_time) * 1000
            
            # Record return value (limit length) / 记录返回值（限制长度）
            result_str = str(result)
            if len(result_str) > 500:
                result_str = result_str[:500] + "..."
            
            logger.debug(f"API return: {func_name} -> {result_str} (elapsed: {elapsed:.2f}ms)")  # API返回 / API return; 耗时 / elapsed
            
            return result
        
        except Exception as e:
            elapsed = (time.time() - start_time) * 1000
            logger.error(f"API exception: {func_name} -> {type(e).__name__}: {str(e)} (elapsed: {elapsed:.2f}ms)", exc_info=True)  # API异常 / API exception; 耗时 / elapsed
            raise
    
    return wrapper


def get_logger() -> logging.Logger:
    """Convenience function: Get global logger / 便捷函数：获取全局日志记录器"""
    return DobotLogger().get_logger()


def set_log_level(log_level: str):
    """
    Set global log level / 设置全局日志级别
    
    Args:
        log_level: "DEBUG", "INFO", "WARNING", "ERROR"
    """
    logger = DobotLogger(log_level=log_level).get_logger()
    logger.setLevel(getattr(logging, log_level.upper()))
    for handler in logger.handlers:
        handler.setLevel(logging.DEBUG)


def get_log_directory() -> str:
    """Get log file storage directory / 获取日志文件存储目录"""
    return DobotLogger().log_directory
