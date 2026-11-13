# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
TCP Connection Manager / TCP连接管理器

Provides universal TCP connection functionality, supports auto-reconnect and connection status monitoring
提供通用的TCP连接功能，支持自动重连和连接状态监听
"""

import socket
import threading
import logging
import time
from typing import Optional, Callable
from .exceptions import ConnectionError

logger = logging.getLogger("dobot_sdk")


class DobotConnection:
    """
    TCP Connection Manager / TCP连接管理器
    
    Responsible for establishing and maintaining TCP connections with the robot, supports:
    负责建立和维护与机器人的TCP连接，支持：
    - Receive timeout setting / 接收超时设置
    - Auto-reconnect mechanism / 自动重连机制
    - Exponential backoff reconnect strategy / 指数退避重连策略
    - Connection status change callback / 连接状态变化回调
    """
    
    def __init__(self, ip: str, port: int, buffer_size: int = 144000):
        """
        Initialize connection / 初始化连接
        
        Args:
            ip: Robot IP address / 机器人IP地址
            port: Port number (29999 or 30004) / 端口号(29999或30004)
            buffer_size: Receive buffer size / 接收缓冲区大小
        """
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._connected = False
        
        # Timeout settings / 超时设置
        self._connect_timeout = 5.0      # Connection timeout (seconds) / 连接超时（秒）
        self._receive_timeout = 10.0     # Receive timeout (seconds) / 接收超时（秒）
        
        # Auto-reconnect settings / 自动重连设置
        self._auto_reconnect = False     # Whether to enable auto-reconnect / 是否启用自动重连
        self._reconnect_running = False  # Whether reconnect thread is running / 重连线程是否运行
        self._reconnect_thread = None    # Reconnect thread / 重连线程
        self._reconnect_callback = None  # Connection status callback function / 连接状态回调函数
        
        # Exponential backoff parameters / 指数退避参数
        self._min_reconnect_delay = 1    # Minimum reconnect delay (seconds) / 最小重连延迟（秒）
        self._max_reconnect_delay = 30   # Maximum reconnect delay (seconds) / 最大重连延迟（秒）
        self._reconnect_attempts = 0     # Current reconnect attempt count / 当前重连尝试次数
        
        # Validate port / 验证端口
        if port not in [29999, 30004, 30005, 30006]:
            raise ValueError(f"Invalid port: {port}. Must be 29999, 30004, 30005, or 30006")
    
    def set_timeout(self, connect_timeout: float = None, receive_timeout: float = None):
        """
        Set timeout / 设置超时时间
        
        Args:
            connect_timeout: Connection timeout (seconds), default 5 seconds
                             连接超时时间（秒），默认5秒
            receive_timeout: Receive timeout (seconds), default 10 seconds
                             接收超时时间（秒），默认10秒
        """
        if connect_timeout is not None:
            self._connect_timeout = connect_timeout
        if receive_timeout is not None:
            self._receive_timeout = receive_timeout
    
    def enable_auto_reconnect(self, enable: bool = True, callback: Callable[[bool], None] = None):
        """
        Enable/disable auto-reconnect / 启用/禁用自动重连
        
        Args:
            enable: Whether to enable auto-reconnect / 是否启用自动重连
            Callback function for connection status changes. It receives a Boolean argument that indicates the connection status.
                      连接状态变化回调函数，接收一个布尔参数表示连接状态
        """
        self._auto_reconnect = enable
        self._reconnect_callback = callback
        
        if enable and not self._reconnect_running and not self._connected:
            self._start_reconnect_loop()
    
    def _start_reconnect_loop(self):
        """Start reconnect loop thread / 启动重连循环线程"""
        if self._reconnect_running:
            return
        
        self._reconnect_running = True
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop,
            daemon=True
        )
        self._reconnect_thread.start()
        logger.info(f"Auto-reconnect started: {self.ip}:{self.port}")  # 自动重连已启动 / Auto-reconnect started
    
    def _stop_reconnect_loop(self):
        """Stop reconnect loop thread / 停止重连循环线程"""
        self._reconnect_running = False
        if self._reconnect_thread:
            self._reconnect_thread.join(timeout=2.0)
            self._reconnect_thread = None
        self._reconnect_attempts = 0
        logger.info(f"Auto-reconnect stopped: {self.ip}:{self.port}")  # 自动重连已停止 / Auto-reconnect stopped
    
    def _reconnect_loop(self):
        """Reconnect loop using exponential backoff strategy / 重连循环，使用指数退避策略"""
        while self._reconnect_running:
            try:
                # Calculate exponential backoff delay / 计算指数退避延迟
                delay = min(
                    self._min_reconnect_delay * (2 ** self._reconnect_attempts),
                    self._max_reconnect_delay
                )
                
                logger.info(f"Waiting {delay:.1f}s before reconnect attempt: {self.ip}:{self.port}")  # 等待X秒后尝试重连 / Waiting X seconds before attempting to reconnect
                time.sleep(delay)
                
                if not self._reconnect_running:
                    break
                
                # Attempt to reconnect / 尝试重新连接
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.settimeout(self._connect_timeout)
                self._socket.connect((self.ip, self.port))
                
                # Set receive buffer and timeout / 设置接收缓冲区和超时
                self._socket.setsockopt(
                    socket.SOL_SOCKET, 
                    socket.SO_RCVBUF, 
                    self.buffer_size
                )
                self._socket.settimeout(self._receive_timeout)
                
                self._connected = True
                self._reconnect_attempts = 0  # Reset reconnect count / 重置重连计数
                
                logger.info(f"TCP reconnected successfully: {self.ip}:{self.port}")  # TCP重连成功 / TCP reconnected
                
                # Call connection success callback / 调用连接成功回调
                if self._reconnect_callback:
                    try:
                        self._reconnect_callback(True)
                    except Exception as e:
                        logger.error(f"Connection success callback failed: {str(e)}")  # 连接成功回调执行失败 / Connection success callback failed
                
                # Reconnect successful, exit reconnect loop / 重连成功，退出重连循环
                self._stop_reconnect_loop()
                
            except socket.timeout:
                self._reconnect_attempts += 1
                logger.warning(f"Reconnect timeout (attempt {self._reconnect_attempts}): {self.ip}:{self.port}")  # 重连超时 / Reconnect timeout
                
            except socket.error as e:
                self._reconnect_attempts += 1
                logger.warning(f"Reconnect failed (attempt {self._reconnect_attempts}): {self.ip}:{self.port} - {str(e)}")  # 重连失败 / Reconnect failed
                
            except Exception as e:
                self._reconnect_attempts += 1
                logger.error(f"Reconnect exception (attempt {self._reconnect_attempts}): {self.ip}:{self.port} - {str(e)}")  # 重连异常 / Reconnect exception
    
    def connect(self, timeout: float = None):
        """
        Establish TCP connection / 建立TCP连接
        
        Args:
            timeout: Connection timeout (seconds), defaults to the value set during initialization
                     连接超时时间（秒），默认为初始化时设置的值
            
        Raises:
            ConnectionError: Raised when connection fails / 连接失败时抛出
        """
        actual_timeout = timeout if timeout is not None else self._connect_timeout
        
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(actual_timeout)
            self._socket.connect((self.ip, self.port))
            
            # Set receive buffer and timeout / 设置接收缓冲区和超时
            self._socket.setsockopt(
                socket.SOL_SOCKET, 
                socket.SO_RCVBUF, 
                self.buffer_size
            )
            self._socket.settimeout(self._receive_timeout)
            
            self._connected = True
            self._reconnect_attempts = 0  # Reset reconnect count / 重置重连计数
            
            logger.info(f"TCP connected successfully: {self.ip}:{self.port}")  # TCP连接成功 / TCP connected
            
            # Call connection success callback / 调用连接成功回调
            if self._reconnect_callback:
                try:
                    self._reconnect_callback(True)
                except Exception as e:
                    logger.error(f"Connection success callback failed: {str(e)}")  # 连接成功回调执行失败 / Connection success callback failed
            
        except socket.timeout:
            logger.error(f"TCP connection timeout: {self.ip}:{self.port}")  # TCP连接超时 / TCP connection timeout
            self._trigger_disconnect_callback()
            raise ConnectionError("Connection timeout", ip=self.ip, port=self.port)  # 连接超时 / Connection timeout
        except socket.error as e:
            logger.error(f"TCP connection failed: {self.ip}:{self.port} - {str(e)}")  # TCP连接失败 / TCP connection failed
            self._trigger_disconnect_callback()
            raise ConnectionError(f"Connection failed: {str(e)}", ip=self.ip, port=self.port)  # 连接失败 / Connection failed
    
    def _trigger_disconnect_callback(self, force: bool = False):
        """
        Trigger connection disconnect callback / 触发连接断开回调
        
        Args:
            force: Whether to force trigger the callback (without checking connection status)
                   是否强制触发回调（不检查连接状态）
        """
        if self._reconnect_callback and (force or self._connected):
            try:
                self._reconnect_callback(False)
            except Exception as e:
                logger.error(f"Connection disconnect callback failed: {str(e)}")  # 连接断开回调执行失败 / Connection disconnect callback failed
    
    def disconnect(self):
        """Close connection / 关闭连接"""
        # Stop auto-reconnect / 停止自动重连
        self._stop_reconnect_loop()
        
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
                self._socket.close()
                logger.info(f"TCP connection disconnected: {self.ip}:{self.port}")  # TCP连接已断开 / TCP disconnected
            except Exception as e:
                logger.warning(f"Error during TCP disconnect: {str(e)}")  # TCP断开连接时发生错误 / Error during TCP disconnect
            finally:
                self._socket = None
                self._connected = False
                # Force trigger disconnect callback (user-initiated disconnect) / 强制触发断开回调（用户主动断开）
                self._trigger_disconnect_callback(force=True)
    
    def send_text(self, text: str):
        """
        Send text command (for Dashboard) / 发送文本命令（Dashboard用）
        
        Args:
            text: Command string to send / 要发送的命令字符串
        """
        if not self._connected or not self._socket:
            raise ConnectionError("Not connected to robot")  # 未连接到机器人 / Not connected to robot
        
        try:
            # Encode and send directly, add newline / 直接编码发送，添加换行符
            command = text if text.endswith('\n') else text + '\n'
            self._socket.send(command.encode('utf-8'))
            logger.debug(f"Sent command: {text.strip()}")  # 发送命令 / Sent command
        except Exception as e:
            self._connected = False
            logger.error(f"Failed to send command: {text.strip()} - {str(e)}")  # 发送命令失败 / Failed to send command
            self._trigger_disconnect_callback()
            
            # If auto-reconnect is enabled, start reconnect loop / 如果启用了自动重连，启动重连循环
            if self._auto_reconnect and not self._reconnect_running:
                self._start_reconnect_loop()
            
            raise ConnectionError(f"Send failed: {str(e)}")  # 发送失败 / Send failed
    
    def receive_text(self, buffer_size: int = 1024) -> str:
        """
        Receive text response (for Dashboard) / 接收文本响应（Dashboard用）
        
        Args:
            buffer_size: Receive buffer size / 接收缓冲区大小
            
        Returns:
            str: Received string / 接收到的字符串
        """
        if not self._connected or not self._socket:
            raise ConnectionError("Not connected to robot")  # 未连接到机器人 / Not connected to robot
        
        try:
            data = self._socket.recv(buffer_size)
            if not data:
                raise ConnectionError("Connection closed")  # 连接已关闭 / Connection closed
            
            response = data.decode('utf-8').strip()
            logger.debug(f"Received response: {response}")  # 接收响应 / Received response
            return response
        except socket.timeout:
            self._connected = False
            logger.error(f"Receive response timeout: {self.ip}:{self.port}")  # 接收响应超时 / Receive response timeout
            self._trigger_disconnect_callback()
            
            # If auto-reconnect is enabled, start reconnect loop / 如果启用了自动重连，启动重连循环
            if self._auto_reconnect and not self._reconnect_running:
                self._start_reconnect_loop()
            
            raise ConnectionError("Receive timeout")  # 接收超时 / Receive timeout
        except Exception as e:
            self._connected = False
            logger.error(f"Failed to receive response: {str(e)}")  # 接收响应失败 / Failed to receive response
            self._trigger_disconnect_callback()
            
            # If auto-reconnect is enabled, start reconnect loop / 如果启用了自动重连，启动重连循环
            if self._auto_reconnect and not self._reconnect_running:
                self._start_reconnect_loop()
            
            raise ConnectionError(f"Receive failed: {str(e)}")  # 接收失败 / Receive failed
    
    def receive_bytes(self, buffer_size: int = 144000) -> bytes:
        """
        Receive raw bytes (for Feedback) / 接收原始字节（Feedback用）
        
        Args:
            buffer_size: Receive buffer size / 接收缓冲区大小
            
        Returns:
            bytes: Received raw bytes / 接收到的原始字节
        """
        if not self._connected or not self._socket:
            raise ConnectionError("Not connected to robot")  # 未连接到机器人 / Not connected to robot
        
        try:
            data = self._socket.recv(buffer_size)
            if not data:
                raise ConnectionError("Connection closed")  # 连接已关闭 / Connection closed
            
            logger.debug(f"Received byte data: {len(data)} bytes")  # 接收字节数据 / Received byte data
            return data
        except socket.timeout:
            self._connected = False
            logger.error(f"Receive byte data timeout: {self.ip}:{self.port}")  # 接收字节数据超时 / Receive byte data timeout
            self._trigger_disconnect_callback()
            
            # If auto-reconnect is enabled, start reconnect loop / 如果启用了自动重连，启动重连循环
            if self._auto_reconnect and not self._reconnect_running:
                self._start_reconnect_loop()
            
            raise ConnectionError("Receive timeout")  # 接收超时 / Receive timeout
        except Exception as e:
            self._connected = False
            logger.error(f"Failed to receive byte data: {str(e)}")  # 接收字节数据失败 / Failed to receive byte data
            self._trigger_disconnect_callback()
            
            # If auto-reconnect is enabled, start reconnect loop / 如果启用了自动重连，启动重连循环
            if self._auto_reconnect and not self._reconnect_running:
                self._start_reconnect_loop()
            
            raise ConnectionError(f"Receive failed: {str(e)}")  # 接收失败 / Receive failed
    
    def send_receive_text(self, text: str, recv_size: int = 1024) -> str:
        """
        Send and receive response (thread-safe) / 发送并接收响应（线程安全）
        
        Args:
            text: String to send / 发送的字符串
            recv_size: Receive buffer size / 接收缓冲区大小
            
        Returns:
            str: Received response / 接收到的响应
        """
        with self._lock:
            self.send_text(text)
            return self.receive_text(recv_size)
    
    @property
    def is_connected(self) -> bool:
        """Check connection status / 检查连接状态"""
        return self._connected
    
    @property
    def reconnect_enabled(self) -> bool:
        """Check if auto-reconnect is enabled / 检查是否启用了自动重连"""
        return self._auto_reconnect
    
    def __enter__(self):
        """Context manager entry / 上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit / 上下文管理器出口"""
        self.disconnect()
        return False
    
    def __del__(self):
        """Destructor / 析构函数"""
        self.disconnect()
