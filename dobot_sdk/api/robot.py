# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Main robot controller
机器人主控制器
Integrates all modules to provide a unified control interface
整合所有模块，提供统一的控制接口"""

import threading
import time
import logging
from typing import Optional, Callable
from ..core.connection import DobotConnection
from ..protocol.feedback import FeedbackParser
from ..models.status import RobotStatus
from .motion import Motion, CoordinateType
from .io import IO
from .communication import Communication
from .plugins import Plugins
from .robot_control import RobotControl
from .error_controller import ErrorController

logger = logging.getLogger("dobot_sdk")


class DobotRobot:
    """
    Dobot robot main control class
    越疆机器人控制主类
    Example:
        with DobotRobot("192.168.1.100") as robot:
            robot.robot_control.EnableRobot()
            pose = [400, 0, 300, 180, 0, 0]
            robot.motion.MovJ(pose, CoordinateType.CARTESIAN)
    """

    def __init__(self, ip: str,
                 dashboard_port: int = 29999,
                 feedback_port: int = 30004,
                 feedback_port_30005: int = None,
                 feedback_port_30006: int = None,
                 connect_timeout: float = 5.0,
                 receive_timeout: float = 10.0):
        """
        Initialize the robot object
        初始化机器人对象

        Args:
            ip: Robot IP address
                机器人IP地址
            dashboard_port: Dashboard port (default 29999)
                            Dashboard端口 (默认29999)
            feedback_port: Feedback port (default 30004)
                           Feedback端口 (默认30004)
                              - 30004: Real-time feedback, every 8ms
                                       实时反馈，每8ms
                              - 30005: Configuration feedback, every 200ms
                                       配置反馈，每200ms
                              - 30006: Configuration feedback, every 1000ms
                                       配置反馈，每1000ms
            feedback_port_30005: Optional second feedback port (default None)
                                 可选的第二个反馈端口(默认None)
            feedback_port_30006: Optional third feedback port (default None)
                                 可选的第三个反馈端口(默认None)
            connect_timeout: Connection timeout in seconds, default 5 seconds
                             连接超时时间（秒），默认5秒
            receive_timeout: Receive timeout in seconds, default 10 seconds
                             接收超时时间（秒），默认10秒
        """
        self.ip = ip
        self._connect_timeout = connect_timeout
        self._receive_timeout = receive_timeout

        # 连接 - Dashboard / Connect - Dashboard
        self._dashboard_conn = DobotConnection(ip, dashboard_port)
        self._dashboard_conn.set_timeout(connect_timeout, receive_timeout)

        # 连接 - Feedback 端口 / Connect - Feedback port
        self._feedback_conn = DobotConnection(ip, feedback_port)
        self._feedback_conn.set_timeout(connect_timeout, receive_timeout)

        # 可选的额外反馈端口连接 / Optional additional feedback port connections
        self._feedback_conn_30005 = None
        if feedback_port_30005:
            self._feedback_conn_30005 = DobotConnection(ip, feedback_port_30005)
            self._feedback_conn_30005.set_timeout(connect_timeout, receive_timeout)

        self._feedback_conn_30006 = None
        if feedback_port_30006:
            self._feedback_conn_30006 = DobotConnection(ip, feedback_port_30006)
            self._feedback_conn_30006.set_timeout(connect_timeout, receive_timeout)

        # 协议解析器（仅Feedback需要） / Protocol parser (Feedback only)
        self._feedback_parser = FeedbackParser()

        # 5个模块 / 5 modules
        self.motion = Motion(self._dashboard_conn)                    # 运动模块 / Motion module
        self.io = IO(self._dashboard_conn)                            # IO模块 / IO module
        self.communication = Communication(self._dashboard_conn)       # 通讯模块 / Communication module
        self.plugins = Plugins(self._dashboard_conn)                  # 插件模块 / Plugins module
        self.robot_control = RobotControl(self._dashboard_conn)      # Robot控制模块 / Robot control module
        self.error = ErrorController(ip)                              # 错误控制模块 / Error control module

        # 状态监控 / Status monitoring
        self._latest_status: Optional[RobotStatus] = None
        self._feedback_thread: Optional[threading.Thread] = None
        self._feedback_running = False
        self._callback: Optional[Callable] = None

        # 连接状态回调 / Connection status callback
        self._connection_callback: Optional[Callable[[bool], None]] = None

    def SetTimeout(self, connect_timeout: float = None, receive_timeout: float = None):
        """
        Set timeout values
        设置超时时间

        Args:
            connect_timeout: Connection timeout in seconds
                             连接超时时间（秒）
            receive_timeout: Receive timeout in seconds
                             接收超时时间（秒）
        """
        self._dashboard_conn.set_timeout(connect_timeout, receive_timeout)
        self._feedback_conn.set_timeout(connect_timeout, receive_timeout)
        if self._feedback_conn_30005:
            self._feedback_conn_30005.set_timeout(connect_timeout, receive_timeout)
        if self._feedback_conn_30006:
            self._feedback_conn_30006.set_timeout(connect_timeout, receive_timeout)

        if connect_timeout is not None:
            self._connect_timeout = connect_timeout
        if receive_timeout is not None:
            self._receive_timeout = receive_timeout

    def EnableAutoReconnect(self, enable: bool = True, callback: Callable[[bool], None] = None):
        """
        Enable or disable auto-reconnect.
        启用/禁用自动重连功能

        Args:
            enable: Whether to enable auto-reconnect
                    是否启用自动重连
            callback: Callback that receives a boolean indicating the connection status.
                      连接状态变化回调函数，接收一个布尔参数表示连接状态
                      True: Connected, False: Disconnected
                      True: 连接成功，False: 连接断开
        """
        self._connection_callback = callback

        def connection_status_handler(is_connected: bool):
            logger.info(f"Robot connection status changed: {'Connected' if is_connected else 'Disconnected'}")  # 机器人连接状态变化: 已连接/已断开 / Robot connection status changed: Connected/Disconnected
            if self._connection_callback:
                try:
                    self._connection_callback(is_connected)
                except Exception as e:
                    logger.error(f"Connection status callback failed: {str(e)}")  # 连接状态回调执行失败 / Connection status callback execution failed

        self._dashboard_conn.enable_auto_reconnect(enable, connection_status_handler)
        self._feedback_conn.enable_auto_reconnect(enable, connection_status_handler)
        if self._feedback_conn_30005:
            self._feedback_conn_30005.enable_auto_reconnect(enable, connection_status_handler)
        if self._feedback_conn_30006:
            self._feedback_conn_30006.enable_auto_reconnect(enable, connection_status_handler)

        logger.info(f"Auto-reconnect {'enabled' if enable else 'disabled'}")  # 自动重连已启用/已禁用 / Auto-reconnect enabled/disabled

    def Connect(self, timeout: float = None):
        """
        Connect to the robot
        连接到机器人

        Args:
            timeout: Connection timeout in seconds, defaults to the value set during initialization
                     连接超时时间（秒），默认为初始化时设置的值
        """
        actual_timeout = timeout if timeout is not None else self._connect_timeout

        logger.info(f"Connecting to robot {self.ip}...")  # 正在连接到机器人 / Connecting to robot
        self._dashboard_conn.connect(actual_timeout)
        self._feedback_conn.connect(actual_timeout)

        # 连接可选的额外反馈端口 / Connect optional additional feedback ports
        if self._feedback_conn_30005:
            self._feedback_conn_30005.connect(actual_timeout)
        if self._feedback_conn_30006:
            self._feedback_conn_30006.connect(actual_timeout)

        logger.info(f"Robot connected successfully: {self.ip}")  # 机器人连接成功 / Robot connected successfully

    @property
    def IsConnected(self) -> bool:
        """
        Check robot connection status
        检查机器人连接状态

        Returns:
            bool: True means connected, False means not connected
                  True表示已连接，False表示未连接
        """
        return (self._dashboard_conn.is_connected and
                self._feedback_conn.is_connected and
                (not self._feedback_conn_30005 or self._feedback_conn_30005.is_connected) and
                (not self._feedback_conn_30006 or self._feedback_conn_30006.is_connected))

    def Disconnect(self):
        """Disconnect / 断开连接"""
        self.StopFeedbackMonitor()
        self._dashboard_conn.disconnect()
        self._feedback_conn.disconnect()

        # 断开可选的额外反馈端口 / Disconnect optional additional feedback ports
        if self._feedback_conn_30005:
            self._feedback_conn_30005.disconnect()
        if self._feedback_conn_30006:
            self._feedback_conn_30006.disconnect()

        logger.info(f"Robot disconnected: {self.ip}")  # 机器人已断开连接 / Robot disconnected

    def GetStatus(self) -> Optional[RobotStatus]:
        """
        Get the latest robot status
        获取最新机器人状态
        Returns:
            Optional[RobotStatus]: The latest cached status, or None until the first feedback frame is parsed successfully.
                                   最近一次缓存的机器人状态。首次成功解析反馈帧前返回 None。
        """
        return self._latest_status

    def StartFeedbackMonitor(self, callback: Callable = None):
        """
        Start the status feedback monitoring thread
        启动状态反馈监控线程
        Args:
            callback: Callback that receives a RobotStatus object when the status is updated.
                      状态更新时调用的回调函数，接收一个 RobotStatus 对象。
        """
        if self._feedback_running:
            return

        self._callback = callback
        self._feedback_running = True

        self._feedback_thread = threading.Thread(
            target=self._feedback_loop,
            daemon=True
        )
        self._feedback_thread.start()
        logger.info("Status monitor started")  # 状态监控已启动 / Status monitor started

    def StopFeedbackMonitor(self):
        """Stop status feedback monitoring / 停止状态反馈监控"""
        self._feedback_running = False
        if self._feedback_thread:
            self._feedback_thread.join(timeout=2.0)
            self._feedback_thread = None
        logger.info("Status monitor stopped")  # 状态监控已停止 / Status monitor stopped

    def _feedback_loop(self):
        """Feedback data receiving loop / 反馈数据接收循环"""
        while self._feedback_running:
            try:
                # 接收原始字节 / Receive raw bytes
                raw = self._feedback_conn.receive_bytes(144000)

                # 若一次读取超过一帧的长度，则丢弃本次数据并再次读取。
                # Discard the data and read again if a TCP read exceeds one feedback frame.
                if len(raw) > 1440:
                    raw = self._feedback_conn.receive_bytes(144000)

                if len(raw) < 1440:
                    time.sleep(0.01)
                    continue

                # 截取1440字节 / Take 1440 bytes
                packet = raw[:1440]

                # 解析 / Parse
                status = self._feedback_parser.parse(packet)
                if status:
                    self._latest_status = status

                    # 调用回调 / Invoke callback
                    if self._callback:
                        try:
                            self._callback(status)
                        except Exception as e:
                            logger.error(f"Callback error: {e}", exc_info=True)  # 回调错误 / Callback error

            except Exception as e:
                logger.error(f"Feedback error: {e}", exc_info=True)  # 反馈错误 / Feedback error
                time.sleep(0.1)

    def __enter__(self):
        """Context manager entry / 上下文管理器入口"""
        self.Connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit / 上下文管理器出口"""
        self.Disconnect()
        return False

    def GetError(self, language: str = "zh_cn") -> dict:
        """
        Get robot alarm information via the HTTP API.
        通过 HTTP 接口获取机器人报警信息。
        Args:
            language: Language setting, supported languages
                      语言设置，支持的语言
                     "zh_cn" - Simplified Chinese
                               简体中文
                     "zh_hant" - Traditional Chinese
                                 繁体中文
                     "en" - English
                            英语
                     "ja" - Japanese
                            日语
                     "de" - German
                            德语
                     "vi" - Vietnamese
                            越南语
                     "es" - Spanish
                            西班牙语
                     "fr" - French
                            法语
                     "ko" - Korean
                            韩语
                     "ru" - Russian
                            俄语

        Returns:
            dict: Alarm information dictionary, format as follows:
                  报警信息字典，格式如下：
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
            If there are no alarms, the method returns {"errMsg": []}.
            如果没有报警，则返回 {"errMsg": []}。
        """
        return self.error.GetError(language)

    def GetErrorFormatted(self, language: str = "zh_cn") -> str:
        """
        Get formatted robot alarm information via the HTTP API.
        通过 HTTP 接口获取格式化的机器人报警信息。
        Args:
            language: Language setting
                      语言设置

        Returns:
            str: Formatted alarm information string
                 格式化的报警信息字符串
        """
        return self.error.GetErrorFormatted(language)

    def __del__(self):
        """Destructor / 析构函数"""
        self.Disconnect()

    # 向后兼容别名 (snake_case -> PascalCase) / Backward compatibility aliases (snake_case -> PascalCase)
    set_timeout = SetTimeout
    enable_auto_reconnect = EnableAutoReconnect
    connect = Connect
    disconnect = Disconnect
    get_status = GetStatus
    start_feedback_monitor = StartFeedbackMonitor
    stop_feedback_monitor = StopFeedbackMonitor
    get_error = GetError
    get_error_formatted = GetErrorFormatted
    @property
    def is_connected(self):
        return self.IsConnected
