# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""I/O related module - Digital IO, Analog IO, End-effector IO control
IO相关模块 - 数字IO、模拟IO、末端IO控制"""

from typing import List, Union
from ..core.connection import DobotConnection


class IO:
    """I/O module - handles all IO related commands
    IO模块 - 处理所有IO相关指令"""

    def __init__(self, connection: DobotConnection):
        self.connection = connection

    def _send_cmd(self, command: str) -> str:
        """Send a command and receive the response
        发送命令并接收响应"""
        return self.connection.send_receive_text(command)

    # ==================== 数字输出端口 / Digital Output ports ====================

    def DO(self, index: int, status: int, time: float = None) -> str:
        """
        Set the status of a Digital Output port (Queue command)
        设置数字输出端口状态（队列指令）
        Args:
            index: DO port index (1-based)
                   DO端口索引 (1-based)
            status: status (0=Off, 1=On)
                    状态(0=Off, 1=On)
            time: output duration in seconds. Valid when status=1, automatically set to 0 when time is reached.
                  输出持续时间(秒)。当status=1时有效，到达时间后自动变为0。

        Returns:
            str: ErrorID,{ResultID},DO(index,status,time);
        """
        if status not in [0, 1]:
            raise ValueError("DO status must be 0 or 1")  # DO状态必须是0或1 / DO status must be 0 or 1
        if time is not None:
            return self._send_cmd(f"DO({index},{status},{time:.3f})")
        return self._send_cmd(f"DO({index},{status})")

    def DOInstant(self, index: int, status: int) -> str:
        """
        Set the status of a Digital Output port (Immediate command)
        设置数字输出端口状态（立即指令）
        Args:
            index: DO port index (1-based)
                   DO端口索引 (1-based)
            status: status (0=Off, 1=On)
                    状态(0=Off, 1=On)

        Returns:
            str: ErrorID,{},DOInstant(index,status);
        """
        if status not in [0, 1]:
            raise ValueError("DO status must be 0 or 1")  # DO状态必须是0或1 / DO status must be 0 or 1
        return self._send_cmd(f"DOInstant({index},{status})")

    def GetDO(self, index: int) -> str:
        """
        Get the status of a Digital Output port (Immediate command)
        获取数字输出端口状态（立即指令）
        Args:
            index: DO port index (1-based)
                   DO端口索引 (1-based)

        Returns:
            str: ErrorID,{status},GetDO(index);
        """
        return self._send_cmd(f"GetDO({index})")

    def DOGroup(self, *index_value_pairs) -> str:
        """
        Set the status of multiple Digital Output ports (Queue command)
        设置多个数字输出端口状态（队列指令）
        Args:
            *index_value_pairs: paired arguments of port index and status, e.g. DOGroup(4,1,6,0,2,1,7,0)
                                端口索引和状态的成对参数，如 DOGroup(4,1,6,0,2,1,7,0)

        Returns:
            str: ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...);
        """
        if len(index_value_pairs) % 2 != 0:
            raise ValueError("DOGroup requires an even number of arguments, supplied as index–value pairs.")  # DOGroup参数必须是偶数个（index与value成对） / DOGroup requires an even number of arguments, supplied as index–value pairs.
        params = []
        for i in range(0, len(index_value_pairs), 2):
            index = index_value_pairs[i]
            value = index_value_pairs[i + 1]
            if not isinstance(index, int):
                raise ValueError("DOGroup index must be of type int")  # DOGroup的index必须是int类型 / DOGroup index must be int
            if value not in [0, 1]:
                raise ValueError("DOGroup value must be 0 or 1")  # DOGroup的value必须是0或1 / DOGroup value must be 0 or 1
            params.append(str(index))
            params.append(str(value))
        return self._send_cmd(f"DOGroup({','.join(params)})")

    def DOGroupDEC(self, indices: Union[List[int], str], value: int) -> str:
        """
        Set the status of multiple Digital Output ports by assigning a decimal value (Queue command)
        通过赋值十进制设置多个数字输出端口状态（队列指令）
        Args:
            indices: list of port indices, e.g. [1,2,3,4,5] or string "{1,2,3,4,5}"
                     端口索引列表，如[1,2,3,4,5] 或字符串 "{1,2,3,4,5}"
            value: decimal value
                   十进制值
        Returns:
            str: ErrorID,{ResultID},DOGroupDEC({index1,index2,...,indexN},value);
        """
        if isinstance(indices, list):
            indices_str = "{" + ",".join(str(i) for i in indices) + "}"
        else:
            indices_str = indices
        return self._send_cmd(f"DOGroupDEC({indices_str},{value})")

    def GetDOGroup(self, *indices) -> str:
        """
        Get the status of multiple Digital Output ports (Immediate command)
        获取多个数字输出端口状态（立即指令）
        Args:
            *indices: DO port numbers to read, e.g. GetDOGroup(1,2)
                      要读取的DO端口编号，如 GetDOGroup(1,2)

        Returns:
            str: ErrorID,{status1,status2,...},GetDOGroup(index1,index2,...);
        """
        params = ",".join(str(i) for i in indices)
        return self._send_cmd(f"GetDOGroup({params})")

    def GetDOGroupDEC(self, indices: Union[List[int], str]) -> str:
        """
        Get the current status of multiple Digital Output ports, returned as a decimal number (Immediate command)
        获取多个数字输出端口当前状态，返回值为十进制数（立即指令）

        Args:
            indices: list of port indices, e.g. [1,2,3] or string "{1,2,3}"
                     端口索引列表，如[1,2,3] 或字符串 "{1,2,3}"

        Returns:
            str: ErrorID,{value},GetDOGroupDEC({index1,...,indexN});
        """
        if isinstance(indices, list):
            indices_str = "{" + ",".join(str(i) for i in indices) + "}"
        else:
            indices_str = indices
        return self._send_cmd(f"GetDOGroupDEC({indices_str})")

    # ==================== 数字输入端口 / Digital Input ports ====================

    def DI(self, index: int) -> str:
        """
        Get the status of a DI port (Immediate command)
        获取DI端口的状态（立即指令）
        Args:
            index: DI port index (1-based)
                   DI端口索引 (1-based)

        Returns:
            str: ErrorID,{status},DI(index);
        """
        return self._send_cmd(f"DI({index})")

    def DIGroup(self, *indices) -> str:
        """
        Get the status of multiple DI ports (Immediate command)
        获取多个DI端口的状态（立即指令）
        Args:
            *indices: DI port numbers to read, e.g. DIGroup(4,6,2,7)
                      要读取的DI端口编号，如 DIGroup(4,6,2,7)

        Returns:
            str: ErrorID,{status1,status2,...},DIGroup(index1,index2,...);
        """
        params = ",".join(str(i) for i in indices)
        return self._send_cmd(f"DIGroup({params})")

    def DIGroupDEC(self, indices: Union[List[int], str]) -> str:
        """
        Get the status of multiple DI ports, returned as a decimal number (Queue command)
        获取多个DI端口的状态，返回值为十进制数（队列指令）

        Args:
            indices: list of port indices, e.g. [1,2] or string "{1,2}"
                     端口索引列表，如[1,2] 或字符串 "{1,2}"

        Returns:
            str: ErrorID,{value},DIGroupDEC({index1,index2,...,indexN});
        """
        if isinstance(indices, list):
            indices_str = "{" + ",".join(str(i) for i in indices) + "}"
        else:
            indices_str = indices
        return self._send_cmd(f"DIGroupDEC({indices_str})")

    # ==================== 模拟输出端口 / Analog Output ports ====================

    def AO(self, index: int, value: float) -> str:
        """
        Set the value of an Analog Output port (Queue command)
        设置模拟输出端口的值（队列指令）
        Args:
            index: AO port index (1-based)
                   AO端口索引 (1-based)
            value: analog value (0.0-10.0V)
                   模拟值(0.0-10.0V)

        Returns:
            str: ErrorID,{ResultID},AO(index,value);
        """
        return self._send_cmd(f"AO({index},{value:.2f})")

    def AOInstant(self, index: int, value: float) -> str:
        """
        Set the value of an Analog Output port (Immediate command)
        设置模拟输出端口的值（立即指令）
        Args:
            index: AO port index (1-based)
                   AO端口索引 (1-based)
            value: analog value (0.0-10.0V)
                   模拟值(0.0-10.0V)

        Returns:
            str: ErrorID,{},AOInstant(index,value);
        """
        return self._send_cmd(f"AOInstant({index},{value:.2f})")

    def GetAO(self, index: int) -> str:
        """
        Get the value of an Analog Output port (Immediate command)
        获取模拟输出端口的值（立即指令）
        Args:
            index: AO port index (1-based)
                   AO端口索引 (1-based)

        Returns:
            str: ErrorID,{value},GetAO(index);
        """
        return self._send_cmd(f"GetAO({index})")

    # ==================== 模拟输入端口 / Analog Input ports ====================

    def AI(self, index: int) -> str:
        """
        Get the value of an AI port (Immediate command)
        获取AI端口的值（立即指令）
        Args:
            index: AI port index (1-based)
                   AI端口索引 (1-based)

        Returns:
            str: ErrorID,{value},AI(index);
        """
        return self._send_cmd(f"AI({index})")

    # ==================== 末端数字输出端口 / End-effector Digital Output ports ====================

    def ToolDO(self, index: int, status: int) -> str:
        """
        Set the status of an end-effector Digital Output port (Queue command)
        设置末端数字输出端口状态（队列指令）
        Args:
            index: End-effector DO port index, range: [1, MAX]
                   末端 DO 端口索引，取值范围：[1, MAX]
            status: status (0=Off, 1=On)
                    状态(0=Off, 1=On)

        Returns:
            str: ErrorID,{ResultID},ToolDO(index,status);
        """
        if status not in [0, 1]:
            raise ValueError("DO status must be 0 or 1")  # DO状态必须是0或1 / DO status must be 0 or 1
        return self._send_cmd(f"ToolDO({index},{status})")

    def ToolDOInstant(self, index: int, status: int) -> str:
        """
        Set the status of an end-effector Digital Output port (Immediate command)
        设置末端数字输出端口状态（立即指令）
        Args:
            index: End-effector DO port index, range: [1, MAX]
                   末端 DO 端口索引，取值范围：[1, MAX]
            status: status (0=Off, 1=On)
                    状态(0=Off, 1=On)

        Returns:
            str: ErrorID,{},ToolDOInstant(index,status);
        """
        if status not in [0, 1]:
            raise ValueError("DO status must be 0 or 1")  # DO状态必须是0或1 / DO status must be 0 or 1
        return self._send_cmd(f"ToolDOInstant({index},{status})")

    def GetToolDO(self, index: int) -> str:
        """
        Get the status of an end-effector Digital Output port (Immediate command)
        获取末端数字输出端口状态（立即指令）
        Args:
            index: End-effector DO port index, range: [1, MAX]
                   末端 DO 端口索引，取值范围：[1, MAX]

        Returns:
            str: ErrorID,{status},GetToolDO(index);
        """
        return self._send_cmd(f"GetToolDO({index})")

    # ==================== 末端数字输入端口 / End-effector Digital Input ports ====================

    def ToolDI(self, index: int) -> str:
        """
        Get the status of an end-effector DI port (Immediate command)
        获取末端DI端口的状态（立即指令）
        Args:
            index: end-effector DI port index
                   末端DI端口索引
        Returns:
            str: ErrorID,{status},ToolDI(index);
        """
        return self._send_cmd(f"ToolDI({index})")

    # ==================== 末端模拟输入端口 / End-effector Analog Input ports ====================

    def ToolAI(self, index: int) -> str:
        """
        Get the value of an end-effector AI port (Immediate command)
        获取末端AI端口的值（立即指令）
        Args:
            index: end-effector AI port index
                   末端AI端口索引
        Returns:
            str: ErrorID,{value},ToolAI(index);
        """
        return self._send_cmd(f"ToolAI({index})")

    # ==================== 末端工具设置 / End-effector tool settings ====================

    def SetTool485(self, baud: int, parity: str = "N", stopbit: int = 1, identify: int = None) -> str:
        """
        Set the end-effector RS485 communication format (Immediate command)
        设置末端485通信格式（立即指令）

        Args:
            baud: baud rate
                  波特率
            parity: parity bit ("O"=Odd, "E"=Even, "N"=None), default "N"
                    校验位("O"=奇校验, "E"=偶校验, "N"=无校验) 默认"N"
            stopbit: stop bits, default 1
                     停止位 默认1
            identify: identification parameter (optional)
                      辨识参数（可选）
        Returns:
            str: ErrorID,{},SetTool485(baud,parity,stopbit[,identify]);
        """
        if identify is not None:
            return self._send_cmd(f'SetTool485({baud},"{parity}",{stopbit},{identify})')
        return self._send_cmd(f'SetTool485({baud},"{parity}",{stopbit})')

    def SetToolPower(self, status: int) -> str:
        """
        Set the power status of the end-effector tool (Immediate command)
        设置末端工具供电状态（立即指令）
        Args:
            status: power status (0=Off, 1=On)
                    供电状态(0-关闭, 1-开启)

        Returns:
            str: ErrorID,{},SetToolPower(status);
        """
        if status not in [0, 1]:
            raise ValueError("status must be 0 or 1")  # 状态必须是0或1 / status must be 0 or 1
        return self._send_cmd(f"SetToolPower({status})")

    def SetToolMode(self, mode: int, type: int = None, identify: int = None) -> str:
        """
        Set the mode of the end-effector multiplexed terminal (Immediate command)
        设置末端复用端子的模式（立即指令）
        Args:
            mode: mode value
                  模式值
            type: type value (optional)
                  类型值（可选）
            identify: identification parameter (optional)
                      辨识参数（可选）
        Returns:
            str: ErrorID,{},SetToolMode(mode[,type[,identify]]);
        """
        params = [str(mode)]
        if type is not None:
            params.append(str(type))
            if identify is not None:
                params.append(str(identify))
        return self._send_cmd(f"SetToolMode({','.join(params)})")
