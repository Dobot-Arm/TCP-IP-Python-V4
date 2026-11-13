# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Communication-related module - Modbus and bus register control
通讯相关模块 - Modbus 和总线寄存器控制"""

from typing import List, Union
from ..core.connection import DobotConnection


class Communication:
    """Communication module - Handles Modbus and bus register communication
    通讯模块 - 处理 Modbus 和总线寄存器通信"""

    def __init__(self, connection: DobotConnection):
        self.connection = connection

    def _send_cmd(self, command: str) -> str:
        """Send a command and receive the response
        发送命令并接收响应"""
        return self.connection.send_receive_text(command)

    # ==================== Modbus 主站创建 / Modbus master creation ====================

    def ModbusCreate(self, ip: str, port: int, slave_id: int, is_rtu: int = 0) -> str:
        """
        Create a Modbus master
        创建 Modbus 主站

        Args:
            ip: Slave IP address / 从站 IP 地址
            port: Slave port / 从站端口
            slave_id: Slave ID / 从站 ID
            is_rtu: Protocol format: 0 for Modbus TCP, 1 for Modbus RTU. Defaults to 0.
                    是否为RTU 模式 (0-TCP, 1-RTU)，默认0

        Returns:
            str: ErrorID,{index},ModbusCreate(ip,port,slave_id,isRTU);
                 index is the returned master index, used for subsequent Modbus calls
                 index 为返回的主站索引，后续调用其他Modbus 指令时使用
        """
        cmd = f'ModbusCreate("{ip}",{port},{slave_id},{is_rtu})'
        return self._send_cmd(cmd)

    def ModbusRTUCreate(self, slave_id: int, baud: int,
                          parity: str = "E", data_bit: int = 8,
                          stop_bit: int = 1) -> str:
        """
        Create a Modbus master based on the RS485 interface
        创建基于 RS485 接口的 Modbus 主站

        Args:
            slave_id: Slave ID / 从站 ID
            baud: Baud rate of the RS485 interface / RS485 接口的波特率
            parity: Parity setting ("O"-Odd parity, "E"-Even parity, "N"-No parity, default "E")
                    奇偶校验位("O"-奇校验, "E"-偶校验, "N"-无校验，默认"E")
            data_bit: Number of data bits, default 8 / 数据位长度，默认 8
            stop_bit: Number of stop bits, default 1 / 停止位长度，默认 1

        Returns:
            str: ErrorID,{index},ModbusRTUCreate(slave_id,baud,parity,data_bit,stop_bit);
        """
        if parity not in ["O", "E", "N"]:
            raise ValueError("parity must be 'O', 'E' or 'N'")  # parity 必须为'O', 'E' 或 'N'

        cmd = f'ModbusRTUCreate({slave_id},{baud},"{parity}",{data_bit},{stop_bit})'
        return self._send_cmd(cmd)

    def ModbusClose(self, index: int) -> str:
        """
        Disconnect from the Modbus slave and release the master
        与 Modbus 从站断开连接，释放主站

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
        """
        return self._send_cmd(f"ModbusClose({index})")

    # ==================== 触点寄存器 / Contact register ====================

    def GetInBits(self, index: int, addr: int, count: int) -> str:
        """
        Read the values of the Modbus slave contact register (discrete input) address
        读取 Modbus 从站触点寄存器（离散输入）地址的值

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
            addr: Contact register start address / 触点寄存器起始地址
            count: Number of consecutive contact register values to read, range: [1, 16]
                   连续读取触点寄存器的值的数量，取值范围：[1, 16]

        Returns:
            str: ErrorID,{value1,value2,...,valuen},GetInBits(index,addr,count);
        """
        if not 1 <= count <= 16:
            raise ValueError("count must be in the range [1, 16]")  # count 取值范围必须是 [1, 16]

        cmd = f"GetInBits({index},{addr},{count})"
        return self._send_cmd(cmd)

    # ==================== 输入寄存器 / Input register ====================

    def GetInRegs(self, index: int, addr: int, count: int, val_type: str = "U16") -> str:
        """
        Read the value of the Modbus slave input register address according to the specified data type
        按照指定的数据类型，读取 Modbus 从站输入寄存器地址的值

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
            addr: Input register start address / 输入寄存器起始地址
            count: Number of consecutive input register values to read, range: [1, 4]
                   连续读取输入寄存器的值的数量，取值范围：[1, 4]
            val_type: Data format to read / 读取的数据格式
                      U16 (16-bit unsigned integer (2 bytes, occupies 1 register));
                      U16 （16位无符号整数（2个字节，占用1个寄存器））；
                      U32 (32-bit unsigned integer (4 bytes, occupies 2 registers));
                      U32（32位无符号整数（4个字节，占用2个寄存器））；
                      F32 (32-bit single-precision float (4 bytes, occupies 2 registers));
                      F32（32位单精度浮点数（4个字节，占用2个寄存器））；
                      F64 (64-bit double-precision float (8 bytes, occupies 4 registers));
                      F64（64位双精度浮点数（8个字节，占用4个寄存器））；
                      Default value is U16
                      默认值为 U16

        Returns:
            str: ErrorID,{value1,value2,...,valuen},GetInRegs(index,addr,count,valType);
        """
        if not 1 <= count <= 4:
            raise ValueError("count must be in the range [1, 4]")  # count 取值范围必须是 [1, 4]
        if val_type not in ["U16", "U32", "F32", "F64"]:
            raise ValueError("val_type must be 'U16', 'U32', 'F32' or 'F64'")  # val_type 必须为'U16', 'U32', 'F32' 或 'F64'

        cmd = f'GetInRegs({index},{addr},{count},"{val_type}")'
        return self._send_cmd(cmd)

    # ==================== 线圈寄存器 / Coil register ====================

    def GetCoils(self, index: int, addr: int, count: int) -> str:
        """
        Read the value of the Modbus slave coil register address
        读取 Modbus 从站线圈寄存器地址的值

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
            addr: Coil register start address / 线圈寄存器起始地址
            count: Number of consecutive coil register values to read, range: [1, 16]
                   连续读取线圈寄存器的值的数量，取值范围：[1, 16]

        Returns:
            str: ErrorID,{value1,value2,...,valuen},GetCoils(index,addr,count);
        """
        if not 1 <= count <= 16:
            raise ValueError("count must be in the range [1, 16]")  # count 取值范围必须是 [1, 16]

        cmd = f"GetCoils({index},{addr},{count})"
        return self._send_cmd(cmd)

    def SetCoils(self, index: int, addr: int, count: int, val_tab: Union[List[int], str]) -> str:
        """
        Write values to consecutive coils starting at the specified address.
        将指定的值写入线圈寄存器指定的地址

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
            addr: Coil register start address / 线圈寄存器起始地址
            count: Number of consecutive coil register values to write, range: [1, 16]
                   连续写入线圈寄存器的值的数量，取值范围：[1, 16]
            val_tab: Values to write. The number of values must equal count./ 要写入的值，数量与count 相同

        Returns:
            str: ErrorID,{},SetCoils(index,addr,count,valTab);
        """
        if not 1 <= count <= 16:
            raise ValueError("count must be in the range [1, 16]")  # count 取值范围必须是 [1, 16]

        if isinstance(val_tab, list):
            val_str = "{" + ",".join(str(v) for v in val_tab) + "}"
        else:
            val_str = val_tab

        cmd = f"SetCoils({index},{addr},{count},{val_str})"
        return self._send_cmd(cmd)

    def SetSingleCoil(self, index: int, addr: int, value: int) -> str:
        """
        Write to a single coil register (new command in V4.6.6)
        写入单个线圈寄存器（V4.6.6新增指令）

        Args:
            index: Master index returned when creating the master / 创建主站时返回的主站索引
            addr: Coil register address / 线圈寄存器地址
            value: Value to write, 0 or 1 / 要写入的值，0或1

        Returns:
            str: ErrorID,{},SetSingleCoil(index,addr,value);
        """
        if value not in [0, 1]:
            raise ValueError("value must be 0 or 1")  # value 必须是 0 或 1

        cmd = f"SetSingleCoil({index},{addr},{value})"
        return self._send_cmd(cmd)

    # ==================== 保持寄存器 / Holding register ====================

    def GetHoldRegs(self, index: int, addr: int, count: int, val_type: str = "U16") -> str:
        """
        Read the value of the Modbus slave holding register address according to the specified data type
        按照指定的数据类型，读取 Modbus 从站保持寄存器地址的值

        Args:
            index: Master index returned when creating the master, supports up to 5 devices, range: [0, 4]
                   创建主站时返回的主站索引，最多支持5个设备，取值范围：[0,4]
            addr: Holding register start address / 保持寄存器起始地址
            count: Number of consecutive holding register values to read
                   连续读取保持寄存器的值的数量
            val_type: Data type to read / 读取的数据类型
                      U16 (16-bit unsigned integer (2 bytes, occupies 1 register));
                      U16：16 位无符号整数（2 字节，占 1 个寄存器）；
                      U32 (32-bit unsigned integer (4 bytes, occupies 2 registers));
                      U32：32 位无符号整数（4 字节，占 2 个寄存器）；
                      F32 (32-bit single-precision float (4 bytes, occupies 2 registers));
                      F32：32 位单精度浮点数（4 字节，占 2 个寄存器）；
                      F64 (64-bit double-precision float (8 bytes, occupies 4 registers));
                      F64：64 位双精度浮点数（8 字节，占 4 个寄存器）；
                      Default value is U16
                      默认值为 U16

        Returns:
            str: ErrorID,{value1,value2,...,valuen},GetHoldRegs(index,addr,count,valType);
        """
        if not 0 <= index <= 4:
            raise ValueError("index must be in the range [0, 4]")  # index 取值范围必须是 [0, 4]
        if val_type not in ["U16", "U32", "F32", "F64"]:
            raise ValueError("val_type must be 'U16', 'U32', 'F32' or 'F64'")  # val_type 必须为'U16', 'U32', 'F32' 或 'F64'

        cmd = f'GetHoldRegs({index},{addr},{count},"{val_type}")'
        return self._send_cmd(cmd)

    def SetHoldRegs(self, index: int, addr: int, count: int,
                      val_tab: Union[List[int], str], val_type: str = "U16") -> str:
        """
        Write values of the selected data type to consecutive holding registers.
        将指定的值以指定的数据类型写入Modbus 从站保持寄存器指定的地址

        Args:
            index: Master index returned when creating the master, supports up to 5 devices, range: [0, 4]
                   创建主站时返回的主站索引，最多支持5个设备，取值范围：[0,4]
            addr: Holding register start address / 保持寄存器起始地址
            count: Number of consecutive holding register values to write, range: [1, 4]
                   连续写入保持寄存器的值的数量，取值范围：[1, 4]
            val_tab: Values to write. The number of values must equal count. / 要写入的值，数量与count 相同
            val_type: Data type to write / 写入的数据类型
                      U16 (16-bit unsigned integer (2 bytes, occupies 1 register));
                      U16：16 位无符号整数（2 字节，占 1 个寄存器）；
                      U32 (32-bit unsigned integer (4 bytes, occupies 2 registers));
                      U32：32 位无符号整数（4 字节，占 2 个寄存器）；
                      F32 (32-bit single-precision float (4 bytes, occupies 2 registers));
                      F32：32 位单精度浮点数（4 字节，占 2 个寄存器）；
                      F64 (64-bit double-precision float (8 bytes, occupies 4 registers));
                      F64：64 位双精度浮点数（8 字节，占 4 个寄存器）；
                      Default value is U16
                      默认值为 U16

        Returns:
            str: ErrorID,{},SetHoldRegs(index,addr,count,valTab,valType);
        """
        if not 0 <= index <= 4:
            raise ValueError("index must be in the range [0, 4]")  # index 取值范围必须是 [0, 4]
        if not 1 <= count <= 4:
            raise ValueError("count must be in the range [1, 4]")  # count 取值范围必须是 [1, 4]
        if val_type not in ["U16", "U32", "F32", "F64"]:
            raise ValueError("val_type must be 'U16', 'U32', 'F32' or 'F64'")  # val_type 必须为'U16', 'U32', 'F32' 或 'F64'

        if isinstance(val_tab, list):
            val_str = "{" + ",".join(str(v) for v in val_tab) + "}"
        else:
            val_str = val_tab

        cmd = f'SetHoldRegs({index},{addr},{count},{val_str},"{val_type}")'
        return self._send_cmd(cmd)

    def SetSingleHoldReg(self, index: int, addr: int, value: int) -> str:
        """
        Write to a single holding register (new command in V4.6.6)
        写入单个保持寄存器（V4.6.6新增指令）

        Args:
            index: Master index returned when creating the master, range: [0, 4]
                   创建主站时返回的主站索引，取值范围：[0, 4]
            addr: Holding register address / 保持寄存器地址
            value: Value to write / 要写入的值

        Returns:
            str: ErrorID,{},SetSingleHoldReg(index,addr,value);
        """
        if not 0 <= index <= 4:
            raise ValueError("index must be in the range [0, 4]")  # index 取值范围必须是 [0, 4]

        cmd = f"SetSingleHoldReg({index},{addr},{value})"
        return self._send_cmd(cmd)

    # ==================== 总线寄存器 - 输入寄存器 / Bus register - Input register ====================

    def GetInputBool(self, address: int) -> str:
        """
        Read the boolean value at the specified input register address.
        读取输入寄存器指定地址处的布尔值。

        Args:
            address: Register address, range: [0, 63] / 寄存器地址，取值范围：[0, 63]

        Returns:
            str: ErrorID,{value},GetInputBool(address);
                 value is the boolean value at the specified register address, returned as 0 or 1.
                 value 表示指定寄存器地址处的值，为 0 或 1。
        """
        if not 0 <= address <= 63:
            raise ValueError("address must be in the range [0, 63]")  # address 取值范围必须是 [0, 63]

        return self._send_cmd(f"GetInputBool({address})")

    def GetInputInt(self, address: int) -> str:
        """
        Read the int32 value at the specified input register address.
        读取输入寄存器指定地址处的 int32 值。

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]

        Returns:
            str: ErrorID,{value},GetInputInt(address);
                 value is the signed 32-bit integer at the specified register address.
                 value 表示指定寄存器地址处的值，为 32 位有符号整数。
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"GetInputInt({address})")

    def GetInputFloat(self, address: int) -> str:
        """
        Read the single-precision floating-point value at the specified input register address.
        读取输入寄存器指定地址处的单精度浮点数。

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]

        Returns:
            str: ErrorID,{value},GetInputFloat(address);
                 value is the single-precision floating-point number at the specified register address.
                 value 为指定寄存器地址处的单精度浮点数。
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"GetInputFloat({address})")

    # ==================== 总线寄存器 - 输出寄存器 / Bus register - Output register ====================

    def GetOutputBool(self, address: int) -> str:
        """
        Read the boolean value at the specified output register address.
        读取输出寄存器指定地址处的布尔值。

        Args:
            address: Register address, range: [0, 63] / 寄存器地址，取值范围：[0, 63]

        Returns:
            str: ErrorID,{value},GetOutputBool(address);
                 value is the boolean value at the specified register address, returned as 0 or 1.
                 value 表示指定寄存器地址处的值，为 0 或 1。
        """
        if not 0 <= address <= 63:
            raise ValueError("address must be in the range [0, 63]")  # address 取值范围必须是 [0, 63]

        return self._send_cmd(f"GetOutputBool({address})")

    def GetOutputInt(self, address: int) -> str:
        """
        Read the int32 value at the specified output register address.
        读取输出寄存器指定地址处的 int32 值。

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]

        Returns:
            str: ErrorID,{value},GetOutputInt(address);
                 value is the signed 32-bit integer at the specified register address.
                 value 表示指定寄存器地址处的值，为 32 位有符号整数。
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"GetOutputInt({address})")

    def GetOutputFloat(self, address: int) -> str:
        """
        Read the single-precision floating-point value at the specified output register address.
        读取输出寄存器指定地址处的单精度浮点数。

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]

        Returns:
            str: ErrorID,{value},GetOutputFloat(address);
                 value is the single-precision floating-point number at the specified register address.
                 value 为指定寄存器地址处的单精度浮点数。
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"GetOutputFloat({address})")

    def SetOutputBool(self, address: int, value: int) -> str:
        """
        Write the boolean value to the specified output register address.
        设置输出寄存器指定地址的 bool 值

        Args:
            address: Register address, range: [0, 63] / 寄存器地址，取值范围：[0, 63]
            value: Value to set, 0 or 1 / 要设置的值，0 或 1

        Returns:
            str: ErrorID,{},SetOutputBool(address,value);
        """
        if not 0 <= address <= 63:
            raise ValueError("address must be in the range [0, 63]")  # address 取值范围必须是 [0, 63]
        if value not in [0, 1]:
            raise ValueError("value must be 0 or 1")  # value 必须为0 或 1

        return self._send_cmd(f"SetOutputBool({address},{value})")

    def SetOutputInt(self, address: int, value: int) -> str:
        """
        Write the int32 value to the specified output register address.
        设置输出寄存器指定地址的 int 值

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]
            value: int32 value to set / 要设置的 int32 值

        Returns:
            str: ErrorID,{},SetOutputInt(address,value);
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"SetOutputInt({address},{value})")

    def SetOutputFloat(self, address: int, value: float) -> str:
        """
        Write the single-precision floating-point value to the specified output register address.
        设置输出寄存器指定地址的 float 值

        Args:
            address: Register address, range: [0, 23] / 寄存器地址，取值范围：[0, 23]
            value: float value to set / 要设置的 float 值

        Returns:
            str: ErrorID,{},SetOutputFloat(address,value);
        """
        if not 0 <= address <= 23:
            raise ValueError("address must be in the range [0, 23]")  # address 取值范围必须是 [0, 23]

        return self._send_cmd(f"SetOutputFloat({address},{value})")
