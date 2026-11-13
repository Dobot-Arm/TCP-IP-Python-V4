import socket
import numpy as np
import os
import re
import json
import threading
import time
from time import sleep
import requests

alarmControllerFile = "files/alarmController.json"
alarmServoFile = "files/alarmServo.json"

#brief dobot_v4_api:CRA\E6\CRAF\NovaLite
#author futingxing
#date 2026-11-9

# Port Feedback
#端口反馈数据
MyType = np.dtype([('len', np.uint16,),
                   ('reserve', np.byte, (6, )),
                   ('DigitalInputs', np.uint64,),
                   ('DigitalOutputs', np.uint64,),
                   ('RobotMode', np.uint64,),
                   ('TimeStamp', np.uint64,),
                   ('RunTime', np.uint64,),
                   ('TestValue', np.uint64,),
                   ('reserve2', np.byte, (8, )),
                   ('SpeedScaling', np.float64,),
                   ('reserve3', np.byte, (16, )),
                   ('VRobot', np.float64, ),      
                   ('IRobot', np.float64,),
                   ('ProgramState', np.float64,),
                   ('SafetyOIn', np.uint16,),
                   ('SafetyOOut', np.uint16,),
                   ('reserve4', np.byte, (76, )),
                   ('QTarget', np.float64, (6, )),
                   ('QDTarget', np.float64, (6, )),
                   ('QDDTarget', np.float64, (6, )),
                   ('ITarget', np.float64, (6, )),
                   ('MTarget', np.float64, (6, )),
                   ('QActual', np.float64, (6, )),
                   ('QDActual', np.float64, (6, )),
                   ('IActual', np.float64, (6, )),
                   ('ActualTCPForce', np.float64, (6, )),
                   ('ToolVectorActual', np.float64, (6, )),
                   ('TCPSpeedActual', np.float64, (6, )),
                   ('TCPForce', np.float64, (6, )),
                   ('ToolVectorTarget', np.float64, (6, )),
                   ('TCPSpeedTarget', np.float64, (6, )),
                   ('MotorTemperatures', np.float64, (6, )),
                   ('JointModes', np.float64, (6, )),
                   ('VActual', np.float64, (6, )),
                   ('HandType', np.byte, (4, )),
                   ('User', np.byte,),
                   ('Tool', np.byte,),
                   ('RunQueuedCmd', np.byte,),
                   ('PauseCmdFlag', np.byte,),
                   ('VelocityRatio', np.byte,),
                   ('AccelerationRatio', np.byte,),
                   ('reserve5', np.byte, ),
                   ('XYZVelocityRatio', np.byte,),
                   ('RVelocityRatio', np.byte,),
                   ('XYZAccelerationRatio', np.byte,),
                   ('RAccelerationRatio', np.byte,),
                   ('reserve6', np.byte,(2,)),
                   ('BrakeStatus', np.byte,),
                   ('EnableStatus', np.byte,),
                   ('DragStatus', np.byte,),
                   ('RunningStatus', np.byte,),
                   ('ErrorStatus', np.byte,),
                   ('JogStatusCR', np.byte,),   
                   ('CRRobotType', np.byte,),
                   ('DragButtonSignal', np.byte,),
                   ('EnableButtonSignal', np.byte,),
                   ('RecordButtonSignal', np.byte,),
                   ('ReappearButtonSignal', np.byte,),
                   ('JawButtonSignal', np.byte,),
                   ('SixForceOnline', np.byte,),
                   ('CollisionState', np.byte,),
                   ('ArmApproachState', np.byte,),
                   ('J4ApproachState', np.byte,),
                   ('J5ApproachState', np.byte,),
                   ('J6ApproachState', np.byte,),
                   ('reserve7', np.byte, (61, )),
                   ('VibrationDisZ', np.float64,),
                   ('CurrentCommandId', np.uint64,),
                   ('MActual', np.float64, (6, )),
                   ('Load', np.float64,),
                   ('CenterX', np.float64,),
                   ('CenterY', np.float64,),
                   ('CenterZ', np.float64,),
                   ('UserValue[6]', np.float64, (6, )),
                   ('ToolValue[6]', np.float64, (6, )),
                   ('reserve8', np.byte, (8, )),
                   ('SixForceValue', np.float64, (6, )),
                   ('TargetQuaternion', np.float64, (4, )),
                   ('ActualQuaternion', np.float64, (4, )),
                   ('AutoManualMode', np.uint16, ),
                   ('ExportStatus', np.uint16, ),
                   ('SafetyState', np.byte, ),
                   ('reserve9', np.byte,(19,))
                   ])

# 读取控制器和伺服告警文件
# Read controller and servo alarm files
def alarmAlarmJsonFile():
    currentDirectory = os.path.dirname(__file__)
    jsonControllorPath = os.path.join(currentDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currentDirectory, alarmServoFile)

    with open(jsonControllorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo

# Tcp通信接口类
# TCP communication interface


class DobotApi:
    def __init__(self, ip, port, *args):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.__globalLock = threading.Lock()
        if args:
            self.text_log = args[0]

        if self.port == 29999 or self.port == 30004 or self.port == 30005:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
                self.socket_dobot.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144000)
            except socket.error:
                print(socket.error)

        else:
            print(f"Connect to dashboard server need use port {self.port} !")

    def log(self, text):
        if self.text_log:
            print(text)

    def send_data(self, string):
       # self.log(f"Send to {self.ip}:{self.port}: {string}")
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)
            while True:
                try:
                    self.socket_dobot = self.reConnect(self.ip, self.port)
                    self.socket_dobot.send(str.encode(string, 'utf-8'))
                    break
                except Exception:
                    sleep(1)

    def wait_reply(self):
        """
        读取返回值
        Read the return value
        """
        data = ""
        try:
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)
            self.socket_dobot = self.reConnect(self.ip, self.port)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
            # self.log(f'Receive from {self.ip}:{self.port}: {data_str}')
            return data_str

    def close(self):
        """
        关闭端口
        Close the port
        """
        if (self.socket_dobot != 0):
            try:
                self.socket_dobot.shutdown(socket.SHUT_RDWR)
                self.socket_dobot.close()
            except socket.error as e:
                print(f"Error while closing socket: {e}")

    def sendRecvMsg(self, string):
        """
        在持有通信锁的情况下发送指令，并等待控制器响应。
        Send a command and wait for the controller response while holding the communication lock.
        """
        with self.__globalLock:
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    def __del__(self):
        self.close()

    def reConnect(self, ip, port):
        while True:
            try:
                socket_dobot = socket.socket()
                socket_dobot.connect((ip, port))
                break
            except Exception:
                sleep(1)
        return socket_dobot

# 控制及运动指令接口类
# Control and motion command interface


class DobotApiDashboard(DobotApi):

    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)

    def _fmt(self, v):
        if isinstance(v, (list, tuple)):
            return "{" + ",".join([self._fmt(x) for x in v]) + "}"
        if isinstance(v, float):
            return ("{:f}".format(v))
        if isinstance(v, int):
            return ("{:d}".format(v))
        return str(v)

    def _build_cmd(self, name, *args, **kwargs):
        parts = []
        for a in args:
            parts.append(self._fmt(a))
        for k, v in kwargs.items():
            parts.append(f"{k}={self._fmt(v)}")
        return f"{name}(" + ",".join(parts) + ")"

    def EnableRobot(self, load=0.0, centerX=0.0, centerY=0.0, centerZ=0.0, isCheck=-1,):
        """
            可选参数
            参数名 类型 说明
            load double 设置负载重量，取值范围不能超过各个型号机器⼈的负载范围。单位：kg
            centerX double X⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            centerY double Y⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            centerZ double Z⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            isCheck int    是否检查负载。1表⽰检查，0表⽰不检查。如果设置为1，则机械臂
            使能后会检查实际负载是否和设置负载⼀致，如果不⼀致会⾃动下使
            能。默认值为0
            可携带的参数数量如下：
            0：不携带参数，表⽰使能时不设置负载重量和偏⼼参数。
            1：携带⼀个参数，该参数表⽰负载重量。
            4：携带四个参数，分别表⽰负载重量和偏⼼参数。
            5：携带五个参数，分别表⽰负载重量、偏⼼参数和是否检查负载。
            Optional parameters
            Parameter name     Type     Description
            load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
            centerX     double     X-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerY     double     Y-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerZ     double     Z-direction eccentric distance. Range: -999 – 999, unit: mm.
            isCheck     int     Check the load or not. 1: check, 0: not check. If set to 1, the robot arm will check whether the actual load is the same as the set load after it is enabled, and if not, it will be automatically disabled. 0 by default.
            The number of parameters that can be contained is as follows:
            0: no parameter (not set load weight and eccentric parameters when enabling the robot).
            1: one parameter (load weight).
            4: four parameters (load weight and eccentric parameters).
            5: five parameters (load weight, eccentric parameters, check the load or not).
                """
        string = 'EnableRobot('
        if load != 0:
            string = string + "{:f}".format(load)
            if centerX != 0 or centerY != 0 or centerZ != 0:
                string = string + ",{:f},{:f},{:f}".format(
                    centerX, centerY, centerZ)
                if isCheck != -1:
                    string = string + ",{:d}".format(isCheck)
        string = string + ')'
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        下使能机器人。
        Disable the robot.
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        清除机器⼈报警。清除报警后，⽤⼾可以根据RobotMode来判断机器⼈是否还处于报警状态。
        部分报警需要解决报警原因或者重启控制柜后才能清除。
        Clear controller alarm information
        Clear the alarms of the robot. After clearing the alarm, you can judge whether the robot is still in the alarm status according to RobotMode.
        Some alarms cannot be cleared unless you resolve the alarm cause or restart the controller.
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def PowerOn(self):
        """
        给机器人上电。
        上电后等待约 10 秒，待初始化完成后再使能。

        Power on the robot.
        Wait about 10 seconds for initialization to complete before enabling the robot.
        """
        string = "PowerOn()"
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
        运⾏脚本文件。
        project_name Script 脚本文件
        Run the script file
        project_name ：Script file name
        """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def Stop(self):
        """
       停⽌已下发的运动指令队列或者RunScript指令运⾏的⼯程。
       Stop queued motion commands or the project started by RunScript.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)

    def Pause(self):
        """
       暂停已下发的运动指令队列或者RunScript指令运⾏的⼯程。
       Pause queued motion commands or the project started by RunScript.
        """
        string = "Pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
       继续已暂停的运动指令队列或者RunScript指令运⾏的⼯程。
       Resume paused motion commands or the paused project.
        """
        string = "Continue()"
        return self.sendRecvMsg(string)

    def EmergencyStop(self, mode):
        """
       紧急停⽌机械臂。急停后机械臂会下使能并报警，需要松开急停、清除报警后才能重新使能。
       必选参数
       参数名 类型 说明
        mode int 急停操作模式。1表⽰按下急停，0表⽰松开急停
       Stop the robot in an emergency. An emergency stop disables the robot and triggers an alarm.
       You need to release the emergency stop and clear the alarm to re-enable the robot arm.
       Required parameter
       Parameter name     Type     Description
        mode     int     E-Stop operation mode. 1: press the E-Stop, 0: release the E-Stop.
        """
        string = "EmergencyStop({:d})".format(mode)
        return self.sendRecvMsg(string)

    def BrakeControl(self, axisID, value):
        """
        描述
        控制指定关节的抱闸。机械臂静⽌时关节会⾃动抱闸，如果⽤⼾需进⾏关节拖拽操作，可松开抱闸，即在机械臂下使能状态，⼿动扶住关节后，下发松开抱闸的指令。
        仅能在机器⼈下使能时控制关节抱闸，否则ErrorID会返回-1。
        必选参数
        参数名  类型  说明
        axisID int 关节轴序号，1表⽰J1轴，2表⽰J2轴，以此类推
        value int 设置抱闸状态。0表⽰抱闸锁死（关节不可移动），1表⽰松开抱闸（关节可移动）
        Description
        Control the brake of the specified joint. The joints automatically brake when the robot is stationary. If you need to drag the joints, you can release the joint brake,
        i.e. hold the joint manually in the disabled status and deliver the command to release the joint brake.
        Joint brake can be controlled only when the robot arm is disabled, otherwise, Error ID will return -1.
        Required parameter:
        Parameter name     Type     Description
        axisID     int     joint ID, 1: J1, 2: J2, and so on
        value     int     Set the status of brake. 0: engage the joint brake (joints cannot be dragged). 1: release the joint brake (joints can be dragged).
        """
        string = "BrakeControl({:d},{:d})".format(axisID, value)
        return self.sendRecvMsg(string)

    #####################################################################

    def SpeedFactor(self, speed):
        """
        设置全局速度⽐例。
           机械臂点动时实际运动加速度/速度⽐例 = 控制软件点动设置中的值 x 全局速度⽐例。
           例：控制软件设置的关节速度为12°/s，全局速率为50%，则实际点动速度为12°/s x 50% =
           6°/s
           机械臂再现时实际运动加速度/速度⽐例 = 运动指令可选参数设置的⽐例 x 控制软件再现设置
           中的值 x 全局速度⽐例。
           例：控制软件设置的坐标系速度为2000mm/s，全局速率为50%，运动指令设置的速率为
           80%，则实际运动速度为2000mm/s x 50% x 80% = 800mm/s
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        取值范围：[1, 100]
        Set the global speed ratio.
           Actual robot acceleration/speed ratio in jogging = value in Jog settings × global speed ratio.
           Example: If the joint speed set in the software is 12°/s and the global speed ratio is 50%, then the actual jog speed is 12°/s x 50% =
           6°/s
           Actual robot acceleration/speed ratio in playback = ratio set in motion command × value in Playback settings
            × global speed ratio.
           Example: If the coordinate system speed set in the software is 2000mm/s, the global speed ratio is 50%, and the speed set in the motion command is
           80%, then the actual speed is 2000mm/s x 50% x 80% = 800mm/s.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Range: [1, 100].
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        设置全局⽤⼾坐标系。⽤⼾下发运动指令时可选择⽤⼾坐标系，如未指定，则会使⽤全局⽤⼾坐标系。
        未设置时默认的全局⽤⼾坐标系为⽤⼾坐标系0。
        Set the global user coordinate system. You can select a user coordinate system while delivering motion commands. If you do not specify the user coordinate system, the global user coordinate system will be used.
        If it is not set, the default global user coordinate system is User coordinate system 0.
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetUser(self, index, table):
        """
        修改指定的用户坐标系。

        参数：
            index (int)：已标定的用户坐标系索引，范围为 [1, 50]。
            table (str)：修改后的坐标系，格式为 {x, y, z, rx, ry, rz}，可通过 CalcUser 计算。

        返回：
            控制器响应字符串，格式为 ErrorID,{},SetUser(index,table);

        Update the specified user coordinate system.

        Args:
            index (int): Index of a calibrated user coordinate system. Range: [1, 50].
            table (str): Updated coordinate system in the format {x, y, z, rx, ry, rz}.
                Use CalcUser to calculate it.

        Returns:
            The controller response string in the format ErrorID,{},SetUser(index,table);
        """
        string = "SetUser({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcUser(self, index, matrix_direction, table):
        """
        计算指定用户坐标系应用偏移后的结果。

        参数：
            index (int)：已标定的用户坐标系索引，范围为 [0, 50]。索引 0 初始为基坐标系。
            matrix_direction (int)：乘法方向。1 表示左乘，偏移相对于基坐标系。
                0 表示右乘，偏移相对于所选用户坐标系。
            table (str)：偏移量，格式为 {x, y, z, rx, ry, rz}。

        返回：
            控制器响应字符串，格式为 ErrorID,{x,y,z,rx,ry,rz},CalcUser(index,matrix_direction,table);
            花括号内为计算得到的用户坐标系。

        Calculate the specified user coordinate system after applying an offset.

        Args:
            index (int): Index of a calibrated user coordinate system. Range: [0, 50].
                Index 0 initially represents the base coordinate system.
            matrix_direction (int): Multiplication order. Use 1 to left-multiply and apply
                the offset in the base coordinate system. Use 0 to right-multiply and
                apply the offset in the selected user coordinate system.
            table (str): Offset in the format {x, y, z, rx, ry, rz}.

        Returns:
            The controller response string in the format
            ErrorID,{x,y,z,rx,ry,rz},CalcUser(index,matrix_direction,table);
            The values in braces describe the calculated user coordinate system.
        """
        string = "CalcUser({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        设置全局⼯具坐标系。⽤⼾下发运动指令时可选择⼯具坐标系，如未指定，则会使⽤全局⼯具坐标系。
        未设置时默认的全局⼯具坐标系为⼯具坐标系0。
        Set the global tool coordinate system. You can select a tool coordinate system while delivering motion commands. If you do not specify the tool coordinate system, the global tool coordinate system will be used.
        If it is not set, the default global tool coordinate system is Tool coordinate system 0.
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool(self, index, table):
        """
        修改指定的工具坐标系。

        参数：
            index (int)：已标定的工具坐标系索引，范围为 [1, 50]。
            table (str)：相对于默认工具坐标系的偏移量，格式为 {x, y, z, rx, ry, rz}。

        返回：
            控制器响应字符串，格式为 ErrorID,{},SetTool(index,table);

        Update the specified tool coordinate system.

        Args:
            index (int): Index of a calibrated tool coordinate system. Range: [1, 50].
            table (str): Offset relative to the default tool coordinate system,
                in the format {x, y, z, rx, ry, rz}.

        Returns:
            The controller response string in the format ErrorID,{},SetTool(index,table);
        """
        string = "SetTool({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcTool(self, index, matrix_direction, table):
        """
        计算指定工具坐标系应用偏移后的结果。

        参数：
            index (int)：已标定的工具坐标系索引，范围为 [0, 50]。索引 0 初始为法兰坐标系。
            matrix_direction (int)：乘法方向。1 表示左乘，偏移相对于法兰坐标系。
                0 表示右乘，偏移相对于所选工具坐标系。
            table (str)：偏移量，格式为 {x, y, z, rx, ry, rz}。

        返回：
            控制器响应字符串，格式为 ErrorID,{x,y,z,rx,ry,rz},CalcTool(index,matrix_direction,table);
            花括号内为计算得到的工具坐标系。

        Calculate the specified tool coordinate system after applying an offset.

        Args:
            index (int): Index of a calibrated tool coordinate system. Range: [0, 50].
                Index 0 initially represents the flange coordinate system.
            matrix_direction (int): Multiplication order. Use 1 to left-multiply and apply
                the offset in the flange coordinate system. Use 0 to right-multiply and
                apply the offset in the selected tool coordinate system.
            table (str): Offset in the format {x, y, z, rx, ry, rz}.

        Returns:
            The controller response string in the format
            ErrorID,{x,y,z,rx,ry,rz},CalcTool(index,matrix_direction,table);
            The values in braces describe the calculated tool coordinate system.
        """
        string = "CalcTool({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def SetPayload(self, load=0.0, X=0.0, Y=0.0, Z=0.0, name='F'):
        '''
        设置机械臂末端负载，⽀持两种设置⽅式。
        ⽅式⼀：直接设置负载参数
        必选参数1
        参数名 类型 说明
        load double  设置负载重量，取值范围不能超过各个型号机器⼈的负载范围。单位：kg
        可选参数1
        参数名 类型 说明
        X double 末端负载质心的 X 轴坐标。取值范围：[-500, 500]，单位：mm。
        Y double 末端负载质心的 Y 轴坐标。取值范围：[-500, 500]，单位：mm。
        Z double 末端负载质心的 Z 轴坐标。取值范围：[-500, 500]，单位：mm。
        需同时设置或不设置这三个参数。偏⼼坐标为负载（含治具）的质⼼在默认⼯具坐标系下的坐标，
        参考下图。

        ⽅式⼆：通过控制软件保存的预设负载参数组设置
        必选参数2
        参数名 类型 说明
        name string 控制软件保存的预设负载参数组的名称
        Set the load of the robot arm.
        Method 1: Set the load parameters directly.
        Required parameter 1
        Parameter name     Type     Description
        load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
        Optional parameter 1
        Parameter name     Type     Description
        X     double     X coordinate of the payload center of mass. Range: [-500, 500]. Unit: mm.
        Y     double     Y coordinate of the payload center of mass. Range: [-500, 500]. Unit: mm.
        Z     double     Z coordinate of the payload center of mass. Range: [-500, 500]. Unit: mm.
        The three parameters need to be set or not set at the same time. The eccentric coordinate is the coordinate of the center of mass of the load (including the fixture) under the default tool coordinate system.
        Refer to the figure below.

        Method 2: Set by the preset load parameter group saved by control software
        Required parameter 2
        Parameter name     Type     Description
        name     string     Name of the preset load parameter group saved by control software.
        '''
        string = 'SetPayload('
        if name != 'F':
            string = string + "{:s}".format(name)
        else:
            if load != 0:
                string = string + "{:f}".format(load)
                if X != 0 or Y != 0 or Z != 0:
                    string = string + ",{:f},{:f},{:f}".format(X, Y, Z)
        string = string + ')'
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        设置关节运动⽅式的加速度⽐例。
        未设置时默认值为100
        Set acceleration ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        设置直线和弧线运动⽅式的加速度⽐例。
        未设置时默认值为100。
        Set acceleration ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelJ(self, speed):
        """
        设置关节运动⽅式的速度⽐例。
        未设置时默认值为100。
        Set the speed ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "VelJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelL(self, speed):
        """
        设置直线和弧线运动⽅式的速度⽐例。
        未设置时默认值为100。
        Set the speed ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "VelL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        设置平滑过渡⽐例，即机械臂连续运动经过多个点时，经过中间点是以直⻆⽅式过渡还是以曲线⽅式过渡。
        未设置时默认值为0。
        平滑过渡⽐例。取值范围：[0, 100]
        Set the continuous path (CP) ratio, that is, when the robot arm moves continuously via multiple points, whether it transitions at a right angle or in a curved way when passing through the point.
        Defaults to 0 if not set.
        Continuous path ratio. Range: [0, 100].
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, level):
        """
        设置碰撞检测等级。
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        必选参数
        参数名 类型 说明
        level int 碰撞检测等级，0表⽰关闭碰撞检测，1~5数字越⼤灵敏度越⾼
        Set the collision detection level.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        level     int     collision detection level, 0: switch off collision detection, 1 – 5: the larger the number, the higher the sensitivity.
        """
        string = "SetCollisionLevel({:d})".format(level)
        return self.sendRecvMsg(string)

    def SetBackDistance(self, distance):
        """
        设置机械臂检测到碰撞后原路回退的距离。
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        必选参数：
        参数名 类型 说明
        distance double 碰撞回退的距离，取值范围：[0,50]，单位：mm
        Set the backoff distance after the robot detects collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        distance     double     collision backoff distance, range: [0,50], unit: mm.
        """
        string = "SetBackDistance({:d})".format(distance)
        return self.sendRecvMsg(string)

    def SetPostCollisionMode(self, mode):
        """
        设置机械臂检测到碰撞后进⼊的状态。
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        必选参数：
        参数名  类型 说明
        mode int  碰撞后处理⽅式，0表⽰检测到碰撞后进⼊停⽌状态，1表⽰检测到碰撞后
        进⼊暂停状态
        Set the robot's response to a detected collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        mode     int     post-collision processing mode, 0: enter the stop status after the collision is detected, 1: enter the pause status after the collision is detected
        """
        string = "SetPostCollisionMode({:d})".format(mode)
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """
        机械臂进⼊拖拽模式。机械臂处于报警状态下时，⽆法通过该指令进⼊拖拽模式。
        The robot arm enters the drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        """
        退出拖拽
        Exit joint drag mode or force-controlled drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def DragSensivity(self, index, value):
        """
        设置拖拽灵敏度。
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        必选参数
        参数名 类型 说明
        index int 轴序号，1~6分别表⽰J1~J6轴，0表⽰所有轴同时设置
        value int 拖拽灵敏度，值越⼩，拖拽时的阻⼒越⼤。取值范围：[1, 90]
        Set the drag sensitivity.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        index     int      axis ID, 1 – 6: J1 – J6, 0: set all axes at the same time.
        value     int     Drag sensitivity. The smaller the value, the greater the force when dragging. Range: [1, 90].
        """
        string = "DragSensivity({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def EnableSafeSkin(self, status):
        """
        开启或关闭安全⽪肤功能。仅对安装了安全⽪肤的机械臂有效。
        必选参数
        参数名 类型 说明
        status int 电⼦⽪肤功能开关，0表⽰关闭，1表⽰开启
        Switch on or off the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        Required parameter:
        Parameter name     Type     Description
        status     int     SafeSkin switch, 0: off, 1: on.
        """
        string = "EnableSafeSkin({:d})".format(status)
        return self.sendRecvMsg(string)

    def SetSafeSkin(self, part, status):
        """
        设置安全⽪肤各个部位的灵敏度。仅对安装了安全⽪肤的机械臂有效。
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        必选参数
        参数名 类型 说明
        part int 要设置的部位，3表⽰⼩臂，4~6分别表⽰J4~J6关节
        status int 灵敏度，0表⽰关闭，1表⽰low，2表⽰middle，3表⽰high
        Set the sensitivity for each part of the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        part     int     The part to be set. 3: forearm, 4 – 6: J4 – J6
        status     int     sensitivity, 0: off, 1: low, 2: middle, 3: high
        """
        string = "SetSafeSkin({:d},{:d})".format(part, status)
        return self.sendRecvMsg(string)

    def SetSafeWallEnable(self, index, value):
        """
        开启或关闭指定的安全墙。
        必选参数：
        参数名 类型 说明
        index int 要设置的安全墙索引，需要先在控制软件中添加对应的安全墙。取值范围：[1,8]
        value int 安全墙开关，0表⽰关闭，1表⽰开启
        Switch on/off the specified safety wall.
        Required parameter:
        Parameter name     Type     Description
        index     int     safety wall index, which needs to be added in the software first. Range: [1,8].
        value      int     safety wall enable flag, 0: off, 1: on.
        """
        string = "SetSafeWallEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def SetWorkZoneEnable(self, index, value):
        """
        开启或关闭指定的⼲涉区。
        必选参数：
        参数名 类型 说明
        index int 要设置的⼲涉区索引，需要先在控制软件中添加对应的⼲涉区。取值范围：[1,6]
        value int ⼲涉区开关，0表⽰关闭，1表⽰开启
        Switch on/off the specified interference area.
        Required parameter:
        Parameter name     Type     Description
        index     int     interference area index, which needs to be added in the software first. Range: [1,6].
        value      int     interference area switch, 0: off, 1: on.
        """
        string = "SetWorkZoneEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    #########################################################################

    def RobotMode(self):
        """
        获取机器⼈当前状态。
        1 ROBOT_MODE_INIT 初始化状态
        2 ROBOT_MODE_BRAKE_OPEN 有任意关节的抱闸松开
        3 ROBOT_MODE_POWEROFF 机械臂下电状态
        4 ROBOT_MODE_DISABLED 未使能（⽆抱闸松开）
        5 ROBOT_MODE_ENABLE 使能且空闲
        6 ROBOT_MODE_BACKDRIVE 拖拽模式
        7 ROBOT_MODE_RUNNING 运⾏状态(⼯程，TCP队列运动等)
        8 ROBOT_MODE_SINGLE_MOVE 单次运动状态（点动、RunTo等）
        9 ROBOT_MODE_ERROR
             有未清除的报警。此状态优先级最⾼，⽆论机械臂
             处于什么状态，有报警时都返回9
        10 ROBOT_MODE_PAUSE ⼯程状态
        11 ROBOT_MODE_COLLISION 碰撞检测触发状态
        Get the current status of the robot.
        1 ROBOT_MODE_INIT  Initialized status
        2 ROBOT_MODE_BRAKE_OPEN  Brake switched on
        3 ROBOT_MODE_POWEROFF  Power-off status
        4 ROBOT_MODE_DISABLED  Disabled (no brake switched on
        5 ROBOT_MODE_ENABLE  Enabled and idle
        6 ROBOT_MODE_BACKDRIVE  Drag mode
        7 ROBOT_MODE_RUNNING  Running status (project, TCP queue)
        8 ROBOT_MODE_SINGLE_MOVE  Single motion status (jog, RunTo)
        9 ROBOT_MODE_ERROR
             There are uncleared alarms. This status has the highest priority. It returns 9 when there is an alarm, regardless of the status of the robot arm.
        10 ROBOT_MODE_PAUSE  Pause status
        11 ROBOT_MODE_COLLISION  Collision status
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PositiveKin(self, J1, J2, J3, J4, J5, J6, user=-1, tool=-1):
        """
       描述
       进⾏正解运算：给定机械臂各关节⻆度，计算机械臂末端在给定的笛卡尔坐标系中的坐标值。
       必选参数
       参数名 类型 说明
       J1 double J1轴位置，单位：度
       J2 double J2轴位置，单位：度
       J3 double J3轴位置，单位：度
       J4 double J4轴位置，单位：度
       J5 double J5轴位置，单位：度
       J6 double J6轴位置，单位：度
       可选参数
       参数名 类型 说明
       格式为"user=index"，index为已标定的⽤⼾坐标系索引。
       User string 不指定时使⽤全局⽤⼾坐标系。
       Tool string  格式为"tool=index"，index为已标定的⼯具坐标系索引。不指定时使⽤全局⼯具坐标系。
       Description
       forward kinematics. Calculate the coordinates of the end of the robot in the specified Cartesian coordinate system, based on the given angle of each joint.
       Required parameter:
       Parameter name     Type     Description
       J1     double     J1-axis position, unit: °
       J2     double     J2-axis position, unit: °
       J3     double     J3-axis position, unit: °
       J4     double     J4-axis position, unit: °
       J5     double     J5-axis position, unit: °
       J6     double     J6-axis position, unit: °
       Optional parameter:
       Parameter name     Type     Description
       Format: "user=index", index: index of the calibrated user coordinate system.
       User     string     The global user coordinate system will be used if it is not specified.
       Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        """
        string = "PositiveKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            J1, J2, J3, J4, J5, J6)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def InverseKin(self, X, Y, Z, Rx, Ry, Rz, user=-1, tool=-1, useJointNear=-1, JointNear=''):
        """
        描述
        进⾏逆解运算：给定机械臂末端在给定的笛卡尔坐标系中的坐标值，计算机械臂各关节⻆度。
        由于笛卡尔坐标仅定义了TCP的空间坐标与倾斜⻆，所以机械臂可以通过多种不同的姿态到达同⼀
        个位姿，意味着⼀个位姿变量可以对应多个关节变量。为得出唯⼀的解，系统需要⼀个指定的关节
        坐标，选择最接近该关节坐标的解作为逆解结果。
        必选参数
        参数名 类型 说明
        X double X轴位置，单位：mm
        Y double Y轴位置，单位：mm
        Z double Z轴位置，单位：mm
        Rx double Rx轴位置，单位：度
        Ry double Ry轴位置，单位：度
        Rz double Rz轴位置，单位：度
        可选参数
        参数名 类型 说明
        User string  格式为"user=index"，index为已标定的⽤⼾坐标系索引。不指定时使⽤全局⽤⼾坐标系。
        Tool string  格式为"tool=index"，index为已标定的⼯具坐标系索引。不指定时使⽤全局⼯具坐标系。
        useJointNear string  ⽤于设置JointNear参数是否有效。
            "useJointNear=0"或不携带表⽰JointNear⽆效，系统根据机械臂当前关节⻆度就近选解。
            "useJointNear=1"表⽰根据JointNear就近选解。
        jointNear string 格式为"jointNear={j1,j2,j3,j4,j5,j6}"，⽤于就近选解的关节坐标。
        Description
        Inverse kinematics. Calculate the joint angles of the robot, based on the given coordinates in the specified Cartesian coordinate system.
        As Cartesian coordinates only define the spatial coordinates and tilt angle of the TCP, the robot arm can reach the same posture through different joint configurations, which means that one posture variable can correspond to multiple joint variables.
        To get a unique solution, the system requires a specified joint coordinate, and the solution closest to this joint coordinate is selected as the inverse solution。
        Required parameter:
        Parameter name     Type     Description
        X     double     X-axis position, unit: mm
        Y     double     Y-axis position, unit: mm
        Z     double     Z-axis position, unit: mm
        Rx     double     Rx-axis position, unit: °
        Ry     double     Ry-axis position, unit: °
        Rz     double     Rz-axis position, unit: °
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system. The global user coordinate system will be used if it is not set.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        useJointNear     string     used to set whether JointNear is effective.
            "useJointNear=0" or null: JointNear data is ineffective. The algorithm selects the joint angles according to the current angle.
            "useJointNear=1": the algorithm selects the joint angles according to JointNear data.
        jointNear     string     Format: "jointNear={j1,j2,j3,j4,j5,j6}", joint coordinates for selecting joint angles.
        """
        string = "InverseKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            X, Y, Z, Rx, Ry, Rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if useJointNear != -1:
            params.append('useJointNear={:d}'.format(useJointNear))
        if JointNear != '':
            params.append('JointNear={:s}'.format(JointNear))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetAngle(self):
        """
        获取机械臂当前位姿的关节坐标。
        Get the joint coordinates of current posture.
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self, user=-1, tool=-1):
        """
        获取机械臂当前位姿在指定的坐标系下的笛卡尔坐标。
        可选参数
        参数名 类型 说明
        User string 格式为"user=index"，index为已标定的⽤⼾坐标系索引。
        Tool string 格式为"tool=index"，index为已标定的工具坐标系索引。
        必须同时传或同时不传，不传时默认为全局⽤⼾和⼯具坐标系。
        Get the Cartesian coordinates of the current posture under the specific coordinate system.
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system.
        They need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set.
        """
        string = "GetPose("
        params = []
        state = True
        if user != -1:
            params.append('user={:d}'.format(user))
            state = not state
        if tool != -1:
            params.append('tool={:d}'.format(tool))
            state = not state
        if not state:
            return 'need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set' # 必须同时传或同时不传坐标系，不传时默认为全局用户和工具坐标系 / Coordinate systems must be passed together or not at all; defaults to global user and tool coordinate systems

        for i, param in enumerate(params):
            if i == len(params)-1:
                string = string + param
            else:
                string = string + param+","

        string = string + ')'
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        获取机械臂当前报警错误码。
        Get the active alarm IDs.
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

     #################################################################

    def DO(self, index, status, time=-1):
        """
        设置数字输出端⼝状态（队列指令）。
        必选参数
        参数名 类型 说明
        index int DO端⼦的编号
        status int DO端⼦的状态，1：ON；0：OFF
        可选参数
        参数名 类型  说明
        time int 持续输出时间，取值范围：[25, 60000]。单位：ms
        如果设置了该参数，系统会在指定时间后对DO⾃动取反。取反为异步动作，
        不会阻塞指令队列，系统执⾏了DO输出后就会执⾏下⼀条指令。
        Set the status of digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1Digital output state: ON, 0: OFF
        Optional parameter:
        Parameter name     Type     Description
        time     int     continuous output time, range: [25,60000]. Unit: ms.
        If this parameter is set, the system will automatically invert the DO after the specified time.
        The inversion is an asynchronous action, which will not block the command queue. After the DO output is executed, the system will execute the next command.
        """
        string = "DO({:d},{:d}".format(index, status)
        params = []
        if time != -1:
            params.append('{:d}'.format(time))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def DOInstant(self, index, status):
        """
        设置数字输出端⼝状态（⽴即指令）。
        必选参数
        参数名 类型 说明
        index int DO端⼦的编号
        status int DO端⼦的状态，1：ON；0：OFF
        Set the status of digital output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1: ON, 0: OFF
        """
        string = "DOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetDO(self, index):
        """
        获取数字输出端⼝状态。
        必选参数
        参数名 类型 说明
        index int DO端⼦的编号
        Get the status of digital output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        """
        string = "GetDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DOGroup(self, *index_value):
        """
        设置多个数字输出端⼝状态（队列指令）。
        必选参数
        参数名 类型 说明
        index1 int 第⼀个DO端⼦的编号
        value1 int 第⼀个DO端⼦的状态，1：ON；0：OFF
        ... ... ...
        indexN int 第N个DO端⼦的编号
        valueN int 第N个DO端⼦的状态，1：ON；0：OFF
        返回
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        ResultID为算法队列ID，可⽤于判断指令执⾏顺序。
        ⽰例
        DOGroup(4,1,6,0,2,1,7,0)
        设置DO_4为ON，DO_6为OFF，DO_2为ON，DO_7为OFF。
        Set the status of multiple digital output ports (queue command).
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DO
        value1      int     status of the first DO, 1: ON, 0: OFF
        ... ... ...
        indexN     int     index of the last DO
        valueN      int     status of the last DO, 1: ON, 0: OFF
        Return
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        ResultID is algorithm queue ID, used to judge the order in which commands are executed.
        Example
        DOGroup(4,1,6,0,2,1,7,0)
        Set DO_4 to ON, DO_6 to OFF, DO_2 to ON, DO_7 to OFF.
        """
        string = "DOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'

        return self.sendRecvMsg(string)

    def GetDOGroup(self, *index_value):
        """
        获取多个数字输出端⼝状态。
        必选参数
        参数名 类型 说明
        index int 第⼀个DO端⼦的编号
        ... ... ...
        indexN int 第N个DO端⼦的编号
        返回
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}分别表⽰DO_1到DO_N的状态，0为OFF，1为ON
        ⽰例
        GetDOGroup(1,2)
        获取DO_1和DO_2的状态。
        Get the status of multiple digital output ports.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the first DO
        ... ... ...
        indexN     int     index of the last DO
        Return
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DO_1 – DO_N. 0: OFF, 1: ON.
        Example
        GetDOGroup(1,2)
        Get the status of DO_1 and DO_2.
        """
        string = "GetDOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDO(self, index, status):
        """
        设置末端数字输出端⼝状态（队列指令）。
        必选参数
        参数名 类型 说明
        index int 末端DO端⼦的编号
        status int 末端DO端⼦的状态，1：ON；0：OFF
        Set the status of tool digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def ToolDOInstant(self, index, status):
        """
        设置末端数字输出端⼝状态（⽴即指令）
        必选参数
        参数名 类型 说明
        index int 末端DO端⼦的编号
        status int 末端DO端⼦的状态，1：ON；0：OFF
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetToolDO(self, index):
        """
        读取指定末端数字输出（DO）的状态。此指令立即执行。

        参数：
            index (int)：末端 DO 端子编号。范围为 [1, MAX]，MAX 取决于末端工具支持的端子数量。

        返回：
            控制器响应字符串，格式为 ErrorID,{value},GetToolDO(index);
            value 为输出状态，0 表示关闭（OFF），1 表示开启（ON）。

        Read the state of the specified tool digital output (DO). This is an immediate command.

        Args:
            index (int): Tool DO terminal number. Range: [1, MAX], where MAX depends
                on the number of terminals supported by the end effector.

        Returns:
            The controller response string in the format ErrorID,{value},GetToolDO(index);
            value is the output state: 0 = OFF, 1 = ON.
        """
        string = "GetToolDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def AO(self, index, value):
        """
        设置模拟输出端⼝的值（队列指令）。
        必选参数
        参数
        名
        类型 说明
        index int AO端⼦的编号
        value double AO端⼦的输出值，电压取值范围：[0,10]，单位：V；电流取值范围：[4,20]，单位：mA
        Set the value of analog output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range: [4,20], unit: mA
        """
        string = "AO({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def AOInstant(self, index, value):
        """
        设置模拟输出端⼝的值（⽴即指令）。
        必选参数
        参数名 类型 说明
        index int AO端⼦的编号
        value double AO端⼦的输出值，电压取值范围：[0,10]，单位：V；电流取值范围：
        [4,20]，单位：mA
        Set the value of analog output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range:
        [4,20], unit: mA
        """
        string = "AOInstant({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def GetAO(self, index):
        """
        获取模拟量输出端⼝的值。
        必选参数
        参数名 类型 说明
        index int AO端⼦的编号
        Get the value of analog output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        """
        string = "GetAO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DI(self, index):
        """
        获取DI端⼝的状态。
        必选参数
        参数名 类型 说明
        index int DI端⼦的编号
        Get status of DI port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DI index
        """
        string = "DI({:d})".format(index)
        return self.sendRecvMsg(string)

    def DIGroup(self, *index_value):
        """
        获取多个DI端⼝的状态。
        必选参数
        参数名 类型 说明
        index1 int 第⼀个DI端⼦的编号
        ... ... ...
        indexN int 第N个DI端⼦的编号
        返回
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}分别表⽰DI_1到DI_N的状态，0为OFF，1为ON
        ⽰例
        DIGroup(4,6,2,7)
        获取DI_4，DI_6，DI_2，DI_7的状态。
        Get status of multiple DI ports.
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DI
        ... ... ...
        indexN     int     index of the last DI
        Return
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DI_1 – DI_N. 0: OFF, 1: ON.
        Example
        DIGroup(4,6,2,7)
        Get the status of DI_4, DI_6, DI_2 and DI_7.
        """
        string = "DIGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDI(self, index):
        """
        获取末端DI端⼝的状态。
        必选参数
        参数名 类型 说明
        index int 末端DI端⼦的编号
        Get the status of tool digital input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DI
        """
        string = "ToolDI({:d})".format(index)
        return self.sendRecvMsg(string)

    def AI(self, index):
        """
        获取AI端⼝的值。
        必选参数
        参数名 类型 说明
        index int AI端⼦的编号
        Get the value of analog input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AI index
        """
        string = "AI({:d})".format(index)
        return self.sendRecvMsg(string)

    def ToolAI(self, index):
        """
        获取末端AI端⼝的值。使⽤前需要通过SetToolMode将端⼦设置为模拟输⼊模式。
        必选参数
        参数名 类型 说明
        index int 末端AI端⼦的编号
        Get the value of tool analog input port. You need to set the port to analog-input mode through SetToolMode before use.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool AI
        """
        string = "ToolAI({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool485(self, index, parity='', stopbit=-1, identify=-1):
        """
        描述:
        设置末端⼯具的RS485接⼝对应的数据格式。
        必选参数
        参数名 类型 说明
        baud int RS485接⼝的波特率
        可选参数
        参数名 类型 说明
        parity string
        是否有奇偶校验位。"O"表⽰奇校验，"E"表⽰偶校验，"N"表⽰⽆奇偶
        校验位。默认值为“N”。
        stopbit int 停⽌位⻓度。取值范围：1，2。默认值为1。
        identify int 当机械臂为多航插机型时，⽤于指定设置的航插。1：航插1；2：航插2
        返回
        ErrorID,{},SetTool485(baud,parity,stopbit);
        ⽰例：
        SetTool485(115200,"N",1)
        将末端⼯具的RS485接⼝对应的波特率设置为115200Hz，⽆奇偶校验位，停⽌位⻓度为1。
        Description:
        Set the data type corresponding to the RS485 interface of the end tool.
        Required parameter:
        Parameter name     Type     Description
        baud     int     baud rate of RS485 interface
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "N" by default.
        stopbit     int     stop bit length Range: 1, 2 1 by default.
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetTool485(baud,parity,stopbit);
        Example
        SetTool485(115200,"N",1)
        Set the baud rate corresponding to the tool RS485 interface to 115200Hz, parity bit to N, and stop bit length to 1.
        """
        string = "SetTool485({:d}".format(index)
        params = []
        if parity != '':
            params.append(parity)
        if string != -1:
            params.append('{:d}'.format(stopbit))
            if identify != -1:
                params.append('{:d}'.format(identify))
        else:
            if identify != -1:
                params.append('1,{:d}'.format(identify))  # 选择航插没设停止位，默认为1  no stop bit, 1 by default
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolPower(self, status, identify=-1):
        """
        设置末端⼯具供电状态，⼀般⽤于重启末端电源，例如对末端夹⽖重新上电初始化。如需连续调⽤
        该接⼝，建议⾄少间隔4ms以上。
        说明：
        Magician E6机器⼈不⽀持该指令，调⽤⽆效果。
        必选参数
        参数名 类型 说明
        status int 末端⼯具供电状态，0：关闭电源；1：打开电源
        可选参数
        参数名 类型 说明
        identify int 当机械臂为多航插机型时，⽤于指定设置的航插。1：航插1；2：航插2
        返回
        ErrorID,{},SetToolPower(status);
        ⽰例：
        SetToolPower(0)
        关闭末端电源。
        Set the power status of the end tool, generally used for restarting the end power, such as re-powering and re-initializing the gripper.
        If you need to call the interface continuously, it is recommended to keep an interval of at least 4 ms.
        NOTE:
        This command is not supported on Magician E6 robot, and there is no effect when calling it.
        Required parameter:
        Parameter name     Type     Description
        status    int     power status of end tool. 0: power off; 1: power on.
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolPower(status);
        Example
        SetToolPower(0)
        Power off the tool.
        """
        string = "SetToolPower({:d}".format(status)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolMode(self, mode, type, identify=-1):
        """
        描述:
        机械臂末端AI接⼝与485接⼝复⽤端⼦时，可通过此接⼝设置末端复⽤端⼦的模式。默认模式为
        485模式。
        说明：
        不⽀持末端模式切换的机械臂调⽤此接⼝⽆效果。
        必选参数
        参数名 类型 说明
        mode int 复⽤端⼦的模式，1：485模式，2：模拟输⼊模式
        type int  当mode为1时，该参数⽆效。当mode为2时，可设置模拟输⼊的模式。
                  个位表⽰AI1的模式，⼗位表⽰AI2的模式，⼗位为0时可仅输⼊个位。
        模式：
        0：0~10V电压输⼊模式
        1：电流采集模式
        2：0~5V电压输⼊模式
        例⼦：
        0：AI1与AI2均为0~10V电压输⼊模式
        1：AI2是0~10V电压输⼊模式，AI1是电流采集模式
        11：AI2和AI1都是电流采集模式
        12：AI2是电流采集模式，AI1是0~5V电压输⼊模式
        20：AI2是0~5V电压输⼊模式，AI1是0~10V电压输⼊模式
        可选参数
        参数名 类型 说明
        identify int 当机械臂为多航插机型时，⽤于指定设置的航插。1：航插1；2：航插2
        返回
        ErrorID,{},SetToolMode(mode,type);
        ⽰例：
        SetToolMode(2,0)
        设置末端复⽤端⼦为模拟输⼊，两路都是0~10V电压输⼊模式。
        Description:
        If the AI interface on the end of the robot arm is multiplexed with the 485 interface, you can set the mode of the end multiplex terminal via this interface.
        485 mode by default.
        NOTE:
        The robot arm without tool RS485 interface has no effect when calling this interface.
        Required parameter:
        Parameter name     Type     Description
        mode     int     mode of the multiplex terminal, 1: 485 mode, 2: AI mode
        type     int     When mode is 1, this parameter is invalid. When mode is 2, you can set the mode of AI.
                  The single digit indicates the mode of AI1, the tens digit indicates the mode of AI2. When the tens digit is 0, you can enter only the single digit.
        Mode:
        0: 0 – 10V voltage input mode
        1: Current collection mode
        2: 0 – 5V voltage input mode
        Example:
        0: AI1 and AI2 are 0 – 10V voltage input mode
        1: AI2 is 0 – 10V voltage input mode, AI1 is current collection mode
        11: AI2 and AI1 are current collection mode
        12: AI2 is current collection mode, AI1 is 0 – 5V voltage input mode
        20: AI2 is 0 – 5V voltage input mode, AI1 is 0 – 10V voltage input mode
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolMode(mode,type);
        Example
        SetToolMode(2,0)
        Set the mode of the end multiplex terminal to AI, both are 0 – 10V voltage input mode.
        """
        string = "SetToolMode({:d},{:d}".format(mode, type)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

     ##################################################################

    def ModbusCreate(self, ip, port, slave_id, isRTU=-1):
        """
        创建Modbus主站，并和从站建⽴连接。最多⽀持同时连接5个设备。
        必选参数
        参数名 类型 说明
        ip string 从站IP地址
        port int 从站端⼝
        slave_id int 从站ID
        可选参数
        参数名 类型 说明
        isRTU int 如果不携带或为0，建⽴modbusTCP通信； 如果为1，建⽴modbusRTU通信
        Create Modbus master, and establish connection with the slave. (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        ip     string     slave IP address
        port     int     slave port
        slave_id     int     slave ID
        Optional parameter:
        Parameter name     Type     Description
        isRTU     int     null or 0: establish ModbusTCP communication; 1: establish ModbusRTU communication
        """
        string = "ModbusCreate({:s},{:d},{:d}".format(ip, port, slave_id)
        params = []
        if isRTU != -1:
            params.append('{:d}'.format(isRTU))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusRTUCreate(self, slave_id, baud, parity='', data_bit=8, stop_bit=-1):
        """
        创建基于RS485接⼝的Modbus主站，并和从站建⽴连接。最多⽀持同时连接5个设备。
        必选参数
        参数名 类型 说明
        slave_id int 从站ID
        baud int RS485接⼝的波特率。
        可选参数
        参数名 类型 说明
        parity string
        是否有奇偶校验位。"O"表⽰奇校验，"E"表⽰偶校验，"N"表⽰⽆奇偶
        校验位。默认值为“E”。
        data_bit int 数据位⻓度。取值范围：8。默认值为8。
        stop_bit int 停⽌位⻓度。取值范围：1，2。默认值为1。
        Create Modbus master station based on RS485, and establish connection with slave station (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        slave_id     int     slave ID
        baud     int     baud rate of RS485 interface.
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "E" by default.
        data_bit     int     data bit length Range: 8 (8 by default).
        stop_bit     int     stop bit length Range: 1, 2 (1 by default).
        """
        string = "ModbusRTUCreate({:d},{:d}".format(slave_id, baud)
        params = []
        if parity != '':
            params.append('{:s}'.format(parity))
        if data_bit != 8:
            params.append('{:d}'.format(data_bit))
        if stop_bit != -1:
            params.append('{:d}'.format(stop_bit))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusClose(self, index):
        """
        和Modbus从站断开连接，释放主站。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
       Disconnect from the Modbus slave and release the master connection.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        """
        string = "ModbusClose({:d})".format(index)
        return self.sendRecvMsg(string)

    def GetInBits(self, index, addr, count):
        """
        读取Modbus从站触点寄存器（离散输⼊）地址的值。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 触点寄存器起始地址
        count int 连续读取触点寄存器的值的数量。取值范围：[1, 16]
        Read the contact register (discrete input) value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the contact register
        count     int     number of contact registers Range: [1, 16].
        """
        string = "GetInBits({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def GetInRegs(self, index, addr, count, valType=''):
        """
        按照指定的数据类型，读取Modbus从站输⼊寄存器地址的值。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 输⼊寄存器起始地址
        count int 连续读取输⼊寄存器的值的数量。取值范围：[1, 4]
        可选参数
        参数名 类型 说明
        valType string
        读取的数据类型：
        U16：16位⽆符号整数（2个字节，占⽤1个寄存器）；
        U32：32位⽆符号整数（4个字节，占⽤2个寄存器）
        F32：32位单精度浮点数（4个字节，占⽤2个寄存器）
        F64：64位双精度浮点数（8个字节，占⽤4个寄存器）
        默认为U16
        Read the input register value with the specified data type from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the input register
        count     int     number of input registers Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two registers).
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetInRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCoils(self, index, addr, count):
        """
        读取Modbus从站线圈寄存器地址的值。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 线圈寄存器起始地址
        count int 连续读取线圈寄存器的值的数量。取值范围：[1, 16]
        Read the coil register value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of coil registers Range: [1, 16].
        """
        string = "GetCoils({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def SetCoils(self, index, addr, count, valTab):
        """
        描述
        将指定的值写⼊线圈寄存器指定的地址。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 线圈寄存器起始地址
        count int 连续写⼊线圈寄存器的值的数量。取值范围：[1, 16]
        valTab string 要写⼊的值，数量与count相同
        返回
        ErrorID,{},SetCoils(index,addr,count,valTab);
        ⽰例
        SetCoils(0,1000,3,{1,0,1})
        从地址为1000的线圈寄存器开始连续写⼊3个值，分别为1，0，1。
        Description
        Write the specified value to the specified address of coil register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of values to be written to the coil register. Range: [1, 16].
        valTab     string     values to be written to the register (The number of values must equal count.)
        Return
        ErrorID,{},SetCoils(index,addr,count,valTab);
        Example
        SetCoils(0,1000,3,{1,0,1})
        Write three values (1 , 0, 1) to the coil register starting from address 1000.
        """
        string = "SetCoils({:d},{:d},{:d},{:s})".format(
            index, addr, count, valTab)
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, index, addr, count, valType=''):
        """
        按照指定的数据类型，读取Modbus从站保持寄存器地址的值。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 保持寄存器起始地址
        count int 连续读取保持寄存器的值的数量。取值范围：[1, 4]
        可选参数
        参数名 类型 说明
        valType string
        读取的数据类型：
        U16：16位⽆符号整数（2个字节，占⽤1个寄存器）；
        U32：32位⽆符号整数（4个字节，占⽤2个寄存器）
        F32：32位单精度浮点数（4个字节，占⽤2个寄存器）
        F64：64位双精度浮点数（8个字节，占⽤4个寄存器）
        默认为U16
        Read values from the specified holding register address using the selected data type.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be read to the holding register. Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two registers)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetHoldRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, index, addr, count, valTab, valType=''):
        """
        将指定的值以指定的数据类型写⼊Modbus从站保持寄存器指定的地址。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 保持寄存器起始地址
        count int 连续写⼊保持寄存器的值的数量。取值范围：[1, 4]
        valTab string 要写⼊的值，数量与count相同。
        可选参数
        参数名 类型 说明
        valType string
        写⼊的数据类型：
        U16：16位⽆符号整数（2个字节，占⽤1个寄存器）；
        U32：32位⽆符号整数（4个字节，占⽤2个寄存器）
        F32：32位单精度浮点数（4个字节，占⽤2个寄存器）
        F64：64位双精度浮点数（8个字节，占⽤4个寄存器）
        默认为U16
        Write the specified value with specified data type to the specified address of holding register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be written to the holding register. Range: [1, 4].
        valTab     string     values to be written to the register (The number of values must equal count.)
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two registers)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "SetHoldRegs({:d},{:d},{:d},{:s}".format(
            index, addr, count, valTab)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)
    ########################################################################

    def GetInputBool(self, address):
        """
        获取输⼊寄存器指定地址的bool类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-63]
        Get the value in bool type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetInputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputInt(self, address):
        """
        获取输⼊寄存器指定地址的int类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        Get the value in int type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputFloat(self, address):
        """
        获取输⼊寄存器指定地址的float类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        Get the value in float type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputBool(self, address):
        """
        获取输出寄存器指定地址的bool类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-63]
        Get the value in bool type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetOutputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputInt(self, address):
        """
        获取输出寄存器指定地址的int类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        Get the value in int type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetOutputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputFloat(self, address):
        """
        获取输出寄存器指定地址的float类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        Get the value in float type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def SetOutputBool(self, address, value):
        """
        设置输出寄存器指定地址的bool类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-63]
        value int 要设置的值，⽀持0或1
        Set the value in bool type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        value     int     value to be set (0 or 1)
        """
        string = "GetInputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputInt(self, address, value):
        """
        设置输出寄存器指定地址的int类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        value int 要设置的值，⽀持整型数
        Set the value in int type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     int     value to be set (integer)
        """
        string = "SetOutputInt({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputFloat(self, address, value):
        """
        设置输出寄存器指定地址的int类型的数值。
        必选参数
        参数名 类型 说明
        address int 寄存器地址，取值范围[0-23]
        value int 要设置的值，⽀持整型数
        Set the value in int type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     int     value to be set (integer)
        """
        string = "SetOutputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    #######################################################################

    def MovJ(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        描述
        从当前位置以关节运动⽅式运动⾄⽬标点。
        必选参数
        参数名 类型 说明
        P string ⽬标点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Move from the current position to the target position through joint motion.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJ(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode param is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovL(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        从当前位置以直线运动⽅式运动⾄⽬标点。
        必选参数
        参数名 类型 说明
        P string ⽬标点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        可选参数
        参数名 类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a    int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v    int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int 执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp  int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r   int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Move from the current position to the target position in a linear mode.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [1, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ServoJ(self, J1, J2, J3, J4, J5, J6, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        参数名 类型 含义
        参数范围
        J1 double 点J1 轴位置，单位：度 是
        J2 double 点J2 轴位置，单位：度 是
        J3 double 点J3 轴位置，单位：度 是
        J4 double 点J4 轴位置，单位：度 是
        J5 double 点J5 轴位置，单位：度 是
        J6 double 点J6 轴位置，单位：度 是
        t float 该点位的运行时间，默认0.1,单位：s 否 [0.004,3600.0]
        aheadtime float 作用类似于PID的D项，默认50，标量，无单位 否 [20.0,100.0]
        gain float 目标位置的比例增益，作用类似于PID的P项，默认500，标量，无单位 否 [200.0,1000.0]
        J1 double Required parameter. J1 axis position of the point, unit: degree
        J2 double Required parameter. J2 axis position of the point, unit: degree
        J3 double Required parameter. J3 axis position of the point, unit: degree
        J4 double Required parameter. J4 axis position of the point, unit: degree
        J5 double Required parameter. J5 axis position of the point, unit: degree
        J6 double Required parameter. J6 axis position of the point, unit: degree
        t float Optional parameter. Running time of the point, unit: s, value range: [0.004,3600.0], default value:0.1
        aheadtime float Optional parameter. Advanced time, similar to the D in PID control. Scalar, no unit, value range: [20.0,100.0], default value: 50.
        gain float Optional parameter. Proportional gain of the target position, similar to the P in PID control. Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f}".format(J1, J2, J3, J4, J5, J6)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)
    def ServoP(self, X, Y, Z, RX, RY, RZ, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        参数名 类型 含义 是否必填 参数范围
        X double X 轴位置，单位：毫米 是
        Y double Y 轴位置，单位：毫米 是
        Z double Z 轴位置，单位：毫米 是
        Rx double Rx 轴位置，单位：度 是
        Ry double Ry 轴位置，单位：度 是
        Rz double Rz 轴位置，单位：度 是
        t float 该点位的运行时间，默认0.1,单位：s 否 [0.004,3600.0]
        aheadtime float 作用类似于PID的D项，默认50，标量，无单位 否 [20.0,100.0]
        gain float 目标位置的比例放大器，作用类似于PID的P项，默认500，标量，无单位 否 [200.0,1000.0]
        X double Required parameter. X axis position, unit: mm
        Y double Required parameter. Y axis position, unit: mm
        Z double Required parameter. Z axis position, unit: mm
        Rx double Required parameter. Rx axis position, unit: degree
        Ry double Required parameter. Ry axis position, unit: degree
        Rz double Required parameter. Rz axis position, unit: degree
        t float Optional parameter. Running time of the point, unit: s, value range: [0.004,3600.0], default value: 0.1
        aheadtime float Optional parameter. Advanced time, similar to the D in PID control. Scalar, no unit, value range: [20.0,100.0], default value: 50.
        gain float Optional parameter. Proportional gain of the target position, similar to the P in PID control. Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f}".format(X, Y, Z, RX, RY, RZ)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovLIO(self, a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        从当前位置以直线运动⽅式运动⾄⽬标点，运动时并⾏设置数字输出端⼝状态。
        必选参数
        参数名 类型 说明
        P string ⽬标点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        {Mode,Distance,Index,Status}为并⾏数字输出参数，⽤于设置当机械臂运动到指定距离或百分⽐
        时，触发指定DO。可设置多组，参数具体含义如下：
        参数名 类型 说明
        Mode int 触发模式。0表⽰距离百分⽐，1表⽰距离数值
        Distance int 指定距离。
        Distance为正数时，表⽰离起点的距离；
        Distance为负数时，表⽰离⽬标点的距离；
        Mode为0时，Distance表⽰和总距离的百分⽐；取值范围：(0,100]；
        Mode为1时，Distance表⽰距离的值。单位：mm
        Index int DO端⼦的编号
        Status int 要设置的DO状态，0表⽰⽆信号，1表⽰有信号
        可选参数
        参数名  类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int 执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Move from the current position to the target position in a linear mode, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: mm.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [1, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovLIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovLIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovJIO(self,  a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, cp=-1,):
        """
        描述
        从当前位置以关节运动⽅式运动⾄⽬标点，运动时并⾏设置数字输出端⼝状态。
        必选参数
        参数名 类型 说明
        P string ⽬标点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        {Mode,Distance,Index,Status}为并⾏数字输出参数，⽤于设置当机械臂运动到指定距离或百分⽐
        时，触发指定DO。可设置多组，参数具体含义如下：
        参数名   类型  说明
        Mode int 触发模式。0表⽰距离百分⽐，1表⽰距离数值。系统会将各关节⻆合成
        ⼀个⻆度向量，并计算终点和起点的⻆度差作为运动的总距离。
        Distance int 指定距离。
        Distance为正数时，表⽰离起点的距离；
        Distance为负数时，表⽰离⽬标点的距离；
        Mode为0时，Distance表⽰和总距离的百分⽐；取值范围：(0,100]；
        Mode为1时，Distance表⽰距离的⻆度。单位：°
        Index int DO端⼦的编号
        Status int 要设置的DO状态，0表⽰⽆信号，1表⽰有信号
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Move from the current position to the target position through joint motion, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        The system will synthesise the joint angles into an angular vector and calculate the angular difference between the end point and the start point as the total distance of the motion.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: °.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovJIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Arc(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        从当前位置以圆弧插补⽅式运动⾄⽬标点。
        需要通过当前位置，圆弧中间点，运动⽬标点三个点确定⼀个圆弧，因此当前位置不能在P1和P2
        确定的直线上。
        必选参数
        参数名 类型 说明
        P1 string 圆弧中间点，⽀持关节变量或位姿变量
        P2 string 运动⽬标点，⽀持关节变量或位姿变量
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        可选参数
        参数名  类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int  执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Move from the current position to the target position in an arc interpolated mode.
        As the arc needs to be determined through the current position, through point and target point, the current position should not be in a straight line determined by P1 and P2.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [1, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Arc(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "Arc(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Circle(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, count, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        描述
        从当前位置进⾏整圆插补运动，运动指定圈数后重新回到当前位置。
        需要通过当前位置，P1，P2三个点确定⼀个整圆，因此当前位置不能在P1和P2确定的直线上，且
        三个点确定的整圆不能超出机械臂的运动范围。
        必选参数
        参数名 类型 说明
        P1 string 整圆中间点，⽀持关节变量或位姿变量
        P2 string 用于定义整圆的第二个输入点，支持关节变量或位姿变量。
        coordinateMode int  目标点的坐标值模式    0为pose方式  1为joint
        count int 进⾏整圆运动的圈数，取值范围：[1,999]。
        可选参数
        参数名  类型  说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为
        准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        Description
        Perform the specified number of full revolutions by circular interpolation and return to the starting position.
        As the circle needs to be determined through the current position, P1 and P2, the current position should not be in a straight line determined by P1 and P2, and the circle determined by the three points cannot exceed the motion range of the robot arm.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     Second input point defining the circle (joint angles or Cartesian pose).
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        count     int     Number of circles, range: [1,999].
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [1, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Circle(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        elif coordinateMode == 1:
            string = "Circle(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id='', coordtype=-1, user=-1, tool=-1):
        """
        启动或停止机器人点动。此指令立即执行，支持在工程暂停时调用。

        参数：
            axis_id (str)：点动轴及方向，区分大小写。
                关节轴为 J1 至 J6，笛卡尔轴为 X、Y、Z、Rx、Ry、Rz。
                在轴名后加 + 或 - 指定方向，例如 J2- 或 X+。
                机器人沿指定轴持续点动。调用 MoveJog() 或传入空字符串可停止点动。
            coordtype (int, 可选)：0 表示关节点动，1 表示用户坐标系，2 表示工具坐标系。
                指定关节轴时，控制器按关节点动处理并忽略此参数。
                指定笛卡尔轴时，使用 1 或 2。传入 0 时控制器返回错误码 -6。
                省略时，控制器沿用上次成功调用时的设置。
            user (int, 可选)：已标定的用户坐标系索引，范围为 [0, 50]。
            tool (int, 可选)：已标定的工具坐标系索引，范围为 [0, 50]。
            Python 封装将 coordtype、user 和 tool 的默认值 -1 视为省略该参数。

        返回：
            控制器响应字符串，格式为 ErrorID,{},MoveJog(...);

        Start or stop jogging the robot. This immediate command can be called while a project is paused.

        Args:
            axis_id (str): Case-sensitive jog axis and direction.
                Joint axes are J1 through J6. Cartesian axes are X, Y, Z, Rx, Ry, and Rz.
                Append + or - to select the direction, for example J2- or X+.
                The robot jogs continuously along the selected axis. Call MoveJog()
                or pass an empty string to stop jogging.
            coordtype (int, optional): 0 = joint jogging, 1 = user coordinate system,
                2 = tool coordinate system.
                For a joint axis, the controller uses joint jogging and ignores this argument.
                For a Cartesian axis, use 1 or 2. The controller returns error code -6 for 0.
                When omitted, the controller uses the setting from the last successful call.
            user (int, optional): Index of a calibrated user coordinate system. Range: [0, 50].
            tool (int, optional): Index of a calibrated tool coordinate system. Range: [0, 50].
            The Python wrapper treats the default value -1 for coordtype, user, and tool as omitted.

        Returns:
            The controller response string in the format ErrorID,{},MoveJog(...);
        """
        string = "MoveJog({:s}".format(axis_id)
        params = []
        if coordtype != -1:
            params.append('coordtype={:d}'.format(coordtype))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetStartPose(self, trace_name):
        """
        描述
        获取指定轨迹的第⼀个点位。
        必选参数
        参数名 类型 说明
        traceName string  轨迹⽂件名（含后缀）
        轨迹⽂件存放在/dobot/userdata/project/process/trajectory/
        如果名称包含中⽂，必须将发送端的编码⽅式设置为UTF-8，否则
        会导致中⽂接收异常
        Description
        Get the start point of the trajectory.
        Required parameter:
        Parameter name     Type     Description
        traceName     string     trajectory file name (with suffix)
        The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise
        it will cause an exception for receiving Chinese.
        """
        string = "GetStartPose({:s})".format(trace_name)
        return self.sendRecvMsg(string)

    def StartPath(self, trace_name, isConst=-1, multi=-1.0, user=-1, tool=-1):
        """
        描述
        根据指定的轨迹⽂件中的记录点位进⾏运动，复现录制的运动轨迹。
        下发轨迹复现指令成功后，⽤⼾可以通过RobotMode指令查询机械臂运⾏状态，
        ROBOT_MODE_RUNNING表⽰机器⼈在轨迹复现运⾏中，变成ROBOT_MODE_IDLE表⽰轨迹复现
        运⾏完成，ROBOT_MODE_ERROR表⽰报警。
        必选参数
        参数名 类型 说明
        traceName string
        轨迹⽂件名（含后缀）轨迹⽂件存放在/dobot/userdata/project/process/trajectory/
        如果名称包含中⽂，必须将发送端的编码⽅式设置为UTF-8，否则会导致中⽂接收异常
        可选参数
        参数名 类型 说明
        isConst int是否匀速复现。
           1表⽰匀速复现，机械臂会按照全局速率匀速复现轨迹；
           0表⽰按照轨迹录制时的原速复现，并可以使⽤multi参数等⽐缩放运
           动速度，此时机械臂的运动速度不受全局速率的影响。
        multi double 复现时的速度倍数，仅当isConst=0时有效；取值范围：[0.25, 2]，默认值为1
        user int  指定轨迹点位对应的⽤⼾坐标系索引，不指定时使⽤轨迹⽂件中记录的⽤⼾坐标系索引
        tool int 指定轨迹点位对应的⼯具坐标系索引，不指定时使⽤轨迹⽂件中记录的⼯具坐标系索引
        Description
        Move according to the recorded points in the specified trajectory file to play back the recorded trajectory.
        After the trajectory playback command is successfully delivered, you can check the robot status via RobotMode command.
        ROBOT_MODE_RUNNING: the robot is in trajectory playback, ROBOT_MODE_IDLE: trajectory playback is completed,
        ROBOT_MODE_ERROR: alarm.
        Required parameter:
        Parameter name     Type     Description
        traceName string
        trajectory file name (with suffix). The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise it will cause an exception for receiving Chinese.
        Optional parameter:
        Parameter name     Type     Description
        isConst     int     if or not to play back at a constant speed.
           1: the trajectory will be played back at the global rate at a uniform rate by the arm;
           0: the trajectory will be played back at the same speed as when it was recorded, and the motion speed can be scaled equivalently using the multi parameter, where the motion speed of the arm is not affected by the global rate.
        multi     double     Speed multiplier in playback, valid only when isConst=0. Range: [0.25, 2], 1 by default.
        user     int     User coordinate system index corresponding to the specified trajectory point (use the user coordinate system index recorded in the trajectory file if not specified).
        tool     int     tool coordinate system index corresponding to the specified trajectory point (use the tool coordinate system index recorded in the trajectory file if not specified).
        """
        string = "StartPath({:s}".format(trace_name)
        params = []
        if isConst != -1:
            params.append('isConst={:d}'.format(isConst))
        if multi != -1:
            params.append('multi={:f}'.format(multi))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        描述
        沿⼯具坐标系进⾏相对运动，末端运动⽅式为关节运动。
        必选参数
        参数名 类型 说明
        offsetX double X轴⽅向偏移量，单位：mm
        offsetY double Y轴⽅向偏移量，单位：mm
        offsetZ double Z轴⽅向偏移量，单位：mm
        offsetRx double Rx轴⽅向偏移量，单位：度
        offsetRy double Ry轴⽅向偏移量，单位：度
        offsetRz double Rz轴⽅向偏移量，单位：度
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        Description
        Perform relative motion along the tool coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        RelMovLTool - 沿工具坐标系相对直线运动
        
        参数:
            offset_x~offset_rz: float - XYZ轴偏移量(mm)，RxRyRz轴偏移量(度)
            user: int, 可选 - 用户坐标系[0,50]
            tool: int, 可选 - 工具坐标系[0,50]
            a: int, 可选 - 加速度比例[1,100]
            v: int, 可选 - 速度比例[1,100]，与speed互斥
            speed: int, 可选 - 目标速度mm/s，与v互斥，以speed为准
            cp: int, 可选 - 平滑过渡比例[0,100]，与r互斥
            r: int, 可选 - 平滑过渡半径mm，与cp互斥，以r为准
        
        返回:
            ErrorID,{ResultID},RelMovLTool(...);
        Perform a relative linear move in the tool coordinate system.

Args:
    offset_x through offset_rz (float): X, Y, and Z offsets in mm,
        and Rx, Ry, and Rz offsets in degrees.
    user (int, optional): User coordinate system index. Range: [0, 50].
    tool (int, optional): Tool coordinate system index. Range: [0, 50].
    a (int, optional): Acceleration ratio. Range: [1, 100].
    v (int, optional): Speed ratio. Range: [1, 100]. Mutually exclusive with speed.
    speed (int, optional): Target speed in mm/s. Mutually exclusive with v.
        If both are specified, speed takes precedence.
    cp (int, optional): Path blending ratio. Range: [0, 100]. Mutually exclusive with r.
    r (int, optional): Blending radius in mm. Mutually exclusive with cp.
        If both are specified, r takes precedence.

Returns:
    ErrorID,{ResultID},RelMovLTool(...);
        """
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        RelMovJUser - 沿用户坐标系相对关节运动
        
        参数:
            offset_x~offset_rz: float - XYZ轴偏移量(mm)，RxRyRz轴偏移量(度)
            user: int, 可选 - 用户坐标系[0,50]
            tool: int, 可选 - 工具坐标系[0,50]
            a: int, 可选 - 加速度比例[1,100]
            v: int, 可选 - 速度比例[1,100]
            cp: int, 可选 - 平滑过渡比例[0,100]
        
        返回:
            ErrorID,{ResultID},RelMovJUser(...);
        Perform a relative joint move with offsets expressed in the user coordinate system.

Args:
    offset_x through offset_rz (float): X, Y, and Z offsets in mm,
        and Rx, Ry, and Rz offsets in degrees.
    user (int, optional): User coordinate system index. Range: [0, 50].
    tool (int, optional): Tool coordinate system index. Range: [0, 50].
    a (int, optional): Acceleration ratio. Range: [1, 100].
    v (int, optional): Speed ratio. Range: [1, 100].
    cp (int, optional): Path blending ratio. Range: [0, 100].

Returns:
    ErrorID,{ResultID},RelMovJUser(...);
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        RelMovLUser - 沿用户坐标系相对直线运动
        
        参数:
            offset_x~offset_rz: float - XYZ轴偏移量(mm)，RxRyRz轴偏移量(度)
            user: int, 可选 - 用户坐标系[0,50]
            tool: int, 可选 - 工具坐标系[0,50]
            a: int, 可选 - 加速度比例[1,100]
            v: int, 可选 - 速度比例[1,100]，与speed互斥
            speed: int, 可选 - 目标速度mm/s，与v互斥，以speed为准
            cp: int, 可选 - 平滑过渡比例[0,100]，与r互斥
            r: int, 可选 - 平滑过渡半径mm，与cp互斥，以r为准
        
        返回:
            ErrorID,{ResultID},RelMovLUser(...);
            Perform a relative linear move in the user coordinate system.

Args:
    offset_x through offset_rz (float): X, Y, and Z offsets in mm,
        and Rx, Ry, and Rz offsets in degrees.
    user (int, optional): User coordinate system index. Range: [0, 50].
    tool (int, optional): Tool coordinate system index. Range: [0, 50].
    a (int, optional): Acceleration ratio. Range: [1, 100].
    v (int, optional): Speed ratio. Range: [1, 100]. Mutually exclusive with speed.
    speed (int, optional): Target speed in mm/s. Mutually exclusive with v.
        If both are specified, speed takes precedence.
    cp (int, optional): Path blending ratio. Range: [0, 100]. Mutually exclusive with r.
    r (int, optional): Blending radius in mm. Mutually exclusive with cp.
        If both are specified, r takes precedence.

Returns:
    ErrorID,{ResultID},RelMovLUser(...);
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJointMovJ(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, a=-1, v=-1, cp=-1):
        """
        RelJointMovJ - 沿关节坐标系相对关节运动
        
        参数:
            offset_x~offset_rz: float - J1~J6轴偏移量(度)
            a: int, 可选 - 加速度比例[1,100]
            v: int, 可选 - 速度比例[1,100]
            cp: int, 可选 - 平滑过渡比例[0,100]
        
        返回:
            ErrorID,{ResultID},RelJointMovJ(...);
        Perform a relative move in joint space using joint motion.

Args:
    offset_x through offset_rz (float): Offsets for joints J1 through J6, in degrees.
    a (int, optional): Acceleration ratio. Range: [1, 100].
    v (int, optional): Speed ratio. Range: [1, 100].
    cp (int, optional): Path blending ratio. Range: [0, 100].

Returns:
    ErrorID,{ResultID},RelJointMovJ(...);
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCurrentCommandID(self):
        """
        获取当前执⾏指令的算法队列ID，可以⽤于判断当前机器⼈执⾏到了哪⼀条指令。
        Get the algorithm queue ID of the currently executed command, which can be used to judge which command is currently being executed by the robot.
        """
        string = "GetCurrentCommandID()"
        return self.sendRecvMsg(string)

    ###################################460新增 / 460 New#############################
    
    ##轨迹恢复指令 / Trajectory Recovery Commands
    def SetResumeOffset(self, distance):
        """
       该指令仅用于焊接工艺。设置轨迹恢复的目标点位相对暂停时的点位沿焊缝回退的距离
       This command is only used for welding processes. Set the distance that the target point of trajectory recovery retreats along the weld seam relative to the paused point.
        """
        string = "SetResumeOffset({:f})".format(distance)
        return self.sendRecvMsg(string)
    
    def PathRecovery(self):
        """
        开始轨迹恢复：工程暂停后，控制机器人回到暂停时的位姿。
        Start trajectory recovery: after the project is paused, control the robot to return to the pose when paused.
        """
        string = "PathRecovery()"
        return self.sendRecvMsg(string)

    def PathRecoveryStop(self):
        """
        轨迹恢复的过程中停止机器人。
        Stop the robot during trajectory recovery.
        """
        string = "PathRecoveryStop()"
        return self.sendRecvMsg(string)

    def PathRecoveryStatus(self):
        """
        查询轨迹恢复的状态。
        Query the status of trajectory recovery.
        """
        string = "PathRecoveryStatus()"
        return self.sendRecvMsg(string)
    
    ##日志导出指令 / Log Export Commands
    def LogExportUSB(self, range):
        """
       将机器人日志导出至插在机器人控制柜USB接口的U盘根目录。
       Export robot logs to the root directory of the USB drive plugged into the USB port of the robot controller.
       导出范围。
       Export range.
        0   导出logs/all 和logs/user文件夹的内容。/ Export the contents of logs/all and logs/user folders.
        1   导出logs文件夹所有内容。/ Export all contents of the logs folder.
        """
        string = "LogExportUSB({:d})".format(range)
        return self.sendRecvMsg(string)
    
    def GetExportStatus(self):
        """
        获取日志导出的状态。
        Get the status of log export.
        其中status表示日志导出状态。
        The response contains the export status.
        0：未开始导出 / 0: Export not started
        1：导出中 / 1: Exporting
        2：导出完成 / 2: Export completed
        3：导出失败，找不到U盘 / 3: Export failed, USB drive not found
        4：导出失败，U盘空间不足 / 4: Export failed, insufficient USB drive space
        5：导出失败，导出过程中U盘被拔出 / 5: Export failed, USB drive removed during export
        导出完成和导出失败的状态会保持到下次用户使用导出功能
        The status of export completed and export failed will be maintained until the user uses the export function next time.
        """
        string = "GetExportStatus()"
        return self.sendRecvMsg(string)

    ##力控指令 / Force Control Commands
    def EnableFTSensor(self, status):
        """
       开启/关闭力传感器。
       Enable/disable the force sensor.
        """
        string = "EnableFTSensor({:d})".format(status)
        return self.sendRecvMsg(string)

    def SixForceHome(self):
        """
        将力传感器当前数值置0，即以传感器当前受力状态作为零点。
        Set the current value of the force sensor to 0, that is, take the current force state of the sensor as the zero point.
        """
        string = "SixForceHome()"
        return self.sendRecvMsg(string)
    
    def GetForce(self, tool = -1):
        """
        获取力传感器当前数值。
        Get the current value of the force sensor.
        tool int 用于指定获取数值时参考的工具坐标系，取值范围：[0,50]。
        tool int Used to specify the tool coordinate system for obtaining values, range: [0,50].
        不指定时使用全局工具坐标系
        If not specified, the global tool coordinate system is used.
        """
        if tool == -1:
            string = "GetForce()"
        else:
            string = "GetForce({:d})".format(tool)
        return self.sendRecvMsg(string)

    def ForceDriveMode(self, x, y, z, rx, ry, rz, user=-1):
        """
        指定可拖拽的方向并进入力控拖拽模式。
        Specify the draggable direction and enter force control drag mode.
        {x,y,z,rx,ry,rz} string
        用于指定可拖拽的方向。
        Used to specify the draggable direction.
        0代表该方向不能拖拽，1代表该方向可以拖拽。
        0 means the direction cannot be dragged, 1 means the direction can be dragged.
        例：
        Example:
        {1,1,1,1,1,1}表示机械臂可在各轴方向上自由拖动
        {1,1,1,1,1,1} means the robot arm can be dragged freely in all axis directions.
        {1,1,1,0,0,0}表示机械臂仅可在XYZ轴方向上拖动
        {1,1,1,0,0,0} means the robot arm can only be dragged in the XYZ axis directions.
        {0,0,0,1,1,1}表示机械臂仅可在RxRyRz轴方向上旋转
        {0,0,0,1,1,1} means the robot arm can only rotate in the RxRyRz axis directions.
        """
        string = ""
        string = "ForceDriveMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"}"
        if user != -1:
            string = string + ',{:d}'.format(user)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def ForceDriveSpeed(self, speed):
        """
        设置力控拖拽速度比例。
        Set the force control drag speed ratio.
        speed int 力控拖拽速度比例，取值范围：[1,100]。
        speed int Force control drag speed ratio, range: [1,100].
        """
        string = "ForceDriveSpeed({:d})".format(speed)
        return self.sendRecvMsg(string)
    
    def FCForceMode(self, x, y, z, rx, ry, rz, fx,fy,fz,frx,fry,frz, reference=-1, user=-1,tool=-1):
        """
        以用户指定的配置参数开启力控。
        Enable force control with user-specified configuration parameters.
        {x,y,z,rx,ry,rz} 
            开启/关闭笛卡尔空间某个方向的力控调节。
            Enable/disable force control adjustment in a Cartesian direction.
            0表示关闭该方向的力控。
            0 means force control in that direction is disabled.
            1表示开启该方向的力控。
            1 means force control in that direction is enabled.
        {fx,fy,fz,frx,fry,frz} 
            目标力：是工具末端与作用对象之间接触力的目标值，是一种模拟力，可以由用户自行设定；目标力方向分别对应笛卡尔空间的{x,y,z,rx,ry,rz}方向。
            Target force: target value of contact force between tool end and acting object, a simulated force settable by user; directions correspond to {x,y,z,rx,ry,rz} of Cartesian space.
            位移方向的目标力范围[-200,200]，单位N；姿态方向的目标力范围[-12,12]，单位N·m。
            Target force range for displacement [-200,200] N; for posture [-12,12] N·m.
            目标力为0时处于柔顺模式，柔顺模式与力控拖动类似。
            When target force is 0, in compliant mode, similar to force control dragging.
        如果某个方向未开启力控调节，则该方向的目标力也不会生效。
        If force control is not enabled in a direction, the target force there will not take effect.
        reference 
            格式为“reference=value”。value表示参考坐标系，默认参考工具坐标系。
            reference=0表示参考工具坐标系，即沿工具坐标系进行力控调节。
            reference=0 means reference tool coordinate system, force control along tool coordinate system.
            reference=1表示参考用户坐标系，即沿用户坐标系进行力控调节。
            reference=1 means reference user coordinate system, force control along user coordinate system.
        user  
            格式为"user=index"，index为已标定的用户坐标系索引。取值范围：[0,50]。
            Format is "user=index", index is calibrated user coordinate system index. Range: [0,50].
        tool  
            格式为"tool=index"，index为已标定的工具坐标系索引。取值范围：[0,50]。
            Format is "tool=index", index is calibrated tool coordinate system index. Range: [0,50].
        """
        string = "FCForceMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"},"+"{"+"{:g},{:g},{:g},{:g},{:g},{:g}".format(fx,fy,fz,frx,fry,frz)+"}"
        params = []
        if reference != -1:
            params.append('reference={:d}'.format(reference))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def FCSetDeviation(self, x, y, z, rx, ry, rz, controltype=-1):
        """
        设置力控模式下的位移和姿态偏差，若力控过程中恒力偏移了较大的距离，机器人会进行相应处理。
        Set the displacement and posture deviation in force control mode. If the constant force deviates by a large distance during force control, the robot will handle it accordingly.
        x、y、z
        代表力控模式下的位移偏差，单位为mm。取值范围：(0,1000]，默认值100mm。
        Represents the displacement deviation in force control mode, unit mm. Range: (0,1000], default 100mm.
        rx、ry、rz
        代表力控模式下的姿态偏差，单位为度。取值范围：(0,360]，默认值36度。
        Represents the posture deviation in force control mode, unit degree. Range: (0,360], default 36 degrees.
        controltype
        表示力控过程中超过规定阈值时，机械臂的处理方式。
        Indicates the handling method of the robot arm when the specified threshold is exceeded during force control.
        0：超过阈值时，机械臂报警（默认值）。
        0: When the threshold is exceeded, the robot arm alarms (default).
        1：超过阈值时，机械臂停止搜寻而在原有轨迹上继续运动。
        1: When the threshold is exceeded, the robot arm stops searching and continues moving on the original trajectory.
        """
        string = "FCSetDeviation("+"{"+"{:g},{:g},{:g},{:g},{:g},{:g}".format(x,y,z,rx,ry,rz)+"}"
        if controltype != -1:
            string = string + ',{:d}'.format(controltype)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetForceLimit(self, x, y, z, rx, ry, rz):
        """
        设置各方向的最大力限制（该设置对所有方向均生效，包含未启用力控的方向）。TCP模式退出后，参数恢复默认值。
        Set the maximum force limit in each direction (this setting applies to all directions, including directions where force control is not enabled).  The parameters revert to their default values when TCP/IP control mode ends.
        x,y,z：取值范围(0,500]，默认值500。
        x,y,z: Range (0,500], default 500.
        rx,ry,rz：取值范围(0,50]，默认值50。
        rx,ry,rz: Range (0,50], default 50.
        """
        string = "FCSetForceLimit({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)
    
    def FCSetMass(self, x, y, z, rx, ry, rz):
        """
        设置力控模式下各方向的惯性系数。TCP模式退出后，参数恢复默认值。
        Set the inertia coefficient in each direction under force control mode. After exiting TCP mode, the parameters restore to default values.
        x,y,z,rx,ry,rz：取值范围(0,10000]，默认值20。
        x,y,z,rx,ry,rz: Range (0,10000], default 20.
        """
        string = "FCSetMass({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)
    
    def FCSetStiffness(self, x, y, z, rx, ry, rz):
        """
        设置力控模式下各方向的弹性系数。TCP模式退出后，参数恢复默认值。
        Set the stiffness coefficient in each direction under force control mode. After exiting TCP mode, the parameters restore to default values.
        x,y,z,rx,ry,rz：取值范围[0,10000]，默认值30。
        x,y,z,rx,ry,rz: Range [0,10000], default 30.
        """
        string = "FCSetStiffness({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)
    
    def FCSetDamping(self, x, y, z, rx, ry, rz):
        """
        设置力控模式下各方向的阻尼系数。TCP模式退出后，参数恢复默认值。
        Set the damping coefficient in each direction under force control mode. After exiting TCP mode, the parameters restore to default values.
        x,y,z,rx,ry,rz：取值范围[0,1000]，默认值50。
        x,y,z,rx,ry,rz: Range [0,1000], default 50.
        """
        string = "FCSetDamping({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)

    def FCOff(self):
        """
        退出力控模式，与FCForceMode配合使用，两者之间的运动指令都会进行力的柔顺控制。
        Exit force control mode. Used together with FCForceMode, motion commands between them will perform force compliance control.
        """
        string = "FCOff()"
        return self.sendRecvMsg(string)

    def FCSetForceSpeedLimit(self, x, y, z, rx, ry, rz):
        """
        设置各方向的力控调节速度。力控速度上限较小时，力控调节速度较慢，适合低速平缓的接触面。
        Set the force control adjustment speed in each direction. When the upper limit is small, adjustment is slow, suitable for low-speed smooth contact surfaces.
        力控速度上限较大时，力控调节速度快，适合高速力控应用。需要根据具体的应用场景进行调整。
        When the upper limit is large, adjustment is fast, suitable for high-speed force control applications. Adjust according to specific application scenarios.
        TCP模式退出后，参数恢复默认值。
        After exiting TCP mode, parameters restore to default values.
        x,y,z：位移方向的力控调节速度，默认值20mm/s。CRA机型取值范围：(0,安全限制TCP速度值]，其他机型：(0,300]。
        x,y,z: Force control adjustment speed in displacement direction, default 20mm/s. CRA models: (0, safety limit TCP speed], others: (0,300].
        rx,ry,rz：姿态方向的力控调节速度，默认值20°/s。CRA机型取值范围：(0,(4安全限制TCP速度值x0.001/3.14x180)]，其他机型：(0,90]。
        rx,ry,rz: Force control adjustment speed in posture direction, default 20°/s. CRA models: (0,(4*safety limit TCP speed*0.001/3.14*180)], others: (0,90].
        """
        string = "FCSetForceSpeedLimit({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)
    
    def FCSetForce(self, x, y, z, rx, ry, rz):
        """
        实时调整各方向的恒力设置。
        Adjust the constant force setting in each direction in real time.
        x,y,z：取值范围[-200,200]，单位N。
        x,y,z: Range [-200,200], unit N.
        rx,ry,rz：取值范围[-12,12]，单位N·m。
        rx,ry,rz: Range [-12,12], unit N·m.
        """
        string = "FCSetForce({:g},{:g},{:g},{:g},{:g},{:g})".format(x,y,z,rx,ry,rz)
        return self.sendRecvMsg(string)
    
    def RequestControl(self):
        """
        RequestControl - 请求将设备控制模式切换为TCP模式
        RequestControl - Request to switch device control mode to TCP mode
        
        只有在TCP模式下才可执行其他TCP指令。
        Only in TCP mode can other TCP commands be executed.
        
        返回: / Returns:
            ErrorID,{},RequestControl();
        
        注意: / Notes:
            仅当机器人处于未上电或下使能（且非暂停或松抱闸状态）时才可切换TCP模式。
            TCP mode can only be switched when the robot is powered off or disabled (and not in pause or brake-released state).
            
            允许切换TCP模式的场景: / Scenarios allowing TCP mode switch:
            - 未上电: 允许 / Powered off: Allowed
            - 下使能（非暂停状态、非抱闸松开状态）: 允许 / Disabled (not paused, not brake-released): Allowed
            - 使能空闲: 不允许 / Enabled idle: Not allowed
            - 拖拽模式: 不允许 / Drag mode: Not allowed
            - 单次运动中: 不允许 / Single move: Not allowed
            - 运行中: 不允许 / Running: Not allowed
            - 暂停: 不允许 / Paused: Not allowed
            - 错误（上使能情况下）: 不允许 / Error (enabled): Not allowed
            - 松抱闸: 不允许 / Brake released: Not allowed
            - 开启手自动模式: 不允许 / Manual/auto mode on: Not allowed
        """
        string = "RequestControl()"
        return self.sendRecvMsg(string)
    
    ## 新增运动指令 / New Motion Commands

    def RelPointTool(self, coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        RelPointTool - 沿工具坐标系笛卡尔点偏移
        RelPointTool - Cartesian point offset along tool coordinate system
        
        参数: / Parameters:
            coordinateMode: int - 坐标模式，0=pose位姿, 1=joint关节 / coordinate mode, 0=pose, 1=joint
            a1~f1: float - 起始点位(x,y,z,rx,ry,rz)或(j1,j2,j3,j4,j5,j6) / start point
            x,y,z,rx,ry,rz: float - 偏移量 / offset
        
        返回: / Returns:
            ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointTool(...);
        """
        string = ""
        if coordinateMode == 0:
            string = "RelPointTool(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointTool(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string = string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def RelPointUser(self,coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        RelPointUser - 沿用户坐标系笛卡尔点偏移
        RelPointUser - Cartesian point offset along user coordinate system
        
        参数: / Parameters:
            coordinateMode: int - 坐标模式，0=pose位姿, 1=joint关节 / coordinate mode, 0=pose, 1=joint
            a1~f1: float - 起始点位(x,y,z,rx,ry,rz)或(j1,j2,j3,j4,j5,j6) / start point
            x,y,z,rx,ry,rz: float - 偏移量 / offset
        
        返回: / Returns:
            ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointUser(...);
        """
        string = ""
        string = ""
        if coordinateMode == 0:
            string = "RelPointUser(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointUser(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string =string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJoint(self, j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6):
        """
        RelJoint - 关节点位偏移
        RelJoint - Joint point offset
        
        参数: / Parameters:
            j1~j6: float - 原始关节位置，单位：度 / original joint positions, unit: degree
            offset1~offset6: float - 关节偏移量，单位：度 / joint offsets, unit: degree
        
        返回: / Returns:
            ErrorID,{J1,J2,J3,J4,J5,J6},RelJoint(...);
        """
        string = "RelJoint({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6)
        return self.sendRecvMsg(string)
    
    def GetError(self, language="zh_cn"):
        """
        通过 HTTP 获取机器人报警信息。
        Retrieve robot alarm information over HTTP.
        参数: / Parameters:
        language (str): 报警语言，默认为 "zh_cn"。 / Alarm language. Defaults to "zh_cn".
                 "zh_cn" - 简体中文 / Simplified Chinese
                 "zh_hant" - 繁体中文 / Traditional Chinese  
                 "en" - 英语 / English
                 "ja" - 日语 / Japanese
                 "de" - 德语 / German
                 "vi" - 越南语 / Vietnamese
                 "es" - 西班牙语 / Spanish
                 "fr" - 法语 / French
                 "ko" - 韩语 / Korean
                 "ru" - 俄语 / Russian
        返回: / Returns:
        请求成功时返回解析后的 JSON，报警响应通常为包含 errMsg 列表的字典。
        报警查询返回非 HTTP 200 状态、请求异常或 JSON 解析失败时，返回 {"errMsg": []}。
        查询成功且无报警时也可能返回空列表。调用方不能仅凭空列表判断查询是否成功。

        On success, return the decoded JSON. An alarm response typically contains an errMsg list.
        Returns {"errMsg": []} if the alarm endpoint returns a non-200 HTTP status, the request fails,
        or JSON decoding fails. A successful query with no alarms can also return an empty list.
        An empty list alone does not indicate whether the query succeeded.

        报警响应格式 / Alarm response format:
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
        try:
            # 首先设置语言 / First set the language
            language_url = f"http://{self.ip}:22000/interface/language"
            language_data = {"type": language}
            
            # 发送POST请求设置语言 / Send POST request to set language
            response = requests.post(language_url, json=language_data, timeout=5)
            if response.status_code != 200:
                print(f"Failed to set language: HTTP {response.status_code}")  # 设置语言失败 / Failed to set language
            
            # 获取报警信息 / Get alarm information
            alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
            response = requests.get(alarm_url, timeout=5)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Failed to get alarm information: HTTP {response.status_code}")  # 获取报警信息失败 / Failed to get alarm information
                return {"errMsg": []}
                
        except requests.exceptions.RequestException as e:
            print(f"HTTP request exception: {e}")  # HTTP请求异常 / HTTP request exception
            return {"errMsg": []}
        except json.JSONDecodeError as e:
            print(f"JSON parsing exception: {e}")  # JSON解析异常 / JSON parsing exception
            return {"errMsg": []}
        except Exception as e:
            print(f"Unknown error while getting alarm information: {e}")  # 获取报警信息时发生未知错误 / Unknown error while getting alarm information
            return {"errMsg": []}

    def ArcIO(self, a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2, coordinateMode, *io_params, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=-1):
        """
        圆弧运动过程中并行设置数字输出端口的状态，可设置多组。
        Set the status of digital output ports in parallel during arc motion, multiple groups can be set.
        """
        string = ""
        if coordinateMode == 0:
            string = "ArcIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "ArcIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        
        for io_param in io_params:
            if isinstance(io_param, (list, tuple)) and len(io_param) == 4:
                string += ",{{{:d},{:d},{:d},{:d}}}".format(*io_param)
            else:
                 print("io_param format is wrong")

        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        if mode != -1:
            params.append('mode={:d}'.format(mode))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def ArcTrackParams(self, sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset):
        string = "ArcTrackParams({:d},{:d},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset)
        return self.sendRecvMsg(string)

    def ArcTrackEnd(self):
        return self.sendRecvMsg("ArcTrackEnd()")

    def CheckOddMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovC(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovJ(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovL(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvInit(self, index):
        """
        CnvInit - 开启传送带 立即指令
        CnvInit - Start conveyor belt, immediate command
        
        开启传送带并下发传送带配置信息。删除所有队列信息，开始检测并存储新的队列信息。
        Start the conveyor belt and send configuration information. Delete all queue information, start detecting and storing new queue information.
        
        原型: / Prototype:
            CnvInit(index)
        
        参数: / Parameters:
            index: int - 传送带编号1/2/3，机器人最多支持三条传送带。设置为其他数值会报错。 / Conveyor belt number 1/2/3, robot supports up to three conveyor belts. Other values will cause an error.
        
        返回: / Returns:
            ErrorID,{},CnvInit(index);
        
        示例: / Example:
            CnvInit(1)  # 开启1号传送带并下发传送带配置信息 / Start conveyor belt 1 and send configuration
        """
        string = "CnvInit({:d})".format(index)
        return self.sendRecvMsg(string)

    def CnvMovL(self, j1, j2, j3, j4, j5, j6, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1):
        """
        CnvMovL - 执行传动带跟随，采取直线轨迹插补 队列指令
        CnvMovL - Execute conveyor belt following with linear trajectory interpolation, queue command
        
        基于工件坐标系，机器人直线运动到目标点位执行传送带跟踪。
        Based on the workpiece coordinate system, the robot moves linearly to the target point to perform conveyor belt tracking.
        
        原型: / Prototype:
            CnvMovL(P,user, tool, a, v, cp|r)
        
        参数: / Parameters:
            j1~j6: float - 目标点，支持关节变量(j1,j2,j3,j4,j5,j6)或位姿变量(x,y,z,rx,ry,rz) / Target point, supports joint or pose variables
            user: int, 可选 - 用户坐标系索引，取值范围：[0,50] / optional - user coordinate system index, range: [0,50]
            tool: int, 可选 - 工具坐标系索引，取值范围：[0,50] / optional - tool coordinate system index, range: [0,50]
            a: int, 可选 - 机器人运动加速度比例，取值范围：[1,100] / optional - robot motion acceleration ratio, range: [1,100]
            v: int, 可选 - 机器人运动速度比例，取值范围：[1,100] / optional - robot motion speed ratio, range: [1,100]
            cp: int, 可选 - 平滑过渡比例，与r互斥，取值范围：[0,100] / optional - CP ratio, mutually exclusive with r, range: [0,100]
            r: int, 可选 - 平滑过渡半径，单位mm，与cp互斥，若同时存在以r为准 / optional - blending radius in mm, mutually exclusive with cp, r takes precedence
        
        返回: / Returns:
            ErrorID,{flag},CnvMovL(P,user, tool, a, v, cp|r);
            flag: 0-执行成功, 1-跟随失败未检测到工件类型, 2-跟随失败工件未进入拾取边界范围, 3-跟随失败工件超出离开边界
            flag: 0-success, 1-follow failed workpiece type not detected, 2-follow failed workpiece not in pick boundary, 3-follow failed workpiece beyond leave boundary
        
        示例:
            CnvMovL(x,y,z,rx,ry,rz, user=1, tool=0, a=20, v=50, cp=100)
        """
        string = "CnvMovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1, j2, j3, j4, j5, j6)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1, mode=1):
        """
        CnvMovC - 执行传动带跟随，采取圆弧轨迹插补 队列指令
        CnvMovC - Execute conveyor belt following with arc trajectory interpolation, queue command
        
        基于工件坐标系，机器人从当前位置通过圆弧运动方式经由中间点位P1运动至目标点位P2后，
        执行传送带跟踪。
        Based on the workpiece coordinate system, the robot moves from current position via arc motion through intermediate point P1 to target point P2, then performs conveyor belt tracking.
        
        原型: / Prototype:
            CnvMovC(P1,P2,user, tool, a, v, cp|r, mode)
        
        参数: / Parameters:
            j1a~j6a: float - 圆弧中间点P1，支持关节变量或位姿变量 / Arc intermediate point P1, supports joint or pose variables
            j1b~j6b: float - 运动目标点P2，支持关节变量或位姿变量 / Motion target point P2, supports joint or pose variables
            user: int, 可选 - 用户坐标系索引，取值范围：[0,50] / optional - user coord system index, range: [0,50]
            tool: int, 可选 - 工具坐标系索引，取值范围：[0,50] / optional - tool coord system index, range: [0,50]
            a: int, 可选 - 机械臂运动加速度比例，取值范围：[1,100] / optional - acceleration ratio, range: [1,100]
            v: int, 可选 - 机械臂运动速度比例，取值范围：[1,100] / optional - speed ratio, range: [1,100]
            cp: int, 可选 - 平滑过渡比例，与r互斥，取值范围：[0,100] / optional - CP ratio, mutually exclusive with r, range: [0,100]
            r: int, 可选 - 平滑过渡半径，单位mm，与cp互斥，若同时存在以r为准 / optional - blending radius mm, mutually exclusive with cp, r takes precedence
            mode: int, 可选 - 姿态控制参数，取值范围：[0, 2] / optional - posture control parameter, range: [0, 2]
                  mode=0: 线性模式 / Linear mode
                  mode=1: 过中间点模式 / Via intermediate point mode
                  mode=2: 固定模式 / Fixed mode
        
        注意: / Notes:
            当设置为mode=1时，为保证圆弧运动速度均匀性，示教时尽可能保证中间点处于实际圆弧一半。
            When mode=1, to ensure uniform arc motion speed, keep the intermediate point at half of the actual arc during teaching.
            当设置为mode=1时，需适当调整各点姿态，否则姿态曲线可能超出机器人可达范围，运行时报错。
            When mode=1, adjust posture of each point properly, otherwise posture curve may exceed robot reachable range and cause runtime error.
        
        返回: / Returns:
            ErrorID,{flag},CnvMovC(P1,P2,user, tool, a, v, cp|r, mode);
            flag: 0-执行成功, 1-跟随失败未检测到工件类型, 2-跟随失败工件未进入拾取边界范围, 3-跟随失败工件超出离开边界
            flag: 0-success, 1-follow failed type not detected, 2-follow failed not in pick boundary, 3-follow failed beyond leave boundary
        
        示例:
            CnvMovC(j1,j2,j3,j4,j5,j6, j7,j8,j9,j10,j11,j12, user=1, tool=0, a=20, v=50, cp=100)
        """
        string = "CnvMovC(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if mode != 1: params.append('mode={:d}'.format(mode))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CreateTray(self, trayName, dims, points):
        """
        CreateTray - 创建托盘，支持创建一维、二维和三维的托盘
        CreateTray - Create tray, supports 1D, 2D, and 3D trays
        
        最多可创建20个托盘，创建同名的托盘时会覆盖已有的托盘，不会增加托盘数量。
        Up to 20 trays can be created. Creating a tray with the same name overwrites the existing one without increasing the count.
        
        原型: / Prototype:
            CreateTray(Trayname, {Count}, {P1,P2}) -- 1D tray / 一维托盘
            CreateTray(Trayname, {row,col}, {P1,P2,P3,P4}) -- 2D tray / 二维托盘
            CreateTray(Trayname, {row,col,layer}, {P1..P8}) -- 3D tray / 三维托盘
        
        参数: / Parameters:
            trayName: string - 托盘名称，最长32字节，不允许为纯数字或纯空格 / Tray name, max 32 bytes, cannot be pure numbers or spaces
            dims: list - 根据托盘维度不同内容不同 / Varies by tray dimension:
                        一维托盘: [Count] - 点位数量，范围[2,50] / point count, range [2,50]
                        二维托盘: [row, col] - 行/列方向点位数量，范围[2,50] / row/col point count, range [2,50]
                        三维托盘: [row, col, layer] - 行/列/层点位数量，范围[2,50] / row/col/layer count, range [2,50]
            points: list - 托盘顶点列表，每点为6元素tuple/list / Tray vertex list, each point is 6-element tuple/list:
                        一维托盘: [P1, P2] - 2 endpoints / 2个端点
                        二维托盘: [P1, P2, P3, P4] - 4 vertices / 4个顶点
                        三维托盘: [P1..P8] - 8 vertices / 8个顶点
        
        返回: / Returns:
            ErrorID,{},CreateTray( ... );
        
        示例: / Example:
            # 创建名称为t1的5个点的一维托盘 / Create 1D tray named t1 with 5 points
            CreateTray("t1", [5], [(x1,y1,z1,rx1,ry1,rz1), (x2,y2,z2,rx2,ry2,rz2)])
            
            # 创建名称为t2的4x5的二维托盘 / Create 2D tray named t2 with 4x5
            CreateTray("t2", [4, 5], [P1, P2, P3, P4])
            
            # 创建名称为t3的4x5x6的三维托盘 / Create 3D tray named t3 with 4x5x6
            CreateTray("t3", [4, 5, 6], [P1, P2, P3, P4, P5, P6, P7, P8])
        """
        string = "CreateTray(" + trayName + ","
        dims_str = "{" + ",".join([str(int(d)) for d in dims]) + "}"
        string += dims_str + ","
        
        pts_str = []
        for pt in points:
            pts_str.append("pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
        string += "{" + ",".join(pts_str) + "}"
        
        string += ")"
        return self.sendRecvMsg(string)

    def EndRTOffset(self):
        return self.sendRecvMsg("EndRTOffset()")

    def StartRTOffset(self):
        """
        发送 StartRTOffset 指令，并返回控制器响应
        StartRTOffset command
        """
        return self.sendRecvMsg("StartRTOffset()")

    def FCCollisionSwitch(self, enable):
        """
        开启/关闭力传感器碰撞检测开关。该指令仅适用于CRAF机型。其他机型调用该指令会报错。
        Enable/disable force sensor collision detection. This command only applies to CRAF models. Other models will report an error.
        switch：0表示关闭力传感器碰撞检测功能；1表示开启力传感器碰撞检测功能。
        switch: 0 means disable force sensor collision detection; 1 means enable it.
        """
        return self.sendRecvMsg("FCCollisionSwitch({:d})".format(enable))

    def SetFCCollision(self, force, torque):
        """
        设置力传感器碰撞检测的阈值参数，当机器人末端的碰撞力或碰撞力矩超过设定的阈值后，机器人根据设置暂停或停止运动。
        Set threshold parameters for force sensor collision detection. When collision force or torque at robot end exceeds the threshold, the robot pauses or stops motion according to settings.
        该指令仅适用于CRAF机型。其他机型调用该指令会报错。
        This command only applies to CRAF models. Other models will report an error.
        force：触发碰撞检测的力阈值，单位N。CR5AF范围[5,150]；CR10AF范围[5,300]；CR20AF范围[5,500]。
        force: Force threshold for collision detection in N. CR5AF range [5,150]; CR10AF [5,300]; CR20AF [5,500].
        torque：触发碰撞检测的力矩阈值，单位N·m。CR5AF范围[0.5,15]；CR10AF范围[0.5,30]；CR20AF范围[0.5,50]。
        torque: Torque threshold for collision detection in N·m. CR5AF range [0.5,15]; CR10AF [0.5,30]; CR20AF [0.5,50].
        """
        return self.sendRecvMsg("SetFCCollision({:g},{:g})".format(force, torque))

    def GetCnvObject(self, objId):
        """
        GetCnvObject - 等待指定工件进入传送带的抓取区域 立即指令
        GetCnvObject - Wait for specified workpiece to enter conveyor pick area, immediate command
        
        等待指定工件进入传送带的抓取区域（即拾取上边界与拾取下边界组成的区域）。
        Wait for the specified workpiece to enter the conveyor pick area (the area between pick upper and lower boundaries).
        
        原型: / Prototype:
            GetCnvObject(objId)
        
        参数: / Parameters:
            objId: int - 工件类型，取值范围 [0, 15] / Workpiece type, range [0, 15]
                   0: 不指定工件类型，获取最先进入队列的工件信息 / Do not specify type, get first workpiece in queue
                   1~15: 获取最先进入队列的指定工件信息 / Get first specified workpiece in queue
        
        返回: / Returns:
            ErrorID,{flag, objId, objframe},GetCnvObject(objId);
            flag: 0-没有工件, 1-有工件, -1-执行错误重新执行, -2-有错误未处理, -3-非跟踪初始化状态需执行CnvInit或StopSyncCnv
            flag: 0-no workpiece, 1-workpiece exists, -1-execution error retry, -2-unhandled error, -3-not tracking init state, run CnvInit or StopSyncCnv
            objId: 工件类型号，仅当参数objId为0时该返回值有意义 / Workpiece type, only meaningful when objId is 0
            objframe: 当前时刻的工件坐标系（参考机器人基坐标系） / Current workpiece coordinate system (relative to robot base)
        
        示例:
            GetCnvObject(0)
        """
        return self.sendRecvMsg("GetCnvObject({:d})".format(objId))

    def DOGroupDEC(self, group, value):
        return self.sendRecvMsg("DOGroupDEC({:d},{:d})".format(group, value))

    def GetDOGroupDEC(self, group, value):
        return self.sendRecvMsg("GetDOGroupDEC({:d},{:d})".format(group, value))

    def DIGroupDEC(self, group, value):
        return self.sendRecvMsg("DIGroupDEC({:d},{:d})".format(group, value))

    def MovS(self, file=None, coordinateMode=-1, points=None, user=-1, tool=-1, v=-1, speed=-1, a=-1, freq=-1):
        """
        MovS - 拟合指定的轨迹
        MovS - Fit the specified trajectory
        
        调用该指令前需要用户自行运行机械臂到轨迹的起始点。
        Before calling this command, the user needs to move the robot arm to the trajectory start point.
        
        原型: / Prototype:
            MovS(P1,P2,P3,... ,user,tool,a,v|speed,freq)
            MovS(file,user,tool,a,v|speed,freq)
        
        参数: / Parameters:
            file: string, 可选 - 待拟合的轨迹文件，格式为"x.csv" / optional - trajectory file to fit, format "x.csv"
            coordinateMode: int, 可选 - 坐标模式，0=位姿(pose)，1=关节(joint) / optional - 0=pose, 1=joint
            points: list, 可选 - 待拟合点位列表，每点为6元素tuple/list，数量范围[4,50] / optional - point list, each 6-element, count [4,50]
            user: int, 可选 - 用户坐标系索引，范围[0,50] / optional - user coord system index, range [0,50]
            tool: int, 可选 - 工具坐标系索引，范围[0,50] / optional - tool coord system index, range [0,50]
            a: int, 可选 - 加速度比例，范围[1,100] / optional - acceleration ratio, range [1,100]
            v: int, 可选 - 速度比例，范围[1,100]，与speed互斥 / optional - speed ratio, range [1,100], mutually exclusive with speed
            speed: int, 可选 - 目标速度，单位mm/s，与v互斥，同时存在以speed为准 / optional - target speed mm/s, mutually exclusive with v, speed takes precedence
            freq: float, 可选 - 滤波系数，范围(0,1]，默认1 / optional - filter coefficient, range (0,1], default 1
        
        注意: / Notes:
            该指令必须输入点位列表points或轨迹文件file其中一个参数
            This command requires either points list or trajectory file parameter.
            CAD输出的轨迹可以设置freq为1，保证精度
            For CAD output trajectories, set freq to 1 to ensure accuracy.
            如果是3D相机等曲线，建议打开滤波(freq<1)，保证曲线的平滑
            For 3D camera curves, enable filtering (freq<1) to ensure smoothness.
        
        返回:
            ErrorID,{},MovS(P1,P2,P3,... ,user,tool,a,v|speed,freq);
            ErrorID,{},MovS(file,user,tool,a,v|speed,freq);
        """
        string = "MovS("
        if file is not None:
             string += "file={:s}".format(file)
        elif points is not None and coordinateMode != -1:
             # points should be a list of tuples/lists
             #points应为列表，其中每个元素为元组或列表。
             pts_str = []
             for pt in points:
                 if coordinateMode == 0:
                     pts_str.append("pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
                 elif coordinateMode == 1:
                     pts_str.append("joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
             string += ",".join(pts_str)
        else:
             print("MovS param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if a != -1:
             params.append('a={:d}'.format(a))
        if freq != -1:
             params.append('freq={:f}'.format(freq))
             
        if len(params) > 0:
             if file is not None or (points is not None and len(points) > 0):
                  string += ","
             string += ",".join(params)
             
        string += ")"
        return self.sendRecvMsg(string)

    def OffsetPara(self, x, y, z, rx, ry, rz):
        """
        设置坐标系偏移值
        Set the coordinate system offset.
        """
        string = "OffsetPara({:f},{:f},{:f},{:f},{:f},{:f})".format(x, y, z, rx, ry, rz)
        return self.sendRecvMsg(string)


    def GetTrayPoint(self, *args, **kwargs):
        """
        GetTrayPoint 指令
        由于缺少该指令精确参数的文档说明，
        本函数采用动态参数方式实现。
        示例：GetTrayPoint("t1", 3)
        GetTrayPoint command
        Due to missing documentation on exact parameters, this function uses dynamic arguments.
        Example: GetTrayPoint("t1", 3)
        """
        return self.sendRecvMsg(self._build_cmd("GetTrayPoint", *args, **kwargs))

    def RunTo(self, a1, b1, c1, d1, e1, f1, moveType, user=-1, tool=-1, a=-1, v=-1):
        """
        发送 RunTo 指令，使机器人从当前位置运动至指定点位。
        RunTo command
        """
        string = ""
        if moveType == 0:
            string = "RunTo(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=0".format(
                a1, b1, c1, d1, e1, f1)
        elif moveType == 1:
            string = "RunTo(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=1".format(
                a1, b1, c1, d1, e1, f1)
        else:
             print("moveType param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def SetArcTrackOffset(self, offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz):
        string = "SetArcTrackOffset({{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz)
        return self.sendRecvMsg(string)

    def SetCnvPointOffset(self, xOffset, yOffset):
        """
        SetCnvPointOffset - 设置传送带用户坐标系下X、Y方向的偏移量 立即指令
        SetCnvPointOffset - Set X/Y offset in conveyor user coordinate system, immediate command
        
        原型: / Prototype:
            SetCnvPointOffset(xOffset, yOffset)
        
        参数: / Parameters:
            xOffset: double - X轴方向偏移量，单位mm / X-axis offset in mm
            yOffset: double - Y轴方向偏移量，单位mm / Y-axis offset in mm
        
        返回: / Returns:
            ErrorID,{},SetCnvPointOffset(xOffset, yOffset);
        
        示例: / Example:
            SetCnvPointOffset(10, 10)
        """
        return self.sendRecvMsg("SetCnvPointOffset({:f},{:f})".format(xOffset, yOffset))

    def SetCnvTimeCompensation(self, time):
        """
        SetCnvTimeCompensation - 设置补偿时间 立即指令
        SetCnvTimeCompensation - Set compensation time, immediate command
        
        设置补偿时间，补偿因视觉触发带来的时间延时导致工件抓取位置偏移。
        Set compensation time to offset workpiece pick position deviation caused by visual trigger time delay.
        
        原型: / Prototype:
            SetCnvTimeCompensation(time)
        
        参数: / Parameters:
            time: int - 补偿时间，单位ms / compensation time in ms
        
        返回: / Returns:
            ErrorID,{},SetCnvTimeCompensation(time);
        
        示例: / Example:
            SetCnvTimeCompensation(100)
        """
        return self.sendRecvMsg("SetCnvTimeCompensation({:d})".format(time))

    def StartSyncCnv(self):
        """
        StartSyncCnv - 开启传送带跟踪功能 立即指令
        StartSyncCnv - Enable conveyor belt tracking function, immediate command
        
        开启传送带跟踪功能。
        Enable conveyor belt tracking function.
        
        返回: / Returns:
            ErrorID,{},StartSyncCnv();
        
        注意: / Notes:
            需要配合StopSyncCnv()指令一起使用，StartSyncCnv()和StopSyncCnv()之间的程序
            Must be used together with StopSyncCnv(). Programs between StartSyncCnv() and StopSyncCnv()
            不得调用除CnvMovL和CnvMovC以外的其他运动指令，否则会报错。
            must not call motion commands other than CnvMovL and CnvMovC, otherwise an error will occur.
        """
        return self.sendRecvMsg("StartSyncCnv()")

    def StopSyncCnv(self):
        """
        StopSyncCnv - 停止传送带跟踪功能 立即指令
        StopSyncCnv - Stop conveyor belt tracking function, immediate command
        
        停止传送带跟踪功能。运行完该指令后才会继续执行该指令后面的其他指令。
        Stop conveyor belt tracking. Subsequent commands only execute after this command completes.
        
        返回: / Returns:
            ErrorID,{},StopSyncCnv();
        
        注意: / Notes:
            需要配合StartSyncCnv()指令一起使用，StartSyncCnv()和StopSyncCnv()之间的程序
            Must be used together with StartSyncCnv(). Programs between StartSyncCnv() and StopSyncCnv()
            不得调用除CnvMovL和CnvMovC以外的其他运动指令，否则会报错。
            must not call motion commands other than CnvMovL and CnvMovC, otherwise an error will occur.
        """
        return self.sendRecvMsg("StopSyncCnv()")

    def TcpSendAndParse(self, cmd):
        """
        将cmd作为TcpSendAndParse指令的参数发送，
        并返回控制器响应。
        TcpSendAndParse command
        """
        return self.sendRecvMsg("TcpSendAndParse(\"{:s}\")".format(cmd))

    def Sleep(self, count):
        return self.sendRecvMsg("Sleep({:d})".format(count))

    def RelPointWeldLine(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2):
        string = "RelPointWeldLine({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5])
        return self.sendRecvMsg(string)

    def RelPointWeldArc(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2, P3):
        string = "RelPointWeldArc({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5])
        return self.sendRecvMsg(string)

    def WeaveStart(self):
        return self.sendRecvMsg("WeaveStart()")

    def WeaveParams(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian, **kwargs):
        string = "WeaveParams({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f}".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        if kwargs:
            for key, value in kwargs.items():
                string += ",{}={}".format(key, value)
        string += ")"
        return self.sendRecvMsg(string)

    def WeaveEnd(self):
        return self.sendRecvMsg("WeaveEnd()")

    def WeldArcSpeedStart(self):
        return self.sendRecvMsg("WeldArcSpeedStart()")

    def WeldArcSpeed(self, speed):
        return self.sendRecvMsg("WeldArcSpeed({:f})".format(speed))

    def WeldArcSpeedEnd(self):
        return self.sendRecvMsg("WeldArcSpeedEnd()")

    def WeldWeaveStart(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian):
        string = "WeldWeaveStart({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f})".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        return self.sendRecvMsg(string)


# Feedback interface
# 反馈数据接口类


class DobotApiFeedBack(DobotApi):
    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)
        self.__MyType = []
        self.last_recv_time = time.perf_counter()
        

    def feedBackData(self):
        """
        返回机械臂状态
        Return the robot status
        """
        self.socket_dobot.setblocking(True)  # Set to blocking mode / 设置为阻塞模式
        data = bytes()
        current_recv_time = time.perf_counter() # Timer, get current time / 计时，获取当前时间
        temp = self.socket_dobot.recv(144000) # Buffer / 缓冲区
        if len(temp) > 1440:    
            temp = self.socket_dobot.recv(144000)
        #print("get:",len(temp))
        i=0
        if len(temp) < 1440:
            while i < 5 :
                #print("re-receive")  # 重新接收 / re-receive
                temp = self.socket_dobot.recv(144000)
                if len(temp) > 1440:
                    break
                i+=1
            if i >= 5:
                raise Exception("Received data packet is missing, please check the network environment")  # 接收数据包缺失，请检查网络环境 / Received data packet is missing, please check the network environment
        
        interval = (current_recv_time - self.last_recv_time) * 1000  # Convert to milliseconds / 转换为毫秒
        self.last_recv_time = current_recv_time
        #print(f"Time interval since last receive: {interval:.3f} ms")
        
        data = temp[0:1440] # Truncate to 1440 bytes / 截取1440字节
        #print(len(data))
        #print(f"Single element size of MyType: {MyType.itemsize} bytes")
        self.__MyType = None   

        if len(data) == 1440:        
            self.__MyType = np.frombuffer(data, dtype=MyType)

        return self.__MyType
        
