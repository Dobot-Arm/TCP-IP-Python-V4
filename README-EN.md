---

typora-root-url: ./picture
---

Chinese version of the README -> please [click here](./README.md)

Dobot   TCP-IP-CR-Python-V4   secondary development API interface ([TCP-IP-CR-Python V4 README](https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4))

# 1\. Introduction

**TCP-IP-CR-Python-V4** is a software development kit designed by Dobot based on Python of TCP/IP protocol. It is developed based on Python language, follows the Dobot-TCP-IP control communication protocol, connects to the device terminal via Socket, and provides users with an easy-to-use API interface. Through TCP-IP-CR-Python-V4, users can quickly connect to the Dobot device and carry out secondary development to control and use the device.

## Pre-dependency

* You can connect your computer to the network interface of the controller with a network cable, and then set the fixed IP to be in the same network segment as the controller IP. You can also connect your computer to the controller via wireless connection.
  
  When connected to LAN1 via wired connection: IP: 192.168.5.1; When connected to LAN2 via wired connection: IP: 192.168.200.1; Wireless connection: IP: 192.168.9.1

* Try pinging the controller IP to make sure it is under the same network segment.

* Setup the Python environment. For details, please refer to [Python Setup and Usage](https://docs.python.org/3/using/index.html).

  Python configuration: pip install numpy

* This API interface and Demo are applicable to V4 and above controller version of CR-V4 series

## Version and Release

### Current version V1.0.0.0

| Version| Date|
|:----------:|:----------:|
| V1.0.0.0| 2024-03-07|

# 2\. Technical support


# 3\. TCP-IP-CR-Python-V4 control protocol

As the communication based on TCP/IP has high reliability, strong practicability and high performance with low cost, many industrial automation projects have a wide demand for controlling robots based on TCP/IP protocol. Dobot robots, designed on the basis of TCP/IP protocol, provide rich interfaces for interaction with external devices.

Robot status:

    1. Error status is the first priority. If the robot reports an error and is not enabled, it returns the error status.
    
    2. Power-off status is the second priority. If the robot is not powered on and not enabled, it returns the power-off status.
    
    3. Collision status is the third priority. If the robot is in a collision and the script is paused, it returns the collision status.
    
    4. Brake-ON status is the fourth priority. If the brake of the robot is switched on but the robot is not enabled, it returns the brake-ON status.
    
    The other statuses are fed back according to the actual situation.

Port feedback:

    Port 29999 is responsible for receiving the commands related to settings and motion control by sending and receiving.
    
    Port 30004 (hereinafter referred to as the real-time feedback port) feeds back robot information every 8ms.
    
    Port 30005 feeds back robot information every 200ms.
    
    Port 30006 is a configurable port to feed back robot information (feed back every 50ms by default).
    
    Cancel port 30003.

For more details, see **Dobot TCP_IP Remote Control Interface Guide**.

# 4\. Obtaining TCP-IP-CR-Python-V4

1. Obtain the secondary development API program of Dobot TCP-IP-CR-Python-V4 from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git
   ```

2. **Note**: Refer to the corresponding README.md file for use.

# 5\. File and class descriptions

1. main.py: The running entrance of the main program.

2. dobot_api.py: encapsulate the robot interface. For details, refer to the TCP_IP Remote Control Interface Guide (https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4.git).

3. files: Files related to the alarm ID, `alarm_controller.json`: Warning alarm profile, `alarm_servo.json`: Servo alarm profile

4. PythonExample.py: Contains usage and code examples of some API interface commands

Class descriptions in the dobot_api directory:

| Class| Define|
|:----------|----------|
| DobotApi| Interface class based on TCP communication, encapsulates the basic business of communication|
| DobotApiDashboard| Inherited from DobotApi, it implements the specific basic functions of the robot|
| DobotApiFeedBack| Inherited from DobotApi, provides real-time feedback on robot status information.|
| MyType| Data type, it feeds back the robot status list|
| alarm_controller| Warning alarm configuration information|
| alarm_servo| Servo alarm configuration information|

**DobotApi**

Interface class based on TCP communication, encapsulates the basic business of communication.

```python
class DobotApi:
    def __init__(self, ip, port, *args):
    ""
    if self.port == 29999 or self.port == 30004:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
    ""        
```

**DobotApiDashboard**

Inherited from DobotApi, it can deliver the command related to control and motion to the robot. It implements the specific basic functions of the robot.

```c++
class DobotApiDashboard(DobotApi):
    """
    Define class dobot_api_dashboard to establish a connection to Dobot
    """
    def EnableRobot(self,*dynParams):
        """
        Enable the robot
        """
        self.send_data(string)
        return self.wait_reply()
      """
```

**DobotApiFeedBack**

Inherited from DobotApi, provides real-time feedback on robot status information.

```python
 class DobotApiFeedBack(DobotApi):
    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)
        self.__MyType = []
        self.__Lock = threading.Lock()
        feed_thread = threading.Thread(target=self.recvFeedData)  # Robot status feedback thread
        feed_thread.daemon = True
        feed_thread.start()
        """
    
```

**MyType**

Data type

```python
MyType=np.dtype([('len', np.int16, ), 
                ('Reserve', np.int16, (3,) ),
                ('digital_input_bits', np.int64, ), 
                ('digital_outputs', np.int64, ), 

                ('Reserve3', np.int8, (24,)),
                ])
                ""
```

**For details, see the Demo.**

# 7\. Common problem

* TCP connection:
  
  Port 29999 can be connected by one client at a time;
  
  Ports 30004 30005 30006 can be connected by multiple clients at the same time;
  
  Port 29999 has mode restriction. Before opening, you need to set the robot to TCP mode, otherwise it cannot respond and return “Control Mode Is Not Tcp” after the command is delivered. There is no mode restriction for real-time feedback ports 30004, 30005, 30006.

* To obtain the robot status, you can monitor the return value of RobotMode().

1. When the controller is in initializing status, it returns 1;

2. When the robot status is not 0 or 1, the robot initialization is completed;

3. When the robot is in power-off status, it returns 3;

4. When the robot is initialized but not enabled, it returns 4;

5. When the robot is enabled but idle, it returns 5, indicating that it can receive motion commands normally;

6. movj, script, and other queue commands are running status and return 7;

7. movJog is a single motion status, it return 8; no power-on status.

     | Mode | Description                   | Note                            |
     | ---- | ---------------------- | ------------------------------- |
     | 1    | ROBOT_MODE_INIT        | Initialized status                      |
     | 2    | ROBOT_MODE_BRAKE_OPEN  | Brake switched on                        |
     | 3    | ROBOT_MODE_POWEROFF    | Power-off status                    |
     | 4    | ROBOT_MODE_DISABLED  | Disabled (no brake switched on)                        |
     | 5    | ROBOT_MODE_ENABLE      | Enabled (idle)                      |
     | 6    | ROBOT_MODE_BACKDRIVE   | Drag                            |
     | 7    | ROBOT_MODE_RUNNING     | Running status (script, TCP queue) |
     | 8    | ROBOT_MODE_SINGLE_MOVE | Single motion status (jog)              |
     | 9    | ROBOT_MODE_ERROR       | Error status                        |
     | 10   | ROBOT_MODE_PAUSE       | Pause status                        |
     | 11   | ROBOT_MODE_COLLISION   | Collision status                        |

* Commands for different status
  
  ​    *Commands can be executed for error status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), the rest are rejected commands and return -2;
  
  ​    *Commands can be executed for emergency stop status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), the rest are rejected commands and return -3;
  
  ​    *Commands can be executed for power-off status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), PowerOn(), the rest are rejected commands and return -4;
  
  ​	**TCP commands are all immediate return commands except EnableRobot() and PowerOn(). The return of the command only represents successful delivering, not completion of execution.

* Coordinate system
  
  After entering TCP/IP mode, the system sets the user/tool coordinate system to 0 by default. It automatically restores the value of the user/tool coordinate system after exiting TCP/IP mode.
  
  ​      Global coordinates: the coordinates set by User() and Tool() commands are the global coordinates, which takes effect for all commands after setting;
  
  ​      Local coordinates: the optional parameters (user/tool) in the motion command are only effective in the current motion command, and will be restored to the global coordinates after the execution of the current command.

* The algorithm allows a queue depth of 64, 64 queued commands can be processed continuously at the same time. If the queued commands sent to the port exceed the queue depth, the port will not be disconnected, and the command interface will return -1 and the command will not be executed.

* The queue command is an immediate return command. The successful return of the interface only means that it was delivered successfully, it does not mean that the execution is completed. If it is judged that the execution is completed, it needs to be combined with CommandID and RobotMode to make a comprehensive judgment.

* The servo run time represents the accumulated time after the robot is enabled, and the controller power-on time represents the time after the controller is powered on, so the controller power-on time is always greater than the servo run time.

* The collision status is not an error, there is no collision error returned in GetErrorID(). You can query the collision status by RobotMode(), which returns 11. You can clear the collision status by ClearError().

* After a collision, you can deliver an error clearing command to clear the collision status, and the robot can be run again without re-enabling.

* After switching to TCP/IP mode, the speed/acceleration/user/tool coordinate system parameters will be set to the default values, so there is no need to call the EnableRobot() command to set the default parameters.

* The return value of the TCP queue command only indicates whether the parameters and syntax of the command conform to the specification, it does not indicate whether the command is successfully executed or not. The TCP queue command returns when it is delivered, and the return of 0 only means that it is delivered successfully, not that it can be executed successfully.

* If an error occurs in parameter type/number of the TCP command , the system will alarm directly and will not give a 0 to handle it, and the command will not be delivered to the algorithm.

* After triggering the emergency stop signal, the robot stops running by default. If it does not stop after 500ms, it will be powered off. If it stops, it will not be powered off. In general, the robot will not be powered off after an emergency stop.

* After setting the default gateway, the default gateway data will be saved and the value will not be changed after a restart.

* The Enable command only executes the enable action without other parameter settings. It will not clear the kinematic parameters and coordinate system parameters.

* The Pause() and Continue() commands are effective for script, and the motion commands (queue-related) are also effective. The robot enters the pause status and the algorithmic queue pauses after calling the Pause() command. You can continue to run the queue commands using the Continue() command. The MovJog command cannot be paused or continued.

* The current TCP does not support the insertion of extraneous characters between commands. You can write in the following formats 
  
  ① MovJ()MovJ()MovJ()
  
  ② MovJ() MovJ() MovJ()

# 8\. Example

* Dobot-Demo realizes TCP control of the robot and other interactions. It connects to the control port and feedback port of the robot respectively. It delivers motion commands to robot, and handles the abnormal status of the robot, etc.

1. Main thread: Connect to the control port, motion port, and feedback port of the robot respectively. Enable the robot.

![](/main_en.png)

2. Feedback status thread: Real-time feedback of robot status information.

![](/feed_en.png)

3. Robot motion thread: Deliver motion commands to robot

![](/move_en.png)

Arrival signal

The queue command is an immediate return command. The successful return of the interface only means that it was delivered successfully, it does not mean that the execution is completed. If it is judged that the execution is completed, it needs to be combined with

* If the current CommandID is greater than the CommandID that delivered, it means the delivered queue command is completed.

* If the current CommandID is equal to the CommandID that delivered, and RobotMode command returns a value of 5, it means the delivered queue command is completed.

4. Exception handling thread: Judge and handle the abnormal status of the robot

![](/exception_en.png)

**Steps to run the Demo:**

1. Obtain the secondary development API program of Dobot TCP-IP-CR-Python-V4 from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git
   ```

2. Connect to the robot via LAN1 interface, and set the local IP address to 192.168.5.X
  
   Control Panel >> Network and Internet >> Network Connections
   
   ![](/netConnect_en.png)
   
   Select the connected Ethernet >> Right click >> Properties >> Internet Protocol Version (TCP/IPV4)
   
   Modify the IP address to 192.168.5.X
   
   ![](/updateIP_en.png)

3. Open the DobotStudio Pro, connect to the robot, and set the robot mode to **TCP**.
  
   ![](/checkTcpMode_en.png)

4. To run the program, you need to detect the dynamic library, open the whole directory in VsCode/PyCharm and run main.py directly.
  
   ![](/runpy.png)

#### DobotDemo

TCP connection: choose the port to connect to the robot arm independently and establish a connection with the robot arm.

```python
class DobotDemo:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999   
        self.feedPortFour = 30004
      #  self.feedPortFour = 30005
      #  self.feedPortFour = 30006
        self.dashboard = None
        ... ...
```

Objects: DobotApiDashboard, DobotApiFeedBack

```python
    def start(self):
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)
        enableState = self.parseResultId(self.dashboard.EnableRobot())
        if enableState[0] != 0:
            print("Failed to enable: Check if port 29999 is occupied)")
            return
        print("Enable successfully:)")
        ...  ...
```

Control robot movement: Deliver motion commands to control the robot to move

```python
    point_a = [-90, 20, 0, 0, 0, 0]
    point_b = [90, 20, 0, 0, 0, 0]

    while True:
        while True:
            p2Id = self.RunPoint(point_a)
            if p2Id[0] == 0:  # Motion commands return the correct values
                self.WaitArrive(p2Id[1])  # Set the commandID and wait for the command to be completed.
                break
            else:
                sleep(5) # If the return value of the motion command is wrong (e.g., non-tcp mode), continue running after sleeping for 5s.

   
# Encapsulate the motion function of the robot arm, deliver motion commands, and return the running results.
 def RunPoint(self, point_list: list):
        recvmovemess = self.dashboard.MovJ(
            point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], 1)
        print("Movj", recvmovemess)
        commandArrID = self.parseResultId(recvmovemess)  # Parse the return value of the Movj command
        return commandArrID
    
    ... ...  ...  ...               
                    
```

A sign that the command is completed: Wait for the robot to complete the motion command, which is similar to the Sync.

```python
  # Judge by CommandID and robotMode
    def WaitArrive(self, p2Id):
        while True:
            while not self.__robotSyncBreak.is_set():
                self.__globalLockValue.acquire()  #
                if self.feedData.robotEnableStatus:
                    if self.feedData.robotCurrentCommandID > p2Id:
                        self.__globalLockValue.release()
                        break
                    else:
                        isFinsh = (self.feedData.robotMode == 5)
                        if self.feedData.robotCurrentCommandID == p2Id and isFinsh:
                            self.__globalLockValue.release()
                            break
                self.__globalLockValue.release()
                sleep(0.01)
            self.__robotSyncBreak.clear()
            break
```

Parse the returned function: After delivering the command, parse the result of the robot arm.

```python
  # Parse the result and return the list 
 def parseResultId(self, valueRecv):
        if valueRecv.find("Not Tcp") != -1:  # Judge whether the robot is in TCP mode by the return value
            print("Control Mode Is Not Tcp")
            return [1]
        recvData = re.findall(r'-?\d+', valueRecv)
        recvData = [int(num) for num in recvData]
        # Return all numeric arrays that returned by the TCP command.
        if len(recvData) == 0:
            return [1]
        return recvData
    
 # Judge the robot status by judging the results (see table below)
  parseResultId: parse all numbers
    
 #  0,{1846},MovJ(joint={90,20,0,0,00,00});            [0,1846,90,20,0,0,0,0]
 #  0,{},clearerror()                                  [0]
 #  0,{[[30,132],[],[],[],[],[],[]]},geterrorid();     [0,30,132]  
 #  Control Mode Is Not Tcp                            [1]  
 #  -1,{},MovJ(joint={90,20,0,0,00,00});               [-1,90,20，...]  
 #  -2,{},MovJ(joint={90,20,0,0,0,0});                 [-2,90,20，...]  
 # -30001,{},MovJ(joi={90,20,0,0,0,0});                [-30001,90,20，...]  
```

###### Parse the result and return the list

| Result  | list[0]                                                     | list[1] | list[...] |
| :------ | ----------------------------------------------------------- | :------ | :-------- |
| 0       | Command delivered successfully                              | -       | -         |
| 1       | Robot is not in TCP mode                                    | -       | -         |
| 2       | Other program exceptions                                    | -       | -         |
| -1      | Failed to execute                                           | -       | -         |
| -2      | Currently in error status, cannot execute commands          | -       | -         |
| -3      | Currently in emergency stop status, cannot execute commands | -       | -         |
| -4      | Currently in power-off status, cannot execute commands      | -       | -         |
| -10000  | Command error                                               | -       | -         |
| -20000  | Parameter number error                                      | -       | -         |
| -30001  | The type of the first parameter is incorrect                | -       | -         |
| -30002  | The type of the second parameter is incorrect               | -       | -         |
| ... ... |                                                             |         |           |
| -40001  | The range of the first parameter is incorrect               | -       | -         |
| ... ... |                                                             |         |           |
| -50001  | The type of the first optional parameter is incorrect       | -       | -         |
| ... ... |                                                             |         |           |
| -60001  | The range of the first optional parameter is incorrect      | -       | -         |
| ... ... |                                                             |         |           |

The list[1] and list[n] values are the results of special commands.

```python
 # Judge the robot status by judging the results
 #  0,{1846},MovJ(joint={90,20,0,0,00,00});            list[1]为commandArrID 1846
 #  0,{[[30,132],[],[],[],[],[],[]]},geterrorid();     The list[1] and list[2] are robot alarm IDs.
```

Obtain information of the robot: Obtain robot status from the feedback port

```python
 ...  ...   
    def GetFeed(self):
        while True:
            feedInfo = self.feedFour.feedBackData()
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                self.__globalLockValue.acquire()  
                
                # Add the feedback data from the robot arm, see **MyType** for details.
                # .......................................................
                self.feedData.robotErrorState = feedInfo['error_status'][0]
                self.feedData.robotEnableStatus = feedInfo['enable_status'][0]
                self.feedData.robotMode = feedInfo['robot_mode'][0]
                self.feedData.robotCurrentCommandID = feedInfo['currentcommandid'][0]
                # .......................................................
                
                self.__globalLockValue.release()
            sleep(0.004)
  ...  ...  

     # Robot status feedback thread
         feed_thread = threading.Thread(
                target=self.GetFeed)  
         feed_thread.daemon = True
         feed_thread.start()
```

Monitor the robot status: Monitor the abnormal status of the robot and error-clearing function

```python
 def ClearRobotError(self):
        # Read controller and servo alarm IDs
        dataController, dataServo = alarmAlarmJsonFile()    
        while True:
            self.__globalLockValue.acquire()  # robotErrorState lock
            if self.feedData.robotErrorState:
                geterrorID = self.parseResultId(self.dashboard.GetErrorID())
                if geterrorID[0] == 0:
                    for i in range(1, len(geterrorID)):
                        alarmState = False
                        
                        # Read controller alarm IDs
                        for item in dataController:
                            if geterrorID[i] == item["id"]:
                                print("Alarm Controller GetErrorID",
                                      i, item["EN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue
                      
                       # Read servo alarm code
                        for item in dataServo:
                            if geterrorID[i] == item["id"]:
                                print("Alarm Servo GetErrorID", i,
                                      item["EN"]["description"])
                        
                         # Clear the error on the robot arm or not
                        choose = input("1, clear the error and the robot keep running: ")
                        ...  ...
```

**Common problem**

**Problem 1: ModuleNotFoundError :  Nomode name 'numpy'**

**Solution:** If the numpy environment is not installed, install numpy      pip install numpy.



**Problem 2: Connection refused, IP:Port has been occupied**

**Solution:** Check if port 29999 is occupied.

**Problem 3: The Demo robot outputs the following status**

| Abnormal robot output| Robot status|
|----------|----------|
| **Command execution failed**| **Failed to execute**|
| **The robot is in an error state**| **Error status**|
| **The robot is in emergency stop status**| **Emergency stop status**|
| **The robot is in power-off status**| **Power-off status**|
| **Control Mode Is Not TCP**| **Robot is not in TCP/IP mode**|

**Make sure the robot is in a safe position before running the Demo to prevent unnecessary collisions.**
