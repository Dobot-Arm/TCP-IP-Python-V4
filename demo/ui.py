"""
Robot control UI interface
# 机器人控制UI界面

Tkinter-based graphical robot control interface
# 基于Tkinter的图形化机器人控制界面
"""

# -*- coding: utf-8 -*-
import sys
import os
import json

# Add parent directory to path for importing dobot_sdk / 添加父目录到路径，以便导入dobot_sdk
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from threading import Thread
import time
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
from dobot_sdk.api.error_controller import (
    get_error,
    format_error_messages_from_http,
    SUPPORTED_LANGUAGES
)

LABEL_JOINT = [["J1-", "J2-", "J3-", "J4-", "J5-", "J6-"],
               ["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"],
               ["J1+", "J2+", "J3+", "J4+", "J5+", "J6+"]]

LABEL_COORD = [["X-", "Y-", "Z-", "Rx-", "Ry-", "Rz-"],
               ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"],
               ["X+", "Y+", "Z+", "Rx+", "Ry+", "Rz+"]]

LABEL_ROBOT_MODE = {
    0: "ROBOT_MODE_UNKNOWN",
    1: "ROBOT_MODE_INIT",
    2: "ROBOT_MODE_BRAKE_OPEN",
    3: "ROBOT_MODE_POWEROFF",
    4: "ROBOT_MODE_DISABLED",
    5: "ROBOT_MODE_ENABLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
    8: "ROBOT_MODE_SINGLE_MOVE",
    9: "ROBOT_MODE_ERROR",
    10: "ROBOT_MODE_PAUSE",
    11: "ROBOT_MODE_JOG"
}


class RobotUI(object):

    def __init__(self, robot_ip="120.79.211.106", di_count=24, do_count=24):
        self.root = Tk()
        self.root.title("Dobot SDK Python Demo V4")
        self.root.geometry("900x900")
        self.robot_ip = robot_ip
        self.di_count = di_count
        self.do_count = do_count
        # Current language setting / 当前语言设置
        self.current_lang = 'zh_cn'

        # Error message cache to avoid repeated refresh / 错误信息缓存，避免重复刷新
        self.last_error_ids = None
        self.last_error_message = None

        # global state dict / 全局状态字典
        self.global_state = {}

        # all buttons / 所有按钮
        self.button_list = []

        # All input fields / 所有输入框
        self.entry_dict = {}

        # Robot Connect / 机器人连接
        self.frame_robot = LabelFrame(self.root, text="Robot Connect",
                                      labelanchor="nw", bg="#FFFFFF", width=870, height=100, border=2)

        # First row controls (vertically centered) / 第一行控件（垂直居中）
        row_y = 0.35

        # IP Address (left) / IP Address (左侧)
        self.label_ip = Label(self.frame_robot, text="IP Address:")
        self.label_ip.place(rely=row_y, x=15, anchor=W)
        ip_port = StringVar(self.root, value="120.79.211.106")
        self.entry_ip = Entry(self.frame_robot, width=16)
        self.entry_ip.place(rely=row_y, x=95, anchor=W)
        self.entry_ip.insert(0, self.robot_ip)

        # Dashboard Port (left-center) / Dashboard Port (左中)
        self.label_dash = Label(self.frame_robot, text="Dashboard:")
        self.label_dash.place(rely=row_y, x=220, anchor=W)
        dash_port = IntVar(self.root, value=29999)
        self.entry_dash = Entry(self.frame_robot, width=6, textvariable=dash_port)
        self.entry_dash.place(rely=row_y, x=300, anchor=W)

        # Feedback Port (center) / Feedback Port (中间)
        self.label_fb = Label(self.frame_robot, text="Feedback:")
        self.label_fb.place(rely=row_y, x=380, anchor=W)
        self.combo_feedback = ttk.Combobox(self.frame_robot, width=12)
        self.combo_feedback["value"] = ["30004 (8ms)", "30005 (200ms)", "30006 (1000ms)"]
        self.combo_feedback.current(0)
        self.combo_feedback["state"] = "readonly"
        self.combo_feedback.place(rely=row_y, x=455, anchor=W)

        # Language selection (right-center) / 语言选择 (右中)
        self.label_lang = Label(self.frame_robot, text="Language:")  # 语言 / Language
        self.label_lang.place(rely=row_y, x=590, anchor=W)
        self.combo_lang = ttk.Combobox(self.frame_robot, width=12)
        self.combo_lang["value"] = [lang[0] for lang in SUPPORTED_LANGUAGES]
        self.combo_lang.current(0)
        self.combo_lang["state"] = "readonly"
        self.combo_lang.place(rely=row_y, x=640, anchor=W)
        self.combo_lang.bind("<<ComboboxSelected>>", self.on_lang_change)

        # Connect/Disconnect (right) / Connect/Disconnect (右侧)
        self.button_connect = self.set_button(master=self.frame_robot,
                                              text="Connect", rely=row_y, x=780, anchor=W, command=self.connect_port)
        self.button_connect["width"] = 8
        self.global_state["connect"] = False

        # Dashboard Function - controls center-aligned / Dashboard Function - 控件居中对齐
        self.frame_dashboard = LabelFrame(self.root, text="Dashboard Function",
                                          labelanchor="nw", bg="#FFFFFF", pady=10, width=870, height=80, border=2)

        # Vertically centered / 垂直居中
        row_y = 0.5

        # Enable/Disable
        self.button_enable = self.set_button(master=self.frame_dashboard,
                                             text="Enable", rely=row_y, x=150, anchor=CENTER, command=self.enable)
        self.button_enable["width"] = 7
        self.global_state["enable"] = False

        self.set_button(master=self.frame_dashboard,
                        text="ClearError", rely=row_y, x=280, anchor=CENTER, command=self.clear_error)

        # Speed Ratio
        self.label_speed = Label(self.frame_dashboard, text="Speed Ratio:")
        self.label_speed.place(rely=row_y, x=410, anchor=CENTER)

        s_value = StringVar(self.root, value="50")
        self.entry_speed = Entry(self.frame_dashboard,
                                 width=6, textvariable=s_value)
        self.entry_speed.place(rely=row_y, x=500, anchor=CENTER)
        self.label_cent = Label(self.frame_dashboard, text="%")
        self.label_cent.place(rely=row_y, x=535, anchor=CENTER)

        self.set_button(master=self.frame_dashboard,
                        text="Confirm", rely=row_y, x=620, anchor=CENTER, command=self.confirm_speed)

        # Move Function
        self.frame_move = LabelFrame(self.root, text="Move Function", labelanchor="nw",
                                     bg="#FFFFFF", width=870, pady=10, height=130, border=2)

        self.set_move(text="X:", label_value=10,
                      default_value="600", entry_value=40, rely=0.1, master=self.frame_move)
        self.set_move(text="Y:", label_value=110,
                      default_value="-260", entry_value=140, rely=0.1, master=self.frame_move)
        self.set_move(text="Z:", label_value=210,
                      default_value="380", entry_value=240, rely=0.1, master=self.frame_move)
        self.set_move(text="Rx:", label_value=310,
                      default_value="170", entry_value=340, rely=0.1, master=self.frame_move)
        self.set_move(text="Ry:", label_value=410,
                      default_value="12", entry_value=440, rely=0.1, master=self.frame_move)
        self.set_move(text="Rz:", label_value=510,
                      default_value="140", entry_value=540, rely=0.1, master=self.frame_move)

        self.set_button(master=self.frame_move, text="MovJ",
                        rely=0.05, x=610, command=self.movj)
        self.set_button(master=self.frame_move, text="MovL",
                        rely=0.05, x=700, command=self.movl)

        self.set_move(text="J1:", label_value=10,
                      default_value="0", entry_value=40, rely=0.5, master=self.frame_move)
        self.set_move(text="J2:", label_value=110,
                      default_value="-20", entry_value=140, rely=0.5, master=self.frame_move)
        self.set_move(text="J3:", label_value=210,
                      default_value="-80", entry_value=240, rely=0.5, master=self.frame_move)
        self.set_move(text="J4:", label_value=310,
                      default_value="30", entry_value=340, rely=0.5, master=self.frame_move)
        self.set_move(text="J5:", label_value=410,
                      default_value="90", entry_value=440, rely=0.5, master=self.frame_move)
        self.set_move(text="J6:", label_value=510,
                      default_value="120", entry_value=540, rely=0.5, master=self.frame_move)

        self.set_button(master=self.frame_move,
                        text="MovJ", rely=0.45, x=610, command=self.joint_movj)

        # Digital IO - three-row layout, evenly spaced / Digital IO - 三行布局，三行均分空间
        self.frame_io = LabelFrame(self.root, text="Digital IO",
                                   labelanchor="nw", bg="#FFFFFF", width=870, height=150, border=2)

        # First row: Digital Outputs setting area (three rows evenly spaced) / 第一行：Digital Outputs设置区域（三行均分，间隔相同）
        row1_y = 1/6  # 约0.1667 / approx 0.1667

        self.label_do = Label(self.frame_io, text="Digital Outputs:")
        self.label_do.place(rely=row1_y, x=15, anchor=W)

        self.label_do_index = Label(self.frame_io, text="Index:")
        self.label_do_index.place(rely=row1_y, x=130, anchor=W)
        i_value = IntVar(self.root, value="1")
        self.entry_index = Entry(self.frame_io, width=5, textvariable=i_value)
        self.entry_index.place(rely=row1_y, x=180, anchor=W)

        self.label_do_status = Label(self.frame_io, text="Status:")
        self.label_do_status.place(rely=row1_y, x=230, anchor=W)
        self.combo_status = ttk.Combobox(self.frame_io, width=5)
        self.combo_status["value"] = ("On", "Off")
        self.combo_status.current(0)
        self.combo_status["state"] = "readonly"
        self.combo_status.place(rely=row1_y, x=285, anchor=W)

        self.button_confirm_do = self.set_button(master=self.frame_io,
                                                 text="Confirm", rely=row1_y, x=350, anchor=W, command=self.confirm_do)

        # Second row: Digital Inputs display area (24 circular indicators in a row, aligned with DO) / 第二行：Digital Inputs显示区域（24个圆形指示灯排成一排，与DO对齐）
        row2_y = 3/6  # 0.5，中间位置 / 0.5, center position

        self.label_di = Label(self.frame_io, text="Digital Inputs:")
        self.label_di.place(rely=row2_y, x=15, anchor=W)
        self.di_indicators = []
        # Display the DI indicators in a row, with the first indicator aligned with the Index label. / 排成一排显示，起始位置对齐
        for i in range(self.di_count):
            canvas = Canvas(self.frame_io, width=18, height=18, bg="white", highlightthickness=1)
            canvas.place(rely=row2_y, x=130 + i*20, anchor=CENTER)
            canvas.create_oval(2, 2, 16, 16, fill="gray", outline="black")
            canvas.create_text(9, 9, text=str(i+1), fill="white", font=("Arial", 8))
            self.di_indicators.append(canvas)

        # Third row: Digital Outputs display area (24 circular indicators in a row, aligned with DI) / 第三行：Digital Outputs显示区域（24个圆形指示灯排成一排，与DI对齐）
        row3_y = 5/6  # 约0.8333，底部位置 / approx 0.8333, bottom position

        self.label_do_display = Label(self.frame_io, text="Digital Outputs:")
        self.label_do_display.place(rely=row3_y, x=15, anchor=W)
        self.do_indicators = []
        # Display in a row, aligned with DI starting position / 排成一排显示，起始位置与DI对齐
        for i in range(self.do_count):
            canvas = Canvas(self.frame_io, width=18, height=18, bg="white", highlightthickness=1)
            canvas.place(rely=row3_y, x=130 + i*20, anchor=CENTER)
            canvas.create_oval(2, 2, 16, 16, fill="gray", outline="black")
            canvas.create_text(9, 9, text=str(i+1), fill="white", font=("Arial", 8))
            self.do_indicators.append(canvas)

        self.frame_feed_log = Frame(
            self.root, bg="#FFFFFF", width=870, pady=10, height=430, border=2)

        # Feedback - left area / Feedback - 左侧区域
        self.frame_feed = LabelFrame(self.frame_feed_log, text="Feedback", labelanchor="nw",
                                     bg="#FFFFFF")
        self.frame_feed.place(relx=0, rely=0, relheight=1, relwidth=0.6)

        # Current Speed Ratio and Robot Mode on the same row / Current Speed Ratio和Robot Mode放在同一行
        self.set_label(self.frame_feed,
                       text="Current Speed Ratio:", rely=0.02, x=10)
        self.label_feed_speed = self.set_label(
            self.frame_feed, "", rely=0.02, x=145)
        self.set_label(self.frame_feed, text="%", rely=0.02, x=175)

        self.set_label(self.frame_feed, text="Robot Mode:", rely=0.02, x=220)
        self.label_robot_mode = self.set_label(
            self.frame_feed, "", rely=0.02, x=310)

        # Jog and get coordinates - Joint and Coord share space evenly / 点动及获取坐标 - Joint和Coord作为整体均分空间
        self.label_feed_dict = {}

        # Joint area (left) and Coord area (right) each take half the space / Joint区域（左侧）和Coord区域（右侧）各占一半空间
        # Joint area / Joint区域
        self.set_label(self.frame_feed, text="Joint:", rely=0.10, x=95)
        for i in range(6):
            row_y = 0.2 + i * 0.12
            self.set_button_bind(self.frame_feed, LABEL_JOINT[0][i], rely=row_y, x=10)
            self.set_label(self.frame_feed, LABEL_JOINT[1][i], rely=row_y + 0.01, x=55)
            self.label_feed_dict[LABEL_JOINT[1][i]] = self.set_label(self.frame_feed, " ", rely=row_y + 0.01, x=95)
            self.set_button_bind(self.frame_feed, LABEL_JOINT[2][i], rely=row_y, x=170)

        # Coord area (right, shares space evenly with Joint area) / Coord区域（右侧，与Joint区域均分）
        self.set_label(self.frame_feed, text="Coord:", rely=0.10, x=335)
        self.combo_coord_type = ttk.Combobox(self.frame_feed, width=10)
        self.combo_coord_type["value"] = ["User", "Tool"]  # 用户坐标系/User, 工具坐标系/Tool
        self.combo_coord_type.current(0)
        self.combo_coord_type["state"] = "readonly"
        self.combo_coord_type.place(rely=0.10, x=390, anchor=W)
        for i in range(6):
            row_y = 0.2 + i * 0.12
            self.set_button_bind(self.frame_feed, LABEL_COORD[0][i], rely=row_y, x=250)
            self.set_label(self.frame_feed, LABEL_COORD[1][i], rely=row_y + 0.01, x=295)
            self.label_feed_dict[LABEL_COORD[1][i]] = self.set_label(self.frame_feed, " ", rely=row_y + 0.01, x=335)
            self.set_button_bind(self.frame_feed, LABEL_COORD[2][i], rely=row_y, x=400)

        # Error Info - upper right / Error Info - 右侧上方
        self.frame_err = LabelFrame(self.frame_feed_log, text="Error Info", labelanchor="nw",
                                    bg="#FFFFFF")
        self.frame_err.place(relx=0.62, rely=0, relheight=0.48, relwidth=0.36)

        self.text_err = ScrolledText(
            self.frame_err, width=170, height=50, relief="flat")
        self.text_err.place(rely=0, relx=0, relheight=0.85, relwidth=1)

        self.set_button(self.frame_err, "Clear", rely=0.88,
                        x=100, command=self.clear_error_info)

        # Log - lower right, similar size to Error Info / Log - 右侧下方，与Error Info大小差不多
        self.frame_log = LabelFrame(self.frame_feed_log, text="Log", labelanchor="nw",
                                    bg="#FFFFFF")
        self.frame_log.place(relx=0.62, rely=0.52, relheight=0.46, relwidth=0.36)

        self.text_log = ScrolledText(
            self.frame_log, width=270, height=140, relief="flat")
        self.text_log.place(rely=0, relx=0, relheight=1, relwidth=1)

        # Initialize robot client / 初始化机器人客户端
        self.robot = None

        # Store current robot mode / 存储当前机器人模式
        self.current_robot_mode = 0

    def on_lang_change(self, event):
        """Language switch handler / 语言切换处理"""
        idx = self.combo_lang.current()
        self.current_lang = SUPPORTED_LANGUAGES[idx][1]
        self.add_log(f"Language switched to: {SUPPORTED_LANGUAGES[idx][0]}")  # 语言已切换为 / Language switched to
        # If robot is already connected, sync robot language setting / 如果已连接机器人，同步设置机器人语言
        if self.robot and self.global_state["connect"]:
            self.robot.error.set_language(self.current_lang)

    def add_log(self, message):
        """Add log message / 添加日志消息"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.text_log.insert(END, f"[{timestamp}] {message}\n")
        self.text_log.see(END)

    def mainloop(self):
        self.root.mainloop()

    def pack(self):
        self.frame_robot.pack()
        self.frame_dashboard.pack()
        self.frame_move.pack()
        self.frame_io.pack()
        self.frame_feed_log.pack()

    def set_move(self, text, label_value, default_value, entry_value, rely, master):
        self.label = Label(master, text=text)
        self.label.place(rely=rely, x=label_value)
        value = StringVar(self.root, value=default_value)
        self.entry_temp = Entry(master, width=6, textvariable=value)
        self.entry_temp.place(rely=rely, x=entry_value)
        self.entry_dict[text] = self.entry_temp

    def move_jog(self, text):
        """Jog control / 点动控制"""
        if self.global_state["connect"] and self.robot:
            if self.current_robot_mode == 9:
                self.add_log("Robot is in error state, cannot perform jog operation")  # 机器人处于错误状态，无法执行点动操作 / Robot in error state, cannot jog
                return
            if text and text[0] in ("J", "j"):
                coord_type = CoordinateType.JOINT
            else:
                idx = self.combo_coord_type.current()
                if idx == 1:
                    coord_type = CoordinateType.TOOL
                else:
                    coord_type = CoordinateType.CARTESIAN
            self.robot.motion.MoveJog(text, coord_type=coord_type)

    def move_stop(self, event):
        """Stop jog / 停止点动"""
        if self.global_state["connect"] and self.robot:
            self.robot.motion.MoveJog("")

    def set_button(self, master, text, rely, x, **kargs):
        self.button = Button(master, text=text, padx=5,
                             command=kargs["command"])
        anchor = kargs.get("anchor", NW)
        self.button.place(rely=rely, x=x, anchor=anchor)

        if text != "Connect":
            self.button["state"] = "disable"
            self.button_list.append(self.button)
        return self.button

    def set_button_bind(self, master, text, rely, x, **kargs):
        self.button = Button(master, text=text, padx=5)
        self.button.bind("<ButtonPress-1>",
                         lambda event: self.move_jog(text=text))
        self.button.bind("<ButtonRelease-1>", self.move_stop)
        self.button.place(rely=rely, x=x)

        if text != "Connect":
            self.button["state"] = "disable"
            self.button_list.append(self.button)
        return self.button

    def set_label(self, master, text, rely, x):
        self.label = Label(master, text=text)
        self.label.place(rely=rely, x=x)
        return self.label

    def connect_port(self):
        """Connect/disconnect robot / 连接/断开机器人"""
        if self.global_state["connect"]:
            # Disconnect / 断开连接
            try:
                if self.robot:
                    self.robot.Disconnect()
                    self.robot = None
                self.add_log("Disconnected successfully")  # 断开成功 / Disconnected successfully
            except Exception as e:
                self.add_log(f"Disconnect failed: {e}")  # 断开失败 / Disconnect failed

            for i in self.button_list:
                i["state"] = "disable"
            self.button_connect["text"] = "Connect"
        else:
            # Establish connection / 建立连接
            try:
                ip = self.entry_ip.get()
                dash_port = int(self.entry_dash.get())

                # Get Feedback port from dropdown / 从下拉框获取Feedback端口
                feedback_selection = self.combo_feedback.get()
                feedback_port = int(feedback_selection.split()[0])

                self.add_log(f"Connecting to robot: {ip}")  # 正在连接机器人 / Connecting to robot
                self.add_log(f"  Dashboard: {dash_port}, Feedback: {feedback_port}")  # 端口信息 / Port info

                self.robot = DobotRobot(
                    ip,
                    dashboard_port=dash_port,
                    feedback_port=feedback_port
                )
                self.robot.Connect()

                # Set robot language / 设置机器人语言
                self.robot.error.set_language(self.current_lang)

                # Request TCP control mode / 请求TCP控制模式
                self.robot.robot_control.RequestControl()

                # Start status feedback monitor / 启动状态反馈监控
                self.robot.StartFeedbackMonitor()

                self.add_log("Connected successfully")  # 连接成功 / Connected successfully
            except Exception as e:
                messagebox.showerror("Attention!", f"Connection Error:{e}")
                self.add_log(f"Connection failed: {e}")  # 连接失败 / Connection failed
                return

            for i in self.button_list:
                i["state"] = "normal"
            self.button_connect["text"] = "Disconnect"

        self.global_state["connect"] = not self.global_state["connect"]
        self.set_feed_back()

    def set_feed_back(self):
        """Start status feedback thread / 启动状态反馈线程"""
        if self.global_state["connect"]:
            thread = Thread(target=self.feed_back)
            thread.start()

    def enable(self):
        """Enable/disable robot / 使能/下使能机器人"""
        if not self.robot:
            return

        # Check robot mode, operation forbidden in error mode / 检查机器人模式，错误模式下禁止操作
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, please clear errors first")  # 机器人处于错误状态，请先清除错误 / Robot in error state, clear errors first
            return

        if self.global_state["enable"]:
            try:
                response = self.robot.robot_control.DisableRobot()
                # Check whether response contains failure info / 检查响应是否包含失败信息
                if response and ("Failed" in str(response) or "Error" in str(response)):
                    self.add_log(f"Disable failed: {response}")  # 下使能失败 / Disable failed
                    return
                self.button_enable["text"] = "Enable"
                self.add_log("Robot disabled")  # 机器人已下使能 / Robot disabled
                self.global_state["enable"] = False
            except Exception as e:
                self.add_log(f"Disable failed: {e}")  # 下使能失败 / Disable failed
        else:
            try:
                response = self.robot.robot_control.EnableRobot()
                # Check whether response contains failure info / 检查响应是否包含失败信息
                if response and ("Failed" in str(response) or "Error" in str(response)):
                    self.add_log(f"Enable failed: {response}")  # 使能失败 / Enable failed
                    return
                self.button_enable["text"] = "Disable"
                self.add_log("Robot enabled")  # 机器人已使能 / Robot enabled
                self.global_state["enable"] = True
            except Exception as e:
                self.add_log(f"Enable failed: {e}")  # 使能失败 / Enable failed

    def clear_error(self):
        """Clear errors / 清除错误"""
        if self.robot:
            try:
                response = self.robot.robot_control.ClearError()
                if response and not str(response).startswith("0,"):
                    self.add_log(f"Clear error failed: {response}")  # 清除错误失败 / Clear error failed
                    return
                self.add_log("Errors cleared")  # 已清除错误 / Errors cleared
                self.clear_error_info()
            except Exception as e:
                self.add_log(f"Clear error failed: {e}")  # 清除错误失败 / Clear error failed

    def confirm_speed(self):
        """Set speed ratio / 设置速度比例"""
        if not self.robot:
            return

        # Check robot mode / 检查机器人模式
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, cannot set speed")  # 机器人处于错误状态，无法设置速度 / Robot in error state, cannot set speed
            return

        try:
            speed = int(self.entry_speed.get())
            response = self.robot.robot_control.SpeedFactor(speed)
            if response and ("Failed" in str(response) or "Error" in str(response)):
                self.add_log(f"Set speed failed: {response}")  # 设置速度失败 / Set speed failed
                return
            self.add_log(f"Speed ratio set to: {speed}%")  # 速度比例设置为 / Speed ratio set to
        except Exception as e:
            self.add_log(f"Set speed failed: {e}")  # 设置速度失败 / Set speed failed

    def movj(self):
        """Cartesian MovJ motion / 笛卡尔坐标MovJ运动"""
        if not self.robot:
            return

        # Check robot mode / 检查机器人模式
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, cannot execute motion")  # 机器人处于错误状态，无法执行运动 / Robot in error state, cannot execute motion
            return

        if not self.global_state["enable"]:
            self.add_log("Robot not enabled, cannot execute motion")  # 机器人未使能，无法执行运动 / Robot not enabled, cannot execute motion
            return

        try:
            x = float(self.entry_dict["X:"].get())
            y = float(self.entry_dict["Y:"].get())
            z = float(self.entry_dict["Z:"].get())
            rx = float(self.entry_dict["Rx:"].get())
            ry = float(self.entry_dict["Ry:"].get())
            rz = float(self.entry_dict["Rz:"].get())

            response = self.robot.motion.MovJ([x, y, z, rx, ry, rz], CoordinateType.CARTESIAN)
            if response and ("Failed" in str(response) or "Error" in str(response)):
                self.add_log(f"MovJ failed: {response}")  # MovJ失败 / MovJ failed
                return
            self.add_log(f"MovJ: ({x}, {y}, {z}, {rx}, {ry}, {rz})")  # MovJ坐标 / MovJ coordinates
        except Exception as e:
            self.add_log(f"MovJ failed: {e}")  # MovJ失败 / MovJ failed

    def movl(self):
        """Cartesian MovL motion / 笛卡尔坐标MovL运动"""
        if not self.robot:
            return

        # Check robot mode / 检查机器人模式
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, cannot execute motion")  # 机器人处于错误状态，无法执行运动 / Robot in error state, cannot execute motion
            return

        if not self.global_state["enable"]:
            self.add_log("Robot not enabled, cannot execute motion")  # 机器人未使能，无法执行运动 / Robot not enabled, cannot execute motion
            return

        try:
            x = float(self.entry_dict["X:"].get())
            y = float(self.entry_dict["Y:"].get())
            z = float(self.entry_dict["Z:"].get())
            rx = float(self.entry_dict["Rx:"].get())
            ry = float(self.entry_dict["Ry:"].get())
            rz = float(self.entry_dict["Rz:"].get())

            response = self.robot.motion.MovL([x, y, z, rx, ry, rz], CoordinateType.CARTESIAN)
            if response and ("Failed" in str(response) or "Error" in str(response)):
                self.add_log(f"MovL failed: {response}")  # MovL失败 / MovL failed
                return
            self.add_log(f"MovL: ({x}, {y}, {z}, {rx}, {ry}, {rz})")  # MovL坐标 / MovL coordinates
        except Exception as e:
            self.add_log(f"MovL failed: {e}")  # MovL失败 / MovL failed

    def joint_movj(self):
        """Joint MovJ motion / 关节坐标MovJ运动"""
        if not self.robot:
            return

        # Check robot mode / 检查机器人模式
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, cannot execute motion")  # 机器人处于错误状态，无法执行运动 / Robot in error state, cannot execute motion
            return

        if not self.global_state["enable"]:
            self.add_log("Robot not enabled, cannot execute motion")  # 机器人未使能，无法执行运动 / Robot not enabled, cannot execute motion
            return

        try:
            j1 = float(self.entry_dict["J1:"].get())
            j2 = float(self.entry_dict["J2:"].get())
            j3 = float(self.entry_dict["J3:"].get())
            j4 = float(self.entry_dict["J4:"].get())
            j5 = float(self.entry_dict["J5:"].get())
            j6 = float(self.entry_dict["J6:"].get())

            response = self.robot.motion.MovJ([j1, j2, j3, j4, j5, j6], CoordinateType.JOINT)
            if response and ("Failed" in str(response) or "Error" in str(response)):
                self.add_log(f"Joint MovJ failed: {response}")  # Joint MovJ失败 / Joint MovJ failed
                return
            self.add_log(f"MovJ(Joint): ({j1}, {j2}, {j3}, {j4}, {j5}, {j6})")  # MovJ关节坐标 / MovJ joint coordinates
        except Exception as e:
            self.add_log(f"Joint MovJ failed: {e}")  # Joint MovJ失败 / Joint MovJ failed

    def confirm_do(self):
        """Set digital output / 设置数字输出"""
        if not self.robot:
            return

        # Check robot mode / 检查机器人模式
        if self.current_robot_mode == 9:
            self.add_log("Robot is in error state, cannot set DO")  # 机器人处于错误状态，无法设置DO / Robot in error state, cannot set DO
            return

        try:
            index = int(self.entry_index.get())
            status = 1 if self.combo_status.get() == "On" else 0

            if status == 1:
                response = self.robot.io.DO(index, 1)
                if response and ("Failed" in str(response) or "Error" in str(response)):
                    self.add_log(f"Set DO{index} failed: {response}")  # 设置DO失败 / Set DO failed
                    return
                self.add_log(f"DO{index} set to high level")  # 设置为高电平 / set to high level
            else:
                response = self.robot.io.DO(index, 0)
                if response and ("Failed" in str(response) or "Error" in str(response)):
                    self.add_log(f"Set DO{index} failed: {response}")  # 设置DO失败 / Set DO failed
                    return
                self.add_log(f"DO{index} set to low level")  # 设置为低电平 / set to low level
        except Exception as e:
            self.add_log(f"Set DO failed: {e}")  # 设置DO失败 / Set DO failed

    def set_feed(self, text_list, x1, x2, x3, x4):
        """Set feedback display area / 设置反馈显示区域"""
        for i in range(6):
            self.set_button_bind(self.frame_feed, text_list[0][i], rely=0.2 + i*0.1, x=x1)
            self.set_label(self.frame_feed, text_list[1][i], rely=0.21 + i*0.1, x=x2)
            self.label_feed_dict[text_list[1][i]] = self.set_label(self.frame_feed, " ", rely=0.21 + i*0.1, x=x3)
            self.set_button_bind(self.frame_feed, text_list[2][i], rely=0.2 + i*0.1, x=x4)

    def feed_back(self):
        """Status feedback loop / 状态反馈循环"""
        while True:
            if not self.global_state["connect"] or not self.robot:
                break

            try:
                status = self.robot.GetStatus()
                if status:
                    # Update speed ratio / 更新速度比例
                    self.label_feed_speed["text"] = str(status.speed_scaling)

                    # Update robot mode / 更新机器人模式
                    mode_value = status.robot_mode.value
                    self.current_robot_mode = mode_value
                    self.label_robot_mode["text"] = LABEL_ROBOT_MODE.get(mode_value, f"Unknown({mode_value})")

                    # Update DI/DO status (using circular indicators) / 更新DI/DO状态（使用圆形指示灯）
                    di_value = status.digital_inputs
                    do_value = status.digital_outputs

                    for i, indicator in enumerate(self.di_indicators):
                        if (di_value >> i) & 1:
                            indicator.itemconfig(1, fill="green")
                        else:
                            indicator.itemconfig(1, fill="gray")

                    for i, indicator in enumerate(self.do_indicators):
                        if (do_value >> i) & 1:
                            indicator.itemconfig(1, fill="green")
                        else:
                            indicator.itemconfig(1, fill="gray")

                    # Update joint coordinates / 更新关节坐标
                    if hasattr(status, 'joint_state') and status.joint_state:
                        q_actual = status.joint_state.q_actual
                        if q_actual:
                            for i, label in enumerate(["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"]):
                                if i < len(q_actual):
                                    self.label_feed_dict[label]["text"] = f"{q_actual[i]:.2f}"

                    # Update Cartesian coordinates / 更新笛卡尔坐标
                    if hasattr(status, 'tool_vector_actual') and status.tool_vector_actual:
                        tool_vector = status.tool_vector_actual.to_list()
                        for i, label in enumerate(["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"]):
                            if i < len(tool_vector):
                                self.label_feed_dict[label]["text"] = f"{tool_vector[i]:.2f}"

                    # Check error status - display whenever there is an error / 检查错误状态 - 只要有错误就显示
                    if mode_value == 9:
                        self.display_error_info()
                    else:
                        # Even if not in ERROR mode, check for uncleared errors / 即使不在ERROR模式，也检查是否有未清除的错误
                        self.display_error_info()
            except Exception as e:
                self.add_log(f"Status feedback error: {e}")  # 状态反馈错误 / Status feedback error

            time.sleep(0.1)

    def display_error_info(self):
        """Display detailed error information (via HTTP interface) / 显示详细错误信息（通过HTTP接口）"""
        if not self.robot:
            return

        try:
            # Get error information via HTTP interface / 通过HTTP接口获取错误信息
            error_info = get_error(self.robot.ip, self.current_lang)

            # Get error message list / 获取错误消息列表
            err_msg_list = error_info.get("errMsg", [])

            # Check whether there are errors / 检查是否有错误
            if not err_msg_list:
                # Clear the display if there are no current errors and a previous error is still displayed. / 如果当前没有错误但仍显示旧错误，则清除显示。
                if self.last_error_message:
                    self.text_err.delete("1.0", "end")
                    self.last_error_message = None
                return

            # Format error information / 格式化错误信息
            error_message = format_error_messages_from_http(error_info)

            # Check whether error info has changed, skip refresh if unchanged / 检查错误信息是否变化，如果没变就不刷新
            if error_message == self.last_error_message:
                return

            # Update display / 更新显示
            self.text_err.delete("1.0", "end")
            self.text_err.insert(END, error_message)
            self.text_err.see(END)
            self.last_error_message = error_message

            # Log / 记录日志
            error_ids = [err.get("id") for err in err_msg_list]
            self.add_log(f"Error info updated: {error_ids}")  # 错误信息已更新 / Error info updated

        except Exception as e:
            self.add_log(f"Failed to get error info: {e}")  # 获取错误信息失败 / Failed to get error info

    def clear_error_info(self):
        """Clear error information display / 清除错误信息显示"""
        self.text_err.delete("1.0", "end")
        # Reset cache so the next error fetch updates correctly / 重置缓存，让下次获取错误时能正确更新
        self.last_error_message = None
        self.last_error_ids = None
