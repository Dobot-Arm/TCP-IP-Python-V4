# -*- coding: utf-8 -*-
import sys
import time
import threading
import queue
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot, CoordinateType, set_log_level
set_log_level('ERROR')  # 只显示ERROR级别日志 / Only show ERROR level logs


class RobotController:
    def __init__(self, ip):
        self.ip = ip
        self.robot = None
        self.running = False

        self.status_queue = queue.Queue(maxsize=10)
        self.monitor_thread = None
        self.latest_status = None
        self.status_lock = threading.Lock()
        self.last_printed_state = None  # 记录上次打印的状态 / Record the last printed state

        # 状态统计 / Status statistics
        self.status_combinations = {}
        self.abnormal_events = []
        self.motion_start_time = None
        self.total_motion_count = 0

    def _monitor_loop(self):
        """
        Monitor thread: continuously receive and process robot status feedback
        监听线程：持续接收并处理机器人状态反馈
        """
        print("[INFO] Monitor thread started, receiving status feedback...")  # [INFO] 监听线程已启动，开始接收状态反馈...
        while self.running:
            try:
                if not self.status_queue.empty():
                    status = self.status_queue.get(timeout=0.1)
                    with self.status_lock:
                        self.latest_status = status
                    self._process_status(status)
                time.sleep(0.005)  # 5ms间隔，降低CPU占用 / 5ms interval to reduce CPU usage
            except queue.Empty:
                continue
            except Exception as e:
                print(f"\n[ERR] Monitor thread exception: {str(e)}")  # [ERR] 监听线程异常: {str(e)}
                break
        print("\n[INFO] Monitor thread stopped")  # [INFO] 监听线程已停止

    def _process_status(self, status):
        """
        Process status data and detect anomalies
        处理状态数据，检测异常
        """
        rs = status.running_status
        rm = status.robot_mode.value if hasattr(status.robot_mode, 'value') else status.robot_mode

        # 统计状态组合 / Count status combinations
        key = (rs, rm)
        self.status_combinations[key] = self.status_combinations.get(key, 0) + 1

        # 检测异常：运行模式下应该运动但实际空闲 / Detect anomaly: should be moving in run mode but is actually idle
        is_abnormal = (rm == 7 and rs == 0)
        if is_abnormal:
            self.abnormal_events.append({'time': time.time(), 'rs': rs, 'rm': rm})

        # 打印实时状态（仅状态变化时打印） / Print real-time status (only when the status changes)
        self._print_status(status)

    def _print_status(self, status):
        """
        Print real-time status information
        打印实时状态信息
        """
        rs = status.running_status
        rm = status.robot_mode.value if hasattr(status.robot_mode, 'value') else status.robot_mode

        # 状态描述映射 / Status description mapping
        mode_desc = {
            1: "Initializing",  # 初始化
            2: "Manual",  # 手动
            3: "Auto",  # 自动
            4: "Remote",  # 远程
            5: "Idle",  # 空闲
            6: "Drag",  # 拖拽
            7: "Running",  # 运行
            9: "Error",  # 错误
            10: "Paused",  # 暂停
            11: "Jog"  # 点动
        }

        # 只在状态变化时打印 / Only print when the status changes
        current_state = (rs, rm)
        if current_state != self.last_printed_state:
            self.last_printed_state = current_state
            status_text = "Moving" if rs != 0 else "Idle"  # 运动中 / 空闲
            status_color = "[RUN]" if rs != 0 else "[IDLE]"
            mode_text = mode_desc.get(rm, f"Unknown({rm})")  # 未知
            pos = status.tool_vector_actual
            print(f"{status_color} RS:{status_text} | RM:{mode_text} | Speed:{status.speed_scaling:.0f}% | X:{pos.x:.1f} Y:{pos.y:.1f} Z:{pos.z:.1f}")  # 速度:{...}%

    def _wait_for_motion_complete(self, timeout=30):
        """
        Wait for motion to complete
        等待运动完成
        """
        print(f"[WAIT] Waiting for motion to complete")  # [WAIT] 等待运动完成
        start = time.time()

        # 等待运动开始 / Wait for motion to start
        while time.time() - start < 5:
            with self.status_lock:
                if self.latest_status and self.latest_status.running_status != 0:
                    break
            time.sleep(0.05)

        # 等待运动结束 / Wait for motion to end
        while time.time() - start < timeout:
            with self.status_lock:
                if self.latest_status and self.latest_status.running_status == 0:
                    # 获取关节角 / Get the joint angles
                    q = self.latest_status.q_actual if self.latest_status else [0]*6
                    elapsed = time.time() - start
                    print(f"[OK] Motion complete: {elapsed:.2f}s | Joint angles: J1={q[0]:.4f} J2={q[1]:.4f} J3={q[2]:.4f}")  # [OK] 运动完成: ... | 关节角: ...
                    return True
            time.sleep(0.05)

        return False

    def execute_point_by_point(self):
        """
        Point-by-point motion test - compare the precision of RS and RM status
        逐点运动测试 - 对比RS和RM状态的精度
        """
        points = [
            [200, -200, 300, 180, 0, -180],
            [200, -200, 400, 180, 0, -180],
            [300, -200, 400, 180, 0, -180],
            [300, -100, 400, 180, 0, -180],
            [200, -200, 300, 180, 0, -180],
        ]

        print(f"\nPoint-by-point motion test: {len(points)} points")  # 逐点运动测试: {len(points)}个点

        # 保存每个点在RM=7和RM=5状态下的关节角 / Save the joint angles of each point under RM=7 and RM=5 states
        point_data = {tuple(p[:3]): {'rm7': [], 'rm5': []} for p in points}

        for idx, point in enumerate(points, 1):
            print(f"  [{idx}/{len(points)}] -> {point[:3]}")
            try:
                self.robot.motion.MovL(point, CoordinateType.CARTESIAN)
                self.total_motion_count += 1
                result = self._wait_for_motion_complete_with_joints(15)
                if result:
                    rm7_joints, rm5_joints = result
                    point_data[tuple(point[:3])]['rm7'].extend(rm7_joints)
                    if rm5_joints:
                        point_data[tuple(point[:3])]['rm5'].append(rm5_joints[-1])
                time.sleep(0.3)
            except Exception as e:
                print(f"    [ERR] {str(e)}")
                break

        # 对比精度 / Compare precision
        print("\n" + "="*60)
        print("RS vs RM Precision Comparison")  # RS vs RM 精度对比
        print("="*60)
        print(f"{'Point':<15} {'J1 range at RM=7':<18} {'J1 range at RM=5':<18} {'Conclusion'}")  # {'点':...}{'RM=7时J1范围':...}{'RM=5时J1范围':...}{'结论'}
        print("-"*60)

        for pos, data in point_data.items():
            rm7 = data['rm7']
            rm5 = data['rm5']

            if len(rm7) >= 2:
                rm7_j1 = [j[0] for j in rm7]
                rm7_range = f"{min(rm7_j1):.4f}~{max(rm7_j1):.4f}"
                rm7_diff = max(rm7_j1) - min(rm7_j1)
            else:
                rm7_range = "Insufficient data"  # 数据不足
                rm7_diff = 0

            if len(rm5) >= 1:
                rm5_j1 = [j[0] for j in rm5]
                rm5_range = f"{min(rm5_j1):.4f}~{max(rm5_j1):.4f}"
                rm5_diff = max(rm5_j1) - min(rm5_j1) if len(rm5) > 1 else 0
            else:
                rm5_range = "Insufficient data"  # 数据不足
                rm5_diff = 0

            conclusion = "RM=5 is more stable" if rm7_diff > rm5_diff else "Similar"  # RM=5更稳定 / 相近
            print(f"{str(pos):<15} {rm7_range:<18} {rm5_range:<18} {conclusion}")

        print("\nCompleted: {0} motions".format(self.total_motion_count))  # 完成: {0}次运动

    def _wait_for_motion_complete_with_joints(self, timeout=30):
        """
        Wait for motion to complete, comparing the joint angle precision of RS and RM states
        等待运动完成，对比RS和RM状态的关节角精度
        """
        start = time.time()

        # 等待运动开始 / Wait for motion to start
        while time.time() - start < 5:
            with self.status_lock:
                if self.latest_status and self.latest_status.running_status != 0:
                    break
            time.sleep(0.05)

        # 收集不同状态下的关节角 / Collect joint angles under different states
        rs0_rm7_joints = []  # RS=0, RM=7 状态 / RS=0, RM=7 state
        rm5_joints = []      # RM=5 状态 / RM=5 state

        motion_ended = False
        while time.time() - start < timeout:
            with self.status_lock:
                if self.latest_status:
                    rs = self.latest_status.running_status
                    rm = self.latest_status.robot_mode
                    q = self.latest_status.joint_state.q_actual

                    # 记录 RM=7 时 RS=0 的关节角（过渡状态） / Record joint angles when RM=7 and RS=0 (transition state)
                    if rs == 0 and rm == 7:
                        rs0_rm7_joints.append(q.copy())
                        motion_ended = True

                    # 记录 RM=5 的关节角（稳定状态） / Record joint angles when RM=5 (stable state)
                    if rm == 5 and rs == 0:
                        rm5_joints.append(q.copy())
                        elapsed = time.time() - start
                        print(f"    [OK] {elapsed:.2f}s | J1={q[0]:.4f} J2={q[1]:.4f} J3={q[2]:.4f}")
                        return (rs0_rm7_joints, rm5_joints)
            time.sleep(0.05)

        return (rs0_rm7_joints, rm5_joints)

    def execute_validation(self):
        """
        Status validation test
        状态验证测试
        """
        print("\n" + "="*50)
        print("Status Validation Test")  # 状态验证测试
        print("="*50)

        test_point = [200, -200, 300, 180, 0, -180]
        print(f"\nTime       RS   RM   State          J1 angle   Stable")  # 时刻       RS   RM   状态           J1角度    稳定
        print("-"*60)

        state_records = []
        start_time = time.time()

        try:
            response = self.robot.motion.MovL(test_point, CoordinateType.CARTESIAN)
            self.total_motion_count += 1

            stable_count = 0
            while stable_count < 50 and (time.time() - start_time) < 20:
                if not self.status_queue.empty():
                    status = self.status_queue.get(block=True, timeout=0.1)
                    rs = status.running_status if hasattr(status, 'running_status') else 0
                    rm = status.robot_mode if hasattr(status, 'robot_mode') else 0
                    j1 = status.q_actual[0] if hasattr(status, 'q_actual') and status.q_actual else 0
                    elapsed = time.time() - start_time

                    state_desc = self._get_state_description(rs, rm)
                    is_stable = (rs == 0 and rm == 5)

                    state_records.append({
                        'time': elapsed, 'rs': rs, 'rm': rm, 'j1': j1, 'stable': is_stable
                    })

                    print(f"{elapsed:6.2f}s  {rs:<4} {rm:<4} {state_desc:<14} {j1:>8.3f}   {'[OK]' if is_stable else ''}")

                    stable_count = stable_count + 1 if is_stable else 0

            # 分析 / Analysis
            print("\nState durations:")  # 状态持续时间:
            state_durations = {}
            for i in range(len(state_records)-1):
                key = f"RS={state_records[i]['rs']},RM={state_records[i]['rm']}"
                dur = state_records[i+1]['time'] - state_records[i]['time']
                state_durations[key] = state_durations.get(key, 0) + dur

            for state, dur in sorted(state_durations.items(), key=lambda x: -x[1]):
                print(f"  {state:<15}: {dur*1000:>6.0f} ms")

            # 结论 / Conclusion
            print("\nConclusion:")  # 结论:
            print("  Joint angles are stable and reliable when RM=5")  # RM=5时关节角稳定可靠
            print("  RM=7 is a transition state. Joint angles may change")  # RM=7时为过渡状态，关节角可能变化

        except Exception as e:
            print(f"[ERR] {str(e)}")

    def _get_state_description(self, rs, rm):
        """
        Get the state description
        获取状态描述
        """
        rs_desc = "Moving" if rs != 0 else "Idle"  # 运动中 / 空闲
        rm_desc = {
            1: "Initializing", 2: "Manual", 3: "Auto", 4: "Remote", 5: "Idle",  # 初始化/手动/自动/远程/空闲
            6: "Drag", 7: "Running", 9: "Error", 10: "Paused", 11: "Jog"  # 拖拽/运行/错误/暂停/点动
        }.get(rm, f"Unknown({rm})")  # 未知
        return f"{rs_desc}/{rm_desc}"

    def execute_continuous(self):
        """
        Continuous fast motion test
        连续快速运动测试
        """
        point_a = [200, -200, 300, 180, 0, -180]
        point_b = [300, -100, 400, 180, 0, -180]

        print(f"\n{'='*50}")
        print(f"Starting continuous motion test (A ↔ B)")  # 开始连续运动测试 (A ↔ B)
        print(f"Point A: {point_a}")  # 点A: {point_a}
        print(f"Point B: {point_b}")  # 点B: {point_b}
        print(f"Press Ctrl+C to stop")  # 按 Ctrl+C 停止
        print(f"{'='*50}")

        count = 0
        start_time = time.time()

        try:
            while self.running:
                target_point = point_a if count % 2 == 0 else point_b
                try:
                    self.robot.motion.MovL(target_point, CoordinateType.CARTESIAN)
                    self.total_motion_count += 1

                    if count > 0 and count % 10 == 0:
                        elapsed = time.time() - start_time
                        rate = count / elapsed if elapsed > 0 else 0
                        print(f"\r[MOVE] Sent {count} commands | Rate: {rate:.1f} commands/sec", end="", flush=True)  # [MOVE] 已发送 {count} 个指令 | 速率: {rate:.1f} 指令/秒

                    count += 1
                    time.sleep(0.05)  # 50ms间隔 / 50ms interval

                except Exception as e:
                    print(f"\n[ERR] Send failed: {str(e)}")  # [ERR] 发送失败: {str(e)}
                    time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[STOP] User terminated continuous motion")  # [STOP] 用户终止连续运动

        elapsed = time.time() - start_time
        rate = count / elapsed if elapsed > 0 else 0

        print(f"\n{'='*50}")
        print(f"Continuous motion test stopped")  # 连续运动测试停止
        print(f"Commands sent: {count}")  # 发送指令数: {count}
        print(f"Total time: {elapsed:.2f}s")  # 总耗时: {elapsed:.2f}s
        print(f"Average rate: {rate:.2f} commands/sec")  # 平均速率: {rate:.2f} 指令/秒
        print(f"{'='*50}")

    def _status_callback(self, status):
        """
        Status callback function
        状态回调函数
        """
        if self.running and not self.status_queue.full():
            try:
                self.status_queue.put(status, block=False)
            except queue.Full:
                # 队列满时丢弃最新数据 / Discard the newest data when the queue is full
                pass

    def start(self):
        """
        Start the controller
        启动控制器
        """
        try:
            self.robot = DobotRobot(self.ip, connect_timeout=5.0, receive_timeout=10.0)
            self.robot.Connect()
            self.robot.robot_control.RequestControl()
            self.robot.robot_control.ClearError()
            self.robot.StartFeedbackMonitor(callback=self._status_callback)
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            print("[OK] Controller started successfully")  # [OK] 控制器启动完成

        except Exception as e:
            print(f"[ERR] Startup failed: {str(e)}")  # [ERR] 启动失败: {str(e)}
            self.stop()
            raise

    def stop(self):
        """
        Stop the controller
        停止控制器
        """
        self.running = False

        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

        if self.robot:
            self.robot.StopFeedbackMonitor()
            try:
                self.robot.robot_control.DisableRobot()
            except:
                pass
            try:
                self.robot.Disconnect()
            except:
                pass

        # 打印总结报告 / Print the summary report
        self._print_summary()

        print("\n[OK] Controller fully stopped")  # [OK] 控制器已完全停止

    def _print_summary(self):
        """
        Print the test summary report
        打印测试总结报告
        """
        if self.abnormal_events or self.total_motion_count > 0:
            print(f"\n[Summary] Motions: {self.total_motion_count} | Anomalies: {len(self.abnormal_events)}")  # [总结] 运动: ...次 | 异常: ...次

        print("\n" + "="*50)


def main(enable_motion=False, motion_type='point_by_point'):
    """
    Main function
    主函数
    """
    ROBOT_IP = "192.168.5.1"
    controller = RobotController(ROBOT_IP)

    try:
        controller.start()
        time.sleep(1)

        # 主线程：发送控制指令 / Main thread: send control commands
        print("\n" + "-"*50)
        # 上电、使能、设置速度 / Power on, enable, set speed
        controller.robot.robot_control.PowerOn()
        controller.robot.robot_control.EnableRobot(load=1.0)
        controller.robot.robot_control.SpeedFactor(30)

        # 运动测试 / Motion test
        if enable_motion:
            print("\nStarting motion test")  # 开始运动测试
            if motion_type == 'point_by_point':
                controller.execute_point_by_point()
            elif motion_type == 'continuous':
                controller.execute_continuous()
            elif motion_type == 'validation':
                controller.execute_validation()
        else:
            while True:
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERR] {str(e)}")
    finally:
        controller.stop()


if __name__ == "__main__":
    ENABLE_MOTION = True
    MOTION_TYPE = 'point_by_point'  # validation / point_by_point / continuous
    main(enable_motion=ENABLE_MOTION, motion_type=MOTION_TYPE)
