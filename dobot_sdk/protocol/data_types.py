# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Data structure definitions / 数据结构定义

Defines the structure of Feedback data packets
定义Feedback数据包的结构
"""

import numpy as np

# Feedback packet size / Feedback数据包大小
FEEDBACK_PACKET_SIZE = 1440

# Test value constant (used to verify packet validity) / 测试值常量（用于验证数据包有效性）
TEST_VALUE_MAGIC = 0x123456789abcdef

# Define the Feedback data structure / 定义Feedback数据结构
MyType = np.dtype([
    ('len', np.uint16),
    ('reserve', np.byte, (6,)),
    ('DigitalInputs', np.uint64),
    ('DigitalOutputs', np.uint64),
    ('RobotMode', np.uint64),
    ('TimeStamp', np.uint64),
    ('RunTime', np.uint64),
    ('TestValue', np.uint64),
    ('reserve2', np.byte, (8,)),
    ('SpeedScaling', np.float64),
    ('reserve3', np.byte, (16,)),
    ('VRobot', np.float64),
    ('IRobot', np.float64),
    ('ProgramState', np.float64),
    ('SafetyIOIn', np.uint16),
    ('SafetyIOOut', np.uint16),
    ('reserve4', np.byte, (76,)),
    ('QTarget', np.float64, (6,)),
    ('QDTarget', np.float64, (6,)),
    ('QDDTarget', np.float64, (6,)),
    ('ITarget', np.float64, (6,)),
    ('MTarget', np.float64, (6,)),
    ('QActual', np.float64, (6,)),
    ('QDActual', np.float64, (6,)),
    ('IActual', np.float64, (6,)),
    ('ActualTCPForce', np.float64, (6,)),
    ('ToolVectorActual', np.float64, (6,)),
    ('TCPSpeedActual', np.float64, (6,)),
    ('TCPForce', np.float64, (6,)),
    ('ToolVectorTarget', np.float64, (6,)),
    ('TCPSpeedTarget', np.float64, (6,)),
    ('MotorTemperatures', np.float64, (6,)),
    ('JointModes', np.float64, (6,)),
    ('VActual', np.float64, (6,)),
    ('HandType', np.byte, (4,)),
    ('User', np.byte),
    ('Tool', np.byte),
    ('RunQueuedCmd', np.byte),
    ('PauseCmdFlag', np.byte),
    ('VelocityRatio', np.byte),
    ('AccelerationRatio', np.byte),
    ('reserve5', np.byte),
    ('XYZVelocityRatio', np.byte),
    ('RVelocityRatio', np.byte),
    ('XYZAccelerationRatio', np.byte),
    ('RAccelerationRatio', np.byte),
    ('reserve6', np.byte, (2,)),
    ('BrakeStatus', np.byte),
    ('EnableStatus', np.byte),
    ('DragStatus', np.byte),
    ('RunningStatus', np.byte),
    ('ErrorStatus', np.byte),
    ('JogStatusCR', np.byte),
    ('CRRobotType', np.byte),
    ('DragButtonSignal', np.byte),
    ('EnableButtonSignal', np.byte),
    ('RecordButtonSignal', np.byte),
    ('ReappearButtonSignal', np.byte),
    ('JawButtonSignal', np.byte),
    ('SixForceOnline', np.byte),
    ('CollisionState', np.byte),
    ('ArmApproachState', np.byte),
    ('J4ApproachState', np.byte),
    ('J5ApproachState', np.byte),
    ('J6ApproachState', np.byte),
    ('reserve7', np.byte, (61,)),
    ('VibrationDisZ', np.float64),
    ('CurrentCommandId', np.uint64),
    ('MActual', np.float64, (6,)),
    ('Load', np.float64),
    ('CenterX', np.float64),
    ('CenterY', np.float64),
    ('CenterZ', np.float64),
    ('UserValue', np.float64, (6,)),
    ('ToolValue', np.float64, (6,)),
    ('reserve8', np.byte, (8,)),
    ('SixForceValue', np.float64, (6,)),
    ('TargetQuaternion', np.float64, (4,)),
    ('ActualQuaternion', np.float64, (4,)),
    ('AutoManualMode', np.uint16),
    ('ExportStatus', np.uint16),
    ('SafetyState', np.byte),
    ('SafeState', np.byte),
    ('reserve9', np.byte, (18,))
])
