# TCP-IP-Python-V4 项目说明文档

## 项目概述

本项目是越疆机器人TCP-IP-CR-Python-V4二次开发API程序，用于通过TCP/IP协议控制越疆机器人。项目提供了完整的机器人控制接口，包括运动控制、状态监控、报警处理等功能。

## 环境要求

### Python版本

- Python 3.6 或更高版本

### 必需安装的库

```bash
# 基础数值计算库
pip install numpy

# JSON数据处理（Python内置，无需安装）
# import json

# 网络通信（Python内置，无需安装）
# import socket

# 多线程支持（Python内置，无需安装）
# import threading

# 时间处理（Python内置，无需安装）
# import time

# 正则表达式（Python内置，无需安装）
# import re

# GUI界面库（如果使用ui.py）
pip install tkinter  # 通常Python自带
```

### 网络配置要求

- 本机IP地址需设置为192.168.X.X网段
- 机器人需切换至TCP/IP模式
- 确保29999和30004端口未被占用

## 主要程序文件及功能

### 1. main.py

**功能**: 项目主入口文件

- 演示基本的机器人连接和控制流程
- 包含完整的机器人操作示例
- 适合初学者了解项目结构

### 2. dobot_api.py

**功能**: 核心API接口文件

- **DobotApi**: 基础通信类，处理TCP连接
- **DobotApiDashboard**: 机器人控制接口类
  - 机器人使能/下使能
  - 运动控制指令（MovJ, MovL, Arc等）
  - 状态查询和设置
  - 报警信息获取（包含新增的GetError接口）
- **DobotApiFeedBack**: 状态反馈类
  - 实时获取机器人状态信息
  - 监控机器人运行模式
  - 获取当前指令ID
- **MyType**: 数据类型定义
- **alarm_controller**: 控制器报警处理
- **alarm_servo**: 伺服报警处理

### 3. ui.py

**功能**: 图形用户界面程序

- 提供可视化的机器人控制界面
- 集成了机器人连接、运动控制、状态显示等功能
- 支持实时显示机器人状态和报警信息
- 优先使用GetError接口获取报警信息，失败时回退到原有方式

### 4. 测试和示例文件

#### get_error_example.py

**功能**: GetError接口使用示例

- 提供RobotErrorMonitor类，用于报警监控
- 演示如何获取和处理多语言报警信息
- 包含报警信息保存到文件的功能
- 注释采用中英文对照

### 5. 文档文件

#### GetError_README.md

**功能**: GetError接口中文说明文档

- 详细说明GetError接口的使用方法
- 包含接口参数、返回值、示例代码等
- 提供故障排除和注意事项

#### GetError_README_EN.md

**功能**: GetError接口英文说明文档

- GetError_README.md的英文版本
- 便于国际用户理解和使用

## 项目目录结构

TCP-IP-Python-V4/
├── main.py                    # 主程序入口
├── dobot_api.py               # 核心API接口
├── ui.py                      # 图形界面程序
├── PythonExample.py           # Python示例
├── get_error_example.py       # GetError使用示例
├── GetError_README.md         # GetError中文文档
├── GetError_README_EN.md      # GetError英文文档
├── README.md                  # 项目说明文档
└── files/                     # 其他支持文件

## 快速开始

### 1. 环境准备

```bash
# 克隆项目
git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git

# 安装依赖
pip install numpy
```

### 2. 网络配置

- 设置本机IP为192.168.X.X网段
- 确保机器人处于TCP/IP模式

### 3. 运行程序

# 运行主程序
python main.py

# 或运行图形界面
python main_UI.py


## 常见问题解决

### 1. ModuleNotFoundError: No module named 'numpy'

**解决方法**: 安装numpy库

```bash
pip install numpy
```

### 2. Connection refused, IP:Port has been occupied

**解决方法**: 检查29999端口是否被占用，关闭占用该端口的程序

### 3. Control Mode Is Not Tcp

**解决方法**: 在DobotStudio Pro中将机器人模式切换至TCP/IP模式

### 4. 机器人状态异常

| 输出信息                             | 机器状态     | 解决方法                 |
| ------------------------------------ | ------------ | ------------------------ |
| Command execution failed             | 指令执行失败 | 检查指令参数和机器人状态 |
| The robot is in an error state       | 机器错误状态 | 清除报警后重试           |
| The robot is in emergency stop state | 急停状态     | 释放急停按钮             |
| The robot is in power down state     | 下电状态     | 给机器人上电             |

## 注意事项

1. **安全第一**: 运行示例前请确保机器人处于安全位置，防止发生碰撞
2. **网络配置**: 确保网络配置正确，IP地址在同一网段
3. **端口占用**: 确保29999和30004端口未被其他程序占用
4. **机器人模式**: 确保机器人处于TCP/IP控制模式
5. **权限问题**: 某些操作可能需要管理员权限

## 技术支持

如遇到问题，请参考：项目README.md文档

- GetError相关文档
- 示例代码和测试程序
- 越疆官方技术支持

---

**版本**: V4
**更新日期**: 2025-9-5
**维护**: dobot_futingxing
