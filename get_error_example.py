#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GetError Interface Usage Example / GetError接口使用示例
Demonstrates how to use the GetError interface to get robot alarm information in real projects
演示如何在实际项目中使用GetError接口获取机器人报警信息
"""

from dobot_api import DobotApiDashboard
import time
import json

class RobotErrorMonitor:
    """
    Robot Error Monitor Class / 机器人报警监控类
    A class for monitoring robot alarm information
    用于监控机器人报警信息的类
    """
    
    def __init__(self, robot_ip="192.168.200.1", dashboard_port=29999):
        self.robot_ip = robot_ip
        self.dashboard_port = dashboard_port
        self.dashboard = None
        
    def connect(self):
        """Connect to robot / 连接到机器人"""
        try:
            self.dashboard = DobotApiDashboard(self.robot_ip, self.dashboard_port)
            print(f"Successfully connected to robot / 成功连接到机器人: {self.robot_ip}:{self.dashboard_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to robot / 连接机器人失败: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from robot / 断开连接"""
        if self.dashboard:
            self.dashboard.close()
            print("Disconnected from robot / 已断开机器人连接")
    
    def get_error_info(self, language="zh_cn"):
        """
        Get error information / 获取报警信息
        
        Args:
            language (str): Language setting, supports / 语言设置，支持:
                           "zh_cn" - Simplified Chinese / 简体中文
                           "zh_hant" - Traditional Chinese / 繁体中文  
                           "en" - English / 英语
                           "ja" - Japanese / 日语
                           "de" - German / 德语
                           "vi" - Vietnamese / 越南语
                           "es" - Spanish / 西班牙语
                           "fr" - French / 法语
                           "ko" - Korean / 韩语
                           "ru" - Russian / 俄语
        
        Returns:
            dict: Error information dictionary / 报警信息字典
        """
        if not self.dashboard:
            print("Not connected to robot / 未连接到机器人")
            return None
            
        return self.dashboard.GetError(language)
    
    def check_errors(self, language="zh_cn"):
        """
        Check and display current error information / 检查并显示当前报警信息
        
        Args:
            language (str): Display language / 显示语言
            
        Returns:
            bool: True means there are errors, False means no errors / True表示有报警，False表示无报警
        """
        error_info = self.get_error_info(language)
        
        if not error_info or "errMsg" not in error_info:
            print("Failed to get error information / 获取报警信息失败")
            return False
        
        errors = error_info["errMsg"]
        
        if not errors:
            print("✅ Robot status normal, no error information / 机器人状态正常，无报警信息")
            return False
        
        print(f"⚠️  Found {len(errors)} error(s) / 发现 {len(errors)} 个报警:")
        print("=" * 50)
        
        for i, error in enumerate(errors, 1):
            print(f"Error / 报警 {i}:")
            print(f"  🆔 ID: {error.get('id', 'N/A')}")
            print(f"  📊 Level / 级别: {error.get('level', 'N/A')}")
            print(f"  📝 Description / 描述: {error.get('description', 'N/A')}")
            print(f"  🔧 Solution / 解决方案: {error.get('solution', 'N/A')}")
            print(f"  🏷️  Mode / 模式: {error.get('mode', 'N/A')}")
            print(f"  📅 Date / 日期: {error.get('date', 'N/A')}")
            print(f"  🕐 Time / 时间: {error.get('time', 'N/A')}")
            print("-" * 30)
        
        return True
    
    def monitor_errors(self, interval=5, language="zh_cn"):
        """
        Continuously monitor error information / 持续监控报警信息
        
        Args:
            interval (int): Check interval (seconds) / 检查间隔（秒）
            language (str): Display language / 显示语言
        """
        print(f"Start monitoring robot error information (check every {interval} seconds) / 开始监控机器人报警信息（每{interval}秒检查一次）")
        print("Press Ctrl+C to stop monitoring / 按 Ctrl+C 停止监控")
        
        try:
            while True:
                print(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Checking error information / 检查报警信息...")
                has_errors = self.check_errors(language)
                
                if has_errors:
                    print("\n⚠️  Recommend handling error information immediately / 建议立即处理报警信息！")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped / 监控已停止")
    
    def save_error_log(self, filename=None, language="zh_cn"):
        """
        Save error information to file / 保存报警信息到文件
        
        Args:
            filename (str): Save filename, default is current timestamp / 保存文件名，默认为当前时间戳
            language (str): Language setting / 语言设置
        """
        if filename is None:
            filename = f"robot_errors_{time.strftime('%Y%m%d_%H%M%S')}.json"
        
        error_info = self.get_error_info(language)
        
        if error_info:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(error_info, f, ensure_ascii=False, indent=2)
                print(f"Error information saved to / 报警信息已保存到: {filename}")
            except Exception as e:
                print(f"Failed to save file / 保存文件失败: {e}")
        else:
            print("Unable to get error information / 无法获取报警信息")

def main():
    """Main function - Demonstrate various usage methods / 主函数 - 演示各种使用方式"""
    
    # Create monitor instance / 创建监控器实例
    monitor = RobotErrorMonitor()
    
    # Connect to robot / 连接机器人
    if not monitor.connect():
        return
    
    try:
        print("\n=== GetError Interface Usage Example / GetError接口使用示例 ===")
        
        # 1. Basic usage - Check current errors / 基本使用 - 检查当前报警
        print("\n1. Check current error information / 检查当前报警信息:")
        monitor.check_errors("zh_cn")
        
        # 2. Multi-language support / 多语言支持
        print("\n2. Multi-language support demonstration / 多语言支持演示:")
        languages = {
            "zh_cn": "简体中文 / Simplified Chinese",
            "en": "English / 英语",
            "ja": "日本語 / Japanese"
        }
        
        for lang_code, lang_name in languages.items():
            print(f"\n--- {lang_name} ({lang_code}) ---")
            monitor.check_errors(lang_code)
        
        # 3. Save error log / 保存报警日志
        print("\n3. Save error log / 保存报警日志:")
        monitor.save_error_log()
        
        # 4. Get raw data / 获取原始数据
        print("\n4. Get raw JSON data / 获取原始JSON数据:")
        raw_data = monitor.get_error_info("zh_cn")
        if raw_data:
            print(json.dumps(raw_data, ensure_ascii=False, indent=2))
        
        # 5. Optional: Start continuous monitoring (uncomment to enable) / 可选：启动持续监控（取消注释以启用）
        # print("\n5. Start continuous monitoring / 启动持续监控:")
        # monitor.monitor_errors(interval=10, language="zh_cn")
        
    finally:
        # Disconnect / 断开连接
        monitor.disconnect()

if __name__ == "__main__":
    main()