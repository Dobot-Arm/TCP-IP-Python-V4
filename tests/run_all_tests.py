"""
Run all tests
运行所有测试
"""

import os
import sys
import subprocess

# 添加SDK路径 / Add SDK path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def run_test(test_file):
    """
    Run a single test file
    运行单个测试文件
    """
    print(f"\n{'='*60}")
    print(f"Running test: {test_file}")  # 运行测试 / Running test
    print('='*60)
    
    try:
        result = subprocess.run(
            [sys.executable, test_file],
            capture_output=True,
            text=True,
            cwd=os.path.dirname(__file__)
        )
        
        print(result.stdout)
        
        if result.stderr:
            print("Error output:")  # 错误输出 / Error output
            print(result.stderr)
        
        return result.returncode == 0
    except Exception as e:
        print(f"Test run failed: {e}")  # 测试运行失败 / Test run failed
        return False


def main():
    """
    Main function
    主函数
    """
    print("="*60)
    print("Dobot SDK V4 - Test Suite")  # 测试套件 / Test Suite
    print("="*60)
    
    test_files = [
        'test_basic.py',
        'test_io.py',
        'test_motion.py',
        'test_robot_control.py',
        'test_plugins.py',
        'test_communication.py'
    ]
    
    passed = 0
    failed = 0
    
    for test_file in test_files:
        file_path = os.path.join(os.path.dirname(__file__), test_file)
        
        if os.path.exists(file_path):
            if run_test(file_path):
                passed += 1
            else:
                failed += 1
        else:
            print(f"\nWarning: Test file does not exist - {test_file}")  # 警告: 测试文件不存在 / Warning: Test file does not exist
            failed += 1
    
    # 输出总结 / Output summary
    print("\n" + "="*60)
    print("Test Summary")  # 测试总结 / Test Summary
    print("="*60)
    print(f"Passed: {passed}")  # 通过 / Passed
    print(f"Failed: {failed}")  # 失败 / Failed
    print(f"Total: {passed + failed}")  # 总计 / Total
    
    if failed == 0:
        print("\n🎉 All tests passed!")  # 所有测试通过 / All tests passed
        return 0
    else:
        print(f"\n❌ {failed} test(s) failed")  # 测试失败 / tests failed
        return 1


if __name__ == "__main__":
    sys.exit(main())
