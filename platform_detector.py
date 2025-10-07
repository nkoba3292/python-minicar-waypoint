# platform_detector.py - プラットフォーム自動検出・ドライバー切り替え
import platform
import sys
import os

def is_raspberry_pi():
    """Raspberry Piで動作しているかを判定"""
    try:
        # /proc/cpuinfo から Raspberry Pi を検出
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()
        return 'Raspberry Pi' in cpuinfo or 'BCM' in cpuinfo
    except:
        return False

def is_windows():
    """Windows環境かを判定"""
    return platform.system() == 'Windows'

def is_linux():
    """Linux環境かを判定"""
    return platform.system() == 'Linux'

def get_platform_info():
    """現在のプラットフォーム情報を取得"""
    info = {
        'system': platform.system(),
        'machine': platform.machine(),
        'processor': platform.processor(),
        'is_raspberry_pi': is_raspberry_pi(),
        'is_windows': is_windows(),
        'is_linux': is_linux(),
        'python_version': sys.version
    }
    return info

def print_platform_info():
    """プラットフォーム情報を表示"""
    info = get_platform_info()
    print("=== Platform Information ===")
    for key, value in info.items():
        print(f"{key}: {value}")
    print("============================")

# 自動検出の実行
if __name__ == "__main__":
    print_platform_info()