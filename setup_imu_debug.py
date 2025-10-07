# setup_imu_debug.py
# -*- coding: utf-8 -*-
"""
BNO055 IMUデバッグ環境セットアップスクリプト
必要なライブラリのインストールと環境設定
"""

import subprocess
import sys
import os

def install_package(package_name):
    """パッケージをインストール"""
    try:
        print(f"📦 Installing {package_name}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
        print(f"✅ {package_name} installed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install {package_name}: {e}")
        return False

def check_package(package_name):
    """パッケージのインストール確認"""
    try:
        __import__(package_name)
        print(f"✅ {package_name} is already installed")
        return True
    except ImportError:
        print(f"⚠️ {package_name} is not installed")
        return False

def setup_imu_debug_environment():
    """IMUデバッグ環境セットアップ"""
    print("🧭 BNO055 IMU Debug Environment Setup")
    print("="*50)
    
    # 必要なパッケージリスト
    packages = [
        'pyserial',      # シリアル通信
        'numpy',         # 数値計算
        'matplotlib',    # グラフ表示（オプション）
    ]
    
    # ラズパイ専用パッケージ
    raspberry_packages = [
        'RPi.GPIO',      # GPIO制御（ラズパイのみ）
    ]
    
    print("📋 Checking required packages...")
    
    # 基本パッケージの確認・インストール
    for package in packages:
        if not check_package(package):
            if not install_package(package):
                print(f"❌ Setup failed: Could not install {package}")
                return False
    
    # ラズパイ環境の場合の追加パッケージ
    if os.path.exists('/proc/device-tree/model'):
        print("🔍 Raspberry Pi detected, installing additional packages...")
        for package in raspberry_packages:
            if not check_package(package):
                install_package(package)
    
    print("\n🎯 Hardware Configuration Guide")
    print("="*50)
    print("📌 BNO055 → Raspberry Pi 4B Connection:")
    print("   VCC  → 3.3V (Pin 1)")
    print("   GND  → GND  (Pin 6)")
    print("   SDA  → GPIO 2 (Pin 3)")
    print("   SCL  → GPIO 3 (Pin 5)")
    print("   PS0  → GND (for I2C mode)")
    print("   PS1  → 3.3V (for I2C mode)")
    print("")
    print("🔧 UART Configuration (if using serial mode):")
    print("   TX   → GPIO 15 (Pin 10)")
    print("   RX   → GPIO 14 (Pin 8)")
    print("   PS0  → 3.3V (for UART mode)")
    print("   PS1  → GND (for UART mode)")
    print("")
    print("⚙️ Raspberry Pi UART Setup:")
    print("   1. sudo raspi-config")
    print("   2. Interface Options → Serial Port")
    print("   3. Login shell: No, Serial interface: Yes")
    print("   4. Add 'enable_uart=1' to /boot/config.txt")
    print("   5. Reboot")
    
    print("\n✅ Setup completed successfully!")
    print("🚀 Run 'python imu_debug_bno055.py' to start debugging")
    
    return True

if __name__ == "__main__":
    setup_imu_debug_environment()