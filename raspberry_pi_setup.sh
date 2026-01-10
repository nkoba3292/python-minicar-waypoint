#!/bin/bash
# raspberry_pi_setup.sh
# Raspberry Pi側の環境セットアップスクリプト

echo "🍓 Raspberry Pi IMU環境セットアップ開始"

# I2C有効化
echo "📡 I2C設定確認..."
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo "✅ I2C有効化完了"
else
    echo "✅ I2C既に有効"
fi

# I2Cツールインストール
echo "🔧 I2Cツールインストール..."
sudo apt update
sudo apt install -y i2c-tools python3-pip python3-smbus

# Pythonライブラリインストール
echo "🐍 Pythonライブラリインストール..."
pip3 install --user pyserial numpy matplotlib adafruit-circuitpython-bno055

# I2C権限設定
echo "🔐 I2C権限設定..."
sudo usermod -a -G i2c $USER

# プロジェクトディレクトリ作成
echo "📁 プロジェクトディレクトリ作成..."
mkdir -p ~/minicar
mkdir -p ~/minicar/logs

# 設定ファイルコピー用のスクリプト作成
cat > ~/minicar/check_sensor.py << 'EOF'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055センサー接続確認スクリプト
"""
import time
import board
import busio
from adafruit_bno055 import BNO055_I2C

def check_i2c_devices():
    """I2Cデバイス確認"""
    import subprocess
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True)
        print("📡 I2Cデバイス検出結果:")
        print(result.stdout)
        return '28' in result.stdout or '29' in result.stdout
    except Exception as e:
        print(f"❌ I2C検出エラー: {e}")
        return False

def test_bno055():
    """BNO055接続テスト"""
    try:
        # I2C初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = BNO055_I2C(i2c)
        
        print("✅ BNO055接続成功")
        
        # 基本情報表示
        print(f"🔢 Chip ID: {hex(sensor.chip_id)}")
        print(f"🔢 Accelerometer ID: {hex(sensor.accelerometer_id)}")
        print(f"🔢 Magnetometer ID: {hex(sensor.magnetometer_id)}")
        print(f"🔢 Gyroscope ID: {hex(sensor.gyroscope_id)}")
        
        # センサーデータテスト
        for i in range(5):
            temp = sensor.temperature
            euler = sensor.euler
            accel = sensor.acceleration
            
            print(f"🌡️ Temperature: {temp}°C")
            if euler[0] is not None:
                print(f"🧭 Euler: {euler}")
            if accel[0] is not None:
                print(f"⚡ Accel: {accel}")
            
            time.sleep(1)
        
        return True
        
    except Exception as e:
        print(f"❌ BNO055接続エラー: {e}")
        return False

if __name__ == "__main__":
    print("🔍 BNO055センサー接続確認開始")
    
    # I2Cデバイス確認
    if check_i2c_devices():
        print("✅ I2Cデバイス検出成功")
    else:
        print("❌ I2Cデバイス検出失敗")
    
    # BNO055テスト
    if test_bno055():
        print("🎉 BNO055テスト完了！")
    else:
        print("💥 BNO055テスト失敗")
EOF

chmod +x ~/minicar/check_sensor.py

echo "🎯 セットアップ完了！"
echo ""
echo "次の手順:"
echo "1. sudo reboot でシステム再起動"
echo "2. ~/minicar/check_sensor.py でセンサー確認"
echo "3. PCからのデプロイを受け入れ"
echo ""
echo "⚠️  再起動が必要です: sudo reboot"