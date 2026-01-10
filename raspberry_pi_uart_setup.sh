#!/bin/bash
# raspberry_pi_uart_setup.sh
# Raspberry Pi UART設定スクリプト

echo "🍓 Raspberry Pi UART設定開始"

# UART有効化設定
echo "📡 UART設定..."

# /boot/config.txt に追加
sudo tee -a /boot/config.txt << 'EOF'

# BNO055 UART設定
enable_uart=1
dtoverlay=disable-bt
core_freq=250
EOF

# /boot/cmdline.txt からconsole削除
echo "🔧 シリアルコンソール無効化..."
sudo sed -i 's/console=serial0,115200 //' /boot/cmdline.txt
sudo sed -i 's/console=ttyAMA0,115200 //' /boot/cmdline.txt

# 必要なライブラリインストール
echo "🐍 Pythonライブラリインストール..."
pip3 install --user pyserial numpy matplotlib

# デバイス権限設定
echo "🔐 シリアルポート権限設定..."
sudo usermod -a -G dialout $USER

# プロジェクトディレクトリ作成
mkdir -p ~/minicar
mkdir -p ~/minicar/logs

# UART接続テストスクリプト作成
cat > ~/minicar/uart_test.py << 'EOF'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 UART接続テストスクリプト
"""
import serial
import time
import struct

def test_uart_connection():
    """UART接続テスト"""
    ports = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']
    
    for port in ports:
        try:
            print(f"🔍 Testing port: {port}")
            
            # シリアルポート開く
            ser = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            print(f"✅ Port {port} opened successfully")
            
            # BNO055 Chip ID読み取りテスト (0x00レジスタ)
            # Write: 0xAA 0x00 0x01 (Read 1 byte from register 0x00)
            ser.write(b'\xAA\x00\x01')
            time.sleep(0.1)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"📡 Response: {response.hex()}")
                
                # BNO055のChip IDは0xA0のはず
                if len(response) >= 2 and response[1] == 0xA0:
                    print("🎉 BNO055 detected!")
                    
                    # 基本情報読み取り
                    test_basic_communication(ser)
                    
                    ser.close()
                    return port
            
            ser.close()
            
        except Exception as e:
            print(f"❌ Error with port {port}: {e}")
    
    return None

def test_basic_communication(ser):
    """基本通信テスト"""
    try:
        # Temperature読み取り (0x34レジスタ)
        ser.write(b'\xAA\x34\x01')
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            temp_data = ser.read(ser.in_waiting)
            if len(temp_data) >= 2:
                temp = temp_data[1]  # 温度データ
                print(f"🌡️ Temperature: {temp}°C")
        
        # システムステータス読み取り (0x39レジスタ)
        ser.write(b'\xAA\x39\x01')
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            status_data = ser.read(ser.in_waiting)
            if len(status_data) >= 2:
                status = status_data[1]
                print(f"📊 System Status: 0x{status:02X}")
                
                # ステータス解析
                system_error = (status >> 0) & 0x0F
                system_status = (status >> 4) & 0x0F
                
                print(f"   System Error: {system_error}")
                print(f"   System Status: {system_status}")
        
    except Exception as e:
        print(f"❌ Communication test error: {e}")

if __name__ == "__main__":
    print("🔍 BNO055 UART接続テスト開始")
    
    working_port = test_uart_connection()
    
    if working_port:
        print(f"✅ BNO055接続成功: {working_port}")
        print("🎯 UART設定完了！")
    else:
        print("❌ BNO055接続失敗")
        print("📋 確認事項:")
        print("  1. 配線確認 (3.3V電源、正しいTX/RX)")
        print("  2. PS0=3.3V, PS1=GND (UART mode)")
        print("  3. sudo reboot 実行済み")
EOF

chmod +x ~/minicar/uart_test.py

echo "⚡ UART設定完了！"
echo ""
echo "🔌 配線確認:"
echo "  BNO055 VCC  → Pi Pin 1 (3.3V)"
echo "  BNO055 GND  → Pi Pin 6 (GND)"
echo "  BNO055 TX   → Pi Pin 10 (GPIO 15/RX)"
echo "  BNO055 RX   → Pi Pin 8  (GPIO 14/TX)"
echo "  BNO055 PS0  → 3.3V (UART mode)"
echo "  BNO055 PS1  → GND  (UART mode)"
echo ""
echo "次の手順:"
echo "1. 💡 配線確認 (3.3V電源重要!)"
echo "2. 🔄 sudo reboot"
echo "3. 🧪 ~/minicar/uart_test.py でテスト"
echo ""
echo "⚠️  再起動が必要です: sudo reboot"
EOF

chmod +x raspberry_pi_uart_setup.sh

echo "🎯 UART設定スクリプト作成完了！"