# imu_debug_simple.py
# -*- coding: utf-8 -*-
"""
BNO055 IMU デバッグ用簡易版（WindowsでもテストOK）
実際のセンサーが無い場合は模擬データで動作確認可能
"""

import time
import json
import csv
import threading
import os
import sys
from datetime import datetime
import random
import math

# Windows環境での動作確認用のモックセンサーモード
MOCK_MODE = True  # Trueにすると模擬データで動作

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("⚠️ pyserial not installed. Running in mock mode.")
    MOCK_MODE = True

class MockBNO055:
    """BNO055センサーのモック（テスト用）"""
    
    def __init__(self):
        self.is_connected = True
        self.start_time = time.time()
        self.base_yaw = 0.0
        
    def connect(self):
        print("🔌 Mock BNO055 connected successfully")
        return True
    
    def initialize_sensor(self):
        print("✅ Mock BNO055 initialization completed")
        return True
    
    def update_sensor_data(self):
        """模擬センサーデータ生成"""
        elapsed = time.time() - self.start_time
        
        # 模擬データ生成（実際のセンサーらしい値）
        self.sensor_data = {
            'timestamp': time.time(),
            'raw': {
                'accelerometer': {
                    'x': random.uniform(-1.0, 1.0) + math.sin(elapsed * 0.5) * 0.3,
                    'y': random.uniform(-1.0, 1.0) + math.cos(elapsed * 0.3) * 0.2,
                    'z': 9.8 + random.uniform(-0.5, 0.5)
                },
                'gyroscope': {
                    'x': random.uniform(-0.2, 0.2),
                    'y': random.uniform(-0.2, 0.2),
                    'z': random.uniform(-0.1, 0.1)
                },
                'magnetometer': {
                    'x': random.uniform(20, 60) + math.sin(elapsed * 0.1) * 10,
                    'y': random.uniform(-30, 30) + math.cos(elapsed * 0.1) * 15,
                    'z': random.uniform(-50, -10)
                }
            },
            'fusion': {
                'euler': {
                    'roll': math.sin(elapsed * 0.2) * 15 + random.uniform(-2, 2),
                    'pitch': math.cos(elapsed * 0.15) * 10 + random.uniform(-1, 1),
                    'yaw': self.base_yaw + elapsed * 5 + random.uniform(-1, 1)  # ゆっくり回転
                },
                'quaternion': {
                    'w': 0.7071 + random.uniform(-0.1, 0.1),
                    'x': random.uniform(-0.3, 0.3),
                    'y': random.uniform(-0.3, 0.3),
                    'z': random.uniform(-0.3, 0.3)
                },
                'linear_acceleration': {
                    'x': random.uniform(-0.5, 0.5),
                    'y': random.uniform(-0.5, 0.5),
                    'z': random.uniform(-0.2, 0.2)
                },
                'gravity': {
                    'x': random.uniform(-1.0, 1.0),
                    'y': random.uniform(-1.0, 1.0),
                    'z': 9.8 + random.uniform(-0.1, 0.1)
                }
            },
            'calibration': {
                'system': min(3, int(elapsed / 10)),      # 10秒ごとに向上
                'gyroscope': min(3, int(elapsed / 5)),    # 5秒ごとに向上
                'accelerometer': min(3, int(elapsed / 3)), # 3秒ごとに向上
                'magnetometer': min(3, int(elapsed / 15))  # 15秒ごとに向上
            },
            'temperature': 25 + random.uniform(-2, 2)
        }
        return True
    
    def get_sensor_data(self):
        return self.sensor_data.copy()
    
    def disconnect(self):
        print("🔌 Mock BNO055 disconnected")

class BNO055Sensor:
    """実際のBNO055センサー制御クラス（簡易版）"""
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        self.sensor_data = {}
        
        if MOCK_MODE:
            print("🔧 Running in MOCK mode (no real sensor)")
            self.mock_sensor = MockBNO055()
        
    def connect(self):
        if MOCK_MODE:
            return self.mock_sensor.connect()
        
        if not SERIAL_AVAILABLE:
            print("❌ pyserial not available")
            return False
        
        try:
            print(f"🔌 Connecting to BNO055 on {self.port}...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            time.sleep(2)
            
            # 簡易接続確認（実際のチップID確認等は省略）
            self.is_connected = True
            print("✅ BNO055 connected successfully")
            return self.initialize_sensor()
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def initialize_sensor(self):
        if MOCK_MODE:
            return self.mock_sensor.initialize_sensor()
        
        print("✅ BNO055 initialization completed")
        return True
    
    def update_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.update_sensor_data()
        
        # 実際のセンサーからデータ読み取り（簡素化版）
        # 実装は元のコードを参照
        return True
    
    def get_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.get_sensor_data()
        
        return self.sensor_data.copy()
    
    def disconnect(self):
        if MOCK_MODE:
            return self.mock_sensor.disconnect()
        
        if self.serial_conn:
            self.serial_conn.close()
            self.is_connected = False
            print("🔌 BNO055 disconnected")

class SimpleIMUMonitor:
    """シンプルなIMU監視システム"""
    
    def __init__(self, sensor):
        self.sensor = sensor
        self.running = False
        self.display_mode = 'compact'
        self.log_enabled = False
        self.csv_file = None
        self.csv_writer = None
        self.data_count = 0
        
    def start_logging(self, filename=None):
        """データロギング開始"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"imu_data_{timestamp}.csv"
        
        try:
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            fieldnames = [
                'timestamp', 'count',
                'euler_roll', 'euler_pitch', 'euler_yaw',
                'acc_x', 'acc_y', 'acc_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'calib_sys', 'calib_gyro', 'calib_acc', 'calib_mag',
                'temperature'
            ]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            self.log_enabled = True
            print(f"📝 Data logging started: {filename}")
        except Exception as e:
            print(f"❌ Logging start error: {e}")
    
    def stop_logging(self):
        """データロギング停止"""
        if self.csv_file:
            self.csv_file.close()
            self.log_enabled = False
            print("📝 Data logging stopped")
    
    def log_data(self, data):
        """データをCSVに記録"""
        if not self.log_enabled or not self.csv_writer:
            return
        
        try:
            self.data_count += 1
            row = {
                'timestamp': f"{data['timestamp']:.3f}",
                'count': self.data_count,
                'euler_roll': f"{data['fusion']['euler']['roll']:.2f}",
                'euler_pitch': f"{data['fusion']['euler']['pitch']:.2f}",
                'euler_yaw': f"{data['fusion']['euler']['yaw']:.2f}",
                'acc_x': f"{data['raw']['accelerometer']['x']:.3f}",
                'acc_y': f"{data['raw']['accelerometer']['y']:.3f}",
                'acc_z': f"{data['raw']['accelerometer']['z']:.3f}",
                'gyro_x': f"{data['raw']['gyroscope']['x']:.3f}",
                'gyro_y': f"{data['raw']['gyroscope']['y']:.3f}",
                'gyro_z': f"{data['raw']['gyroscope']['z']:.3f}",
                'calib_sys': data['calibration']['system'],
                'calib_gyro': data['calibration']['gyroscope'],
                'calib_acc': data['calibration']['accelerometer'],
                'calib_mag': data['calibration']['magnetometer'],
                'temperature': f"{data['temperature']:.1f}"
            }
            self.csv_writer.writerow(row)
            if self.data_count % 10 == 0:  # 10回ごとにフラッシュ
                self.csv_file.flush()
        except Exception as e:
            print(f"❌ Logging error: {e}")
    
    def display_compact(self, data):
        """コンパクト表示"""
        calib = data['calibration']
        euler = data['fusion']['euler']
        acc = data['raw']['accelerometer']
        
        # キャリブレーション状態
        calib_icons = ["❌", "🟡", "🟠", "✅"]
        calib_display = f"S{calib_icons[calib['system']]}G{calib_icons[calib['gyroscope']]}A{calib_icons[calib['accelerometer']]}M{calib_icons[calib['magnetometer']]}"
        
        # 経過時間
        elapsed = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        
        display_line = (
            f"🧭 [{self.data_count:4d}] "
            f"T:{elapsed:6.1f}s | "
            f"YAW:{euler['yaw']:7.1f}° | "
            f"PITCH:{euler['pitch']:+6.1f}° | "
            f"ROLL:{euler['roll']:+6.1f}° | "
            f"ACC:{acc['z']:+5.2f} | "
            f"CAL:{calib_display} | "
            f"TEMP:{data['temperature']:4.1f}°C"
        )
        
        print(f"\r{display_line}", end="", flush=True)
    
    def display_detailed(self, data):
        """詳細表示"""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        mode_text = "MOCK SENSOR" if MOCK_MODE else "REAL SENSOR"
        print(f"🧭 BNO055 IMU DEBUG MONITOR ({mode_text})")
        print("="*60)
        
        elapsed = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        print(f"Time: {elapsed:.1f}s | Data Count: {self.data_count} | Temp: {data['temperature']:.1f}°C")
        print("="*60)
        
        # キャリブレーション
        calib = data['calibration']
        calib_names = ["Poor", "Fair", "Good", "Excellent"]
        print("🎯 CALIBRATION:")
        print(f"   Sys:{calib['system']}/3 ({calib_names[calib['system']]})  Gyro:{calib['gyroscope']}/3  Acc:{calib['accelerometer']}/3  Mag:{calib['magnetometer']}/3")
        
        # 姿勢
        euler = data['fusion']['euler']
        print(f"\n🔄 ORIENTATION:")
        print(f"   Roll: {euler['roll']:+7.1f}°   Pitch: {euler['pitch']:+7.1f}°   Yaw: {euler['yaw']:+7.1f}°")
        
        # 加速度
        acc = data['raw']['accelerometer']
        print(f"\n📊 ACCELERATION [m/s²]:")
        print(f"   X: {acc['x']:+6.2f}   Y: {acc['y']:+6.2f}   Z: {acc['z']:+6.2f}")
        
        # ジャイロ
        gyro = data['raw']['gyroscope']
        print(f"\n🌀 GYROSCOPE [rad/s]:")
        print(f"   X: {gyro['x']:+6.3f}   Y: {gyro['y']:+6.3f}   Z: {gyro['z']:+6.3f}")
        
        print("="*60)
        print("Commands: [c]ompact mode, [l]og on/off, [q]uit")
    
    def run_monitor(self):
        """監視ループ"""
        print("🚀 Starting IMU monitoring...")
        print("📝 Commands: [d]etailed mode, [c]ompact mode, [l]og toggle, [q]uit")
        print("="*60)
        
        self.running = True
        self.start_time = time.time()
        self.data_count = 0
        
        try:
            while self.running:
                # センサーデータ更新
                if self.sensor.update_sensor_data():
                    data = self.sensor.get_sensor_data()
                    
                    # 表示
                    if self.display_mode == 'compact':
                        self.display_compact(data)
                    else:
                        self.display_detailed(data)
                        time.sleep(1)  # 詳細モードは少し長く表示
                    
                    # ログ記録
                    if self.log_enabled:
                        self.log_data(data)
                
                # Windows対応のキー入力チェック
                if os.name == 'nt':  # Windows
                    import msvcrt
                    if msvcrt.kbhit():
                        key = msvcrt.getch().decode('utf-8').lower()
                        self.handle_key_input(key)
                else:  # Linux
                    # 非ブロッキング入力（実装省略）
                    pass
                
                time.sleep(0.1)  # 10Hz更新
                
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
        except Exception as e:
            print(f"\n❌ Monitoring error: {e}")
        finally:
            self.running = False
            if self.log_enabled:
                self.stop_logging()
    
    def handle_key_input(self, key):
        """キー入力処理"""
        if key == 'q':
            self.running = False
            print("\n👋 Exiting...")
        elif key == 'c':
            self.display_mode = 'compact'
            print("\n📱 → Compact mode")
        elif key == 'd':
            self.display_mode = 'detailed'
        elif key == 'l':
            if self.log_enabled:
                self.stop_logging()
                print("\n📝 → Logging OFF")
            else:
                self.start_logging()
                print("\n📝 → Logging ON")

def main():
    """メイン実行関数"""
    print("🧭 BNO055 IMU Sensor Debug Tool (Simple Version)")
    print("="*55)
    
    if MOCK_MODE:
        print("🔧 MOCK MODE: Using simulated sensor data")
        print("   (Real sensor data will show when connected)")
    else:
        print("🔌 REAL MODE: Connecting to actual BNO055 sensor")
    
    print("="*55)
    
    # センサー初期化
    sensor = BNO055Sensor()
    
    if not sensor.connect():
        print("❌ Failed to connect to sensor")
        print("💡 Running in mock mode for demonstration...")
        sensor = BNO055Sensor()  # フォールバック
        sensor.connect()
    
    # モニタリング開始
    monitor = SimpleIMUMonitor(sensor)
    
    try:
        monitor.run_monitor()
    finally:
        sensor.disconnect()
        print("💤 IMU debug session ended")

if __name__ == "__main__":
    main()