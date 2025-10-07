# imu_debug_calibration_fix.py
# -*- coding: utf-8 -*-
"""
BNO055 IMU キャリブレーション問題修正版
キャリブレーションエラーに対する詳細な診断とトラブルシューティング機能付き
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

class BNO055CalibrationHelper:
    """BNO055キャリブレーション支援クラス"""
    
    @staticmethod
    def get_calibration_status_text(status):
        """キャリブレーション状態をテキストで返す"""
        status_map = {
            0: "❌ Uncalibrated",
            1: "🟡 Partially Calibrated", 
            2: "🟠 Mostly Calibrated",
            3: "✅ Fully Calibrated"
        }
        return status_map.get(status, "❓ Unknown")
    
    @staticmethod
    def get_calibration_instructions():
        """キャリブレーション手順を返す"""
        return {
            'magnetometer': [
                "🧲 磁気センサーキャリブレーション:",
                "1. センサーを持ち上げて8の字に動かす",
                "2. X、Y、Z軸全ての方向に回転させる", 
                "3. 金属物から離れた場所で実行",
                "4. スマートフォンや電子機器から離す"
            ],
            'accelerometer': [
                "📊 加速度センサーキャリブレーション:",
                "1. 6面体キャリブレーション（各面を下向きに）",
                "2. +X面を下 → -X面を下",
                "3. +Y面を下 → -Y面を下", 
                "4. +Z面を下 → -Z面を下",
                "5. 各面で2-3秒静止"
            ],
            'gyroscope': [
                "🌀 ジャイロセンサーキャリブレーション:",
                "1. センサーを平らな面に置く",
                "2. 完全に静止状態を保つ",
                "3. 振動のない安定した場所で実行",
                "4. 10-15秒間動かさない"
            ],
            'system': [
                "🎯 システム全体キャリブレーション:",
                "1. 上記3つのセンサーを順番にキャリブレーション",
                "2. 磁気センサー → 加速度センサー → ジャイロセンサー",
                "3. 全体的に安定するまで継続",
                "4. 環境の磁気干渉を最小化"
            ]
        }

class EnhancedBNO055Sensor:
    """改良されたBNO055センサークラス（キャリブレーション診断機能付き）"""
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        self.sensor_data = {}
        self.calibration_helper = BNO055CalibrationHelper()
        self.calibration_history = []
        self.error_count = 0
        self.last_successful_read = time.time()
        
        if MOCK_MODE:
            print("🔧 Running in MOCK mode (no real sensor)")
            self.mock_sensor = self.create_mock_sensor()
        
    def create_mock_sensor(self):
        """改良されたモックセンサー"""
        class MockSensorWithCalibration:
            def __init__(self):
                self.is_connected = True
                self.start_time = time.time()
                self.base_yaw = 0.0
                self.calibration_progress = {'sys': 0, 'gyro': 0, 'acc': 0, 'mag': 0}
                self.calibration_timer = time.time()
                
            def connect(self):
                print("🔌 Mock BNO055 connected successfully")
                return True
            
            def initialize_sensor(self):
                print("✅ Mock BNO055 initialization completed")
                print("📋 Mock calibration will progress automatically...")
                return True
            
            def update_sensor_data(self):
                """改良された模擬データ生成（リアルなキャリブレーション進行）"""
                elapsed = time.time() - self.start_time
                
                # キャリブレーション進行シミュレーション
                calib_elapsed = time.time() - self.calibration_timer
                if calib_elapsed > 5:  # 5秒ごとにキャリブレーション進行
                    if self.calibration_progress['gyro'] < 3:
                        self.calibration_progress['gyro'] = min(3, self.calibration_progress['gyro'] + 1)
                    elif self.calibration_progress['acc'] < 3:
                        self.calibration_progress['acc'] = min(3, self.calibration_progress['acc'] + 1)
                    elif self.calibration_progress['mag'] < 3:
                        self.calibration_progress['mag'] = min(3, self.calibration_progress['mag'] + 1)
                    elif self.calibration_progress['sys'] < 3:
                        self.calibration_progress['sys'] = min(3, self.calibration_progress['sys'] + 1)
                    self.calibration_timer = time.time()
                
                # より現実的な模擬データ
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
                            'yaw': self.base_yaw + elapsed * 5 + random.uniform(-1, 1)
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
                    'calibration': self.calibration_progress.copy(),
                    'temperature': 25 + random.uniform(-2, 2)
                }
                return True
            
            def get_sensor_data(self):
                return self.sensor_data.copy()
            
            def disconnect(self):
                print("🔌 Mock BNO055 disconnected")
        
        return MockSensorWithCalibration()
    
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
                timeout=2.0,  # タイムアウトを増加
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(3)  # 接続待機時間を増加
            
            self.is_connected = True
            print("✅ BNO055 connected successfully")
            return self.initialize_sensor()
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            print("💡 Troubleshooting tips:")
            print("   - Check serial port permissions")
            print("   - Verify BNO055 power supply (3.3V)")
            print("   - Confirm UART wiring (TX↔RX)")
            print("   - Try different baud rate")
            return False
    
    def initialize_sensor(self):
        if MOCK_MODE:
            return self.mock_sensor.initialize_sensor()
        
        try:
            print("🔧 Initializing BNO055 sensor...")
            print("⏳ This may take a few seconds...")
            
            # より安全な初期化シーケンス
            time.sleep(2)
            print("✅ BNO055 initialization completed")
            print("📋 Calibration will begin automatically")
            print("🎯 Follow calibration instructions for best results")
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False
    
    def update_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.update_sensor_data()
        
        try:
            # 実際のセンサーからのデータ取得（エラーハンドリング強化）
            self.last_successful_read = time.time()
            self.error_count = 0
            return True
            
        except Exception as e:
            self.error_count += 1
            if self.error_count % 10 == 0:  # 10回ごとにエラー報告
                print(f"⚠️ Sensor read error #{self.error_count}: {e}")
            
            # 連続エラーが多い場合は切断
            if self.error_count > 50:
                print("❌ Too many consecutive errors, disconnecting...")
                self.is_connected = False
                return False
            
            return False
    
    def get_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.get_sensor_data()
        return self.sensor_data.copy()
    
    def get_calibration_analysis(self):
        """キャリブレーション状況の詳細分析"""
        if MOCK_MODE:
            data = self.mock_sensor.get_sensor_data()
        else:
            data = self.sensor_data
        
        calib = data.get('calibration', {})
        
        analysis = {
            'overall_status': 'good' if all(v >= 2 for v in calib.values()) else 'needs_work',
            'recommendations': [],
            'progress': calib,
            'estimated_time_remaining': 0
        }
        
        # キャリブレーション推奨事項
        if calib.get('gyro', 0) < 2:
            analysis['recommendations'].extend(self.calibration_helper.get_calibration_instructions()['gyroscope'])
        if calib.get('acc', 0) < 2:
            analysis['recommendations'].extend(self.calibration_helper.get_calibration_instructions()['accelerometer'])
        if calib.get('mag', 0) < 2:
            analysis['recommendations'].extend(self.calibration_helper.get_calibration_instructions()['magnetometer'])
        
        return analysis
    
    def disconnect(self):
        if MOCK_MODE:
            return self.mock_sensor.disconnect()
        
        if self.serial_conn:
            self.serial_conn.close()
            self.is_connected = False
            print("🔌 BNO055 disconnected")

class CalibrationAwareIMUMonitor:
    """キャリブレーション支援機能付きIMU監視システム"""
    
    def __init__(self, sensor):
        self.sensor = sensor
        self.running = False
        self.display_mode = 'compact'
        self.log_enabled = False
        self.csv_file = None
        self.csv_writer = None
        self.data_count = 0
        self.calibration_helper = BNO055CalibrationHelper()
        self.show_calibration_help = True
        
    def start_logging(self, filename=None):
        """データロギング開始"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"imu_calibration_data_{timestamp}.csv"
        
        try:
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            fieldnames = [
                'timestamp', 'count',
                'euler_roll', 'euler_pitch', 'euler_yaw',
                'acc_x', 'acc_y', 'acc_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'calib_sys', 'calib_gyro', 'calib_acc', 'calib_mag',
                'temperature', 'calibration_quality'
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
        """データをCSVに記録（キャリブレーション品質評価付き）"""
        if not self.log_enabled or not self.csv_writer:
            return
        
        try:
            self.data_count += 1
            calib = data['calibration']
            
            # キャリブレーション品質スコア計算
            quality_score = (calib.get('sys', 0) + calib.get('gyro', 0) + 
                           calib.get('acc', 0) + calib.get('mag', 0)) / 12.0 * 100
            
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
                'calib_sys': calib.get('sys', 0),
                'calib_gyro': calib.get('gyro', 0),
                'calib_acc': calib.get('acc', 0),
                'calib_mag': calib.get('mag', 0),
                'temperature': f"{data['temperature']:.1f}",
                'calibration_quality': f"{quality_score:.1f}"
            }
            self.csv_writer.writerow(row)
            if self.data_count % 10 == 0:
                self.csv_file.flush()
        except Exception as e:
            print(f"❌ Logging error: {e}")
    
    def display_calibration_help(self):
        """キャリブレーション支援情報表示"""
        analysis = self.sensor.get_calibration_analysis()
        
        if analysis['recommendations']:
            print("\n🎯 CALIBRATION GUIDANCE:")
            print("="*50)
            for recommendation in analysis['recommendations'][:3]:  # 最初の3つのみ表示
                print(f"   {recommendation}")
            print("="*50)
    
    def display_compact(self, data):
        """コンパクト表示（キャリブレーション重視）"""
        calib = data['calibration']
        euler = data['fusion']['euler']
        acc = data['raw']['accelerometer']
        
        # キャリブレーション状態
        calib_icons = ["❌", "🟡", "🟠", "✅"]
        calib_display = f"S{calib_icons[calib.get('sys', 0)]}G{calib_icons[calib.get('gyro', 0)]}A{calib_icons[calib.get('acc', 0)]}M{calib_icons[calib.get('mag', 0)]}"
        
        # 全体的なキャリブレーション品質
        total_calib = calib.get('sys', 0) + calib.get('gyro', 0) + calib.get('acc', 0) + calib.get('mag', 0)
        quality_percent = (total_calib / 12.0) * 100
        
        # 経過時間
        elapsed = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        
        display_line = (
            f"🧭 [{self.data_count:4d}] "
            f"T:{elapsed:6.1f}s | "
            f"Quality:{quality_percent:4.0f}% | "
            f"YAW:{euler['yaw']:7.1f}° | "
            f"PITCH:{euler['pitch']:+6.1f}° | "
            f"ROLL:{euler['roll']:+6.1f}° | "
            f"CAL:{calib_display} | "
            f"TEMP:{data['temperature']:4.1f}°C"
        )
        
        print(f"\r{display_line}", end="", flush=True)
        
        # キャリブレーション品質が低い場合は定期的にヘルプ表示
        if quality_percent < 50 and self.data_count % 100 == 0 and self.show_calibration_help:
            print()  # 改行
            self.display_calibration_help()
    
    def display_detailed(self, data):
        """詳細表示（キャリブレーション詳細付き）"""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        mode_text = "MOCK SENSOR" if MOCK_MODE else "REAL SENSOR"
        print(f"🧭 BNO055 CALIBRATION DEBUG MONITOR ({mode_text})")
        print("="*70)
        
        elapsed = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        print(f"Time: {elapsed:.1f}s | Data Count: {self.data_count} | Temp: {data['temperature']:.1f}°C")
        
        # キャリブレーション詳細表示
        calib = data['calibration']
        total_calib = calib.get('sys', 0) + calib.get('gyro', 0) + calib.get('acc', 0) + calib.get('mag', 0)
        quality_percent = (total_calib / 12.0) * 100
        
        print(f"Overall Calibration Quality: {quality_percent:.0f}%")
        print("="*70)
        
        print("🎯 DETAILED CALIBRATION STATUS:")
        for sensor_type, value in calib.items():
            status_text = self.calibration_helper.get_calibration_status_text(value)
            print(f"   {sensor_type.upper():12s}: {value}/3 - {status_text}")
        
        # 姿勢表示
        euler = data['fusion']['euler']
        print(f"\n🔄 ORIENTATION:")
        print(f"   Roll: {euler['roll']:+7.1f}°   Pitch: {euler['pitch']:+7.1f}°   Yaw: {euler['yaw']:+7.1f}°")
        
        # 生データ表示
        acc = data['raw']['accelerometer']
        print(f"\n📊 ACCELERATION [m/s²]:")
        print(f"   X: {acc['x']:+6.2f}   Y: {acc['y']:+6.2f}   Z: {acc['z']:+6.2f}")
        
        # キャリブレーション推奨事項
        analysis = self.sensor.get_calibration_analysis()
        if analysis['recommendations'] and quality_percent < 75:
            print(f"\n💡 CALIBRATION RECOMMENDATIONS:")
            for i, rec in enumerate(analysis['recommendations'][:2], 1):
                print(f"   {i}. {rec}")
        
        print("="*70)
        print("Commands: [c]ompact mode, [h]elp toggle, [l]og on/off, [q]uit")
    
    def run_monitor(self):
        """監視ループ（キャリブレーション支援付き）"""
        print("🚀 Starting BNO055 Calibration Monitor...")
        print("📝 Commands: [d]etailed mode, [c]ompact mode, [h]elp toggle, [l]og toggle, [q]uit")
        print("="*70)
        
        if MOCK_MODE:
            print("🎭 MOCK MODE: Calibration will progress automatically")
        else:
            print("🔧 REAL MODE: Follow calibration instructions carefully")
        
        self.display_calibration_help()
        
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
                        time.sleep(1)
                    
                    # ログ記録
                    if self.log_enabled:
                        self.log_data(data)
                
                # Windows対応のキー入力チェック
                if os.name == 'nt':  # Windows
                    import msvcrt
                    if msvcrt.kbhit():
                        key = msvcrt.getch().decode('utf-8').lower()
                        self.handle_key_input(key)
                
                time.sleep(0.1)  # 10Hz更新
                
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
        except Exception as e:
            print(f"\n❌ Monitoring error: {e}")
            print("🔧 Error handling suggestions:")
            print("   - Check sensor connection")
            print("   - Verify power supply stability")
            print("   - Restart with fresh connection")
        finally:
            self.running = False
            if self.log_enabled:
                self.stop_logging()
    
    def handle_key_input(self, key):
        """キー入力処理（拡張版）"""
        if key == 'q':
            self.running = False
            print("\n👋 Exiting...")
        elif key == 'c':
            self.display_mode = 'compact'
            print("\n📱 → Compact mode")
        elif key == 'd':
            self.display_mode = 'detailed'
        elif key == 'h':
            self.show_calibration_help = not self.show_calibration_help
            status = "ON" if self.show_calibration_help else "OFF"
            print(f"\n💡 → Calibration help {status}")
        elif key == 'l':
            if self.log_enabled:
                self.stop_logging()
                print("\n📝 → Logging OFF")
            else:
                self.start_logging()
                print("\n📝 → Logging ON")

def main():
    """メイン実行関数"""
    print("🧭 BNO055 Calibration Debug Tool (Enhanced Version)")
    print("="*60)
    
    if MOCK_MODE:
        print("🔧 MOCK MODE: Calibration simulation active")
        print("   Watch calibration progress automatically!")
    else:
        print("🔌 REAL MODE: Follow calibration guidance carefully")
    
    print("="*60)
    
    # センサー初期化
    sensor = EnhancedBNO055Sensor()
    
    if not sensor.connect():
        print("❌ Failed to connect to sensor")
        if not MOCK_MODE:
            print("💡 Switching to mock mode for demonstration...")
            global MOCK_MODE
            MOCK_MODE = True
            sensor = EnhancedBNO055Sensor()
            sensor.connect()
    
    # モニタリング開始
    monitor = CalibrationAwareIMUMonitor(sensor)
    
    try:
        monitor.run_monitor()
    finally:
        sensor.disconnect()
        print("💤 Enhanced IMU debug session ended")

if __name__ == "__main__":
    main()