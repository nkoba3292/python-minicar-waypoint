# imu_debug_simple.py

# --- SimpleVersion管理 ---
SIMPLE_VERSION = 1  # 修正のたびに+1する
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
MOCK_MODE = False  # Trueにすると模擬データで動作

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
    def connect(self):
        if MOCK_MODE:
            self.mock_sensor = MockBNO055()
            self.is_connected = self.mock_sensor.connect()
            return self.is_connected
        try:
            if not SERIAL_AVAILABLE:
                print("⚠️ pyserial not available. Cannot connect real sensor.")
                return False
            import serial
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"[DEBUG] serial.Serial config: {self.serial_conn}")
            self.is_connected = True
            print(f"🔌 BNO055 connected on {self.port} at {self.baudrate}bps")

            # --- BNO055初期化: 動作モード設定（NDOF） ---
            # 動作モード設定: 0x3Dレジスタに0x0C（NDOF）を書き込む
            try:
                # CONFIGモードへ
                self.serial_conn.write(bytes([0xAA, 0x00, 0x3D, 0x00]))
                time.sleep(0.05)
                resp1 = self.serial_conn.read(2)
                print(f"[DEBUG] CONFIG mode set resp: {list(resp1)}")
                # NDOFモードへ
                self.serial_conn.write(bytes([0xAA, 0x00, 0x3D, 0x0C]))
                time.sleep(0.05)
                resp2 = self.serial_conn.read(2)
                print(f"[DEBUG] NDOF mode set resp: {list(resp2)}")
                time.sleep(0.5)  # モード切替後の安定待機
                print("✅ BNO055 mode set to NDOF (waited 0.5s)")
            except Exception as e:
                print(f"⚠️ BNO055 mode set error: {e}")

            return True
        except Exception as e:
            print(f"❌ BNO055 connect error: {e}")
            self.is_connected = False
            return False
    """実際のBNO055センサー制御クラス（簡易版）"""
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False

    def update_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.update_sensor_data()
        try:
            import struct
            # BNO055のレジスタアドレス
            ACC_ADDR = 0x08  # 加速度
            GYR_ADDR = 0x14  # ジャイロ
            MAG_ADDR = 0x0E  # 地磁気
            EUL_ADDR = 0x1A  # オイラー角
            CALIB_ADDR = 0x35  # キャリブレーション
            TEMP_ADDR = 0x34  # 温度
            SYS_STATUS_ADDR = 0x39
            SYS_ERR_ADDR = 0x3A
            CHIP_ID_ADDR = 0x00

            def read_vector(addr, length):
                # BNO055のレジスタからlengthバイト読む
                cmd = bytes([0xAA, 0x01, addr, length])
                self.serial_conn.write(cmd)
                time.sleep(0.01)
                resp = self.serial_conn.read(2 + length)
                print(f"[DEBUG] read_vector addr=0x{addr:02X} cmd={list(cmd)} resp={list(resp)} hex={resp.hex()}")
                if len(resp) == 0:
                    print(f"[DEBUG] No response from BNO055")
                    return None
                if resp[0] == 0xEE:
                    print(f"[DEBUG] BNO055 error response: code=0x{resp[1]:02X}")
                    return None
                if len(resp) != 2 + length or resp[0] != 0xBB:
                    print(f"[DEBUG] Unexpected response format: {list(resp)}")
                    return None
                return resp[2:]

            # --- ステータス・エラー・IDレジスタの取得 ---
            chip_id = read_vector(CHIP_ID_ADDR, 1)
            print(f"[DEBUG] CHIP_ID: {chip_id}")
            sys_status = read_vector(SYS_STATUS_ADDR, 1)
            print(f"[DEBUG] SYS_STATUS: {sys_status}")
            sys_err = read_vector(SYS_ERR_ADDR, 1)
            print(f"[DEBUG] SYS_ERR: {sys_err}")

            # --- キャリブレーション状態の取得・表示 ---
            calib_raw = read_vector(CALIB_ADDR, 1)
            if calib_raw:
                calib_byte = calib_raw[0]
                calib = {
                    'system':   (calib_byte >> 6) & 0x03,
                    'gyroscope':(calib_byte >> 4) & 0x03,
                    'accelerometer':(calib_byte >> 2) & 0x03,
                    'magnetometer': calib_byte & 0x03
                }
                print(f"[DEBUG] CALIBRATION: raw=0x{calib_byte:02X} sys={calib['system']} gyro={calib['gyroscope']} acc={calib['accelerometer']} mag={calib['magnetometer']}")
            else:
                print("[DEBUG] CALIBRATION: read failed")

            # 加速度
            acc_raw = read_vector(ACC_ADDR, 6)
            if acc_raw:
                acc = struct.unpack('<hhh', acc_raw)
                acc = tuple([v/100.0 for v in acc])
            else:
                acc = (0.0, 0.0, 0.0)
            # ジャイロ
            gyr_raw = read_vector(GYR_ADDR, 6)
            if gyr_raw:
                gyr = struct.unpack('<hhh', gyr_raw)
                gyr = tuple([v/16.0 for v in gyr])
            else:
                gyr = (0.0, 0.0, 0.0)
            # 地磁気
            mag_raw = read_vector(MAG_ADDR, 6)
            if mag_raw:
                mag = struct.unpack('<hhh', mag_raw)
                mag = tuple([v/16.0 for v in mag])
            else:
                mag = (0.0, 0.0, 0.0)
            # オイラー角
            eul_raw = read_vector(EUL_ADDR, 6)
            if eul_raw:
                eul = struct.unpack('<hhh', eul_raw)
                eul = tuple([v/16.0 for v in eul])
            else:
                eul = (0.0, 0.0, 0.0)
            # キャリブレーション
            calib_raw = read_vector(CALIB_ADDR, 1)
            if calib_raw:
                calib_byte = calib_raw[0]
                calib = {
                    'system':   (calib_byte >> 6) & 0x03,
                    'gyroscope':(calib_byte >> 4) & 0x03,
                    'accelerometer':(calib_byte >> 2) & 0x03,
                    'magnetometer': calib_byte & 0x03
                }
            else:
                calib = {'system':0, 'gyroscope':0, 'accelerometer':0, 'magnetometer':0}
            # 温度
            temp_raw = read_vector(TEMP_ADDR, 1)
            if temp_raw:
                temp = int.from_bytes(temp_raw, 'little')
            else:
                temp = 25.0

            self.sensor_data = {
                'timestamp': time.time(),
                'raw': {
                    'accelerometer': {'x': acc[0], 'y': acc[1], 'z': acc[2]},
                    'gyroscope': {'x': gyr[0], 'y': gyr[1], 'z': gyr[2]},
                    'magnetometer': {'x': mag[0], 'y': mag[1], 'z': mag[2]}
                },
                'fusion': {
                    'euler': {'roll': eul[0], 'pitch': eul[1], 'yaw': eul[2]},
                    'quaternion': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'gravity': {'x': 0.0, 'y': 0.0, 'z': 9.8}
                },
                'calibration': calib,
                'temperature': temp
            }
            print(f"[DEBUG] update_sensor_data: {self.sensor_data}")
            return True
        except Exception as e:
            print(f"[DEBUG] update_sensor_data error: {e}")
            return False
    
    def get_sensor_data(self):
        if MOCK_MODE:
            return self.mock_sensor.get_sensor_data()
        print(f"[DEBUG] get_sensor_data: {self.sensor_data}")
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
        # --- 初期化直後にディレイ追加 ---
        time.sleep(1)
        try:
            while self.running:
                try:
                    # センサーデータ更新
                    if self.sensor.update_sensor_data():
                        data = self.sensor.get_sensor_data()
                        print(f"\n[DEBUG] sensor_data: {data}")  # データ内容を可視化
                        # 表示
                        if self.display_mode == 'compact':
                            self.display_compact(data)
                        else:
                            self.display_detailed(data)
                            time.sleep(1)  # 詳細モードは少し長く表示
                        # ログ記録
                        if self.log_enabled:
                            self.log_data(data)
                except Exception as e:
                    print(f"\n❌ Monitoring error (data acquisition): {e}")
                    # 必要ならここで再接続やリトライ処理も可
                    self.running = False
                    break
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
    global SIMPLE_VERSION
    print(f"🧭 BNO055 IMU Sensor Debug Tool (Simple Version)  [simpleversion={SIMPLE_VERSION}]")
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
    SIMPLE_VERSION += 1
    main()