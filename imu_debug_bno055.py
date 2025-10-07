# imu_debug_bno055.py
# -*- coding: utf-8 -*-
"""
BNO055 IMU センサーデバッグ用サンプルコード
ラズベリーパイ4B + BNO055 センサー（シリアル接続）

機能:
- BNO055との安定したシリアル通信
- 生データ（加速度、ジャイロ、磁気）の取得
- センサーフュージョン結果（姿勢、オイラー角）の取得
- リアルタイム表示とデータロギング
- キャリブレーション状態監視
"""

import serial
import time
import json
import csv
import struct
from datetime import datetime
import threading
import os
import sys

class BNO055Sensor:
    """BNO055 センサー制御クラス"""
    
    # BNO055 レジスタアドレス
    REGISTERS = {
        'CHIP_ID': 0x00,
        'ACC_DATA_X_LSB': 0x08,
        'MAG_DATA_X_LSB': 0x0E,
        'GYR_DATA_X_LSB': 0x14,
        'EUL_DATA_X_LSB': 0x1A,
        'QUA_DATA_W_LSB': 0x20,
        'LIA_DATA_X_LSB': 0x28,
        'GRV_DATA_X_LSB': 0x2E,
        'TEMP': 0x34,
        'CALIB_STAT': 0x35,
        'OPR_MODE': 0x3D,
        'PWR_MODE': 0x3E,
        'SYS_TRIGGER': 0x3F,
        'UNIT_SEL': 0x3B
    }
    
    # 動作モード
    OPERATION_MODES = {
        'CONFIG': 0x00,
        'ACCONLY': 0x01,
        'MAGONLY': 0x02,
        'GYRONLY': 0x03,
        'ACCMAG': 0x04,
        'ACCGYRO': 0x05,
        'MAGGYRO': 0x06,
        'AMG': 0x07,
        'IMUPLUS': 0x08,
        'COMPASS': 0x09,
        'M4G': 0x0A,
        'NDOF_FMC_OFF': 0x0B,
        'NDOF': 0x0C
    }
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        """
        BNO055センサー初期化
        
        Args:
            port (str): シリアルポート（ラズパイの場合通常 /dev/serial0）
            baudrate (int): ボーレート
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        self.data_lock = threading.Lock()
        
        # 最新センサーデータ
        self.sensor_data = {
            'timestamp': 0.0,
            'raw': {
                'accelerometer': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'gyroscope': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'magnetometer': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'fusion': {
                'euler': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                'quaternion': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'gravity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'calibration': {
                'system': 0,
                'gyroscope': 0,
                'accelerometer': 0,
                'magnetometer': 0
            },
            'temperature': 0.0
        }
        
    def connect(self):
        """BNO055センサーに接続"""
        try:
            print(f"🔌 Connecting to BNO055 on {self.port} at {self.baudrate} baud...")
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            time.sleep(2)  # 接続待機
            
            # チップID確認
            chip_id = self.read_register(self.REGISTERS['CHIP_ID'])
            if chip_id == 0xA0:  # BNO055のチップID
                print(f"✅ BNO055 connected successfully (Chip ID: 0x{chip_id:02X})")
                self.is_connected = True
                return self.initialize_sensor()
            else:
                print(f"❌ Invalid chip ID: 0x{chip_id:02X} (expected 0xA0)")
                return False
                
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def initialize_sensor(self):
        """センサー初期化設定"""
        try:
            print("🔧 Initializing BNO055 sensor...")
            
            # コンフィグモードに設定
            self.write_register(self.REGISTERS['OPR_MODE'], self.OPERATION_MODES['CONFIG'])
            time.sleep(0.03)
            
            # システムリセット
            self.write_register(self.REGISTERS['SYS_TRIGGER'], 0x20)
            time.sleep(1.0)
            
            # 単位設定（度、m/s²、rps）
            self.write_register(self.REGISTERS['UNIT_SEL'], 0x01)
            time.sleep(0.03)
            
            # NDOF（9軸センサーフュージョン）モードに設定
            self.write_register(self.REGISTERS['OPR_MODE'], self.OPERATION_MODES['NDOF'])
            time.sleep(0.1)
            
            print("✅ BNO055 initialization completed")
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False
    
    def read_register(self, address):
        """レジスタから1バイト読み取り"""
        if not self.serial_conn:
            return None
        try:
            # 読み取りコマンド送信
            command = bytearray([0xAA, 0x01, address, 0x01])
            self.serial_conn.write(command)
            
            # レスポンス受信
            response = self.serial_conn.read(2)
            if len(response) >= 2 and response[0] == 0xBB:
                return response[1]
            return None
        except Exception as e:
            print(f"❌ Register read error: {e}")
            return None
    
    def write_register(self, address, value):
        """レジスタに1バイト書き込み"""
        if not self.serial_conn:
            return False
        try:
            # 書き込みコマンド送信
            command = bytearray([0xAA, 0x00, address, 0x01, value])
            self.serial_conn.write(command)
            
            # レスポンス確認
            response = self.serial_conn.read(2)
            return len(response) >= 2 and response[0] == 0xEE and response[1] == 0x01
        except Exception as e:
            print(f"❌ Register write error: {e}")
            return False
    
    def read_vector(self, address, count=3):
        """ベクトルデータ（複数バイト）読み取り"""
        if not self.serial_conn:
            return [0.0] * count
        try:
            # 読み取りコマンド送信
            byte_count = count * 2  # 16ビット値 × count
            command = bytearray([0xAA, 0x01, address, byte_count])
            self.serial_conn.write(command)
            
            # レスポンス受信
            response = self.serial_conn.read(byte_count + 1)
            if len(response) >= byte_count + 1 and response[0] == 0xBB:
                values = []
                for i in range(count):
                    lsb = response[1 + i*2]
                    msb = response[2 + i*2]
                    raw_value = (msb << 8) | lsb
                    
                    # 16ビット符号付き整数として解釈
                    if raw_value > 32767:
                        raw_value -= 65536
                    values.append(raw_value)
                return values
            return [0.0] * count
        except Exception as e:
            print(f"❌ Vector read error: {e}")
            return [0.0] * count
    
    def update_sensor_data(self):
        """全センサーデータ更新"""
        if not self.is_connected:
            return False
        
        try:
            with self.data_lock:
                timestamp = time.time()
                
                # 生データ読み取り
                acc_raw = self.read_vector(self.REGISTERS['ACC_DATA_X_LSB'])
                gyro_raw = self.read_vector(self.REGISTERS['GYR_DATA_X_LSB'])
                mag_raw = self.read_vector(self.REGISTERS['MAG_DATA_X_LSB'])
                
                # センサーフュージョンデータ読み取り
                euler_raw = self.read_vector(self.REGISTERS['EUL_DATA_X_LSB'])
                quat_raw = self.read_vector(self.REGISTERS['QUA_DATA_W_LSB'], 4)
                lin_acc_raw = self.read_vector(self.REGISTERS['LIA_DATA_X_LSB'])
                gravity_raw = self.read_vector(self.REGISTERS['GRV_DATA_X_LSB'])
                
                # キャリブレーション状態
                calib_status = self.read_register(self.REGISTERS['CALIB_STAT'])
                
                # 温度
                temp_raw = self.read_register(self.REGISTERS['TEMP'])
                
                # データ変換・格納
                self.sensor_data.update({
                    'timestamp': timestamp,
                    'raw': {
                        'accelerometer': {
                            'x': acc_raw[0] / 100.0,  # m/s²
                            'y': acc_raw[1] / 100.0,
                            'z': acc_raw[2] / 100.0
                        },
                        'gyroscope': {
                            'x': gyro_raw[0] / 16.0,  # rps
                            'y': gyro_raw[1] / 16.0,
                            'z': gyro_raw[2] / 16.0
                        },
                        'magnetometer': {
                            'x': mag_raw[0] / 16.0,  # μT
                            'y': mag_raw[1] / 16.0,
                            'z': mag_raw[2] / 16.0
                        }
                    },
                    'fusion': {
                        'euler': {
                            'roll': euler_raw[2] / 16.0,   # 度
                            'pitch': euler_raw[1] / 16.0,
                            'yaw': euler_raw[0] / 16.0
                        },
                        'quaternion': {
                            'w': quat_raw[0] / 16384.0,
                            'x': quat_raw[1] / 16384.0,
                            'y': quat_raw[2] / 16384.0,
                            'z': quat_raw[3] / 16384.0
                        },
                        'linear_acceleration': {
                            'x': lin_acc_raw[0] / 100.0,  # m/s²
                            'y': lin_acc_raw[1] / 100.0,
                            'z': lin_acc_raw[2] / 100.0
                        },
                        'gravity': {
                            'x': gravity_raw[0] / 100.0,  # m/s²
                            'y': gravity_raw[1] / 100.0,
                            'z': gravity_raw[2] / 100.0
                        }
                    },
                    'calibration': {
                        'system': (calib_status >> 6) & 0x03,
                        'gyroscope': (calib_status >> 4) & 0x03,
                        'accelerometer': (calib_status >> 2) & 0x03,
                        'magnetometer': calib_status & 0x03
                    },
                    'temperature': temp_raw if temp_raw else 0
                })
                
            return True
            
        except Exception as e:
            print(f"❌ Data update error: {e}")
            return False
    
    def get_sensor_data(self):
        """最新センサーデータ取得（スレッドセーフ）"""
        with self.data_lock:
            return self.sensor_data.copy()
    
    def disconnect(self):
        """センサー切断"""
        if self.serial_conn:
            self.serial_conn.close()
            self.is_connected = False
            print("🔌 BNO055 disconnected")

class IMUMonitor:
    """IMU データリアルタイム監視システム"""
    
    def __init__(self, sensor):
        self.sensor = sensor
        self.running = False
        self.display_mode = 'compact'  # 'compact' or 'detailed'
        self.log_enabled = False
        self.csv_file = None
        self.csv_writer = None
        
    def start_logging(self, filename=None):
        """データロギング開始"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"imu_data_{timestamp}.csv"
        
        try:
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            fieldnames = [
                'timestamp', 'dt',
                'acc_x', 'acc_y', 'acc_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'mag_x', 'mag_y', 'mag_z',
                'euler_roll', 'euler_pitch', 'euler_yaw',
                'quat_w', 'quat_x', 'quat_y', 'quat_z',
                'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
                'gravity_x', 'gravity_y', 'gravity_z',
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
    
    def log_data(self, data, prev_timestamp=0.0):
        """データをCSVに記録"""
        if not self.log_enabled or not self.csv_writer:
            return
        
        try:
            dt = data['timestamp'] - prev_timestamp if prev_timestamp > 0 else 0.0
            
            row = {
                'timestamp': f"{data['timestamp']:.6f}",
                'dt': f"{dt:.6f}",
                'acc_x': f"{data['raw']['accelerometer']['x']:.3f}",
                'acc_y': f"{data['raw']['accelerometer']['y']:.3f}",
                'acc_z': f"{data['raw']['accelerometer']['z']:.3f}",
                'gyro_x': f"{data['raw']['gyroscope']['x']:.3f}",
                'gyro_y': f"{data['raw']['gyroscope']['y']:.3f}",
                'gyro_z': f"{data['raw']['gyroscope']['z']:.3f}",
                'mag_x': f"{data['raw']['magnetometer']['x']:.1f}",
                'mag_y': f"{data['raw']['magnetometer']['y']:.1f}",
                'mag_z': f"{data['raw']['magnetometer']['z']:.1f}",
                'euler_roll': f"{data['fusion']['euler']['roll']:.2f}",
                'euler_pitch': f"{data['fusion']['euler']['pitch']:.2f}",
                'euler_yaw': f"{data['fusion']['euler']['yaw']:.2f}",
                'quat_w': f"{data['fusion']['quaternion']['w']:.4f}",
                'quat_x': f"{data['fusion']['quaternion']['x']:.4f}",
                'quat_y': f"{data['fusion']['quaternion']['y']:.4f}",
                'quat_z': f"{data['fusion']['quaternion']['z']:.4f}",
                'lin_acc_x': f"{data['fusion']['linear_acceleration']['x']:.3f}",
                'lin_acc_y': f"{data['fusion']['linear_acceleration']['y']:.3f}",
                'lin_acc_z': f"{data['fusion']['linear_acceleration']['z']:.3f}",
                'gravity_x': f"{data['fusion']['gravity']['x']:.3f}",
                'gravity_y': f"{data['fusion']['gravity']['y']:.3f}",
                'gravity_z': f"{data['fusion']['gravity']['z']:.3f}",
                'calib_sys': data['calibration']['system'],
                'calib_gyro': data['calibration']['gyroscope'],
                'calib_acc': data['calibration']['accelerometer'],
                'calib_mag': data['calibration']['magnetometer'],
                'temperature': data['temperature']
            }
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            print(f"❌ Logging error: {e}")
    
    def display_compact(self, data):
        """コンパクト表示（1行）"""
        calib = data['calibration']
        euler = data['fusion']['euler']
        acc = data['raw']['accelerometer']
        gyro = data['raw']['gyroscope']
        
        # キャリブレーション状態表示
        calib_status = f"S{calib['system']}G{calib['gyroscope']}A{calib['accelerometer']}M{calib['magnetometer']}"
        
        display_line = (
            f"🧭 IMU | "
            f"YAW:{euler['yaw']:7.2f}° | "
            f"PITCH:{euler['pitch']:+6.2f}° | "
            f"ROLL:{euler['roll']:+6.2f}° | "
            f"ACC:[{acc['x']:+6.2f},{acc['y']:+6.2f},{acc['z']:+6.2f}] | "
            f"GYRO:[{gyro['x']:+6.2f},{gyro['y']:+6.2f},{gyro['z']:+6.2f}] | "
            f"CAL:{calib_status} | "
            f"T:{data['temperature']:3d}°C"
        )
        
        print(f"\r{display_line}", end="", flush=True)
    
    def display_detailed(self, data):
        """詳細表示"""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("🧭 BNO055 IMU SENSOR DEBUG MONITOR")
        print("="*80)
        print(f"Timestamp: {datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S.%f')[:-3]}")
        print(f"Temperature: {data['temperature']}°C")
        print("="*80)
        
        # キャリブレーション状態
        calib = data['calibration']
        calib_status = ["❌ Poor", "🟡 Fair", "🟠 Good", "✅ Excellent"]
        print("🎯 CALIBRATION STATUS:")
        print(f"   System:        {calib['system']}/3 {calib_status[calib['system']]}")
        print(f"   Gyroscope:     {calib['gyroscope']}/3 {calib_status[calib['gyroscope']]}")
        print(f"   Accelerometer: {calib['accelerometer']}/3 {calib_status[calib['accelerometer']]}")
        print(f"   Magnetometer:  {calib['magnetometer']}/3 {calib_status[calib['magnetometer']]}")
        print()
        
        # 生データ
        raw = data['raw']
        print("📊 RAW SENSOR DATA:")
        print(f"   Accelerometer [m/s²]: X={raw['accelerometer']['x']:+7.3f} Y={raw['accelerometer']['y']:+7.3f} Z={raw['accelerometer']['z']:+7.3f}")
        print(f"   Gyroscope [rad/s]:    X={raw['gyroscope']['x']:+7.3f} Y={raw['gyroscope']['y']:+7.3f} Z={raw['gyroscope']['z']:+7.3f}")
        print(f"   Magnetometer [μT]:    X={raw['magnetometer']['x']:+7.1f} Y={raw['magnetometer']['y']:+7.1f} Z={raw['magnetometer']['z']:+7.1f}")
        print()
        
        # センサーフュージョン結果
        fusion = data['fusion']
        print("🔄 SENSOR FUSION RESULTS:")
        print(f"   Euler Angles [°]:     Roll={fusion['euler']['roll']:+7.2f} Pitch={fusion['euler']['pitch']:+7.2f} Yaw={fusion['euler']['yaw']:+7.2f}")
        print(f"   Quaternion:           W={fusion['quaternion']['w']:+7.4f} X={fusion['quaternion']['x']:+7.4f} Y={fusion['quaternion']['y']:+7.4f} Z={fusion['quaternion']['z']:+7.4f}")
        print(f"   Linear Accel [m/s²]:  X={fusion['linear_acceleration']['x']:+7.3f} Y={fusion['linear_acceleration']['y']:+7.3f} Z={fusion['linear_acceleration']['z']:+7.3f}")
        print(f"   Gravity [m/s²]:       X={fusion['gravity']['x']:+7.3f} Y={fusion['gravity']['y']:+7.3f} Z={fusion['gravity']['z']:+7.3f}")
        print("="*80)
        print("Commands: [c]ompact mode, [l]og toggle, [q]uit")
    
    def run_monitor(self):
        """監視ループ実行"""
        print("🚀 Starting IMU monitoring...")
        print("Commands: [d]etailed mode, [c]ompact mode, [l]og toggle, [q]uit")
        print("="*80)
        
        self.running = True
        prev_timestamp = 0.0
        
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
                    
                    # ログ記録
                    if self.log_enabled:
                        self.log_data(data, prev_timestamp)
                    
                    prev_timestamp = data['timestamp']
                
                # キー入力チェック（非ブロッキング）
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key_input(key)
                
                time.sleep(0.05)  # 20Hz更新
                
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
        if key.lower() == 'q':
            self.running = False
        elif key.lower() == 'c':
            self.display_mode = 'compact'
            print("\n📱 Switched to compact mode")
        elif key.lower() == 'd':
            self.display_mode = 'detailed'
            print("\n📋 Switched to detailed mode")
        elif key.lower() == 'l':
            if self.log_enabled:
                self.stop_logging()
            else:
                self.start_logging()

def main():
    """メイン実行関数"""
    print("🧭 BNO055 IMU Sensor Debug Tool")
    print("="*50)
    print("🔌 Hardware Setup:")
    print("   • Raspberry Pi 4B")
    print("   • BNO055 9-axis IMU sensor")
    print("   • Serial connection (UART)")
    print("="*50)
    
    # シリアルポート設定（環境に応じて変更）
    if os.name == 'nt':  # Windows
        port = 'COM3'  # Windowsの場合
    else:  # Linux/Raspberry Pi
        port = '/dev/serial0'  # ラズパイのGPIO UART
    
    print(f"📡 Connecting to BNO055 on {port}...")
    
    # センサー初期化
    sensor = BNO055Sensor(port=port)
    
    if not sensor.connect():
        print("❌ Failed to connect to BNO055 sensor")
        print("🔧 Troubleshooting:")
        print("   1. Check serial port connection")
        print("   2. Verify BNO055 power supply")
        print("   3. Confirm UART configuration")
        print("   4. Check port permissions (Linux)")
        return
    
    # モニタリング開始
    monitor = IMUMonitor(sensor)
    
    try:
        monitor.run_monitor()
    finally:
        sensor.disconnect()
        print("💤 IMU debug session ended")

if __name__ == "__main__":
    # Windowsの場合はselect不要な方法を使用
    try:
        import select
    except ImportError:
        # Windowsの場合の代替実装
        import msvcrt
        
        def mock_select_stdin():
            return msvcrt.kbhit()
        
        # selectの代替として使用
        sys.stdin.in_select = mock_select_stdin
    
    main()