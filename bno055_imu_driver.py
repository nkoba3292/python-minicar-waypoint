# bno055_imu_driver.py - BNO055 IMUセンサー制御
import json
import math
import time
from platform_detector import is_raspberry_pi

# Raspberry Pi環境でのみライブラリをインポート
if is_raspberry_pi():
    try:
        import board
        import busio
        import adafruit_bno055
    except ImportError as e:
        print(f"BNO055 libraries not available: {e}")
        board = busio = adafruit_bno055 = None
else:
    board = busio = adafruit_bno055 = None

class BNO055IMUDriver:
    def __init__(self, config_file="config.json"):
        # 設定読み込み
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            self.imu_config = config.get('sensors', {}).get('imu', {})
        except Exception as e:
            print(f"Config file error: {e}")
            self.imu_config = {}
        
        # 設定値
        self.i2c_address = int(self.imu_config.get('i2c_address', '0x28'), 16)
        self.auto_calibration = self.imu_config.get('auto_calibration', True)
        
        # BNO055インスタンス
        self.sensor = None
        
        # キャリブレーション状態
        self.calibration_status = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        
        if is_raspberry_pi() and board is not None:
            self._init_hardware()
        else:
            print("Running in mock mode - BNO055 not initialized")
    
    def _init_hardware(self):
        """BNO055ハードウェアの初期化"""
        try:
            # I2C初期化
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)
            
            # センサーモード設定（NDOF = 9DOF融合）
            self.sensor.mode = adafruit_bno055.NDOF_MODE
            
            print("BNO055 initialized successfully")
            print(f"Sensor ID: {self.sensor.sensor_id}")
            print(f"Software version: {self.sensor.sw_revision}")
            print(f"Bootloader version: {self.sensor.bootloader_revision}")
            
            # 自動キャリブレーション開始
            if self.auto_calibration:
                self._start_calibration()
                
        except Exception as e:
            print(f"BNO055 initialization failed: {e}")
            self.sensor = None
    
    def _start_calibration(self):
        """自動キャリブレーション開始"""
        print("Starting BNO055 calibration...")
        print("Please move the sensor in figure-8 pattern for magnetometer calibration")
        
        start_time = time.time()
        while time.time() - start_time < 30:  # 30秒間キャリブレーション
            status = self.get_calibration_status()
            if all(status[key] >= 2 for key in ['sys', 'gyro', 'accel', 'mag']):
                print("✓ Calibration completed successfully!")
                break
            time.sleep(1)
        else:
            print("⚠ Calibration timeout, continuing with current calibration")
    
    def get_yaw(self):
        """ヨー角を取得（ラジアン）"""
        if self.sensor is not None:
            try:
                euler = self.sensor.euler
                if euler is not None:
                    # BNO055のオイラー角は度単位、ラジアンに変換
                    yaw_degrees = euler[0] if euler[0] is not None else 0.0
                    return math.radians(yaw_degrees)
            except Exception as e:
                print(f"Yaw reading error: {e}")
        
        # モックモードまたはエラー時
        return 0.0
    
    def get_euler_angles(self):
        """オイラー角を取得（度）"""
        if self.sensor is not None:
            try:
                euler = self.sensor.euler
                return {
                    'yaw': euler[0] if euler and euler[0] is not None else 0.0,
                    'roll': euler[1] if euler and euler[1] is not None else 0.0,
                    'pitch': euler[2] if euler and euler[2] is not None else 0.0
                }
            except Exception as e:
                print(f"Euler angles reading error: {e}")
        
        return {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
    
    def get_quaternion(self):
        """クォータニオンを取得"""
        if self.sensor is not None:
            try:
                quat = self.sensor.quaternion
                return {
                    'w': quat[0] if quat and quat[0] is not None else 1.0,
                    'x': quat[1] if quat and quat[1] is not None else 0.0,
                    'y': quat[2] if quat and quat[2] is not None else 0.0,
                    'z': quat[3] if quat and quat[3] is not None else 0.0
                }
            except Exception as e:
                print(f"Quaternion reading error: {e}")
        
        return {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def get_acceleration(self):
        """加速度を取得（m/s²）"""
        if self.sensor is not None:
            try:
                accel = self.sensor.acceleration
                return {
                    'x': accel[0] if accel and accel[0] is not None else 0.0,
                    'y': accel[1] if accel and accel[1] is not None else 0.0,
                    'z': accel[2] if accel and accel[2] is not None else 9.8
                }
            except Exception as e:
                print(f"Acceleration reading error: {e}")
        
        return {'x': 0.0, 'y': 0.0, 'z': 9.8}
    
    def get_gyro(self):
        """角速度を取得（rad/s）"""
        if self.sensor is not None:
            try:
                gyro = self.sensor.gyro
                return {
                    'x': math.radians(gyro[0]) if gyro and gyro[0] is not None else 0.0,
                    'y': math.radians(gyro[1]) if gyro and gyro[1] is not None else 0.0,
                    'z': math.radians(gyro[2]) if gyro and gyro[2] is not None else 0.0
                }
            except Exception as e:
                print(f"Gyro reading error: {e}")
        
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def get_calibration_status(self):
        """キャリブレーション状態を取得"""
        if self.sensor is not None:
            try:
                status = self.sensor.calibration_status
                self.calibration_status = {
                    'sys': status[0],
                    'gyro': status[1], 
                    'accel': status[2],
                    'mag': status[3]
                }
            except Exception as e:
                print(f"Calibration status error: {e}")
        
        return self.calibration_status
    
    def is_calibrated(self):
        """十分にキャリブレーションされているかチェック"""
        status = self.get_calibration_status()
        return all(status[key] >= 2 for key in ['sys', 'gyro', 'accel', 'mag'])
    
    def get_sensor_info(self):
        """センサー情報を取得"""
        info = {
            'connected': self.sensor is not None,
            'calibration_status': self.get_calibration_status(),
            'is_calibrated': self.is_calibrated()
        }
        
        if self.sensor is not None:
            try:
                info.update({
                    'sensor_id': self.sensor.sensor_id,
                    'sw_revision': self.sensor.sw_revision,
                    'bootloader_revision': self.sensor.bootloader_revision,
                    'temperature': self.sensor.temperature
                })
            except Exception as e:
                print(f"Sensor info error: {e}")
        
        return info

# 使用例とテスト
if __name__ == "__main__":
    import time
    
    print("=== BNO055 IMU Driver Test ===")
    imu = BNO055IMUDriver()
    
    try:
        # センサー情報表示
        info = imu.get_sensor_info()
        print(f"Sensor Info: {info}")
        
        # リアルタイムデータ取得テスト
        print("\nReading sensor data (Press Ctrl+C to stop)...")
        while True:
            yaw = imu.get_yaw()
            euler = imu.get_euler_angles()
            calib = imu.get_calibration_status()
            
            print(f"Yaw: {math.degrees(yaw):6.2f}° | "
                  f"Euler: {euler['yaw']:6.1f}°, {euler['roll']:6.1f}°, {euler['pitch']:6.1f}° | "
                  f"Calib: S{calib['sys']} G{calib['gyro']} A{calib['accel']} M{calib['mag']}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest completed")