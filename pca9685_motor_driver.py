# pca9685_motor_driver.py - PCA9685を使用したモーター・サーボ制御
import json
from platform_detector import is_raspberry_pi

# Raspberry Pi環境でのみライブラリをインポート
if is_raspberry_pi():
    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685
        from adafruit_motor import servo
        import digitalio
    except ImportError as e:
        print(f"PCA9685 libraries not available: {e}")
        board = busio = PCA9685 = servo = digitalio = None
else:
    board = busio = PCA9685 = servo = digitalio = None

class PCA9685MotorDriver:
    def __init__(self, config_file="config.json"):
        # 設定読み込み
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            self.pca_config = config.get('pca9685', {})
        except Exception as e:
            print(f"Config file error: {e}")
            self.pca_config = {}
        
        # 設定値
        self.i2c_address = int(self.pca_config.get('i2c_address', '0x40'), 16)
        self.frequency = self.pca_config.get('frequency', 50)
        self.motor_channel = self.pca_config.get('motor_channel', 0)
        self.servo_channel = self.pca_config.get('servo_channel', 1)
        
        # PWM値設定
        self.motor_min = self.pca_config.get('motor_min', 1000)
        self.motor_max = self.pca_config.get('motor_max', 2000)
        self.motor_neutral = self.pca_config.get('motor_neutral', 1500)
        self.servo_min = self.pca_config.get('servo_min', 1000) 
        self.servo_max = self.pca_config.get('servo_max', 2000)
        self.servo_center = self.pca_config.get('servo_center', 1500)
        
        # 現在の状態
        self.speed = 0
        self.steer_angle = 0
        
        # ハードウェア初期化
        self.pca = None
        self.motor_servo = None
        self.steer_servo = None
        
        if is_raspberry_pi() and board is not None:
            self._init_hardware()
        else:
            print("Running in mock mode - PCA9685 not initialized")
    
    def _init_hardware(self):
        """PCA9685ハードウェアの初期化"""
        try:
            # I2C初期化
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=self.i2c_address)
            self.pca.frequency = self.frequency
            
            # サーボ初期化
            self.motor_servo = servo.Servo(self.pca.channels[self.motor_channel], 
                                         min_pulse=self.motor_min, 
                                         max_pulse=self.motor_max)
            self.steer_servo = servo.Servo(self.pca.channels[self.servo_channel],
                                         min_pulse=self.servo_min,
                                         max_pulse=self.servo_max)
            
            # 初期位置設定
            self.motor_servo.angle = 90  # ニュートラル
            self.steer_servo.angle = 90  # センター
            
            print("PCA9685 initialized successfully")
            
        except Exception as e:
            print(f"PCA9685 initialization failed: {e}")
            self.pca = None
    
    def accel(self, duty):
        """
        モーター速度制御
        duty: -100 to 100 (負の値は後進)
        """
        self.speed = max(-100, min(100, duty))  # -100~100に制限
        
        if self.pca is not None:
            try:
                # duty値を角度に変換 (-100~100 → 0~180)
                angle = 90 + (self.speed * 0.9)  # 90±90度
                angle = max(0, min(180, angle))
                self.motor_servo.angle = angle
                print(f"[PCA9685 Motor] Speed {self.speed} → Angle {angle:.1f}°")
            except Exception as e:
                print(f"Motor control error: {e}")
        else:
            print(f"[Mock Motor] Speed {self.speed}")
    
    def steer(self, duty):
        """
        ステアリング制御
        duty: -90 to 90 (負の値は左、正の値は右)
        """
        self.steer_angle = max(-90, min(90, duty))  # -90~90に制限
        
        if self.pca is not None:
            try:
                # duty値を角度に変換 (-90~90 → 0~180)
                angle = 90 + self.steer_angle  # 90±90度
                angle = max(0, min(180, angle))
                self.steer_servo.angle = angle
                print(f"[PCA9685 Steer] Angle {self.steer_angle} → Servo {angle:.1f}°")
            except Exception as e:
                print(f"Steering control error: {e}")
        else:
            print(f"[Mock Steer] Angle {self.steer_angle}")
    
    def stop(self):
        """緊急停止"""
        self.accel(0)
        self.steer(0)
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        if self.pca is not None:
            try:
                self.stop()
                self.pca.deinit()
                print("PCA9685 cleaned up")
            except Exception as e:
                print(f"Cleanup error: {e}")

# 使用例とテスト
if __name__ == "__main__":
    import time
    
    print("=== PCA9685 Motor Driver Test ===")
    motor = PCA9685MotorDriver()
    
    try:
        # テストシーケンス
        print("Testing motor control...")
        motor.accel(30)   # 前進30%
        time.sleep(2)
        
        print("Testing steering...")
        motor.steer(45)   # 右に45度
        time.sleep(2)
        
        motor.steer(-45)  # 左に45度
        time.sleep(2)
        
        print("Returning to center...")
        motor.steer(0)    # センター
        motor.accel(0)    # 停止
        
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        motor.cleanup()