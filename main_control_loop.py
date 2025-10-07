# main_waypoint_control.py - プラットフォーム対応版
import time
import threading
import queue
import numpy as np
import math
import json
import sys
import argparse
from narrow_passage_control import is_in_narrow_passage, narrow_passage_control
from platform_detector import is_raspberry_pi, get_platform_info

# コマンドライン引数の解析
def parse_arguments():
    parser = argparse.ArgumentParser(description='Autonomous Car Control System')
    parser.add_argument('--speed', type=float, default=None, 
                       help='Speed scale factor (e.g., 0.33 for 1/3 speed, 0.5 for half speed)')
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug mode (automatically sets speed to 0.33)')
    parser.add_argument('--config', default='config.json', 
                       help='Configuration file path')
    return parser.parse_args()

# コマンドライン引数解析
args = parse_arguments()

# 設定ファイル読み込み
try:
    with open(args.config, 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    print(f"Warning: {args.config} not found, using default settings")
    config = {}

# プラットフォーム情報取得
platform_info = get_platform_info()
print(f"Running on: {platform_info['system']} ({'Raspberry Pi' if platform_info['is_raspberry_pi'] else 'PC/Mock'})")

# ---- プラットフォーム別モジュール読み込み ----
if platform_info['is_raspberry_pi']:
    print("Loading Raspberry Pi hardware drivers...")
    try:
        from pca9685_motor_driver import PCA9685MotorDriver
        from bno055_imu_driver import BNO055IMUDriver
        PLATFORM_MODE = "raspberry_pi"
        print("✓ Hardware drivers loaded successfully")
    except ImportError as e:
        print(f"Hardware drivers not found: {e}")
        print("Fallback to mock mode")
        PLATFORM_MODE = "mock"
else:
    print("Running in PC/Mock mode")
    PLATFORM_MODE = "mock"

# ---- 自作モジュール（旧版との互換性維持） ----
try:
    import ultrasonic_array_thread
    import motor_drive
    import imu_sense_thread
except ImportError:
    print("Legacy modules not found, using mocks")
    ultrasonic_array_thread = None
    motor_drive = None
    imu_sense_thread = None

# ---- 設定（config.json から読み込み、デフォルト値で補完） ----
WAYPOINT_FILE = config.get('waypoints', {}).get('file', 'quarify.json')
LOOP_DELAY = config.get('system', {}).get('loop_delay', 0.05)
SAFE_DIST_FRONT = config.get('system', {}).get('safe_dist_front', 5)
SAFE_DIST_SIDE = config.get('system', {}).get('safe_dist_side', 7)
USE_NARROW_PASSAGE = config.get('system', {}).get('use_narrow_passage', True)

# デバッグ・速度制御設定（コマンドライン引数で上書き可能）
DEBUG_MODE = args.debug or config.get('system', {}).get('debug_mode', False)
SPEED_SCALE = args.speed if args.speed is not None else config.get('system', {}).get('speed_scale', 1.0)

# デバッグモードが有効な場合、速度を自動的に1/3に
if DEBUG_MODE and args.speed is None:
    SPEED_SCALE = 0.33

# プラットフォーム別設定
if PLATFORM_MODE == "raspberry_pi":
    platform_config = config.get('platform_specific', {}).get('raspberry_pi', {})
elif PLATFORM_MODE == "mock":
    platform_config = config.get('platform_specific', {}).get('pc_test', {})
else:
    platform_config = {}

print(f"Configuration loaded: Waypoints={WAYPOINT_FILE}, Loop={LOOP_DELAY}ms, Platform={PLATFORM_MODE}")
if DEBUG_MODE:
    print(f"🚧 DEBUG MODE: Speed reduced to {SPEED_SCALE*100:.0f}% of normal speed")
else:
    print("🏎️ NORMAL MODE: Full speed operation")

# ---- 車両制御フラグ ----
running_flag = threading.Event()  # 走行中フラグ
pause_flag = threading.Event()    # 一時停止フラグ

# ---- 状態送信キュー ----
status_queue = queue.Queue()

# ---- Waypoint読み込み ----
with open(WAYPOINT_FILE, 'r') as f:
    waypoints = json.load(f)

# ---- 超音波センサスレッド初期化 ----
class UltrasonicArrayMock:
    def __init__(self):
        self.lock = threading.Lock()
        self.FR = 100
        self.LH = 100
        self.RH = 100
        self.RLH = 100
        self.RRH = 100

    def update(self):
        # 仮の距離をランダムで更新
        import random
        with self.lock:
            self.FR = random.uniform(5, 200)
            self.LH = random.uniform(5, 200)
            self.RH = random.uniform(5, 200)
            self.RLH = random.uniform(5, 200)
            self.RRH = random.uniform(5, 200)

    def get_all(self):
        with self.lock:
            return self.FR, self.LH, self.RH, self.RLH, self.RRH

ultra_array = UltrasonicArrayMock()

def ultrasonic_loop():
    while True:
        ultra_array.update()
        time.sleep(0.05)  # 50ms

threading.Thread(target=ultrasonic_loop, daemon=True).start()

# ---- モータドライブ（プラットフォーム別） ----
if PLATFORM_MODE == "raspberry_pi" and 'PCA9685MotorDriver' in globals():
    print("Initializing PCA9685 motor driver...")
    motor = PCA9685MotorDriver(config_file=args.config)
else:
    print("Using motor mock...")
    
    class MotorDriveMock:
        def __init__(self):
            self.speed = 0
            self.steer_angle = 0

        def accel(self, duty):
            self.speed = duty * SPEED_SCALE  # 速度制限適用
            if DEBUG_MODE:
                print(f"[Motor Mock] Speed {self.speed:.2f} (scaled from {duty:.2f})")

        def steer(self, duty):
            self.steer_angle = duty
            if DEBUG_MODE:
                print(f"[Motor Mock] Steer {duty:.2f}°")

        def stop(self):
            self.accel(0)
            self.steer(0)
            
    motor = MotorDriveMock()

# ---- IMUセンサー（プラットフォーム別） ----
if PLATFORM_MODE == "raspberry_pi" and 'BNO055IMUDriver' in globals():
    print("Initializing BNO055 IMU driver...")
    imu_sensor = BNO055IMUDriver(config_file=args.config)
    
    def get_yaw():
        return imu_sensor.get_yaw()
        
    def get_sensor_info():
        return imu_sensor.get_sensor_info()
else:
    print("Using IMU mock...")
    
    class IMUMock:
        def __init__(self):
            self.yaw = 0.0
            
        def get_yaw(self):
            # 仮のヨー角変化をシミュレート
            import random
            self.yaw += random.uniform(-0.1, 0.1)
            return self.yaw
            
        def get_sensor_info(self):
            return {'connected': False, 'is_calibrated': True}
    
    imu_mock = IMUMock()
    
    def get_yaw():
        return imu_mock.get_yaw()
        
    def get_sensor_info():
        return imu_mock.get_sensor_info()

# IMU 2段階キャリブレーション: BNO055内蔵 + コース環境補正
print("Setting up IMU calibration system...")

# Stage 1: BNO055内蔵キャリブレーション（センサーレベル）
if PLATFORM_MODE == "raspberry_pi" and 'imu_sensor' in globals():
    print("✓ BNO055 hardware calibration active")
    
    def get_hardware_calibrated_yaw():
        """BNO055内蔵キャリブレーション済みヨー角"""
        return imu_sensor.get_yaw()
        
    # キャリブレーション状態確認
    if imu_sensor.is_calibrated():
        print("✓ BNO055 sensor calibration complete")
    else:
        print("⚠ BNO055 sensor calibration in progress...")
else:
    print("Using mock IMU (BNO055 hardware calibration unavailable)")
    
    def get_hardware_calibrated_yaw():
        """モック環境での基本ヨー角"""
        return get_yaw()

# Stage 2: 実走行コース環境補正（マップ表示付き推奨）
course_calibrator = None

# レース用：事前保存されたキャリブレーションファイル読み込み専用
# 注意: キャリブレーション取得は事前に imu_visual_calibration.py で実行済み

course_offset = 0.0
calibration_method = "none"

# 優先順位でキャリブレーションファイルを探索・読み込み
calibration_files = [
    ("imu_custom_calib.json", "custom_user_selection"),  # 最優先：ユーザーカスタム選択
    ("imu_visual_calib.json", "visual_map"),
    ("imu_landmark_calib.json", "landmark"),  
    ("imu_2point_calib.json", "2point")
]

for calib_file, method in calibration_files:
    try:
        with open(calib_file, 'r') as f:
            calib_data = json.load(f)
        
        # カスタムキャリブレーション形式の処理
        if method == "custom_user_selection":
            if 'calibration_result' in calib_data and 'yaw_offset' in calib_data['calibration_result']:
                course_offset = math.radians(calib_data['calibration_result']['yaw_offset'])
            else:
                print(f"⚠ Invalid custom calibration format in {calib_file}")
                continue
        else:
            # 従来形式の処理
            course_offset = calib_data.get('offset', 0.0)
        
        calibration_method = method
        
        print(f"✓ Course calibration loaded: {method.upper().replace('_', ' ')} PRECISION")
        print(f"  File: {calib_file}")
        print(f"  Offset: {math.degrees(course_offset):.2f}°")
        
        # タイムスタンプ表示
        if 'calibration_date' in calib_data:
            print(f"  Date: {calib_data['calibration_date']}")
        elif 'timestamp' in calib_data:
            print(f"  Date: {calib_data['timestamp']}")
        
        # カスタムキャリブレーション情報
        if method == "custom_user_selection" and 'usage_instructions' in calib_data:
            instructions = calib_data['usage_instructions']
            print(f"  Reference: {instructions.get('reference_heading', 'N/A')}")
            print(f"  Position: {instructions.get('vehicle_position', 'N/A')}")
        
        break
        
    except FileNotFoundError:
        continue
    except Exception as e:
        print(f"⚠ Error loading {calib_file}: {e}")
        continue

if calibration_method == "none":
    print("⚠ No course calibration found, using hardware calibration only")
    print("  Run imu_visual_calibration.py before race to generate calibration")

# キャリブレーション適用関数（軽量版）
def get_course_calibrated_yaw():
    """段階的キャリブレーション: BNO055内蔵 → コース環境補正"""
    hardware_yaw = get_hardware_calibrated_yaw()  # Stage 1
    calibrated = hardware_yaw + course_offset     # Stage 2
    
    # -π to π の範囲に正規化
    while calibrated > math.pi:
        calibrated -= 2 * math.pi
    while calibrated < -math.pi:
        calibrated += 2 * math.pi
        
    return calibrated

# 最終キャリブレーション済みヨー角
get_calibrated_yaw = get_course_calibrated_yaw
yaw = get_calibrated_yaw()

if calibration_method != "none":
    print(f"✓ Race-ready calibration active ({calibration_method}): {math.degrees(yaw):.2f}°")
else:
    print(f"○ Hardware calibration only: {math.degrees(yaw):.2f}°")

print(f"Initial Yaw: {math.degrees(yaw):.2f}°")

# ---- 超音波センサスレッド初期化 ----
class UltrasonicArrayMock:
    def __init__(self):
        self.lock = threading.Lock()
        self.FR = 100
        self.LH = 100
        self.RH = 100
        self.RLH = 100
        self.RRH = 100

    def update(self):
        # 仮の距離をランダムで更新
        import random
        with self.lock:
            self.FR = random.uniform(5, 200)
            self.LH = random.uniform(5, 200)
            self.RH = random.uniform(5, 200)
            self.RLH = random.uniform(5, 200)
            self.RRH = random.uniform(5, 200)

    def get_all(self):
        with self.lock:
            return self.FR, self.LH, self.RH, self.RLH, self.RRH

ultra_array = UltrasonicArrayMock()

def ultrasonic_loop():
    while True:
        ultra_array.update()
        time.sleep(0.05)  # 50ms

threading.Thread(target=ultrasonic_loop, daemon=True).start()

# ---- オドメトリシステム (モック) ----
class OdometryMock:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        
    def get_position(self):
        return self.x, self.y
        
    def update_position(self, dx, dy):
        self.x += dx
        self.y += dy

odom = OdometryMock()

# ---- キャリブレーション済みYaw値取得関数 ----
def get_current_yaw():
    """現在のキャリブレーション済みヨー角を取得"""
    if PLATFORM_MODE == "raspberry_pi" and 'get_calibrated_yaw' in globals():
        return get_calibrated_yaw()
    else:
        return get_yaw()

# 初期Yaw値取得
current_yaw = get_current_yaw()
print(f"Current Yaw: {math.degrees(current_yaw):.2f}°")

# ---- Legacy IMUキャリブレーション (オプション) ----
# （Raspberry Pi実機環境で利用される可能性のある旧システム互換性維持用）

# ---- メイン制御ループ ----
def waypoint_control_loop():
    idx = 0
    n_wp = len(waypoints)
    running_flag.wait()  # スタート待ち
    lap = 1  # 1周目からスタート

    while idx < n_wp:
        if pause_flag.is_set():
            motor.accel(0)
            motor.steer(0)
            time.sleep(0.05)
            continue

        # 現在 waypoint
        wp = waypoints[idx]
        wp_x, wp_y = wp['x'], wp['y']

        # 超音波距離取得
        FR, LH, RH, RLH, RRH = ultra_array.get_all()

        # 狭路判定・制御
        if USE_NARROW_PASSAGE and is_in_narrow_passage(LH, RH):
            print("Narrow passage detected!")
            if wp.get('narrow', False):
                # 周回ごとに進路を選択
                if lap == 1:
                    lane = "left"
                elif lap == 2:
                    lane = "center"
                elif lap == 3:
                    lane = "right"
                elif lap == 4:
                    lane = "left"
                else:
                    lane = "center"
                narrow_passage_control(motor, LH, RH, lane=lane)
                status_queue.put({'event':'narrow_passage', 'wp_idx': idx, 'lane': lane})
                time.sleep(0.1)
                continue

        # 障害物判定
        if FR < SAFE_DIST_FRONT:
            motor.accel(-50)
            motor.steer(0)
            print("Obstacle Front! Stop/Reverse")
            status_queue.put({'event':'obstacle_front', 'wp_idx': idx})
            time.sleep(0.1)
            continue

        # ウェイポイント情報の活用
        target_speed = wp.get('v', 50)  # ウェイポイント指定速度、デフォルト50
        target_yaw = wp.get('yaw', 0)   # ウェイポイント指定方位角
        
        # デバッグモード時の速度調整
        if DEBUG_MODE:
            target_speed = target_speed * SPEED_SCALE
            if target_speed < 10:  # 最低速度保証
                target_speed = 10
        
        # 進行方向計算（簡易）
        dx = wp_x - 0  # 仮: 現在位置=0
        dy = wp_y - 0
        
        # ウェイポイントにyaw情報があれば使用、なければ計算
        if 'yaw' in wp:
            angle_to_wp = target_yaw
        else:
            angle_to_wp = math.degrees(math.atan2(dy, dx))

        motor.accel(target_speed)
        motor.steer(angle_to_wp)

        # waypoint到達判定（簡易距離閾値）
        if math.hypot(dx, dy) < 5:
            idx += 1
            # 周回数更新
            if idx % len(waypoints) == 0:
                lap += 1

        # 状態送信
        status_queue.put({'wp_idx': idx, 'FR':FR, 'LH':LH, 'RH':RH, 'RLH':RLH, 'RRH':RRH, 'speed':motor.speed, 'steer':motor.steer_angle})
        time.sleep(LOOP_DELAY)

    motor.accel(0)
    motor.steer(0)
    print("Waypoint traversal complete")
    status_queue.put({'event':'complete'})

# ---- スタート・ストップ・復帰コマンド ----
def start():
    print("Start command received")
    pause_flag.clear()
    running_flag.set()

def stop():
    print("Stop command received")
    pause_flag.set()
    motor.accel(0)
    motor.steer(0)

def resume():
    print("Resume command received")
    pause_flag.clear()

# ---- メイン ----
if __name__ == '__main__':
    threading.Thread(target=waypoint_control_loop, daemon=True).start()
    print("Press Enter to START")
    input()
    start()

    try:
        while True:
            # 状態モニタ表示
            while not status_queue.empty():
                s = status_queue.get()
                print(f"Current Waypoint: {s.get('wp_idx', '-')}, Status: {s}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Emergency stop")
        motor.accel(0)
        motor.steer(0)
