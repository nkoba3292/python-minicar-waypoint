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
    parser.add_argument('--no-ultrasonic', action='store_true',
                       help='Disable ultrasonic sensors (use mock only - prevents WiFi disconnection)')
    parser.add_argument('--disable-wifi', action='store_true',
                       help='Disable WiFi during operation for maximum hardware stability')
    return parser.parse_args()

def manage_wifi(enable=True):
    """
    WiFiを有効化/無効化する（RF-kill対応版）
    Args:
        enable (bool): True=WiFi有効, False=WiFi無効
    Returns:
        bool: 成功=True, 失敗=False
    """
    import subprocess
    try:
        if enable:
            print("[WiFi] Enabling WiFi interface...")
            
            # 1. RF-kill解除
            print("[WiFi] Unblocking RF-kill...")
            subprocess.run(['sudo', 'rfkill', 'unblock', 'wifi'], 
                         check=False, capture_output=True, timeout=5)
            time.sleep(1)
            
            # 2. インターフェース有効化
            print("[WiFi] Bringing up interface...")
            subprocess.run(['sudo', 'ifconfig', 'wlan0', 'up'], 
                         check=True, capture_output=True, timeout=5)
            time.sleep(1)
            
            # 3. wpa_supplicant再起動（接続確立）
            print("[WiFi] Restarting wpa_supplicant...")
            subprocess.run(['sudo', 'systemctl', 'restart', 'wpa_supplicant'], 
                         check=False, capture_output=True, timeout=10)
            time.sleep(2)
            
            # 4. dhcpcd再起動（IPアドレス取得）
            print("[WiFi] Restarting dhcpcd...")
            subprocess.run(['sudo', 'systemctl', 'restart', 'dhcpcd'], 
                         check=False, capture_output=True, timeout=10)
            time.sleep(3)
            
            print("[WiFi] WiFi enabled and reconnecting...")
        else:
            print("[WiFi] Disabling WiFi interface for hardware stability...")
            subprocess.run(['sudo', 'ifconfig', 'wlan0', 'down'], 
                         check=True, capture_output=True, timeout=5)
            print("[WiFi] WiFi disabled - CPU/GPIO resources freed")
        return True
    except subprocess.TimeoutExpired:
        print(f"[ERROR] WiFi {'enable' if enable else 'disable'} timeout")
        return False
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] WiFi {'enable' if enable else 'disable'} failed: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] WiFi management error: {e}")
        return False

# コマンドライン引数解析
args = parse_arguments()

# 設定ファイル読み込み
try:
    with open(args.config, 'r') as f:
        config = json.load(f)
    print(f"[OK] Configuration loaded from {args.config}")
except FileNotFoundError:
    print(f"[INFO] {args.config} not found, using default settings")
    config = {}

# プラットフォーム情報取得
platform_info = get_platform_info()
print(f"Running on: {platform_info['system']} ({'Raspberry Pi' if platform_info['is_raspberry_pi'] else 'PC/Mock'})")

# ---- プラットフォーム別モジュール読み込み ----
if platform_info['is_raspberry_pi']:
    print("Loading Raspberry Pi hardware drivers...")
    try:
        from pca9685_motor_driver import PCA9685MotorDriver
        from IMU_sensor_bno055 import IMUSensorBNO055  # 新UART版IMU
        PLATFORM_MODE = "raspberry_pi"
        print("[OK] Hardware drivers loaded successfully")
    except ImportError as e:
        print(f"Hardware drivers not found: {e}")
        print("Fallback to mock mode")
        PLATFORM_MODE = "mock"
else:
    print("Running in PC/Mock mode")
    PLATFORM_MODE = "mock"

# ---- 超音波センサーモジュール読み込み（ポーリング方式） ----
try:
    import ultrasonic_array_polling
    print("[OK] Ultrasonic array module loaded (polling mode)")
except ImportError as e:
    print(f"[INFO] Ultrasonic array module not found: {e}")
    print("      Will use mock ultrasonic if needed")
    ultrasonic_array_polling = None

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

print(f"Configuration loaded: Waypoints={WAYPOINT_FILE}, Loop={LOOP_DELAY*1000:.0f}ms, Platform={PLATFORM_MODE}")
if DEBUG_MODE:
    print(f"[DEBUG MODE] Speed reduced to {SPEED_SCALE*100:.0f}% of normal speed")
else:
    print("[NORMAL MODE] Full speed operation")

# ---- 車両制御フラグ ----
running_flag = threading.Event()  # 走行中フラグ
pause_flag = threading.Event()    # 一時停止フラグ

# ---- 状態送信キュー ----
status_queue = queue.Queue()

# ---- Waypoint読み込み ----
with open(WAYPOINT_FILE, 'r') as f:
    waypoints = json.load(f)

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
# ====== デバッグ用: IMU無効化 ======
# 超音波センサー単体テストのため、IMUを一時的に無効化
# 元に戻す場合: 下の3行のコメントを外し、この行と次の行を削除
USE_IMU_DEBUG = False  # True=IMU有効, False=IMU無効（超音波テスト用）

if USE_IMU_DEBUG and PLATFORM_MODE == "raspberry_pi" and 'IMUSensorBNO055' in globals():
# if PLATFORM_MODE == "raspberry_pi" and 'IMUSensorBNO055' in globals():  # ← 元に戻す時はこの行のコメント外す
    print("Initializing IMU_sensor_bno055 (UART) driver...")
    imu_sensor = IMUSensorBNO055(
        port='/dev/serial0',
        baudrate=115200,
        offset=0.0,
        calib_file='bno055_calibdata.bin'
    )
    
    def get_yaw():
        return imu_sensor.get_yaw()
        
    def get_sensor_info():
        return imu_sensor.get_sensor_info()
else:
    print("Using IMU mock... (DEBUG: IMU disabled for ultrasonic test)")
    
    class IMUMock:
        def __init__(self):
            self.yaw = 0.0
            self.calibrated = True  # モックは常にキャリブレーション済み
            
        def get_yaw(self):
            # 仮のヨー角変化をシミュレート
            import random
            self.yaw += random.uniform(-0.1, 0.1)
            return self.yaw
            
        def get_sensor_info(self):
            return {'connected': False, 'is_calibrated': True}
        
        def get_position(self):
            # 位置情報のモック（0, 0固定）
            return (0.0, 0.0)
        
        def is_calibrated(self):
            # キャリブレーション状態（常にTrue）
            return True
        
        def get_calibration_status(self):
            # キャリブレーションステータス（全て完了）
            return {'sys': 3, 'gyro': 3, 'accel': 3, 'mag': 3}
    
    imu_mock = IMUMock()
    imu_sensor = imu_mock  # imu_sensorとしてもアクセス可能に
    
    def get_yaw():
        return imu_mock.get_yaw()
        
    def get_sensor_info():
        return imu_mock.get_sensor_info()

# IMU 2段階キャリブレーション: BNO055内蔵 + コース環境補正
print("Setting up IMU calibration system...")

# Stage 1: BNO055内蔵キャリブレーション（センサーレベル）
if PLATFORM_MODE == "raspberry_pi" and 'imu_sensor' in globals():
    print("[OK] BNO055 hardware calibration active (UART mode)")
    
    def get_hardware_calibrated_yaw():
        """BNO055内蔵キャリブレーション済みヨー角"""
        return imu_sensor.get_yaw()
        
    # キャリブレーション状態確認
    if imu_sensor.is_calibrated():
        print("[OK] BNO055 sensor calibration complete")
    else:
        print("[WARN] BNO055 sensor calibration in progress...")
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
            # 新形式: calibration_result.yaw_offset
            if 'calibration_result' in calib_data and 'yaw_offset' in calib_data['calibration_result']:
                course_offset = math.radians(calib_data['calibration_result']['yaw_offset'])
            # 旧形式: calculated_offset（ラジアン）
            elif 'calculated_offset' in calib_data:
                course_offset = calib_data['calculated_offset']
            else:
                print(f"[WARN] Invalid custom calibration format in {calib_file}")
                print(f"       Expected 'calibration_result.yaw_offset' or 'calculated_offset'")
                continue
        else:
            # 従来形式の処理
            course_offset = calib_data.get('offset', 0.0)
        
        calibration_method = method
        
        print(f"[OK] Course calibration loaded: {method.upper().replace('_', ' ')} PRECISION")
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
        print(f"[WARN] Error loading {calib_file}: {e}")
        continue

if calibration_method == "none":
    print("[WARN] No course calibration found, using hardware calibration only")
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
    print(f"[OK] Race-ready calibration active ({calibration_method}): {math.degrees(yaw):.2f}°")
else:
    print(f"[INFO] Hardware calibration only: {math.degrees(yaw):.2f}°")

print(f"Initial Yaw: {math.degrees(yaw):.2f}°")

# ---- 超音波センサスレッド初期化 ----
ultra_array = None  # グローバル変数として初期化

# システムリソース確認（WiFi切断診断用）
try:
    import psutil
    mem = psutil.virtual_memory()
    cpu_percent = psutil.cpu_percent(interval=1)
    print(f"[INFO] System resources before ultrasonic init:")
    print(f"       Memory: {mem.percent}% used ({mem.available/1024/1024:.0f}MB available)")
    print(f"       CPU: {cpu_percent}% usage")
except ImportError:
    print("[INFO] psutil not available - install for resource monitoring")

# コマンドラインで無効化されている場合はスキップ
if args.no_ultrasonic:
    print("[INFO] Ultrasonic sensors disabled by --no-ultrasonic flag")
    print("       Using mock ultrasonic (WiFi disconnection prevention mode)")
elif PLATFORM_MODE == "raspberry_pi" and ultrasonic_array_polling is not None:
    print("Initializing ultrasonic array (polling mode - togikai-compliant)...")
    try:
        # togikai準拠：待機なし、即座に初期化
        ultra_array = ultrasonic_array_polling.UltrasonicSensorsPolling(debug_mode=False)
        print("[OK] Ultrasonic array initialized (polling mode)")
        
        # 初回測定テスト
        print("Testing initial sensor measurement...")
        distances = ultra_array.measure_once()
        print(f"  Initial readings: FR={distances[0]:.1f} L45={distances[1]:.1f} L90={distances[2]:.1f} R45={distances[3]:.1f} R90={distances[4]:.1f}")
        
        if distances and any(d < 200 for d in distances):
            print("[OK] Ultrasonic sensors working")
        else:
            print(f"[WARN] All sensors timeout - check hardware connections")
            # タイムアウトでも続行（障害物なしとして動作）
        
    except Exception as e:
        print(f"[WARN] Ultrasonic initialization failed: {e}")
        import traceback
        traceback.print_exc()
        ultra_array = None  # 例外時はNoneにしてモックへフォールバック

# モックへのフォールバック（Raspberry Piでの初期化失敗時 or PC環境）
if ultra_array is None:
    print("Using ultrasonic mock...")
    
    class UltrasonicArrayMock:
        def __init__(self):
            self.lock = threading.Lock()
            self.FR = 100
            self.LH = 100
            self.RH = 100
            self.RLH = 100
            self.RRH = 100

        def update(self):
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
        
        def get_distances(self):
            return self.get_all()

    ultra_array = UltrasonicArrayMock()

    def ultrasonic_loop():
        while True:
            ultra_array.update()
            time.sleep(0.1)

    threading.Thread(target=ultrasonic_loop, daemon=True).start()

# センサー初期化確認
print(f"[INFO] Ultrasonic sensor type: {type(ultra_array).__name__}")
print(f"[INFO] Has get_distances: {hasattr(ultra_array, 'get_distances')}")

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

        # 超音波距離取得（ポーリング方式: 必要な時だけ測定）
        # 10サイクルに1回測定（50ms × 10 = 500ms間隔）
        if hasattr(ultra_array, 'measure_once'):
            # ポーリングモード: 10サイクルに1回測定
            if idx % 10 == 0 or idx == 0:
                distances = ultra_array.measure_once()
            else:
                distances = ultra_array.get_distances()  # 前回の値を使用
        else:
            # モックモード: 常に取得
            distances = ultra_array.get_distances()
        
        if distances is None or len(distances) < 5:
            print(f"[WARN] Invalid ultrasonic data: {distances}")
            FR, LH, RH, RLH, RRH = 100, 100, 100, 100, 100  # デフォルト値
        else:
            FR, LH, RH, RLH, RRH = distances[0], distances[1], distances[2], distances[3], distances[4]
        
        # デバッグ: 最初のループで値を確認
        if idx == 0 and DEBUG_MODE:
            print(f"[DEBUG] Ultrasonic raw: FR={FR} LH={LH} RH={RH} RLH={RLH} RRH={RRH}")

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
        
        # 現在位置取得 (IMUまたはオドメトリから)
        if PLATFORM_MODE == "raspberry_pi" and 'imu_sensor' in globals() and hasattr(imu_sensor, 'get_position'):
            pos_data = imu_sensor.get_position()
            # get_position()が返す値の数を確認
            if isinstance(pos_data, (list, tuple)):
                if len(pos_data) >= 2:
                    current_x, current_y = pos_data[0], pos_data[1]
                else:
                    current_x, current_y = 0, 0  # フォールバック
            else:
                current_x, current_y = 0, 0  # フォールバック
        else:
            current_x, current_y = odom.get_position()
        
        # 現在のヨー角取得
        current_yaw = get_current_yaw()
        
        # 目標方位角の決定
        if 'yaw' in wp:
            # Waypointにyawが指定されている場合はそれを使用
            target_yaw_rad = math.radians(wp['yaw'])
        else:
            # Waypointへの方向を現在位置から計算
            dx = wp_x - current_x
            dy = wp_y - current_y
            target_yaw_rad = math.atan2(dy, dx)
        
        # ヨー角差分計算（-π to π に正規化）
        yaw_error = target_yaw_rad - current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # ステアリング角度に変換（比例制御）
        steer_gain = config.get('motor', {}).get('steer_correction_factor', 2.0)
        steer_angle = math.degrees(yaw_error) * steer_gain
        
        # ステアリング角度制限（-45°〜+45°）
        steer_angle = max(-45, min(45, steer_angle))

        motor.accel(target_speed)
        motor.steer(steer_angle)
        
        # Waypoint到達判定用の距離計算
        dx_check = wp_x - current_x
        dy_check = wp_y - current_y
        distance_to_wp = math.hypot(dx_check, dy_check)
        
        # デバッグ情報表示（コンパクト版）
        if DEBUG_MODE:
            print(f"[DEBUG] WP:{idx}/{n_wp-1} Tgt:({wp_x:.1f},{wp_y:.1f},{math.degrees(target_yaw_rad):.0f}°) Cur:({current_x:.1f},{current_y:.1f},{math.degrees(current_yaw):.0f}°) Ctrl:(A={target_speed:.0f},S={steer_angle:.0f}°) Err:(Y={math.degrees(yaw_error):.0f}°,D={distance_to_wp:.2f}m)")

        # Waypoint到達判定
        if distance_to_wp < 0.5:  # 50cm以内で到達判定
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
    wifi_was_disabled = False
    
    try:
        # WiFi無効化オプションが指定されている場合
        if args.disable_wifi and platform_info['is_raspberry_pi']:
            print("\n" + "="*60)
            print("WiFi Disable Mode - Maximum Hardware Stability")
            print("="*60)
            print("WARNING: Remote monitoring/control will be unavailable")
            print("Press Ctrl+C to cancel, or wait 5 seconds to continue...")
            print("="*60)
            try:
                time.sleep(5)
                if manage_wifi(enable=False):
                    wifi_was_disabled = True
                    print("[OK] WiFi disabled - System resources optimized")
                else:
                    print("[WARN] WiFi disable failed - Continuing with WiFi enabled")
            except KeyboardInterrupt:
                print("\nWiFi disable cancelled by user")
                sys.exit(0)
        
        # メイン制御スレッド起動
        threading.Thread(target=waypoint_control_loop, daemon=True).start()
        print("Press Enter to START")
        input()
        start()

        try:
            while True:
                # 状態モニタ表示（メインループと同期）
                while not status_queue.empty():
                    s = status_queue.get()
                    print(f"WP:{s.get('wp_idx', '-')} FR:{s.get('FR', 0):.1f} LH:{s.get('LH', 0):.1f} RH:{s.get('RH', 0):.1f} Spd:{s.get('speed', 0)} St:{s.get('steer', 0):.1f}")
                time.sleep(0.05)  # 50ms（メインループと同期して1回ずつ表示）

        except KeyboardInterrupt:
            print("\nEmergency stop")
            motor.accel(0)
            motor.steer(0)
    
    finally:
        # 終了処理：WiFi復旧
        if wifi_was_disabled:
            print("\n[WiFi] Restoring WiFi connection...")
            if manage_wifi(enable=True):
                print("[OK] WiFi restored")
            else:
                print("[WARN] WiFi restore failed - may need manual recovery: sudo rfkill unblock wifi && sudo ifconfig wlan0 up")
        
        # 超音波センサー停止
        if 'ultra_array' in globals() and ultra_array is not None:
            try:
                ultra_array.stop()
                print("[OK] Ultrasonic sensors stopped")
            except:
                pass
        
        print("Program terminated")
