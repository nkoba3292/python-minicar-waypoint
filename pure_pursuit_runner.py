# --- スタック判定・復帰用変数 ---
stuck_timer = None
stuck_state = False

# --- 障害物回避用 ---
obstacle_state = False
# --- 異常旋回判定用 ---
last_yaw = None
yaw_spin_timer = None
spin_detected = False
reverse_timer = None
reverse_state = False

try:
    import togikai_ultrasonic
except ImportError:
    # Mock ultrasonic module for development/testing
    class MockUltrasonic:
        @staticmethod
        def Mesure(gpio, time, trig_pin, echo_pin):
            return 50.0  # Return default distance value
    
    togikai_ultrasonic = MockUltrasonic()
    print("[WARN] Using mock ultrasonic for development")

try:
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        # Mock GPIO class for development/testing
        class MockGPIO:
            BOARD = "BOARD"
            OUT = "OUT"
            IN = "IN"
            LOW = 0
            HIGH = 1
            
            @staticmethod
            def setmode(mode):
                pass
            
            @staticmethod
            def setup(pins, mode, initial=None):
                pass
            
            @staticmethod
            def output(pin, value):
                pass
            
            @staticmethod
            def input(pin):
                return 0
            
            @staticmethod
            def cleanup():
                pass
        
        GPIO = MockGPIO()
        print("[WARN] Using mock GPIO for development")
    # 超音波センサのピン設定（例: 02_togikai_sample.pyと同じ）
    t_list = [15,13,35,32,36]
    e_list = [26,24,37,31,38]
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(t_list, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(e_list, GPIO.IN)
    ultrasonic_available = True
except (ImportError, ModuleNotFoundError) as e:
    print(f"[WARN] RPi.GPIO または togikai_ultrasonic モジュールが見つかりません: {e}")
    GPIO = None
    ultrasonic_available = False
    # 障害物回避用変数の初期化
    obstacle_state = False
    last_yaw = None
    yaw_spin_timer = None
    spin_detected = False
    reverse_timer = None
    reverse_state = False
except Exception as e:
    print(f"[WARN] 超音波センサ初期化失敗: {e}")
    GPIO = None
    ultrasonic_available = False
    # 障害物回避用変数の初期化
    obstacle_state = False
    last_yaw = None
    yaw_spin_timer = None
    spin_detected = False
    reverse_timer = None
    reverse_state = False
"""
pure_pursuit_runner.py
アッカーマン型ラジコン車両のためのPure Pursuit方式waypoint追従サンプル
- togikai_drive.pyのAccel/Steerを利用
- waypointファイル(JSON)を読み込み
- IMUやオドメトリは仮想値でテスト可能
"""

import time
import sys
import math
import json
import traceback
sys.path.append('/home/pi/togikai/togikai_function/')
try:
    import togikai_drive
    import PCA9685
except ImportError as e:
    print(f"[ERROR] 必要なモジュールのインポートに失敗: {e}")
    sys.exit(1)

# --- IMUセンサ初期化 ---
try:
    from IMU_sensor_bno055 import IMUSensorBNO055
    imu_sensor = IMUSensorBNO055()
    print("[INFO] IMUセンサ初期化成功")
except Exception as e:
    print(f"[ERROR] IMUセンサ初期化失敗: {e}")
    imu_sensor = None

# PWM初期化
try:
    pwm = PCA9685.PCA9685(address=0x40, busnum=1)
    pwm.set_pwm_freq(60)
    PWM_PARAM = togikai_drive.ReadPWMPARAM(pwm)
except Exception as e:
    print(f"[ERROR] PWM初期化失敗: {e}")
    traceback.print_exc()
    sys.exit(1)

# --- グローバル変数（現在位置） ---
x = 0.0
y = 0.0

# --- 制御パラメータ ---
MAX_SPEED = 100  # 最大スロットル値
LOOKAHEAD_DIST = 0.5  # 前方注視距離[m]

# --- Waypointデータの読み込み ---
def load_waypoints(filepath):
    """
    JSONファイルからwaypointデータを読み込む
    
    Args:
        filepath: waypointファイルのパス
    
    Returns:
        waypoints: waypoint辞書のリスト
    """
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Waypointファイルが見つかりません: {filepath}")
        return []
    except json.JSONDecodeError:
        print(f"[ERROR] Waypointファイルの形式が正しくありません: {filepath}")
        return []

# デフォルトのwaypoints（ファイルが無い場合の代替）
waypoints = [
    {'x': 0, 'y': 0, 'v': 50, 'yaw': 0},
    {'x': 100, 'y': 0, 'v': 50, 'yaw': 0},
    {'x': 100, 'y': 100, 'v': 50, 'yaw': 90},
    {'x': 0, 'y': 100, 'v': 50, 'yaw': 180}
]

# waypointファイルを読み込み（存在する場合）
try:
    loaded_waypoints = load_waypoints('waypoints.json')
    if loaded_waypoints:
        waypoints = loaded_waypoints
        print(f"[INFO] Waypointファイル読み込み完了: {len(waypoints)}個")
    else:
        print(f"[INFO] デフォルトWaypoint使用: {len(waypoints)}個")
except Exception as e:
    print(f"[WARN] Waypoint読み込みエラー、デフォルト使用: {e}")

# --- goto_waypoint関数: 1つのwaypointへ向かう制御 ---
def goto_waypoint(waypoint, imu_offset=0.0, lookahead_dist=0.5, max_speed=70):
    """
    1つのwaypointへ向かう走行制御
    - Pure Pursuit制御
    - センサ連携（IMU・超音波）
    - 例外復帰（スタック・逆走・スピン・障害物回避）
    
    Args:
        waypoint: dict形式のwaypoint {'x': ..., 'y': ..., 'v': ..., 'yaw': ...}
        imu_offset: IMUオフセット角度
        lookahead_dist: 前方注視距離[m]
        max_speed: 最大スロットル[0-100]
    
    Returns:
        'reached': waypoint到達
        'stopped': ユーザー停止
        'error': エラー発生
    """
    global x, y
    global stuck_timer, stuck_state
    global reverse_timer, reverse_state
    global last_yaw, yaw_spin_timer, spin_detected
    global obstacle_state
    
    target_x = waypoint['x']*0.05-3.2
    target_y = waypoint['y']*0.05-1.5
    target_yaw = waypoint.get('yaw', 0)
    target_v = waypoint.get('v', max_speed)
    
    # waypoint到達まで制御ループ
    while True:
        try:
            # TODO: waypoint制御ロジックを実装
            pass
        except Exception as e:
            print(f"[ERROR] waypoint制御エラー: {e}")
            return 'error'


# --- メインループ ---
try:
    for i, wp in enumerate(waypoints):
        # --- 障害物回避判定 ---
        obstacle_detected = False
        steer_dir = 0
        # 前方・側方の超音波センサ値で障害物判定
        if FRdis is not None and FRdis < 20.0:
            obstacle_detected = True
            # 左右比較（右が広いなら右へ、左が広いなら左へ）
            if LHdis is not None and RHdis is not None:
                if LHdis > RHdis:
                    steer_dir = -100  # 左へ
                else:
                    steer_dir = 100   # 右へ
            elif LHdis is not None:
                steer_dir = -100
            elif RHdis is not None:
                steer_dir = 100
        # 進行方向waypoint上に障害物（仮: FRdis/LHdis/RHdisすべて20cm未満）
        if (FRdis is not None and FRdis < 20.0) and (LHdis is not None and LHdis < 20.0) and (RHdis is not None and RHdis < 20.0):
            obstacle_detected = True
            steer_dir = 0  # どちらも狭い場合は直進徐行

        # --- 障害物回避動作 ---
        if obstacle_detected or obstacle_state:
            print(f"[OBSTACLE] 障害物回避: 徐行duty=20, steer={steer_dir}")
            togikai_drive.Accel(PWM_PARAM, pwm, time, 20)
            togikai_drive.Steer(PWM_PARAM, pwm, time, steer_dir)
            obstacle_state = True
            # 障害物が消えたら復帰
            if FRdis is not None and FRdis > 30.0 and (LHdis is None or LHdis > 30.0) and (RHdis is None or RHdis > 30.0):
                print("[OBSTACLE] 障害物消失→purepursuit復帰")
                obstacle_state = False
            else:
                time.sleep(0.1)
                continue
        # --- 異常旋回（スピン）判定 ---
        if last_yaw is not None:
            yaw_delta = abs(yaw - last_yaw)
            # 360度超えを考慮
            if yaw_delta > 180:
                yaw_delta = 360 - (yaw_delta % 360)
            if yaw_spin_timer is None:
                yaw_spin_timer = time.time()
                spin_sum = 0
            spin_sum += yaw_delta
            # 1秒以内に±720°以上回転したら異常スピン
            if spin_sum >= 720 and (time.time() - yaw_spin_timer) < 1.0:
                print(f"[SPIN] 異常旋回検出: ΔYAW={spin_sum:.1f}° 1秒以内")
                spin_detected = True
            # 1秒経過でリセット
            if (time.time() - yaw_spin_timer) >= 1.0:
                yaw_spin_timer = time.time()
                spin_sum = 0
        else:
            spin_sum = 0
            yaw_spin_timer = time.time()
        last_yaw = yaw

        # --- 異常旋回復帰 ---
        if spin_detected:
            # 最も近いwaypointを再設定
            min_dist = float('inf')
            min_idx = 0
            for idx, wp2 in enumerate(waypoints):
                dx2 = wp2['x']*0.05-3.2 - x
                dy2 = wp2['y']*0.05-1.5 - y
                d2 = math.hypot(dx2, dy2)
                if d2 < min_dist:
                    min_dist = d2
                    min_idx = idx
            print(f"[SPIN] waypoint再設定: index={min_idx} 距離={min_dist:.2f}m")
            i = min_idx - 1  # forループで+1されるため
            spin_detected = False
            continue
        # --- 逆走判定 ---
        # 進行方向ベクトル
        dx = wp['x']*0.05-3.2 - x
        dy = wp['y']*0.05-1.5 - y
        wp_dir = math.degrees(math.atan2(dy, dx))
        # YAWとの差分（-180～180°）
        yaw_diff = ((yaw - wp_dir + 180) % 360) - 180
        # 逆走判定
        if abs(yaw_diff) >= 90:
            if reverse_timer is None:
                reverse_timer = time.time()
            elif time.time() - reverse_timer > 5.0 and not reverse_state:
                reverse_state = True
                print(f"[REVERSE] 逆走判定: YAW差={yaw_diff:.1f}° その場左折復帰開始")
        else:
            reverse_timer = None
            reverse_state = False

        # --- 逆走復帰動作 ---
        if reverse_state:
            # その場左折（DUTY=-100）
            print("[REVERSE] その場左折 (duty=-100)")
            togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
            togikai_drive.Steer(PWM_PARAM, pwm, time, -100)
            time.sleep(0.2)
            # 衝突時は既存スタック判定に移行（us_stuck & accel_stuck）
            if us_stuck and accel_stuck:
                print("[REVERSE] 左折中に壁衝突→スタック復帰へ移行")
                reverse_state = False
                reverse_timer = None
                stuck_state = True
                continue
            # YAWが進行方向に近づいたら復帰
            dx = wp['x']*0.05-3.2 - x
            dy = wp['y']*0.05-1.5 - y
            wp_dir = math.degrees(math.atan2(dy, dx))
            yaw_diff = ((yaw - wp_dir + 180) % 360) - 180
            if abs(yaw_diff) < 60:
                print(f"[REVERSE] 進行方向復帰: YAW差={yaw_diff:.1f}°→purepursuit復帰")
                reverse_state = False
                reverse_timer = None
                continue
            # 逆走復帰中はpurepursuit本体スキップ
            continue
        # --- 超音波センサ計測（20Hz） ---
        FRdis = LHdis = RHdis = RLHdis = RRHdis = None
        if ultrasonic_available:
            try:
                FRdis = togikai_ultrasonic.Mesure(GPIO, time, 15, 26)
                LHdis = togikai_ultrasonic.Mesure(GPIO, time, 13, 24)
                RHdis = togikai_ultrasonic.Mesure(GPIO, time, 32, 31)
                RLHdis = togikai_ultrasonic.Mesure(GPIO, time, 35, 37)
                RRHdis = togikai_ultrasonic.Mesure(GPIO, time, 36, 38)
                print(f"[US] FR:{FRdis:.1f} LH:{LHdis:.1f} RH:{RHdis:.1f} RLH:{RLHdis:.1f} RRH:{RRHdis:.1f}")
            except Exception as e:
                print(f"[WARN] 超音波センサ計測失敗: {e}")

        # --- IMU加速度取得 ---
        accel_val = None
        if imu_sensor is not None:
            try:
                imu_data = imu_sensor.get_all()
                yaw = imu_data.get('yaw', 0)
                accel_val = imu_data.get('accel', None)
            except Exception as e:
                print(f"[WARN] IMU値取得失敗: {e}")
                yaw = 0
        else:
            yaw = 0

        # --- 衝突・スタック判定 ---
    # stuck_timer, stuck_stateはグローバル定義済み
        us_list = [d for d in [FRdis, LHdis, RHdis, RLHdis, RRHdis] if d is not None]
        us_stuck = any(d <= 5.0 for d in us_list)
        accel_stuck = False
        if accel_val is not None and isinstance(accel_val, (tuple, list)):
            accel_stuck = all(abs(a) < 0.1 for a in accel_val)
        now = time.time()
        if us_stuck and accel_stuck:
            if stuck_timer is None:
                stuck_timer = now
            elif now - stuck_timer > 1.0 and not stuck_state:
                stuck_state = True
                print("[STUCK] 衝突・スタック判定: 復帰動作開始")
        else:
            stuck_timer = None
            stuck_state = False

        # --- スタック復帰動作 ---
        if stuck_state:
            # 1秒バック
            print("[STUCK] 1秒バック (duty=-100)")
            togikai_drive.Accel(PWM_PARAM, pwm, time, -100)
            togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
            time.sleep(1.0)
            # 1秒右旋回
            print("[STUCK] 1秒右旋回 (duty=100)")
            togikai_drive.Accel(PWM_PARAM, pwm, time, -100)
            togikai_drive.Steer(PWM_PARAM, pwm, time, 100)
            time.sleep(1.0)
            # 停止
            togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
            togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
            # 前方クリアまで待機
            print("[STUCK] 前方クリア待機")
            while True:
                if ultrasonic_available:
                    try:
                        FRdis = togikai_ultrasonic.Mesure(GPIO, time, 15, 26)
                        print(f"[STUCK] FR:{FRdis:.1f}")
                        if FRdis > 10.0:
                            print("[STUCK] 前方クリア、purepursuit復帰")
                            break
                    except Exception as e:
                        print(f"[WARN] 超音波センサ計測失敗: {e}")
                time.sleep(0.1)
            stuck_state = False
            stuck_timer = None
            continue  # purepursuit本体はスキップして次ループへ

        # --- Waypoint座標取得 ---
        target_x = wp['x']*0.05-3.2
        target_y = wp['y']*0.05-1.5
        target_yaw = wp.get('yaw', 0)
        target_v = wp.get('v', MAX_SPEED)

        # --- Pure Pursuit制御 ---
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        if dist < LOOKAHEAD_DIST:
            print(f"[DEBUG] {i}: 近すぎるためスキップ (dist={dist:.2f})")
            continue  # 近すぎる場合は次のwaypointへ

        # 車両座標系に変換
        local_x = math.cos(math.radians(-yaw))*dx - math.sin(math.radians(-yaw))*dy
        local_y = math.sin(math.radians(-yaw))*dx + math.cos(math.radians(-yaw))*dy

        # 前方注視点が後方ならスキップ
        if local_x < 0:
            print(f"[DEBUG] {i}: 注視点が後方のためスキップ (local_x={local_x:.2f})")
            continue

        # ピュアパシュートの操舵角計算
        L = 0.25  # ホイールベース[m]（実車値に合わせて調整）
        curvature = 2*local_y/(LOOKAHEAD_DIST**2)
        steer_angle = math.degrees(math.atan(L*curvature))
        steer_angle = max(min(steer_angle, 30), -30)  # ステア最大角度制限

        # --- モータ・サーボ出力 ---
        try:
            togikai_drive.Accel(PWM_PARAM, pwm, time, int(target_v))
            togikai_drive.Steer(PWM_PARAM, pwm, time, int(steer_angle/30*100))
            print(f"[INFO] {i}: v={target_v}, steer={steer_angle:.1f}° (x={x:.2f}, y={y:.2f}, yaw={yaw:.1f})")
        except Exception as e:
            print(f"[ERROR] Accel/Steer出力失敗: {e}")
            traceback.print_exc()
            break

        # --- 仮想車両位置を更新（IMU+速度指令による簡易オドメトリ） ---
        v = target_v / 100.0  # [m/s] 仮定（100→1.0m/s, 50→0.5m/s）
        dt = 0.05  # 制御周期[s]
        yaw_rad = math.radians(yaw)
        x += math.cos(yaw_rad) * v * dt
        y += math.sin(yaw_rad) * v * dt
        time.sleep(dt)
except KeyboardInterrupt:
    print("[INFO] 手動停止")
except Exception as e:
    print(f"[ERROR] メインループ外エラー: {e}")
    traceback.print_exc()
