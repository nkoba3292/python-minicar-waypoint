"""
pure_pursuit_runner.py (リファクタ版)
1つのwaypointへ向かう走行制御関数を提供
- Pure Pursuit制御
- センサ連携（IMU・超音波）
- 例外復帰（スタック・逆走・スピン・障害物回避）
"""

import time
import sys
import math
import traceback
from datetime import datetime

sys.path.append('/home/pi/togikai/togikai_function/')

# グローバルインポート（サンプルコードと同様）
MODULES_AVAILABLE = False
togikai_drive = None
togikai_ultrasonic = None
PCA9685 = None

try:
    import togikai_drive
    import togikai_ultrasonic
    import PCA9685
    MODULES_AVAILABLE = True
    print("[INFO] ハードウェアモジュール正常インポート")
except ImportError as e:
    print(f"[WARN] モジュールインポート失敗（PCモード）: {e}")
    MODULES_AVAILABLE = False
    
    # モックモジュールを作成（PCでのテスト用）
    class MockModule:
        @staticmethod
        def ReadPWMPARAM(pwm):
            return [[1500, 1500, 1500], [1500, 1500, 1500]]  # デフォルト値
        @staticmethod
        def Accel(pwm_param, pwm, time_module, value):
            print(f"[MOCK] Accel: {value}")
        @staticmethod
        def Steer(pwm_param, pwm, time_module, value):
            print(f"[MOCK] Steer: {value}")
        @staticmethod
        def Mesure(gpio, time_module, trig_pin, echo_pin):
            return 50.0  # デフォルト距離値
    
    togikai_drive = MockModule()
    togikai_ultrasonic = MockModule()
    
    class MockPCA9685:
        def __init__(self, address=0x40, busnum=1):
            self.address = address
            self.busnum = busnum
        def set_pwm_freq(self, freq):
            print(f"[MOCK] PWM周波数設定: {freq}Hz")
        def set_pwm(self, channel, on, off):
            print(f"[MOCK] PWM ch{channel}: on={on}, off={off}")
    
    # PCA9685モジュールのモック
    class MockPCA9685Module:
        PCA9685 = MockPCA9685
    
    PCA9685 = MockPCA9685Module()
    print("[INFO] モックモジュールで動作中（PC環境）")

# --- ログ管理 ---
exception_log = []

def log_exception(exception_type, message, details=None):
    """例外処置ログを記録"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    log_entry = {
        'timestamp': timestamp,
        'type': exception_type,
        'message': message,
        'details': details or {}
    }
    exception_log.append(log_entry)
    print(f"[{timestamp}] [{exception_type}] {message}")
    if details:
        for key, value in details.items():
            print(f"  - {key}: {value}")

def get_exception_log():
    """例外処置ログを取得"""
    return exception_log

def save_exception_log(filename='exception_log.json'):
    """例外処置ログをファイルに保存"""
    import json
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(exception_log, f, indent=2, ensure_ascii=False)
        print(f"[INFO] 例外ログを保存: {filename}")
    except Exception as e:
        print(f"[ERROR] ログ保存失敗: {e}")

# --- グローバル初期化 ---
pwm = None
PWM_PARAM = None
imu_sensor = None
ultrasonic_available = False
GPIO = None

# --- 超音波センサー値（10Hz更新） ---
ultrasonic_distances = {
    'FR': None,   # 前方
    'LH': None,   # 左45度
    'RH': None,   # 右90度
    'RLH': None,  # 右45度
    'RRH': None   # 左90度
}
ultrasonic_thread = None
ultrasonic_thread_active = False

# --- 状態変数 ---
x = 0.0
y = 0.0
stuck_timer = None
stuck_state = False
reverse_timer = None
reverse_state = False
last_yaw = None
yaw_spin_timer = None
spin_sum = 0
spin_detected = False
obstacle_state = False

# --- 制御値（モニタリング用） ---
current_steer = 0
current_accel = 0

def get_current_steer():
    """現在のSTEER値を取得"""
    return current_steer

def get_current_accel():
    """現在のACCEL値を取得"""
    return current_accel

def initialize_hardware():
    """ハードウェア初期化"""
    global pwm, PWM_PARAM, imu_sensor, ultrasonic_available, GPIO
    
    # 超音波センサ・GPIO初期化（PWMより先に実行）
    try:
        if MODULES_AVAILABLE:
            import RPi.GPIO as GPIO_module
            GPIO = GPIO_module
            GPIO.setwarnings(False)  # GPIO警告を抑制
            GPIO.setmode(GPIO.BOARD)
            # 超音波センサ初期設定
            # Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
            t_list = [15,13,35,32,36]
            GPIO.setup(t_list, GPIO.OUT, initial=GPIO.LOW)
            # Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
            e_list = [26,24,37,31,38]
            GPIO.setup(e_list, GPIO.IN)
            ultrasonic_available = True
            print("[INFO] GPIO・超音波センサ初期化成功")
        else:
            # モック環境ではGPIOを使用しない
            print("[INFO] モック環境のためGPIO初期化をスキップ")
            ultrasonic_available = False
    except Exception as e:
        print(f"[WARN] GPIO・超音波センサ初期化失敗: {e}")
        import traceback
        traceback.print_exc()
        ultrasonic_available = False
    
    # PWM・モータ初期化
    try:
        pwm = PCA9685.PCA9685(address=0x40, busnum=1)
        pwm.set_pwm_freq(60)
        
        PWM_PARAM = togikai_drive.ReadPWMPARAM(pwm)
        
        # ガード処理（サンプルコードより）
        # Steer Right
        if PWM_PARAM[0][0] - PWM_PARAM[0][1] >= 100:
            PWM_PARAM[0][0] = PWM_PARAM[0][1] + 100
        # Steer Left
        if PWM_PARAM[0][1] - PWM_PARAM[0][2] >= 100:
            PWM_PARAM[0][2] = PWM_PARAM[0][1] - 100
        
        # PWM直接テスト（ステアリング中央値・アクセル停止値）
        try:
            steer_center = PWM_PARAM[0][1]
            pwm.set_pwm(0, 0, int(steer_center))
            time.sleep(0.5)
            
            accel_stop = PWM_PARAM[1][1]
            pwm.set_pwm(1, 0, int(accel_stop))
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] PWM直接テスト失敗: {e}")
            import traceback
            traceback.print_exc()
        
        # 操舵、駆動モーターの初期化（0に設定）
        print("[INFO] モーター初期化中... (Accel=0, Steer=0)")
        togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
        togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
        
        # スピコン初期化待機（ピピピ音確認用）
        print("[INFO] スピードコントローラー初期化待機中... (3秒)")
        print("       ※ ピピピ音が3回鳴ることを確認してください")
        time.sleep(3.0)
        
        print("[INFO] PWM・モータ初期化成功")
        print(f"[INFO] 最終PWM_PARAM: {PWM_PARAM}")
    except Exception as e:
        print(f"[ERROR] PWM初期化失敗: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # IMUセンサ初期化
    try:
        from IMU_sensor_bno055 import IMUSensorBNO055
        imu_sensor = IMUSensorBNO055()
        print("[INFO] IMUセンサ初期化成功")
    except Exception as e:
        print(f"[WARN] IMUセンサ初期化失敗: {e}")
        imu_sensor = None
    
    # 超音波センサー測定スレッドを起動
    start_ultrasonic_thread()
    
    return True

def start_ultrasonic_thread():
    """超音波センサー測定スレッドを起動（100ms周期 = 10Hz）"""
    global ultrasonic_thread, ultrasonic_thread_active, ultrasonic_distances, ultrasonic_available, GPIO
    
    if not ultrasonic_available:
        print("[INFO] 超音波センサー無効 - 測定スレッドは起動しません")
        return
    
    import threading
    ultrasonic_thread_active = True
    
    def ultrasonic_measurement_loop():
        """超音波センサー測定ループ（10Hz）"""
        global ultrasonic_distances
        
        def safe_measure(gpio, time_module, trig, echo, sensor_name):
            """安全な超音波センサー測定（負の値を防ぐ）"""
            try:
                value = togikai_ultrasonic.Mesure(gpio, time_module, trig, echo)
                # 生データを出力してタイムアウト／特殊値を確認
                print(f"[RAW] {sensor_name} Pin{trig}/{echo} -> {value}")
                if value is None or value < 0:
                    print(f"[WARN] {sensor_name}センサー異常値: {value} -> 999.9cm")
                    return 999.9  # エラー時は遠方扱い
                return value
            except Exception as e:
                print(f"[ERROR] {sensor_name}センサー測定失敗: {e}")
                return 999.9
        
        while ultrasonic_thread_active:
            try:
                start_time = time.time()
                
                # 5個のセンサーを測定（合計25-100ms）
                ultrasonic_distances['FR'] = safe_measure(GPIO, time, 15, 26, 'FR(前)')
                ultrasonic_distances['LH'] = safe_measure(GPIO, time, 13, 24, 'LH(左45°)')
                ultrasonic_distances['RH'] = safe_measure(GPIO, time, 32, 31, 'RH(右90°)')
                ultrasonic_distances['RLH'] = safe_measure(GPIO, time, 35, 37, 'RLH(右45°)')
                # togikai サンプルに合わせたピン順: TRIG=36, ECHO=38
                ultrasonic_distances['RRH'] = safe_measure(GPIO, time, 36, 38, 'RRH(左90°)')
                
                # 100ms周期を維持（測定時間を考慮）
                elapsed = time.time() - start_time
                sleep_time = max(0.001, 0.1 - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                print(f"[WARN] 超音波センサー測定エラー: {e}")
                time.sleep(0.1)
    
    # スレッド起動
    ultrasonic_thread = threading.Thread(target=ultrasonic_measurement_loop, daemon=True)
    ultrasonic_thread.start()
    print("[INFO] 超音波センサー測定スレッド起動（100ms周期 = 10Hz）")

def stop_ultrasonic_thread():
    """超音波センサー測定スレッドを停止"""
    global ultrasonic_thread_active, ultrasonic_thread
    
    if ultrasonic_thread is not None:
        ultrasonic_thread_active = False
        ultrasonic_thread.join(timeout=1.0)
        print("[INFO] 超音波センサー測定スレッド停止")

def get_ultrasonic_distances():
    """現在の超音波センサー値を取得"""
    return ultrasonic_distances.copy()

def goto_waypoint(waypoint, imu_offset=0.0, lookahead_dist=0.5, max_speed=70, timeout=30.0):
    """
    1つのwaypointへ向かう走行制御
    
    Args:
        waypoint: dict形式 {'x': ..., 'y': ..., 'v': ..., 'yaw': ...}
        imu_offset: IMUオフセット角度
        lookahead_dist: 前方注視距離[m]
        max_speed: 最大スロットル[0-100]
        timeout: タイムアウト時間[秒]
    
    Returns:
        'reached': waypoint到達
        'stopped': ユーザー停止
        'timeout': タイムアウト
        'error': エラー発生
    """
    global x, y
    global stuck_timer, stuck_state
    global reverse_timer, reverse_state
    global last_yaw, yaw_spin_timer, spin_sum, spin_detected
    global obstacle_state
    global current_steer, current_accel
    global pwm, PWM_PARAM, imu_sensor, ultrasonic_available, GPIO
    
    target_x = waypoint['x']*0.05-3.2
    target_y = waypoint['y']*0.05-1.5
    target_yaw = waypoint.get('yaw', 0)
    target_v = waypoint.get('v', max_speed)
    
    start_time = time.time()
    last_monitor_time = [start_time]  # リストで管理（ミュータブル）
    
    # ハードウェア初期化チェック
    if pwm is None or PWM_PARAM is None:
        print("[ERROR] PWM未初期化！initialize_hardware()を先に実行してください")
        log_exception('ERROR', 'PWM未初期化', {'pwm': str(pwm), 'PWM_PARAM': str(PWM_PARAM)})
        return 'error'
    
    print(f"[START] Waypoint制御開始: Target ({target_x:.2f}, {target_y:.2f}), Speed: {target_v}")
    print(f"[DEBUG] PWM初期化状態: pwm={pwm is not None}, PWM_PARAM={PWM_PARAM is not None}")
    
    # waypoint到達まで制御ループ
    while True:
        # タイムアウトチェック
        if time.time() - start_time > timeout:
            log_exception('TIMEOUT', 'Waypoint到達タイムアウト', {
                'timeout': f"{timeout}s",
                'elapsed': f"{time.time() - start_time:.1f}s",
                'target_x': f"{target_x:.2f}m",
                'target_y': f"{target_y:.2f}m"
            })
            print(f"[WARN] Waypoint到達タイムアウト")
            return 'timeout'
        
        try:
            # --- 超音波センサ値取得（別スレッドで測定済み） ---
            distances = get_ultrasonic_distances()
            FRdis = distances.get('FR')
            LHdis = distances.get('LH')
            RHdis = distances.get('RH')
            RLHdis = distances.get('RLH')
            RRHdis = distances.get('RRH')
            
            # --- IMU取得（高速版: get_essential） ---
            yaw = 0.0
            accel_val = None
            if imu_sensor is not None:
                try:
                    imu_data = imu_sensor.get_essential()  # 高速版（YAW+加速度のみ）
                    yaw = imu_data.get('yaw', 0) + imu_offset
                    accel_val = imu_data.get('accel', None)
                except Exception as e:
                    print(f"[WARN] IMU値取得失敗: {e}")
            
            # --- 衝突・スタック判定 ---
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
                log_exception('STUCK', '衝突・スタック復帰開始', {
                    'us_stuck': us_stuck,
                    'accel_stuck': accel_stuck,
                    'stuck_duration': f"{now - stuck_timer:.1f}s"
                })
                print("[STUCK] 1秒バック (duty=-100)")
                current_accel = -100
                current_steer = 0
                togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
                togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
                time.sleep(1.0)
                print("[STUCK] 1秒右旋回 (duty=100)")
                current_accel = -100
                current_steer = 100
                togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
                togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
                time.sleep(1.0)
                current_accel = 0
                current_steer = 0
                togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
                togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
                print("[STUCK] 前方クリア待機")
                while True:
                    # 別スレッドで測定済みの値を取得
                    distances = get_ultrasonic_distances()
                    FRdis = distances.get('FR')
                    if FRdis is not None and FRdis > 10.0:
                        log_exception('STUCK', '前方クリア→復帰', {'FRdis': f"{FRdis:.1f}cm"})
                        print("[STUCK] 前方クリア、purepursuit復帰")
                        break
                    time.sleep(0.1)
                stuck_state = False
                stuck_timer = None
                continue
            
            # --- 逆走判定 ---
            dx = target_x - x
            dy = target_y - y
            wp_dir = math.degrees(math.atan2(dy, dx))
            yaw_diff = ((yaw - wp_dir + 180) % 360) - 180
            
            if abs(yaw_diff) >= 90:
                if reverse_timer is None:
                    reverse_timer = time.time()
                elif time.time() - reverse_timer > 5.0 and not reverse_state:
                    reverse_state = True
                    log_exception('REVERSE', '逆走判定→その場左折開始', {
                        'yaw': f"{yaw:.1f}°",
                        'wp_dir': f"{wp_dir:.1f}°",
                        'yaw_diff': f"{yaw_diff:.1f}°",
                        'reverse_duration': f"{time.time() - reverse_timer:.1f}s"
                    })
                    print(f"[REVERSE] 逆走判定: YAW差={yaw_diff:.1f}°")
            else:
                reverse_timer = None
                reverse_state = False
            
            # --- 逆走復帰動作 ---
            if reverse_state:
                print("[REVERSE] その場左折")
                current_accel = 0
                current_steer = -100
                togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
                togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
                time.sleep(0.2)
                if us_stuck and accel_stuck:
                    log_exception('REVERSE', '左折中に壁衝突→スタック復帰へ移行', {
                        'us_stuck': us_stuck,
                        'accel_stuck': accel_stuck
                    })
                    print("[REVERSE] 左折中に壁衝突→スタック復帰へ")
                    reverse_state = False
                    reverse_timer = None
                    stuck_state = True
                    continue
                yaw_diff_new = ((yaw - wp_dir + 180) % 360) - 180
                if abs(yaw_diff_new) < 60:
                    log_exception('REVERSE', '進行方向復帰→purepursuit継続', {
                        'yaw_diff_new': f"{yaw_diff_new:.1f}°"
                    })
                    print(f"[REVERSE] 進行方向復帰")
                    reverse_state = False
                    reverse_timer = None
                continue
            
            # --- 異常旋回判定 ---
            if last_yaw is not None:
                yaw_delta = abs(yaw - last_yaw)
                if yaw_delta > 180:
                    yaw_delta = 360 - (yaw_delta % 360)
                if yaw_spin_timer is None:
                    yaw_spin_timer = time.time()
                    spin_sum = 0
                spin_sum += yaw_delta
                if spin_sum >= 720 and (time.time() - yaw_spin_timer) < 1.0:
                    log_exception('SPIN', '異常旋回検出→再計算', {
                        'spin_sum': f"{spin_sum:.1f}°",
                        'duration': f"{time.time() - yaw_spin_timer:.1f}s",
                        'yaw': f"{yaw:.1f}°"
                    })
                    print(f"[SPIN] 異常旋回検出: ΔYAW={spin_sum:.1f}°")
                    spin_detected = True
                if (time.time() - yaw_spin_timer) >= 1.0:
                    yaw_spin_timer = time.time()
                    spin_sum = 0
            else:
                spin_sum = 0
                yaw_spin_timer = time.time()
            last_yaw = yaw
            
            if spin_detected:
                log_exception('SPIN', '異常旋回から復帰→waypoint再アプローチ', {
                    'current_x': f"{x:.2f}m",
                    'current_y': f"{y:.2f}m"
                })
                print("[SPIN] 異常旋回からの復帰: 現在位置から再計算")
                spin_detected = False
                # waypointへの再アプローチ
                continue
            
            # --- 障害物回避判定 ---
            obstacle_detected = False
            steer_dir = 0
            if FRdis is not None and FRdis < 20.0:
                obstacle_detected = True
                # 左右の超音波センサーを比較して、より広い方向に切る
                if LHdis is not None and RHdis is not None:
                    steer_dir = -100 if LHdis > RHdis else 100
                elif LHdis is not None:
                    steer_dir = -100
                elif RHdis is not None:
                    steer_dir = 100
                else:
                    # 左右のセンサーがない場合はデフォルトで左
                    steer_dir = -100
            
            # --- 障害物回避動作 ---
            if obstacle_detected or obstacle_state:
                if not obstacle_state:  # 初回検出時のみログ
                    log_exception('OBSTACLE', '障害物回避開始→徐行', {
                        'FRdis': f"{FRdis:.1f}cm" if FRdis else 'N/A',
                        'LHdis': f"{LHdis:.1f}cm" if LHdis else 'N/A',
                        'RHdis': f"{RHdis:.1f}cm" if RHdis else 'N/A',
                        'steer_dir': steer_dir
                    })
                print(f"[OBSTACLE] 障害物回避: 徐行duty=70, steer={steer_dir}")
                print(f"[DEBUG] 障害物回避前: current_accel={current_accel}, current_steer={current_steer}")
                current_accel = 70  # 前進（障害物回避時は徐行）
                current_steer = steer_dir
                print(f"[DEBUG] 障害物回避後: current_accel={current_accel}, current_steer={current_steer}")
                print(f"[DEBUG] PWM_PARAM値: STEER_R={PWM_PARAM[0][0]}, STEER_C={PWM_PARAM[0][1]}, STEER_L={PWM_PARAM[0][2]}")
                print(f"[DEBUG] PWM_PARAM値: ACCEL_F={PWM_PARAM[1][0]}, ACCEL_S={PWM_PARAM[1][1]}, ACCEL_R={PWM_PARAM[1][2]}")
                
                # togikai_driveをスキップして、PWM直接送信でテスト
                print(f"[DEBUG] === PWM直接送信テスト ===")
                # ステアリング計算（符号を逆転）
                if current_steer >= 0:
                    # 正の値（右）→ LEFT方向に計算（逆転）
                    steer_pwm = int(PWM_PARAM[0][1] + (PWM_PARAM[0][2] - PWM_PARAM[0][1])*current_steer/100)
                else:
                    # 負の値（左）→ RIGHT方向に計算（逆転）
                    steer_pwm = int(PWM_PARAM[0][1] - (PWM_PARAM[0][1] - PWM_PARAM[0][0])*abs(current_steer)/100)
                
                # アクセル計算（前進/後退対応）
                if current_accel >= 0:
                    # 正の値 → 前進（FORWARD方向）
                    accel_pwm = int(PWM_PARAM[1][1] - (PWM_PARAM[1][1] - PWM_PARAM[1][0])*current_accel/100)
                else:
                    # 負の値 → 後退（REVERSE方向）
                    accel_pwm = int(PWM_PARAM[1][1] + (PWM_PARAM[1][2] - PWM_PARAM[1][1])*abs(current_accel)/100)
                
                print(f"[DEBUG] 計算結果: steer_pwm={steer_pwm}, accel_pwm={accel_pwm}")
                print(f"[DEBUG] PWM送信: ch=0 (STEER) -> {steer_pwm}")
                pwm.set_pwm(0, 0, steer_pwm)
                print(f"[DEBUG] PWM送信: ch=1 (ACCEL) -> {accel_pwm}")
                pwm.set_pwm(1, 0, accel_pwm)
                print(f"[DEBUG] PWM直接送信完了")
                
                # 元のtogikai_drive呼び出し（コメントアウト）
                # togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
                # togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
                
                # --- PWM直接テスト（一時的にコメントアウト） ---
                # print(f"[DEBUG] PWM直接テスト開始...")
                # steer_pwm_direct = PWM_PARAM[0][2] if current_steer < 0 else PWM_PARAM[0][0]
                # accel_pwm_direct = int(PWM_PARAM[1][1] - (PWM_PARAM[1][1] - PWM_PARAM[1][0])*70/100)
                # print(f"[DEBUG] PWM直接送信: ch=0 (STEER) -> {steer_pwm_direct}, ch=1 (ACCEL) -> {accel_pwm_direct}")
                # pwm.set_pwm(0, 0, steer_pwm_direct)
                # pwm.set_pwm(1, 0, accel_pwm_direct)
                # print(f"[DEBUG] PWM直接テスト完了")
                
                obstacle_state = True
                if FRdis is not None and FRdis > 30.0:
                    log_exception('OBSTACLE', '障害物消失→purepursuit復帰', {
                        'FRdis': f"{FRdis:.1f}cm"
                    })
                    print("[OBSTACLE] 障害物消失→purepursuit復帰")
                    obstacle_state = False
                else:
                    print(f"[DEBUG] 障害物継続中: FRdis={FRdis}, 0.5秒待機してcontinue")
                    # モーター出力を維持するため、PWM直接再送信
                    print(f"[DEBUG] 再送信前: accel={current_accel}, steer={current_steer}")
                    
                    # ステアリング計算（符号逆転版）
                    if current_steer >= 0:
                        steer_pwm = int(PWM_PARAM[0][1] + (PWM_PARAM[0][2] - PWM_PARAM[0][1])*current_steer/100)
                    else:
                        steer_pwm = int(PWM_PARAM[0][1] - (PWM_PARAM[0][1] - PWM_PARAM[0][0])*abs(current_steer)/100)
                    
                    # アクセル計算（前進/後退対応）
                    if current_accel >= 0:
                        # 正の値 → 前進（FORWARD方向）
                        accel_pwm = int(PWM_PARAM[1][1] - (PWM_PARAM[1][1] - PWM_PARAM[1][0])*current_accel/100)
                    else:
                        # 負の値 → 後退（REVERSE方向）
                        accel_pwm = int(PWM_PARAM[1][1] + (PWM_PARAM[1][2] - PWM_PARAM[1][1])*abs(current_accel)/100)
                    
                    pwm.set_pwm(0, 0, steer_pwm)
                    pwm.set_pwm(1, 0, accel_pwm)
                    print(f"[DEBUG] PWM再送信完了: steer_pwm={steer_pwm}, accel_pwm={accel_pwm}")
                    
                    time.sleep(0.5)  # 0.1秒→0.5秒に延長
                    continue
            
            # --- Pure Pursuit制御 ---
            dx = target_x - x
            dy = target_y - y
            dist = math.hypot(dx, dy)
            
            print(f"[DEBUG] Pure Pursuit: dist={dist:.3f}m, target=({target_x:.2f},{target_y:.2f}), current=({x:.2f},{y:.2f})")
            
            # waypoint到達判定
            if dist < lookahead_dist:
                log_exception('REACHED', 'Waypoint到達', {
                    'distance': f"{dist:.2f}m",
                    'target_x': f"{target_x:.2f}m",
                    'target_y': f"{target_y:.2f}m",
                    'current_x': f"{x:.2f}m",
                    'current_y': f"{y:.2f}m"
                })
                print(f"[INFO] Waypoint到達 (dist={dist:.2f}m)")
                togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
                togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
                return 'reached'
            
            # 車両座標系に変換
            local_x = math.cos(math.radians(-yaw))*dx - math.sin(math.radians(-yaw))*dy
            local_y = math.sin(math.radians(-yaw))*dx + math.cos(math.radians(-yaw))*dy
            
            # 前方注視点が後方ならスキップ
            if local_x < 0:
                print(f"[DEBUG] 注視点が後方 (local_x={local_x:.2f}) - STEER/ACCEL維持 (STEER: {current_steer}, ACCEL: {current_accel})")
                time.sleep(0.05)
                continue
            
            # 操舵角計算
            L = 0.25  # ホイールベース[m]
            curvature = 2*local_y/(lookahead_dist**2)
            steer_angle = math.degrees(math.atan(L*curvature))
            steer_angle = max(min(steer_angle, 30), -30)
            
            # モータ・サーボ出力
            current_accel = int(target_v)
            current_steer = int(steer_angle/30*100)
            
            print(f"[DEBUG] モーター出力: ACCEL={current_accel}, STEER={current_steer}, steer_angle={steer_angle:.1f}°")
            print(f"[DEBUG] togikai_drive.Accel呼び出し中...")
            togikai_drive.Accel(PWM_PARAM, pwm, time, current_accel)
            print(f"[DEBUG] togikai_drive.Steer呼び出し中...")
            togikai_drive.Steer(PWM_PARAM, pwm, time, current_steer)
            print(f"[DEBUG] モーター出力完了")
            
            # 1秒ごと（または初回）にSTEER/ACCEL値を表示
            current_time = time.time()
            if current_time - last_monitor_time[0] >= 1.0:
                elapsed = current_time - start_time
                print(f"[{elapsed:.1f}s] 🎮 STEER: {current_steer:+4d} | 🚀 ACCEL: {current_accel:3d} | Pos: ({x:.2f}, {y:.2f})")
                last_monitor_time[0] = current_time
            
            # 簡易オドメトリ更新
            v = target_v / 100.0
            dt = 0.05
            yaw_rad = math.radians(yaw)
            x += math.cos(yaw_rad) * v * dt
            y += math.sin(yaw_rad) * v * dt
            time.sleep(dt)
            
        except KeyboardInterrupt:
            log_exception('USER_STOP', 'ユーザー手動停止（KeyboardInterrupt）', {})
            print("[INFO] ユーザー停止")
            togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
            togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
            return 'stopped'
        except Exception as e:
            log_exception('ERROR', 'goto_waypoint内で予期しないエラー発生', {
                'error': str(e),
                'traceback': traceback.format_exc()
            })
            print(f"[ERROR] goto_waypoint内エラー: {e}")
            traceback.print_exc()
            return 'error'

def reset_position(start_x=0.0, start_y=0.0):
    """現在位置をリセット"""
    global x, y
    x = start_x
    y = start_y
    print(f"[INFO] 位置リセット: x={x:.2f}, y={y:.2f}")

# スタンドアロン実行用（テスト）
if __name__ == "__main__":
    if initialize_hardware():
        # テスト用waypoint
        test_waypoint = {'x': 100, 'y': 100, 'v': 50, 'yaw': 0}
        result = goto_waypoint(test_waypoint)
        print(f"[INFO] 結果: {result}")
