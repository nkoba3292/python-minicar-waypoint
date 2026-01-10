"""
IMU制御自律走行プログラム v0.6
- Yawキャリブレーション対応
- 位置推定機能
- 予選/本選モード対応
- 車輪速センサ統合（74HC590カウンタ）
- 白線検出機能有効化（74HC74ラッチ回路）

v0.6: v0.5からの改良版
- searchingモードの制御ロジック変更
  - Yaw制御・座標制御を使用しない
  - 完全にtogikai制御ロジック（壁追従）で探索
  - WP到達までは壁センサーのみで走行

v0.5からの継承:
- Yaw方位処理の2段階分離（初期位置補正→回転方向逆転）
- 障害物回避モードのGPIO設定修正（BCM番号完全統一）
- 制御モード表示の改善（recovery時に経過時間表示）
- 座標ジャンプ問題修正（復帰時に座標維持）
- モーター目標速度表示追加（Tgt表示）
- searchingモード追加（復帰後のWP探索状態を明確化）

v0.4からの継承:
- 白線検出機能の有効化（ENABLE_LINE_DETECTION = True）
- 2種類の白線パターンを識別:
  1. スタートライン: 単発検出3回（2m間隔×3本）
  2. 中間マット: 連続検出5回以上（白色ゾーン）
- チェックポイント判別によるWP位置補正
- 回避モード時の復帰処理に使用

v0.3からの継承:
  - 5周期ごとにパルスカウントから速度計算（車輪直径58mm、PPR=1.0）
  - 位置推定でIMU加速度積分の代わりに車輪速度×Yaw角を使用
  - 目標速度>10 かつ パルス=0 で衝突検出
"""
import os
import sys
sys.path.append('/home/pi/togikai/togikai_function/')
import togikai_drive
import togikai_ultrasonic
import signal
import RPi.GPIO as GPIO
#import Adafruit_PCA9685
import PCA9685
import time
import numpy as np
import json
import math
from wheel_counter_74hc590 import Counter74HC590

# File: 02_IMUPLUS_togikai_sample_new_v0.6.py
# v0.6: 探索モード制御改善版

# GPIOピン番号の指示方法（sensor_placement_testと同じBCMモード）
GPIO.setwarnings(False)  # 警告を無効化
GPIO.cleanup()  # 以前の設定をクリーンアップ
GPIO.setmode(GPIO.BCM)

#超音波センサ初期設定（BCMピン番号）
# Triger -- Fr:22, FrLH:27, RrLH:19, FrRH:12, RrRH:16 (BOARD: 15,13,35,32,36)
t_list=[22,27,19,12,16]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
# Echo -- Fr:7, FrLH:8, RrLH:26, FrRH:6, RrRH:20 (BOARD: 26,24,37,31,38)
e_list=[7,8,26,6,20]
GPIO.setup(e_list,GPIO.IN)

# フォトリフレクタ + 74HC74ラッチ回路初期設定（BCMピン番号）
# sensor_placement_testと同じ配線
LATCH_OUTPUT = 23  # BCM 23 (BOARD 16) - ラッチ出力読み取り
LATCH_CLEAR = 18   # BCM 18 (BOARD 12) - ラッチクリア信号
GPIO.setup(LATCH_OUTPUT, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
GPIO.setup(LATCH_CLEAR, GPIO.OUT, initial=GPIO.HIGH)  # CLR=HIGH(非クリア状態)

#PWM制御の初期設定
##モータドライバ:PCA9685のPWMのアドレスを設定
pwm = PCA9685.PCA9685(address=0x40,busnum=1)
##動作周波数を設定
pwm.set_pwm_freq(60)

#アライメント調整済みPWMパラメータ読み込み
PWM_PARAM = togikai_drive.ReadPWMPARAM(pwm)

#Gard 210523
#Steer Right
if PWM_PARAM[0][0] - PWM_PARAM[0][1] >= 100: #No change!
    PWM_PARAM[0][0] = PWM_PARAM[0][1] + 100  #No change!
    
#Steer Left
if PWM_PARAM[0][1] - PWM_PARAM[0][2] >= 100: #No change!
    PWM_PARAM[0][2] = PWM_PARAM[0][1] - 100  #No change!

# ---- モード選択 ----
print("=== レースモード選択 ===")
print("1: 予選 (3周+駐車)")
print("2: 本選レーン1")
print("3: 本選レーン2")
print("4: 本選レーン3")
mode = input("モード選択 (1-4): ")

if mode == '1':
    WAYPOINT_FILE = 'waypoints_qualifying.json'
elif mode == '2':
    WAYPOINT_FILE = 'waypoints_final1.json'
elif mode == '3':
    WAYPOINT_FILE = 'waypoints_final2.json'
elif mode == '4':
    WAYPOINT_FILE = 'waypoints_final3.json'
else:
    print("無効な選択。デフォルト: 予選モード")
    WAYPOINT_FILE = 'waypoints_qualifying.json'

# ---- Waypoint読み込み ----
with open(WAYPOINT_FILE, 'r') as f:
    waypoints = json.load(f)
print(f"Loaded {len(waypoints)} waypoints from {WAYPOINT_FILE}")

# Waypoint制御変数
idx = 1  # 現在の目標waypoint index（WP0スタート→WP1を目標）
n_wp = len(waypoints)  # 総waypoint数
lap = 1  # 周回数（後で使用）
current_x, current_y = 0.0, 0.0  # 現在位置（グローバル座標）[m]
wp_start_time = None  # 各waypoint開始時刻
accumulated_distance = 0.0  # 累積移動距離[m]
last_update_time = None  # 前回の位置更新時刻
velocity_x, velocity_y = 0.0, 0.0  # 速度（グローバル座標）[m/s]
initial_yaw_offset = 0.0  # Yawオフセット（キャリブレーション値）
position_tracking_enabled = True  # 位置推定有効化フラグ
narrow_passage_active = False  # 狭路通過中フラグ

# 車輪速センサ関連変数
wheel_counter = None  # Counter74HC590インスタンス
wheel_speed_buffer = []  # 速度計測バッファ（5周期分）
WHEEL_SAMPLE_CYCLES = 5  # 速度計測周期数
wheel_cycle_count = 0  # 現在の周期カウント
wheel_last_total = 0  # 前回の累積カウント
wheel_last_time = 0.0  # 前回の速度計測時刻
wheel_velocity_mps = 0.0  # 車輪速度 [m/s]
WHEEL_DIAMETER = 0.0557  # 車輪直径 [m] (55.7mm) - 実測周長175mm
WHEEL_PPR = 2.0  # 1回転あたりのパルス数（SK1816ラッチ型: N極+S極で2パルス/回転）
collision_detected = False  # 衝突検出フラグ
zero_pulse_count = 0  # パルス0の連続カウント（衝突判定用）

# 制御モード管理
control_mode = "normal"  # 制御モード: normal, narrow_passage, obstacle_avoidance, recovery, searching
avoidance_start_wp = None  # 回避開始時のwaypoint index
avoidance_start_time = None  # 回避開始時刻
side_avoidance_active = False  # 左右回避中フラグ
side_avoidance_direction = None  # 回避方向: "left" or "right"

# 白線検出管理（フォトリフレクタ）
line_count = 0  # 累積白線検出数
line_detections = []  # [(時刻, 連続検出数), ...]
last_line_time = 0.0  # 最後の白線検出時刻
consecutive_line_count = 0  # 連続検出カウント
target_checkpoint = None  # 目標チェックポイント: "start_line" or "middle_mat"
checkpoint_wp_idx = None  # チェックポイントのWP index

# レースモード判定（ファイル名から）
if 'final' in WAYPOINT_FILE:
    race_mode = "final"
    TIME_LIMIT = 150.0  # 本選: 2分30秒
    print(f"[本選モード] タイムリミット: {TIME_LIMIT}秒")
else:
    race_mode = "qualifying"
    TIME_LIMIT = None  # 予選: 制限なし
    print("[予選モード] 3周回+駐車")

race_finished = False  # レース終了フラグ

# デバッグモード・安全距離の設定
DEBUG_MODE = True  # デバッグモード（True=ホールIC動作確認用）
SPEED_SCALE = 1.0   # 速度スケール（デバッグ時は0.33など）
SAFE_DIST_FRONT = 30  # 前方安全距離（cm）
USE_NARROW_PASSAGE = True  # 狭路判定を使用する
ENABLE_LINE_DETECTION = False  # 白線検出機能有効化

#これはtogikaiのパラメータ
#前壁との最小距離
Cshort = 20
#右左折判定基準
short = 70
#モーター出力
FORWARD_S = 70 #<=100
FORWARD_C = 70 #<=100
REVERSE = -60 #<=100
#Stear（新車両用に修正）
LEFT = -90 #<=100  # 修正: 負の値で左
CENTER = 0
RIGHT = 90 #<=100  # 修正: 正の値で右
#データ記録用配列作成（13列に拡張: 時刻、センサー5個、yaw、accel_x、accel_y、モード、idx、line_count、distance_to_wp）
d = np.zeros(13)
#操舵、駆動モーターの初期化
togikai_drive.Accel(PWM_PARAM,pwm,time,0)
togikai_drive.Steer(PWM_PARAM,pwm,time,0)

# 追加IMU初期化
from IMU_sensor_bno055 import IMUSensorBNO055
imu = IMUSensorBNO055(port='/dev/serial0', baudrate=115200)

# === 車輪速センサ初期化 ===
print('=== 車輪速センサ初期化 ===')
# 74HC590ピン定義（BCMピン番号 - sensor_placement_testと同じ）
WHEEL_LATCH_PIN = 24   # RCLK (BOARD 18)
WHEEL_DATA_PINS = [13, 21, 25]  # D0, D1, D2 (BOARD 33, 40, 22)
WHEEL_CLEAR_PIN = 17   # CCLR (BOARD 11)
WHEEL_DEBOUNCE_TIME = 0.2  # デバウンス時間 [s] (200ms)

try:
    wheel_counter = Counter74HC590(WHEEL_LATCH_PIN, WHEEL_DATA_PINS, WHEEL_CLEAR_PIN, 
                                   debounce_time=WHEEL_DEBOUNCE_TIME)
    wheel_last_total = wheel_counter.total_count
    wheel_last_time = time.time()
    print(f'[車輪速センサ] 初期化完了 (初期値: {wheel_counter.last_value}, デバウンス: {WHEEL_DEBOUNCE_TIME*1000:.0f}ms)')
except Exception as e:
    print(f'[車輪速センサ] 初期化失敗: {e}')
    print('[WARN] 車輪速センサなしで続行します')
    wheel_counter = None

# === Yawキャリブレーション（3秒静止） ===
print('=== Yawキャリブレーション開始（3秒間静止）===')

yaw_samples = []
calib_start_time = time.time()

while time.time() - calib_start_time < 3.0:
    try:
        yaw_data = imu.get_euler_angles()
        if yaw_data and 'yaw' in yaw_data:
            yaw_samples.append(yaw_data['yaw'])
    except:
        pass
    time.sleep(0.1)

# 平均Yaw計算
if len(yaw_samples) > 0:
    measured_yaw = sum(yaw_samples) / len(yaw_samples)
else:
    measured_yaw = 0.0
    print('[WARN] Yaw測定失敗、オフセット=0として継続')

# WP0のYawを基準にオフセット計算
target_yaw = waypoints[0].get('yaw', 180.0)
initial_yaw_offset = target_yaw - measured_yaw

# 360°を跨ぐ場合の補正
if initial_yaw_offset > 180:
    initial_yaw_offset -= 360
elif initial_yaw_offset < -180:
    initial_yaw_offset += 360

print(f'[CALIB] 測定:{measured_yaw:.1f}° 目標:{target_yaw:.1f}° オフセット:{initial_yaw_offset:+.1f}° 完了')

#一時停止（Enterを押すとプログラム実行開始）
print('Press Enter to start')
input()

#開始時間
start_time = time.time()


# ===== 白線検出・チェックポイント判別関数 =====

def reset_latch():
    """ラッチ回路をリセット（CLRパルス送信）"""
    GPIO.output(LATCH_CLEAR, GPIO.LOW)
    time.sleep(0.001)  # 1msパルス
    GPIO.output(LATCH_CLEAR, GPIO.HIGH)

def check_line_detection():
    """
    白線検出チェック（ポーリング）
    戻り値: True=白線検出, False=検出なし
    """
    if GPIO.input(LATCH_OUTPUT) == GPIO.HIGH:
        reset_latch()
        return True
    return False

def identify_checkpoint_pattern(detections_history, current_time):
    """
    白線パターンからチェックポイントを判別
    
    Args:
        detections_history: [(時刻, 連続検出数), ...] のリスト
        current_time: 現在時刻
    
    Returns:
        "start_line": スタートライン検出
        "middle_mat": 中間地点マット検出
        None: 判別不可
    """
    if not detections_history:
        return None
    
    # 最新の検出パターンを確認
    latest = detections_history[-1]
    latest_consecutive = latest[1]
    
    # 連続検出が多い = 中間地点の連続マット
    if latest_consecutive >= 5:
        print(f"[パターン判別] 中間マット検出（連続{latest_consecutive}回）")
        return "middle_mat"
    
    # 単発検出が3回 = スタートライン（2m間隔×3本）
    # 過去10秒以内の単発検出（連続1回）を確認
    recent_singles = [d for d in detections_history 
                      if d[1] == 1 and current_time - d[0] < 10.0]
    
    if len(recent_singles) >= 3:
        print(f"[パターン判別] スタートライン検出（単発3回）")
        return "start_line"
    
    return None

def find_waypoint_by_checkpoint(checkpoint_type):
    """
    チェックポイント種類からWPインデックスを検索
    
    Args:
        checkpoint_type: "start_line" or "middle_mat"
    
    Returns:
        WPインデックス or None
    """
    for wp_idx, wp in enumerate(waypoints):
        if wp.get('checkpoint') == checkpoint_type:
            return wp_idx
    return None

def get_recovery_waypoint_qualifying(current_wp_idx):
    """
    予選モード専用: 現在のWPインデックスから復帰先WPを決定
    
    Args:
        current_wp_idx: 回避発生時のWPインデックス
    
    Returns:
        復帰先WPインデックス
    """
    # WP範囲と復帰先のマッピング
    recovery_map = [
        (3, 8, 9),     # WP3-8 → WP9
        (10, 23, 24),  # WP10-23 → WP24
        (27, 32, 33),  # WP27-32 → WP33
        (34, 47, 48),  # WP34-47 → WP48
        (51, 56, 57),  # WP51-56 → WP57
        (58, 71, 72),  # WP58-71 → WP72
    ]
    
    for start, end, recovery in recovery_map:
        if start <= current_wp_idx <= end:
            return recovery
    
    # 該当範囲外の場合はNone（従来のチェックポイント方式を使用）
    return None



#ここから走行用プログラム
try:
    # 初期化
    wp_start_time = time.time()
    last_update_time = time.time()
    accumulated_distance = 0.0
    # WP0座標で初期化（スタート地点がWP0）
    current_x = waypoints[0]['x']
    current_y = waypoints[0]['y']
    velocity_x, velocity_y = 0.0, 0.0
    position_tracking_enabled = True  # 最初から位置推定有効
    
    while True:
        # レース終了チェック
        if race_finished:
            print("[レース終了] プログラム終了")
            break
            
        #Frセンサ距離 (BCMピン番号)
        FRdis = togikai_ultrasonic.Mesure(GPIO,time,22,7)  # Trig=BCM22(BOARD15), Echo=BCM7(BOARD26)
        #FrLHセンサ距離
        LHdis = togikai_ultrasonic.Mesure(GPIO,time,27,8)  # Trig=BCM27(BOARD13), Echo=BCM8(BOARD24)
        #FrRHセンサ距離
        RHdis = togikai_ultrasonic.Mesure(GPIO,time,12,6)  # Trig=BCM12(BOARD32), Echo=BCM6(BOARD31)
        #RrLHセンサ距離
        RLHdis = togikai_ultrasonic.Mesure(GPIO,time,19,26)  # Trig=BCM19(BOARD35), Echo=BCM26(BOARD37)
        #RrRHセンサ距離
        RRHdis = togikai_ultrasonic.Mesure(GPIO,time,16,20)  # Trig=BCM16(BOARD36), Echo=BCM20(BOARD38)
        
        # 超音波センサー異常値フィルタ（-1000〜500cmの範囲外を200cmに補正）
        def filter_ultrasonic(value):
            if value < -1000 or value > 500:
                return 200.0  # 異常値は200cmとして扱う
            return value
        
        FRdis = filter_ultrasonic(FRdis)
        LHdis = filter_ultrasonic(LHdis)
        RHdis = filter_ultrasonic(RHdis)
        RLHdis = filter_ultrasonic(RLHdis)
        RRHdis = filter_ultrasonic(RRHdis)

        
        # ===== 白線検出処理 =====
        if ENABLE_LINE_DETECTION:
            current_time = time.time()
            
            # ラッチ状態チェック
            if check_line_detection():
                line_count += 1
                
                # 連続検出判定（前回から0.3秒以内なら連続）
                if current_time - last_line_time < 0.3:
                    consecutive_line_count += 1
                else:
                    # 前回の連続検出終了 → パターン記録
                    if consecutive_line_count > 0:
                        line_detections.append((last_line_time, consecutive_line_count))
                        print(f"[白線] 連続{consecutive_line_count}回検出")
                    consecutive_line_count = 1
                
                last_line_time = current_time
                print(f"[白線検出] 累計{line_count}本目")
            
            # 連続検出が途切れた場合の記録
            elif consecutive_line_count > 0 and current_time - last_line_time > 0.5:
                line_detections.append((last_line_time, consecutive_line_count))
                print(f"[白線] 連続{consecutive_line_count}回検出終了")
                
                # チェックポイント判別
                detected_checkpoint = identify_checkpoint_pattern(line_detections, current_time)
                if detected_checkpoint:
                    print(f"[チェックポイント] {detected_checkpoint} 確定")
                    
                    # 回避モード中ならWP復帰処理
                    if control_mode == "obstacle_avoidance" and target_checkpoint == detected_checkpoint:
                        checkpoint_wp_idx = find_waypoint_by_checkpoint(detected_checkpoint)
                        if checkpoint_wp_idx is not None:
                            control_mode = "recovery"
                            print(f"[復帰] {detected_checkpoint}検出、WP{checkpoint_wp_idx}へ復帰準備")
                
                consecutive_line_count = 0
        else:
            # 白線検出無効化時はチェックポイント復帰を使用しない
            current_time = time.time()
        
        
        #IMUから現在のヨー角・加速度xy取得（Yawオフセット適用、方位反転）
        raw_yaw = imu.get_euler_angles()['yaw']
        # 初期位置補正 → 回転方向を逆転
        adjusted_yaw = (raw_yaw + initial_yaw_offset) % 360
        current_yaw = (360.0 - adjusted_yaw) % 360
        accel_x = imu.get_acceleration()['x']
        accel_y = imu.get_acceleration()['y']
        
        # ===== 車輪速センサ計測（毎ループ） =====
        pulse_delta_display = 0  # 表示用のデルタパルス
        if wheel_counter is not None:
            current_time_wheel = time.time()
            
            current_value, delta_pulses, current_total = wheel_counter.read_incremental()
            pulse_delta_display = delta_pulses  # 表示用に保存（補正前）
            
            # ===== パルスカウント誤検出補正 =====
            # 1周期あたり0または+1が正常、+2や+3は誤検出なので+1に補正
            if delta_pulses >= 2:
                pulse_delta_original = delta_pulses
                delta_pulses = 1  # +2または+3 → +1に補正
                if DEBUG_MODE:
                    print(f"[補正] パルス {pulse_delta_original} → 1 に補正")
            
            # デバッグ: 読み取り結果
            if DEBUG_MODE:
                print(f"[READ] current={current_value} delta={delta_pulses} total={current_total} last_time_diff={current_time_wheel-wheel_last_time:.3f}s")
            
            # 初回時刻設定
            if wheel_last_time == 0.0:
                wheel_last_time = current_time_wheel
            
            # 時間差分
            delta_time_wheel = current_time_wheel - wheel_last_time
            
            # 速度計算: v = (パルス数 / PPR) * 車輪周長 / 時間
            # read_incremental()が既に前回からの差分を返すので、そのまま使用
            if delta_time_wheel > 0.001 and delta_pulses > 0:
                # パルスがある場合は速度を更新
                wheel_circumference = math.pi * WHEEL_DIAMETER
                rotations = delta_pulses / WHEEL_PPR
                distance_traveled = rotations * wheel_circumference
                wheel_velocity_mps = distance_traveled / delta_time_wheel
                
                # デバッグ表示
                if DEBUG_MODE:
                    print(f"[車輪速] Δpulse={delta_pulses} Δt={delta_time_wheel:.3f}s v={wheel_velocity_mps:.3f}m/s")
            elif delta_pulses == 0 and delta_time_wheel > 0.1:
                # 0.1秒以上パルスがない場合は速度を減衰
                wheel_velocity_mps *= 0.9
            
            # 時刻更新（毎ループ）
            wheel_last_time = current_time_wheel

        # 現在 waypoint取得
        wp = waypoints[idx]
        wp_x, wp_y = wp['x'], wp['y']
        target_speed = wp.get('v', 50)  # ウェイポイント指定速度、デフォルト50
        target_yaw = wp.get('yaw', None)  # ウェイポイント指定方位角

        # デバッグモード時の速度調整
        if DEBUG_MODE:
            target_speed = target_speed * SPEED_SCALE
            if target_speed < 20:  # 最低duty保証（20に変更）
                target_speed = 20
        
        # ===== 衝突判定（パルス停止検出） =====
        if wheel_counter is not None:
            # パルス0の連続カウント
            if target_speed > 10 and pulse_delta_display == 0:
                zero_pulse_count += 1
            else:
                zero_pulse_count = 0
            
            # 連続10ループ以上パルス0 かつ 速度<0.01で衝突判定
            if zero_pulse_count >= 10 and wheel_velocity_mps < 0.01:
                if not collision_detected:
                    collision_detected = True
                    print(f"[衝突検出] 車輪パルス停止（{zero_pulse_count}ループ連続）！")
                    
                    # 側方回避中の衝突 → 回避モードに遷移
                    if side_avoidance_active:
                        side_avoidance_active = False
                        side_avoidance_direction = None
                        print("[衝突] 側方回避から回避モードへ遷移")
            else:
                collision_detected = False
                
        # 狭路判定・制御（waypointに"narrow": trueがある場合のみ）
        if wp.get('narrow', False):
            control_mode = "narrow_passage"  # 狭路モードに切替
            narrow_passage_active = True  # 狭路通過中
            
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
            
            # 狭路通過制御(togikai_drive版)
            base_speed = 30  # 狭路時は低速
            togikai_drive.Accel(PWM_PARAM, pwm, time, base_speed)
            
            # レーンに応じてステアリング補正
            # 新車両: 正の値=右、負の値=左
            if lane == "left":
                # 左レーン:左壁に寄せる（左に切る=負の値）
                # 右壁との距離を大きく、左壁との距離を小さく保つ
                steer_correction = (LHdis - RHdis) * 2 - 20  # -20で左寄り強化
            elif lane == "center":
                # 中央レーン:左右均等（中央維持）
                steer_correction = (LHdis - RHdis) * 2
            elif lane == "right":
                # 右レーン:右壁に寄せる（右に切る=正の値）
                # 左壁との距離を大きく、右壁との距離を小さく保つ
                steer_correction = (LHdis - RHdis) * 2 + 20  # +20で右寄り強化
            else:
                steer_correction = (LHdis - RHdis) * 2
            
            togikai_drive.Steer(PWM_PARAM, pwm, time, steer_correction)
            print(f"\rNarrow passaging (Lap:{lap}, Lane:{lane}, Steer:{steer_correction:.0f})              ", end='', flush=True)
            continue
            
        # 狭路通過完了検出（狭路→通常区間への遷移）
        if narrow_passage_active and not wp.get('narrow', False):
            control_mode = "normal"  # 通常モードに復帰
            narrow_passage_active = False
            lap += 1
            print(f"===== Lap {lap} 開始（狭路通過完了） =====")

        # ===== 左右45度センサーによる側方障害物回避 =====
        SIDE_DANGER_DIST = 30  # 回避開始距離 [cm]
        SIDE_SAFE_DIST = 40    # 回避解除距離 [cm]
        
        # 回避中の状態チェック
        if side_avoidance_active:
            # 回避解除条件: 両側とも40cm以上確保
            if LHdis >= SIDE_SAFE_DIST and RHdis >= SIDE_SAFE_DIST:
                side_avoidance_active = False
                side_avoidance_direction = None
                print("\n[側方回避] 解除（安全距離確保）")
            else:
                # 回避継続: ステアリング強制（座標演算は継続）
                if side_avoidance_direction == "left":
                    togikai_drive.Steer(PWM_PARAM, pwm, time, LEFT)
                    print(f"\r[側方回避] 左ステア継続 (LH:{LHdis:.0f} RH:{RHdis:.0f})    ", end='', flush=True)
                elif side_avoidance_direction == "right":
                    togikai_drive.Steer(PWM_PARAM, pwm, time, RIGHT)
                    print(f"\r[側方回避] 右ステア継続 (LH:{LHdis:.0f} RH:{RHdis:.0f})    ", end='', flush=True)
                # continue削除: 座標演算を継続
        else:
            # 回避開始判定
            if RHdis < SIDE_DANGER_DIST and LHdis >= SIDE_DANGER_DIST:
                # 右側に障害物 → 左にステア
                side_avoidance_active = True
                side_avoidance_direction = "left"
                print(f"\n[側方回避] 右側障害物検出({RHdis:.0f}cm) → 左ステア開始")
                togikai_drive.Steer(PWM_PARAM, pwm, time, LEFT)
                # continue削除: 座標演算を継続
            elif LHdis < SIDE_DANGER_DIST and RHdis >= SIDE_DANGER_DIST:
                # 左側に障害物 → 右にステア
                side_avoidance_active = True
                side_avoidance_direction = "right"
                print(f"\n[側方回避] 左側障害物検出({LHdis:.0f}cm) → 右ステア開始")
                togikai_drive.Steer(PWM_PARAM, pwm, time, RIGHT)
                # continue削除: 座標演算を継続

        # 障害物判定（前方安全距離）または衝突検出時
        if FRdis < Cshort or collision_detected:
            # 回避モードに切替
            if control_mode != "obstacle_avoidance":
                control_mode = "obstacle_avoidance"
                avoidance_start_wp = idx
                avoidance_start_time = time.time()
                position_tracking_enabled = False  # 位置推定を一時停止
                
                # 側方回避中なら解除
                if side_avoidance_active:
                    side_avoidance_active = False
                    side_avoidance_direction = None
                    print("[回避モード] 側方回避を解除")
                
                # 予選モード専用の復帰先WP決定
                if race_mode == "qualifying":
                    checkpoint_wp_idx = get_recovery_waypoint_qualifying(idx)
                    if checkpoint_wp_idx is not None:
                        target_checkpoint = f"WP{checkpoint_wp_idx}"
                        print(f"[回避モード] WP{idx}から回避開始 → {target_checkpoint}へ復帰予定")
                    else:
                        # 範囲外の場合は従来の前半/後半判定
                        if idx < n_wp // 2:
                            target_checkpoint = "middle_mat"
                            checkpoint_wp_idx = find_waypoint_by_checkpoint("middle_mat")
                            print(f"[回避モード] WP{idx}から回避開始 → 中間マットを目指します")
                        else:
                            target_checkpoint = "start_line"
                            checkpoint_wp_idx = find_waypoint_by_checkpoint("start_line")
                            print(f"[回避モード] WP{idx}から回避開始 → スタートラインを目指します")
                else:
                    # 本選モード: 従来の前半/後半判定
                    if idx < n_wp // 2:
                        target_checkpoint = "middle_mat"
                        checkpoint_wp_idx = find_waypoint_by_checkpoint("middle_mat")
                        print(f"[回避モード] WP{idx}から回避開始 → 中間マットを目指します")
                    else:
                        target_checkpoint = "start_line"
                        checkpoint_wp_idx = find_waypoint_by_checkpoint("start_line")
                        print(f"[回避モード] WP{idx}から回避開始 → スタートラインを目指します")
            
            # 後退＋右ステアリング（距離が十分開くまで）
            togikai_drive.Accel(PWM_PARAM, pwm, time, REVERSE) 
            togikai_drive.Steer(PWM_PARAM, pwm, time, RIGHT)
            print(f"\r[{control_mode}] Obstacle! Reversing... Target:{target_checkpoint}", end='', flush=True)
            time.sleep(1.5)  # 後退時間を1秒→1.5秒に延長
            
            # 停止して距離確認
            togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
            time.sleep(0.2)
            
            # 再度前方距離をチェック
            FRdis_check = togikai_ultrasonic.Mesure(GPIO,time,22,7)  # Trig=BCM22, Echo=BCM7
            FRdis_check = filter_ultrasonic(FRdis_check)
            
            if FRdis_check >= Cshort + 10:
                # 十分に距離が開いた → 復帰モードへ
                control_mode = "recovery"
                print(f"\n[復帰モード] 障害物クリア（距離:{FRdis_check:.0f}cm）、WP{idx}への復帰を試行")
            else:
                # まだ近い → 継続して回避
                print(f"\n[回避継続] 距離不足（{FRdis_check:.0f}cm）、再度後退")
            
            continue

        # 回避モードから復帰チェック
        if control_mode == "obstacle_avoidance" and FRdis >= Cshort + 10:
            control_mode = "recovery"
            print(f"\n[復帰モード] 障害物クリア、WP{idx}への復帰を試行")
        
        # 復帰モード処理
        if control_mode == "recovery":
            # 復帰先WPへの復帰処理
            if checkpoint_wp_idx is not None:
                idx = checkpoint_wp_idx
                # 座標は維持（実際の位置を反映し続ける）- WPインデックスのみ変更
                # current_x, current_y はそのまま（衝突前の位置から継続）
                velocity_x = 0.0  # 車輪速度ベース（グローバル座標X成分）
                velocity_y = 0.0  # 車輪速度ベース（グローバル座標Y成分）
                position_tracking_enabled = True
                
                # 予選モードの場合はWP番号、本選モードはチェックポイント名で表示
                if race_mode == "qualifying" and target_checkpoint.startswith("WP"):
                    print(f"[通常モード復帰] {target_checkpoint}で再開（現在地:{current_x:.1f},{current_y:.1f}m）")
                else:
                    print(f"[通常モード復帰] {target_checkpoint}からWP{idx}で再開（現在地:{current_x:.1f},{current_y:.1f}m）")
            else:
                # チェックポイントWPが見つからない場合は簡易復帰
                position_tracking_enabled = True
                print(f"[簡易復帰] WP{idx}から再開")
            
            # 復帰後、前進開始（十分に障害物から離れるまで）
            print("[復帰] 前進開始")
            recovery_forward_time = 0
            max_recovery_time = 3.0  # 最大3秒前進
            
            while recovery_forward_time < max_recovery_time:
                # 前進コマンド
                togikai_drive.Accel(PWM_PARAM, pwm, time, FORWARD_S)
                togikai_drive.Steer(PWM_PARAM, pwm, time, CENTER)
                time.sleep(0.3)
                recovery_forward_time += 0.3
                
                # 前方距離チェック
                FRdis_recovery = togikai_ultrasonic.Mesure(GPIO,time,22,7)  # Trig=BCM22, Echo=BCM7
                FRdis_recovery = filter_ultrasonic(FRdis_recovery)
                
                # 十分に離れた、または遠くなったら通常モードへ
                if FRdis_recovery >= Cshort + 20:  # 40cm以上離れたら
                    print(f"[復帰完了] 十分に離れました（距離:{FRdis_recovery:.0f}cm）")
                    break
                
                print(f"\r[{control_mode}] 前進継続 距離:{FRdis_recovery:.0f}cm 経過:{recovery_forward_time:.1f}s", end='', flush=True)
            
            # 探索モードに移行（WP到達まで探索状態）
            control_mode = "searching"
            print("\n[探索モード] WP復帰を試行中")
            continue  # 次のループへ
        
        # Yaw制御ロジック（searchingモード以外でtarget_yawがある場合のみ）
        if control_mode != "searching" and target_yaw is not None:
            current_yaw_rad = math.radians(current_yaw)
            target_yaw_rad = math.radians(target_yaw)
            
            # Yaw誤差計算（符号反転: current - targetで計算）
            # 例: 目標180°、現在179° → 誤差-1° → 右に切る（正の値）
            yaw_error = current_yaw_rad - target_yaw_rad
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            # ステアリング角度計算
            steer_gain = 2.0
            steer_angle = steer_gain * math.degrees(yaw_error)
            
            # ステアリング角度制限（±90度）
            steer_angle = max(-90, min(90, steer_angle))
            
            # モーター制御
            togikai_drive.Accel(PWM_PARAM, pwm, time, target_speed)
            togikai_drive.Steer(PWM_PARAM, pwm, time, steer_angle)
            
            if DEBUG_MODE:
                print(f"[YAW] WP:{idx} Target:{math.degrees(target_yaw_rad):.0f}° Current:{math.degrees(current_yaw_rad):.0f}° Error:{math.degrees(yaw_error):.0f}° Steer:{steer_angle:.0f}°")
        else:
            # searchingモード、またはtarget_yawがない場合はtogikai制御を実行
            if FRdis >= Cshort:
                if LHdis <= short and RHdis >= short:
                   togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_C)
                   togikai_drive.Steer(PWM_PARAM,pwm,time,RIGHT) #original = "+"
                   comment = "右旋回"
                elif LHdis > short and RHdis < short:
                   togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_C)
                   togikai_drive.Steer(PWM_PARAM,pwm,time,LEFT) #original = "-"
                   comment = "左旋回"
                elif LHdis < short and RHdis < short:
                    if (LHdis - RHdis)>10:
                        togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_C)
                        togikai_drive.Steer(PWM_PARAM,pwm,time,LEFT) #original = "-"
                        comment = "左旋回"
                    elif(RHdis - LHdis) > 10:
                        togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_C)
                        togikai_drive.Steer(PWM_PARAM,pwm,time,RIGHT) #original = "+"
                        comment = "右旋回"
                    else:
                        togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_S)
                        togikai_drive.Steer(PWM_PARAM,pwm,time,0)
                        comment = "直進中"
                else:
                    togikai_drive.Accel(PWM_PARAM,pwm,time,FORWARD_S)
                    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
                    comment = "直進中"
            elif time.time()-start_time < 1:
                pass
            else:
                togikai_drive.Accel(PWM_PARAM,pwm,time,REVERSE)
                togikai_drive.Steer(PWM_PARAM,pwm,time,0)
                time.sleep(0.1)
                togikai_drive.Accel(PWM_PARAM,pwm,time,0)
                togikai_drive.Steer(PWM_PARAM,pwm,time,0)
                print('Stop by obstacle!')

        #距離データを配列に記録（詳細情報を追加）
        # モードを数値化: normal=0, narrow_passage=1, obstacle_avoidance=2, recovery=3, searching=4
        mode_num = {"normal": 0, "narrow_passage": 1, "obstacle_avoidance": 2, "recovery": 3, "searching": 4}.get(control_mode, 0)
        
        # WP距離計算（position_tracking_enabled時のみ）
        if position_tracking_enabled:
            distance_to_wp = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2)
        else:
            distance_to_wp = -1.0  # 無効値
        
        # データ記録（13列）
        d = np.vstack([d,[
            time.time()-start_time,  # 1. 経過時間
            FRdis, RHdis, LHdis, RRHdis, RLHdis,  # 2-6. センサー
            current_yaw, accel_x, accel_y,  # 7-9. IMU
            mode_num,  # 10. 制御モード
            idx,  # 11. 現在のWP index
            line_count,  # 12. 累積白線カウント
            distance_to_wp  # 13. WPまでの距離
        ]])
        
        # ===== リアルタイム表示（1行に統合） =====
        race_time = time.time() - start_time
        
        # 車輪速パルス情報取得（デバッグ用）
        wheel_pulse_info = ""
        if wheel_counter is not None:
            current_pulse_value = wheel_counter.last_value
            total_pulses = wheel_counter.total_count
            # パルス検出状況を記号で表示
            pulse_status = "●" if pulse_delta_display > 0 else "○"  # ●=検出中、○=停止
            wheel_pulse_info = f"{pulse_status}Cnt:{current_pulse_value} Tot:{total_pulses} Δ:{pulse_delta_display} "
            
            # パルス検出状況を常に表示（デバッグ用）
            if DEBUG_MODE:
                print(f"[パルス] {pulse_status} raw_cnt={current_pulse_value} total={total_pulses} delta={pulse_delta_display} v={wheel_velocity_mps:.3f}m/s")
        
        if position_tracking_enabled:
            # 1行に統合表示（整理版フォーマット）
            if wheel_counter is not None:
                # パターン1: 位置推定ON + 車輪速センサーON
                print(f'\r{race_time:.1f} [{control_mode}] Pos({current_x:.1f},{current_y:.1f}), {current_yaw:.0f}, {wheel_velocity_mps:.2f} | WP{idx}({wp_x:.1f},{wp_y:.1f}) | {FRdis:.0f},{RHdis:.0f},{LHdis:.0f},{RRHdis:.0f},{RLHdis:.0f}', end='', flush=True)
            else:
                # パターン2: 位置推定ON + 車輪速センサーOFF
                print(f'\r{race_time:.1f} [{control_mode}] Pos({current_x:.1f},{current_y:.1f}), {current_yaw:.0f}, D{distance_to_wp:.2f} | WP{idx}({wp_x:.1f},{wp_y:.1f}) | {FRdis:.0f},{RHdis:.0f},{LHdis:.0f},{RRHdis:.0f},{RLHdis:.0f}', end='', flush=True)
        else:
            if wheel_counter is not None:
                # パターン3: 位置推定OFF + 車輪速センサーON
                print(f'\r{race_time:.1f} [{control_mode}] {current_yaw:.0f}, {wheel_velocity_mps:.2f} | WP{idx}({wp_x:.1f},{wp_y:.1f}) | {FRdis:.0f},{RHdis:.0f},{LHdis:.0f},{RRHdis:.0f},{RLHdis:.0f}', end='', flush=True)
            else:
                # パターン4: 位置推定OFF + 車輪速センサーOFF
                print(f'\r{race_time:.1f} [{control_mode}] {current_yaw:.0f}, Tgt{target_speed:.0f} | WP{idx}({wp_x:.1f},{wp_y:.1f}) | {FRdis:.0f},{RHdis:.0f},{LHdis:.0f},{RRHdis:.0f},{RLHdis:.0f}', end='', flush=True)

        # ===== 位置推定 =====
        if position_tracking_enabled:
            # 位置推定は車輪速センサと同じタイミング・delta_timeを使用
            
            # ===== 車輪速度ベースの位置推定 =====
            if wheel_counter is not None and wheel_velocity_mps is not None and pulse_delta_display > 0:
                # 車輪速度を使用（Yaw角で方向を決定）
                # キャリブレーション済みのYawをそのまま使用
                # Yaw=180°で-X方向（X減少）、Yaw=0°/360°で+X方向（X増加）
                yaw_rad = math.radians(current_yaw)
                
                # 車両前方方向を速度ベクトルとする
                # グローバル座標系: 
                # Yaw=0°→+X方向、Yaw=90°→+Y方向、Yaw=180°→-X方向、Yaw=270°→-Y方向
                velocity_x = wheel_velocity_mps * math.cos(yaw_rad)  # X方向成分
                velocity_y = wheel_velocity_mps * math.sin(yaw_rad)  # Y方向成分
                
                # 位置更新: パルス検出時の時間差を使用
                current_x += velocity_x * delta_time_wheel
                current_y += velocity_y * delta_time_wheel
                
                # 累積移動距離を計算（Waypoint到達判定用）
                delta_distance = wheel_velocity_mps * delta_time_wheel
                accumulated_distance += delta_distance
                
                if DEBUG_MODE:
                    print(f"[位置推定] v={wheel_velocity_mps:.3f}m/s Yaw={current_yaw:.0f}° Δt={delta_time_wheel:.3f}s ΔX={velocity_x*delta_time_wheel:.3f} ΔY={velocity_y*delta_time_wheel:.3f} Pos=({current_x:.2f},{current_y:.2f})")
            
            else:
                # 車輪速度センサが使用できない場合は静止と判定（IMU加速度積分は使用しない）
                velocity_x = 0.0
                velocity_y = 0.0
                # 位置は更新しない（静止中）
                # 累積移動距離も更新しない
                delta_distance = 0.0
            
            # ===== Waypoint到達判定 =====
            # 現在waypointとの距離を計算
            wp = waypoints[idx]
            distance_to_wp = math.hypot(wp['x'] - current_x, wp['y'] - current_y)
            
            # Waypoint到達判定: 距離が0.5m未満、または一定時間経過（タイムアウト: 15秒）
            current_time = time.time()
            time_in_wp = current_time - wp_start_time
            
            if distance_to_wp < 0.5 or time_in_wp > 15.0:
                print(f"[WP到達] idx:{idx} -> {idx+1}, 位置補正:({current_x:.2f},{current_y:.2f})→({wp['x']:.2f},{wp['y']:.2f})")
                
                # 探索モードからの復帰（WP到達で通常モードへ）
                if control_mode == "searching":
                    control_mode = "normal"
                    print("[探索完了] 通常走行モードへ復帰")
                
                # 位置を強制補正（ドリフト誤差吸収）
                current_x = waypoints[idx]['x']
                current_y = waypoints[idx]['y']
                velocity_x = 0.0
                velocity_y = 0.0
                
                idx += 1
                
                # WP20以降到達でゴール（本選のゴールシーケンス完了）
                if race_mode == "final" and idx > 20:
                    race_finished = True
                    togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
                    togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
                    final_time = time.time() - start_time
                    print(f"[ゴール] 完走時間: {final_time:.2f}秒")
                    continue
                
                # 最終waypointに到達（WP17の次）
                if idx >= len(waypoints):
                    elapsed_time = time.time() - start_time
                    
                    # 本選モード: 時間チェック
                    if race_mode == "final" and TIME_LIMIT is not None:
                        if elapsed_time >= TIME_LIMIT:
                            # 150秒経過 → ゴールシーケンス（WP18以降）へ
                            if len(waypoints) > 17:
                                # WP18が存在する場合は継続（idxはそのまま）
                                print(f"[タイムアップ {elapsed_time:.1f}秒] ゴールシーケンス開始")
                            else:
                                # WP18がない場合は停止
                                race_finished = True
                                togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
                                togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
                                print(f"[タイムアップ] ゴールWP不足のため停止")
                                continue
                        else:
                            # 150秒未満 → WP0に戻って周回継続
                            idx = 0
                            current_x = waypoints[0]['x']
                            current_y = waypoints[0]['y']
                            velocity_x = 0.0
                            velocity_y = 0.0
                            narrow_passage_active = False
                            remaining_time = TIME_LIMIT - elapsed_time
                            print(f"[周回完了] WP0に戻ります（残り時間: {remaining_time:.1f}秒）")
                    else:
                        # 予選モード: 常にWP0に戻る
                        idx = 0
                        current_x = waypoints[0]['x']
                        current_y = waypoints[0]['y']
                        velocity_x = 0.0
                        velocity_y = 0.0
                        narrow_passage_active = False
                        print(f"[周回完了] WP0に戻ります")
                
                # 次のwaypoint用にリセット
                wp_start_time = time.time()
                accumulated_distance = 0.0
            
            # デバッグ表示（位置情報）- 車輪速度ベース
            if DEBUG_MODE:
                speed_info = f"V:{wheel_velocity_mps:.2f}m/s" if wheel_counter is not None else "V:N/A"
                print(f"[POS] X:{current_x:.2f}m Y:{current_y:.2f}m {speed_info} WP_dist:{distance_to_wp:.2f}m")

#        time.sleep(0.05)

except KeyboardInterrupt:
    print('stop!')
    np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
    togikai_drive.Accel(PWM_PARAM,pwm,time,0)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
    GPIO.cleanup()
