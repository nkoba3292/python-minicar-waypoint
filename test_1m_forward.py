"""
1m直進テストプログラム（IMUステアリング補正付き）
- 車輪速センサーで距離を計測
- IMUのYaw角を維持してステアリング補正
- 1m進んだら停止
- 距離推定精度を確認
"""

import RPi.GPIO as GPIO
import time
import math
import sys

# togikaiライブラリのパス追加
sys.path.append('/home/pi/togikai/togikai_function/')
import togikai_drive
import PCA9685
from wheel_counter_74hc590 import Counter74HC590
from IMU_sensor_bno055 import IMUSensorBNO055

# GPIOピン番号の指示方法
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# PWM制御の初期設定
pwm = PCA9685.PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)

# アライメント調整済みPWMパラメータ読み込み
PWM_PARAM = togikai_drive.ReadPWMPARAM(pwm)

# モーターの初期化
togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
togikai_drive.Steer(PWM_PARAM, pwm, time, 0)

# IMU初期化
print('=== IMU初期化 ===')
imu = IMUSensorBNO055(port='/dev/serial0', baudrate=115200)

# 車輪速センサ初期化
print('=== 車輪速センサ初期化 ===')
WHEEL_LATCH_PIN = 24   # RCLK (BOARD 18)
WHEEL_DATA_PINS = [13, 21, 25]  # D0, D1, D2 (BOARD 33, 40, 22)
WHEEL_CLEAR_PIN = 17   # CCLR (BOARD 11)

wheel_counter = None
try:
    wheel_counter = Counter74HC590(WHEEL_LATCH_PIN, WHEEL_DATA_PINS, WHEEL_CLEAR_PIN)
    print(f'[車輪速センサ] 初期化完了 (初期値: {wheel_counter.last_value})')
except Exception as e:
    print(f'[車輪速センサ] 初期化失敗: {e}')
    sys.exit(1)

# 車輪パラメータ
WHEEL_DIAMETER = 0.0474  # 車輪直径 [m] (47.4mm) - 実測補正済み (2.0m/2.20m)
WHEEL_PPR = 1.0  # 1回転あたりのパルス数

# 目標距離
TARGET_DISTANCE = 2.0  # 2m

# 走行速度
FORWARD_SPEED = 100  # PWM値（全速）

# ステアリング制御パラメータ
YAW_P_GAIN = 3.0  # Yaw角度補正ゲイン（大きいほど補正が強い）
MAX_STEER = 30  # 最大ステアリング角度制限

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

# 初期Yaw角を基準値として保存
if len(yaw_samples) > 0:
    initial_yaw = sum(yaw_samples) / len(yaw_samples)
else:
    initial_yaw = 0.0
    print('[WARN] Yaw測定失敗、0度として継続')

print(f'[CALIB] 初期Yaw角: {initial_yaw:.1f}° 完了')

print('\n=== 1m直進テスト（IMU補正付き）===')
print(f'目標距離: {TARGET_DISTANCE}m')
print(f'速度: {FORWARD_SPEED}')
print(f'Yaw補正ゲイン: {YAW_P_GAIN}')
print('Press Enter to start')
input()

try:
    # 初期化
    total_distance = 0.0
    wheel_last_total = wheel_counter.total_count
    wheel_last_time = time.time()
    
    # 前進開始
    togikai_drive.Accel(PWM_PARAM, pwm, time, FORWARD_SPEED)
    
    start_time = time.time()
    last_print_time = start_time
    
    print(f"\n{'時刻':<8} {'パルス':<8} {'距離[m]':<10} {'Yaw[deg]':<10} {'ΔYaw':<8} {'Steer':<8} {'Status'}")
    print("-" * 85)
    
    while total_distance < TARGET_DISTANCE:
        # 車輪速センサ読み取り
        current_value, delta_pulses, current_total = wheel_counter.read_incremental()
        current_time = time.time()
        
        # パルス誤検出補正
        if delta_pulses >= 2:
            delta_pulses = 1
        
        # 時間差分
        delta_time = current_time - wheel_last_time
        
        # 距離計算
        if delta_pulses > 0 and delta_time > 0.001:
            wheel_circumference = math.pi * WHEEL_DIAMETER
            rotations = delta_pulses / WHEEL_PPR
            distance_increment = rotations * wheel_circumference
            total_distance += distance_increment
        
        # IMUからYaw角取得
        try:
            current_yaw = imu.get_euler_angles()['yaw']
            
            # Yaw角の差分計算（-180〜180度に正規化）
            yaw_error = current_yaw - initial_yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360
            
            # ステアリング補正計算（P制御）
            # 右に曲がっている（yaw_error > 0）なら左に切る（負の値）
            steer_correction = -yaw_error * YAW_P_GAIN
            
            # ステアリング制限
            steer_correction = max(-MAX_STEER, min(MAX_STEER, steer_correction))
            
            # ステアリング適用
            togikai_drive.Steer(PWM_PARAM, pwm, time, int(steer_correction))
            
            # 経過時間
            elapsed = current_time - start_time
            
            # 表示（200ms間隔 + ステアリング補正状況）
            if current_time - last_print_time > 0.2:
                # ステアリング補正状態
                if abs(steer_correction) > 5:
                    status = f"←補正中" if steer_correction < 0 else "補正中→"
                else:
                    status = "直進中"
                
                print(f"{elapsed:7.2f}s  {current_total:<8}  {total_distance:9.4f}  {current_yaw:9.2f}  {yaw_error:7.2f}  {steer_correction:7.1f}  {status}")
                last_print_time = current_time
            
        except Exception as e:
            print(f"[WARN] IMU読み取りエラー: {e}")
        
        wheel_last_time = current_time
        
        # 少し待機（過度なCPU使用を防ぐ）
        time.sleep(0.01)
    
    # 停止
    togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
    togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
    
    end_time = time.time()
    elapsed_total = end_time - start_time
    
    print("-" * 70)
    print(f"\n=== テスト完了 ===")
    print(f"走行距離: {total_distance:.4f}m")
    print(f"目標距離: {TARGET_DISTANCE}m")
    print(f"誤差: {abs(total_distance - TARGET_DISTANCE):.4f}m ({abs(total_distance - TARGET_DISTANCE) / TARGET_DISTANCE * 100:.1f}%)")
    print(f"所要時間: {elapsed_total:.2f}秒")
    print(f"平均速度: {total_distance / elapsed_total:.3f}m/s")
    
    # IMUデータ取得
    try:
        final_yaw = imu.get_euler_angles()['yaw']
        yaw_drift = final_yaw - initial_yaw
        if yaw_drift > 180:
            yaw_drift -= 360
        elif yaw_drift < -180:
            yaw_drift += 360
        print(f"初期Yaw角: {initial_yaw:.1f}°")
        print(f"最終Yaw角: {final_yaw:.1f}°")
        print(f"Yawドリフト: {yaw_drift:+.1f}°")
    except:
        pass

except KeyboardInterrupt:
    print("\n[中断] テスト中止")

finally:
    # クリーンアップ
    togikai_drive.Accel(PWM_PARAM, pwm, time, 0)
    togikai_drive.Steer(PWM_PARAM, pwm, time, 0)
    GPIO.cleanup()
    print("[完了] モーター停止、GPIO解放")
