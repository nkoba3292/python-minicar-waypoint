#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU制御自律走行プログラム（新車両用）
- Yawキャリブレーション対応
- 位置推定機能
- 予選/本選モード対応
"""

import os
import sys
sys.path.append('/home/pi/togikai/togikai_function/')

# UTF-8エンコーディング設定（文字化け防止）
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

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

# GPIOピン番号の指示方法
GPIO.setwarnings(False)  # 警告を無効化
GPIO.cleanup()  # 以前の設定をクリーンアップ
GPIO.setmode(GPIO.BOARD)

#超音波センサ初期設定
# Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
t_list=[15,13,35,32,36]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
# Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
e_list=[26,24,37,31,38]
GPIO.setup(e_list,GPIO.IN)

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
DEBUG_MODE = False  # デバッグモード（True=デバッグ情報表示）
SPEED_SCALE = 1.0   # 速度スケール（デバッグ時は0.33など）
SAFE_DIST_FRONT = 20  # 前方安全距離（cm）
USE_NARROW_PASSAGE = True  # 狭路判定を使用する

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
#データ記録用配列作成
d = np.zeros(9)
#操舵、駆動モーターの初期化
togikai_drive.Accel(PWM_PARAM,pwm,time,0)
togikai_drive.Steer(PWM_PARAM,pwm,time,0)

# 追加IMU初期化
from IMU_sensor_bno055 import IMUSensorBNO055
imu = IMUSensorBNO055(port='/dev/serial0', baudrate=115200)

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
            
        #Frセンサ距離
        FRdis = togikai_ultrasonic.Mesure(GPIO,time,15,26)
        #FrLHセンサ距離
        LHdis = togikai_ultrasonic.Mesure(GPIO,time,13,24)
        #FrRHセンサ距離
        RHdis = togikai_ultrasonic.Mesure(GPIO,time,32,31)
        #RrLHセンサ距離
        RLHdis = togikai_ultrasonic.Mesure(GPIO,time,35,37)
        #RrRHセンサ距離
        RRHdis = togikai_ultrasonic.Mesure(GPIO,time,36,38)
        
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

        
        #IMUから現在のヨー角・加速度xy取得（Yawオフセット適用）
        raw_yaw = imu.get_euler_angles()['yaw']
        current_yaw = (raw_yaw + initial_yaw_offset) % 360
        accel_x = imu.get_acceleration()['x']
        accel_y = imu.get_acceleration()['y']

        # 現在 waypoint取得
        wp = waypoints[idx]
        wp_x, wp_y = wp['x'], wp['y']
        target_speed = wp.get('v', 50)  # ウェイポイント指定速度、デフォルト50
        target_yaw = wp.get('yaw', None)  # ウェイポイント指定方位角

        # デバッグモード時の速度調整
        if DEBUG_MODE:
            target_speed = target_speed * SPEED_SCALE
            if target_speed < 10:  # 最低速度保証
                target_speed = 10
                
        # 狭路判定・制御（waypointに"narrow": trueがある場合のみ）
        if wp.get('narrow', False):
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
            narrow_passage_active = False
            lap += 1
            print(f"===== Lap {lap} 開始（狭路通過完了） =====")

        # 障害物判定（前方安全距離）
        if FRdis < Cshort:
            togikai_drive.Accel(PWM_PARAM, pwm, time, REVERSE) 
            togikai_drive.Steer(PWM_PARAM, pwm, time, RIGHT)
            print("Obstacle Front! Reverse")
            time.sleep(1)
            continue

        # Yaw制御ロジック（target_yawがある場合のみ）
        if target_yaw is not None:
            current_yaw_rad = math.radians(current_yaw)
            target_yaw_rad = math.radians(target_yaw)
            
            # Yaw誤差計算（-πからπに正規化）
            yaw_error = target_yaw_rad - current_yaw_rad
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
            # target_yawがない場合は元のtogikai制御を実行
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

        #距離データを配列に記録
        d = np.vstack([d,[time.time()-start_time, FRdis, RHdis, LHdis, RRHdis, RLHdis,current_yaw,accel_x,accel_y]])
        
        # ===== リアルタイム表示（1行に統合） =====
        race_time = time.time() - start_time
        
        if position_tracking_enabled:
            distance_to_wp = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2)
            target_yaw_display = target_yaw if target_yaw is not None else 0.0
            # 1行に統合表示
            print(f'\rT:{race_time:.1f}s WP{idx}:({wp_x:.1f},{wp_y:.1f})m D:{distance_to_wp:.2f}m Y:{current_yaw:.0f}° Fr:{FRdis:.0f} FrL:{LHdis:.0f} FrR:{RHdis:.0f} RrL:{RLHdis:.0f} RrR:{RRHdis:.0f} Ax:{accel_x:.2f} Ay:{accel_y:.2f}', end='', flush=True)
        else:
            print(f'\rT:{race_time:.1f}s WP{idx} Fr:{FRdis:.0f} FrL:{LHdis:.0f} FrR:{RHdis:.0f} Y:{current_yaw:.0f}°', end='', flush=True)

        # ===== 位置推定 =====
        if position_tracking_enabled:
            current_time = time.time()
            delta_time = current_time - last_update_time
            
            # 静止判定閾値（ドリフト防止）- より厳しく設定
            ACCEL_THRESHOLD = 0.05  # m/s² (0.15→0.05に変更)
            accel_magnitude = math.sqrt(accel_x**2 + accel_y**2)
            
            if accel_magnitude < ACCEL_THRESHOLD:
                # 静止中 → 速度をゼロリセット（ドリフト防止）
                velocity_x = 0.0
                velocity_y = 0.0
            else:
                # 修正: 位置推定を if の中に移動
                # 車両座標系（前方=Y、右=X）→ グローバル座標系への変換
                # Yaw角を使って回転変換
                yaw_rad = math.radians(current_yaw)
                
                # 回転行列による座標変換
                # グローバルX = 車体X * cos(yaw) - 車体Y * sin(yaw)
                # グローバルY = 車体X * sin(yaw) + 車体Y * cos(yaw)
                accel_global_x = accel_x * math.cos(yaw_rad) - accel_y * math.sin(yaw_rad)
                accel_global_y = accel_x * math.sin(yaw_rad) + accel_y * math.cos(yaw_rad)
                
                # 速度更新: v = v0 + a*dt
                velocity_x += accel_global_x * delta_time
                velocity_y += accel_global_y * delta_time
            
                # 位置更新: x = x0 + v*dt
                current_x += velocity_x * delta_time
                current_y += velocity_y * delta_time
                
                # 累積移動距離を計算（Waypoint到達判定用）
                delta_distance = math.sqrt((velocity_x * delta_time)**2 + (velocity_y * delta_time)**2)
                accumulated_distance += delta_distance
            
            last_update_time = current_time
            
            # ===== Waypoint到達判定 =====
            # 現在waypointとの距離を計算
            wp = waypoints[idx]
            distance_to_wp = math.hypot(wp['x'] - current_x, wp['y'] - current_y)
            
            # Waypoint到達判定: 距離が0.5m未満、または一定時間経過（タイムアウト: 15秒）
            time_in_wp = current_time - wp_start_time
            
            if distance_to_wp < 0.5 or time_in_wp > 15.0:
                print(f"[WP到達] idx:{idx} -> {idx+1}, 位置補正:({current_x:.2f},{current_y:.2f})→({wp['x']:.2f},{wp['y']:.2f})")
                
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
            
            # デバッグ表示（位置情報）
            if DEBUG_MODE:
                print(f"[POS] X:{current_x:.2f}m Y:{current_y:.2f}m Vx:{velocity_x:.2f}m/s Vy:{velocity_y:.2f}m/s WP_dist:{distance_to_wp:.2f}m")

        time.sleep(0.05)

except KeyboardInterrupt:
    print('stop!')
    np.savetxt('/home/pi/code/record_data.csv', d, fmt='%.3e')
    togikai_drive.Accel(PWM_PARAM,pwm,time,0)
    togikai_drive.Steer(PWM_PARAM,pwm,time,0)
    GPIO.cleanup()
