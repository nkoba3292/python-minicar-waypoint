#!/usr/bin/env python3
"""
新車両用PWMキャリブレーションツール
配線: Ch.0=サーボ, Ch.1=ESC
"""

import sys
sys.path.append('/home/pi/togikai/togikai_function/')
import PCA9685
import time

SERVO_CHANNEL = 0   # ステアリングサーボ
ESC_CHANNEL = 1     # スロットル（ESC）

def writetofile(path, STEERING_RIGHT_PWM, STEERING_CENTER_PWM, STEERING_LEFT_PWM,
                THROTTLE_FORWARD_PWM, THROTTLE_STOPPED_PWM, THROTTLE_REVERSE_PWM):
    """キャリブレーション結果をファイルに保存"""
    f = open(path, 'w')
    f.write('DO NOT CHANGE PARAMETER!!\n')
    f.write('STEERING_RIGHT_PWM\n')
    f.write(str(STEERING_RIGHT_PWM))
    f.write('\n')
    f.write('STEERING_CENTER_PWM\n')
    f.write(str(STEERING_CENTER_PWM))
    f.write('\n')
    f.write('STEERING_LEFT_PWM\n')
    f.write(str(STEERING_LEFT_PWM))
    f.write('\n\n')
    f.write('THROTTLE_FORWARD_PWM\n')
    f.write(str(THROTTLE_FORWARD_PWM))
    f.write('\n')
    f.write('THROTTLE_STOPPED_PWM\n')
    f.write(str(THROTTLE_STOPPED_PWM))
    f.write('\n')
    f.write('THROTTLE_REVERSE_PWM\n')
    f.write(str(THROTTLE_REVERSE_PWM))
    f.write('\n\n')
    f.close()
    print(f"保存完了: {path}")


# ファイルパス
path = '/home/pi/togikai/alignment_parameter.txt'

# 既存ファイルから読み込み（なければデフォルト値）
try:
    with open(path) as f:
        l = f.readlines()
        STEERING_RIGHT_PWM = int(l[2])
        STEERING_CENTER_PWM = int(l[4])
        STEERING_LEFT_PWM = int(l[6])
        THROTTLE_FORWARD_PWM = int(l[9])
        THROTTLE_STOPPED_PWM = int(l[11])
        THROTTLE_REVERSE_PWM = int(l[13])
    print("既存の設定を読み込みました")
except:
    STEERING_RIGHT_PWM = 290  # 右=PWM減少
    STEERING_CENTER_PWM = 390
    STEERING_LEFT_PWM = 490   # 左=PWM増加
    THROTTLE_FORWARD_PWM = 470
    THROTTLE_STOPPED_PWM = 390
    THROTTLE_REVERSE_PWM = 310
    print("デフォルト値を使用します")

print(f"現在値: CENTER={STEERING_CENTER_PWM}, STOP={THROTTLE_STOPPED_PWM}")
print("")

# PCA9685初期化
pwm = PCA9685.PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)

print("=" * 70)
print("【注意】ESCのスライドスイッチを確認してください")
print("  - ブレーキモードならOFF推奨")
print("  - リバース機能ならON推奨")
print("=" * 70)
input("確認後、Enterを押してください...")
print("")

# ステップ1: ステアリング中央調整
print("=" * 70)
print("ステップ1: ステアリング中央位置調整")
print("=" * 70)
print("")
print("【目的】タイヤが真っすぐになるPWM値を見つける")
print("")
print("操作方法:")
print("  l : 左に調整（PWM +5）")
print("  r : 右に調整（PWM -5）")
print("  e : 確定して次へ")
print("")

while True:
    print(f"現在のPWM: {STEERING_CENTER_PWM}")
    ad = input("入力 (l/r/e): ")
    
    if ad == 'l':
        STEERING_CENTER_PWM += 5  # 左=PWM増加
    elif ad == 'r':
        STEERING_CENTER_PWM -= 5  # 右=PWM減少
    elif ad == 'e':
        # 中央値から±100で左右を決定
        STEERING_RIGHT_PWM = STEERING_CENTER_PWM - 100  # 右=PWM減少
        STEERING_LEFT_PWM = STEERING_CENTER_PWM + 100   # 左=PWM増加
        print(f"確定: CENTER={STEERING_CENTER_PWM}, RIGHT={STEERING_RIGHT_PWM}, LEFT={STEERING_LEFT_PWM}")
        break
    
    pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_CENTER_PWM)

print("")

# ステップ2: ESC停止位置調整
print("=" * 70)
print("ステップ2: ESC停止位置（ニュートラル）調整")
print("=" * 70)
print("")
print("【重要】モーターが完全に停止するPWM値を見つける")
print("")
print("操作方法:")
print("  + : 前進側に調整（PWM -5）")
print("  - : 後退側に調整（PWM +5）")
print("  e : 確定して次へ")
print("")

while True:
    print(f"現在のPWM: {THROTTLE_STOPPED_PWM}")
    ad = input("入力 (+/-/e): ")
    
    if ad == '+':
        THROTTLE_STOPPED_PWM -= 5
    elif ad == '-':
        THROTTLE_STOPPED_PWM += 5
    elif ad == 'e':
        # 停止値から±80で前進/後退を決定
        THROTTLE_FORWARD_PWM = THROTTLE_STOPPED_PWM + 80
        THROTTLE_REVERSE_PWM = THROTTLE_STOPPED_PWM - 80
        print(f"確定: STOP={THROTTLE_STOPPED_PWM}, FORWARD={THROTTLE_FORWARD_PWM}, REVERSE={THROTTLE_REVERSE_PWM}")
        break
    
    pwm.set_pwm(ESC_CHANNEL, 0, THROTTLE_STOPPED_PWM)

print("")

# ステップ3: 動作確認
print("=" * 70)
print("ステップ3: 動作確認")
print("=" * 70)
print("")

# ニュートラルに戻す
pwm.set_pwm(ESC_CHANNEL, 0, THROTTLE_STOPPED_PWM)
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_CENTER_PWM)
time.sleep(2)

print("ステアリングテスト:")
print("  右...")
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_RIGHT_PWM)
time.sleep(2)
print("  中央...")
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_CENTER_PWM)
time.sleep(2)
print("  左...")
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_LEFT_PWM)
time.sleep(2)
print("  中央...")
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_CENTER_PWM)
time.sleep(1)
print("  OK")

print("")
print("スロットルテスト（軽く前進）:")
print("  注意: 車両が動くのでタイヤを浮かせるか、持ってください")
test = input("テストしますか？ (y/n): ")

if test.lower() == 'y':
    print("  前進10%...")
    forward_pwm = int(THROTTLE_STOPPED_PWM + (THROTTLE_FORWARD_PWM - THROTTLE_STOPPED_PWM) * 0.1)
    pwm.set_pwm(ESC_CHANNEL, 0, forward_pwm)
    time.sleep(1.5)
    
    print("  停止...")
    pwm.set_pwm(ESC_CHANNEL, 0, THROTTLE_STOPPED_PWM)
    time.sleep(2)
    print("  OK")
else:
    print("  スキップ")

# 停止状態で終了
pwm.set_pwm(ESC_CHANNEL, 0, THROTTLE_STOPPED_PWM)
pwm.set_pwm(SERVO_CHANNEL, 0, STEERING_CENTER_PWM)

print("")
print("=" * 70)
print("ファイル保存")
print("=" * 70)
save = input("この設定を保存しますか？ (y/n): ")

if save.lower() == 'y':
    writetofile(path, STEERING_RIGHT_PWM, STEERING_CENTER_PWM, STEERING_LEFT_PWM,
                THROTTLE_FORWARD_PWM, THROTTLE_STOPPED_PWM, THROTTLE_REVERSE_PWM)
    print("")
    print("【完了】")
    print(f"  STEERING: RIGHT={STEERING_RIGHT_PWM}, CENTER={STEERING_CENTER_PWM}, LEFT={STEERING_LEFT_PWM}")
    print(f"  THROTTLE: FORWARD={THROTTLE_FORWARD_PWM}, STOP={THROTTLE_STOPPED_PWM}, REVERSE={THROTTLE_REVERSE_PWM}")
else:
    print("保存せずに終了します")

# クリーンアップ
pwm.set_pwm(ESC_CHANNEL, 0, 0)
pwm.set_pwm(SERVO_CHANNEL, 0, 0)

print("")
print("次のステップ:")
print("  1. メインプログラムを実行してテスト")
print("  2. 必要に応じて微調整")
print("")
