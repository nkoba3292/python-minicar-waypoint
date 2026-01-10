#!/usr/bin/env python3
"""
モーター・サーボ基本動作テスト
配線・PCA9685・PWM信号を段階的に確認
"""

import sys
sys.path.append('/home/pi/togikai/togikai_function/')
import PCA9685
import time

print("=== モーター・サーボ動作テスト ===")

# Step 1: PCA9685初期化チェック
print("\n[1] PCA9685初期化...")
try:
    pwm = PCA9685.PCA9685(address=0x40, busnum=1)
    pwm.set_pwm_freq(60)
    print("✓ PCA9685初期化成功（アドレス:0x40）")
except Exception as e:
    print(f"✗ PCA9685初期化失敗: {e}")
    print("  → I2C接続確認: sudo i2cdetect -y 1")
    sys.exit(1)

# Step 2: ニュートラル信号送信（全チャンネル停止）
print("\n[2] ニュートラル信号送信（全停止）...")
try:
    # alignment_parameter.txtの値を使用
    STEER_CENTER = 390
    THROTTLE_CENTER = 390
    
    # ステアリング: ニュートラル
    pwm.set_pwm(0, 0, STEER_CENTER)
    # スロットル: ニュートラル
    pwm.set_pwm(15, 0, THROTTLE_CENTER)
    # 他のチャンネル: 標準ニュートラル
    for ch in range(1, 15):
        pwm.set_pwm(ch, 0, 307)
    
    print("✓ 全チャンネルにニュートラル信号送信")
    print(f"  Ch.0(ステアリング): {STEER_CENTER}")
    print(f"  Ch.15(スロットル): {THROTTLE_CENTER}")
    time.sleep(2)
except Exception as e:
    print(f"✗ PWM信号送信失敗: {e}")
    sys.exit(1)

# Step 3: ステアリングサーボテスト
print("\n[3] ステアリングサーボテスト（チャンネル0）...")
print("  中央 → 右 → 中央 → 左 → 中央")
try:
    # alignment_parameter.txtの値を使用
    STEER_RIGHT = 490
    STEER_CENTER = 390
    STEER_LEFT = 290
    
    # 中央
    pwm.set_pwm(0, 0, STEER_CENTER)
    print(f"  中央（PWM={STEER_CENTER}）")
    time.sleep(1)
    
    # 右
    pwm.set_pwm(0, 0, STEER_RIGHT)
    print(f"  右（PWM={STEER_RIGHT}）")
    time.sleep(1)
    
    # 中央
    pwm.set_pwm(0, 0, STEER_CENTER)
    print("  中央に戻す")
    time.sleep(1)
    
    # 左
    pwm.set_pwm(0, 0, STEER_LEFT)
    print(f"  左（PWM={STEER_LEFT}）")
    time.sleep(1)
    
    # 中央
    pwm.set_pwm(0, 0, STEER_CENTER)
    print("  中央に戻す")
    time.sleep(1)
    
    print("✓ ステアリングサーボテスト完了")
except Exception as e:
    print(f"✗ ステアリング制御失敗: {e}")

# Step 4: スピードコントローラーテスト
print("\n[4] スピードコントローラーテスト（チャンネル15）...")
print("  ニュートラル → 前進微速 → ニュートラル → 後退微速 → ニュートラル")

try:
    # alignment_parameter.txtの値を使用
    THROTTLE_FORWARD = 470
    THROTTLE_STOPPED = 390
    THROTTLE_REVERSE = 310
    
    # ニュートラル（キャリブレーション）
    pwm.set_pwm(15, 0, THROTTLE_STOPPED)
    print(f"  ニュートラル（PWM={THROTTLE_STOPPED}）- 3秒待機")
    time.sleep(3)
    
    # 前進微速
    forward_pwm = THROTTLE_STOPPED + int((THROTTLE_FORWARD - THROTTLE_STOPPED) * 0.3)
    pwm.set_pwm(15, 0, forward_pwm)
    print(f"  前進微速（PWM={forward_pwm}）")
    time.sleep(2)
    
    # ニュートラル
    pwm.set_pwm(15, 0, THROTTLE_STOPPED)
    print("  ニュートラルに戻す")
    time.sleep(2)
    
    # 後退微速
    reverse_pwm = THROTTLE_STOPPED - int((THROTTLE_STOPPED - THROTTLE_REVERSE) * 0.3)
    pwm.set_pwm(15, 0, reverse_pwm)
    print(f"  後退微速（PWM={reverse_pwm}）")
    time.sleep(2)
    
    # ニュートラル
    pwm.set_pwm(15, 0, THROTTLE_STOPPED)
    print("  ニュートラルに戻す")
    time.sleep(1)
    
    print("✓ スピードコントローラーテスト完了")
except Exception as e:
    print(f"✗ スピコン制御失敗: {e}")

# Step 5: 全停止
print("\n[5] 全チャンネル停止...")
for ch in range(16):
    pwm.set_pwm(ch, 0, 0)
print("✓ テスト完了")

print("\n=== 診断結果 ===")
print("【動作した場合】")
print("  → ソフトウェア・PCA9685・基本配線はOK")
print("  → togikai_drive.pyのパラメータ確認が必要")
print("")
print("【動作しなかった場合】")
print("  ステアリングのみ動かない:")
print("    - サーボ電源(5V)確認")
print("    - チャンネル0の配線確認")
print("    - サーボ本体の故障確認")
print("")
print("  スピコンのみ動かない（ピピピ音なし）:")
print("    - スピコン電源(バッテリー)確認")
print("    - チャンネル15の配線確認")
print("    - スピコンのキャリブレーション要否")
print("    - モーター接続確認")
print("")
print("  両方動かない:")
print("    - PCA9685の電源(5V)確認")
print("    - PCA9685の外部電源端子(V+)確認")
print("    - I2C配線(SDA/SCL)確認")
