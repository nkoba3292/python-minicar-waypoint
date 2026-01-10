#!/usr/bin/env python3
"""
74HC590 Pin 12 (G'/Output Enable) 診断スクリプト

Pin 12 (G') は Output Enable (active LOW):
- LOW (0V) → 出力有効
- HIGH (3.3V) → 出力無効（Hi-Z状態）

BCM 21 が常に 0 で 700Ω の原因を特定する
"""

import RPi.GPIO as GPIO
import time

# ピン配置
DATA_PINS = {
    'D0 (QA-pin15)': 13,  # BCM 13 = BOARD 33
    'D1 (QB-pin1)':  21,  # BCM 21 = BOARD 40 ← 問題のピン
    'D2 (QC-pin2)':  25,  # BCM 25 = BOARD 22
}

CONTROL_PINS = {
    'CCLR (pin10)': 17,   # BCM 17 = BOARD 11
    'RCLK (pin13)': 24,   # BCM 24 = BOARD 18
}

# Pin 12 (G') は現在どこに接続されている？
# 可能性のあるピンをチェック
POSSIBLE_G_PINS = [18, 23]  # BCM 18, 23 が未使用

def check_pin_voltage(pin_bcm, pin_name):
    """指定ピンの状態を読み取り"""
    GPIO.setup(pin_bcm, GPIO.IN)
    
    # プルアップ/ダウン無しで読む
    time.sleep(0.1)
    no_pull = GPIO.input(pin_bcm)
    
    # プルアップで読む
    GPIO.setup(pin_bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(0.1)
    with_pullup = GPIO.input(pin_bcm)
    
    # プルダウンで読む
    GPIO.setup(pin_bcm, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    time.sleep(0.1)
    with_pulldown = GPIO.input(pin_bcm)
    
    return no_pull, with_pullup, with_pulldown

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    print("=" * 60)
    print("74HC590 Pin 12 (G'/Output Enable) 診断")
    print("=" * 60)
    print()
    
    print("【重要】Pin 12 (G') は active LOW:")
    print("  - Pin 12 = GND (0V) → 出力有効")
    print("  - Pin 12 = VCC (3.3V) → 出力無効（Hi-Z）")
    print()
    
    # 現在のデータピン状態をチェック
    print("=== 現在のデータピン状態 ===")
    for name, pin in DATA_PINS.items():
        no_pull, pullup, pulldown = check_pin_voltage(pin, name)
        print(f"{name:20} BCM{pin:2d}: ", end="")
        print(f"通常={no_pull}  Pull-up={pullup}  Pull-down={pulldown}")
        
        if no_pull == 0 and pullup == 0 and pulldown == 0:
            print(f"  → {name} は GND に強く引かれている！（異常）")
    
    print()
    
    # Pin 12 (G') の可能性があるピンをチェック
    print("=== Pin 12 (G') 候補ピンのチェック ===")
    print("BCM 18, 23 をチェック...")
    print()
    
    for pin in POSSIBLE_G_PINS:
        no_pull, pullup, pulldown = check_pin_voltage(pin, f"BCM{pin}")
        print(f"BCM {pin}: 通常={no_pull}  Pull-up={pullup}  Pull-down={pulldown}")
        
        if no_pull == 1 and pullup == 1 and pulldown == 0:
            print(f"  → BCM {pin} は VCC に弱く引かれている（可能性: Pin 12）")
        elif no_pull == 0 and pullup == 1 and pulldown == 0:
            print(f"  → BCM {pin} はフローティング（未接続の可能性）")
    
    print()
    print("=" * 60)
    print("診断結果と推奨事項")
    print("=" * 60)
    print()
    print("QB (pin 1) が GND に 700Ω で接続されている原因:")
    print()
    print("1. 【最も可能性が高い】IC の QB 出力が破損")
    print("   → 内部で GND 側 FET が常時 ON")
    print("   → 新しい 74HC590 に交換してください")
    print()
    print("2. Pin 12 (G') が HIGH になっている")
    print("   → しかし Hi-Z 状態では 700Ω にならない")
    print("   → この可能性は低い")
    print()
    print("3. 外部回路で QB を GND にプルダウン")
    print("   → ラズパイ 40p を外して再測定して確認")
    print()
    
    # 制御信号の状態もチェック
    print("=== 制御信号の状態 ===")
    for name, pin in CONTROL_PINS.items():
        GPIO.setup(pin, GPIO.IN)
        val = GPIO.input(pin)
        print(f"{name:20} BCM{pin:2d}: {val}")
    
    print()
    print("【次のステップ】")
    print("1. ラズパイの物理 40p を外す")
    print("2. マルチメータで 74HC590 pin 1 と GND の抵抗を再測定")
    print("   - 700Ω のまま → IC が破損（交換必要）")
    print("   - 高抵抗 (>100kΩ) → ラズパイ側の問題")
    print()
    print("3. Pin 12 (G') の電圧を直接測定")
    print("   - 0V → 正常（出力有効）")
    print("   - 3.3V → 出力無効になっている")
    print()
    
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断されました")
        GPIO.cleanup()
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
        GPIO.cleanup()
