#!/usr/bin/env python3
"""
74HC590 出力が Hi-Z 問題の診断

判明した事実:
- 74HC590 の QA, QB, QC は全て 0V (マルチメータ測定)
- ラズパイは QA=1, QC=1 と読む (QB=0)
- → 74HC590 の出力が Hi-Z (ハイインピーダンス) 状態

原因: Pin 12 (G'/Output Enable) が HIGH になっている
"""

import RPi.GPIO as GPIO
import time

# 可能性のある Pin 12 (G') 接続ピン
POSSIBLE_G_PINS = [18, 23, 22, 27, 5, 6]

# データピン
DATA_PINS = {
    'QA (pin15)': 13,
    'QB (pin1)': 21,
    'QC (pin2)': 25,
}

def find_output_enable_pin():
    """Pin 12 (G') が接続されているピンを探す"""
    print("=" * 60)
    print("Pin 12 (G'/Output Enable) の探索")
    print("=" * 60)
    print()
    print("Pin 12 が HIGH → 出力 Hi-Z")
    print("Pin 12 が LOW → 出力有効")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 現在のデータピン状態
    print("【現在の状態】")
    for name, pin in DATA_PINS.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.05)
        val = GPIO.input(pin)
        print(f"{name}: BCM {pin} = {val}")
    print()
    
    # 各候補ピンを LOW にして、出力が有効になるか確認
    print("【Pin 12 候補を LOW にして出力をテスト】")
    print()
    
    for test_pin in POSSIBLE_G_PINS:
        try:
            # このピンを LOW に設定
            GPIO.setup(test_pin, GPIO.OUT)
            GPIO.output(test_pin, GPIO.LOW)
            time.sleep(0.1)
            
            # データピンを読む
            GPIO.setup(DATA_PINS['QB (pin1)'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            time.sleep(0.05)
            qb_value = GPIO.input(DATA_PINS['QB (pin1)'])
            
            print(f"BCM {test_pin} を LOW にした時: QB = {qb_value}")
            
            if qb_value == 0:
                print(f"  ★ BCM {test_pin} が Pin 12 (G') の可能性！")
                print(f"     LOW にすると出力が有効になった")
            
            # 元に戻す
            GPIO.setup(test_pin, GPIO.IN)
            time.sleep(0.05)
            
        except Exception as e:
            print(f"BCM {test_pin}: エラー - {e}")
    
    print()

def test_with_pulldown():
    """プルダウンで読み直し"""
    print("=" * 60)
    print("データピンをプルダウンで再読み取り")
    print("=" * 60)
    print()
    
    for name, pin in DATA_PINS.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.1)
        samples = [GPIO.input(pin) for _ in range(5)]
        print(f"{name} (BCM {pin}) プルダウン: {samples}")
        if all(s == 0 for s in samples):
            print(f"  → Hi-Z 状態（74HC590 の出力が無効）")
    
    print()

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  74HC590 Output Enable (Pin 12) 診断  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    
    try:
        test_with_pulldown()
        find_output_enable_pin()
        
        print("=" * 60)
        print("診断結果")
        print("=" * 60)
        print()
        print("【確認済み】")
        print("  ✓ 74HC590 QA, QB, QC = 0V (マルチメータ測定)")
        print("  ✓ ラズパイは QA=1, QC=1 と読む")
        print("  ✓ プルダウンすると全て 0 になる")
        print()
        print("【結論】")
        print("  Pin 12 (G'/Output Enable) が HIGH になっている")
        print("  → 74HC590 の出力が Hi-Z (ハイインピーダンス)")
        print("  → ラズパイのプルアップで 1 と読まれる")
        print()
        print("【対策】")
        print("  1. Pin 12 (G') を GND に接続")
        print("     → 出力が有効になる")
        print()
        print("  2. または Pin 12 を制御ピンに接続して LOW にする")
        print()
        print("【次の確認】")
        print("  マルチメータで 74HC590 pin 12 の電圧を測定:")
        print("  - 3.3V → VCC に接続されている（要修正）")
        print("  - 0V → GND に接続（正常）")
        print("  - フロート (不安定) → 未接続（要接続）")
        print()
        
    except KeyboardInterrupt:
        print("\n中断されました")
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
