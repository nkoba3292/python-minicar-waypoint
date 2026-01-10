#!/usr/bin/env python3
"""
74HC590 詳細診断 - Pin 12 が GND なのに出力が Hi-Z の原因調査

確認済み事実:
1. Pin 1, 2, 15 (QB, QC, QA) = 0V (マルチメータ)
2. Pin 12 (G') = 0V (GND) ← 出力有効のはず
3. ラズパイは QA=1, QC=1 と読む (QB=0)

矛盾: Pin 12 = GND なら出力は有効なはず
     なのに電圧 0V = 出力は無効？

可能性:
1. VCC (pin 16) が供給されていない
2. GND (pin 8) が未接続
3. 測定ミス（別のピンを測定していた）
"""

import RPi.GPIO as GPIO
import time

DATA_PINS = [13, 21, 25]  # QA, QB, QC

def test_drive_strength():
    """74HC590 の出力駆動能力をテスト"""
    print("=" * 60)
    print("74HC590 出力駆動力テスト")
    print("=" * 60)
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    for pin in DATA_PINS:
        print(f"BCM {pin} のテスト:")
        
        # プルダウンで読む
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.1)
        val_down = GPIO.input(pin)
        print(f"  プルダウン: {val_down}")
        
        # プルアップで読む
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        time.sleep(0.1)
        val_up = GPIO.input(pin)
        print(f"  プルアップ: {val_up}")
        
        # プル無しで読む
        GPIO.setup(pin, GPIO.IN)
        time.sleep(0.1)
        val_none = GPIO.input(pin)
        print(f"  プル無し: {val_none}")
        
        # 診断
        if val_down == 0 and val_up == 1 and val_none == 1:
            print(f"  → Hi-Z (ハイインピーダンス) 状態")
        elif val_down == 0 and val_up == 0 and val_none == 0:
            print(f"  → LOW 駆動（正常）")
        elif val_down == 1 and val_up == 1 and val_none == 1:
            print(f"  → HIGH 駆動（正常）")
        else:
            print(f"  → 不明な状態")
        
        print()
    
    GPIO.cleanup()

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  74HC590 矛盾状態の診断  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    print("【矛盾している状態】")
    print("  - マルチメータ: QA, QB, QC = 0V")
    print("  - Pin 12 (G') = 0V (GND) ← 出力有効のはず")
    print("  - ラズパイ読み取り: QA=1, QB=0, QC=1")
    print()
    print("【可能性のある原因】")
    print("  1. VCC (pin 16) が供給されていない")
    print("  2. GND (pin 8) が未接続")
    print("  3. ラズパイの GPIO が別の回路と接続")
    print("  4. 測定ミス")
    print()
    
    test_drive_strength()
    
    print("=" * 60)
    print("次の確認事項")
    print("=" * 60)
    print()
    print("【重要】以下を確認してください:")
    print()
    print("1. 74HC590 の VCC (pin 16) 電圧測定")
    print("   期待値: 3.0V ~ 3.6V")
    print("   0V なら電源が供給されていない")
    print()
    print("2. 74HC590 の GND (pin 8) 確認")
    print("   ラズパイ GND と導通があるか")
    print()
    print("3. ピン番号の再確認")
    print("   74HC590 を上から見て:")
    print("   ┌─────┴─────┐")
    print("  1│○          │16 ← VCC")
    print("  2│           │15")
    print("   :           :")
    print("  8│           │9")
    print("   └───────────┘")
    print("   ↑ GND")
    print()
    print("4. ラズパイのピン配線確認")
    print("   物理33p (BCM 13) → 74HC590 pin 15 (QA)")
    print("   物理40p (BCM 21) → 74HC590 pin 1 (QB)")
    print("   物理22p (BCM 25) → 74HC590 pin 2 (QC)")
    print()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断されました")
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
