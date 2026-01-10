#!/usr/bin/env python3
"""
74HC590 ピン配線の検証

判明した事実:
- QB (pin 1) → BCM 21: 正常に LOW 駆動
- QA (pin 15) → BCM 13: Hi-Z (配線ミス？)
- QC (pin 2) → BCM 25: Hi-Z (配線ミス？)

QA と QC が Hi-Z = 配線が間違っている可能性
"""

import RPi.GPIO as GPIO
import time

# 全ての使用可能な GPIO ピン
ALL_PINS = [13, 17, 18, 21, 22, 23, 24, 25, 27]

def scan_all_pins_for_low():
    """全 GPIO ピンをスキャンして LOW 駆動されているピンを探す"""
    print("=" * 60)
    print("全 GPIO ピンスキャン - LOW 駆動ピンを探す")
    print("=" * 60)
    print()
    print("QB (pin 1) が BCM 21 で LOW 駆動されているように、")
    print("QA と QC も LOW 駆動されているピンがあるはず")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    low_pins = []
    
    for pin in ALL_PINS:
        try:
            # プルアップで読む
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            time.sleep(0.05)
            val_up = GPIO.input(pin)
            
            # プルダウンで読む
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            time.sleep(0.05)
            val_down = GPIO.input(pin)
            
            # プル無しで読む
            GPIO.setup(pin, GPIO.IN)
            time.sleep(0.05)
            val_none = GPIO.input(pin)
            
            status = ""
            if val_up == 0 and val_down == 0 and val_none == 0:
                status = "LOW 駆動 ★"
                low_pins.append(pin)
            elif val_up == 1 and val_down == 1 and val_none == 1:
                status = "HIGH 駆動"
            elif val_up == 1 and val_down == 0:
                status = "Hi-Z"
            else:
                status = "不明"
            
            print(f"BCM {pin:2d} (物理{GPIO.gpio_function(pin):2s}): PU={val_up} PD={val_down} PN={val_none} → {status}")
            
        except Exception as e:
            print(f"BCM {pin:2d}: エラー - {e}")
    
    print()
    print(f"LOW 駆動されているピン: {low_pins}")
    print()
    
    GPIO.cleanup()
    return low_pins

def verify_physical_pins():
    """物理ピン配置の確認ガイド"""
    print("=" * 60)
    print("配線確認ガイド")
    print("=" * 60)
    print()
    print("【現在の期待される配線】")
    print("  ラズパイ物理33p (BCM 13) → 74HC590 pin 15 (QA)")
    print("  ラズパイ物理40p (BCM 21) → 74HC590 pin 1 (QB) ✓ 動作確認済み")
    print("  ラズパイ物理22p (BCM 25) → 74HC590 pin 2 (QC)")
    print()
    print("【74HC590 ピン配置】")
    print("        ┌─────┴─────┐")
    print("  QB  1 │○          │ 16  VCC (3.6V)")
    print("  QC  2 │           │ 15  QA")
    print("      3 │           │ 14  SI")
    print("      4 │           │ 13  RCLK")
    print("      5 │  74HC590  │ 12  G' (GND)")
    print("      6 │           │ 11  CCLK")
    print("      7 │           │ 10  CCLR")
    print("  GND 8 │           │  9  QH'")
    print("        └───────────┘")
    print()
    print("【確認してください】")
    print("  1. 物理33p のワイヤーが 74HC590 の pin 15 に接続されているか")
    print("  2. 物理22p のワイヤーが 74HC590 の pin 2 に接続されているか")
    print()
    print("【可能性】")
    print("  - QA と QC の配線が外れている")
    print("  - または別のピンに接続されている")
    print("  - 上記スキャンで LOW 駆動のピンが見つかれば、そこに配線されている")
    print()

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  74HC590 配線診断  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    
    low_pins = scan_all_pins_for_low()
    verify_physical_pins()
    
    print("=" * 60)
    print("診断結果")
    print("=" * 60)
    print()
    
    if len(low_pins) == 1 and 21 in low_pins:
        print("【結果】")
        print("  BCM 21 (QB) だけが LOW 駆動")
        print("  QA と QC が見つからない")
        print()
        print("【考えられる原因】")
        print("  1. QA (pin 15) と QC (pin 2) の配線が外れている")
        print("  2. カウンタ値が 001 (= 1) で、QA=0, QB=1, QC=0")
        print("     → これなら正常動作")
        print()
        print("【次の確認】")
        print("  カウンタにパルスを入力して値が変化するか確認")
        print("  または CCLR でカウンタをクリアしてみる")
    elif len(low_pins) == 3:
        print("【結果】")
        print(f"  3つの LOW 駆動ピンが見つかりました: {low_pins}")
        print("  これらが QA, QB, QC の可能性があります")
        print()
        print("【次の確認】")
        print("  各ピンが 74HC590 のどのピンに接続されているか確認")
    else:
        print("【結果】")
        print(f"  LOW 駆動ピン: {low_pins}")
        print("  予想外の結果です")
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
