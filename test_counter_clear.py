#!/usr/bin/env python3
"""
74HC590 カウンタクリアとカウント値の確認

現在のカウンタ値が 001 (decimal 1) の可能性
CCLRでクリアして 000 になるか確認
"""

import RPi.GPIO as GPIO
import time

# ピン設定
DATA_PINS = {
    'D0 (QA-pin15)': 13,
    'D1 (QB-pin1)': 21,
    'D2 (QC-pin2)': 25,
}

CCLR_PIN = 17  # BCM 17 (物理11p) → 74HC590 pin 10
RCLK_PIN = 24  # BCM 24 (物理18p) → 74HC590 pin 13

def read_counter():
    """カウンタ値を読み取る"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # データピンを入力に設定（プルダウン）
    for name, pin in DATA_PINS.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    time.sleep(0.1)
    
    # 読み取り
    d0 = GPIO.input(13)
    d1 = GPIO.input(21)
    d2 = GPIO.input(25)
    
    value = (d2 << 2) | (d1 << 1) | d0
    
    return d0, d1, d2, value

def clear_counter():
    """カウンタをクリア"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 制御ピンを出力に設定
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    
    # CCLR を HIGH にする（通常状態）
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)
    
    # CCLR を LOW パルス（カウンタクリア）
    print("カウンタをクリア中...")
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.05)
    
    # RCLK でラッチ（カウンタ値を出力レジスタに転送）
    print("ラッチ中...")
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.05)

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  74HC590 カウンタクリアテスト  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    
    try:
        # クリア前の値を読む
        print("=" * 60)
        print("クリア前のカウンタ値")
        print("=" * 60)
        d0_before, d1_before, d2_before, val_before = read_counter()
        print(f"D2 D1 D0 = {d2_before} {d1_before} {d0_before} (Binary)")
        print(f"値 = {val_before} (Decimal)")
        print()
        
        # カウンタをクリア
        print("=" * 60)
        print("カウンタクリア実行")
        print("=" * 60)
        clear_counter()
        print("完了")
        print()
        
        # クリア後の値を読む
        print("=" * 60)
        print("クリア後のカウンタ値")
        print("=" * 60)
        d0_after, d1_after, d2_after, val_after = read_counter()
        print(f"D2 D1 D0 = {d2_after} {d1_after} {d0_after} (Binary)")
        print(f"値 = {val_after} (Decimal)")
        print()
        
        # 判定
        print("=" * 60)
        print("結果")
        print("=" * 60)
        print()
        
        if val_before == val_after:
            print(f"⚠ カウンタ値が変化しませんでした ({val_before} → {val_after})")
            print()
            print("考えられる原因:")
            print("  1. CCLR (BCM 17) が 74HC590 pin 10 に正しく接続されていない")
            print("  2. RCLK (BCM 24) が 74HC590 pin 13 に正しく接続されていない")
            print("  3. カウンタがすでに 0 だった")
        else:
            print(f"✓ カウンタ値が変化しました ({val_before} → {val_after})")
            if val_after == 0:
                print("✓ カウンタは正常にクリアされました")
                print()
                print("【重要】74HC590 は正常に動作しています！")
                print("  - QA, QB, QC の配線は正しい")
                print("  - カウンタは動作可能")
                print("  - センサ入力を接続すればカウント開始できます")
            else:
                print(f"⚠ クリア後も値が {val_after} です")
        
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
