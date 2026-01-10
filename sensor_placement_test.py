#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
センサ実機テスト - 車輪カウンタ + 白線検出

目的:
  - 車輪RPR220の感度確認
  - 白線RPR220の感度確認
  - センサ配置の最適化

使用方法:
  python sensor_placement_test.py
"""

import RPi.GPIO as GPIO
import time
import sys

# ピン定義
# 74HC590 車輪カウンタ
QA_PIN = 13
QB_PIN = 21
QC_PIN = 25
CCLR_PIN = 17
RCLK_PIN = 24

# 74HC74 白線検出
WHITE_Q_PIN = 23
WHITE_CLR_PIN = 18

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 車輪カウンタ (入力)
    GPIO.setup(QA_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # 車輪カウンタ制御 (出力)
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    
    # 白線検出 (入力/出力)
    GPIO.setup(WHITE_Q_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(WHITE_CLR_PIN, GPIO.OUT, initial=GPIO.HIGH)

def read_wheel_counter():
    """車輪カウンタ値を読み取り"""
    d0 = GPIO.input(QA_PIN)
    d1 = GPIO.input(QB_PIN)
    d2 = GPIO.input(QC_PIN)
    return d0 | (d1 << 1) | (d2 << 2)

def clear_wheel_counter():
    """車輪カウンタをクリア"""
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.001)

def latch_wheel_counter():
    """車輪カウンタをラッチ"""
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)

def read_white_line():
    """白線検出状態を読み取り"""
    return GPIO.input(WHITE_Q_PIN) == 1

def clear_white_line():
    """白線検出をクリア"""
    GPIO.output(WHITE_CLR_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(WHITE_CLR_PIN, GPIO.HIGH)
    time.sleep(0.001)

def test_wheel_sensor():
    """車輪センサテスト"""
    print("=" * 70)
    print("【車輪センサ感度テスト】")
    print("=" * 70)
    print()
    print("車輪をゆっくり回してください")
    print("カウンタが増えることを確認")
    print("Enter で終了")
    print()
    
    # カウンタクリア
    clear_wheel_counter()
    latch_wheel_counter()
    
    prev_count = 0
    pulse_count = 0
    
    print("開始... (Enter で次のテストへ)")
    print()
    
    while True:
        # キー入力チェック（非ブロッキング）
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        
        # ラッチして読み取り
        latch_wheel_counter()
        count = read_wheel_counter()
        
        if count != prev_count:
            ts = time.strftime('%H:%M:%S')
            diff = (count - prev_count) % 8
            pulse_count += diff
            print(f"[{ts}] カウンタ: {prev_count}→{count} (差分:{diff}) 累積パルス:{pulse_count}")
            prev_count = count
        
        time.sleep(0.05)  # 50ms間隔
    
    print()
    print(f"✓ 検出パルス合計: {pulse_count}回")
    print()
    
    if pulse_count == 0:
        print("⚠ パルスが検出されませんでした")
        print()
        print("確認事項:")
        print("  □ RPR220の電源(5V)が供給されているか")
        print("  □ センサと車輪の距離は適切か (5-10mm程度)")
        print("  □ 車輪に反射用マーカー(白テープ等)が貼ってあるか")
        print("  □ センサの向きは正しいか (LEDとフォトトランジスタが反射面に向いている)")
    else:
        print("✓ センサは動作しています")
        print()
        print("感度チェック:")
        if pulse_count < 5:
            print("  ⚠ パルスが少ない → センサ距離を調整")
        elif pulse_count > 50:
            print("  ⚠ パルスが多すぎる → ノイズの可能性、距離を調整")
        else:
            print("  ✓ 良好な感度")
    print()

def test_white_line_sensor():
    """白線センサテスト"""
    print("=" * 70)
    print("【白線センサ感度テスト】")
    print("=" * 70)
    print()
    print("白線の上にセンサを置いてテスト")
    print("Enter で終了")
    print()
    
    # クリア
    clear_white_line()
    
    detection_count = 0
    prev_state = False
    
    print("開始... (白線の上を移動してください)")
    print()
    
    while True:
        # キー入力チェック
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        
        detected = read_white_line()
        
        if detected and not prev_state:
            ts = time.strftime('%H:%M:%S')
            detection_count += 1
            print(f"[{ts}] ★ 白線検出 #{detection_count}")
            
            # 自動クリア
            time.sleep(0.3)
            clear_white_line()
            prev_state = False
        elif not detected:
            prev_state = False
        
        time.sleep(0.05)
    
    print()
    print(f"✓ 検出回数: {detection_count}回")
    print()
    
    if detection_count == 0:
        print("⚠ 白線が検出されませんでした")
        print()
        print("確認事項:")
        print("  □ RPR220の電源(5V)が供給されているか")
        print("  □ センサと白線の距離は適切か (5-10mm程度)")
        print("  □ 白線の幅は十分か (20mm以上)")
        print("  □ 白線のコントラストは十分か (黒地に白)")
        print("  □ センサの向きは正しいか")
    else:
        print("✓ センサは動作しています")
        print()
        print("感度チェック:")
        if detection_count < 3:
            print("  ⚠ 検出回数が少ない → センサ距離を調整")
        else:
            print("  ✓ 良好な感度")
    print()

def main():
    print("=" * 70)
    print("センサ実機テスト - 車輪 + 白線")
    print("=" * 70)
    print()
    
    try:
        setup_gpio()
        
        # テスト1: 車輪センサ
        test_wheel_sensor()
        
        input("白線センサテストに進みます (Enter)...")
        print()
        
        # テスト2: 白線センサ
        test_white_line_sensor()
        
        print("=" * 70)
        print("テスト完了")
        print("=" * 70)
        print()
        print("【次のステップ】")
        print()
        print("1. センサ配置の最適化")
        print("   - 車輪センサ: 車輪との距離 5-10mm")
        print("   - 白線センサ: 地面との距離 5-10mm")
        print()
        print("2. センサの固定")
        print("   - 振動で位置がずれないように")
        print("   - 配線のストレス軽減")
        print()
        print("3. 実走行テスト")
        print("   - 統合プログラムで動作確認")
        print()
        
    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
