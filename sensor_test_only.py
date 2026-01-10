#!/usr/bin/env python3
"""
02_IMUPLUS_togikai_sample_new_v0.1.py のセンサテスト版
PCA9685を無効化してセンサ（74HC590、白線検出）のみテスト
"""

import RPi.GPIO as GPIO
import time
import sys

# GPIOピン番号の指示方法
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

print("=" * 60)
print("センサテストモード（PCA9685無効）")
print("=" * 60)
print()

# 白線検出用ピン設定（例）
# BOARD番号で指定（物理ピン番号）
LATCH_OUTPUT = 11  # 物理ピン11 (GPIO17 BCM) - ラッチ出力読み取り
LATCH_CLEAR = 12   # 物理ピン12 (GPIO18 BCM) - ラッチクリア信号
GPIO.setup(LATCH_OUTPUT, GPIO.IN)
GPIO.setup(LATCH_CLEAR, GPIO.OUT, initial=GPIO.HIGH)  # CLR=HIGH(非クリア状態)

# 74HC590 カウンタ読み取り用ピン（BCMモード換算）
# 物理33p = BCM 13, 物理40p = BCM 21, 物理22p = BCM 25
COUNTER_D0 = 33  # 物理ピン33 (BCM 13)
COUNTER_D1 = 40  # 物理ピン40 (BCM 21)
COUNTER_D2 = 22  # 物理ピン22 (BCM 25)
COUNTER_CCLR = 11  # 物理ピン11 (BCM 17)
COUNTER_RCLK = 18  # 物理ピン18 (BCM 24)

GPIO.setup(COUNTER_D0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(COUNTER_D1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(COUNTER_D2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(COUNTER_CCLR, GPIO.OUT)
GPIO.setup(COUNTER_RCLK, GPIO.OUT)

print("✓ GPIOピン初期化完了")
print()
print("【設定】")
print(f"  白線ラッチ出力: 物理{LATCH_OUTPUT}p")
print(f"  白線クリア:     物理{LATCH_CLEAR}p")
print(f"  カウンタD0:     物理{COUNTER_D0}p (BCM 13)")
print(f"  カウンタD1:     物理{COUNTER_D1}p (BCM 21)")
print(f"  カウンタD2:     物理{COUNTER_D2}p (BCM 25)")
print(f"  カウンタクリア: 物理{COUNTER_CCLR}p (BCM 17)")
print(f"  カウンタラッチ: 物理{COUNTER_RCLK}p (BCM 24)")
print()

def read_counter():
    """74HC590 カウンタ値を読む"""
    d0 = GPIO.input(COUNTER_D0)
    d1 = GPIO.input(COUNTER_D1)
    d2 = GPIO.input(COUNTER_D2)
    value = (d2 << 2) | (d1 << 1) | d0
    return value, d0, d1, d2

def latch_counter():
    """カウンタ値をラッチ"""
    GPIO.output(COUNTER_RCLK, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(COUNTER_RCLK, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(COUNTER_RCLK, GPIO.LOW)
    time.sleep(0.005)

def clear_counter():
    """カウンタをクリア"""
    GPIO.output(COUNTER_CCLR, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(COUNTER_CCLR, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(COUNTER_CCLR, GPIO.HIGH)
    time.sleep(0.01)

def read_line_sensor():
    """白線センサを読む"""
    return GPIO.input(LATCH_OUTPUT)

def test_sensors():
    """センサテスト"""
    print("=" * 60)
    print("センサモニタリング開始")
    print("=" * 60)
    print()
    print("Ctrl+C で終了")
    print()
    
    # カウンタをクリア
    print("カウンタをクリアしています...")
    clear_counter()
    latch_counter()
    time.sleep(0.1)
    
    last_counter = None
    last_line = None
    
    try:
        while True:
            # カウンタ読み取り
            latch_counter()
            time.sleep(0.01)
            counter_val, d0, d1, d2 = read_counter()
            
            # 白線センサ読み取り
            line_val = read_line_sensor()
            
            # 変化があれば表示
            if counter_val != last_counter or line_val != last_line:
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] カウンタ: {counter_val} (D2={d2} D1={d1} D0={d0}) | 白線: {line_val}")
                
                last_counter = counter_val
                last_line = line_val
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print()
        print("=" * 60)
        print("テスト終了")
        print("=" * 60)
        print()

def main():
    try:
        test_sensors()
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        GPIO.cleanup()
        print("GPIO クリーンアップ完了")

if __name__ == "__main__":
    main()
