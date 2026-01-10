#!/usr/bin/env python3
"""
test_hall_sensor_counter.py

ホールIC + 74HC590カウンタの動作確認プログラム

目的:
- RPR220からホールICへの置き換え後の動作確認
- カウンタ値がホイール回転に応じて増加することを確認
- リアルタイムでカウンタ値・速度を表示

使用方法:
  sudo python3 test_hall_sensor_counter.py
  
手順:
1. 車輪を手で回転させる
2. カウンタが増加することを確認
3. Ctrl+Cで終了

ピン配線（sensor_placement_testと同じ）:
  D0 (LSB) -> BCM 13 (BOARD 33)
  D1       -> BCM 21 (BOARD 40)
  D2       -> BCM 25 (BOARD 22)
  CCLR     -> BCM 17 (BOARD 11) - Counter Clear
  LATCHSTRO-> BCM 24 (BOARD 18) - Latch Strobe
"""

import time
import RPi.GPIO as GPIO
from wheel_counter_74hc590 import Counter74HC590

# ピン定義（BCMピン番号）
WHEEL_LATCH_PIN = 24   # RCLK (BOARD 18)
WHEEL_DATA_PINS = [13, 21, 25]  # D0, D1, D2 (BOARD 33, 40, 22)
WHEEL_CLEAR_PIN = 17   # CCLR (BOARD 11)

# 車輪パラメータ
WHEEL_DIAMETER = 0.0557  # 車輪直径 [m] (55.7mm) - 実測周長175mm
WHEEL_PPR = 2.0  # SK1816ラッチ型: N極+S極で2パルス/回転

def main():
    print("=" * 60)
    print("ホールIC + 74HC590 動作確認プログラム")
    print("=" * 60)
    print(f"ピン配線:")
    print(f"  LATCH: BCM {WHEEL_LATCH_PIN} (BOARD 18)")
    print(f"  DATA:  BCM {WHEEL_DATA_PINS} (BOARD 33, 40, 22)")
    print(f"  CLEAR: BCM {WHEEL_CLEAR_PIN} (BOARD 11)")
    print(f"車輪パラメータ:")
    print(f"  直径: {WHEEL_DIAMETER*1000:.1f}mm, PPR: {WHEEL_PPR}")
    print(f"  チャタリング対策: delta値を1に制限")
    print("=" * 60)
    print("車輪を手で回転させてカウンタの動作を確認してください。")
    print("Ctrl+Cで終了します。")
    print("=" * 60)
    
    # GPIO初期化
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    # カウンタ初期化
    try:
        wheel_counter = Counter74HC590(WHEEL_LATCH_PIN, WHEEL_DATA_PINS, WHEEL_CLEAR_PIN)
        print(f"[OK] カウンタ初期化完了 (初期値: {wheel_counter.last_value})")
    except Exception as e:
        print(f"[ERROR] カウンタ初期化失敗: {e}")
        GPIO.cleanup()
        return
    
    # 測定ループ
    last_time = time.time()
    last_total = wheel_counter.total_count
    
    try:
        print("\n{:>10s} {:>10s} {:>10s} {:>10s} {:>15s}".format(
            "時刻[s]", "現在値", "デルタ", "累積", "速度[m/s]"
        ))
        print("-" * 60)
        
        while True:
            current_time = time.time()
            elapsed = current_time - last_time
            
            # カウンタ読み取り
            current_value, delta_pulses, current_total = wheel_counter.read_incremental()
            
            # 速度計算
            if elapsed > 0.001 and delta_pulses > 0:
                wheel_circumference = WHEEL_DIAMETER * 3.14159
                distance = (delta_pulses / WHEEL_PPR) * wheel_circumference
                velocity = distance / elapsed
            else:
                velocity = 0.0
            
            # 表示（変化があった場合のみ）
            if delta_pulses > 0 or elapsed > 1.0:
                print("{:10.3f} {:10d} {:10d} {:10d} {:15.4f}".format(
                    current_time - wheel_counter.start_time,
                    current_value,
                    delta_pulses,
                    current_total,
                    velocity
                ))
                last_time = current_time
                last_total = current_total
            
            time.sleep(0.05)  # 20Hzサンプリング
    
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("測定終了")
        final_value, _, final_total = wheel_counter.read_incremental()
        print(f"最終累積カウント: {final_total}")
        print(f"最終現在値: {final_value}")
        
        # 回転数計算
        rotations = final_total / WHEEL_PPR
        distance = rotations * WHEEL_DIAMETER * 3.14159
        print(f"推定回転数: {rotations:.2f}回転")
        print(f"推定移動距離: {distance:.3f}m")
        print("=" * 60)
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
