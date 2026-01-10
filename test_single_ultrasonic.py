#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
超音波センサー1個テスト - ハードウェア診断用
"""

import RPi.GPIO as GPIO
import time

# テストするセンサー（Front）
TRIG_PIN = 15  # BOARD番号
ECHO_PIN = 26  # BOARD番号

print("="*60)
print("Single Ultrasonic Sensor Test")
print(f"TRIG: GPIO{TRIG_PIN} (BOARD)")
print(f"ECHO: GPIO{ECHO_PIN} (BOARD)")
print("="*60)

try:
    # GPIO初期化
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    # ピン設定
    print(f"Setting up TRIG pin {TRIG_PIN}...")
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    print(f"Setting up ECHO pin {ECHO_PIN}...")
    GPIO.setup(ECHO_PIN, GPIO.IN)
    
    print("\nStarting measurements (Ctrl+C to stop)...")
    print("-"*60)
    
    count = 0
    success_count = 0
    
    while True:
        count += 1
        
        # トリガー送信
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)  # 10μs
        GPIO.output(TRIG_PIN, GPIO.LOW)
        
        # エコー受信（詳細ログ）
        start_time = time.time()
        timeout_low = False
        timeout_high = False
        
        # LOW待ち
        while GPIO.input(ECHO_PIN) == GPIO.LOW:
            if time.time() - start_time > 0.02:
                timeout_low = True
                break
        sigoff = time.time()
        
        if timeout_low:
            print(f"[{count:3d}] TIMEOUT - Echo never went HIGH (TRIG not working or no connection)")
            time.sleep(1)
            continue
        
        # HIGH待ち
        while GPIO.input(ECHO_PIN) == GPIO.HIGH:
            if time.time() - sigoff > 0.02:
                timeout_high = True
                break
        sigon = time.time()
        
        if timeout_high:
            print(f"[{count:3d}] TIMEOUT - Echo stuck HIGH (sensor or wiring issue)")
            time.sleep(1)
            continue
        
        # 距離計算
        distance = (sigon - sigoff) * 34000 / 2
        distance = min(distance, 200.0)
        
        success_count += 1
        success_rate = (success_count / count) * 100
        
        print(f"[{count:3d}] Distance: {distance:6.1f}cm | Time: {(sigon-sigoff)*1000000:.1f}μs | Success: {success_rate:.0f}%")
        
        # 次の測定まで待機
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n" + "="*60)
    print(f"Test stopped. Success rate: {success_count}/{count} ({(success_count/count*100):.1f}%)")
    print("="*60)

finally:
    GPIO.cleanup()
    print("GPIO cleaned up")
