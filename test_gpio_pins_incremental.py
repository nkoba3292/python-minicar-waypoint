#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
超音波センサー1個ずつ追加テスト - WiFi切断原因特定用
"""

import RPi.GPIO as GPIO
import time
import subprocess
import sys

def check_wifi_connection():
    """WiFi接続確認"""
    try:
        result = subprocess.run(['ping', '-c', '1', '-W', '1', '8.8.8.8'], 
                              capture_output=True, timeout=2)
        return result.returncode == 0
    except:
        return False

def test_gpio_pins_one_by_one():
    """GPIO ピンを1つずつ初期化してWiFi接続確認"""
    
    # 超音波センサーピン定義
    sensors = [
        (15, 26, "Front"),
        (13, 24, "Left45"),
        (35, 37, "Left90"),
        (32, 31, "Right45"),
        (36, 38, "Right90")
    ]
    
    print("="*60)
    print("GPIO Pin Test - One by One")
    print("="*60)
    print("\nThis test will initialize GPIO pins one by one")
    print("and check WiFi connection after each initialization.")
    print("")
    
    try:
        # GPIO初期化
        GPIO.setwarnings(False)
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        print("[OK] GPIO mode set to BOARD")
        
        # WiFi初期状態確認
        print("\n[CHECK] Initial WiFi status...")
        if check_wifi_connection():
            print("  ✅ WiFi: Connected")
        else:
            print("  ❌ WiFi: Disconnected (already disconnected before test!)")
            return
        
        time.sleep(2)
        
        # 各センサーのピンを1つずつ設定
        for i, (trig, echo, name) in enumerate(sensors, 1):
            print(f"\n{'='*60}")
            print(f"Step {i}/5: Initializing {name} sensor")
            print(f"  TRIG pin: {trig}, ECHO pin: {echo}")
            print(f"{'='*60}")
            
            # TRIGピン設定
            print(f"  Setting up TRIG pin {trig}...")
            GPIO.setup(trig, GPIO.OUT, initial=GPIO.LOW)
            time.sleep(0.1)
            print(f"  [OK] TRIG pin {trig} configured")
            
            # WiFi確認
            print(f"  [CHECK] WiFi status after TRIG setup...")
            time.sleep(0.5)
            if check_wifi_connection():
                print(f"    ✅ WiFi: Still connected")
            else:
                print(f"    ❌ WiFi: DISCONNECTED after TRIG pin {trig} setup!")
                print(f"\n⚠️  PROBLEM FOUND: TRIG pin {trig} ({name}) causes WiFi disconnect")
                return
            
            # ECHOピン設定
            print(f"  Setting up ECHO pin {echo}...")
            GPIO.setup(echo, GPIO.IN)
            time.sleep(0.1)
            print(f"  [OK] ECHO pin {echo} configured")
            
            # WiFi確認
            print(f"  [CHECK] WiFi status after ECHO setup...")
            time.sleep(0.5)
            if check_wifi_connection():
                print(f"    ✅ WiFi: Still connected")
            else:
                print(f"    ❌ WiFi: DISCONNECTED after ECHO pin {echo} setup!")
                print(f"\n⚠️  PROBLEM FOUND: ECHO pin {echo} ({name}) causes WiFi disconnect")
                return
            
            print(f"\n  ✅ {name} sensor pins initialized successfully, WiFi OK")
            time.sleep(1)
        
        print(f"\n{'='*60}")
        print("ALL PINS INITIALIZED - WiFi Still Connected!")
        print(f"{'='*60}")
        print("\nNow testing actual measurements...")
        
        # 実際の測定テスト（各センサー1回ずつ）
        for trig, echo, name in sensors:
            print(f"\n[MEASURE] {name} sensor...")
            try:
                # トリガー送信
                GPIO.output(trig, GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(trig, GPIO.LOW)
                
                # エコー受信
                start = time.time()
                while GPIO.input(echo) == GPIO.LOW:
                    if time.time() - start > 0.02:
                        break
                sigoff = time.time()
                
                while GPIO.input(echo) == GPIO.HIGH:
                    if time.time() - sigoff > 0.02:
                        break
                sigon = time.time()
                
                dist = min((sigon - sigoff) * 34000 / 2, 200)
                print(f"  Distance: {dist:.1f}cm")
                
                # WiFi確認
                time.sleep(0.5)
                if check_wifi_connection():
                    print(f"  ✅ WiFi: OK after measurement")
                else:
                    print(f"  ❌ WiFi: DISCONNECTED after {name} measurement!")
                    print(f"\n⚠️  PROBLEM: Measurement operation on {name} causes WiFi disconnect")
                    return
                
            except Exception as e:
                print(f"  ⚠️  Measurement error: {e}")
            
            time.sleep(1)
        
        print(f"\n{'='*60}")
        print("✅ ALL TESTS PASSED - WiFi Stable")
        print(f"{'='*60}")
        print("\nConclusion: GPIO initialization and measurement do NOT cause WiFi disconnect")
        print("Problem may be in threading or continuous operation.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up GPIO...")
        GPIO.cleanup()

if __name__ == "__main__":
    print("\n⚠️  WARNING: This test requires active internet connection")
    print("⚠️  Test will check WiFi by pinging 8.8.8.8")
    print("\nPress Ctrl+C to abort, or Enter to continue...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted")
        sys.exit(0)
    
    test_gpio_pins_one_by_one()
