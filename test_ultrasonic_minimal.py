#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
超音波センサー最小限テスト - WiFi切断原因特定用
段階的にセンサー数を増やしてテスト
"""

import RPi.GPIO as GPIO
import time
import sys

def test_single_sensor(trig_pin, echo_pin, sensor_name="Sensor"):
    """1つのセンサーのみテスト"""
    print(f"\n{'='*60}")
    print(f"Testing {sensor_name}: Trig={trig_pin}, Echo={echo_pin}")
    print(f"{'='*60}")
    
    try:
        # GPIO初期化
        GPIO.setwarnings(False)
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        
        # ピン設定
        GPIO.setup(trig_pin, GPIO.OUT, initial=GPIO.LOW)
        time.sleep(0.1)
        GPIO.setup(echo_pin, GPIO.IN)
        time.sleep(0.1)
        
        print(f"[OK] GPIO setup complete for {sensor_name}")
        
        # 10回測定テスト
        print(f"Starting 10 measurements (2 second interval)...")
        for i in range(10):
            try:
                # トリガー送信
                GPIO.output(trig_pin, GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(trig_pin, GPIO.LOW)
                
                # エコー受信
                start_time = time.time()
                timeout_count = 0
                
                # LOW待ち（タイムアウト付き）
                while GPIO.input(echo_pin) == GPIO.LOW:
                    if time.time() - start_time > 0.02:
                        timeout_count += 1
                        break
                sigoff = time.time()
                
                # HIGH待ち（タイムアウト付き）
                while GPIO.input(echo_pin) == GPIO.HIGH:
                    if time.time() - sigoff > 0.02:
                        timeout_count += 1
                        break
                sigon = time.time()
                
                # 距離計算
                if timeout_count == 0:
                    distance = (sigon - sigoff) * 34000 / 2
                    distance = min(distance, 200)
                    status = "OK"
                else:
                    distance = 200
                    status = "TIMEOUT"
                
                print(f"  [{i+1}/10] {distance:.1f}cm ({status})")
                
                # WiFi接続確認（pingで簡易確認）
                if i % 3 == 0:
                    import subprocess
                    try:
                        result = subprocess.run(['ping', '-c', '1', '-W', '1', '8.8.8.8'], 
                                              capture_output=True, timeout=2)
                        wifi_status = "Connected" if result.returncode == 0 else "Disconnected"
                        print(f"       WiFi: {wifi_status}")
                    except:
                        print(f"       WiFi: Unable to check")
                
                # 測定間隔
                time.sleep(2)
                
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"  [{i+1}/10] Measurement error: {e}")
        
        print(f"\n[OK] {sensor_name} test completed successfully")
        return True
        
    except Exception as e:
        print(f"\n[ERROR] {sensor_name} test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # クリーンアップ
        try:
            GPIO.cleanup([trig_pin, echo_pin])
        except:
            pass

def test_multiple_sensors_sequential():
    """複数センサーを順次テスト（高速ループなし）"""
    print(f"\n{'='*60}")
    print(f"Testing MULTIPLE sensors (Sequential, Slow)")
    print(f"{'='*60}")
    
    sensors = [
        (15, 26, "Front"),
        (13, 24, "Left45"),
        (35, 37, "Left90"),
        (32, 31, "Right45"),
        (36, 38, "Right90")
    ]
    
    try:
        # GPIO初期化
        GPIO.setwarnings(False)
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        
        # 全ピン設定
        for trig, echo, name in sensors:
            GPIO.setup(trig, GPIO.OUT, initial=GPIO.LOW)
            time.sleep(0.05)
            GPIO.setup(echo, GPIO.IN)
            time.sleep(0.05)
            print(f"[OK] {name} pins configured")
        
        # 5サイクル測定（間隔長め）
        print(f"\nStarting 5 measurement cycles (5 second interval)...")
        for cycle in range(5):
            print(f"\nCycle {cycle+1}/5:")
            distances = []
            
            for trig, echo, name in sensors:
                try:
                    # 測定
                    GPIO.output(trig, GPIO.HIGH)
                    time.sleep(0.00001)
                    GPIO.output(trig, GPIO.LOW)
                    
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
                    distances.append(f"{name}={dist:.1f}")
                    
                    # センサー間待機（重要）
                    time.sleep(0.1)
                    
                except Exception as e:
                    distances.append(f"{name}=ERROR")
            
            print(f"  {', '.join(distances)}")
            
            # WiFi確認
            if cycle % 2 == 0:
                import subprocess
                try:
                    result = subprocess.run(['ping', '-c', '1', '-W', '1', '8.8.8.8'], 
                                          capture_output=True, timeout=2)
                    print(f"  WiFi: {'Connected' if result.returncode == 0 else 'Disconnected'}")
                except:
                    print(f"  WiFi: Check failed")
            
            # サイクル間待機
            time.sleep(5)
        
        print(f"\n[OK] Multiple sensor test completed")
        return True
        
    except Exception as e:
        print(f"\n[ERROR] Multiple sensor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        GPIO.cleanup()

def main():
    print("="*60)
    print("Ultrasonic Sensor WiFi Disconnection Test")
    print("="*60)
    
    if len(sys.argv) > 1 and sys.argv[1] == "single":
        # 単一センサーテスト（最も安全）
        print("\nMode: Single sensor test")
        test_single_sensor(15, 26, "Front Sensor")
    
    elif len(sys.argv) > 1 and sys.argv[1] == "multi":
        # 複数センサーテスト（順次、低速）
        print("\nMode: Multiple sensors (sequential, slow)")
        test_multiple_sensors_sequential()
    
    else:
        print("\nUsage:")
        print("  python3 test_ultrasonic_minimal.py single  # Test 1 sensor only")
        print("  python3 test_ultrasonic_minimal.py multi   # Test all 5 sensors (slow)")
        print("\nRecommended: Start with 'single' test first")
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        GPIO.cleanup()
    except Exception as e:
        print(f"\n\nTest failed: {e}")
        import traceback
        traceback.print_exc()
        GPIO.cleanup()
