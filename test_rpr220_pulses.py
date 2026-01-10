#!/usr/bin/env python3
"""
RPR220 フォトリフレクタのパルス検出テスト

74HC590 の CCLK (pin 11) に接続されている RPR220 の出力をモニタ
パルスが来ているか確認する

想定接続:
  RPR220 出力 → 74HC590 pin 11 (CCLK)
  
このスクリプトでは、CCLK に接続されているピンを探して
パルスをカウントする
"""

import RPi.GPIO as GPIO
import time

# 可能性のある CCLK 接続ピン
# 74HC590 pin 11 (CCLK) がどの GPIO に接続されているか不明なので探す
POSSIBLE_CCLK_PINS = [18, 23, 22, 27, 5, 6, 12, 16, 20, 26]

# カウンタ読み取り用ピン
DATA_PINS = {
    'D0': 13,
    'D1': 21,
    'D2': 25,
}

CCLR_PIN = 17
RCLK_PIN = 24

def read_counter_value():
    """現在のカウンタ値を読む"""
    d0 = GPIO.input(DATA_PINS['D0'])
    d1 = GPIO.input(DATA_PINS['D1'])
    d2 = GPIO.input(DATA_PINS['D2'])
    return (d2 << 2) | (d1 << 1) | d0

def latch_counter():
    """カウンタ値をラッチ（出力レジスタに転送）"""
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.01)

def clear_counter():
    """カウンタをクリア"""
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)

def test_manual_pulse():
    """手動でパルスを生成してカウンタが動作するか確認"""
    print("=" * 60)
    print("手動パルステスト")
    print("=" * 60)
    print()
    print("CCLK 候補ピンに手動でパルスを送ってカウンタが増えるか確認")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # データピンを入力に設定
    for pin in DATA_PINS.values():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # 制御ピンを出力に設定
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    
    # カウンタをクリア
    clear_counter()
    latch_counter()
    
    before = read_counter_value()
    print(f"クリア後のカウンタ値: {before}")
    print()
    
    # 各候補ピンでテスト
    for test_pin in POSSIBLE_CCLK_PINS:
        try:
            GPIO.setup(test_pin, GPIO.OUT)
            
            # カウンタをクリア
            clear_counter()
            latch_counter()
            
            # 3パルス送る
            for i in range(3):
                GPIO.output(test_pin, GPIO.LOW)
                time.sleep(0.001)
                GPIO.output(test_pin, GPIO.HIGH)
                time.sleep(0.001)
            
            # ラッチして読む
            latch_counter()
            time.sleep(0.05)
            counter_val = read_counter_value()
            
            print(f"BCM {test_pin:2d}: 3パルス送信後 → カウンタ値 = {counter_val}")
            
            if counter_val == 3:
                print(f"  ★ BCM {test_pin} が CCLK (pin 11) です！")
                print(f"     RPR220 はこのピンに接続されているはず")
                return test_pin
            
            # ピンを入力に戻す
            GPIO.setup(test_pin, GPIO.IN)
            
        except Exception as e:
            print(f"BCM {test_pin:2d}: エラー - {e}")
    
    print()
    print("⚠ CCLK ピンが見つかりませんでした")
    return None

def monitor_rpr220_pulses(cclk_pin=None):
    """RPR220 のパルスをモニタ"""
    print()
    print("=" * 60)
    print("RPR220 パルスモニタ")
    print("=" * 60)
    print()
    
    if cclk_pin is None:
        print("CCLK ピンが不明です。")
        print("以下のピンをチェックして RPR220 出力を探します:")
        print(f"  {POSSIBLE_CCLK_PINS}")
        print()
        
        for pin in POSSIBLE_CCLK_PINS:
            try:
                GPIO.setup(pin, GPIO.IN)
                samples = []
                print(f"BCM {pin:2d} をモニタ中 (5秒間)...", end=" ")
                
                start_time = time.time()
                last_val = GPIO.input(pin)
                pulse_count = 0
                
                while time.time() - start_time < 5:
                    val = GPIO.input(pin)
                    if val != last_val:
                        pulse_count += 1
                        last_val = val
                    time.sleep(0.001)
                
                print(f"変化: {pulse_count}回")
                
                if pulse_count > 10:
                    print(f"  ★ BCM {pin} でパルスを検出！")
                    cclk_pin = pin
                    break
                
            except Exception as e:
                print(f"エラー - {e}")
    
    if cclk_pin:
        print()
        print(f"BCM {cclk_pin} (CCLK) のパルスをカウント")
        print("車輪を回してください...")
        print("Ctrl+C で終了")
        print()
        
        GPIO.setup(cclk_pin, GPIO.IN)
        
        # カウンタをクリア
        clear_counter()
        
        last_val = GPIO.input(cclk_pin)
        pulse_count = 0
        
        try:
            while True:
                val = GPIO.input(cclk_pin)
                if val != last_val and val == 1:  # 立ち上がりエッジ
                    pulse_count += 1
                    
                    # ラッチして現在のカウンタ値を読む
                    latch_counter()
                    counter_val = read_counter_value()
                    
                    print(f"パルス #{pulse_count}: カウンタ値 = {counter_val}")
                
                last_val = val
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n終了")

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  RPR220 フォトリフレクタ パルス検出  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    print("【目的】")
    print("  1. 74HC590 pin 11 (CCLK) がどの GPIO に接続されているか探す")
    print("  2. RPR220 からパルスが来ているか確認")
    print("  3. カウンタが正常にカウントアップするか確認")
    print()
    
    try:
        # CCLK ピンを探す
        cclk_pin = test_manual_pulse()
        
        # RPR220 パルスをモニタ
        monitor_rpr220_pulses(cclk_pin)
        
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
