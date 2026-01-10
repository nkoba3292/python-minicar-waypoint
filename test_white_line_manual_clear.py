#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 手動クリアテスト

セット動作と推定:
- 白線検出でQ=1
- 一度Q=1になるとリセットしないと戻らない
"""

import RPi.GPIO as GPIO
import time

LINE_PIN = 23       # 74HC74 Q
CONTROL_PIN = 18    # 74HC74 CLR (pin 1)

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(CONTROL_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def clear_ff():
    """CLRパルスでFFをクリア"""
    print("  CLRパルス送信中...")
    GPIO.output(CONTROL_PIN, GPIO.LOW)
    time.sleep(0.1)  # 100ms LOW
    GPIO.output(CONTROL_PIN, GPIO.HIGH)
    time.sleep(0.1)

def read_q():
    return GPIO.input(LINE_PIN)

def main():
    print("=" * 70)
    print("74HC74 手動クリアテスト")
    print("=" * 70)
    print()
    
    try:
        setup_gpio()
        
        print("【現在の状態確認】")
        q = read_q()
        print(f"現在のQ: {q}")
        print()
        
        if q == 1:
            print("Q=1 です。CLRでクリアしてみます。")
            print()
            input("Enter でクリア実行...")
            clear_ff()
            q_after = read_q()
            print(f"クリア後のQ: {q_after}")
            print()
            
            if q_after == 0:
                print("✓ クリア成功! Q=1→0")
                print()
                print("【結論: セット動作確認】")
                print("  - D入力 = HIGH (VCC接続)")
                print("  - 白線検出 → Q=1にセット")
                print("  - CLRでQ=0にリセット必要")
                print()
            else:
                print("✗ クリアできませんでした")
                print("  CLRパルスの極性または配線を確認")
                print()
        else:
            print("Q=0 です。白線を検出させてQ=1にしてみます。")
            print()
            input("白い紙を近づけて Enter...")
            
            for i in range(50):  # 5秒間モニタ
                q = read_q()
                if q == 1:
                    print(f"✓ Q=1に変化しました!")
                    break
                time.sleep(0.1)
            else:
                print("✗ Q=1になりませんでした")
                print("  RPR220の動作を確認")
            print()
        
        print()
        print("【連続テスト】")
        print("白線検出 → クリア を3回繰り返します")
        print()
        
        for cycle in range(3):
            print(f"--- サイクル {cycle + 1} ---")
            
            # 初期状態
            q_start = read_q()
            print(f"開始時Q: {q_start}")
            
            if q_start == 1:
                print("  既にQ=1、クリアします")
                clear_ff()
                q_start = read_q()
                print(f"  クリア後Q: {q_start}")
            
            # 白線検出待ち
            print("  白い紙を近づけてください...")
            detected = False
            for i in range(100):  # 10秒待機
                q = read_q()
                if q == 1 and q != q_start:
                    print(f"  ✓ 検出! Q={q_start}→1")
                    detected = True
                    break
                time.sleep(0.1)
            
            if not detected:
                print("  ✗ 検出されませんでした")
            
            time.sleep(1)
            
            # クリア
            print("  クリア実行...")
            clear_ff()
            q_after_clear = read_q()
            print(f"  クリア後Q: {q_after_clear}")
            
            if q_after_clear == 0:
                print("  ✓ クリア成功")
            else:
                print("  ✗ クリア失敗")
            
            print()
            time.sleep(1)
        
        print("=" * 70)
        print("テスト完了")
        print("=" * 70)
        print()
        
    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
