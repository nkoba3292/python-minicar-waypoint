#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出回路テスト (CLR入力方式)

実際の接続構成:
  RPR220 Dout → 74HC74 CLR入力 (C端子)
  74HC74 Q出力 → ラズパイ GPIO23
  ラズパイ GPIO18 → 74HC74 /R (リセット制御)

動作原理:
  - RPR220が白線検出 → Dout変化 → CLRに入力
  - CLRの状態でQがクリア/セットされる
  - ラズパイはQを読み取るのみ
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
LINE_PIN = 23       # BCM23 ← 74HC74 Q出力
RESET_PIN = 18      # BCM18 → 74HC74 /R

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 出力ピン
    GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
    
    # 入力ピン
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def reset_control(level):
    """リセット制御"""
    GPIO.output(RESET_PIN, level)

def read_line_status():
    """Q出力を読み取り (デバウンス付き)"""
    samples = []
    for _ in range(3):
        samples.append(GPIO.input(LINE_PIN))
        time.sleep(0.001)
    return 1 if sum(samples) >= 2 else 0

def main():
    print("=" * 70)
    print("74HC74 白線検出回路テスト")
    print("=" * 70)
    print()
    
    print("【実際の接続】")
    print()
    print("  RPR220 Dout → 74HC74 C (CLR端子)")
    print("  74HC74 Q → ラズパイ GPIO23")
    print("  ラズパイ GPIO18 → 74HC74 /R (リセット)")
    print()
    print("【74HC74 端子】")
    print("  C (CLR) : センサ入力")
    print("  /R      : リセット制御")
    print("  Q       : 出力")
    print()
    
    print("この構成では:")
    print("  - センサがCLRを制御")
    print("  - ラズパイは/Rとして別のリセット制御")
    print("  - Qを読み取るのみ")
    print()
    
    try:
        setup_gpio()
        
        print("-" * 70)
        print("【テスト: 連続モニタリング】")
        print("-" * 70)
        print()
        print("0.2秒ごとにQ出力を監視します")
        print("白い紙を近づけて反応を確認")
        print("Ctrl+C で終了")
        print()
        
        # リセット解除
        reset_control(GPIO.HIGH)
        time.sleep(0.1)
        
        prev_status = None
        detection_count = 0
        change_count = 0
        
        while True:
            status = read_line_status()
            
            if status != prev_status:
                ts = time.strftime('%H:%M:%S')
                change_count += 1
                
                if status == 1:
                    print(f"[{ts}] Q=1 (HIGH)")
                else:
                    print(f"[{ts}] Q=0 (LOW)")
                
                # 白線検出判定は極性による
                # ここでは変化回数をカウント
                detection_count += 1
                prev_status = status
            
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print()
        print()
        print("=" * 70)
        print("テスト終了")
        print("=" * 70)
        print()
        print(f"状態変化回数: {change_count}回")
        print()
        print("【回路動作の確認方法】")
        print()
        print("白い紙を近づけたときに:")
        print("  - Q出力が変化した → 回路は動作している")
        print("  - Q出力が変化しない → センサまたは配線の問題")
        print()
        print("【次のステップ】")
        print("1. センサとQ出力の関係を確認")
        print("   白い紙でQ=1 or Q=0?")
        print()
        print("2. 極性を特定して定数を設定")
        print("   WHITE_LINE_LEVEL = 1 or 0")
        print()
        print("3. リセット機能の確認")
        print("   GPIO18で/Rを制御してQ出力がリセットされるか")
        print()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
