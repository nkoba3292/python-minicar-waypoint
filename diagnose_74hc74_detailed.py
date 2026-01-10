#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出回路 詳細診断

Q出力が常に0の原因を特定するための診断ツール

確認項目:
1. 74HC74のピン電圧
2. RPR220センサの動作
3. CLK/CLRパルスの動作
4. D入力の状態
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
LATCH_PIN = 24      # BCM24 → 74HC74 CLK
LINE_PIN = 23       # BCM23 ← 74HC74 Q
LINE_RESET_PIN = 18 # BCM18 → 74HC74 CLR

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(LATCH_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LINE_RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def main():
    print("=" * 70)
    print("74HC74 白線検出回路 詳細診断")
    print("=" * 70)
    print()
    
    print("【74HC74 ピン配置 (14ピンDIP)】")
    print()
    print("       ┌─────┴─────┐")
    print("  CLR1 1│○          │14 VCC")
    print("    D1 2│           │13 CLR2")
    print("  CLK1 3│           │12 D2")
    print("  SET1 4│  74HC74   │11 CLK2")
    print("    Q1 5│           │10 SET2")
    print("   Q1' 6│           │9  Q2")
    print("   GND 7│           │8  Q2'")
    print("       └───────────┘")
    print()
    
    print("【確認する接続】")
    print()
    print("  白線検出用 (FF1側を使用と仮定):")
    print("    pin 1 (CLR1) ← ラズパイ GPIO18 (CLR制御)")
    print("    pin 2 (D1)   ← RPR220 Dout (白線検出信号)")
    print("    pin 3 (CLK1) ← ラズパイ GPIO24 (ラッチストローブ)")
    print("    pin 4 (SET1) → VCC (通常HIGH)")
    print("    pin 5 (Q1)   → ラズパイ GPIO23 (白線状態読み取り)")
    print("    pin 7 (GND)  → GND")
    print("    pin 14 (VCC) → 3.3V")
    print()
    
    try:
        setup_gpio()
        
        print("-" * 70)
        print("【診断1: GPIO制御ピンの動作確認】")
        print("-" * 70)
        print()
        
        print("CLR (GPIO18) をHIGH → LOW → HIGH と切り替えます")
        print("マルチメータでpin 1の電圧変化を確認してください")
        print()
        
        for i in range(3):
            print(f"  {i+1}. CLR = HIGH (3.3V)")
            GPIO.output(LINE_RESET_PIN, GPIO.HIGH)
            time.sleep(2)
            
            print(f"     CLR = LOW (0V)")
            GPIO.output(LINE_RESET_PIN, GPIO.LOW)
            time.sleep(2)
        
        GPIO.output(LINE_RESET_PIN, GPIO.HIGH)
        print("     CLR = HIGH (3.3V)")
        print()
        
        input("確認できたら Enter...")
        print()
        
        print("CLK (GPIO24) をLOW → HIGH → LOW と切り替えます")
        print("マルチメータでpin 3の電圧変化を確認してください")
        print()
        
        for i in range(3):
            print(f"  {i+1}. CLK = LOW (0V)")
            GPIO.output(LATCH_PIN, GPIO.LOW)
            time.sleep(2)
            
            print(f"     CLK = HIGH (3.3V)")
            GPIO.output(LATCH_PIN, GPIO.HIGH)
            time.sleep(2)
        
        GPIO.output(LATCH_PIN, GPIO.LOW)
        print("     CLK = LOW (0V)")
        print()
        
        input("確認できたら Enter...")
        print()
        
        print("-" * 70)
        print("【診断2: 74HC74電源とSET確認】")
        print("-" * 70)
        print()
        print("マルチメータで以下を測定してください:")
        print()
        print("  pin 14 (VCC): _____ V  (期待: 3.3V)")
        print("  pin 7 (GND):  _____ V  (期待: 0V)")
        print("  pin 4 (SET1): _____ V  (期待: 3.3V、VCCに接続)")
        print()
        print("もしSET1が0VまたはフロートならQ出力は常に0になります")
        print()
        
        input("確認できたら Enter...")
        print()
        
        print("-" * 70)
        print("【診断3: D入力 (RPR220出力) の確認】")
        print("-" * 70)
        print()
        print("RPR220の出力 (74HC74 pin 2, D1入力) をマルチメータで測定:")
        print()
        print("1. 白い紙がない状態:")
        print("   D1 (pin 2): _____ V")
        print()
        
        input("測定したら Enter...")
        
        print()
        print("2. 白い紙を近づけた状態:")
        print("   D1 (pin 2): _____ V")
        print()
        
        input("測定したら Enter...")
        print()
        
        print("期待される動作:")
        print("  - 白い紙なし: D1 = 0V (または3.3V、センサ極性による)")
        print("  - 白い紙あり: D1 = 3.3V (または0V、センサ極性による)")
        print()
        print("もしD1が常に0Vまたは常に3.3Vなら:")
        print("  → RPR220が動作していない")
        print("  → RPR220の電源を確認 (通常5V)")
        print("  → RPR220の配線を確認")
        print()
        
        input("Enter で次へ...")
        print()
        
        print("-" * 70)
        print("【診断4: Q出力の測定】")
        print("-" * 70)
        print()
        print("74HC74 pin 5 (Q1) をマルチメータで測定:")
        print()
        
        # CLRでリセット
        print("1. CLR = LOW (リセット)")
        GPIO.output(LINE_RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(LINE_RESET_PIN, GPIO.HIGH)
        time.sleep(0.1)
        
        q_gpio = GPIO.input(LINE_PIN)
        print(f"   ラズパイ読み取り: Q = {q_gpio}")
        print("   マルチメータ測定: Q (pin 5) = _____ V")
        print()
        
        input("測定したら Enter...")
        print()
        
        print("2. 白い紙を近づけて、CLKパルス送信")
        input("   白い紙を準備したら Enter...")
        
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.001)
        GPIO.output(LATCH_PIN, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.1)
        
        q_gpio = GPIO.input(LINE_PIN)
        print(f"   ラズパイ読み取り: Q = {q_gpio}")
        print("   マルチメータ測定: Q (pin 5) = _____ V")
        print()
        
        input("測定したら Enter...")
        print()
        
        print("-" * 70)
        print("【診断5: 手動でD入力をテスト】")
        print("-" * 70)
        print()
        print("RPR220を一時的に外して、D1 (pin 2) を直接制御します:")
        print()
        print("テスト手順:")
        print("  1. RPR220のDout配線を74HC74から外す")
        print("  2. D1 (pin 2) をジャンパ線で3.3Vに接続")
        print("  3. CLKパルスを送信")
        print("  4. Q出力を確認")
        print()
        print("もしこれでQ=1になれば:")
        print("  → 74HC74は正常、RPR220に問題")
        print()
        print("それでもQ=0なら:")
        print("  → 74HC74が故障、またはSET/CLR配線ミス")
        print()
        
        input("テストする場合は準備して Enter (スキップも可)...")
        
        print()
        print("D1を3.3Vに接続しましたか? (y/n): ", end="")
        response = input().strip().lower()
        
        if response == 'y':
            print()
            print("CLRでリセット...")
            GPIO.output(LINE_RESET_PIN, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(LINE_RESET_PIN, GPIO.HIGH)
            time.sleep(0.1)
            
            print("CLKパルス送信...")
            GPIO.output(LATCH_PIN, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(LATCH_PIN, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(LATCH_PIN, GPIO.LOW)
            time.sleep(0.1)
            
            q_gpio = GPIO.input(LINE_PIN)
            print(f"Q出力: {q_gpio}")
            print()
            
            if q_gpio == 1:
                print("✓ 74HC74は正常動作")
                print("  → RPR220またはRPR220との接続に問題")
            else:
                print("✗ Q=0のまま")
                print("  → 74HC74故障、またはSET/CLR配線ミス")
        
        print()
        print("=" * 70)
        print("診断完了")
        print("=" * 70)
        print()
        
        print("【チェックリスト】")
        print()
        print("  □ VCC (pin 14) = 3.3V")
        print("  □ GND (pin 7) = 0V")
        print("  □ SET1 (pin 4) = 3.3V (VCCに接続)")
        print("  □ CLR1 (pin 1) = GPIO18で制御可能")
        print("  □ CLK1 (pin 3) = GPIO24で制御可能")
        print("  □ D1 (pin 2) = RPR220出力、白い紙で電圧変化")
        print("  □ Q1 (pin 5) = GPIO23で読み取り可能")
        print()
        print("【よくある問題】")
        print()
        print("1. SET1がGNDに接続 → Q出力は常に0")
        print("   対策: SET1をVCC (3.3V) に接続")
        print()
        print("2. CLR1がLOWまたはフロート → Q出力は常に0")
        print("   対策: CLR1を通常HIGH、リセット時のみLOW")
        print()
        print("3. RPR220が動作していない → D入力が変化しない")
        print("   対策: RPR220の電源(5V)とGNDを確認")
        print()
        print("4. 極性が逆 → 白線で0、白線なしで1")
        print("   対策: WHITE_LINE_LEVEL = 0 に変更")
        print()
        
    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
