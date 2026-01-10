#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC590 ラッチストローブ (RCLK) 動作確認テスト

74HC590は2段構成:
  内部カウンタ → RCLK (ラッチストローブ) → 出力レジスタ → QA/QB/QC

このテストでRCLKの動作を確認:
  1. カウンタ値が変化してもRCLKを送るまで出力は変わらない
  2. RCLKを送ると出力レジスタに転送される
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
QA_PIN = 13    # 74HC590 pin 15 (D0)
QB_PIN = 21    # 74HC590 pin 1  (D1)
QC_PIN = 25    # 74HC590 pin 2  (D2)
CCLR_PIN = 17  # 74HC590 pin 10 (カウンタクリア)
RCLK_PIN = 24  # 74HC590 pin 13 (ラッチクロック)

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 出力ピン (QA/QB/QC) を入力として設定
    GPIO.setup(QA_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # 制御ピンを出力として設定
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    
    # 初期状態
    GPIO.output(CCLR_PIN, GPIO.HIGH)  # クリア無効
    GPIO.output(RCLK_PIN, GPIO.LOW)   # RCLKはLOW

def read_counter():
    """カウンタ値を読み取り (3ビット)"""
    d0 = GPIO.input(QA_PIN)
    d1 = GPIO.input(QB_PIN)
    d2 = GPIO.input(QC_PIN)
    value = d0 | (d1 << 1) | (d2 << 2)
    return value, d2, d1, d0

def clear_counter():
    """カウンタをクリア"""
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)

def send_rclk_pulse():
    """RCLKパルスを送信 (内部カウンタ → 出力レジスタに転送)"""
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)  # 1ms HIGH
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)  # 1ms LOW

def simulate_count_pulses(num_pulses):
    """
    RPR220からのパルスをシミュレート
    注意: 実際にはRPR220がCCLK (pin 11) に接続されているため、
    このテストでは手動でRPR220の前で手を振る必要があります。
    """
    print(f"  RPR220の前で手を{num_pulses}回振ってください...")
    print("  (5秒待機)")
    time.sleep(5)

def main():
    print("=" * 60)
    print("74HC590 ラッチストローブ (RCLK) 動作確認テスト")
    print("=" * 60)
    print()
    
    print("【74HC590の2段構成】")
    print()
    print("  RPR220 → CCLK → [内部カウンタ] → RCLK → [出力レジスタ] → QA/QB/QC")
    print()
    print("  - 内部カウンタ: CCLKでカウントアップ")
    print("  - 出力レジスタ: RCLKで内部カウンタの値を転送")
    print("  - QA/QB/QC: 出力レジスタの値を出力")
    print()
    print("【このテストの目的】")
    print("  RCLKを送るまで出力が変わらないことを確認")
    print()
    
    try:
        setup_gpio()
        
        # カウンタをクリア
        print("カウンタをクリアしています...")
        clear_counter()
        value, d2, d1, d0 = read_counter()
        print(f"初期値: {value} (D2={d2} D1={d1} D0={d0})")
        print()
        
        input("Enter を押してテスト開始...")
        print()
        
        # === テスト1: RCLKなしでカウント ===
        print("-" * 60)
        print("【テスト1: RCLKなしでカウント】")
        print("-" * 60)
        print()
        print("RPR220の前で手を数回振ってください")
        print("内部カウンタは増えますが、出力は変わらないはずです")
        print()
        
        value_before, d2, d1, d0 = read_counter()
        print(f"カウント前の出力: {value_before} (D2={d2} D1={d1} D0={d0})")
        print()
        
        print("RPR220の前で手を3回振ってください...")
        print("(10秒待機)")
        time.sleep(10)
        
        value_after, d2, d1, d0 = read_counter()
        print(f"カウント後の出力: {value_after} (D2={d2} D1={d1} D0={d0})")
        print()
        
        if value_after == value_before:
            print("✓ 正常: RCLKなしでは出力が変わりませんでした")
        else:
            print("✗ 異常: RCLKなしで出力が変わりました")
            print("  → 74HC590が異常、またはRCLKピン接続ミスの可能性")
        print()
        
        input("Enter を押して次のテストへ...")
        print()
        
        # === テスト2: RCLKで転送 ===
        print("-" * 60)
        print("【テスト2: RCLKで転送】")
        print("-" * 60)
        print()
        print("RCLKパルスを送信して、内部カウンタの値を出力に転送します")
        print()
        
        value_before, d2, d1, d0 = read_counter()
        print(f"RCLK前の出力: {value_before} (D2={d2} D1={d1} D0={d0})")
        print()
        
        print("RCLKパルスを送信中...")
        send_rclk_pulse()
        time.sleep(0.1)
        
        value_after, d2, d1, d0 = read_counter()
        print(f"RCLK後の出力: {value_after} (D2={d2} D1={d1} D0={d0})")
        print()
        
        if value_after != value_before:
            print(f"✓ 正常: 出力が変化しました ({value_before} → {value_after})")
            print("  → 内部カウンタの値が出力レジスタに転送されました")
        else:
            print("? 内部カウンタが増えていなかった可能性があります")
            print("  → RPR220がカウントしていない、または既に転送済み")
        print()
        
        input("Enter を押して次のテストへ...")
        print()
        
        # === テスト3: 複数回のカウントとラッチ ===
        print("-" * 60)
        print("【テスト3: 複数回のカウントとラッチ】")
        print("-" * 60)
        print()
        print("1. RPR220で2回カウント")
        print("2. RCLK送信")
        print("3. RPR220で2回カウント")
        print("4. RCLK送信")
        print()
        
        clear_counter()
        send_rclk_pulse()  # 0を出力に反映
        time.sleep(0.1)
        
        for cycle in range(3):
            print(f"--- サイクル {cycle + 1} ---")
            
            value_before, d2, d1, d0 = read_counter()
            print(f"出力: {value_before} (D2={d2} D1={d1} D0={d0})")
            print()
            
            print("RPR220の前で手を2回振ってください...")
            print("(5秒待機)")
            time.sleep(5)
            
            value_no_rclk, d2, d1, d0 = read_counter()
            print(f"カウント後 (RCLK前): {value_no_rclk} (D2={d2} D1={d1} D0={d0})")
            
            if value_no_rclk == value_before:
                print("  ✓ 出力は変化なし (内部カウンタのみ増加)")
            else:
                print("  ✗ 出力が変化 (異常)")
            print()
            
            print("RCLKパルス送信...")
            send_rclk_pulse()
            time.sleep(0.1)
            
            value_after, d2, d1, d0 = read_counter()
            print(f"RCLK後: {value_after} (D2={d2} D1={d1} D0={d0})")
            
            if value_after != value_before:
                print(f"  ✓ 出力が更新されました ({value_before} → {value_after})")
            else:
                print("  ? 出力が変わらない (内部カウンタが増えていない可能性)")
            print()
            
            time.sleep(1)
        
        print("=" * 60)
        print("テスト完了")
        print("=" * 60)
        print()
        print("【結果の解釈】")
        print()
        print("正常な動作:")
        print("  - RPR220でカウントしても、RCLKなしでは出力が変わらない")
        print("  - RCLKを送ると出力が更新される")
        print()
        print("異常な動作:")
        print("  - RCLKなしで出力が変わる")
        print("    → RCLK (GPIO24) が常にHIGHまたは不定状態")
        print("    → 74HC590 pin 13 の接続を確認")
        print()
        
    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
