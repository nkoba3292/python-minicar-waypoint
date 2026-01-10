#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出回路テスト (正確な構成版)

実際の接続構成:
  RPR220 Dout (プルアップ) → 74HC74 pin 3 (CLK1)
  74HC74 pin 5 (Q1) → ラズパイ GPIO23
  ラズパイ GPIO18 → 74HC74 CLR または SET

74HC74の動作:
  - D入力が何か固定値（HIGH or LOW）に設定されている
  - RPR220のパルス（CLK）で、その固定値がQにラッチされる
  - 白線検出 → パルス発生 → Qが変化
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
LINE_PIN = 23       # BCM23 ← 74HC74 Q (pin 5)
CONTROL_PIN = 18    # BCM18 → 74HC74 CLR/SET制御

def setup_gpio():
    """GPIO初期化
    
    GPIO18 → 74HC74 pin 1 (CLR1)
    - CLRはアクティブLOW (LOW でクリア)
    - 通常動作時は HIGH を維持
    - 外部プルアップ/ダウンなし → 初期値を明示的に HIGH に設定
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # CLR制御ピン - 初期値を HIGH に設定（クリア無効）
    # 外部プルアップなしなので、初期化前にフロート状態を避けるため
    # 最初から HIGH を出力
    GPIO.setup(CONTROL_PIN, GPIO.OUT, initial=GPIO.HIGH)
    
    # 入力ピン (Q出力) - 外部プルアップあるためPUD_OFF
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def control_output(level):
    """制御ピン操作"""
    GPIO.output(CONTROL_PIN, level)

def read_q_output():
    """Q出力を読み取り"""
    return GPIO.input(LINE_PIN)

def main():
    print("=" * 70)
    print("74HC74 白線検出回路テスト（正確な構成版）")
    print("=" * 70)
    print()
    
    print("【実際の接続】")
    print()
    print("  RPR220 Dout (プルアップ) → 74HC74 pin 3 (CLK1)")
    print("  74HC74 pin 5 (Q1) → ラズパイ GPIO23")
    print("  ラズパイ GPIO18 → 74HC74 pin 1 (CLR1)")
    print()
    print("注: GPIO18は外部プルアップ/ダウンなし")
    print("    初期化時に明示的にHIGHを出力（クリア無効状態）")
    print()
    
    print("【74HC74 ピン配置】")
    print()
    print("       ┌─────┴─────┐")
    print("  CLR1 1│○          │14 VCC")
    print("    D1 2│           │13 CLR2")
    print("  CLK1 3│ ← RPR220  │12 D2")
    print("  SET1 4│           │11 CLK2")
    print("    Q1 5│ → GPIO23  │10 SET2")
    print("   Q1' 6│           │9  Q2")
    print("   GND 7│           │8  Q2'")
    print("       └───────────┘")
    print()
    
    print("【動作原理】")
    print()
    print("D入力の設定により動作が異なります:")
    print()
    print("パターンA: D1 (pin 2) = HIGH (VCCに接続)")
    print("  - RPR220パルス → CLK立ち上がり → Q=HIGH にラッチ")
    print("  - 白線検出 → Q=HIGH")
    print()
    print("パターンB: D1 (pin 2) = LOW (GNDに接続)")
    print("  - RPR220パルス → CLK立ち上がり → Q=LOW にラッチ")
    print("  - 白線検出 → Q=LOW")
    print()
    print("パターンC: D1 (pin 2) = Q1' (pin 6) に接続 (トグル動作)")
    print("  - RPR220パルスごとにQが反転")
    print("  - 白線検出 → Qがトグル (0→1→0→1...)")
    print()
    
    try:
        setup_gpio()
        
        # 制御ピンをHIGHに設定（通常動作）
        control_output(GPIO.HIGH)
        time.sleep(0.1)
        
        print("-" * 70)
        print("【テスト: 連続モニタリング】")
        print("-" * 70)
        print()
        print("白い紙を RPR220 の前で動かして反応を確認")
        print("Ctrl+C で終了")
        print()
        
        prev_q = None
        change_count = 0
        q_history = []
        
        print("開始...")
        print()
        
        while True:
            q = read_q_output()
            
            if q != prev_q:
                ts = time.strftime('%H:%M:%S.') + f"{int(time.time() * 1000) % 1000:03d}"
                change_count += 1
                
                if q == 1:
                    print(f"[{ts}] Q: 0 → 1  (変化 #{change_count})")
                else:
                    print(f"[{ts}] Q: 1 → 0  (変化 #{change_count})")
                
                q_history.append(q)
                # 最新5個を保持
                if len(q_history) > 5:
                    q_history.pop(0)
                
                prev_q = q
            
            time.sleep(0.01)  # 10ms間隔で高速チェック
            
    except KeyboardInterrupt:
        print()
        print()
        print("=" * 70)
        print("テスト結果")
        print("=" * 70)
        print()
        print(f"Q出力の変化回数: {change_count}回")
        print()
        
        if change_count == 0:
            print("✗ Q出力が変化しませんでした")
            print()
            print("原因の可能性:")
            print("  1. RPR220が動作していない")
            print("     → RPR220の電源(5V)を確認")
            print("     → しきい値調整VRがあれば調整")
            print("  2. RPR220 Dout → 74HC74 CLK (pin 3) の配線が接続されていない")
            print("  3. D1 (pin 2) がフロート状態")
            print("     → D1をVCCまたはGNDに接続")
            print("  4. CLR1またはSET1が異常状態")
            print("     → CLR1 (pin 1) = HIGH")
            print("     → SET1 (pin 4) = HIGH")
            print()
        else:
            print("✓ Q出力が変化しました")
            print()
            print("変化パターンの分析:")
            print(f"  最近のQ履歴: {q_history}")
            print()
            
            # トグル動作かどうか判定
            if len(q_history) >= 4:
                is_toggle = True
                for i in range(len(q_history) - 1):
                    if q_history[i] == q_history[i + 1]:
                        is_toggle = False
                        break
                
                if is_toggle:
                    print("  動作タイプ: トグル動作")
                    print("  → D1 (pin 2) が Q1' (pin 6) に接続されている")
                    print("  → RPR220パルスごとにQが反転")
                    print()
                    print("  使用方法:")
                    print("    - 変化回数をカウント = 白線検出回数")
                    print("    - Q=1 または Q=0 の状態は白線検出を意味しない")
                    print()
                else:
                    # 最終的なQの状態を確認
                    final_q = read_q_output()
                    if final_q == 1:
                        print("  動作タイプ: セット動作")
                        print("  → D1 (pin 2) = HIGH (VCCに接続)")
                        print("  → 白線検出時: Q=1")
                        print("  → WHITE_LINE_LEVEL = 1")
                    else:
                        print("  動作タイプ: クリア動作")
                        print("  → D1 (pin 2) = LOW (GNDに接続)")
                        print("  → 白線検出時: Q=0")
                        print("  → WHITE_LINE_LEVEL = 0")
                    print()
        
        print()
        print("【GPIO18の役割確認テスト】")
        print()
        print("GPIO18は74HC74 pin 1 (CLR1) に接続")
        print("CLR1はアクティブLOW: LOW でクリア、HIGH で通常動作")
        print()
        print("GPIO18の現在の状態: HIGH")
        print("GPIO18をLOWにします (クリア実行)...")
        control_output(GPIO.LOW)
        time.sleep(0.5)
        q_low = read_q_output()
        print(f"  GPIO18=LOW 時の Q: {q_low}")
        
        print("GPIO18をHIGHに戻します (通常動作)...")
        control_output(GPIO.HIGH)
        time.sleep(0.5)
        q_high = read_q_output()
        print(f"  GPIO18=HIGH 時の Q: {q_high}")
        print()
        
        if q_low == 0 and q_low != q_high:
            print("✓ GPIO18 = CLR (pin 1) 動作確認")
            print("  → GPIO18=LOW でQ=0にクリア")
            print("  → GPIO18=HIGH で通常動作")
        elif q_low != q_high:
            print("✓ GPIO18でQ出力を制御できます")
            print(f"  → GPIO18=LOW でQ={q_low}")
            print(f"  → GPIO18=HIGH でQ={q_high}")
        else:
            print("? GPIO18でQ出力が変化しませんでした")
            print("  → CLRが効いていない、または既にクリア状態")
        print()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
