#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出 動作パターン特定テスト

前回の結果:
  - Q出力変化: 2回 (1→0, 0→1)
  - 白線検出は動作している
  
今回の目的:
  - トグル動作 or セット/クリア動作を特定
  - より多くのサンプルを取得
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
    GPIO.output(CONTROL_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(CONTROL_PIN, GPIO.HIGH)
    time.sleep(0.01)

def read_q():
    return GPIO.input(LINE_PIN)

def main():
    print("=" * 70)
    print("74HC74 動作パターン特定テスト")
    print("=" * 70)
    print()
    
    try:
        setup_gpio()
        
        # === ステップ1: クリア実行 ===
        print("【ステップ1: 初期化（クリア）】")
        print()
        print("CLRパルスを送信してQ=0にクリア...")
        clear_ff()
        q_initial = read_q()
        print(f"クリア後のQ: {q_initial}")
        
        if q_initial == 1:
            print("⚠ クリアしてもQ=1のまま")
            print("  → D入力がHIGHでCLKパルス待ち状態の可能性")
        else:
            print("✓ Q=0にクリアされました")
        print()
        
        input("Enter を押してモニタリング開始...")
        print()
        
        # === ステップ2: 連続モニタリング ===
        print("-" * 70)
        print("【ステップ2: 白線検出モニタリング】")
        print("-" * 70)
        print()
        print("白い紙を何回か近づけて、パターンを観察")
        print("目標: 5回以上の変化を観察")
        print("Ctrl+C で終了")
        print()
        
        prev_q = q_initial
        changes = []  # (時刻, 前の値, 新しい値)
        change_count = 0
        
        print(f"開始時のQ: {prev_q}")
        print()
        
        while True:
            q = read_q()
            
            if q != prev_q:
                ts = time.strftime('%H:%M:%S')
                change_count += 1
                changes.append((ts, prev_q, q))
                
                print(f"[{ts}] 変化 #{change_count}: Q={prev_q}→{q}")
                
                prev_q = q
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print()
        print()
        print("=" * 70)
        print("動作パターン分析")
        print("=" * 70)
        print()
        
        print(f"検出された変化: {change_count}回")
        print()
        
        if change_count == 0:
            print("✗ 変化が検出されませんでした")
            print("  → RPR220が動作していない可能性")
        
        elif change_count >= 1:
            print("変化の履歴:")
            for i, (ts, old, new) in enumerate(changes, 1):
                print(f"  {i}. [{ts}] {old}→{new}")
            print()
            
            # パターン分析
            transitions_0_to_1 = sum(1 for _, old, new in changes if old == 0 and new == 1)
            transitions_1_to_0 = sum(1 for _, old, new in changes if old == 1 and new == 0)
            
            print("遷移の統計:")
            print(f"  0→1: {transitions_0_to_1}回")
            print(f"  1→0: {transitions_1_to_0}回")
            print()
            
            # 最終状態
            final_q = read_q()
            print(f"最終的なQ出力: {final_q}")
            print()
            
            # パターン判定
            if abs(transitions_0_to_1 - transitions_1_to_0) <= 1:
                print("【判定: トグル動作】")
                print()
                print("  動作: D入力 = Q' (反転出力)")
                print("  特徴: RPR220パルスごとにQが反転")
                print()
                print("  プログラムでの使用方法:")
                print("    - Q=1やQ=0は白線検出を意味しない")
                print("    - 変化回数をカウント = 白線通過回数")
                print()
                print("  コード例:")
                print("    change_count = 0")
                print("    prev_q = read_q()")
                print("    while True:")
                print("        q = read_q()")
                print("        if q != prev_q:")
                print("            change_count += 1  # 白線通過")
                print("            prev_q = q")
                print()
            
            elif transitions_0_to_1 > transitions_1_to_0 * 2:
                print("【判定: セット動作】")
                print()
                print("  動作: D入力 = HIGH (VCCに接続)")
                print("  特徴: 白線検出でQ=1にセット、クリアしないと戻らない")
                print()
                print("  プログラムでの使用方法:")
                print("    WHITE_LINE_LEVEL = 1")
                print("    if read_q() == 1:")
                print("        # 白線検出")
                print("        clear_ff()  # 検出後クリア")
                print()
            
            elif transitions_1_to_0 > transitions_0_to_1 * 2:
                print("【判定: クリア動作】")
                print()
                print("  動作: D入力 = LOW (GNDに接続)")
                print("  特徴: 白線検出でQ=0にクリア")
                print()
                print("  プログラムでの使用方法:")
                print("    WHITE_LINE_LEVEL = 0")
                print("    if read_q() == 0:")
                print("        # 白線検出")
                print()
            
            else:
                print("【判定: 不明確】")
                print("  → より多くのサンプルが必要")
                print("  → もう一度テストを実行してください")
                print()
        
        print()
        print("【次のステップ】")
        print()
        
        if change_count >= 3:
            print("✓ パターンが特定できました")
            print("  → white_line_poll.py を上記の使用方法に修正")
            print("  → または統合用のクラス/関数を作成")
        else:
            print("⚠ もう一度テストを実行して、より多くの変化を観察してください")
            print("  → 白い紙を5回以上近づける")
        print()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
