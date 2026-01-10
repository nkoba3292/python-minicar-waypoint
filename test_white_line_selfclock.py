#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出回路テスト (CLK接続なし版)

接続構成:
  RPR220 Dout → 74HC74 D入力
  RPR220 Dout → 74HC74 CLK入力 (セルフラッチ)
  74HC74 Q出力 → ラズパイ GPIO23
  ラズパイ GPIO18 → 74HC74 /CLR (リセット)

動作:
  - RPR220がパルスを出すと自動的にDがQにラッチされる（セルフクロック）
  - ラズパイはQを読むだけ
  - リセットが必要な場合はGPIO18で/CLRを制御
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
LINE_PIN = 23       # BCM23 (物理16p) ← 74HC74 Q出力
LINE_RESET_PIN = 18 # BCM18 (物理12p) → 74HC74 /CLR

# リセット極性
RESET_ACTIVE_LEVEL = GPIO.LOW    # /CLR active LOW
RESET_INACTIVE_LEVEL = GPIO.HIGH # /CLR inactive HIGH

# Q出力の極性 (白線検出時の状態)
WHITE_LINE_LEVEL = 1    # Qが1のとき白線検出 (極性により変更)
# WHITE_LINE_LEVEL = 0  # Qが0のとき白線検出

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 出力ピン (リセットのみ)
    GPIO.setup(LINE_RESET_PIN, GPIO.OUT, initial=RESET_INACTIVE_LEVEL)
    
    # 入力ピン (Q出力)
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def reset_latch():
    """リセットパルスを送信してQをクリア"""
    GPIO.output(LINE_RESET_PIN, RESET_ACTIVE_LEVEL)
    time.sleep(0.0001)  # 100μs
    GPIO.output(LINE_RESET_PIN, RESET_INACTIVE_LEVEL)
    time.sleep(0.0001)  # 安定待ち

def read_line_status():
    """Q出力を読み取り (デバウンス付き)"""
    samples = []
    for _ in range(3):
        samples.append(GPIO.input(LINE_PIN))
        time.sleep(0.001)
    # 多数決
    return 1 if sum(samples) >= 2 else 0

def main():
    print("=" * 70)
    print("74HC74 白線検出回路テスト (セルフクロック方式)")
    print("=" * 70)
    print()
    
    print("【回路構成】")
    print()
    print("  RPR220 Dout ─┬─ 74HC74 D入力")
    print("               └─ 74HC74 CLK入力 (セルフラッチ)")
    print()
    print("  74HC74 Q出力 → ラズパイ GPIO23 (物理16p)")
    print("  ラズパイ GPIO18 (物理12p) → 74HC74 /CLR (リセット)")
    print()
    print("【動作原理】")
    print("  - RPR220が白線を検出 → Dout=HIGH")
    print("  - Dout=HIGHがDとCLKに入力される")
    print("  - CLKの立ち上がりでD(HIGH)がQにラッチ")
    print("  - Q=HIGH → ラズパイで白線検出を読み取り")
    print("  - リセット時: GPIO18でCLR=LOWにしてQ=LOWに戻す")
    print()
    
    try:
        setup_gpio()
        
        # === テスト1: 初期状態確認 ===
        print("-" * 70)
        print("【テスト1: 初期状態確認】")
        print("-" * 70)
        print()
        
        status = read_line_status()
        print(f"現在のQ出力: {status}")
        print()
        
        if status == WHITE_LINE_LEVEL:
            print("⚠ すでに白線が検出されている状態")
            print("  → リセットしてクリアします")
            reset_latch()
            time.sleep(0.1)
            status = read_line_status()
            print(f"リセット後のQ出力: {status}")
        else:
            print("✓ 初期状態は白線未検出")
        print()
        
        input("Enter を押して次のテストへ...")
        print()
        
        # === テスト2: 白線検出テスト ===
        print("-" * 70)
        print("【テスト2: 白線検出テスト】")
        print("-" * 70)
        print()
        print("RPR220センサの前に白い紙を持っていきます")
        print("RPR220が反応すると自動的に74HC74がラッチします")
        print()
        input("白い紙を準備したら Enter...")
        print()
        
        print("現在のQ出力 (白い紙を近づける前): ", end="")
        status_before = read_line_status()
        print(status_before)
        print()
        
        print("白い紙をRPR220の前にゆっくり近づけてください...")
        print("(10秒間モニタリング)")
        print()
        
        for i in range(20):
            status = read_line_status()
            if status != status_before:
                print(f"★ 変化検出! Q = {status_before} → {status}")
                if status == WHITE_LINE_LEVEL:
                    print("✓ 白線検出成功!")
                status_before = status
            time.sleep(0.5)
        
        print()
        final_status = read_line_status()
        print(f"最終的なQ出力: {final_status}")
        print()
        
        if final_status == WHITE_LINE_LEVEL:
            print("✓ 白線検出状態がラッチされています")
        else:
            print("✗ 白線が検出されませんでした")
            print()
            print("考えられる原因:")
            print("  1. RPR220が白線を検出していない")
            print("     → RPR220の電源(5V)を確認")
            print("     → RPR220のしきい値調整(VRがあれば)")
            print("  2. RPR220 Doutが74HC74に接続されていない")
            print("  3. 74HC74の配線ミス")
            print("     → D入力とCLK入力が両方RPR220 Doutに接続されているか")
            print("  4. WHITE_LINE_LEVELの極性が逆")
            print("     → スクリプト内のWHITE_LINE_LEVEL = 0 に変更")
        print()
        
        input("Enter を押してリセットテストへ...")
        print()
        
        # === テスト3: リセット動作確認 ===
        print("-" * 70)
        print("【テスト3: リセット動作確認】")
        print("-" * 70)
        print()
        
        status_before_reset = read_line_status()
        print(f"リセット前のQ出力: {status_before_reset}")
        print()
        
        print("リセットパルス送信...")
        reset_latch()
        time.sleep(0.1)
        
        status_after_reset = read_line_status()
        print(f"リセット後のQ出力: {status_after_reset}")
        print()
        
        if status_after_reset == 0:
            print("✓ リセット成功 (Q=0)")
        elif status_after_reset != status_before_reset:
            print("✓ リセットで状態が変化しました")
        else:
            print("? リセットしても状態が変わらない")
            print("  → /CLR (GPIO18) の配線を確認")
        print()
        
        input("Enter を押して連続モニタへ...")
        print()
        
        # === テスト4: 連続モニタリング ===
        print("-" * 70)
        print("【テスト4: 連続モニタリング】")
        print("-" * 70)
        print()
        print("0.5秒ごとに状態を監視します")
        print("白い紙を近づけたり遠ざけたりして動作確認")
        print("検出後、自動的にリセットします")
        print("Ctrl+C で終了")
        print()
        
        # 初期リセット
        reset_latch()
        time.sleep(0.1)
        
        prev_status = None
        detection_count = 0
        
        while True:
            status = read_line_status()
            
            # 状態表示
            if status != prev_status:
                ts = time.strftime('%H:%M:%S')
                if status == WHITE_LINE_LEVEL:
                    print(f"[{ts}] ★ 白線検出! (Q={status})")
                    detection_count += 1
                    # 自動リセット
                    time.sleep(0.5)
                    reset_latch()
                    print(f"[{ts}]   リセット完了")
                else:
                    print(f"[{ts}]   白線なし (Q={status})")
                prev_status = status
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print()
        print()
        print("=" * 70)
        print("テスト終了")
        print("=" * 70)
        print()
        print(f"白線検出回数: {detection_count}回")
        print()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
