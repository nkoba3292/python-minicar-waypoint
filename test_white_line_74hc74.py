#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
74HC74 白線検出回路テスト

74HC74 D-フリップフロップを使用したRPR220白線センサのテスト

回路構成:
  RPR220 Dout → 74HC74 D入力
  ラズパイ GPIO → 74HC74 CLK (ラッチストローブ)
  74HC74 Q出力 → ラズパイ GPIO (白線検出)
  ラズパイ GPIO → 74HC74 CLR (リセット)

動作:
  1. RPR220が白線を検出するとDがLOW (またはHIGH、極性による)
  2. CLKパルスでDの状態をQにラッチ
  3. Qをラズパイで読み取り
  4. CLRでリセット
"""

import RPi.GPIO as GPIO
import time

# ピン定義 (BCM番号)
LATCH_PIN = 24      # BCM24 (物理18p) → 74HC74 CLK
LINE_PIN = 23       # BCM23 (物理16p) ← 74HC74 Q出力
LINE_RESET_PIN = 18 # BCM18 (物理12p) → 74HC74 CLR

# 極性設定 (ハードウェアに合わせて調整)
LATCH_IDLE_LEVEL = GPIO.LOW      # CLKアイドル状態
LATCH_ACTIVE_LEVEL = GPIO.HIGH   # CLKアクティブ (立ち上がりエッジでラッチ)
RESET_ACTIVE_LEVEL = GPIO.LOW    # CLR active LOW (通常)
RESET_INACTIVE_LEVEL = GPIO.HIGH # CLR inactive HIGH

# Q出力の極性 (白線検出時の状態)
# WHITE_LINE_LEVEL = 0  # Qが0のとき白線検出
WHITE_LINE_LEVEL = 1    # Qが1のとき白線検出 (極性により変更)

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 出力ピン
    GPIO.setup(LATCH_PIN, GPIO.OUT, initial=LATCH_IDLE_LEVEL)
    GPIO.setup(LINE_RESET_PIN, GPIO.OUT, initial=RESET_INACTIVE_LEVEL)
    
    # 入力ピン (プルアップ/プルダウンなし、外付け抵抗に任せる)
    GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def pulse_latch():
    """CLKパルスを送信してDをQにラッチ"""
    GPIO.output(LATCH_PIN, LATCH_IDLE_LEVEL)
    time.sleep(0.00006)  # 60μs
    # 立ち上がりエッジ生成
    GPIO.output(LATCH_PIN, LATCH_ACTIVE_LEVEL)
    time.sleep(0.00006)  # 60μs
    GPIO.output(LATCH_PIN, LATCH_IDLE_LEVEL)
    time.sleep(0.00005)  # 出力安定待ち

def reset_latch():
    """CLRパルスを送信してQをリセット"""
    GPIO.output(LINE_RESET_PIN, RESET_ACTIVE_LEVEL)
    time.sleep(0.00006)  # 60μs
    GPIO.output(LINE_RESET_PIN, RESET_INACTIVE_LEVEL)
    time.sleep(0.00005)  # 出力安定待ち

def read_line_status():
    """Q出力を読み取り (デバウンス付き)"""
    samples = []
    for _ in range(3):
        samples.append(GPIO.input(LINE_PIN))
        time.sleep(0.001)
    # 多数決
    return 1 if sum(samples) >= 2 else 0

def main():
    print("=" * 60)
    print("74HC74 白線検出回路テスト")
    print("=" * 60)
    print()
    
    print("【回路構成】")
    print()
    print("  RPR220 Dout → 74HC74 D入力")
    print("  ラズパイ GPIO24 (物理18p) → 74HC74 CLK")
    print("  74HC74 Q出力 → ラズパイ GPIO23 (物理16p)")
    print("  ラズパイ GPIO18 (物理12p) → 74HC74 CLR")
    print()
    
    print("【テスト内容】")
    print("  1. リセット動作確認")
    print("  2. ラッチ動作確認")
    print("  3. 白線検出の連続モニタリング")
    print()
    
    try:
        setup_gpio()
        
        # === テスト1: リセット動作 ===
        print("-" * 60)
        print("【テスト1: リセット動作】")
        print("-" * 60)
        print()
        
        print("リセットパルス送信...")
        reset_latch()
        status = read_line_status()
        print(f"リセット後のQ出力: {status}")
        print()
        
        if status == 0:
            print("✓ リセット成功 (Q=0)")
        else:
            print("⚠ Q=1 (リセット極性が逆の可能性、または白線検出中)")
        print()
        
        input("Enter を押して次のテストへ...")
        print()
        
        # === テスト2: ラッチ動作 ===
        print("-" * 60)
        print("【テスト2: ラッチ動作】")
        print("-" * 60)
        print()
        
        print("RPR220センサの前に白い紙を置いてください")
        input("準備できたら Enter を押してください...")
        print()
        
        print("現在のQ出力 (ラッチ前): ", end="")
        status_before = read_line_status()
        print(status_before)
        print()
        
        print("ラッチパルス送信...")
        pulse_latch()
        
        status_after = read_line_status()
        print(f"ラッチ後のQ出力: {status_after}")
        print()
        
        if status_after == WHITE_LINE_LEVEL:
            print("✓ 白線検出成功")
        else:
            print("⚠ 白線が検出されていない")
            print("  原因可能性:")
            print("    - RPR220が白線を検出していない")
            print("    - WHITE_LINE_LEVELの極性が逆")
            print("    - 74HC74の配線ミス")
        print()
        
        input("白い紙を取り除いて Enter を押してください...")
        print()
        
        print("リセットパルス送信...")
        reset_latch()
        status_reset = read_line_status()
        print(f"リセット後のQ出力: {status_reset}")
        print()
        
        if status_reset != status_after:
            print("✓ リセット成功 (状態が変化)")
        else:
            print("? リセットしても状態が変わらない")
        print()
        
        input("Enter を押して連続モニタへ...")
        print()
        
        # === テスト3: 連続モニタリング ===
        print("-" * 60)
        print("【テスト3: 連続モニタリング】")
        print("-" * 60)
        print()
        print("0.5秒ごとにラッチして状態を監視します")
        print("白い紙を近づけたり遠ざけたりして動作確認")
        print("Ctrl+C で終了")
        print()
        
        # 初期リセット
        reset_latch()
        
        prev_status = None
        detection_count = 0
        
        while True:
            # ラッチ
            pulse_latch()
            
            # 読み取り
            status = read_line_status()
            
            # 状態表示
            if status != prev_status:
                ts = time.strftime('%H:%M:%S')
                if status == WHITE_LINE_LEVEL:
                    print(f"[{ts}] ★ 白線検出! (Q={status})")
                    detection_count += 1
                else:
                    print(f"[{ts}]   白線なし (Q={status})")
                prev_status = status
            
            # リセット (次回検出のため)
            reset_latch()
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print()
        print()
        print("=" * 60)
        print("テスト終了")
        print("=" * 60)
        print()
        print(f"白線検出回数: {detection_count}回")
        print()
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
