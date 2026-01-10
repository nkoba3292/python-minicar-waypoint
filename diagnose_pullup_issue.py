#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
プルアップ抵抗不足診断

症状: センサに触れないとカウントしない、触れるとカウントアップする

原因:
  RPR220 Dout → 74HC590 CCLK (pin 11) にプルアップ抵抗が不足
  → 信号線が浮いている（フローティング状態）
  → 指で触れると人体静電容量でHIGHレベルに引き上げられる

対策:
  RPR220 Doutに5V→10kΩ→Doutのプルアップ抵抗を追加
"""

import RPi.GPIO as GPIO
import time

# ピン定義
QA_PIN = 13
QB_PIN = 21
QC_PIN = 25
CCLR_PIN = 17
RCLK_PIN = 24

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(QA_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    GPIO.output(RCLK_PIN, GPIO.LOW)

def read_counter():
    """カウンタ値読み取り"""
    d0 = GPIO.input(QA_PIN)
    d1 = GPIO.input(QB_PIN)
    d2 = GPIO.input(QC_PIN)
    return d0 | (d1 << 1) | (d2 << 2)

def latch_counter():
    """カウンタラッチ"""
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)

def clear_counter():
    """カウンタクリア"""
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.001)

def main():
    print()
    print("=" * 70)
    print("  プルアップ抵抗不足診断")
    print("=" * 70)
    print()
    print("【症状】")
    print("  - センサに触れないとカウントアップしない")
    print("  - センサに触れるとカウントアップする")
    print()
    print("【原因】")
    print("  RPR220 Dout信号線にプルアップ抵抗が不足しています")
    print("  → 信号線がフローティング状態（電圧不定）")
    print("  → 指で触れると人体静電容量でHIGHに引き上げられる")
    print("  → 74HC590がクロックパルスと誤認識")
    print()
    print("【対策】")
    print("  RPR220 Doutに10kΩのプルアップ抵抗を追加してください")
    print()
    print("  回路:")
    print("    5V電源")
    print("      |")
    print("    [10kΩ] ← この抵抗を追加")
    print("      |")
    print("    RPR220 Dout ----→ 74HC590 CCLK (pin 11)")
    print("      |")
    print("   (RPR220)")
    print("      |")
    print("     GND")
    print()
    print("【プルアップ抵抗の役割】")
    print("  - RPR220がOFF時: 抵抗でDoutをHIGHレベルに保持")
    print("  - RPR220がON時: RPR220内部でGNDに接続、Doutは LOW")
    print("  → 安定したHIGH/LOW信号を74HC590に供給")
    print()
    print("【確認方法】")
    print("  1. カウンターボードを確認")
    print("  2. RPR220 Doutピン付近に10kΩ抵抗があるか？")
    print("  3. 抵抗の一端が5V、もう一端がDoutに接続されているか？")
    print()
    
    input("Enter で動作テストを開始...")
    print()
    
    try:
        setup_gpio()
        
        # テスト1: プルアップなしでのノイズ確認
        print("-" * 70)
        print("【テスト1: プルアップ抵抗なしの状態確認】")
        print("-" * 70)
        print()
        print("カウンタをクリアして10秒間監視します")
        print("何も触れずに待ってください...")
        print()
        
        clear_counter()
        latch_counter()
        
        for i in range(10):
            latch_counter()
            count = read_counter()
            print(f"{i+1}秒: カウント = {count}")
            time.sleep(1)
        
        latch_counter()
        final_count = read_counter()
        
        print()
        if final_count > 0:
            print(f"⚠️ 何も触れていないのにカウント = {final_count}")
            print("   → ノイズでカウントアップしています")
            print("   → プルアップ抵抗が必要です")
        else:
            print("✓ カウントは0のまま")
            print("  （プルアップ抵抗が実装されている可能性）")
        
        print()
        input("Enter で次のテスト...")
        print()
        
        # テスト2: 指で触れた時の動作
        print("-" * 70)
        print("【テスト2: センサに触れた時の動作】")
        print("-" * 70)
        print()
        print("カウンタをクリアします")
        
        clear_counter()
        latch_counter()
        
        print("RPR220センサのDout配線（74HC590 CCLKに接続している線）に")
        print("指で軽く触れてください...")
        input("触れたらEnterを押す...")
        
        latch_counter()
        touch_count = read_counter()
        
        print()
        print(f"カウント = {touch_count}")
        print()
        
        if touch_count > 0:
            print("⚠️ 触れただけでカウントアップしました")
            print("   → 信号線がフローティング状態です")
            print("   → プルアップ抵抗を追加してください")
        else:
            print("✓ 触れてもカウントアップしませんでした")
            print("  （プルアップ抵抗が正常に機能）")
        
        print()
        print("-" * 70)
        print("【診断結果】")
        print("-" * 70)
        print()
        
        if final_count > 0 or touch_count > 0:
            print("❌ プルアップ抵抗が不足しています")
            print()
            print("【対策手順】")
            print()
            print("■ 必要な部品:")
            print("  - 10kΩ 抵抗 × 1本")
            print("  - ジャンパーワイヤまたは半田（配線用）")
            print()
            print("■ 接続方法:")
            print("  1. カウンターボードの電源を切る")
            print("  2. RPR220 Dout配線を確認")
            print("     （74HC590 CCLK pin 11に接続されている線）")
            print("  3. 10kΩ抵抗の一端を5V電源に接続")
            print("  4. 10kΩ抵抗のもう一端をRPR220 Doutに接続")
            print("  5. 半田付けまたはブレッドボードで固定")
            print()
            print("■ 接続後の確認:")
            print("  1. このスクリプトを再実行")
            print("  2. テスト1で何も触れずにカウント=0を確認")
            print("  3. 車輪を回してカウントアップすることを確認")
            print()
            print("■ 抵抗値の選定:")
            print("  - 推奨: 10kΩ (茶黒橙金)")
            print("  - 許容範囲: 4.7kΩ～20kΩ")
            print("  - 小さすぎる(<1kΩ): 電流が大きすぎる")
            print("  - 大きすぎる(>100kΩ): ノイズに弱い")
            print()
            print("【補足】")
            print("  IF基板にプルアップ抵抗が実装されている場合:")
            print("  - IF基板の5V電源が正常か確認")
            print("  - 抵抗の半田不良や断線を確認")
            print("  - テスターで抵抗値を測定（5V-Dout間で10kΩ程度）")
        else:
            print("✓ プルアップ抵抗は正常に機能しています")
            print()
            print("別の原因を調査してください:")
            print("  - RPR220の電源電圧（4.5V～5.5V）")
            print("  - センサと車輪の距離（5mm～10mm）")
            print("  - 車輪マーカーの反射率（白色テープ推奨）")
            print("  - RPR220の故障（別のセンサで試す）")
        
        print()
    
    except KeyboardInterrupt:
        print("\n診断を中断しました")
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
