#!/usr/bin/env python3
"""
RPR220 → 74HC590 カウンタ動作テスト

RPR220 Dout は 74HC590 pin 11 (CCLK) に直接接続されている
GPIO では読めないので、カウンタ値の変化で確認する

テスト方法:
1. カウンタをクリア
2. RPR220 の前で手を振る
3. カウンタ値が増えるか確認
"""

import RPi.GPIO as GPIO
import time

# ピン設定
DATA_PINS = {
    'D0 (QA)': 13,
    'D1 (QB)': 21,
    'D2 (QC)': 25,
}

CCLR_PIN = 17  # BCM 17 → 74HC590 pin 10 (CCLR)
RCLK_PIN = 24  # BCM 24 → 74HC590 pin 13 (RCLK)

def read_counter():
    """カウンタ値を読む (0-7)"""
    d0 = GPIO.input(13)
    d1 = GPIO.input(21)
    d2 = GPIO.input(25)
    return (d2 << 2) | (d1 << 1) | d0

def latch_counter():
    """カウンタ値をラッチ（出力レジスタに転送）"""
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.005)

def clear_counter():
    """カウンタをクリア"""
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)

def monitor_counter_continuous():
    """カウンタ値を連続モニタ"""
    print("=" * 60)
    print("カウンタ値の連続モニタ")
    print("=" * 60)
    print()
    print("RPR220 の前で手を振ってください")
    print("カウンタ値が増えれば RPR220 は正常に動作しています")
    print()
    print("Ctrl+C で終了")
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
    print("カウンタをクリアしています...")
    clear_counter()
    latch_counter()
    time.sleep(0.1)
    
    initial = read_counter()
    print(f"初期値: {initial}")
    print()
    print("モニタ開始 (0.5秒ごとに更新)")
    print()
    
    last_value = initial
    
    try:
        while True:
            # ラッチして読む
            latch_counter()
            time.sleep(0.05)
            
            current = read_counter()
            
            if current != last_value:
                # カウンタ値が変化した
                d0 = current & 1
                d1 = (current >> 1) & 1
                d2 = (current >> 2) & 1
                
                print(f"カウンタ: {last_value} → {current} (D2={d2} D1={d1} D0={d0})")
                
                if current > last_value:
                    print(f"  ✓ RPR220 が {current - last_value} パルス検出！")
                elif current < last_value:
                    print(f"  ※ カウンタがラップアラウンド (7 → 0)")
                
                last_value = current
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print()
        print("=" * 60)
        print("テスト終了")
        print("=" * 60)
        print()
        
        final = read_counter()
        total_pulses = (final - initial) % 8
        
        print(f"初期値: {initial}")
        print(f"最終値: {final}")
        print(f"検出パルス数: {total_pulses} (モジュロ8)")
        print()
        
        if total_pulses > 0:
            print("✓ RPR220 は正常に動作しています！")
            print()
            print("【次のステップ】")
            print("  1. 車輪にエンコーダディスク（白黒パターン）を取り付け")
            print("  2. RPR220 を車輪に固定")
            print("  3. 車輪を回転させてカウントテスト")
        else:
            print("⚠ パルスが検出されませんでした")
            print()
            print("【確認事項】")
            print("  1. RPR220 の LED が光っているか (スマホカメラで確認)")
            print("  2. RPR220 の前で手や白い紙を素早く動かしたか")
            print("  3. RPR220 Dout → 74HC590 pin 11 (CCLK) の配線")
            print("  4. プルアップ抵抗の値 (推奨: 10kΩ)")

def test_initial_state():
    """初期状態の確認"""
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  RPR220 → 74HC590 動作テスト  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    print("【接続構成】")
    print("  RPR220 Dout → (プルアップ抵抗) → 74HC590 pin 11 (CCLK)")
    print("  74HC590 出力:")
    print("    - pin 15 (QA/D0) → BCM 13 (物理33p)")
    print("    - pin 1  (QB/D1) → BCM 21 (物理40p)")
    print("    - pin 2  (QC/D2) → BCM 25 (物理22p)")
    print()
    print("【テスト内容】")
    print("  RPR220 からのパルスで 74HC590 がカウントアップするか確認")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # ピン設定
    for pin in DATA_PINS.values():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    
    # 現在の値を確認
    latch_counter()
    time.sleep(0.1)
    current = read_counter()
    
    print(f"現在のカウンタ値: {current}")
    print()

def main():
    try:
        test_initial_state()
        monitor_counter_continuous()
        
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
