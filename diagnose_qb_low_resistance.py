#!/usr/bin/env python3
"""
74HC590 QB (pin 1) が GND に 700Ω で接続されている原因を特定

IC交換以外の可能性を全て調査する
"""

import RPi.GPIO as GPIO
import time

# ピン配置
QB_PIN = 21  # BCM 21 = BOARD 40 = 74HC590 pin 1 (QB)

CONTROL_PINS = {
    'CCLR (pin10)': 17,   # BCM 17 = BOARD 11
    'RCLK (pin13)': 24,   # BCM 24 = BOARD 18
}

# 未使用の可能性があるピン（Pin 12 G' 候補）
UNUSED_PINS = [18, 23, 22, 27]  # BCM

def test_qb_with_different_modes():
    """QB ピンを異なるモードで読み取り"""
    print("=" * 60)
    print("テスト1: QB (BCM 21) の詳細診断")
    print("=" * 60)
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # モード1: 入力（プル無し）
    GPIO.setup(QB_PIN, GPIO.IN)
    time.sleep(0.2)
    samples = [GPIO.input(QB_PIN) for _ in range(10)]
    print(f"入力モード（プル無し）: {samples}")
    print(f"  → 常に 0: {all(s == 0 for s in samples)}")
    print()
    
    # モード2: 入力（プルアップ）
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(0.2)
    samples = [GPIO.input(QB_PIN) for _ in range(10)]
    print(f"入力モード（プルアップ）: {samples}")
    print(f"  → 常に 0: {all(s == 0 for s in samples)}")
    if all(s == 0 for s in samples):
        print("  ★ プルアップしても 0 = 強いGND駆動（700Ω の証拠）")
    print()
    
    # モード3: 入力（プルダウン）
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    time.sleep(0.2)
    samples = [GPIO.input(QB_PIN) for _ in range(10)]
    print(f"入力モード（プルダウン）: {samples}")
    print()

def test_control_signals():
    """CCLR と RCLK の状態を確認"""
    print("=" * 60)
    print("テスト2: 制御信号の状態")
    print("=" * 60)
    print()
    
    for name, pin in CONTROL_PINS.items():
        GPIO.setup(pin, GPIO.IN)
        val = GPIO.input(pin)
        print(f"{name}: BCM {pin} = {val}")
    
    print()
    print("期待値:")
    print("  CCLR (pin10) = 1 (HIGH) → カウンタ動作中")
    print("  RCLK (pin13) = 1 or 0 → ラッチ信号")
    print()

def test_unused_pins():
    """未使用ピンから Pin 12 (G') を探す"""
    print("=" * 60)
    print("テスト3: Pin 12 (G'/Output Enable) の探索")
    print("=" * 60)
    print()
    print("Pin 12 (G') が HIGH だと出力が Hi-Z になるが、")
    print("QB が GND に 700Ω = Hi-Z ではない")
    print()
    
    for pin in UNUSED_PINS:
        try:
            GPIO.setup(pin, GPIO.IN)
            time.sleep(0.1)
            no_pull = GPIO.input(pin)
            
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            time.sleep(0.1)
            with_up = GPIO.input(pin)
            
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            time.sleep(0.1)
            with_down = GPIO.input(pin)
            
            print(f"BCM {pin:2d}: 通常={no_pull} PullUp={with_up} PullDown={with_down}")
            
            if no_pull == 1 and with_up == 1 and with_down == 0:
                print(f"  → フロート（未接続の可能性）")
            elif no_pull == 1 and with_up == 1 and with_down == 1:
                print(f"  → VCC に接続（Pin 12 候補？）")
            elif no_pull == 0 and with_up == 0 and with_down == 0:
                print(f"  → GND に接続")
        except Exception as e:
            print(f"BCM {pin}: エラー - {e}")
    
    print()

def test_cclr_pulse():
    """CCLR パルスで QB が変化するか確認"""
    print("=" * 60)
    print("テスト4: CCLR (Counter Clear) パルステスト")
    print("=" * 60)
    print()
    print("CCLR を LOW パルスしてカウンタをクリア")
    print("QB が 0 から変化するか確認")
    print()
    
    CCLR_PIN = 17
    
    # 現在の QB 値
    GPIO.setup(QB_PIN, GPIO.IN)
    before = GPIO.input(QB_PIN)
    print(f"CCLR パルス前の QB: {before}")
    
    # CCLR を LOW パルス
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.1)
    
    # RCLK でラッチ
    RCLK_PIN = 24
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.1)
    
    # CCLR パルス後の QB 値
    after = GPIO.input(QB_PIN)
    print(f"CCLR パルス後の QB: {after}")
    
    if before == 0 and after == 0:
        print("  → QB は変化なし（常に 0）")
        print("  ★ これは異常！カウンタがクリアされても 0 のまま")
    print()

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  74HC590 QB (pin 1) 700Ω 問題の詳細診断  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    print("目的: IC交換以外の原因を全て調査")
    print()
    
    try:
        test_qb_with_different_modes()
        test_control_signals()
        test_unused_pins()
        test_cclr_pulse()
        
        print("=" * 60)
        print("総合診断結果")
        print("=" * 60)
        print()
        print("【確認済み】")
        print("  ✓ IC型番: TC74HC590AP（東芝）")
        print("  ✓ 配線: 物理40p → 74HC590 pin 1")
        print("  ✓ QB と GND 間: 700Ω（異常に低い）")
        print()
        print("【次の確認事項】")
        print()
        print("1. ★最優先★ Pin 12 (G'/OE) の電圧測定")
        print("   マルチメータで直接測定:")
        print("   - 0V (GND) → 出力有効（正常）")
        print("   - 3.3V → 出力無効（異常、Hi-Zになるはず）")
        print("   - 不安定 → 配線ミスまたは未接続")
        print()
        print("2. Pin 11 (CCLK) にパルスが来ているか")
        print("   RPR220 からのパルス入力を確認")
        print("   - オシロスコープで測定")
        print("   - または LED + 抵抗で目視確認")
        print()
        print("3. VCC (pin 16) と GND (pin 8) の電圧")
        print("   - 3.3V ± 0.3V が正常範囲")
        print()
        print("4. 他の出力ピン (QA, QC) も GND に低抵抗か確認")
        print("   - QA (pin 15): 物理33p を外して測定")
        print("   - QC (pin 2): 物理22p を外して測定")
        print("   - QB だけ異常 → QB 出力の故障")
        print("   - 全出力が低抵抗 → Pin 12 (G') の問題")
        print()
        print("【700Ω の解釈】")
        print("  - 正常な出力: >100kΩ (Hi-Z または HIGH/LOW駆動)")
        print("  - 700Ω は CMOS 出力としては異常")
        print("  - 可能性:")
        print("    a) QB 出力トランジスタの破損（GND側ON固定）")
        print("    b) Pin 12 が中途半端な電圧（1V付近）")
        print("    c) 外部にプルダウン回路がある")
        print()
        
    except KeyboardInterrupt:
        print("\n\n中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        GPIO.cleanup()
        print("\nGPIO クリーンアップ完了")

if __name__ == "__main__":
    main()
