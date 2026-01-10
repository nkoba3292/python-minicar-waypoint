#!/usr/bin/env python3
"""
RPR220 フォトリフレクタの出力テスト（車輪未取付け）

手や紙などを RPR220 の前に置いて、出力が変化するか確認
"""

import RPi.GPIO as GPIO
import time

# RPR220 が接続されている可能性のあるピン
# 74HC590 pin 11 (CCLK) に接続されているはず
POSSIBLE_RPR220_PINS = [18, 23, 22, 27, 5, 6, 12, 16, 20, 26]

def find_rpr220_output():
    """RPR220 出力ピンを探す"""
    print("=" * 60)
    print("RPR220 出力ピンの探索")
    print("=" * 60)
    print()
    print("各ピンの状態を読み取ります")
    print("手や紙を RPR220 の前に置いたり離したりして、")
    print("値が変化するピンを探してください")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 全候補ピンを入力に設定
    for pin in POSSIBLE_RPR220_PINS:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        except:
            pass
    
    time.sleep(0.1)
    
    print("初期値を読み取ります...")
    print()
    print("BCM Pin | 初期値 | 説明")
    print("--------+--------+------------------------")
    
    initial_values = {}
    for pin in POSSIBLE_RPR220_PINS:
        try:
            val = GPIO.input(pin)
            initial_values[pin] = val
            print(f"  {pin:2d}    |   {val}    | ", end="")
            if val == 1:
                print("HIGH (障害物なし or プルアップ)")
            else:
                print("LOW (障害物あり or GND)")
        except Exception as e:
            print(f"  {pin:2d}    | エラー | {e}")
    
    print()
    print("=" * 60)
    print("RPR220 のテスト開始")
    print("=" * 60)
    print()
    print("【テスト方法】")
    print("  1. RPR220 の前に手や白い紙を置く")
    print("  2. 10秒間モニタします")
    print("  3. 手や紙を近づけたり離したりしてください")
    print()
    print("開始します... (10秒間)")
    print()
    
    # 10秒間モニタ
    changes = {pin: 0 for pin in POSSIBLE_RPR220_PINS}
    last_values = initial_values.copy()
    
    start_time = time.time()
    while time.time() - start_time < 10:
        for pin in POSSIBLE_RPR220_PINS:
            try:
                val = GPIO.input(pin)
                if val != last_values[pin]:
                    changes[pin] += 1
                    print(f"BCM {pin:2d}: {last_values[pin]} → {val} (変化 #{changes[pin]})")
                    last_values[pin] = val
            except:
                pass
        time.sleep(0.01)
    
    print()
    print("=" * 60)
    print("結果")
    print("=" * 60)
    print()
    
    active_pins = [pin for pin, count in changes.items() if count > 0]
    
    if len(active_pins) == 0:
        print("⚠ どのピンも変化しませんでした")
        print()
        print("【確認事項】")
        print("  1. RPR220 に電源が供給されているか (VCC/GND)")
        print("  2. RPR220 の LED が光っているか (スマホカメラで確認)")
        print("  3. RPR220 の出力ピンが本当に GPIO に接続されているか")
        print("  4. RPR220 の出力がどこかに接続されているか")
        print()
        print("【次の確認】")
        print("  RPR220 の出力ピンを直接マルチメータで測定:")
        print("  - 障害物なし: HIGH (2~3V)")
        print("  - 障害物あり: LOW (0V)")
    else:
        print(f"✓ {len(active_pins)} ピンで変化を検出しました:")
        for pin in active_pins:
            print(f"  BCM {pin}: {changes[pin]}回変化")
        print()
        print("【推奨】")
        if len(active_pins) == 1:
            print(f"  BCM {active_pins[0]} が RPR220 出力 (CCLK) の可能性が高い")
            return active_pins[0]
        else:
            print(f"  複数のピンが反応しています")
            print(f"  最も変化が多いピンが RPR220 出力の可能性が高い:")
            max_pin = max(active_pins, key=lambda p: changes[p])
            print(f"  → BCM {max_pin} ({changes[max_pin]}回)")
            return max_pin
    
    return None

def continuous_monitor(rpr220_pin):
    """RPR220 出力を連続モニタ"""
    print()
    print("=" * 60)
    print(f"BCM {rpr220_pin} の連続モニタ")
    print("=" * 60)
    print()
    print("手や紙を RPR220 の前に置いたり離したりしてください")
    print("Ctrl+C で終了")
    print()
    
    GPIO.setup(rpr220_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    last_val = GPIO.input(rpr220_pin)
    pulse_count = 0
    
    print(f"初期値: {last_val}")
    print()
    
    try:
        while True:
            val = GPIO.input(rpr220_pin)
            if val != last_val:
                pulse_count += 1
                state = "反射あり (LOW)" if val == 0 else "反射なし (HIGH)"
                print(f"変化 #{pulse_count}: {last_val} → {val} ({state})")
                last_val = val
            time.sleep(0.01)
    except KeyboardInterrupt:
        print()
        print(f"合計 {pulse_count} 回の変化を検出しました")

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  RPR220 出力テスト (車輪未取付け)  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    print("【目的】")
    print("  RPR220 の出力が正常に動作しているか確認")
    print()
    print("【テスト内容】")
    print("  1. RPR220 出力ピンを自動検出")
    print("  2. 手や紙を近づけて出力変化を確認")
    print()
    
    try:
        rpr220_pin = find_rpr220_output()
        
        if rpr220_pin:
            continuous_monitor(rpr220_pin)
        else:
            print()
            print("【追加テスト】")
            print("マルチメータで RPR220 出力ピンの電圧を直接測定してください:")
            print("  1. 障害物なし (手を離す): 2~3V (HIGH)")
            print("  2. 障害物あり (手を近づける): 0V (LOW)")
            print()
            print("もし電圧が変化しなければ:")
            print("  - RPR220 の電源 (VCC/GND) を確認")
            print("  - RPR220 が故障している可能性")
        
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
