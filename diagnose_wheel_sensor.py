#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
車輪センサ診断スクリプト

問題: 車輪速センサの感度が良くない
原因候補:
  1. RPR220の電源電圧不足
  2. センサと車輪の距離が遠い
  3. 車輪マーカーが不適切（反射率が低い）
  4. 周囲光の影響
  5. RPR220のDout信号レベルが不足
  6. 74HC590のCCLK入力の閾値に達していない

診断手順:
  1. RPR220の電源電圧確認（5V系統）
  2. センサ距離の目視確認
  3. カウンタ値の連続読み取り（RCLK後の変化）
  4. RPR220 Dout電圧の測定（テスターが必要）
  5. プルアップ抵抗の確認
"""

import RPi.GPIO as GPIO
import time
import sys
import select

# ピン定義
QA_PIN = 13   # 74HC590 D0
QB_PIN = 21   # 74HC590 D1
QC_PIN = 25   # 74HC590 D2
CCLR_PIN = 17 # 74HC590 CCLR
RCLK_PIN = 24 # 74HC590 RCLK

def setup_gpio():
    """GPIO初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 入力ピン
    GPIO.setup(QA_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QB_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(QC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # 制御ピン
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(RCLK_PIN, GPIO.OUT)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    GPIO.output(RCLK_PIN, GPIO.LOW)

def read_counter():
    """カウンタ値を読み取り"""
    d0 = GPIO.input(QA_PIN)
    d1 = GPIO.input(QB_PIN)
    d2 = GPIO.input(QC_PIN)
    return d0 | (d1 << 1) | (d2 << 2)

def clear_counter():
    """カウンタクリア"""
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.001)

def latch_counter():
    """カウンタラッチ"""
    GPIO.output(RCLK_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK_PIN, GPIO.LOW)
    time.sleep(0.001)

def check_power():
    """電源電圧確認ガイド"""
    print("=" * 70)
    print("【ステップ1: 電源電圧確認】")
    print("=" * 70)
    print()
    print("テスターで以下を測定してください:")
    print("  - RPR220の電源ピン(Vcc): 4.5V～5.5Vであること")
    print("  - RPR220のGNDピン: 0Vであること")
    print("  - Raspberry Piの5V出力: 5V±5%であること")
    print()
    print("■ 測定方法:")
    print("  1. テスターを電圧測定モード(DC 20V)に設定")
    print("  2. 黒プローブをGNDに接続")
    print("  3. 赤プローブをRPR220のVccピンに接続")
    print("  4. 電圧を読み取る")
    print()
    print("■ 判定:")
    print("  - 4.5V～5.5V: OK")
    print("  - 4.5V未満: 電源電圧不足（配線抵抗、接触不良）")
    print("  - 5.5V以上: 電源電圧過剰（レギュレータ異常）")
    print()
    input("確認したらEnterを押してください...")
    print()

def check_distance():
    """センサ距離確認ガイド"""
    print("=" * 70)
    print("【ステップ2: センサ距離確認】")
    print("=" * 70)
    print()
    print("RPR220と車輪の距離を目視確認してください:")
    print("  - 推奨距離: 5mm～10mm")
    print("  - 近すぎる(<3mm): 飽和して検出できない場合あり")
    print("  - 遠すぎる(>15mm): 反射光が弱くて検出できない")
    print()
    print("■ 測定方法:")
    print("  1. 定規またはノギスでセンサと車輪の距離を測定")
    print("  2. センサの発光面と受光面の中心から車輪表面まで")
    print()
    print("現在の距離(mm)を入力してください: ", end="")
    sys.stdout.flush()
    distance_str = input().strip()
    
    try:
        distance = float(distance_str)
        print()
        if distance < 3:
            print("⚠️ 距離が近すぎます（< 3mm）")
            print("   → 3mm～5mmに調整してください")
        elif distance > 15:
            print("⚠️ 距離が遠すぎます（> 15mm）")
            print("   → 5mm～10mmに調整してください")
        else:
            print("✓ 距離は適切です（5mm～10mm）")
    except ValueError:
        print("（距離未入力、スキップします）")
    print()
    input("確認したらEnterを押してください...")
    print()

def check_marker():
    """車輪マーカー確認ガイド"""
    print("=" * 70)
    print("【ステップ3: 車輪マーカー確認】")
    print("=" * 70)
    print()
    print("車輪のマーカーを確認してください:")
    print("  - 推奨: 白色テープまたは反射テープ")
    print("  - 幅: 10mm以上")
    print("  - コントラスト: 黒い車輪に白いマーカー")
    print()
    print("■ 良いマーカー:")
    print("  - 白色ビニールテープ（光沢あり）")
    print("  - 反射テープ（自転車用など）")
    print("  - アルミテープ")
    print()
    print("■ 悪いマーカー:")
    print("  - 車輪本体と色が近い")
    print("  - マーカーが小さすぎる（< 5mm）")
    print("  - 汚れて反射率が低下している")
    print()
    input("確認したらEnterを押してください...")
    print()

def test_continuous_read():
    """連続読み取りテスト"""
    print("=" * 70)
    print("【ステップ4: 連続読み取りテスト】")
    print("=" * 70)
    print()
    print("車輪をゆっくり回しながらカウンタ値を監視します")
    print("Enter で終了")
    print()
    print("  [RCLK前] → [RCLK後] (差分)")
    print("-" * 40)
    
    # カウンタクリア
    clear_counter()
    latch_counter()
    
    count = 0
    try:
        while True:
            # Enter 入力チェック（非ブロッキング）
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                break
            
            # RCLK前の値
            before_count = read_counter()
            
            # ラッチ
            latch_counter()
            
            # RCLK後の値
            after_count = read_counter()
            
            # 差分
            diff = after_count - before_count
            
            if diff != 0:
                count += 1
                print(f"{count:3d}: [{before_count}] → [{after_count}] (差分: {diff:+d})")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        pass
    
    print()
    print(f"カウント変化回数: {count}")
    print()
    
    if count == 0:
        print("⚠️ カウンタ値が変化しませんでした")
        print()
        print("考えられる原因:")
        print("  1. RPR220が車輪の回転を検出していない")
        print("     → 電源電圧、距離、マーカーを再確認")
        print("  2. RPR220 DoutとCCLK (74HC590 pin 11)の配線が断線")
        print("     → テスターで導通確認")
        print("  3. 74HC590のCCLK入力の閾値に達していない")
        print("     → Dout電圧を測定（次のステップ）")
    elif count < 5:
        print("⚠️ カウント変化が少ないです")
        print("   → センサ距離またはマーカーを調整")
    else:
        print("✓ カウント変化を検出しました")
    print()
    input("確認したらEnterを押してください...")
    print()

def check_dout_voltage():
    """Dout電圧確認ガイド"""
    print("=" * 70)
    print("【ステップ5: RPR220 Dout電圧確認】")
    print("=" * 70)
    print()
    print("テスターでRPR220 Dout電圧を測定してください:")
    print()
    print("■ 測定方法:")
    print("  1. テスターを電圧測定モード(DC 20V)に設定")
    print("  2. 黒プローブをGNDに接続")
    print("  3. 赤プローブをRPR220 Dout (74HC590 CCLK接続点)に接続")
    print("  4. 車輪を回さない状態で電圧を測定 → HIGH電圧")
    print("  5. 車輪のマーカー上にセンサを置いて電圧を測定 → LOW電圧")
    print()
    print("■ 判定:")
    print("  - HIGH電圧: 3.0V～5.0V → OK")
    print("  - LOW電圧: 0V～0.8V → OK")
    print("  - HIGH-LOW差: 2.0V以上 → OK")
    print()
    print("  - HIGH電圧が低い(<3.0V): プルアップ抵抗を追加")
    print("  - LOW電圧が高い(>0.8V): RPR220が検出していない")
    print("  - 差が小さい(<2.0V): コントラスト不足")
    print()
    input("確認したらEnterを押してください...")
    print()

def check_pullup():
    """プルアップ抵抗確認ガイド"""
    print("=" * 70)
    print("【ステップ6: プルアップ抵抗確認】")
    print("=" * 70)
    print()
    print("RPR220 Doutのプルアップ抵抗を確認してください:")
    print()
    print("■ 回路:")
    print("  5V ---- [10kΩ] ---- RPR220 Dout ---- 74HC590 CCLK (pin 11)")
    print("                           |")
    print("                       (RPR220内部)")
    print("                           |")
    print("                          GND")
    print()
    print("■ 確認方法:")
    print("  1. 回路図または基板上でプルアップ抵抗の有無を確認")
    print("  2. テスターの抵抗測定モードで抵抗値を測定")
    print("     （電源OFF、Dout-5V間の抵抗）")
    print()
    print("■ 判定:")
    print("  - 抵抗値 5kΩ～20kΩ: OK")
    print("  - 抵抗値 無限大(∞): プルアップ抵抗なし → 追加必要")
    print("  - 抵抗値 < 1kΩ: 抵抗値が小さすぎる → 10kΩに変更")
    print()
    print("プルアップ抵抗は実装されていますか? (y/n): ", end="")
    sys.stdout.flush()
    answer = input().strip().lower()
    
    if answer == 'n':
        print()
        print("⚠️ プルアップ抵抗がありません")
        print()
        print("対策:")
        print("  1. 10kΩ抵抗を準備")
        print("  2. 5V ---- [10kΩ] ---- RPR220 Dout に接続")
        print("  3. 半田付けまたはブレッドボード配線")
    else:
        print("✓ プルアップ抵抗あり")
    print()
    input("確認したらEnterを押してください...")
    print()

def main():
    """メイン診断フロー"""
    print()
    print("=" * 70)
    print("  車輪センサ診断スクリプト")
    print("=" * 70)
    print()
    print("このスクリプトは車輪センサの感度が良くない原因を診断します")
    print()
    
    try:
        setup_gpio()
        
        # ステップ1: 電源電圧確認
        check_power()
        
        # ステップ2: センサ距離確認
        check_distance()
        
        # ステップ3: 車輪マーカー確認
        check_marker()
        
        # ステップ4: 連続読み取りテスト
        test_continuous_read()
        
        # ステップ5: Dout電圧確認
        check_dout_voltage()
        
        # ステップ6: プルアップ抵抗確認
        check_pullup()
        
        # 総合判定
        print("=" * 70)
        print("【診断完了】")
        print("=" * 70)
        print()
        print("上記の診断結果から、以下を確認してください:")
        print()
        print("1. 電源電圧が4.5V～5.5Vであること")
        print("2. センサ距離が5mm～10mmであること")
        print("3. 車輪マーカーが白色で反射率が高いこと")
        print("4. カウンタ値が変化すること")
        print("5. Dout電圧がHIGH/LOWで2V以上の差があること")
        print("6. プルアップ抵抗(10kΩ)が実装されていること")
        print()
        print("問題が解決しない場合:")
        print("  - RPR220の故障（別のRPR220で試す）")
        print("  - 74HC590の故障（別のICで試す）")
        print("  - 配線の断線（テスターで導通確認）")
        print()
    
    except KeyboardInterrupt:
        print("\n診断を中断しました")
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
