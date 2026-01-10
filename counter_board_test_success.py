"""
問題解決: カウンターボード動作確認完了

【診断結果】
✓ カウンターボードは正常動作
✓ 74HC590は正常動作
✓ RPR220フォトリフレクタは正常動作
✓ 3.3V + GND のみの接続で問題なし
✓ ラズパイ側電圧も正常値

【テスト結果の解析】
カウンタ値の遷移:
  0 → 1 (D2=0 D1=0 D0=1) ✓ RPR220 1パルス検出
  1 → 2 (D2=0 D1=1 D0=0) ✓ RPR220 1パルス検出
  2 → 3 (D2=0 D1=1 D0=1) ✓ RPR220 1パルス検出
  3 → 4 (D2=1 D1=0 D0=0) ✓ RPR220 1パルス検出
  4 → 6 (D2=1 D1=1 D0=0) ✓ RPR220 2パルス検出 (4→5→6)
  6 → 7 (D2=1 D1=1 D0=1) ✓ RPR220 1パルス検出
  7 → 0 (D2=0 D1=0 D0=0) ✓ ロールアラウンド (3ビット=0-7)

結論: カウンターボードは完全に正常動作している

【原因の特定】
カウンターボード自体に問題はなく、以前の電源異常 (5V→0V, 3.3V→4V) の
原因は「IF基板」にあると確定。

考えられるIF基板の問題:
1. IF基板上の信号ラインに5Vプルアップ抵抗
2. IF基板の配線ミス (5Vと3.3Vの混線)
3. PCA9685への電源供給経路の問題

【次のステップ】
"""

print("=" * 70)
print("問題解決報告")
print("=" * 70)
print()

print("【確認完了事項】")
print("✓ カウンターボード: 正常動作")
print("✓ 74HC590: 正常動作 (0-7カウント)")
print("✓ RPR220: 正常動作 (パルス検出)")
print("✓ 電源: 3.3V + GND のみで正常動作")
print()

print("【問題の切り分け】")
print()
print("カウンターボード単体 (3.3V + GND 直接接続):")
print("  → 正常動作 ✓")
print()
print("カウンターボード + IF基板:")
print("  → 電源異常 (5V→0V, 3.3V→4V) ✗")
print()
print("結論: 問題はIF基板にある")
print()

print("=" * 70)
print("IF基板の問題箇所の特定")
print("=" * 70)
print()

print("【確認1: IF基板の5V配線】")
print()
print("IF基板を目視確認:")
print()
print("  1. ラズパイ側 pin 2/4 (5V) からの配線")
print("     → どこに接続されているか？")
print()
print("     可能性:")
print("       (A) PCA9685のVCC (正常)")
print("       (B) 信号ラインのプルアップ抵抗 (問題)")
print("       (C) カウンターボード側コネクタ (問題)")
print()
print("  2. カウンターボード側コネクタのピン配置")
print("     ピン番号 | 信号     | 現在の接続")
print("     ---------+----------+------------------")
print("     1        | 3.3V     | ラズパイ pin 1 (3.3V)")
print("     2        | GND      | ラズパイ pin 6 (GND)")
print("     3        | D0       | ラズパイ pin 33 (GPIO13)")
print("     4        | D1       | ラズパイ pin 40 (GPIO21)")
print("     5        | D2       | ラズパイ pin 22 (GPIO25)")
print("     6        | CCLR     | ラズパイ pin 11 (GPIO17)")
print("     7        | RCLK     | ラズパイ pin 18 (GPIO24)")
print("     ...      | ...      | ...")
print()
print("  3. IF基板上のプルアップ抵抗")
print("     D0/D1/D2/CCLR/RCLK の信号ライン周辺に")
print("     プルアップ抵抗 (通常10kΩ) があるか？")
print()
print("     もしあった場合、その抵抗の接続先:")
print("       □ 3.3Vライン (正常)")
print("       □ 5Vライン (問題の原因)")
print()

print("【確認2: IF基板の抵抗測定】")
print()
print("IF基板を外して測定:")
print()
print("  1. D0ピン ↔ 5Vピン の抵抗")
print("     測定値: _____ kΩ")
print("     正常: ∞ (無限大、接続なし)")
print("     異常: 10kΩ程度 (プルアップ抵抗経由で接続)")
print()
print("  2. D1ピン ↔ 5Vピン の抵抗")
print("     測定値: _____ kΩ")
print()
print("  3. D2ピン ↔ 5Vピン の抵抗")
print("     測定値: _____ kΩ")
print()
print("  4. CCLR ↔ 5Vピン の抵抗")
print("     測定値: _____ kΩ")
print()
print("  5. RCLK ↔ 5Vピン の抵抗")
print("     測定値: _____ kΩ")
print()

print("【確認3: IF基板の電圧測定】")
print()
print("IF基板のみをラズパイに接続 (カウンターボード外した状態):")
print()
print("  1. IF基板側コネクタの各ピン電圧")
print("     3.3Vピン: _____ V (期待: 3.3V)")
print("     5Vピン: _____ V (期待: 5V)")
print("     D0ピン: _____ V (期待: 3.3V or Hi-Z)")
print("     D1ピン: _____ V")
print("     D2ピン: _____ V")
print()
print("  もしD0/D1/D2が5V付近だったら:")
print("    → IF基板上で5Vにプルアップされている (原因確定)")
print()

print("=" * 70)
print("対処法")
print("=" * 70)
print()

print("【対策A: IF基板の5Vプルアップを除去】")
print()
print("もしIF基板上の信号ラインに5Vへのプルアップ抵抗があったら:")
print()
print("  1. その抵抗を除去")
print("     → はんだ吸取器で取り外し")
print()
print("  2. または3.3Vへのプルアップに変更")
print("     変更前: D0 ─[10kΩ]─ 5V")
print("     変更後: D0 ─[10kΩ]─ 3.3V")
print()

print("【対策B: IF基板をバイパス (暫定対策)】")
print()
print("IF基板を使わずに直接接続:")
print()
print("  ラズパイ → ジャンパ線 → カウンターボード")
print("  ラズパイ → IF基板 (5V供給のみ) → PCA9685")
print()
print("  利点:")
print("    - カウンターボードは確実に動作")
print("    - PCA9685も5V供給可能")
print()
print("  欠点:")
print("    - 配線が複雑になる")
print()

print("【対策C: IF基板の再設計 (長期対策)】")
print()
print("IF基板の設計変更:")
print()
print("  1. 信号ラインのプルアップは3.3Vのみ")
print("  2. 5VラインはPCA9685専用")
print("  3. カウンターボードには3.3VとGNDのみ供給")
print()

print("=" * 70)
print("次のアクション (推奨順)")
print("=" * 70)
print()
print("1. IF基板の目視確認")
print("   → D0/D1/D2/CCLR/RCLK周辺のプルアップ抵抗を探す")
print()
print("2. IF基板単体で抵抗測定")
print("   → D0ピン ↔ 5Vピン の抵抗")
print("   → 10kΩ程度なら原因確定")
print()
print("3. IF基板のみ接続して電圧測定")
print("   → D0/D1/D2ピンの電圧")
print("   → 5V付近なら5Vプルアップあり")
print()
print("4. 対策実施")
print("   → 5Vプルアップ抵抗を除去または3.3Vに変更")
print()
print("5. 全体動作確認")
print("   → ラズパイ + IF基板 + カウンターボード + PCA9685")
print("   → 電圧確認: 5V=5V, 3.3V=3.3V")
print("   → i2cdetect -y 1 で0x40検出")
print("   → test_rpr220_counter.py で動作確認")
print()

print("=" * 70)
print("現在のステータス")
print("=" * 70)
print()
print("✓ カウンターボード: 動作確認完了")
print("✓ RPR220センサ: 動作確認完了")
print("✓ 74HC590: 動作確認完了")
print("⚠ IF基板: 調査・修正が必要")
print("⚠ PCA9685: IF基板修正後にテスト")
print()
print("進捗: 約70% (ハードウェア診断完了、IF基板修正待ち)")
print()
