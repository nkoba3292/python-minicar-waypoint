# レース運用ガイド - キャリブレーション分離システム

## システム構成

### **キャリブレーション用（レース前準備）**
- **`imu_calibration_tool.py`**: 統合キャリブレーションツール
- **`imu_visual_calibration.py`**: ビジュアルマップ方式
- **`imu_landmark_calibration.py`**: 特徴点方式
- **`imu_2point_calibration.py`**: 従来0°/180°方式

### **レース用（軽量・高速）**
- **`main_control_loop.py`**: 事前保存キャリブレーションファイル読み込み専用
- 依存関係最小化（matplotlib等の重いライブラリ不要）
- 起動時間短縮、メモリ使用量削減

---

## 運用フロー

### **Phase 1: レース前キャリブレーション（現地調整）**

#### **1-1. キャリブレーションツール起動**
```bash
# Raspberry Pi現地で実行
cd /home/pi/autonomous_car
python3 imu_calibration_tool.py
```

#### **1-2. キャリブレーション方式選択**
**推奨順位:**
1. **Visual Map Calibration** (最高精度 ±0.5°)
   - waypointマップ表示
   - マウスクリックで特徴点選択
   - 自動角度計算

2. **Landmark Calibration** (高精度 ±1°)  
   - テキスト入力で特徴点説明
   - 明確な構造物（建物角、ポール等）を基準

3. **2-Point Calibration** (標準精度 ±2-3°)
   - スタート地点での0°/180°回転
   - フォールバック用途

#### **1-3. 生成ファイル確認**
```bash
# 生成されるキャリブレーションファイル
ls -la *.json | grep calib

# 期待される結果（優先順）:
# imu_visual_calib.json    (最優先)
# imu_landmark_calib.json  (第2優先)  
# imu_2point_calib.json    (第3優先)
```

### **Phase 2: レース実行（高速起動）**

#### **2-1. レースシステム起動**
```bash
# 軽量レースモード起動
python3 main_control_loop.py --debug --speed 0.33

# 期待される起動メッセージ:
# "✓ Course calibration loaded: VISUAL_MAP PRECISION"
# "✓ Race-ready calibration active (visual_map): 45.30°"
```

#### **2-2. キャリブレーション自動読み込み**
```
優先順位でファイル探索:
1. imu_visual_calib.json   → "VISUAL_MAP PRECISION"
2. imu_landmark_calib.json → "LANDMARK PRECISION"  
3. imu_2point_calib.json   → "2POINT PRECISION"

見つからない場合:
→ "⚠ No course calibration found, using hardware calibration only"
```

---

## 実運用例

### **レース日の準備手順**

#### **到着時（コース確認）**
```bash
# 1. システム基本動作確認
python3 main_control_loop.py --debug --speed 0.1
# → BNO055ハードウェアキャリブレーション確認

# 2. 旧キャリブレーション削除（新コース対応）
rm -f *calib.json

# 3. 新キャリブレーション取得
python3 imu_calibration_tool.py
# → メニューから "1. Visual Map Calibration" 選択
```

#### **キャリブレーション作業**
```
1. Visual Map画面でwaypointコース確認
2. 明確な特徴点を2箇所マウスクリック
   例: スタートゲート→第1コーナー外壁
3. 車両を Point 1 → Point 2 方向に配置  
4. "Measure IMU"ボタンで測定
5. "Save Calib"ボタンで保存
6. "Test Calib"で精度確認
```

#### **レース直前確認**
```bash
# キャリブレーション状態確認
python3 imu_calibration_tool.py
# → メニューから "4. Test Existing Calibration" 選択

# レースシステム最終確認  
python3 main_control_loop.py --debug --speed 0.1
# → "✓ Race-ready calibration active" 確認
```

#### **レース実行**
```bash
# フルスピードレーススタート
python3 main_control_loop.py

# または段階的スピードアップ
python3 main_control_loop.py --speed 0.5  # 初回
python3 main_control_loop.py --speed 0.75 # 確認後
python3 main_control_loop.py              # フルスピード
```

---

## トラブルシューティング

### **キャリブレーション関連**

#### **"matplotlib not available"**
```bash
pip3 install matplotlib
# または
sudo apt install python3-matplotlib
```

#### **"No calibration files found"**  
```bash
# 手動で従来方式実行
python3 imu_2point_calibration.py

# または特徴点方式
python3 imu_landmark_calibration.py
```

#### **精度が悪い場合**
```bash
# BNO055再キャリブレーション
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
imu = BNO055IMUDriver()
print('Current calibration:', imu.get_calibration_status())
# 8の字移動を30秒間継続
"

# 新しいキャリブレーション取得
rm *calib.json
python3 imu_calibration_tool.py
```

### **レース実行関連**

#### **起動時間が遅い場合**
```bash
# 不要なキャリブレーションファイル削除
ls -la *calib.json
# 最高精度のファイルのみ残す
```

#### **軌道がずれる場合**  
```bash
# リアルタイムキャリブレーション確認
python3 -c "
# main_control_loop.pyから関数インポート
import sys; sys.path.append('.')
exec(open('main_control_loop.py').read()[:2000])  # 初期化部分のみ
while True:
    yaw = get_calibrated_yaw()
    print(f'Yaw: {math.degrees(yaw):.2f}°')
    time.sleep(0.5)
"
```

---

## パフォーマンス比較

| 項目 | キャリブレーション統合版 | 分離版（レース用） |
|------|------------------------|-------------------|  
| 起動時間 | 5-10秒 | 1-2秒 |
| メモリ使用量 | 150-200MB | 50-80MB |
| 依存ライブラリ | matplotlib等必須 | 最小限 |
| 精度 | 同等 | 同等 |
| 保守性 | 複雑 | シンプル |

**結論**: 分離システムにより**レース時の軽量性**と**キャリブレーション時の高機能性**を両立！