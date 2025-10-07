# course_calibration_guide.md - 実走行コースでのIMUキャリブレーション手順

## キャリブレーション戦略

### **2段階キャリブレーションの目的**

1. **BNO055内蔵キャリブレーション（Stage 1）**
   - **目的**: センサー物理誤差の補正
   - **対象**: ジャイロドリフト、加速度計オフセット、磁気干渉
   - **タイミング**: システム起動時（自動実行）
   - **精度**: ±2°程度

2. **実走行コース環境補正（Stage 2）**
   - **目的**: コース固有環境誤差の補正
   - **対象**: 磁気偏角、取り付け角度、周辺金属影響
   - **タイミング**: コース設置後（手動実行）
   - **精度**: ±0.5°程度（目標）

## 実走行での校正手順

### **準備作業**
```bash
# 1. 車両をコースのスタート地点に配置
# 2. 車両の向きを進行方向（コース0°方向）に正確に合わせる
# 3. 周辺に磁気干渉源がないことを確認
```

### **Stage 1: BNO055基本校正**
```bash
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
import time

print('=== BNO055 Hardware Calibration ===')
imu = BNO055IMUDriver()

print('Calibrating... (Move sensor in figure-8 pattern)')
for i in range(30):
    status = imu.get_calibration_status()
    print(f'Progress: Sys={status[\"sys\"]} Gyro={status[\"gyro\"]} Accel={status[\"accel\"]} Mag={status[\"mag\"]}')
    time.sleep(1)

if imu.is_calibrated():
    print('✓ BNO055 hardware calibration complete')
else:
    print('⚠ Continue moving sensor for better calibration')
"
```

### **Stage 2: コース環境校正（推奨: 特徴点2点方式）**

#### **特徴点の選定基準**
```
✅ 良い特徴点の例:
- コーナーのエッジ（壁の角）
- ゴールポスト、スタート/フィニッシュゲート
- 明確な建造物の角（柱、フェンスの角等）
- コース境界の明確な変化点

❌ 避けるべき基準:
- 曖昧なライン（太さ、角度が不明確）
- 移動可能な物体（コーン、旗等）
- 磁気干渉源の近く（金属構造物）
```

#### **特徴点ベースキャリブレーション実行（推奨）**
```bash
# 高精度特徴点ベースキャリブレーション
python3 imu_landmark_calibration.py
```

**実行時の操作手順:**
1. **特徴点1**: 車両を明確な基準方向に配置
   - 例: 「スタートゲート中央 → ゴールポスト中央」（0°）
   - 例: 「建物角A → 建物角B」（45°）
   - 車両前方を目標方向に正確に向ける
   - 正確な方位角（度）を入力
   - `Enter`で測定

2. **特徴点2**: 車両を別の確実な方向に配置  
   - 例: 「第1コーナー → 第2コーナー」（90°）
   - 例: 「フェンス交点C → ポールD」（135°）
   - 2点間の角度差は60°以上推奨（90°〜120°が理想）
   - 正確な方位角（度）を入力
   - `Enter`で測定

3. **自動計算**: 2点の方位から精密オフセット値を計算し `imu_landmark_calib.json` に保存

#### **従来方式（フォールバック）**
```bash
# 従来の0°/180°回転方式
python3 imu_2point_calibration.py
```
- スタート地点での手動180°回転
- 特徴点が不明確な場合の代替手段

### **校正結果の確認**
```bash
# キャリブレーションファイル内容
cat imu_2point_calib.json

# 統合キャリブレーション動作テスト
python3 -c "
import sys
sys.path.append('.')

# 段階的キャリブレーション結果を確認
from bno055_imu_driver import BNO055IMUDriver
from imu_2point_calibration import IMU2PointCalibration
import math

# Stage 1: BNO055 hardware
imu = BNO055IMUDriver()
raw_yaw = imu.get_yaw()
print(f'Raw BNO055 yaw: {math.degrees(raw_yaw):.2f}°')

# Stage 2: Course correction
calib = IMU2PointCalibration()
if calib.load_calibration():
    final_yaw = calib.calibrate_yaw(raw_yaw)
    correction = math.degrees(final_yaw - raw_yaw)
    print(f'Course-corrected yaw: {math.degrees(final_yaw):.2f}°')
    print(f'Applied correction: {correction:.2f}°')
else:
    print('No course calibration applied')
"
```

## 実走行テスト手順

### **低速テスト**
```bash
# デバッグモードで動作確認
python3 main_control_loop.py --debug --speed 0.1

# 確認ポイント:
# - "✓ BNO055 sensor calibration complete"
# - "✓ Course environment calibration loaded"
# - "✓ Full calibration applied: XXX.XX°"
```

### **方位精度テスト**
```bash
# 静止状態での方位安定性確認
python3 -c "
from main_control_loop import get_calibrated_yaw
import math, time

print('Testing yaw stability (10 seconds)...')
for i in range(10):
    yaw = get_calibrated_yaw()
    print(f'{i+1:2d}s: {math.degrees(yaw):6.2f}°')
    time.sleep(1)
"
```

### **回転精度テスト**
```bash
# 手動で車両を90°ずつ回転させながら測定
python3 -c "
from main_control_loop import get_calibrated_yaw
import math

print('Rotate vehicle manually and press Enter at each 90° position:')
angles = ['0°', '90°', '180°', '270°']
for expected in angles:
    input(f'Position vehicle at {expected}, then press Enter: ')
    yaw = get_calibrated_yaw()
    print(f'Expected: {expected}, Measured: {math.degrees(yaw):6.2f}°')
"
```

## トラブルシューティング

### **キャリブレーション精度が悪い場合**
```bash
# 1. 磁気干渉源チェック
# - 周辺の金属物、モーター、電子機器を確認
# - 車載バッテリー、モーターからIMUを離す

# 2. BNO055再校正
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
imu = BNO055IMUDriver()
status = imu.get_calibration_status()
print(f'Current calibration: {status}')
# 全て3になるまで8の字移動を継続
"

# 3. 2点キャリブレーション再実行
rm imu_2point_calib.json
python3 imu_2point_calibration.py
```

### **異常な角度値が出る場合**
```bash
# キャリブレーションファイル削除してリセット
rm imu_2point_calib.json

# 基本動作確認
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
import math
imu = BNO055IMUDriver()
for i in range(5):
    yaw = imu.get_yaw()
    print(f'Raw yaw: {math.degrees(yaw):6.2f}°')
    time.sleep(1)
"
```

## 実用運転での注意点

### **校正済みシステムの確認**
- 起動時に `"✓ Full calibration applied"` が表示されること
- 静止時のヨー角変動が ±1°以内であること
- 手動回転テストで ±2°以内の精度であること

### **定期的な再校正**
- **毎日の運転前**: BNO055状態確認（自動）
- **週1回**: 2点キャリブレーション再実行（手動）
- **コース変更時**: 必ず2点キャリブレーション実行

これにより、**センサーレベル + 環境レベル**の両方の誤差を補正し、実走行で高精度な自律制御が可能になります。