# Raspberry Pi 4B 統合テスト チェックリスト

## 事前準備

### ハードウェア接続確認
- [ ] **PCA9685 PWMドライバ**
  - VCC → 3.3V または 5V
  - GND → GND
  - SDA → GPIO 2 (SDA)
  - SCL → GPIO 3 (SCL)
  - I2Cアドレス: 0x40 (デフォルト)

- [ ] **BNO055 IMUセンサー**
  - VIN → 3.3V
  - GND → GND  
  - SDA → GPIO 2 (SDA)
  - SCL → GPIO 3 (SCL)
  - I2Cアドレス: 0x28 (デフォルト)

- [ ] **モーター・サーボ接続**
  - Drive Motor → PCA9685 Channel 0
  - Steering Servo → PCA9685 Channel 1

### ソフトウェア準備
- [ ] Raspberry Pi OSアップデート
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```

- [ ] I2C有効化
  ```bash
  sudo raspi-config  # Interface Options → I2C → Enable
  ```

- [ ] I2Cツールインストール
  ```bash
  sudo apt install -y i2c-tools python3-pip
  ```

## デプロイメント手順

### 1. ファイル転送
```bash
# Windowsから実行
.\deploy_to_pi.bat
# または
bash deploy_to_pi.sh
```

### 2. 手動デプロイの場合
```bash
# ディレクトリ作成
ssh pi@raspberrypi.local "mkdir -p /home/pi/autonomous_car"

# ファイル転送
scp -r *.py *.json *.txt pi@raspberrypi.local:/home/pi/autonomous_car/

# Raspberry Piにログイン
ssh pi@raspberrypi.local
cd autonomous_car
```

### 3. 依存関係インストール
```bash
pip3 install -r requirements.txt
```

## ハードウェア診断テスト

### I2C機器検出
```bash
# I2C機器スキャン
i2cdetect -y 1

# 期待される結果:
# 0x28: BNO055 IMU
# 0x40: PCA9685 PWM
```

### PCA9685テスト
```bash
python3 -c "
from pca9685_motor_driver import PCA9685MotorDriver
motor = PCA9685MotorDriver()
print('PCA9685 initialized successfully')
motor.stop()
"
```

### BNO055テスト
```bash
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
imu = BNO055IMUDriver()
info = imu.get_sensor_info()
print(f'BNO055 info: {info}')
"
```

## システム統合テスト

### 1. モックモード動作確認
```bash
# PCと同じ動作をPiで確認
python3 main_control_loop.py --debug --speed 0.1
```

### 2. ハードウェアモード動作確認
```bash
# 実機モードでの動作確認（低速）
python3 main_control_loop.py --debug --speed 0.1
```

### 3. IMU 2段階キャリブレーション

#### Stage 1: BNO055内蔵キャリブレーション（自動）
```bash
# BNO055センサーレベル校正（システム起動時に自動実行）
python3 -c "
from bno055_imu_driver import BNO055IMUDriver
imu = BNO055IMUDriver()
print('BNO055 calibration status:', imu.get_calibration_status())
print('Is fully calibrated:', imu.is_calibrated())
"
```

#### Stage 2: 実走行コース環境補正（手動・3方式対応）

##### **方式1: ビジュアルマップキャリブレーション（最高精度・推奨）**
```bash
# ウェイポイントマップ表示付きキャリブレーション
python3 imu_visual_calibration.py

# デモ確認（初回時）
python3 visual_calibration_demo.py
```
- **特長**: コースマップを表示し、マウスクリックで特徴点選択
- **精度**: ±0.5°（マップ精度に依存）
- **操作**: GUI画面でインタラクティブに操作

##### **方式2: 特徴点ベースキャリブレーション（高精度）**
```bash
# コマンドライン版特徴点キャリブレーション
python3 imu_landmark_calibration.py
```
- **特長**: 物理的特徴点（建物角等）を基準に手動入力
- **精度**: ±1°（測定精度に依存）
- **操作**: テキスト入力で特徴点説明と角度指定

##### **方式3: 従来2点キャリブレーション（標準精度）**
```bash
# 0°/180°回転方式（フォールバック）
python3 imu_2point_calibration.py
```
- **特長**: スタート地点での180°回転
- **精度**: ±2-3°（人為誤差含む）
- **操作**: 車両を手動で180°回転

##### **キャリブレーション確認**
```bash
# 保存されたキャリブレーションファイル確認
ls -la *calib.json

# 統合システムでの最終確認
python3 -c "
from main_control_loop import get_calibrated_yaw
import math
yaw = get_calibrated_yaw()
print(f'Final calibrated yaw: {math.degrees(yaw):.2f}°')
"
```

#### キャリブレーション手順（推奨）
1. **室内でのBNO055校正**: センサーを8の字に回転（30秒）
2. **コース設置**: 車両を実際のスタート地点に配置
3. **方位基準設定**: 真っ直ぐ進む方向を0°として記録
4. **環境補正実行**: 2点キャリブレーションでコース誤差を補正

## 動作確認チェックポイント

### システム起動時
- [ ] プラットフォーム検出: "Running on: Linux (Raspberry Pi)"
- [ ] ハードウェアドライバ初期化成功
- [ ] 設定ファイル(config.json)読み込み成功
- [ ] Waypointデータ(quarify.json)読み込み成功

### センサー動作
- [ ] BNO055: キャリブレーション状態確認
- [ ] PCA9685: PWM出力テスト
- [ ] IMU: ヨー角読み取り正常

### モーター制御
- [ ] アクセル指令: 速度制限適用確認
- [ ] ステアリング指令: 正常動作確認
- [ ] 緊急停止: 即座に停止確認

## 安全確認事項

### テスト前
- [ ] 車両をジャッキアップまたは車輪を浮かせる
- [ ] 緊急停止スイッチ準備
- [ ] 十分な作業スペース確保

### テスト中
- [ ] 低速モード(--speed 0.1)から開始
- [ ] デバッグ出力でモーター指令値確認
- [ ] 異常動作時は即座にCtrl+C

### 実走行前
- [ ] 完全なハードウェア機能テスト完了
- [ ] IMUキャリブレーション完了
- [ ] 安全な屋外テストコース確保

## トラブルシューティング

### I2C機器が検出されない
```bash
# I2C有効化確認
sudo raspi-config

# I2C権限確認
sudo usermod -a -G i2c pi
```

### Python依存関係エラー
```bash
# システムライブラリ確認
sudo apt install python3-dev libffi-dev

# CircuitPython再インストール
pip3 uninstall adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-pca9685
```

### モーターが動作しない
```bash
# PCA9685 PWM周波数確認
python3 -c "
import board, busio, adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50
print('PWM frequency set to 50Hz')
"
```

## ログファイル確認
```bash
# システムログ
journalctl -u autonomous_car -f

# Python実行ログ
python3 main_control_loop.py --debug 2>&1 | tee test_log.txt
```

## 成功基準
- [ ] 全ハードウェア機器の正常認識
- [ ] モック環境と同等の制御ループ動作
- [ ] 速度制限・デバッグ機能の正常動作
- [ ] IMUキャリブレーションによる精確な姿勢制御
- [ ] 緊急停止・例外処理の確実な動作