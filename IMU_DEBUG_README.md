# IMU_DEBUG_README.md

# BNO055 IMU センサーデバッグツール

ラズベリーパイ4B + BNO055センサーのIMUデータをリアルタイム監視・デバッグするためのPythonツール群です。

## 📁 ファイル構成

```
📂 IMU Debug Tools
├── 📄 imu_debug_bno055.py      # 完全版：実際のBNO055センサー用
├── 📄 imu_debug_simple.py      # 簡易版：Windows/Mockモード対応
├── 📄 setup_imu_debug.py       # 環境セットアップスクリプト
└── 📄 IMU_DEBUG_README.md      # このファイル
```

## 🚀 クイックスタート

### 1. 環境セットアップ
```bash
# 必要なライブラリをインストール
python setup_imu_debug.py

# または手動でインストール
pip install pyserial numpy matplotlib
```

### 2. 実行方法

#### Windows環境でのテスト実行（模擬データ）
```bash
python imu_debug_simple.py
```

#### ラズパイ実環境での実行
```bash
python imu_debug_bno055.py
```

## 🔧 ハードウェア接続

### BNO055 → Raspberry Pi 4B

#### I2C接続（推奨）
```
BNO055     →  ラズパイ4B
VCC (3.3V) →  Pin 1 (3.3V)
GND        →  Pin 6 (GND)
SDA        →  Pin 3 (GPIO 2)
SCL        →  Pin 5 (GPIO 3)
PS0        →  GND (I2Cモード)
PS1        →  3.3V (I2Cモード)
```

#### UART接続（シリアル）
```
BNO055     →  ラズパイ4B
VCC (3.3V) →  Pin 1 (3.3V)
GND        →  Pin 6 (GND)
TX         →  Pin 10 (GPIO 15)
RX         →  Pin 8  (GPIO 14)
PS0        →  3.3V (UARTモード)
PS1        →  GND  (UARTモード)
```

## ⚙️ ラズパイUART設定

### シリアル有効化
```bash
sudo raspi-config
# Interface Options → Serial Port
# Login shell: No
# Serial interface: Yes
```

### /boot/config.txt に追加
```
enable_uart=1
dtoverlay=disable-bt
```

### 再起動
```bash
sudo reboot
```

## 📊 機能一覧

### 🧭 imu_debug_bno055.py（完全版）

**主要機能:**
- ✅ BNO055との安定したシリアル通信
- 📊 生センサーデータ取得（加速度・ジャイロ・磁気）
- 🔄 センサーフュージョン結果（姿勢・オイラー角・クォータニオン）
- 📈 リアルタイム監視表示
- 📝 CSVデータロギング
- 🎯 キャリブレーション状態監視

**表示モード:**
- **コンパクト**: 1行リアルタイム表示
- **詳細**: 全データ画面表示

**操作キー:**
- `c`: コンパクトモード
- `d`: 詳細モード  
- `l`: ログオン/オフ
- `q`: 終了

### 🔧 imu_debug_simple.py（簡易版）

**特徴:**
- 🪟 Windows環境対応
- 🎭 MOCKモード（模擬データ）
- 📱 簡易操作
- 💾 基本ロギング機能

**MOCKモード:**
- 実際のセンサーなしでインターフェースをテスト
- リアルな模擬データ生成
- キャリブレーション状態シミュレーション

## 📈 出力データ例

### コンパクト表示
```
🧭 [  42] T:  12.3s | YAW:  123.4° | PITCH: +5.2° | ROLL: -2.1° | ACC:+9.78 | CAL:✅✅🟠🟡 | TEMP:26.5°C
```

### 詳細表示
```
🧭 BNO055 IMU SENSOR DEBUG MONITOR
========================================
Time: 15.7s | Data Count: 157 | Temp: 26.8°C
========================================
🎯 CALIBRATION:
   Sys:3/3 (Excellent)  Gyro:3/3  Acc:2/3  Mag:1/3

🔄 ORIENTATION:
   Roll:   -2.3°   Pitch:   +5.1°   Yaw: +123.7°

📊 ACCELERATION [m/s²]:
   X:  +0.15   Y:  -0.23   Z:  +9.78

🌀 GYROSCOPE [rad/s]:
   X:  +0.002   Y:  -0.001   Z:  +0.000
```

### CSVログファイル
```csv
timestamp,count,euler_roll,euler_pitch,euler_yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,calib_sys,calib_gyro,calib_acc,calib_mag,temperature
1696745231.123,1,-2.34,5.12,123.45,0.15,-0.23,9.78,0.002,-0.001,0.000,3,3,2,1,26.8
```

## 🛠️ トラブルシューティング

### 🔌 接続エラー
```
❌ Failed to connect to BNO055 sensor
```
**解決策:**
1. シリアルポート確認（`/dev/serial0`, `COM3`等）
2. BNO055電源供給確認
3. UART設定確認
4. ポート権限確認: `sudo chmod 666 /dev/serial0`

### 📡 データ取得エラー
```
❌ Register read error
```
**解決策:**
1. ボーレート確認（通常115200）
2. 配線確認（TX↔RX, 3.3V電源）
3. PS0/PS1ピン設定確認

### 🎯 キャリブレーション不良
```
⚠️ Calibration validation failed
```
**解決策:**
1. センサーを8字に動かす
2. 平面で回転させる
3. 磁気干渉源から離す
4. 金属から離す

## 📝 ログファイル

### 自動生成ファイル
- `imu_data_YYYYMMDD_HHMMSS.csv`: タイムスタンプ付きログ
- データは10Hz（0.1秒間隔）で記録
- Excel/MATLABで解析可能

### ログ項目
- **タイムスタンプ**: 高精度時刻
- **生データ**: 加速度・ジャイロ・磁気センサー
- **フュージョン**: オイラー角・クォータニオン
- **キャリブレーション**: 各センサーの精度状態
- **システム**: 温度・データカウント

## 🎯 活用例

### 🏁 自律走行車での使用
```python
# メインシステムでIMUデータを活用
sensor = BNO055Sensor()
sensor.connect()

while racing:
    sensor.update_sensor_data()
    data = sensor.get_sensor_data()
    
    # 現在の姿勢を取得
    yaw = data['fusion']['euler']['yaw']
    
    # 車両制御に活用
    steering_correction = calculate_steering(target_yaw, yaw)
```

### 📊 データ解析
```python
import pandas as pd
import matplotlib.pyplot as plt

# ログファイル読み込み
df = pd.read_csv('imu_data_20231007_143022.csv')

# ヨー角の時系列プロット
plt.plot(df['timestamp'], df['euler_yaw'])
plt.xlabel('Time [s]')
plt.ylabel('Yaw [°]')
plt.title('IMU Yaw Angle Over Time')
plt.show()
```

## 🔗 関連リンク

- [BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [PySerial Documentation](https://pyserial.readthedocs.io/)

## 📄 ライセンス

MIT License - 自由に使用・改変可能

---
**作成日**: 2025年10月7日  
**対応環境**: Raspberry Pi 4B, Windows 10/11  
**センサー**: BNO055 9軸IMU  
**言語**: Python 3.7+