# WiFi切断問題のトラブルシューティングガイド

## 問題の症状
超音波センサー初期化後、すぐにWiFi接続が切断される。

## 考えられる原因

### 1. GPIOカーネルドライバの問題
- 症状: GPIO操作時にカーネルパニックまたはドライバクラッシュ
- 診断: `dmesg | tail -50` でカーネルログを確認
- 対策: カーネルモジュールの再ロード

### 2. 電源不足
- 症状: 超音波センサー5個 + IMU + モーター + WiFiで電流不足
- 診断: `vcgencmd get_throttled` で電圧低下を確認
- 対策: 
  - より強力な電源アダプタ（5V 3A以上）
  - 超音波センサーの個別電源供給

### 3. CPU/メモリ枯渇
- 症状: システムリソース不足でWiFiドライバが停止
- 診断: `top` または `htop` でリソース使用率を確認
- 対策: 不要なサービスを停止

### 4. GPIO競合
- 症状: I2C（IMU/モーター）とGPIO（超音波）の競合
- 診断: `gpio readall` で現在のピン状態を確認
- 対策: ピン配置の見直し

### 5. タイミング問題
- 症状: 連続的なGPIO操作でシステム応答不能
- 診断: 測定周期が速すぎる
- 対策: 測定間隔を延長（既に実装済み）

## 診断手順

### ステップ1: 超音波センサーなしでテスト
```bash
sudo python3 main_control_loop_integrated.py --no-ultrasonic --debug
```
**期待結果**: WiFi接続が維持される
- ✅ 維持される → 超音波センサーが原因
- ❌ 切断される → 他のモジュール（IMU/モーター）が原因

### ステップ2: システムリソース確認
```bash
# メモリ確認
free -h

# CPU確認
top -n 1

# 電圧確認
vcgencmd measure_volts
vcgencmd get_throttled

# カーネルログ確認
dmesg | tail -50
```

### ステップ3: GPIO状態確認
```bash
# GPIO使用状況
gpio readall

# I2Cデバイス確認
i2cdetect -y 1
```

### ステップ4: 段階的テスト
```bash
# 1. IMUのみ
sudo python3 -c "from IMU_sensor_bno055 import IMUSensorBNO055; imu = IMUSensorBNO055()"

# 2. モーターのみ
sudo python3 -c "from pca9685_motor_driver import PCA9685MotorDriver; motor = PCA9685MotorDriver()"

# 3. 超音波のみ
sudo python3 ultrasonic_array_thread.py
```

## 推奨される対策

### 即効性のある対策
1. **超音波センサーを無効化して動作確認**
   ```bash
   sudo python3 main_control_loop_integrated.py --no-ultrasonic --debug
   ```

2. **電源を強化**
   - 5V 3A以上のACアダプタを使用
   - USB-Cケーブルの品質確認

3. **不要なサービスを停止**
   ```bash
   sudo systemctl stop bluetooth
   sudo systemctl stop avahi-daemon
   ```

### 根本的な対策
1. **超音波センサーの個別電源**
   - 5V外部電源を超音波センサーに供給
   - GNDは共通接続を維持

2. **GPIO使用を最小化**
   - 超音波センサー数を減らす（5個→3個）
   - 測定頻度を下げる（150ms→300ms）

3. **I2C超音波センサーに変更**
   - GPIOの代わりにI2Cバス使用
   - システム負荷が大幅に軽減

4. **センサーマルチプレクサ使用**
   - TCA9548A等のI2Cマルチプレクサ
   - 1つのGPIOで複数センサーを制御

## 緊急回避策

### WiFi切断を回避する設定
```bash
# WiFi省電力モードを無効化
sudo iw dev wlan0 set power_save off

# WiFiドライバの安定化
sudo nano /etc/modprobe.d/8192cu.conf
# 追加: options 8192cu rtw_power_mgnt=0 rtw_enusbss=0

# カーネルパラメータ調整
sudo nano /boot/cmdline.txt
# 追加: dwc_otg.speed=1 (USB1.1モードで動作)
```

### 再起動不要の復旧
```bash
# WiFiインターフェイスの再起動
sudo ip link set wlan0 down
sudo ip link set wlan0 up
sudo dhclient wlan0
```

## 現在の実装状況

### 実装済みの対策
- ✅ GPIO初期化時の待機時間追加（10ms）
- ✅ 測定周期の延長（100ms→150ms）
- ✅ センサー間待機の追加（10ms）
- ✅ 例外処理の強化
- ✅ エラー時の安全な戻り値（200cm）
- ✅ --no-ultrasonic オプション追加

### 未実装の対策
- ⏳ 電源強化
- ⏳ センサー数削減
- ⏳ I2C超音波センサーへの変更
- ⏳ WiFi省電力モード無効化

## 連絡先情報
問題が解決しない場合は、以下の情報を収集してください：
```bash
# システム情報
uname -a
cat /proc/cpuinfo | grep Model
vcgencmd measure_temp
vcgencmd measure_volts
vcgencmd get_throttled

# ログ情報
dmesg | grep -i "wifi\|wlan\|gpio\|usb" > wifi_debug.log
journalctl -u wpa_supplicant -n 100 >> wifi_debug.log
```
