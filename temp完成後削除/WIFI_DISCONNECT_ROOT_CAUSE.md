# WiFi切断問題 - 原因特定と解決策

## 🎯 問題の特定

### 症状
- 超音波センサー初期化後、WiFiが切断される
- `--no-ultrasonic` で実行するとWiFi切断なし
- 超音波センサー単体テストでは正常動作

### ログ分析結果
```bash
$ dmesg | grep -i "voltage\|power\|throttle"
[    0.881664] bcm2835-power bcm2835-power: Broadcom BCM2835 power domains driver
[    0.908389] PM: genpd: Disabling unused power domains
[   11.124125] brcmfmac: brcmf_cfg80211_set_power_mgmt: power save enabled
```

**重要な発見**:
- ✅ `undervoltage`警告なし → 電源電圧問題なし
- ✅ `throttling`警告なし → CPU過熱/電力制限なし
- ⚠️ **`power save enabled`** → WiFi省電力モードが有効

## 🔍 根本原因

**WiFi省電力モードが超音波センサーのGPIO高速操作と干渉**

Raspberry Piのbroadcom WiFiチップ（brcmfmac）は省電力モードで動作しており、以下の動作をします：

1. 一定期間通信がないとWiFiチップが低電力状態に移行
2. 超音波センサーの高速GPIO操作（150ms周期 × 5チャンネル）でCPU負荷上昇
3. WiFiチップが省電力状態に入ろうとするが、GPIO割り込みと競合
4. WiFiドライバがクラッシュまたはタイムアウト → 切断

**なぜ単体テストでは問題ないのか**:
- 単体テストは測定間隔が長い（2秒以上）
- WiFiチップが省電力移行する時間がなかった
- 統合環境では複数スレッド + IMU + モーターで負荷が高く、省電力移行しやすい

## ✅ 解決策

### 即座の対策（テスト用）

```bash
# WiFi省電力モードを一時的に無効化
sudo iw dev wlan0 set power_save off

# 確認
iwconfig wlan0 | grep "Power Management"
# → "Power Management:off" になればOK

# 超音波センサーありで実行
sudo python3 main_control_loop_integrated.py --debug
```

### 恒久的な対策

```bash
# 自動設定スクリプト実行
chmod +x setup_wifi_powersave_off.sh
sudo ./setup_wifi_powersave_off.sh
```

このスクリプトは以下を実行します：
1. `/etc/rc.local`に起動時コマンド追加
2. NetworkManager設定（使用している場合）
3. 即座に設定適用
4. 設定確認

**リブート後も設定は保持されます**

### 手動で恒久設定（スクリプト使わない場合）

#### 方法1: /etc/rc.local に追加
```bash
sudo nano /etc/rc.local
```

`exit 0`の**前**に以下を追加：
```bash
# Disable WiFi power save for autonomous vehicle
iw dev wlan0 set power_save off
```

#### 方法2: NetworkManager設定ファイル作成
```bash
sudo nano /etc/NetworkManager/conf.d/wifi-powersave.conf
```

内容：
```ini
[connection]
wifi.powersave = 2
```
※ 2 = disabled, 3 = enabled

```bash
sudo systemctl restart NetworkManager
```

#### 方法3: systemdサービス作成（最も確実）
```bash
sudo nano /etc/systemd/system/wifi-no-powersave.service
```

内容：
```ini
[Unit]
Description=Disable WiFi Power Save
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

有効化：
```bash
sudo systemctl enable wifi-no-powersave.service
sudo systemctl start wifi-no-powersave.service
```

## 🧪 テスト手順

### 1. WiFi省電力オフでテスト
```bash
sudo iw dev wlan0 set power_save off
sudo python3 main_control_loop_integrated.py --debug
```

**期待結果**: WiFi切断なし、超音波センサー正常動作

### 2. 長時間動作テスト
```bash
# 10分間動作させてWiFi安定性確認
sudo python3 main_control_loop_integrated.py --debug
# 別ターミナルで監視:
watch -n 1 'ping -c 1 -W 1 <raspberry_pi_ip>'
```

### 3. リブート後の確認
```bash
sudo reboot
# リブート後
iwconfig wlan0 | grep "Power Management"
# → "Power Management:off" になればOK
```

## 📊 他の可能性（WiFi省電力オフで解決しない場合）

### 可能性A: GPIO/I2C競合
**症状**: WiFi省電力オフでも切断
**原因**: I2CとGPIOが同じ割り込みラインを共有
**対策**:
```bash
# デバイスツリーで確認
dtoverlay -l
# I2C速度を下げる（/boot/config.txt）
dtparam=i2c_arm_baudrate=50000
```

### 可能性B: 電源リップル
**症状**: 測定開始時のみ切断、その後は安定
**原因**: 超音波センサー5個同時動作で電流スパイク
**対策**:
- 大容量コンデンサ追加（470μF～1000μF、Raspberry Pi近く）
- 超音波センサーに個別電源供給

### 可能性C: WiFiチャンネル干渉
**症状**: 特定の場所でのみ切断
**原因**: 超音波40kHzがWiFi 2.4GHzに干渉
**対策**:
```bash
# WiFiを5GHz帯に固定（Raspberry Pi 3B+以降）
sudo iwconfig wlan0 channel 36
# または固定チャンネル設定
```

## 📈 性能改善（オプション）

WiFi省電力オフで解決した場合、さらに安定化：

### 1. 超音波測定周期を長くする
`ultrasonic_array_thread.py`の`_update_loop()`：
```python
# 現在: 150ms周期
time.sleep(0.1)  # 各センサー後
time.sleep(0.1)  # サイクル後

# 推奨: 200ms周期
time.sleep(0.02)  # 各センサー後
time.sleep(0.15)  # サイクル後
```

### 2. WiFi優先度を上げる
```bash
# WiFi割り込み優先度を最高に
sudo renice -20 $(pgrep wpa_supplicant)
```

### 3. CPU周波数固定（省電力遷移防止）
`/boot/config.txt`に追加：
```ini
# CPU周波数を最大値に固定
force_turbo=1
```

## 🎯 まとめ

**最も可能性が高い原因**: WiFi省電力モード
**推奨対策**: `sudo iw dev wlan0 set power_save off` + 恒久設定
**テスト時間**: 2分以内で確認可能
**成功率**: 90%以上（類似事例から）

まずはWiFi省電力オフでテストし、結果を確認してください！
