#!/bin/bash
# WiFi省電力モード恒久無効化スクリプト

echo "Setting up WiFi power management disable..."

# 1. /etc/rc.local に追加（起動時に自動実行）
if ! grep -q "iw dev wlan0 set power_save off" /etc/rc.local 2>/dev/null; then
    echo "Adding power_save off to /etc/rc.local..."
    sudo sed -i '/^exit 0/i # Disable WiFi power save for autonomous vehicle\niw dev wlan0 set power_save off\n' /etc/rc.local
    echo "[OK] Added to /etc/rc.local"
else
    echo "[OK] Already configured in /etc/rc.local"
fi

# 2. NetworkManager設定（使用している場合）
if [ -d "/etc/NetworkManager" ]; then
    echo "Configuring NetworkManager..."
    sudo tee /etc/NetworkManager/conf.d/wifi-powersave.conf > /dev/null <<EOF
[connection]
wifi.powersave = 2
EOF
    echo "[OK] NetworkManager configured (powersave=2 means disabled)"
    sudo systemctl restart NetworkManager 2>/dev/null || true
fi

# 3. 即座に適用
echo "Applying settings immediately..."
sudo iw dev wlan0 set power_save off

# 4. 確認
echo ""
echo "Current settings:"
iwconfig wlan0 | grep "Power Management"

echo ""
echo "=========================================="
echo "WiFi Power Save Disabled Successfully"
echo "=========================================="
echo ""
echo "Settings will persist after reboot."
echo "Test autonomous vehicle operation now!"
