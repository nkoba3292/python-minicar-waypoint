#!/bin/bash
# GPIO-WiFi競合診断スクリプト

echo "=========================================="
echo "GPIO-WiFi Conflict Diagnostic Tool"
echo "=========================================="
echo ""

# 1. カーネルモジュール確認
echo "=== Loaded Kernel Modules ==="
lsmod | grep -E "brcm|gpio|i2c"
echo ""

# 2. GPIO使用状況確認
echo "=== GPIO Pin Usage ==="
if [ -f /sys/kernel/debug/gpio ]; then
    sudo cat /sys/kernel/debug/gpio | grep -E "gpio-15|gpio-26|gpio-13|gpio-24|gpio-35|gpio-37|gpio-32|gpio-31|gpio-36|gpio-38"
else
    echo "GPIO debug not available"
fi
echo ""

# 3. I2C使用状況確認
echo "=== I2C Device Status ==="
sudo i2cdetect -y 1
echo ""

# 4. WiFiドライバ情報
echo "=== WiFi Driver Info ==="
lsmod | grep brcmfmac
modinfo brcmfmac | grep -E "version|description"
echo ""

# 5. 割り込み情報
echo "=== Interrupt Usage ==="
cat /proc/interrupts | grep -E "gpio|brcm|i2c"
echo ""

# 6. デバイスツリー確認
echo "=== Device Tree Overlays ==="
dtoverlay -l
echo ""

# 7. カーネルログ（直近50行）
echo "=== Recent Kernel Messages ==="
dmesg | tail -50
echo ""

echo "=========================================="
echo "Diagnostic complete. Check for conflicts above."
echo "=========================================="
