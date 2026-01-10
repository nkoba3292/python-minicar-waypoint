#!/bin/bash
# WiFiメモリ消費量調査スクリプト

echo "=========================================="
echo "WiFi Memory Usage Analysis"
echo "=========================================="
echo ""

# 1. システム全体のメモリ使用状況
echo "=== System Memory Overview ==="
free -h
echo ""

# 2. WiFi関連プロセスのメモリ使用量
echo "=== WiFi Related Processes Memory ==="
echo "Process                PID     VSZ    RSS   %MEM  %CPU"
echo "-----------------------------------------------------"
ps aux | grep -E "wpa_supplicant|dhcpcd|NetworkManager" | grep -v grep | awk '{printf "%-20s %6s %6s %6s %5s %5s\n", $11, $2, $5, $6, $4, $3}'
echo ""

# 3. カーネルモジュールのメモリ使用量
echo "=== WiFi Kernel Module Memory ==="
if [ -f /proc/modules ]; then
    echo "Module          Size  Used_by"
    echo "--------------------------------"
    cat /proc/modules | grep -E "brcm|cfg80211|mac80211" | awk '{printf "%-15s %6s %s\n", $1, $2, $4}'
fi
echo ""

# 4. ネットワークバッファのメモリ使用量
echo "=== Network Buffer Memory ==="
cat /proc/net/sockstat
echo ""

# 5. slab情報（カーネルメモリアロケータ）
echo "=== Kernel Slab (WiFi related) ==="
if [ -f /proc/slabinfo ]; then
    sudo cat /proc/slabinfo | head -2
    sudo cat /proc/slabinfo | grep -E "sock|skb|tcp|udp" | head -10
fi
echo ""

# 6. WiFiドライバの詳細情報
echo "=== WiFi Driver Details ==="
lsmod | grep brcmfmac | awk '{print "Module: "$1", Size: "$2" bytes, Used by: "$3}'
echo ""

# 7. メモリマップ（WiFi関連）
echo "=== Memory Map Summary ==="
cat /proc/meminfo | grep -E "MemTotal|MemFree|MemAvailable|Buffers|Cached|Slab|SReclaimable|SUnreclaim"
echo ""

# 8. WiFi省電力モードとメモリの関係
echo "=== WiFi Power Management ==="
iwconfig wlan0 2>/dev/null | grep "Power Management"
iw dev wlan0 get power_save 2>/dev/null
echo ""

# 9. 比較: WiFi停止前後のメモリ
echo "=== Memory Test: WiFi ON vs OFF ==="
echo "Current memory (WiFi ON):"
free -m | grep Mem | awk '{print "  Used: "$3" MB, Free: "$4" MB, Available: "$7" MB"}'

echo ""
echo "To test WiFi OFF memory, run:"
echo "  sudo ifconfig wlan0 down"
echo "  free -m"
echo "  sudo ifconfig wlan0 up"

echo ""
echo "=========================================="
echo "Analysis Complete"
echo "=========================================="
