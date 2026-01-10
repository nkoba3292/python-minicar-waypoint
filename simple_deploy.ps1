# simple_deploy.ps1
# 簡単デプロイスクリプト

param(
    [string]$RaspberryIP = "172.20.10.3"
)

Write-Host "🚀 Raspberry Pi への簡単デプロイ開始" -ForegroundColor Green
Write-Host "IP: $RaspberryIP" -ForegroundColor Cyan

# 転送するファイル一覧
$files = @(
    "imu_debug_ultimate.py",
    "imu_debug_simple.py",
    "imu_debug_bno055.py",
    "config.json",
    "raspberry_pi_uart_setup.sh"
)

Write-Host "📦 転送ファイル:" -ForegroundColor Yellow
foreach ($file in $files) {
    if (Test-Path $file) {
        Write-Host "  ✅ $file" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $file (見つからない)" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "📋 手動転送手順:" -ForegroundColor Yellow
Write-Host "1. WinSCP、FileZilla、または他のSFTPクライアントを使用"
Write-Host "2. 接続先: pi@$RaspberryIP"
Write-Host "3. 転送先: /home/pi/minicar/"
Write-Host ""

Write-Host "🔧 またはPuTTY/SSH経由で以下を実行:" -ForegroundColor Cyan
Write-Host "mkdir -p ~/minicar"
Write-Host ""

Write-Host "🎯 転送後の実行手順:" -ForegroundColor Green
Write-Host "cd ~/minicar"
Write-Host "python3 imu_debug_simple.py"
Write-Host ""

# SSH接続テスト用のコマンド生成
Write-Host "💡 SSH接続テスト:" -ForegroundColor Magenta
Write-Host "ssh pi@$RaspberryIP"