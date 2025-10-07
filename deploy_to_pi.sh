#!/bin/bash
# deploy_to_pi.sh - Raspberry Pi 4Bへのデプロイメントスクリプト

# 設定
PI_USER="pi"
PI_HOST="raspberrypi.local"  # または IPアドレス
PI_DEST="/home/pi/autonomous_car"
LOCAL_SRC="."

echo "=== Raspberry Pi 4B Deployment Script ==="
echo "Target: $PI_USER@$PI_HOST:$PI_DEST"

# Raspberry Piへの接続確認
echo "Testing SSH connection..."
if ! ssh -o ConnectTimeout=5 $PI_USER@$PI_HOST "echo 'SSH OK'"; then
    echo "Error: Cannot connect to Raspberry Pi"
    echo "Make sure:"
    echo "1. Raspberry Pi is powered on and connected"
    echo "2. SSH is enabled on Raspberry Pi"
    echo "3. Hostname/IP address is correct"
    exit 1
fi

# ディレクトリ作成
echo "Creating destination directory..."
ssh $PI_USER@$PI_HOST "mkdir -p $PI_DEST"

# ファイル転送
echo "Copying files..."
rsync -avz --progress \
    --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.pytest_cache' \
    $LOCAL_SRC/ $PI_USER@$PI_HOST:$PI_DEST/

# Python依存関係のインストール
echo "Installing Python dependencies..."
ssh $PI_USER@$PI_HOST "cd $PI_DEST && pip3 install -r requirements.txt"

# 実行権限設定
echo "Setting executable permissions..."
ssh $PI_USER@$PI_HOST "cd $PI_DEST && chmod +x *.sh"

# システムサービス登録（オプション）
echo "Would you like to install as a system service? (y/n)"
read -r install_service
if [[ $install_service == "y" || $install_service == "Y" ]]; then
    echo "Installing system service..."
    ssh $PI_USER@$PI_HOST "cd $PI_DEST && sudo cp autonomous_car.service /etc/systemd/system/"
    ssh $PI_USER@$PI_HOST "sudo systemctl daemon-reload"
    ssh $PI_USER@$PI_HOST "sudo systemctl enable autonomous_car.service"
    echo "Service installed. Start with: sudo systemctl start autonomous_car"
fi

echo "=== Deployment Complete ==="
echo "To run the system:"
echo "ssh $PI_USER@$PI_HOST"
echo "cd $PI_DEST"
echo "python3 main_control_loop.py"