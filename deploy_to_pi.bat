@echo off
REM deploy_to_pi.bat - Windows用Raspberry Pi 4Bデプロイスクリプト

REM 設定
set PI_USER=pi
set PI_HOST=raspberrypi.local
set PI_DEST=/home/pi/autonomous_car
set LOCAL_SRC=.

echo === Raspberry Pi 4B Deployment Script (Windows) ===
echo Target: %PI_USER%@%PI_HOST%:%PI_DEST%

REM SSH接続確認
echo Testing SSH connection...
ssh -o ConnectTimeout=5 %PI_USER%@%PI_HOST% "echo SSH OK"
if errorlevel 1 (
    echo Error: Cannot connect to Raspberry Pi
    echo Make sure:
    echo 1. Raspberry Pi is powered on and connected
    echo 2. SSH is enabled on Raspberry Pi
    echo 3. Hostname/IP address is correct
    echo 4. SSH client is installed on Windows
    pause
    exit /b 1
)

REM ディレクトリ作成
echo Creating destination directory...
ssh %PI_USER%@%PI_HOST% "mkdir -p %PI_DEST%"

REM ファイル転送（SCP使用）
echo Copying files...
scp -r *.py *.json *.sh %PI_USER%@%PI_HOST%:%PI_DEST%/

REM Python依存関係のインストール
echo Installing Python dependencies...
ssh %PI_USER%@%PI_HOST% "cd %PI_DEST% && pip3 install -r requirements.txt"

REM 実行権限設定
echo Setting executable permissions...
ssh %PI_USER%@%PI_HOST% "cd %PI_DEST% && chmod +x *.sh"

echo === Deployment Complete ===
echo To run the system:
echo ssh %PI_USER%@%PI_HOST%
echo cd %PI_DEST%
echo python3 main_control_loop.py

pause