@echo off
REM PC-Raspberry Pi IMU Debug Workflow
REM Usage: debug_helper.bat [command] [raspberry_ip]

set RASPBERRY_IP=%2
if "%RASPBERRY_IP%"=="" set RASPBERRY_IP=192.168.1.100

echo ========================================
echo   PC-RaspberryPi IMU Debug Helper
echo ========================================
echo Raspberry Pi IP: %RASPBERRY_IP%
echo.

if "%1"=="setup" goto setup
if "%1"=="debug" goto debug
if "%1"=="deploy" goto deploy
if "%1"=="remote" goto remote
if "%1"=="sync" goto sync
if "%1"=="monitor" goto monitor
goto help

:setup
echo 🔧 Setting up development environment...
python debug_workflow.py --mode setup
goto end

:debug
echo 🐛 Running IMU debug (PC mode)...
python imu_debug_simple.py
goto end

:deploy
echo 🚀 Deploying to Raspberry Pi...
python debug_workflow.py --mode deploy --raspberry-ip %RASPBERRY_IP%
goto end

:remote
echo 🔗 Starting remote debug session...
python debug_workflow.py --mode remote --raspberry-ip %RASPBERRY_IP%
goto end

:sync
echo 📥 Syncing logs from Raspberry Pi...
python debug_workflow.py --mode sync --raspberry-ip %RASPBERRY_IP%
goto end

:monitor
echo 👁️ Monitoring Raspberry Pi...
python debug_workflow.py --mode monitor --raspberry-ip %RASPBERRY_IP%
goto end

:help
echo.
echo Available commands:
echo   setup   - Setup development environment
echo   debug   - Run IMU debug on PC (mock mode)
echo   deploy  - Deploy files to Raspberry Pi
echo   remote  - Start remote debug session
echo   sync    - Sync log files from Raspberry Pi
echo   monitor - Monitor Raspberry Pi connection
echo.
echo Usage: debug_helper.bat [command] [raspberry_ip]
echo Example: debug_helper.bat deploy 192.168.1.100
echo.

:end
pause