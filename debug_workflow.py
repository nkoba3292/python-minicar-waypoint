# debug_workflow.py
# -*- coding: utf-8 -*-
"""
PC（VS Code）⇔ ラズパイ 効率的デバッグワークフロー
"""

import os
import sys
import json
import subprocess
import socket
import time
from datetime import datetime
import argparse

class DebugWorkflow:
    """PC-ラズパイ間の効率的デバッグ管理"""
    
    def __init__(self):
        self.config = self.load_config()
        self.is_raspberry_pi = self.detect_platform()
        self.debug_mode = self.config.get('system', {}).get('debug_mode', True)
        
    def detect_platform(self):
        """プラットフォーム自動検出"""
        try:
            with open('/proc/cpuinfo', 'r') as f:
                if 'Raspberry Pi' in f.read():
                    return True
        except FileNotFoundError:
            pass
        return False
    
    def load_config(self):
        """設定ファイル読み込み"""
        try:
            with open('config.json', 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print("config.json not found")
            return {}
    
    def setup_environment(self):
        """環境別セットアップ"""
        if self.is_raspberry_pi:
            print("🍓 Raspberry Pi detected - Setting up hardware mode")
            self.setup_raspberry_pi()
        else:
            print("💻 PC detected - Setting up development mode")
            self.setup_pc_environment()
    
    def setup_raspberry_pi(self):
        """ラズパイ環境セットアップ"""
        # I2C有効化確認
        i2c_check = subprocess.run(['lsmod'], capture_output=True, text=True)
        if 'i2c_dev' not in i2c_check.stdout:
            print("⚠️ I2C not enabled. Run: sudo raspi-config")
        
        # GPIO権限確認
        if os.getuid() != 0:
            print("⚠️ Running without sudo. Some features may not work.")
        
        print("✅ Raspberry Pi environment ready")
    
    def setup_pc_environment(self):
        """PC開発環境セットアップ"""
        print("🔧 Setting up PC development environment")
        
        # モックモード設定
        os.environ['MOCK_SENSORS'] = 'True'
        os.environ['DEBUG_MODE'] = 'True'
        os.environ['SIMULATION_MODE'] = 'True'
        
        print("✅ PC development environment ready")
    
    def run_imu_debug(self, mode='auto'):
        """IMUデバッグ実行"""
        if self.is_raspberry_pi:
            # 実機モード
            debug_script = 'imu_debug_ultimate.py'
            print(f"🍓 Running on Raspberry Pi: {debug_script}")
        else:
            # PC開発モード
            if mode == 'mock':
                debug_script = 'imu_debug_simple.py'
            else:
                debug_script = 'imu_debug_ultimate.py'
            print(f"💻 Running on PC: {debug_script}")
        
        try:
            subprocess.run([sys.executable, debug_script], check=True)
        except subprocess.CalledProcessError as e:
            print(f"❌ Error running {debug_script}: {e}")
    
    def deploy_to_raspberry_pi(self, raspberry_ip, files=None):
        """ラズパイへのデプロイ"""
        if files is None:
            files = [
                'main_control_loop.py',
                'imu_debug_ultimate.py',
                'imu_debug_bno055.py',
                'imu_debug_simple.py',
                'config.json',
                'vehicle_interface.py',
                'raspberry_pi_uart_setup.sh'
            ]
        
        print(f"🚀 Deploying to Raspberry Pi: {raspberry_ip}")
        
        for file in files:
            if os.path.exists(file):
                cmd = f"scp {file} pi@{raspberry_ip}:~/minicar/"
                try:
                    subprocess.run(cmd.split(), check=True)
                    print(f"✅ Deployed: {file}")
                except subprocess.CalledProcessError:
                    print(f"❌ Failed to deploy: {file}")
    
    def remote_debug_session(self, raspberry_ip):
        """リモートデバッグセッション開始"""
        print(f"🔗 Starting remote debug session with {raspberry_ip}")
        
        # SSH接続でリモートデバッグ開始
        ssh_cmd = f"ssh -t pi@{raspberry_ip} 'cd ~/minicar && python3 imu_debug_ultimate.py'"
        
        try:
            subprocess.run(ssh_cmd, shell=True)
        except KeyboardInterrupt:
            print("\n🛑 Remote debug session ended")
    
    def sync_logs(self, raspberry_ip, local_log_dir='logs'):
        """ログファイル同期"""
        if not os.path.exists(local_log_dir):
            os.makedirs(local_log_dir)
        
        # ラズパイからログを取得
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sync_cmd = f"rsync -av pi@{raspberry_ip}:~/minicar/*.csv {local_log_dir}/raspi_{timestamp}/"
        
        try:
            subprocess.run(sync_cmd.split(), check=True)
            print(f"✅ Logs synced to: {local_log_dir}/raspi_{timestamp}/")
        except subprocess.CalledProcessError:
            print("❌ Failed to sync logs")
    
    def monitor_raspberry_pi(self, raspberry_ip):
        """ラズパイ監視"""
        print(f"👁️ Monitoring Raspberry Pi: {raspberry_ip}")
        
        while True:
            try:
                # ping確認
                result = subprocess.run(['ping', '-c', '1', raspberry_ip], 
                                      capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0:
                    print(f"✅ {datetime.now().strftime('%H:%M:%S')} - Raspberry Pi online")
                else:
                    print(f"❌ {datetime.now().strftime('%H:%M:%S')} - Raspberry Pi offline")
                
                time.sleep(10)
                
            except KeyboardInterrupt:
                print("\n🛑 Monitoring stopped")
                break
            except subprocess.TimeoutExpired:
                print(f"⚠️ {datetime.now().strftime('%H:%M:%S')} - Ping timeout")


def main():
    parser = argparse.ArgumentParser(description='IMU Debug Workflow Manager')
    parser.add_argument('--mode', choices=['setup', 'debug', 'deploy', 'remote', 'sync', 'monitor'], 
                       default='setup', help='Operation mode')
    parser.add_argument('--raspberry-ip', default='192.168.1.100', 
                       help='Raspberry Pi IP address')
    parser.add_argument('--debug-type', choices=['auto', 'mock', 'hardware'], 
                       default='auto', help='Debug type')
    
    args = parser.parse_args()
    
    workflow = DebugWorkflow()
    
    if args.mode == 'setup':
        workflow.setup_environment()
    
    elif args.mode == 'debug':
        workflow.run_imu_debug(args.debug_type)
    
    elif args.mode == 'deploy':
        workflow.deploy_to_raspberry_pi(args.raspberry_ip)
    
    elif args.mode == 'remote':
        workflow.remote_debug_session(args.raspberry_ip)
    
    elif args.mode == 'sync':
        workflow.sync_logs(args.raspberry_ip)
    
    elif args.mode == 'monitor':
        workflow.monitor_raspberry_pi(args.raspberry_ip)


if __name__ == "__main__":
    main()