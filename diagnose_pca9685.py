#!/usr/bin/env python3
"""
PCA9685 (モータードライバ) 診断スクリプト

I2Cデバイスの接続確認とトラブルシューティング
"""

import subprocess
import time

def check_i2c_devices():
    """I2Cデバイスをスキャン"""
    print("=" * 60)
    print("I2Cデバイススキャン")
    print("=" * 60)
    print()
    
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True, timeout=5)
        print(result.stdout)
        
        if '40' in result.stdout:
            print("✓ PCA9685 (0x40) が検出されました")
            return True
        else:
            print("⚠ PCA9685 (0x40) が検出されません")
            return False
            
    except subprocess.TimeoutExpired:
        print("⚠ i2cdetect がタイムアウトしました")
        return False
    except FileNotFoundError:
        print("⚠ i2cdetect コマンドが見つかりません")
        print("  インストール: sudo apt-get install i2c-tools")
        return False
    except Exception as e:
        print(f"エラー: {e}")
        return False

def check_i2c_bus():
    """I2Cバスの状態確認"""
    print()
    print("=" * 60)
    print("I2Cバスの状態")
    print("=" * 60)
    print()
    
    try:
        # I2Cが有効か確認
        result = subprocess.run(['ls', '/dev/i2c-*'], 
                              capture_output=True, text=True, shell=True)
        
        if result.stdout:
            print("✓ I2Cバスが有効です:")
            print(result.stdout)
        else:
            print("⚠ I2Cバスが見つかりません")
            print()
            print("【有効化方法】")
            print("  sudo raspi-config")
            print("  → Interface Options → I2C → Enable")
            
    except Exception as e:
        print(f"エラー: {e}")

def main():
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║  PCA9685 診断  ║".center(60))
    print("╚" + "═" * 58 + "╝")
    print()
    
    check_i2c_bus()
    pca_found = check_i2c_devices()
    
    print()
    print("=" * 60)
    print("診断結果と対処法")
    print("=" * 60)
    print()
    
    if not pca_found:
        print("【PCA9685 が検出されない場合の対処法】")
        print()
        print("1. 配線確認:")
        print("   - VCC (PCA9685) ← 5V (ラズパイ)")
        print("   - GND (PCA9685) ← GND (ラズパイ)")
        print("   - SDA (PCA9685) ← GPIO 2 (ラズパイ物理3p)")
        print("   - SCL (PCA9685) ← GPIO 3 (ラズパイ物理5p)")
        print()
        print("2. 電源確認:")
        print("   - PCA9685 の VCC が 5V か確認")
        print("   - 電源容量が十分か確認")
        print()
        print("3. アドレス確認:")
        print("   - PCA9685 のアドレスピン設定")
        print("   - デフォルトは 0x40")
        print()
        print("4. I2C有効化:")
        print("   sudo raspi-config")
        print("   Interface Options → I2C → Enable")
        print()
        print("5. 再起動:")
        print("   sudo reboot")
    else:
        print("✓ PCA9685 は検出されていますが、通信エラーが発生")
        print()
        print("【対処法】")
        print()
        print("1. 他のプログラムがPCA9685を使用中:")
        print("   - 他のプロセスを終了")
        print("   - 以下で確認:")
        print("     ps aux | grep python")
        print()
        print("2. I2Cバスのリセット:")
        print("   sudo systemctl restart pigpiod")
        print("   または")
        print("   sudo reboot")
        print()
        print("3. スクリプト内でGPIO.cleanup()を追加:")
        print("   try-finallyブロックで確実にクリーンアップ")

if __name__ == "__main__":
    main()
