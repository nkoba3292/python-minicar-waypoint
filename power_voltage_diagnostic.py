#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 電源電圧診断ツール
5V vs 3.3V接続の影響を分析
"""

import time
import logging

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class PowerVoltageDiagnostic:
    """電源電圧診断クラス"""
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.error_count = 0
        self.success_count = 0
        
    def connect(self):
        """接続確立"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=5,
                write_timeout=5
            )
            time.sleep(2)
            logger.info(f"Connected to {self.port}")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def test_power_stability(self, duration=60):
        """電源安定性テスト"""
        print(f"\n⚡ BNO055 電源安定性テスト ({duration}秒)")
        print("="*60)
        print("🎯 目的: 5V電源によるエラー0x07の影響を測定")
        print("📊 測定項目: 成功率、エラー頻度、応答時間")
        print("="*60)
        
        start_time = time.time()
        test_count = 0
        response_times = []
        
        while time.time() - start_time < duration:
            test_count += 1
            
            # Chip ID読み取りテスト
            success, response_time = self._test_chip_id_read()
            
            if success:
                self.success_count += 1
                response_times.append(response_time)
                status = "✅"
            else:
                self.error_count += 1
                status = "❌"
            
            # 進捗表示
            elapsed = time.time() - start_time
            success_rate = (self.success_count / test_count) * 100
            
            print(f"\r{status} [{test_count:3d}] T:{elapsed:5.1f}s | 成功率:{success_rate:5.1f}% | エラー:{self.error_count:3d}", end="", flush=True)
            
            time.sleep(0.1)  # 100ms間隔
        
        print()  # 改行
        self._print_results(response_times)
    
    def _test_chip_id_read(self):
        """Chip ID読み取りテスト"""
        try:
            start_time = time.time()
            
            # バッファクリア
            self.serial_conn.reset_input_buffer()
            
            # Chip ID読み取りコマンド
            command = bytes([0xAA, 0x01, 0x00, 0x01])
            self.serial_conn.write(command)
            self.serial_conn.flush()
            
            # 応答読み取り
            response = self.serial_conn.read(10)
            response_time = (time.time() - start_time) * 1000  # ms
            
            # 応答解析
            if len(response) >= 3 and response[0] == 0xBB:
                if response[2] == 0xA0:  # BNO055 Chip ID
                    return True, response_time
            elif len(response) >= 2 and response[0] == 0xEE:
                # エラー応答
                error_code = response[1] if len(response) > 1 else 0
                if error_code == 0x07:
                    logger.debug("Error 0x07 detected")
                return False, response_time
            
            return False, response_time
            
        except Exception as e:
            logger.debug(f"Test error: {e}")
            return False, 0
    
    def _print_results(self, response_times):
        """結果表示"""
        total_tests = self.success_count + self.error_count
        success_rate = (self.success_count / total_tests) * 100 if total_tests > 0 else 0
        
        print("\n📊 電源安定性テスト結果")
        print("="*60)
        print(f"📈 総テスト数: {total_tests}")
        print(f"✅ 成功回数: {self.success_count}")
        print(f"❌ エラー回数: {self.error_count}")
        print(f"📊 成功率: {success_rate:.1f}%")
        
        if response_times:
            avg_time = sum(response_times) / len(response_times)
            min_time = min(response_times)
            max_time = max(response_times)
            print(f"⏱️ 平均応答時間: {avg_time:.1f}ms")
            print(f"⚡ 最小応答時間: {min_time:.1f}ms") 
            print(f"🐌 最大応答時間: {max_time:.1f}ms")
        
        print("\n💡 診断結果:")
        if success_rate > 95:
            print("✅ 電源は安定しています")
        elif success_rate > 80:
            print("⚠️ 電源にやや不安定性があります")
            print("   🔧 推奨: 3.3V電源への変更を検討")
        else:
            print("🚨 電源が不安定です")
            print("   🔧 必須: 3.3V電源への変更")
            print("   📌 現在の5V電源がエラー0x07の原因である可能性が高い")
        
        print("\n🔌 配線確認:")
        print("現在: BNO055 VCC → Raspberry Pi 5V (Pin 2/4)")
        print("推奨: BNO055 VCC → Raspberry Pi 3.3V (Pin 1/17)")
    
    def disconnect(self):
        """切断"""
        if self.serial_conn:
            self.serial_conn.close()
            logger.info("Disconnected")

def main():
    """メイン実行"""
    print("⚡ BNO055 電源電圧診断ツール")
    print("="*60)
    print("🎯 5V電源接続によるエラー0x07の影響を測定")
    print("="*60)
    
    if not SERIAL_AVAILABLE:
        print("❌ pyserial が利用できません")
        return
    
    diagnostic = PowerVoltageDiagnostic()
    
    if not diagnostic.connect():
        print("❌ BNO055への接続に失敗しました")
        return
    
    try:
        # 60秒間の安定性テスト
        diagnostic.test_power_stability(60)
        
    except KeyboardInterrupt:
        print("\n\n🛑 テスト中断")
        
    finally:
        diagnostic.disconnect()
    
    print("\n🔧 次のステップ:")
    print("1. この結果をもとに電源電圧を判断")
    print("2. 必要に応じて3.3V電源に変更")
    print("3. imu_debug_ultimate.py で再テスト")

if __name__ == "__main__":
    main()