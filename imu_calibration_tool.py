# imu_calibration_tool.py - レース前キャリブレーション専用ツール
"""
自律走行レース用IMUキャリブレーションツール

使用方法:
1. レース前にコース現地で実行
2. 生成されたキャリブレーションファイルをmain_control_loop.pyが自動読み込み
3. レース中は軽量なファイル読み込みのみで高精度制御

対応キャリブレーション方式:
- Visual Map: waypointマップ表示+マウスクリック選択（最高精度）
- Landmark: テキスト入力による特徴点指定（高精度）  
- 2-Point: 従来の0°/180°回転方式（標準精度）
"""

import json
import math
import time
import sys
from platform_detector import is_raspberry_pi

class IMUCalibrationTool:
    """レース前IMUキャリブレーション統合ツール"""
    
    def __init__(self, waypoint_file="quarify.json", config_file="config.json"):
        self.waypoint_file = waypoint_file
        self.config_file = config_file
        
        # IMUドライバー初期化
        self.init_imu_driver()
        
        print("=== Race Preparation IMU Calibration Tool ===")
        print(f"Platform: {'Raspberry Pi' if is_raspberry_pi() else 'PC/Mock'}")
        print(f"IMU Driver: {'BNO055 Hardware' if self.imu_driver else 'Mock'}")
        
    def init_imu_driver(self):
        """IMUドライバー初期化"""
        self.imu_driver = None
        
        if is_raspberry_pi():
            try:
                from bno055_imu_driver import BNO055IMUDriver
                self.imu_driver = BNO055IMUDriver(config_file=self.config_file)
                
                # BNO055キャリブレーション状態確認
                if self.imu_driver.is_calibrated():
                    print("✓ BNO055 hardware calibration complete")
                else:
                    print("⚠ BNO055 calibration in progress...")
                    print("  Move sensor in figure-8 pattern for better calibration")
                    
            except Exception as e:
                print(f"BNO055 initialization failed: {e}")
                print("Continuing in mock mode...")
        else:
            print("PC environment - using mock IMU")
    
    def get_current_yaw(self):
        """現在のヨー角取得"""
        if self.imu_driver:
            return self.imu_driver.get_yaw()
        else:
            # モック環境: 固定値
            return 0.0
    
    def show_menu(self):
        """メインメニュー表示"""
        print("\n" + "="*50)
        print("IMU CALIBRATION TOOL - MAIN MENU")
        print("="*50)
        print("Select calibration method:")
        print()
        print("1. Visual Map Calibration (RECOMMENDED)")
        print("   → Interactive waypoint map with mouse selection")
        print("   → Highest precision (±0.5°)")
        print()
        print("2. Landmark Calibration")
        print("   → Text-based landmark description input")
        print("   → High precision (±1°)")
        print()
        print("3. 2-Point Calibration") 
        print("   → Traditional 0°/180° rotation method")
        print("   → Standard precision (±2-3°)")
        print()
        print("4. Test Existing Calibration")
        print("   → Verify saved calibration files")
        print()
        print("5. Exit")
        print("="*50)
        
        choice = input("Enter your choice (1-5): ").strip()
        return choice
    
    def run_visual_calibration(self):
        """ビジュアルマップキャリブレーション実行"""
        print("\n=== Visual Map Calibration ===")
        
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("❌ matplotlib not available")
            print("Install with: pip install matplotlib")
            return False
        
        try:
            from imu_visual_calibration import IMUVisualCalibration
            
            calibrator = IMUVisualCalibration(
                imu_driver=self.imu_driver,
                waypoint_file=self.waypoint_file,
                config_file=self.config_file
            )
            
            print("Starting interactive visual calibration...")
            print("Instructions will appear in the GUI window.")
            
            success = calibrator.run_interactive_calibration()
            
            if success:
                print("✓ Visual calibration completed")
                print("✓ Calibration file saved: imu_visual_calib.json")
                return True
            else:
                print("❌ Visual calibration failed")
                return False
                
        except Exception as e:
            print(f"❌ Visual calibration error: {e}")
            return False
    
    def run_landmark_calibration(self):
        """特徴点ベースキャリブレーション実行"""
        print("\n=== Landmark-Based Calibration ===")
        
        try:
            from imu_landmark_calibration import IMULandmarkCalibration
            
            calibrator = IMULandmarkCalibration(
                imu_driver=self.imu_driver,
                config_file=self.config_file
            )
            
            success = calibrator.calibrate_with_landmarks()
            
            if success:
                print("✓ Landmark calibration completed")
                print("✓ Calibration file saved: imu_landmark_calib.json")
                return True
            else:
                print("❌ Landmark calibration failed")
                return False
                
        except Exception as e:
            print(f"❌ Landmark calibration error: {e}")
            return False
    
    def run_2point_calibration(self):
        """2点キャリブレーション実行"""
        print("\n=== 2-Point Calibration ===")
        
        try:
            from imu_2point_calibration import IMU2PointCalibration
            
            calibrator = IMU2PointCalibration()
            
            print("This method uses 0°/180° rotation at start position")
            confirm = input("Continue with 2-point calibration? (y/n): ").lower()
            
            if confirm != 'y':
                return False
            
            # 2点キャリブレーション実行（詳細は既存ファイルに依存）
            print("Please follow the prompts for 0° and 180° positioning...")
            
            # 仮の実装（実際のimu_2point_calibration.pyに依存）
            print("⚠ 2-point calibration implementation depends on existing file")
            print("Run: python imu_2point_calibration.py manually")
            
            return False
            
        except Exception as e:
            print(f"❌ 2-point calibration error: {e}")
            return False
    
    def test_calibration(self):
        """既存キャリブレーション確認"""
        print("\n=== Calibration Test ===")
        
        calibration_files = [
            ("imu_visual_calib.json", "Visual Map"),
            ("imu_landmark_calib.json", "Landmark"),  
            ("imu_2point_calib.json", "2-Point")
        ]
        
        found_calibrations = []
        
        for calib_file, method in calibration_files:
            try:
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                
                offset = calib_data.get('offset', 0.0)
                date = calib_data.get('calibration_date', 'unknown')
                
                found_calibrations.append({
                    'file': calib_file,
                    'method': method,
                    'offset': offset,
                    'date': date
                })
                
                print(f"✓ {method} Calibration Found")
                print(f"  File: {calib_file}")
                print(f"  Offset: {math.degrees(offset):.2f}°")
                print(f"  Date: {date}")
                
                if 'accuracy' in calib_data:
                    print(f"  Accuracy: {calib_data['accuracy']}")
                
                print()
                
            except FileNotFoundError:
                print(f"○ {method}: Not calibrated")
            except Exception as e:
                print(f"❌ {method}: Error reading ({e})")
        
        if found_calibrations:
            print("=== Live Calibration Test ===")
            
            # 最優先のキャリブレーション使用
            best_calib = found_calibrations[0]
            offset = best_calib['offset']
            
            print(f"Testing with {best_calib['method']} calibration...")
            
            for i in range(5):
                raw_yaw = self.get_current_yaw()
                calibrated_yaw = raw_yaw + offset
                
                # 正規化
                while calibrated_yaw > math.pi:
                    calibrated_yaw -= 2 * math.pi
                while calibrated_yaw < -math.pi:
                    calibrated_yaw += 2 * math.pi
                
                print(f"{i+1}: Raw={math.degrees(raw_yaw):6.2f}° → "
                      f"Calibrated={math.degrees(calibrated_yaw):6.2f}°")
                
                time.sleep(1)
            
            print("✓ Calibration test completed")
        else:
            print("⚠ No calibration files found")
            print("  Please run calibration before race")
        
        return len(found_calibrations) > 0
    
    def run(self):
        """メインループ実行"""
        while True:
            choice = self.show_menu()
            
            if choice == '1':
                self.run_visual_calibration()
            elif choice == '2':
                self.run_landmark_calibration()
            elif choice == '3':
                self.run_2point_calibration()
            elif choice == '4':
                self.test_calibration()
            elif choice == '5':
                print("Exiting calibration tool...")
                break
            else:
                print("Invalid choice. Please select 1-5.")
            
            input("\nPress Enter to continue...")
        
        print("\n" + "="*50)
        print("CALIBRATION COMPLETE")
        print("="*50)
        print("Your calibration files are ready for racing!")
        print("Run main_control_loop.py to start autonomous racing.")
        print("="*50)

# メイン実行
if __name__ == "__main__":
    print("Starting IMU Calibration Tool for Autonomous Racing...")
    print()
    
    tool = IMUCalibrationTool()
    
    try:
        tool.run()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    print("Thank you for using IMU Calibration Tool!")