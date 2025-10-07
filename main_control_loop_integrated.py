# main_control_loop.py
# -*- coding: utf-8 -*-
"""
統合メインシステム - 走行モード選択 → IMUキャリブレーション → レース実行
"""
import json
import time
import os
import sys
from datetime import datetime

# IMUキャリブレーションシステムをインポート
try:
    from imu_calibration_vehicle_footprint import IMUCalibrationSystem
    CALIBRATION_AVAILABLE = True
except ImportError:
    CALIBRATION_AVAILABLE = False
    print("⚠️ Warning: IMU calibration system not available")

class RaceMonitor:
    """レース中のリアルタイム監視システム"""
    def __init__(self):
        # 監視データ
        self.race_data = {
            'race_time': 0.0,
            'current_waypoint_index': 0,
            'total_waypoints': 0,
            'imu_yaw': 0.0,
            'imu_offset': 0.0,
            'ultrasonic_distances': {'FL': 0.0, 'FR': 0.0, 'BL': 0.0, 'BR': 0.0},
            'current_waypoint': {'x': 0.0, 'y': 0.0, 'v': 0.0, 'yaw': 0.0},
            'next_waypoint': {'x': 0.0, 'y': 0.0, 'v': 0.0, 'yaw': 0.0},
            'vehicle_speed': 0.0,
            'steering_angle': 0.0,
            'distance_to_waypoint': 0.0,
            'battery_level': 100.0,
            'cpu_usage': 0.0
        }
        
    def update_imu_data(self, yaw, offset=0.0):
        """IMUデータ更新"""
        self.race_data['imu_yaw'] = yaw
        self.race_data['imu_offset'] = offset
        
    def update_ultrasonic_data(self, fl, fr, bl, br):
        """超音波センサーデータ更新"""
        self.race_data['ultrasonic_distances'] = {
            'FL': fl, 'FR': fr, 'BL': bl, 'BR': br
        }
        
    def update_waypoint_data(self, current_index, waypoints, current_pos=(0.0, 0.0)):
        """ウェイポイントデータ更新"""
        self.race_data['current_waypoint_index'] = current_index
        self.race_data['total_waypoints'] = len(waypoints)
        
        if 0 <= current_index < len(waypoints):
            wp = waypoints[current_index]
            self.race_data['current_waypoint'] = {
                'x': wp.get('x', 0) * 0.05 - 3.2,
                'y': wp.get('y', 0) * 0.05 - 1.5,
                'v': wp.get('v', 100),
                'yaw': wp.get('yaw', 0)
            }
            
            # 現在位置からの距離計算
            dx = self.race_data['current_waypoint']['x'] - current_pos[0]
            dy = self.race_data['current_waypoint']['y'] - current_pos[1]
            self.race_data['distance_to_waypoint'] = (dx**2 + dy**2)**0.5
        
        # 次のウェイポイント
        next_index = current_index + 1
        if next_index < len(waypoints):
            wp_next = waypoints[next_index]
            self.race_data['next_waypoint'] = {
                'x': wp_next.get('x', 0) * 0.05 - 3.2,
                'y': wp_next.get('y', 0) * 0.05 - 1.5,
                'v': wp_next.get('v', 100),
                'yaw': wp_next.get('yaw', 0)
            }
            
    def update_vehicle_data(self, speed, steering_angle):
        """車両データ更新"""
        self.race_data['vehicle_speed'] = speed
        self.race_data['steering_angle'] = steering_angle
        
    def update_system_data(self, race_time, battery=100.0, cpu=0.0):
        """システムデータ更新"""
        self.race_data['race_time'] = race_time
        self.race_data['battery_level'] = battery
        self.race_data['cpu_usage'] = cpu
        
    def display_monitor_line(self, mode_name="Unknown"):
        """コンパクトな1行監視表示"""
        data = self.race_data
        
        # フォーマット済み表示文字列作成
        monitor_line = (
            f"🏁 {mode_name} | "
            f"T:{data['race_time']:6.1f}s | "
            f"WP:{data['current_waypoint_index']+1:3d}/{data['total_waypoints']:3d} | "
            f"IMU:{data['imu_yaw']:6.1f}° | "
            f"Speed:{data['vehicle_speed']:6.1f} | "
            f"Steer:{data['steering_angle']:+5.1f}° | "
            f"Dist:{data['distance_to_waypoint']:5.2f}m | "
            f"US: FL:{data['ultrasonic_distances']['FL']:4.1f} FR:{data['ultrasonic_distances']['FR']:4.1f} | "
            f"Bat:{data['battery_level']:5.1f}%"
        )
        
        # 画面クリア＋表示（同じ行を更新）
        print(f"\r{monitor_line}", end="", flush=True)
        
    def display_detailed_monitor(self, mode_name="Unknown"):
        """詳細監視表示"""
        data = self.race_data
        
        # 画面クリア（Windows対応）
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("🏁 RACE MONITORING DASHBOARD")
        print("="*80)
        print(f"Mode: {mode_name} | Time: {data['race_time']:.1f}s | WP: {data['current_waypoint_index']+1}/{data['total_waypoints']}")
        print("="*80)
        
        # IMUデータ
        print(f"🧭 IMU Data                📍 Current Waypoint        🚗 Vehicle Status")
        print(f"Yaw:    {data['imu_yaw']:6.1f}°         Target: ({data['current_waypoint']['x']:5.1f}, {data['current_waypoint']['y']:5.1f})    Speed:  {data['vehicle_speed']:6.1f}")
        print(f"Offset: {data['imu_offset']:+6.1f}°         Distance: {data['distance_to_waypoint']:5.2f}m       Steering: {data['steering_angle']:+6.1f}°")
        print()
        
        # 超音波センサー
        print(f"🔊 Ultrasonic Sensors      🎯 Next Waypoint           ⚡ System Status")
        print(f"FL: {data['ultrasonic_distances']['FL']:4.1f}m  FR: {data['ultrasonic_distances']['FR']:4.1f}m     Target: ({data['next_waypoint']['x']:5.1f}, {data['next_waypoint']['y']:5.1f})    Battery: {data['battery_level']:5.1f}%")
        print(f"BL: {data['ultrasonic_distances']['BL']:4.1f}m  BR: {data['ultrasonic_distances']['BR']:4.1f}m     Speed: {data['next_waypoint']['v']:6.1f}          CPU: {data['cpu_usage']:5.1f}%")
        print("="*80)
        
    def mock_sensor_update(self):
        """センサーデータのモック更新（テスト用）"""
        import random
        
        # モックIMUデータ
        self.update_imu_data(random.uniform(0, 360), self.race_data['imu_offset'])
        
        # モック超音波データ
        self.update_ultrasonic_data(
            random.uniform(0.5, 3.0),
            random.uniform(0.5, 3.0),
            random.uniform(0.5, 3.0),
            random.uniform(0.5, 3.0)
        )
        
        # モック車両データ
        self.update_vehicle_data(
            random.uniform(50, 120),
            random.uniform(-30, 30)
        )
        
        # モックシステムデータ
        self.update_system_data(
            self.race_data['race_time'] + 0.1,
            random.uniform(70, 100),
            random.uniform(20, 60)
        )

class MainControlSystem:
    def __init__(self):
        # 走行モード定義（waypoint_editor_multi_mode.pyと同じ）
        self.DRIVING_MODES = {
            'qualifying': {'name': 'Qualifying', 'file': 'waypoints_qualifying.json'},
            'qualifying_backup': {'name': 'Qualifying Backup', 'file': 'waypoints_qualifying_backup.json'},
            'final': {'name': 'Final Race', 'file': 'waypoints_final.json'},
            'final_backup': {'name': 'Final Backup', 'file': 'waypoints_final_backup.json'}
        }
        
        self.selected_mode = None
        self.waypoints = []
        self.calibration_data = None
        self.imu_offset = 0.0
        
        # システム状態
        self.system_ready = False
        self.calibration_completed = False
        
        # モニタリングシステム
        self.monitor = RaceMonitor()
        
    def display_startup_banner(self):
        """システム起動時のバナー表示"""
        print("\n" + "="*70)
        print("🏁 AUTONOMOUS RACING SYSTEM - MAIN CONTROL")
        print("="*70)
        print("System Features:")
        print("• 4-Mode Waypoint Racing (Qualifying/Final + Backup)")
        print("• IMU Calibration Integration")
        print("• Real-time Race Control")
        print("• Safety & Monitoring")
        print(f"Calibration System: {'✅ Available' if CALIBRATION_AVAILABLE else '❌ Not Available'}")
        print("="*70)
    
    def select_driving_mode(self):
        """走行モード選択"""
        print("\n📋 DRIVING MODE SELECTION")
        print("="*50)
        
        # 利用可能なモード表示
        mode_list = []
        for i, (mode_key, mode_info) in enumerate(self.DRIVING_MODES.items(), 1):
            file_exists = os.path.exists(mode_info['file'])
            status = "✅ Ready" if file_exists else "❌ No Data"
            print(f"  {i}. {mode_info['name']} - {status}")
            mode_list.append(mode_key)
        
        # モード選択
        while True:
            try:
                print(f"\nSelect mode (1-{len(mode_list)}): ", end="")
                choice = int(input())
                if 1 <= choice <= len(mode_list):
                    selected_key = mode_list[choice - 1]
                    self.selected_mode = selected_key
                    
                    # Waypointファイル読み込み
                    waypoint_file = self.DRIVING_MODES[selected_key]['file']
                    if self.load_waypoints(waypoint_file):
                        print(f"✅ Mode Selected: {self.DRIVING_MODES[selected_key]['name']}")
                        print(f"📍 Waypoints Loaded: {len(self.waypoints)} points")
                        return True
                    else:
                        print(f"❌ Failed to load waypoints from {waypoint_file}")
                        continue
                else:
                    print("❌ Invalid selection. Please try again.")
            except ValueError:
                print("❌ Please enter a number.")
            except KeyboardInterrupt:
                print("\n🛑 Operation cancelled.")
                return False
    
    def load_waypoints(self, filename):
        """Waypointファイル読み込み"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                self.waypoints = json.load(f)
            return True
        except Exception as e:
            print(f"❌ Error loading waypoints: {e}")
            return False
    
    def run_imu_calibration(self):
        """IMUキャリブレーション実行"""
        if not CALIBRATION_AVAILABLE:
            print("❌ IMU Calibration system not available")
            return False
        
        print("\n🎯 IMU CALIBRATION")
        print("="*50)
        print("📡 Starting IMU calibration system...")
        print("📋 Instructions:")
        print("  1. Position vehicle at Pos 1 (red footprint)")  
        print("  2. Click 'Measure IMU 1' button")
        print("  3. Move to Pos 2 (blue footprint)")
        print("  4. Click 'Measure IMU 2' button") 
        print("  5. Click 'Save Results' to complete")
        print("  6. Close the calibration window to continue")
        print("\n🔄 Launching calibration interface...")
        
        try:
            # IMUキャリブレーションシステム実行
            calibration_system = IMUCalibrationSystem()
            calibration_system.run_calibration_system()
            
            # キャリブレーション完了後の確認メッセージ
            print("\n🎯 Calibration window closed.")
            print("⏳ Loading calibration results...")
            
            # キャリブレーション完了後、ファイル読み込み
            return self.load_calibration_data()
            
        except Exception as e:
            print(f"❌ Calibration error: {e}")
            return False
    
    def load_calibration_data(self):
        """キャリブレーションデータ読み込み"""
        calibration_files = ["imu_custom_calib.json"]
        
        for filename in calibration_files:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    self.calibration_data = json.load(f)
                
                if self.calibration_data.get('validation', {}).get('is_valid', False):
                    self.imu_offset = self.calibration_data.get('calculated_offset', 0.0)
                    print(f"✅ Calibration loaded: Offset = {self.imu_offset:.2f}°")
                    self.calibration_completed = True
                    return True
                else:
                    print(f"⚠️ Calibration validation failed: {self.calibration_data.get('validation', {}).get('message', 'Unknown error')}")
                    
            except FileNotFoundError:
                continue
            except Exception as e:
                print(f"❌ Error reading calibration: {e}")
        
        print("❌ No valid calibration data found")
        return False
    
    def system_status_check(self):
        """システム状態確認"""
        print("\n🔍 SYSTEM STATUS CHECK")
        print("="*50)
        
        # 走行モード確認
        mode_status = "✅ Ready" if self.selected_mode else "❌ Not Selected"
        waypoint_status = f"✅ {len(self.waypoints)} points" if self.waypoints else "❌ No Data"
        
        # キャリブレーション確認
        calib_status = "✅ Completed" if self.calibration_completed else "❌ Required"
        offset_info = f"({self.imu_offset:.2f}°)" if self.calibration_completed else ""
        
        print(f"📋 Driving Mode: {mode_status}")
        if self.selected_mode:
            print(f"   → {self.DRIVING_MODES[self.selected_mode]['name']}")
        print(f"📍 Waypoints: {waypoint_status}")
        print(f"🎯 IMU Calibration: {calib_status} {offset_info}")
        
        # システム準備完了判定
        self.system_ready = (self.selected_mode is not None and 
                           len(self.waypoints) > 0 and 
                           self.calibration_completed)
        
        status_icon = "🟢" if self.system_ready else "🔴"
        status_text = "READY FOR RACING" if self.system_ready else "SETUP INCOMPLETE"
        
        print(f"\n{status_icon} System Status: {status_text}")
        return self.system_ready
    
    def wait_for_race_start(self):
        """レース開始待機"""
        if not self.system_ready:
            print("❌ System not ready for racing")
            return False
        
        print("\n🏁 RACE START PREPARATION")
        print("="*50)
        print("📋 Pre-race Checklist:")
        print(f"   ✅ Mode: {self.DRIVING_MODES[self.selected_mode]['name']}")
        print(f"   ✅ Waypoints: {len(self.waypoints)} loaded")
        print(f"   ✅ IMU Offset: {self.imu_offset:.2f}°")
        print("\n🚗 VEHICLE POSITIONING INSTRUCTIONS:")
        print("="*50)
        print("   1. 🎯 Place vehicle at START position on the course")
        print("   2. 🧭 Align vehicle with the start line direction")
        print("   3. 🔋 Check battery level (ensure sufficient power)")
        print("   4. 👀 Ensure clear racing path (no obstacles)")
        print("   5. ⚡ Verify all systems operational")
        print("   6. 🔧 Double-check vehicle is properly calibrated")
        
        print(f"\n🟢 System is READY for racing!")
        print(f"⏳ When vehicle is positioned correctly, press ENTER to start racing...")
        
        try:
            input()  # Enter待機
            return True
        except KeyboardInterrupt:
            print("\n🛑 Race start cancelled.")
            return False
    
    def run_race(self):
        """レース実行（リアルタイムモニタリング付きメインループ）"""
        print("\n🏁 RACE STARTED!")
        print("="*80)
        print(f"🎯 Mode: {self.DRIVING_MODES[self.selected_mode]['name']}")
        print(f"📍 Following {len(self.waypoints)} waypoints")
        print(f"🧭 IMU Offset: {self.imu_offset:.2f}°")
        print("🔍 Real-time monitoring active...")
        print("="*80)
        
        race_start_time = time.time()
        mode_name = self.DRIVING_MODES[self.selected_mode]['name']
        
        # モニターシステム初期化
        self.monitor.update_system_data(0.0)
        
        # レースループ（モニタリング統合版）
        try:
            for i, waypoint in enumerate(self.waypoints):
                elapsed = time.time() - race_start_time
                
                # 現在位置（仮想位置 - 実際にはGPS/オドメトリから取得）
                current_pos = (
                    waypoint.get('x', 0) * 0.05 - 3.2,  # 仮想現在地
                    waypoint.get('y', 0) * 0.05 - 1.5
                )
                
                # モニタリングデータ更新
                self.monitor.update_waypoint_data(i, self.waypoints, current_pos)
                self.monitor.update_system_data(elapsed)
                
                # センサーデータ更新（モック - 実際のセンサーと置き換え）
                self.monitor.mock_sensor_update()
                
                # IMUオフセット適用
                self.monitor.race_data['imu_offset'] = self.imu_offset
                
                # リアルタイム監視表示
                self.monitor.display_monitor_line(mode_name)
                
                # 制御ロジック（ここに実装）
                # - IMU読み取り + オフセット補正
                # - モーター制御
                # - 超音波センサー監視
                # - Pure Pursuit アルゴリズム
                # - 障害物回避
                
                time.sleep(0.1)  # 制御周期（10Hz）
                
                # 詳細表示モード切り替え（デバッグ用）
                # 5秒ごとに詳細表示（オプション）
                if i % 50 == 0 and i > 0:  # 5秒ごと
                    print()  # 改行
                    self.monitor.display_detailed_monitor(mode_name)
                    time.sleep(2)  # 2秒間詳細表示
                
        except KeyboardInterrupt:
            print(f"\n🛑 RACE STOPPED (Manual Stop)")
            
        except Exception as e:
            print(f"\n❌ RACE ERROR: {e}")
            
        finally:
            # レース終了処理
            total_time = time.time() - race_start_time
            print(f"\n\n🏁 RACE COMPLETED")
            print("="*80)
            print(f"   Total Time: {total_time:.1f} seconds")
            print(f"   Waypoints: {len(self.waypoints)} processed")
            print(f"   Mode: {mode_name}")
            print(f"   Final Waypoint: {self.monitor.race_data['current_waypoint_index']+1}")
            print("="*80)
            
            # 最終統計表示
            print("\n📊 RACE STATISTICS:")
            print(f"   Average Speed: {self.monitor.race_data['vehicle_speed']:.1f}")
            print(f"   Final IMU Reading: {self.monitor.race_data['imu_yaw']:.1f}°")
            print(f"   IMU Offset Used: {self.imu_offset:.2f}°")
    
    def main_loop(self):
        """メインシステムループ"""
        self.display_startup_banner()
        
        try:
            # ① 走行モード選択
            if not self.select_driving_mode():
                print("🛑 System startup cancelled.")
                return
            
            # ② IMUキャリブレーション
            print(f"\n🎯 Ready for IMU calibration...")
            if not self.run_imu_calibration():
                print("❌ Calibration required for racing.")
                return
            
            # ③ システム状態確認
            if not self.system_status_check():
                print("❌ System setup incomplete.")
                return
            
            # ④ レース開始待機
            if not self.wait_for_race_start():
                print("🛑 Race cancelled.")
                return
            
            # ⑤ レース実行
            self.run_race()
            
        except Exception as e:
            print(f"\n❌ System Error: {e}")
        
        finally:
            print("\n📴 Main Control System shutdown.")

def main():
    """メイン実行関数"""
    system = MainControlSystem()
    system.main_loop()

if __name__ == "__main__":
    main()