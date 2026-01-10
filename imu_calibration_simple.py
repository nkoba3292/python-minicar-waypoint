# imu_calibration_simple.py
# -*- coding: utf-8 -*-
"""
GUI無しバージョン - コンソール出力のみでIMUデータと位置推定を確認
"""
import json
import math
import time
import numpy as np
from datetime import datetime

class IMUCalibrationSimple:
    def __init__(self):
        """初期化"""
        print("=" * 60)
        print("IMU Calibration System - Console Mode")
        print("=" * 60)
        
        # IMU初期化
        self.imu = None
        self.imu_available = False
        self.mock_mode = False
        
        # キャリブレーションデータ
        self.calibration_data = None
        
        # 現在の測定値
        self.current_measurement = 0
        
        # 位置・速度データ（加速度積分）
        self.current_pos = [0.0, 0.0]  # [x, y]
        self.current_velocity = [0.0, 0.0]  # [vx, vy]
        self.current_yaw = 0.0  # 現在のYAW角度（deg）
        
        # 重力補正用の基準値（静止時の加速度平均）
        self.gravity_baseline = [0.0, 0.0, 0.0]  # [ax, ay, az]
        self.baseline_samples = []
        self.baseline_ready = False
        
        # タイマー
        self.last_time = None
        self.update_count = 0
        
        print("\n[INIT] Initializing IMU...")
        self.initialize_imu()
        
    def initialize_imu(self):
        """IMU初期化"""
        try:
            # BNO055ドライバーをインポート（Raspberry Pi上でのみ動作）
            import sys
            import os
            current_dir = os.path.dirname(os.path.abspath(__file__))
            if current_dir not in sys.path:
                sys.path.insert(0, current_dir)
            print(f"[DEBUG] Python path: {sys.path[0]}")
            print(f"[DEBUG] Current dir: {current_dir}")
            
            from bno055_imu_driver import BNO055IMUDriver
            
            self.imu = BNO055IMUDriver()
            
            # キャリブレーションファイル読み込み
            calib_file = 'imu_2point_calib.json'
            try:
                with open(calib_file, 'r') as f:
                    self.calibration_data = json.load(f)
                print(f"✓ Calibration loaded: {calib_file}")
            except FileNotFoundError:
                print(f"⚠ No calibration file found: {calib_file}")
                self.calibration_data = None
            
            self.imu_available = True
            self.mock_mode = False
            print("✓ IMU initialized (Real hardware)")
            
        except (ImportError, ModuleNotFoundError) as e:
            print(f"⚠ BNO055 module not found (expected on PC): {e}")
            print(f"✓ Using MOCK mode for PC testing")
            self.imu_available = True
            self.mock_mode = True
            self.mock_time = 0.0
            
        except Exception as e:
            print(f"✗ IMU initialization failed: {e}")
            print(f"✓ Using MOCK mode for PC testing")
            self.imu_available = True
            self.mock_mode = True
            self.mock_time = 0.0
    
    def reset_position(self):
        """位置・速度リセット"""
        self.current_pos = [0.0, 0.0]
        self.current_velocity = [0.0, 0.0]
        self.baseline_samples = []
        self.baseline_ready = False
        self.update_count = 0
        self.last_time = None
        print("\n[RESET] Position and velocity reset")
    
    def update_position_from_accel(self, accel_x, accel_y, accel_z, dt):
        """加速度から位置を推定（二重積分）"""
        self.update_count += 1
        
        # デバッグ出力（最初の30回のみ）
        if self.update_count <= 30:
            print(f"\n[TRACE #{self.update_count}] update_position_from_accel called")
            print(f"  Raw accel: ax={accel_x:.3f}, ay={accel_y:.3f}, az={accel_z:.3f} m/s²")
            print(f"  dt={dt:.4f}s, baseline_ready={self.baseline_ready}")
        
        # dtチェック（最初の20回は起動時の異常値をスキップ）
        if self.update_count > 20:
            if dt > 2.0:  # 2秒以上は異常
                print(f"⚠️ Abnormal dt={dt:.2f}s - skipping update (count > 20)")
                if dt > 1.0:
                    print(f"   Expected: 0.05s (20Hz), Actual: {dt:.2f}s")
                return
        else:
            if self.update_count <= 30:
                print(f"✓ Initial startup (count ≤ 20) - dt check bypassed")
        
        # 重力の基準値をキャリブレーション（最初の20サンプルで平均）
        if not self.baseline_ready:
            self.baseline_samples.append([accel_x, accel_y, accel_z])
            if len(self.baseline_samples) >= 20:
                # 平均を計算
                samples_array = np.array(self.baseline_samples)
                self.gravity_baseline = samples_array.mean(axis=0).tolist()
                self.baseline_ready = True
                print(f"\n[BASELINE] Gravity compensation calibrated:")
                print(f"  Baseline: ax={self.gravity_baseline[0]:.3f}, "
                      f"ay={self.gravity_baseline[1]:.3f}, "
                      f"az={self.gravity_baseline[2]:.3f} m/s²")
            else:
                if self.update_count <= 30:
                    print(f"  BASELINE collecting... ({len(self.baseline_samples)}/20)")
                return
        
        # 重力補正（基準値を引く）
        accel_x_comp = accel_x - self.gravity_baseline[0]
        accel_y_comp = accel_y - self.gravity_baseline[1]
        
        if self.update_count <= 30:
            print(f"  Compensated: ax={accel_x_comp:.3f}, ay={accel_y_comp:.3f} m/s²")
        
        # ノイズフィルタ（0.1m/s²以下は無視）
        threshold = 0.1
        if abs(accel_x_comp) < threshold:
            accel_x_comp = 0.0
        if abs(accel_y_comp) < threshold:
            accel_y_comp = 0.0
        
        if self.update_count <= 30:
            print(f"  Filtered: ax={accel_x_comp:.3f}, ay={accel_y_comp:.3f} m/s²")
        
        # 速度を更新（v = v0 + a*dt）
        self.current_velocity[0] += accel_x_comp * dt
        self.current_velocity[1] += accel_y_comp * dt
        
        # 速度減衰（摩擦・空気抵抗を模擬）
        decay = 0.99
        self.current_velocity[0] *= decay
        self.current_velocity[1] *= decay
        
        if self.update_count <= 30:
            print(f"  Velocity: vx={self.current_velocity[0]:.3f}, vy={self.current_velocity[1]:.3f} m/s")
        
        # 位置を更新（x = x0 + v*dt）
        self.current_pos[0] += self.current_velocity[0] * dt
        self.current_pos[1] += self.current_velocity[1] * dt
        
        if self.update_count <= 30:
            print(f"  Position: x={self.current_pos[0]:.3f}, y={self.current_pos[1]:.3f} m")
    
    def update_imu_status(self):
        """IMUステータス更新"""
        if not self.imu_available:
            return
        
        # タイマー間隔を測定
        current_time = time.time()
        if self.last_time is not None:
            dt = current_time - self.last_time
            if self.update_count <= 10:
                print(f"\n[TIMER] Interval: {dt:.4f}s (expected: 0.05s for 20Hz)")
        else:
            dt = 0.05  # 初回
        self.last_time = current_time
        
        try:
            # モックモード確認
            if self.update_count == 1 or self.update_count == 31:
                print(f"\n[DEBUG] update_count={self.update_count}, mock_mode={self.mock_mode}")
            
            # モックモード
            if self.mock_mode:
                self.mock_time += dt
                
                # モックデータ生成
                calib_status_str = "Sys:3 Gyro:3 Accel:3 Mag:3 (MOCK)"
                
                # YAWは時計回りに回転
                raw_yaw = (self.mock_time * 10) % 360  # 10度/秒
                self.current_yaw = raw_yaw
                yaw_display = f"YAW: {raw_yaw:.1f}° (MOCK)"
                
                # 加速度は正弦波
                accel_x = 0.5 * math.sin(self.mock_time * 2 * math.pi / 5)  # 5秒周期
                accel_y = 0.3 * math.cos(self.mock_time * 2 * math.pi / 5)
                accel_z = 0.0
                accel_display = f"Accel: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} m/s² (MOCK)"
                
                # 位置推定を更新
                self.update_position_from_accel(accel_x, accel_y, accel_z, dt)
                
            else:
                # 実IMUデータ
                if self.update_count <= 35:
                    print(f"\n[DEBUG] Reading real IMU data (update_count={self.update_count})")
                
                # IMUキャリブレーションステータス
                calib = self.imu.get_calibration_status()
                calib_status_str = f"Sys:{calib[0]} Gyro:{calib[1]} Accel:{calib[2]} Mag:{calib[3]}"
                
                # オイラー角（YAW）
                euler = self.imu.read_euler()
                if euler is not None:
                    raw_yaw = euler[0]  # 0-360度
                    if raw_yaw is not None:
                        self.current_yaw = raw_yaw
                        if self.current_measurement == 0:
                            # Point A
                            yaw_display = f"Point A: {raw_yaw:.1f}°"
                        elif self.current_measurement == 1:
                            # Point B
                            if self.calibration_data and 'point_a' in self.calibration_data:
                                point_a_yaw = self.calibration_data['point_a']['yaw']
                                delta_yaw = raw_yaw - point_a_yaw
                                if delta_yaw > 180:
                                    delta_yaw -= 360
                                elif delta_yaw < -180:
                                    delta_yaw += 360
                                yaw_display = f"Point B: {raw_yaw:.1f}° (Δ{delta_yaw:+.1f}°)"
                            else:
                                yaw_display = f"Point B: {raw_yaw:.1f}°"
                        else:
                            yaw_display = f"YAW: {raw_yaw:.1f}°"
                    else:
                        yaw_display = "YAW: --"
                else:
                    yaw_display = "YAW: --"
                
                # 加速度（線形加速度）
                linear_accel = self.imu.read_linear_acceleration()
                if linear_accel is not None:
                    accel_x, accel_y, accel_z = linear_accel
                    accel_display = f"Accel: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} m/s²"
                    
                    # 位置推定を更新
                    self.update_position_from_accel(accel_x, accel_y, accel_z, dt)
                else:
                    accel_display = "Accel: --"
            
            # コンソール出力（10回ごと）
            if self.update_count % 10 == 0:
                print(f"\n[STATUS] {calib_status_str}")
                print(f"  {yaw_display}")
                print(f"  {accel_display}")
                print(f"  Position: x={self.current_pos[0]:.3f}m, y={self.current_pos[1]:.3f}m")
                print(f"  Velocity: vx={self.current_velocity[0]:.3f}m/s, vy={self.current_velocity[1]:.3f}m/s")
            
        except Exception as e:
            import traceback
            print(f"[ERROR] IMU status update failed: {e}")
            traceback.print_exc()
    
    def run(self, duration=10):
        """指定時間だけIMUデータを取得・表示"""
        print(f"\n[RUN] Starting data collection for {duration} seconds...")
        print("Press Ctrl+C to stop")
        
        self.reset_position()
        
        start_time = time.time()
        try:
            while True:
                self.update_imu_status()
                
                # 20Hzで更新
                time.sleep(0.05)
                
                # 終了チェック
                elapsed = time.time() - start_time
                if elapsed >= duration:
                    break
                    
        except KeyboardInterrupt:
            print("\n\n[STOP] Stopped by user")
        
        print(f"\n[FINISH] Collection complete")
        print(f"  Total updates: {self.update_count}")
        print(f"  Final position: x={self.current_pos[0]:.3f}m, y={self.current_pos[1]:.3f}m")
        print(f"  Final velocity: vx={self.current_velocity[0]:.3f}m/s, vy={self.current_velocity[1]:.3f}m/s")


if __name__ == '__main__':
    system = IMUCalibrationSimple()
    
    # 10秒間データ取得
    system.run(duration=10)
