# imu_calibration_vehicle_footprint.py
# -*- coding: utf-8 -*-
"""
waypoint_editor_multi_mode.pyと同じ高品質表示でIMUキャリブレーション
車両フットプリント（180mm×350mm）を描画してIMU測定を行う
"""
import matplotlib
# Waylandの警告を回避するためバックエンドを明示的に設定
matplotlib.use('TkAgg')  # または 'Qt5Agg'
import matplotlib.pyplot as plt
print('✓ matplotlib (backend: TkAgg)')
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.widgets import Button
print('✓ matplotlib widgets')
import json
import math
import numpy as np
print('✓ numpy')
from datetime import datetime
from course_map import grid_matrix, world_to_grid, grid_to_world, start_pos, goal_pos, obstacles, start_lines, pylons
from course_map import x_min, x_max, y_min, y_max, resolution
print('✓ course_map')
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.widgets import Button

# 文字化け完全修正 - 英語表示に変更
plt.rcParams['font.family'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class IMUCalibrationSystem:
    def __init__(self):
        # 車両フットプリント設定（メートル単位）
        self.vehicle_width = 0.180   # 180mm
        self.vehicle_length = 0.350  # 350mm
        
        # 測定データ
        self.measurement_points = []
        self.current_measurement = 1  # 1 or 2
        self.max_measurements = 2
        
        # 車両フットプリント位置設定（実際のコース壁に沿った最適位置）
        self.footprint_positions = [
            {"x": -2.95, "y": -0.45, "yaw": 90.0},   # Position 1: 左壁沿い、上向き
            {"x": 2.3, "y": 4.75, "yaw": 0.0}       # Position 2: 上壁沿い、右向き
        ]
        
        # プロット要素
        self.fig = None
        self.ax = None
        self.vehicle_footprints = []
        self.measurement_texts = []
        
        # IMU mockup mode
        self.mock_mode = True
        
        # IMUセンサー
        self.imu_sensor = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.imu_offset = 0.0  # キャリブレーションオフセット
        
        # 加速度積分用（簡易位置推定）
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_update_time = None
        
        # 重力補正用（静止時の加速度基準値）
        self.accel_baseline_x = None
        self.accel_baseline_y = None
        self.baseline_samples = []
        self.baseline_ready = False
        
        # リアルタイム表示用テキスト
        self.imu_status_text = None
        self.target_pos_text = None
        self.realtime_timer = None
        
    def setup_plot(self):
        """waypoint_editor_multi_mode.pyと同じ高品質プロット設定"""
        # インタラクティブモードを有効化（リアルタイム更新に必要）
        plt.ion()
        
        self.fig, self.ax = plt.subplots(figsize=(14, 10), dpi=100)
        plt.subplots_adjust(bottom=0.20, left=0.12, right=0.98, top=0.95)
        
        # 背景グリッド（正しいワールド座標で表示）
        self.ax.imshow(grid_matrix, cmap="Greys", origin="lower", 
                      extent=[x_min, x_max, y_min, y_max], alpha=0.7)
        
        # 軸設定：正しいアスペクト比と範囲（ワールド座標）
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(x_min - 0.2, x_max + 0.2)
        self.ax.set_ylim(y_min - 0.2, y_max + 0.2)
        
        # 軸ラベル（ワールド座標表示）
        self.ax.set_xlabel(f'X [meters] - Course Width: {x_max-x_min:.1f}m', fontsize=14, fontweight='bold')
        self.ax.set_ylabel(f'Y [meters] - Course Height: {y_max-y_min:.1f}m', fontsize=14, fontweight='bold')
        self.ax.set_title('IMU Calibration System - Vehicle Footprint Method', fontsize=16, fontweight='bold')
        
        # 障害物描画（ワールド座標）
        for obs in obstacles:
            x0, y0 = obs["start"]
            x1, y1 = obs["end"]
            left = min(x0, x1)
            bottom = min(y0, y1)
            width = abs(x1 - x0)
            height = abs(y1 - y0)
            rect = Rectangle((left, bottom), width, height, 
                           facecolor="lightgreen", alpha=0.6, edgecolor='green', linewidth=2)
            self.ax.add_patch(rect)
        
        # スタートライン描画（ワールド座標）
        for line_def in start_lines:
            x0, y0 = line_def["start"]
            x1, y1 = line_def["end"]
            self.ax.plot([x0, x1], [y0, y1], "b-", linewidth=4, alpha=0.8, label='Start Lines')
        
        # パイロン描画（ワールド座標）
        for p in pylons:
            x, y = p["pos"]
            circ = Circle((x, y), radius=0.1, facecolor="darkorange", alpha=0.9, 
                         edgecolor='red', linewidth=2)
            self.ax.add_patch(circ)
    
    def draw_vehicle_footprints(self):
        """車両フットプリント（180mm×350mm長方形）を描画"""
        for i, pos in enumerate(self.footprint_positions):
            x_center = pos["x"]
            y_center = pos["y"]
            yaw_deg = pos["yaw"]
            yaw_rad = math.radians(yaw_deg)
            
            # 長方形の角を計算（車両中心基準）
            half_length = self.vehicle_length / 2
            half_width = self.vehicle_width / 2
            
            # 回転前の長方形の角（ローカル座標）
            corners_local = np.array([
                [-half_length, -half_width],  # 後左
                [half_length, -half_width],   # 前左
                [half_length, half_width],    # 前右
                [-half_length, half_width]    # 後右
            ])
            
            # 回転行列を適用
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            
            # 回転後の座標計算
            corners_rotated = corners_local @ rotation_matrix.T
            corners_world = corners_rotated + np.array([x_center, y_center])
            
            # 長方形描画
            color = 'red' if i == self.current_measurement - 1 else 'blue'
            alpha = 0.8 if i == self.current_measurement - 1 else 0.4
            
            # Polygon として描画
            footprint = Polygon(corners_world, closed=True, 
                              facecolor=color, alpha=alpha, edgecolor='black', linewidth=2)
            self.ax.add_patch(footprint)
            self.vehicle_footprints.append(footprint)
            
            # 位置ラベル（重ならないように配置）
            label = f"Pos {i+1}\n({x_center:.1f}, {y_center:.1f})\nYaw: {yaw_deg:.1f}°"
            # 文字位置をフットプリントから少し離す
            text_offset_x = 0.6 if i == 0 else -0.6  # Pos1は右、Pos2は左にオフセット
            text_offset_y = 0.3 if i == 0 else -0.3  # Pos1は上、Pos2は下にオフセット
            text = self.ax.text(x_center + text_offset_x, y_center + text_offset_y, label, 
                              ha='center', va='center', fontsize=10, fontweight='bold',
                              bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
            self.measurement_texts.append(text)
            
            # 車両前方向矢印
            arrow_length = 0.4
            dx = arrow_length * math.cos(yaw_rad)
            dy = arrow_length * math.sin(yaw_rad)
            self.ax.arrow(x_center, y_center, dx, dy, 
                         head_width=0.08, head_length=0.12, 
                         fc=color, ec='black', alpha=0.9, linewidth=1.5)
    
    def initialize_imu(self):
        """IMUセンサー初期化"""
        # キャリブレーションファイルを読み込み
        import os
        calib_file = "imu_2point_calib.json"
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r', encoding='utf-8') as f:
                    calib_data = json.load(f)
                self.imu_offset = calib_data.get('offset', 0.0)
                print(f"✓ Loaded calibration offset: {self.imu_offset:.2f}°")
            except Exception as e:
                print(f"⚠️ Failed to load calibration file: {e}")
                self.imu_offset = 0.0
        else:
            print(f"⚠️ Calibration file not found: {calib_file}")
            self.imu_offset = 0.0
        
        # IMUセンサー初期化
        try:
            from IMU_sensor_bno055 import IMUSensorBNO055
            self.imu_sensor = IMUSensorBNO055()
            self.mock_mode = False
            print("✓ IMU BNO055 initialized")
        except Exception as e:
            print(f"⚠️ IMU initialization failed: {e}")
            print("   Running in MOCK mode")
            self.mock_mode = True
        
        # 起動時に位置をPoint 1で初期化
        pos1 = self.footprint_positions[0]
        self.reset_position(pos1['x'], pos1['y'])
        print(f"✓ Initial position set to Point 1: ({pos1['x']:.2f}m, {pos1['y']:.2f}m)")
        
        # 初期化状態を確認
        print("\n" + "="*60)
        print("🔍 INITIALIZATION STATUS CHECK")
        print("="*60)
        self.print_initialization_status()
    
    def print_initialization_status(self):
        """初期化状態を詳細に表示"""
        print(f"IMU Sensor:")
        print(f"  - imu_sensor: {self.imu_sensor}")
        print(f"  - mock_mode: {self.mock_mode}")
        print(f"  - imu_offset: {self.imu_offset} (type: {type(self.imu_offset)})")
        
        print(f"\nPosition & Orientation:")
        print(f"  - current_x: {self.current_x} (type: {type(self.current_x)})")
        print(f"  - current_y: {self.current_y} (type: {type(self.current_y)})")
        print(f"  - current_yaw: {self.current_yaw} (type: {type(self.current_yaw)})")
        
        print(f"\nVelocity & Integration:")
        print(f"  - velocity_x: {self.velocity_x} (type: {type(self.velocity_x)})")
        print(f"  - velocity_y: {self.velocity_y} (type: {type(self.velocity_y)})")
        print(f"  - last_update_time: {self.last_update_time}")
        
        print(f"\nGravity Baseline:")
        print(f"  - accel_baseline_x: {self.accel_baseline_x}")
        print(f"  - accel_baseline_y: {self.accel_baseline_y}")
        print(f"  - baseline_ready: {self.baseline_ready}")
        print(f"  - baseline_samples: {len(self.baseline_samples)} samples collected")
        
        print(f"\nMeasurement State:")
        print(f"  - current_measurement: {self.current_measurement}/{self.max_measurements}")
        print(f"  - footprint_positions length: {len(self.footprint_positions)}")
        print(f"  - Valid index range: 0-{len(self.footprint_positions)-1}")
        print(f"  - measurement_points: {len(self.measurement_points)} points")
        
        # IMUから実際にデータを取得してテスト
        if self.imu_sensor is not None and not self.mock_mode:
            print(f"\n🧪 IMU Data Test:")
            try:
                test_data = self.imu_sensor.get_all()
                print(f"  - get_all() successful: {test_data is not None}")
                if test_data:
                    print(f"  - yaw: {test_data.get('yaw')} (type: {type(test_data.get('yaw'))})")
                    print(f"  - accel: {test_data.get('accel')} (type: {type(test_data.get('accel'))})")
                    if test_data.get('accel'):
                        print(f"    └─ accel length: {len(test_data.get('accel'))}")
                    calib_status = self.imu_sensor.get_calibration_status()
                    if calib_status:
                        print(f"  - calibration: S:{calib_status.get('sys')} G:{calib_status.get('gyro')} A:{calib_status.get('accel')} M:{calib_status.get('mag')}")
            except Exception as e:
                print(f"  - ❌ IMU data test failed: {e}")
        else:
            print(f"\n🧪 IMU Data Test: Skipped (MOCK mode)")
        
        print("="*60 + "\n")
    
    def reset_position(self, x, y):
        """位置をリセット（キャリブレーション開始時に使用）"""
        self.current_x = x
        self.current_y = y
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_update_time = None
        
        # 重力基準値もリセット（再キャリブレーション）
        self.accel_baseline_x = None
        self.accel_baseline_y = None
        self.baseline_samples = []
        self.baseline_ready = False
        
        print(f"   Position reset to: ({x:.2f}, {y:.2f})")
        print(f"   Gravity baseline will be recalibrated...")
    
    def update_position_from_accel(self, accel_data):
        """加速度から位置を簡易推定（二重積分）"""
        import time as time_module
        
        # デバッグ: 関数が呼ばれていることを確認
        if not hasattr(self, '_update_called_count'):
            self._update_called_count = 0
        self._update_called_count += 1
        if self._update_called_count <= 30:  # 最初30回まで表示
            print(f"[TRACE #{self._update_called_count}] update_position_from_accel called, accel_data={accel_data}")
        
        current_time = time_module.time()
        if self.last_update_time is None:
            self.last_update_time = current_time
            if self._update_called_count <= 30:
                print(f"[TRACE #{self._update_called_count}] First call - initializing last_update_time={current_time:.3f}")
            return
        
        dt = current_time - self.last_update_time
        if self._update_called_count <= 30:
            print(f"[TRACE #{self._update_called_count}] dt={dt:.4f}s")
            # 異常に大きなdtの場合、原因を推測
            if dt > 1.0:
                print(f"          ⚠️ WARNING: dt is very large! Timer may be running slowly.")
                print(f"          Expected: ~0.05s (20Hz), Actual: {dt:.4f}s")
        
        # 異常な時間差は無視（ただし最初の20回は無条件で許可）
        if self._update_called_count > 20:
            max_dt = 2.0  # 安定後は2秒まで
            if dt <= 0 or dt > max_dt:
                self.last_update_time = current_time
                if self._update_called_count <= 30:
                    print(f"[TRACE #{self._update_called_count}] ⚠️ Abnormal dt={dt:.4f}s (max={max_dt:.1f}s) - skipping update (count > 20)")
                return
        else:
            if self._update_called_count <= 30:
                print(f"[TRACE #{self._update_called_count}] ✓ Initial startup (count ≤ 20) - dt check bypassed (dt={dt:.4f}s)")
        
        # dtが大きすぎる場合は制限（積分誤差を防ぐ）
        if dt > 0.2:  # 200ms以上は制限
            if self._update_called_count <= 30:
                print(f"[TRACE #{self._update_called_count}] Large dt detected, clamping {dt:.4f}s -> 0.2s")
            dt = 0.2
        
        # 加速度データ取得（m/s^2単位）
        ax_ms2 = 0.0
        ay_ms2 = 0.0
        
        if self._update_called_count <= 30:
            print(f"[TRACE #{self._update_called_count}] Checking accel_data: type={type(accel_data)}, value={accel_data}")
        
        if accel_data is not None and isinstance(accel_data, (tuple, list)) and len(accel_data) >= 3:
            ax, ay, az = accel_data[0], accel_data[1], accel_data[2]
            
            if self._update_called_count <= 30:
                print(f"[TRACE #{self._update_called_count}] Valid accel data: ax={ax}, ay={ay}, az={az}")
            
            # 重力補正（簡易: z軸の重力を無視）
            # BNO055の加速度は mg (ミリg) 単位なので m/s^2 に変換
            ax_ms2 = ax * 9.81 / 1000.0
            ay_ms2 = ay * 9.81 / 1000.0
            
            # 重力基準値のキャリブレーション（静止時の加速度を記録）
            if not self.baseline_ready:
                self.baseline_samples.append((ax_ms2, ay_ms2))
                if self._update_called_count <= 25:
                    print(f"[BASELINE] Collecting sample {len(self.baseline_samples)}/20: ax={ax_ms2:+.4f}, ay={ay_ms2:+.4f}")
                if len(self.baseline_samples) >= 20:  # 20サンプル（1秒@20Hz）で平均
                    self.accel_baseline_x = sum(s[0] for s in self.baseline_samples) / len(self.baseline_samples)
                    self.accel_baseline_y = sum(s[1] for s in self.baseline_samples) / len(self.baseline_samples)
                    self.baseline_ready = True
                    print(f"[BASELINE] Gravity compensation calibrated:")
                    print(f"           X: {self.accel_baseline_x:+.4f} m/s², Y: {self.accel_baseline_y:+.4f} m/s²")
                    print(f"           Vehicle is now ready to track motion!")
                else:
                    if self._update_called_count <= 25:
                        print(f"[BASELINE] Calibrating... ({len(self.baseline_samples)}/20)")
                return  # キャリブレーション中は位置更新しない
            
            # 重力基準値を引いて、実際の動きによる加速度を取得
            ax_ms2 -= self.accel_baseline_x
            ay_ms2 -= self.accel_baseline_y
            
            # デバッグ出力（初回のみ詳細表示）
            if not hasattr(self, '_debug_accel_printed'):
                print(f"[DEBUG] Accel data: ax={ax:.2f}mg, ay={ay:.2f}mg, az={az:.2f}mg")
                print(f"        Converted: ax={ax_ms2:.4f}m/s², ay={ay_ms2:.4f}m/s² (after baseline compensation)")
                print(f"        Noise filter threshold: 0.1 m/s² (disabled for testing)")
                self._debug_accel_printed = True
            
            # ノイズフィルタ（一時的に無効化してテスト）
            # if abs(ax_ms2) < 0.1:
            #     ax_ms2 = 0.0
            # if abs(ay_ms2) < 0.1:
            #     ay_ms2 = 0.0
        else:
            # エラー時: 加速度=0として処理（速度減衰のみ適用）
            if not hasattr(self, '_debug_error_count'):
                self._debug_error_count = 0
            self._debug_error_count += 1
            if self._debug_error_count <= 5:
                print(f"[WARNING] IMU accel data error (count: {self._debug_error_count})")
                print(f"          accel_data type: {type(accel_data)}, value: {accel_data}")
                if accel_data is not None:
                    print(f"          Is tuple/list? {isinstance(accel_data, (tuple, list))}")
                    if isinstance(accel_data, (tuple, list)):
                        print(f"          Length: {len(accel_data)}")
        
        # 速度更新（台形積分、エラー時は加速度=0）
        self.velocity_x += ax_ms2 * dt
        self.velocity_y += ay_ms2 * dt
        
        # 速度減衰（摩擦・空気抵抗の簡易モデル、エラー時も適用）
        # 20Hzなので decay=0.99 で適度な減衰（1秒で18%減少）
        decay = 0.99
        self.velocity_x *= decay
        self.velocity_y *= decay
        
        # 位置更新（エラー時も現在の速度で移動継続）
        self.current_x += self.velocity_x * dt
        self.current_y += self.velocity_y * dt
        
        # デバッグ出力（最初10回は常に表示）
        if not hasattr(self, '_debug_pos_count'):
            self._debug_pos_count = 0
        if self._debug_pos_count < 15:  # 15回まで表示（BASELINE完了後も確認できるように）
            print(f"[POS #{self._debug_pos_count+1}] dt={dt:.4f}s")
            print(f"  Raw accel: ({ax_ms2:+.4f}, {ay_ms2:+.4f}) m/s²")
            print(f"  Velocity:  ({self.velocity_x:+.4f}, {self.velocity_y:+.4f}) m/s")
            print(f"  Position:  ({self.current_x:.4f}, {self.current_y:.4f}) m")
            self._debug_pos_count += 1
        elif self._debug_pos_count == 15:
            print("[POS] Debug output complete. Position updates continue silently.")
            print(f"      Current position: ({self.current_x:.4f}, {self.current_y:.4f}) m")
            print(f"      Watch the real-time status on screen for updates.")
            self._debug_pos_count += 1
        
        self.last_update_time = current_time
    
    def update_imu_status(self):
        """IMUステータスをリアルタイムで更新（20Hz）"""
        # デバッグ: 関数が呼ばれていることを確認
        if not hasattr(self, '_status_update_count'):
            self._status_update_count = 0
            self._last_status_time = None
            print("[TRACE] update_imu_status() is being called...")
        
        # タイマー間隔を測定
        import time
        current_time = time.time()
        if self._last_status_time is not None:
            interval = current_time - self._last_status_time
            if self._status_update_count <= 5:
                print(f"[TIMER] update_imu_status called, interval={interval:.4f}s (expected: 0.05s)")
        self._last_status_time = current_time
        self._status_update_count += 1
        
        try:
            calib_status_str = ""  # 初期化（重要！）
            calib_status_str = ""
            if self.imu_sensor is not None and not self.mock_mode:
                # 実際のIMUから値を取得
                imu_data = self.imu_sensor.get_all()
                
                # デバッグ: IMUデータの内容確認（最初3回のみ）
                if self._status_update_count <= 3:
                    print(f"[IMU] get_all() returned: {imu_data}")
                
                raw_yaw = imu_data.get('yaw', 0.0)
                # yawがNoneの場合は0.0にフォールバック
                if raw_yaw is None:
                    raw_yaw = 0.0
                
                # デバッグ: yawとoffsetの値確認（最初3回のみ）
                if self._status_update_count <= 3:
                    print(f"[YAW] raw_yaw={raw_yaw} (type={type(raw_yaw)}), imu_offset={self.imu_offset} (type={type(self.imu_offset)})")
                
                # キャリブレーションオフセットを適用
                # 安全性チェック（imu_offsetのみ）
                if self.imu_offset is None:
                    print(f"[WARNING] imu_offset is None, setting to 0.0")
                    self.imu_offset = 0.0
                
                self.current_yaw = raw_yaw + self.imu_offset
                # 0-360度に正規化
                while self.current_yaw < 0:
                    self.current_yaw += 360
                while self.current_yaw >= 360:
                    self.current_yaw -= 360
                
                # 加速度データから位置を推定
                accel_data = imu_data.get('accel', None)
                self.update_position_from_accel(accel_data)
                
                # キャリブレーション状態を取得
                calib_status = self.imu_sensor.get_calibration_status()
                if calib_status:
                    calib_status_str = (f"\nCalib: S:{calib_status['sys']} "
                                      f"G:{calib_status['gyro']} "
                                      f"A:{calib_status['accel']} "
                                      f"M:{calib_status['mag']}")
            else:
                # MOCKモード: ダミー値（加速度積分もシミュレート）
                import random
                self.current_yaw = random.uniform(0, 360)
                # MOCKモードでも加速度データを生成して積分テスト
                mock_accel = (random.uniform(-200, 200), random.uniform(-200, 200), 1000)
                self.update_position_from_accel(mock_accel)
            
            # ステータステキストを更新（リアルタイムデータのみ）
            # 型チェック（エラー防止）
            if self.current_x is None:
                self.current_x = 0.0
            if self.current_y is None:
                self.current_y = 0.0
            if self.current_yaw is None:
                self.current_yaw = 0.0
            
            status_str = f"🧭 IMU Real-time Status\n"
            status_str += f"POS: ({self.current_x:.2f}, {self.current_y:.2f})m*\n"
            status_str += f"YAW: {self.current_yaw:.1f}°"
            if not self.mock_mode and self.imu_offset != 0.0:
                status_str += f" (offset: {self.imu_offset:+.1f}°)"
            status_str += f"\nMode: {'REAL' if not self.mock_mode else 'MOCK'}"
            status_str += calib_status_str
            status_str += "\n* Accel-based position estimation"
            
            # デバッグ: 画面更新が動作しているか確認（最初3回のみ）
            if self._status_update_count <= 3:
                print(f"[SCREEN] Updating display: POS=({self.current_x:.2f}, {self.current_y:.2f}), YAW={self.current_yaw:.1f}°")
            
            # 目標位置情報を別表示（左上）
            # 安全性チェック: current_measurementが範囲内か確認
            measurement_index = self.current_measurement - 1
            if measurement_index < 0 or measurement_index >= len(self.footprint_positions):
                # 範囲外の場合は最初の位置を使用
                measurement_index = 0
                if self._status_update_count <= 3:
                    print(f"[WARNING] current_measurement={self.current_measurement} out of range, using index 0")
            
            target_pos = self.footprint_positions[measurement_index]
            target_str = f"📍 Target Position {self.current_measurement}\n"
            target_str += f"POS: ({target_pos['x']:.2f}, {target_pos['y']:.2f})m\n"
            target_str += f"YAW: {target_pos['yaw']:.1f}°"
            
            if self.imu_status_text is None:
                # 初回作成（右上に表示）
                self.imu_status_text = self.ax.text(0.98, 0.98, status_str,
                                                    transform=self.ax.transAxes,
                                                    fontsize=11, fontweight='bold',
                                                    bbox=dict(boxstyle="round,pad=0.5", 
                                                             facecolor='lightgreen', alpha=0.9),
                                                    verticalalignment='top',
                                                    horizontalalignment='right')
                # 目標位置表示（左上）
                self.target_pos_text = self.ax.text(0.02, 0.92, target_str,
                                                    transform=self.ax.transAxes,
                                                    fontsize=11, fontweight='bold',
                                                    bbox=dict(boxstyle="round,pad=0.5",
                                                             facecolor='lightyellow', alpha=0.9),
                                                    verticalalignment='top',
                                                    horizontalalignment='left')
            else:
                # 更新
                self.imu_status_text.set_text(status_str)
                self.target_pos_text.set_text(target_str)
            
            # 再描画
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] IMU status update failed: {e}")
            print(f"[ERROR] Exception type: {type(e).__name__}")
            print(f"[ERROR] Full traceback:")
            traceback.print_exc()
        
        # 処理時間を測定
        if self._status_update_count <= 5:
            processing_time = time.time() - current_time
            print(f"[TIMER] update_imu_status processing time: {processing_time:.4f}s")
            # 変数の型情報も表示
            print(f"[DEBUG] Variable types and values:")
            print(f"        current_x: type={type(self.current_x)}, value={self.current_x}")
            print(f"        current_y: type={type(self.current_y)}, value={self.current_y}")
            print(f"        current_yaw: type={type(self.current_yaw)}, value={self.current_yaw}")
            print(f"        imu_offset: type={type(self.imu_offset)}, value={self.imu_offset}")
            if hasattr(self, 'imu_sensor') and self.imu_sensor is not None:
                print(f"        IMU sensor: initialized")
            else:
                print(f"        IMU sensor: None or not initialized")
    
    def start_realtime_update(self):
        """リアルタイム更新タイマーを開始（20Hz）"""
        from matplotlib.animation import FuncAnimation
        self.realtime_timer = FuncAnimation(self.fig, lambda frame: self.update_imu_status(),
                                           interval=50, cache_frame_data=False)  # 50ms = 20Hz
    
    def setup_buttons(self):
        """コントロールボタンの設定"""
        # Measure IMU ボタン
        ax_measure = plt.axes([0.15, 0.05, 0.15, 0.05])
        self.button_measure = Button(ax_measure, f"Measure IMU {self.current_measurement}")
        self.button_measure.on_clicked(self.measure_imu)
        
        # Save ボタン
        ax_save = plt.axes([0.35, 0.05, 0.12, 0.05])
        self.button_save = Button(ax_save, "Save Results")
        self.button_save.on_clicked(self.save_calibration)
        
        # Reset ボタン
        ax_reset = plt.axes([0.52, 0.05, 0.12, 0.05])
        self.button_reset = Button(ax_reset, "Reset All")
        self.button_reset.on_clicked(self.reset_measurements)
        
        # Status text
        self.status_text = self.ax.text(0.02, 0.98, "Ready for measurement", 
                                       transform=self.ax.transAxes, 
                                       fontsize=12, fontweight='bold',
                                       bbox=dict(boxstyle="round,pad=0.5", facecolor='lightblue', alpha=0.8),
                                       verticalalignment='top')
    
    def mock_imu_measurement(self):
        """IMU測定のモック（実際のBNO055実装時に置き換え）"""
        import random
        # フットプリント角度に基づいたリアルな測定値をシミュレート
        expected_yaw = self.footprint_positions[self.current_measurement - 1]["yaw"]
        # ±5度のランダムノイズを追加
        noise = random.uniform(-5.0, 5.0)
        measured_yaw = expected_yaw + noise
        
        # 0-360度範囲に正規化
        while measured_yaw < 0:
            measured_yaw += 360
        while measured_yaw >= 360:
            measured_yaw -= 360
            
        return measured_yaw
    
    def measure_imu(self, event):
        """IMU測定実行"""
        if self.current_measurement > self.max_measurements:
            self.update_status("All measurements completed. Click Save Results.")
            return
        
        # 最初の測定時に位置をリセット（Point 1の座標で初期化）
        if self.current_measurement == 1:
            pos_data = self.footprint_positions[0]
            self.reset_position(pos_data['x'], pos_data['y'])
            print(f"Position reset to Point 1: ({pos_data['x']:.2f}m, {pos_data['y']:.2f}m)")
        
        # IMU値測定
        if self.mock_mode:
            measured_yaw = self.mock_imu_measurement()
        else:
            # 実際のBNO055測定コードをここに追加
            measured_yaw = self.mock_imu_measurement()
        
        # 測定結果を保存（推定位置も記録）
        pos_data = self.footprint_positions[self.current_measurement - 1].copy()
        pos_data["measured_yaw"] = measured_yaw
        pos_data["measured_x"] = self.current_x  # 加速度積分による推定X座標
        pos_data["measured_y"] = self.current_y  # 加速度積分による推定Y座標
        pos_data["measurement_time"] = datetime.now().isoformat()
        self.measurement_points.append(pos_data)
        
        # 位置誤差を計算
        expected_x = pos_data['x']
        expected_y = pos_data['y']
        pos_error = ((self.current_x - expected_x)**2 + (self.current_y - expected_y)**2)**0.5
        print(f"Position estimation: ({self.current_x:.2f}m, {self.current_y:.2f}m) | "
              f"Expected: ({expected_x:.2f}m, {expected_y:.2f}m) | Error: {pos_error:.2f}m")
        
        # 測定結果を画面に表示
        self.display_measurement_result(measured_yaw)
        
        # 次の測定へ
        self.current_measurement += 1
        if self.current_measurement <= self.max_measurements:
            self.button_measure.label.set_text(f"Measure IMU {self.current_measurement}")
            self.update_status(f"M{self.current_measurement-1} OK. Move to Pos {self.current_measurement} -> Measure")
            self.update_vehicle_highlights()
        else:
            self.button_measure.label.set_text("All Complete")
            self.update_status("All done! Click Save Results")
        
        self.fig.canvas.draw()
    
    def display_measurement_result(self, measured_yaw):
        """測定結果をプロット上に表示"""
        pos = self.footprint_positions[self.current_measurement - 1]
        expected_yaw = pos["yaw"]
        error = measured_yaw - expected_yaw
        
        # エラー正規化（-180 to +180）
        while error > 180:
            error -= 360
        while error <= -180:
            error += 360
        
        # 結果表示テキスト
        result_text = f"IMU {self.current_measurement}:\nExpected: {expected_yaw:.1f}°\nMeasured: {measured_yaw:.1f}°\nError: {error:+.1f}°"
        
        # 既存のテキストを更新
        if len(self.measurement_texts) > self.current_measurement - 1:
            old_text = self.measurement_texts[self.current_measurement - 1]
            old_text.set_text(result_text)
            
            # エラーに応じて色を変更
            if abs(error) <= 2.0:  # 2度以内なら緑
                old_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor='lightgreen', alpha=0.9))
            elif abs(error) <= 5.0:  # 5度以内なら黄
                old_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor='yellow', alpha=0.9))
            else:  # それ以上なら赤
                old_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor='lightcoral', alpha=0.9))
    
    def update_vehicle_highlights(self):
        """現在測定する車両フットプリントをハイライト"""
        for i, footprint in enumerate(self.vehicle_footprints):
            if i == self.current_measurement - 1:  # 現在の測定位置
                footprint.set_facecolor('red')
                footprint.set_alpha(0.8)
            else:  # 完了した位置
                footprint.set_facecolor('green')
                footprint.set_alpha(0.4)
    
    def update_status(self, message):
        """ステータステキスト更新"""
        self.status_text.set_text(message)
    
    def check_measurement_validity(self):
        """測定値の妥当性チェック（外れ値検出）"""
        if len(self.measurement_points) < 2:
            return True, "Insufficient data for validation"
        
        errors = []
        for point in self.measurement_points:
            expected = point["yaw"]
            measured = point["measured_yaw"]
            error = measured - expected
            
            # エラー正規化
            while error > 180:
                error -= 360
            while error <= -180:
                error += 360
            
            errors.append(abs(error))
        
        max_error = max(errors)
        avg_error = sum(errors) / len(errors)
        
        # 外れ値判定基準
        if max_error > 15.0:  # 15度以上の誤差
            return False, f"Large measurement error detected: {max_error:.1f}°"
        
        if avg_error > 8.0:  # 平均誤差が8度以上
            return False, f"High average error: {avg_error:.1f}°"
        
        return True, f"Measurements valid. Max error: {max_error:.1f}°, Avg: {avg_error:.1f}°"
    
    def save_calibration(self, event):
        """キャリブレーション結果の保存"""
        if len(self.measurement_points) < self.max_measurements:
            self.update_status(f"Need {self.max_measurements} measurements. Currently have {len(self.measurement_points)}.")
            return
        
        # 測定値妥当性チェック
        is_valid, validation_msg = self.check_measurement_validity()
        
        # キャリブレーションファイル作成
        calibration_data = {
            "calibration_type": "vehicle_footprint",
            "measurement_count": len(self.measurement_points),
            "vehicle_dimensions": {
                "width_mm": self.vehicle_width * 1000,
                "length_mm": self.vehicle_length * 1000
            },
            "measurements": self.measurement_points,
            "validation": {
                "is_valid": is_valid,
                "message": validation_msg
            },
            "created_at": datetime.now().isoformat()
        }
        
        # オフセット計算（2点の平均）
        if len(self.measurement_points) >= 2:
            # === Yawオフセット計算 ===
            offsets = []
            for point in self.measurement_points:
                expected = point["yaw"]
                measured = point["measured_yaw"]
                offset = expected - measured
                
                # 正規化
                while offset > 180:
                    offset -= 360
                while offset <= -180:
                    offset += 360
                
                offsets.append(offset)
            
            avg_offset = sum(offsets) / len(offsets)
            calibration_data["calculated_offset"] = avg_offset
            
            # === 位置情報計算 ===
            point1 = self.measurement_points[0]
            point2 = self.measurement_points[1]
            
            # 2点間の距離（期待値）
            dx = point2["x"] - point1["x"]
            dy = point2["y"] - point1["y"]
            distance_expected = math.sqrt(dx**2 + dy**2)
            
            # 2点間の角度（期待値）
            angle_expected = math.degrees(math.atan2(dy, dx))
            while angle_expected < 0:
                angle_expected += 360
            
            # 位置情報サマリー
            calibration_data["position_info"] = {
                "reference_point": {
                    "name": "Point 1 (Recommended Start Position)",
                    "x": point1["x"],
                    "y": point1["y"],
                    "yaw": point1["yaw"]
                },
                "verification_point": {
                    "name": "Point 2 (Verification Position)",
                    "x": point2["x"],
                    "y": point2["y"],
                    "yaw": point2["yaw"]
                },
                "distance_between_points": distance_expected,
                "angle_between_points": angle_expected,
                "usage_notes": [
                    "Use Point 1 coordinates as race start position",
                    "Call reset_position(x={}, y={}) at race start".format(point1["x"], point1["y"]),
                    "Simple odometry will accumulate error over time",
                    "Recommended for short-distance courses (<10m)"
                ]
            }
            
            print(f"\n📍 Position Calibration Summary:")
            print(f"   Reference Point 1: ({point1['x']:.2f}, {point1['y']:.2f})")
            print(f"   Verification Point 2: ({point2['x']:.2f}, {point2['y']:.2f})")
            print(f"   Distance: {distance_expected:.2f}m")
            print(f"   Angle: {angle_expected:.1f}°")

        
        # ファイル保存
        filename = f"imu_calibration_footprint_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            # 標準ファイル名でもコピー（詳細版）
            standard_filename = "imu_custom_calib.json"
            with open(standard_filename, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            # imu_2point_calib.json形式でも保存（シンプル版）
            if len(self.measurement_points) >= 2:
                simple_calib = {
                    "offset": calibration_data.get("calculated_offset", 0.0),
                    "points": {
                        "point1": {
                            "x": point1["x"],
                            "y": point1["y"],
                            "yaw": point1["yaw"],
                            "measured_yaw": point1["measured_yaw"]
                        },
                        "point2": {
                            "x": point2["x"],
                            "y": point2["y"],
                            "yaw": point2["yaw"],
                            "measured_yaw": point2["measured_yaw"]
                        }
                    },
                    "created_at": datetime.now().isoformat()
                }
                simple_filename = "imu_2point_calib.json"
                with open(simple_filename, 'w', encoding='utf-8') as f:
                    json.dump(simple_calib, f, indent=2, ensure_ascii=False)
                print(f"   Also saved: {simple_filename} (simple format)")
            
            if is_valid:
                self.update_status(f"✅ SUCCESS: Calibration saved to {filename}")
                print(f"✅ CALIBRATION COMPLETED SUCCESSFULLY!")
                print(f"Files saved: {filename}, {standard_filename}, imu_2point_calib.json")
                print(f"Calculated Yaw offset: {calibration_data.get('calculated_offset', 'N/A'):.2f}°")
                if "position_info" in calibration_data:
                    print(f"Start position: ({point1['x']:.2f}, {point1['y']:.2f})")
            else:
                self.update_status(f"⚠️ WARNING: Saved with validation errors. Check {filename}")
                print(f"⚠️ CALIBRATION SAVED WITH WARNINGS!")
                print(f"Validation issue: {validation_msg}")
            
        except Exception as e:
            self.update_status(f"❌ ERROR: Failed to save calibration - {str(e)}")
            print(f"❌ Save error: {e}")
    
    def reset_measurements(self, event):
        """測定データをリセット"""
        self.measurement_points.clear()
        self.current_measurement = 1
        self.button_measure.label.set_text(f"Measure IMU {self.current_measurement}")
        self.update_status("Reset OK. Start at Pos 1")
        
        # フットプリント色をリセット
        for footprint in self.vehicle_footprints:
            footprint.set_facecolor('blue')
            footprint.set_alpha(0.4)
        
        # 最初のフットプリントをハイライト
        if self.vehicle_footprints:
            self.vehicle_footprints[0].set_facecolor('red')
            self.vehicle_footprints[0].set_alpha(0.8)
        
        # テキストをリセット
        for i, text in enumerate(self.measurement_texts):
            pos = self.footprint_positions[i]
            label = f"Pos {i+1}\n({pos['x']:.1f}, {pos['y']:.1f})\nYaw: {pos['yaw']:.1f}°"
            text.set_text(label)
            text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
        
        self.fig.canvas.draw()
    
    def run_calibration_system(self):
        """キャリブレーションシステム実行"""
        print("🎯 IMU Calibration System - Vehicle Footprint Method")
        print("="*60)
        print("Features:")
        print("• High-quality course map display (same as waypoint_editor_multi_mode.py)")
        print("• Vehicle footprint visualization (180mm × 350mm)")
        print("• 2-point IMU measurement system")
        print("• Real-time IMU status display (updates every 1 second)")
        print("• Automatic validation and error detection")
        print("• Calibration file generation")
        print(f"\nCourse Information:")
        print(f"   X range: {x_min:.1f}m - {x_max:.1f}m = {x_max-x_min:.1f}m")
        print(f"   Y range: {y_min:.1f}m - {y_max:.1f}m = {y_max-y_min:.1f}m")
        print(f"   Grid resolution: {resolution}m/pixel")
        
        # IMU初期化
        print("\nInitializing IMU sensor...")
        self.initialize_imu()
        
        self.setup_plot()
        self.draw_vehicle_footprints()
        self.setup_buttons()
        
        # リアルタイム更新開始
        print("Starting real-time IMU display (20Hz updates)...")
        print("[DEBUG] Matplotlib interactive mode:", plt.isinteractive())
        self.start_realtime_update()
        print("[DEBUG] FuncAnimation started successfully")
        
        print("\nInstructions:")
        print("1. Position your vehicle at Pos 1 (red footprint)")
        print("2. Align vehicle orientation with the footprint")
        print("3. Click 'Measure IMU 1' button")
        print("4. Move to Pos 2 and repeat")
        print("5. Click 'Save Results' when both measurements are complete")
        print("\n📊 Real-time IMU status is displayed in the top-right corner")
        
        self.update_status("Ready: Pos 1 -> Measure IMU 1")
        
        plt.show()

# メイン実行
def main():
    calibration_system = IMUCalibrationSystem()
    calibration_system.run_calibration_system()

if __name__ == "__main__":
    main()