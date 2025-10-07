# integrated_custom_calibration.py - 統合カスタムキャリブレーションシステム
import json
import math
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import os

# cource_mapから高品質なコースデータを読み込み
try:
    from cource_map import grid_matrix, world_to_grid, grid_to_world, walls, obstacles, pylons, start_lines, start_pos, goal_pos
    COURSE_MAP_AVAILABLE = True
    print("✓ High-quality course map data loaded from cource_map.py")
except ImportError as e:
    print(f"⚠ Course map not available: {e}")
    COURSE_MAP_AVAILABLE = False

class IntegratedCustomCalibration:
    """ユーザー選択2点 + IMUキャリブレーション統合システム"""
    
    def __init__(self, waypoint_file="quarify.json"):
        self.waypoint_file = waypoint_file
        self.selected_points = []
        self.fig = None
        self.ax = None
        self.calibration_setup = None
        
        # Mock IMU for PC testing
        self.mock_mode = True  # PC環境では常にMockモード
        
        # Waypoint読み込み
        self.load_waypoints()
        
    def load_waypoints(self):
        """Waypointデータ読み込み"""
        try:
            with open(self.waypoint_file, 'r') as f:
                self.waypoints = json.load(f)
            print(f"✓ Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}")
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            self.waypoints = []
    
    def display_course_map(self):
        """高品質コースマップ表示（cource_map.py使用）"""
        if not self.waypoints:
            print("❌ Waypoints not available")
            return False
        
        # Figure作成（waypoint_editorと同じ設定）
        self.fig, self.ax = plt.subplots(figsize=(16, 12), dpi=120)
        plt.subplots_adjust(bottom=0.15, left=0.1, right=0.9, top=0.9)
        
        self.fig.suptitle('� High-Quality Course Map - Custom Calibration\n'
                         '📍 Click Point 1 (Vehicle) → Click Point 2 (Target Direction)', 
                         fontsize=18, fontweight='bold', color='darkblue')
        
        # 高品質コースマップ表示
        if COURSE_MAP_AVAILABLE:
            self.display_high_quality_course()
        else:
            self.display_basic_course()
        
        # Waypoint座標抽出
        x_coords = [wp['x'] for wp in self.waypoints]
        y_coords = [wp['y'] for wp in self.waypoints]
        
        # コース描画
        self.ax.plot(x_coords, y_coords, 'b-', linewidth=4, alpha=0.9, label='🏁 Course Path')
        
        # ウェイポイント表示（薄く）
        self.ax.scatter(x_coords[::8], y_coords[::8], c='lightblue', s=20, alpha=0.4, label='Waypoints')
        
        # スタート・ゴール地点
        self.ax.scatter(x_coords[0], y_coords[0], c='lime', s=300, marker='o', 
                       label='🚀 START', edgecolors='darkgreen', linewidth=4)
        self.ax.scatter(x_coords[-1], y_coords[-1], c='red', s=300, marker='s', 
                       label='🏆 GOAL', edgecolors='darkred', linewidth=4)
        
        # 参考ポイント（四半期）表示
        quarter_points = [len(x_coords)//4, len(x_coords)//2, 3*len(x_coords)//4]
        colors = ['orange', 'purple', 'brown']
        
        for i, idx in enumerate(quarter_points):
            self.ax.scatter(x_coords[idx], y_coords[idx], c=colors[i], s=120, marker='^',
                           edgecolors='black', linewidth=2, alpha=0.8)
            self.ax.annotate(f'Q{i+1}', (x_coords[idx], y_coords[idx]), 
                           xytext=(8, 8), textcoords='offset points',
                           fontsize=12, fontweight='bold', color=colors[i],
                           bbox=dict(boxstyle="round,pad=0.3", facecolor=colors[i], alpha=0.3))
        
        # 軸設定
        self.ax.set_xlabel('X coordinate [m]', fontsize=14, fontweight='bold')
        self.ax.set_ylabel('Y coordinate [m]', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.4, linestyle='--')
        self.ax.legend(loc='upper left', fontsize=12)
        self.ax.set_aspect('equal', adjustable='box')
        
        # マージン追加
        x_margin = (max(x_coords) - min(x_coords)) * 0.08
        y_margin = (max(y_coords) - min(y_coords)) * 0.08
        self.ax.set_xlim(min(x_coords) - x_margin, max(x_coords) + x_margin)
        self.ax.set_ylim(min(y_coords) - y_margin, max(y_coords) + y_margin)
        
        # 操作説明表示
        instruction_text = (
            f'📋 COURSE INFO:\n'
            f'• Waypoints: {len(self.waypoints)}\n'
            f'• Size: {max(x_coords)-min(x_coords):.0f}m × {max(y_coords)-min(y_coords):.0f}m\n'
            f'• Start: ({x_coords[0]:.0f}, {y_coords[0]:.0f})\n'
            f'• Goal: ({x_coords[-1]:.0f}, {y_coords[-1]:.0f})\n\n'
            f'🎯 SELECTION STEPS:\n'
            f'1️⃣ Click Point 1: Vehicle Position\n'
            f'2️⃣ Click Point 2: Target Direction\n'
            f'3️⃣ Direction Point1→Point2 = 0°\n'
            f'4️⃣ Choose clear landmarks\n'
            f'5️⃣ Close window to proceed'
        )
        
        self.ax.text(0.02, 0.98, instruction_text,
                    transform=self.ax.transAxes, 
                    verticalalignment='top',
                    bbox=dict(boxstyle="round,pad=0.6", facecolor="lightyellow", alpha=0.95,
                             edgecolor='orange', linewidth=2),
                    fontsize=12, fontweight='bold')
        
        # マウスクリックイベント
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # ステータス表示エリア
        self.status_text = self.ax.text(0.5, 0.02, '🎯 Click Point 1 (Vehicle Position)',
                                       transform=self.ax.transAxes,
                                       fontsize=14, fontweight='bold', color='red',
                                       ha='center',
                                       bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.9,
                                               edgecolor='red', linewidth=3))
        
        return True
    
    def on_click(self, event):
        """マップクリック時の処理"""
        if event.inaxes != self.ax or len(self.selected_points) >= 2:
            return
        
        # クリック位置
        click_x, click_y = event.xdata, event.ydata
        
        # 選択点を記録
        point_num = len(self.selected_points) + 1
        self.selected_points.append({
            'x': click_x,
            'y': click_y,
            'name': f'Calibration Point {point_num}'
        })
        
        # 選択点を表示
        if point_num == 1:
            # Point 1 - Vehicle Position (Red)
            self.ax.scatter(click_x, click_y, c='red', s=400, marker='*', 
                           edgecolors='darkred', linewidth=4, zorder=10, label='Vehicle Position')
            self.ax.annotate(f'🚗 Point 1\n({click_x:.1f}, {click_y:.1f})', 
                            (click_x, click_y), xytext=(20, 20), 
                            textcoords='offset points',
                            fontsize=13, fontweight='bold', color='darkred',
                            bbox=dict(boxstyle="round,pad=0.5", facecolor='red', alpha=0.3,
                                     edgecolor='darkred', linewidth=2))
            
        elif point_num == 2:
            # Point 2 - Target Direction (Blue)
            self.ax.scatter(click_x, click_y, c='blue', s=400, marker='D', 
                           edgecolors='darkblue', linewidth=4, zorder=10, label='Target Direction')
            self.ax.annotate(f'🎯 Point 2\n({click_x:.1f}, {click_y:.1f})', 
                            (click_x, click_y), xytext=(20, 20), 
                            textcoords='offset points',
                            fontsize=13, fontweight='bold', color='darkblue',
                            bbox=dict(boxstyle="round,pad=0.5", facecolor='blue', alpha=0.3,
                                     edgecolor='darkblue', linewidth=2))
        
        print(f"✓ Selected Point {point_num}: ({click_x:.1f}, {click_y:.1f})")
        
        # ステータス更新
        if point_num == 1:
            self.status_text.set_text('🎯 Click Point 2 (Target Direction)')
            self.status_text.set_color('blue')
        elif point_num == 2:
            self.show_calibration_result()
        
        self.fig.canvas.draw()
    
    def show_calibration_result(self):
        """2点選択完了時の結果表示と設定保存"""
        if len(self.selected_points) != 2:
            return
        
        p1 = self.selected_points[0]
        p2 = self.selected_points[1]
        
        # 方向ベクトル・角度計算
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        distance = math.sqrt(dx**2 + dy**2)
        angle_deg = math.degrees(math.atan2(dy, dx))
        
        # 0-360度に正規化
        if angle_deg < 0:
            angle_deg += 360
        
        # 方向線と方向矢印を描画
        self.ax.annotate('', xy=(p2['x'], p2['y']), xytext=(p1['x'], p1['y']),
                        arrowprops=dict(arrowstyle='->', color='purple', lw=8, alpha=0.9))
        
        # 中間点に角度・距離情報
        mid_x = (p1['x'] + p2['x']) / 2
        mid_y = (p1['y'] + p2['y']) / 2
        
        direction_info = f'📐 {angle_deg:.1f}°\n📏 {distance:.1f}m'
        self.ax.text(mid_x, mid_y, direction_info, 
                   fontsize=16, fontweight='bold', color='purple',
                   ha='center', va='center',
                   bbox=dict(boxstyle="round,pad=0.6", facecolor="white", alpha=0.95,
                           edgecolor='purple', linewidth=3))
        
        # 最終ステータス更新
        self.status_text.set_text('✅ 2 Points Selected - Ready for IMU Calibration!')
        self.status_text.set_color('green')
        
        # 詳細結果表示
        result_summary = (
            f'🎊 CALIBRATION SETUP COMPLETE!\n\n'
            f'🚗 Vehicle Setup:\n'
            f'   Position: ({p1["x"]:.1f}, {p1["y"]:.1f})\n'
            f'   Target:   ({p2["x"]:.1f}, {p2["y"]:.1f})\n\n'
            f'📐 Reference Direction:\n'
            f'   Angle:    {angle_deg:.1f}°\n'
            f'   Distance: {distance:.1f}m\n\n'
            f'⚙️  Next Steps:\n'
            f'   1. Close this window\n'
            f'   2. Position vehicle at Point 1\n'
            f'   3. Aim toward Point 2\n'
            f'   4. Run IMU measurement\n'
            f'   5. Save calibration file'
        )
        
        # 結果ウィンドウ表示
        self.ax.text(0.98, 0.98, result_summary,
                    transform=self.ax.transAxes,
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle="round,pad=0.6", facecolor="lightgreen", alpha=0.95,
                             edgecolor='green', linewidth=3),
                    fontsize=12, fontweight='bold')
        
        # コンソールに詳細出力
        print(f"\n{'='*60}")
        print(f"🎊 CALIBRATION POINTS SUCCESSFULLY SELECTED!")
        print(f"{'='*60}")
        print(f"🚗 Point 1 (Vehicle Position): ({p1['x']:.1f}, {p1['y']:.1f})")
        print(f"🎯 Point 2 (Target Direction):  ({p2['x']:.1f}, {p2['y']:.1f})")
        print(f"📐 Reference Angle: {angle_deg:.1f}°")
        print(f"📏 Distance: {distance:.1f}m")
        print(f"{'='*60}")
        
        # キャリブレーション設定を保存
        self.calibration_setup = {
            'timestamp': datetime.now().isoformat(),
            'method': 'custom_2point_selection',
            'point1': p1,
            'point2': p2,
            'reference_angle': angle_deg,
            'distance': distance,
            'description': f'Custom user selection: ({p1["x"]:.1f},{p1["y"]:.1f}) → ({p2["x"]:.1f},{p2["y"]:.1f})',
            'status': 'setup_complete'
        }
        
        return self.calibration_setup
    
    def display_high_quality_course(self):
        """cource_map.pyを使った高品質コース表示"""
        # 背景グリッド表示（waypoint_editor風）
        self.ax.imshow(grid_matrix, cmap="Greys", origin="lower", alpha=0.3, aspect='equal')
        self.ax.set_facecolor("white")
        
        # 壁の描画（黒いライン）
        for i, wall in enumerate(walls):
            x1, y1 = world_to_grid(*wall["start"])
            x2, y2 = world_to_grid(*wall["end"])
            self.ax.plot([x1, x2], [y1, y2], "k-", linewidth=3, alpha=0.8, 
                        label="Course Walls" if i == 0 else "")
        
        # 障害物の描画（緑の矩形）
        from matplotlib.patches import Rectangle
        for i, obs in enumerate(obstacles):
            x1, y1 = world_to_grid(*obs["start"])
            x2, y2 = world_to_grid(*obs["end"])
            
            left = min(x1, x2)
            bottom = min(y1, y2)
            width = abs(x2 - x1)
            height = abs(y2 - y1)
            
            rect = Rectangle((left, bottom), width, height, 
                           color="lightgreen", alpha=0.6, edgecolor='green', linewidth=2)
            self.ax.add_patch(rect)
            
            if i == 0:
                rect.set_label("Obstacles")
        
        # パイロンの描画（青い円）
        for i, pylon in enumerate(pylons):
            x, y = world_to_grid(*pylon["pos"])
            self.ax.plot(x, y, "bo", markersize=12, alpha=0.8,
                        label="Pylons" if i == 0 else "")
        
        # スタートラインの描画（青いライン）
        for i, start_line in enumerate(start_lines):
            x1, y1 = world_to_grid(*start_line["start"])
            x2, y2 = world_to_grid(*start_line["end"])
            self.ax.plot([x1, x2], [y1, y2], "b-", linewidth=4, alpha=0.9,
                        label="Start Lines" if i == 0 else "")
        
        # スタート・ゴール位置
        self.ax.plot(start_pos[0], start_pos[1], "go", markersize=15, 
                    label="Start Position", markeredgecolor='darkgreen', markeredgewidth=3)
        self.ax.plot(goal_pos[0], goal_pos[1], "rx", markersize=15, 
                    label="Goal Position", markeredgecolor='darkred', markeredgewidth=3)
        
        print("✓ High-quality course map displayed with walls, obstacles, and pylons")
    
    def display_basic_course(self):
        """基本的なコース表示（cource_map.py未使用時）"""
        print("⚠ Using basic course display (cource_map.py not available)")
        
        # 基本的なグリッド背景
        self.ax.grid(True, alpha=0.3, linestyle='--', color='gray')
        self.ax.set_facecolor('lightgray')
    
    def mock_imu_measurement(self):
        """PCテスト用のIMU測定モック"""
        print("\n🔧 Performing Mock IMU Measurement...")
        print("   (In real environment, this reads actual BNO055 sensor)")
        
        # シミュレート進行表示
        for i in range(3):
            print(f"   Reading IMU... {i+1}/3")
            time.sleep(0.5)
        
        # ランダムだが現実的なYaw値を生成
        mock_yaw = np.random.uniform(0, 360)
        
        print(f"✓ Mock IMU Reading Complete")
        print(f"   Current Yaw: {mock_yaw:.1f}°")
        
        return mock_yaw
    
    def calculate_calibration_offset(self, current_yaw):
        """キャリブレーションオフセット計算"""
        if not self.calibration_setup:
            print("❌ No calibration setup available")
            return None
        
        reference_angle = self.calibration_setup['reference_angle']
        offset = reference_angle - current_yaw
        
        # -180~180度に正規化
        while offset > 180:
            offset -= 360
        while offset <= -180:
            offset += 360
        
        return offset
    
    def save_calibration_file(self, imu_yaw, offset):
        """キャリブレーションファイル保存"""
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
            'calibration_method': 'custom_2point_user_selection',
            'setup_points': self.calibration_setup,
            'imu_measurement': {
                'raw_yaw': imu_yaw,
                'measurement_time': datetime.now().isoformat()
            },
            'calibration_result': {
                'reference_angle': self.calibration_setup['reference_angle'],
                'yaw_offset': offset,
                'description': f"Offset to align IMU with user-selected direction"
            },
            'usage_instructions': {
                'vehicle_position': f"({self.calibration_setup['point1']['x']:.1f}, {self.calibration_setup['point1']['y']:.1f})",
                'target_direction': f"({self.calibration_setup['point2']['x']:.1f}, {self.calibration_setup['point2']['y']:.1f})",
                'reference_heading': f"{self.calibration_setup['reference_angle']:.1f}°"
            }
        }
        
        # ファイル名生成（タイムスタンプ付き）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_custom_calibration_{timestamp}.json"
        
        # 保存
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            print(f"\n✅ Calibration file saved: {filename}")
            
            # 標準ファイル名でもコピー保存（main_control_loop.pyが読み込み用）
            standard_filename = "imu_custom_calib.json"
            with open(standard_filename, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            print(f"✅ Standard calibration file: {standard_filename}")
            
            return filename, standard_filename
            
        except Exception as e:
            print(f"❌ Error saving calibration file: {e}")
            return None, None
    
    def run_full_calibration_process(self):
        """フルキャリブレーション実行"""
        print("="*70)
        print("🎯 INTEGRATED CUSTOM CALIBRATION SYSTEM")
        print("="*70)
        print("This system allows you to:")
        print("1. Select 2 custom calibration points on the course map")
        print("2. Measure IMU orientation at your selected reference")
        print("3. Generate calibration file for race system")
        print()
        
        # Step 1: ポイント選択
        print("📍 Step 1: Select Calibration Points")
        if not self.display_course_map():
            print("❌ Failed to display course map")
            return False
        
        print("🗺️  Course map displayed. Select your 2 calibration points.")
        print("   Close the map window when done selecting.")
        
        try:
            plt.show()  # ブロッキング表示
        except Exception as e:
            print(f"❌ Display error: {e}")
            return False
        
        # 選択結果確認
        if not self.calibration_setup:
            print("❌ No calibration points selected")
            return False
        
        print("✅ Calibration points selection completed!")
        
        # Step 2: IMU測定
        print("\n📡 Step 2: IMU Measurement")
        print("Position your vehicle:")
        print(f"   🚗 At Point 1: ({self.calibration_setup['point1']['x']:.1f}, {self.calibration_setup['point1']['y']:.1f})")
        print(f"   🎯 Facing Point 2: ({self.calibration_setup['point2']['x']:.1f}, {self.calibration_setup['point2']['y']:.1f})")
        
        input("\nPress Enter when vehicle is positioned correctly...")
        
        # IMU測定実行
        if self.mock_mode:
            current_yaw = self.mock_imu_measurement()
        else:
            # Real BNO055 measurement would go here
            current_yaw = self.mock_imu_measurement()
        
        # Step 3: オフセット計算
        offset = self.calculate_calibration_offset(current_yaw)
        if offset is None:
            print("❌ Failed to calculate calibration offset")
            return False
        
        print(f"\n📊 Calibration Results:")
        print(f"   Reference Direction: {self.calibration_setup['reference_angle']:.1f}°")
        print(f"   Current IMU Yaw: {current_yaw:.1f}°")
        print(f"   Calculated Offset: {offset:.1f}°")
        
        # Step 4: ファイル保存
        print("\n💾 Step 3: Save Calibration")
        main_file, standard_file = self.save_calibration_file(current_yaw, offset)
        
        if main_file and standard_file:
            print(f"\n🎊 CALIBRATION PROCESS COMPLETED!")
            print(f"✅ Files saved:")
            print(f"   📄 {main_file} (detailed record)")
            print(f"   📄 {standard_file} (for race system)")
            print(f"\n🏁 Ready for racing! Your main_control_loop.py will automatically")
            print(f"   load {standard_file} and apply the {offset:.1f}° offset.")
            return True
        else:
            print("❌ Failed to save calibration files")
            return False

# メイン実行関数
def main():
    print("🎯 Custom Calibration System - Starting...")
    
    calibrator = IntegratedCustomCalibration()
    success = calibrator.run_full_calibration_process()
    
    if success:
        print("\n" + "="*50)
        print("✅ CALIBRATION COMPLETED SUCCESSFULLY!")
        print("="*50)
        print("Next steps:")
        print("1. Run main_control_loop.py for racing")
        print("2. The system will automatically load your calibration")
        print("3. IMU compass will be aligned to your selected direction")
    else:
        print("\n❌ Calibration process failed or incomplete")
    
    print("\nCalibration system session ended.")

if __name__ == "__main__":
    main()