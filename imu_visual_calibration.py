# imu_visual_calibration.py - マップ表示付きIMUキャリブレーション
import json
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
import numpy as np
from platform_detector import is_raspberry_pi

class IMUVisualCalibration:
    """マップ表示付きIMUキャリブレーション"""
    
    def __init__(self, imu_driver=None, waypoint_file="quarify.json", config_file="config.json"):
        self.imu_driver = imu_driver
        self.waypoint_file = waypoint_file
        self.calibration_file = "imu_visual_calib.json"
        self.offset = 0.0
        
        # Waypoint読み込み
        self.load_waypoints()
        
        # 設定読み込み
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            self.imu_config = config.get('sensors', {}).get('imu', {})
        except Exception as e:
            print(f"Config file error: {e}")
            self.imu_config = {}
        
        # キャリブレーション状態
        self.calibration_points = []
        self.selected_points = []
        self.fig = None
        self.ax = None
    
    def load_waypoints(self):
        """Waypointデータ読み込み"""
        try:
            with open(self.waypoint_file, 'r') as f:
                self.waypoints = json.load(f)
            print(f"✓ Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}")
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            self.waypoints = []
    
    def get_current_yaw(self):
        """現在のヨー角を取得（ハードウェア/モック自動切り替え）"""
        if self.imu_driver:
            return self.imu_driver.get_yaw()
        else:
            # モック環境では固定値
            return 0.0
    
    def create_course_map(self):
        """コースマップを作成・表示"""
        if not self.waypoints:
            print("Waypoints not available for map display")
            return False
        
        # Figure作成
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.suptitle('IMU Calibration - Course Map', fontsize=14, fontweight='bold')
        
        # Waypoint座標抽出
        x_coords = [wp['x'] for wp in self.waypoints]
        y_coords = [wp['y'] for wp in self.waypoints]
        
        # コース描画
        self.ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Course Path')
        self.ax.scatter(x_coords, y_coords, c='lightblue', s=20, alpha=0.6, label='Waypoints')
        
        # スタート・ゴール地点
        self.ax.scatter(x_coords[0], y_coords[0], c='green', s=100, marker='o', 
                       label='START', edgecolors='darkgreen', linewidth=2)
        self.ax.scatter(x_coords[-1], y_coords[-1], c='red', s=100, marker='s', 
                       label='GOAL', edgecolors='darkred', linewidth=2)
        
        # 軸設定
        self.ax.set_xlabel('X coordinate')
        self.ax.set_ylabel('Y coordinate')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()
        self.ax.set_aspect('equal', adjustable='box')
        
        # マージン追加
        x_margin = (max(x_coords) - min(x_coords)) * 0.1
        y_margin = (max(y_coords) - min(y_coords)) * 0.1
        self.ax.set_xlim(min(x_coords) - x_margin, max(x_coords) + x_margin)
        self.ax.set_ylim(min(y_coords) - y_margin, max(y_coords) + y_margin)
        
        # 特徴点候補を表示
        self.highlight_landmark_candidates()
        
        # マウスクリックイベント
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        # 操作説明
        self.ax.text(0.02, 0.98, 
                    'INSTRUCTIONS:\n'
                    '1. Click on map to select calibration points\n'
                    '2. Position vehicle toward selected direction\n'
                    '3. Use buttons below to record measurements',
                    transform=self.ax.transAxes, 
                    verticalalignment='top',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                    fontsize=9)
        
        return True
    
    def highlight_landmark_candidates(self):
        """特徴点候補をハイライト表示"""
        if len(self.waypoints) < 10:
            return
        
        # コース上の特徴的なポイントを自動抽出
        landmarks = []
        
        # スタート地点
        landmarks.append({
            'point': (self.waypoints[0]['x'], self.waypoints[0]['y']),
            'name': 'START',
            'angle': 0
        })
        
        # ゴール地点  
        landmarks.append({
            'point': (self.waypoints[-1]['x'], self.waypoints[-1]['y']),
            'name': 'GOAL',
            'angle': 180
        })
        
        # 曲率の高い地点（コーナー）を抽出
        n_points = len(self.waypoints)
        for i in range(10, n_points-10, 20):  # 20ポイントごとにサンプリング
            # 前後の点から曲率を計算
            prev_wp = self.waypoints[i-5]
            curr_wp = self.waypoints[i]
            next_wp = self.waypoints[i+5]
            
            # ベクトル計算
            v1 = np.array([curr_wp['x'] - prev_wp['x'], curr_wp['y'] - prev_wp['y']])
            v2 = np.array([next_wp['x'] - curr_wp['x'], next_wp['y'] - curr_wp['y']])
            
            # 角度変化を計算
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                angle_change = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
                
                # 曲率が高い地点を特徴点候補とする
                if angle_change > 0.5:  # 約30度以上の角度変化
                    landmarks.append({
                        'point': (curr_wp['x'], curr_wp['y']),
                        'name': f'CORNER_{len(landmarks)-1}',
                        'angle': math.degrees(math.atan2(v2[1], v2[0]))
                    })
        
        # 特徴点候補を表示
        for landmark in landmarks:
            x, y = landmark['point']
            self.ax.scatter(x, y, c='orange', s=80, marker='^', 
                          edgecolors='darkorange', linewidth=2, 
                          label='Landmark Candidate' if landmark == landmarks[0] else "")
            
            # ラベル表示
            self.ax.annotate(landmark['name'], 
                           (x, y), xytext=(5, 5), 
                           textcoords='offset points',
                           fontsize=8, fontweight='bold',
                           bbox=dict(boxstyle="round,pad=0.2", facecolor="orange", alpha=0.7))
        
        self.landmark_candidates = landmarks
    
    def on_map_click(self, event):
        """マップクリック時の処理"""
        if event.inaxes != self.ax:
            return
        
        if len(self.selected_points) >= 2:
            print("Already selected 2 calibration points. Clear and start again.")
            return
        
        # クリック位置
        click_x, click_y = event.xdata, event.ydata
        
        # クリック位置を記録
        point_num = len(self.selected_points) + 1
        self.selected_points.append({
            'x': click_x,
            'y': click_y,
            'name': f'CAL_POINT_{point_num}'
        })
        
        # 選択点を表示
        self.ax.scatter(click_x, click_y, c='red', s=150, marker='*', 
                       edgecolors='darkred', linewidth=2)
        self.ax.annotate(f'Calibration Point {point_num}', 
                        (click_x, click_y), xytext=(10, 10), 
                        textcoords='offset points',
                        fontsize=10, fontweight='bold', color='red',
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="red", alpha=0.2))
        
        # 2点選択完了時
        if len(self.selected_points) == 2:
            self.show_direction_line()
            self.create_control_buttons()
        
        self.fig.canvas.draw()
        
        print(f"Selected calibration point {point_num}: ({click_x:.1f}, {click_y:.1f})")
    
    def show_direction_line(self):
        """2点間の方向線を表示"""
        if len(self.selected_points) != 2:
            return
        
        p1 = self.selected_points[0]
        p2 = self.selected_points[1]
        
        # 方向ベクトル
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        
        # 角度計算
        angle_deg = math.degrees(math.atan2(dy, dx))
        if angle_deg < 0:
            angle_deg += 360
        
        # 方向線を描画
        self.ax.annotate('', xy=(p2['x'], p2['y']), xytext=(p1['x'], p1['y']),
                        arrowprops=dict(arrowstyle='->', color='red', lw=3))
        
        # 角度表示
        mid_x = (p1['x'] + p2['x']) / 2
        mid_y = (p1['y'] + p2['y']) / 2
        self.ax.text(mid_x, mid_y, f'{angle_deg:.1f}°', 
                    fontsize=12, fontweight='bold', color='red',
                    ha='center', va='center',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        self.calibration_angle = math.radians(angle_deg)
        print(f"Direction vector: {angle_deg:.1f}° (from Point 1 to Point 2)")
    
    def create_control_buttons(self):
        """制御ボタンを作成"""
        # ボタン用スペース作成
        plt.subplots_adjust(bottom=0.2)
        
        # ボタン配置
        ax_measure = plt.axes([0.1, 0.05, 0.15, 0.04])
        ax_clear = plt.axes([0.3, 0.05, 0.15, 0.04])
        ax_save = plt.axes([0.5, 0.05, 0.15, 0.04])
        ax_test = plt.axes([0.7, 0.05, 0.15, 0.04])
        
        # ボタン作成
        self.btn_measure = Button(ax_measure, 'Measure IMU')
        self.btn_clear = Button(ax_clear, 'Clear Points')
        self.btn_save = Button(ax_save, 'Save Calib')
        self.btn_test = Button(ax_test, 'Test Calib')
        
        # ボタンイベント
        self.btn_measure.on_clicked(self.measure_imu)
        self.btn_clear.on_clicked(self.clear_points)
        self.btn_save.on_clicked(self.save_calibration)
        self.btn_test.on_clicked(self.test_calibration)
    
    def measure_imu(self, event):
        """IMU測定実行"""
        if len(self.selected_points) != 2:
            print("Please select 2 calibration points first")
            return
        
        print("\n=== IMU Measurement ===")
        print("Position vehicle FROM Point 1 TO Point 2 direction")
        input("Press Enter when vehicle is positioned correctly...")
        
        # 測定実行
        print("Measuring... (3 seconds stabilization)")
        time.sleep(3)
        
        measured_yaw = self.get_current_yaw()
        expected_yaw = self.calibration_angle
        
        # オフセット計算
        self.offset = expected_yaw - measured_yaw
        
        # -π to π の範囲に正規化
        while self.offset > math.pi:
            self.offset -= 2 * math.pi
        while self.offset < -math.pi:
            self.offset += 2 * math.pi
        
        print(f"Measured yaw: {math.degrees(measured_yaw):.2f}°")
        print(f"Expected yaw: {math.degrees(expected_yaw):.2f}°")
        print(f"Calculated offset: {math.degrees(self.offset):.2f}°")
        
        # 結果をマップに表示
        result_text = f"Offset: {math.degrees(self.offset):.2f}°"
        self.ax.text(0.02, 0.85, result_text, 
                    transform=self.ax.transAxes,
                    fontsize=12, fontweight='bold', color='blue',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        self.fig.canvas.draw()
    
    def clear_points(self, event):
        """選択点をクリア"""
        self.selected_points = []
        self.calibration_angle = 0
        
        # マップを再描画
        self.ax.clear()
        self.create_course_map()
        
        print("Calibration points cleared")
    
    def save_calibration(self, event):
        """キャリブレーションデータ保存"""
        if not hasattr(self, 'offset'):
            print("No calibration data to save. Please measure first.")
            return
        
        # キャリブレーションデータ
        calib_data = {
            'offset': self.offset,
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'method': 'visual_map_based',
            'calibration_points': {
                'point1': self.selected_points[0],
                'point2': self.selected_points[1],
                'direction_angle': math.degrees(self.calibration_angle)
            },
            'accuracy': 'visual_map_precision'
        }
        
        try:
            with open(self.calibration_file, 'w') as f:
                json.dump(calib_data, f, indent=2)
            print(f"✓ Visual calibration saved to {self.calibration_file}")
            
            # 保存成功をマップに表示
            self.ax.text(0.02, 0.78, "Calibration Saved!", 
                        transform=self.ax.transAxes,
                        fontsize=12, fontweight='bold', color='green',
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
            self.fig.canvas.draw()
            
        except Exception as e:
            print(f"Save error: {e}")
    
    def test_calibration(self, event):
        """キャリブレーション動作テスト"""
        if not hasattr(self, 'offset'):
            print("No calibration to test. Please measure and save first.")
            return
        
        print("\n=== Calibration Test ===")
        print("Testing calibrated yaw reading...")
        
        raw_yaw = self.get_current_yaw()
        calibrated_yaw = self.calibrate_yaw(raw_yaw)
        
        print(f"Raw yaw: {math.degrees(raw_yaw):.2f}°")
        print(f"Calibrated yaw: {math.degrees(calibrated_yaw):.2f}°")
        print(f"Applied offset: {math.degrees(self.offset):.2f}°")
    
    def load_calibration(self):
        """キャリブレーションデータ読み込み"""
        try:
            with open(self.calibration_file, 'r') as f:
                calib_data = json.load(f)
            
            self.offset = calib_data.get('offset', 0.0)
            
            print(f"✓ Visual calibration loaded")
            print(f"  Method: {calib_data.get('method', 'unknown')}")
            print(f"  Date: {calib_data.get('calibration_date', 'unknown')}")
            
            if 'calibration_points' in calib_data:
                points = calib_data['calibration_points']
                print(f"  Direction: {points.get('direction_angle', 0):.1f}°")
            
            return True
            
        except FileNotFoundError:
            print(f"⚠️ Calibration file {self.calibration_file} not found")
            return False
        except Exception as e:
            print(f"Load error: {e}")
            return False
    
    def calibrate_yaw(self, raw_yaw):
        """ヨー角にキャリブレーションを適用"""
        calibrated = raw_yaw + self.offset
        
        # -π to π の範囲に正規化
        while calibrated > math.pi:
            calibrated -= 2 * math.pi
        while calibrated < -math.pi:
            calibrated += 2 * math.pi
            
        return calibrated
    
    def run_interactive_calibration(self):
        """インタラクティブキャリブレーション実行"""
        print("=== Visual Map-Based IMU Calibration ===")
        
        if not self.create_course_map():
            print("Cannot create course map. Check waypoint file.")
            return False
        
        print("\nInstructions:")
        print("1. Click on the map to select 2 calibration points")
        print("2. Position your vehicle from Point 1 toward Point 2")
        print("3. Click 'Measure IMU' to record the measurement")
        print("4. Click 'Save Calib' to save the calibration")
        print("5. Close the window when done")
        
        plt.show()
        
        return True

# 使用例とテスト
if __name__ == "__main__":
    print("=== Visual IMU Calibration System ===")
    
    # IMUドライバーの初期化（可能であれば）
    imu_driver = None
    try:
        if is_raspberry_pi():
            from bno055_imu_driver import BNO055IMUDriver
            imu_driver = BNO055IMUDriver()
            print("✓ BNO055 driver loaded")
        else:
            print("⚠️ Mock mode (PC environment)")
    except Exception as e:
        print(f"IMU driver error: {e}")
    
    # 視覚的キャリブレーション実行
    visual_calibrator = IMUVisualCalibration(imu_driver)
    
    choice = input("Select mode: [c]alibrate / [l]oad_test: ").lower()
    
    if choice == 'c':
        visual_calibrator.run_interactive_calibration()
    elif choice == 'l':
        if visual_calibrator.load_calibration():
            # テストモード
            print("Test mode - showing calibrated yaw readings...")
            try:
                while True:
                    if imu_driver:
                        raw_yaw = imu_driver.get_yaw()
                        calibrated_yaw = visual_calibrator.calibrate_yaw(raw_yaw)
                        print(f"Raw: {math.degrees(raw_yaw):6.2f}° → Calibrated: {math.degrees(calibrated_yaw):6.2f}°")
                    else:
                        print("Mock mode: no real sensor data")
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nTest completed")
    else:
        print("Invalid selection")