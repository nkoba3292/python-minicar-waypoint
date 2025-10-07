# custom_calibration_points.py - ユーザーカスタムキャリブレーション2点選択
import json
import math
import matplotlib.pyplot as plt
import numpy as np
from platform_detector import is_raspberry_pi

class CustomCalibrationSelector:
    """ユーザーが自由に2点を選択できるキャリブレーションシステム"""
    
    def __init__(self, waypoint_file="quarify.json"):
        self.waypoint_file = waypoint_file
        self.selected_points = []
        self.fig = None
        self.ax = None
        
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
        """コースマップを表示してポイント選択を開始"""
        if not self.waypoints:
            print("❌ Waypoints not available")
            return False
        
        # Figure作成
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.fig.suptitle('Custom Calibration Points Selection\n'
                         'Click 2 points on the course for your calibration reference', 
                         fontsize=16, fontweight='bold')
        
        # Waypoint座標抽出
        x_coords = [wp['x'] for wp in self.waypoints]
        y_coords = [wp['y'] for wp in self.waypoints]
        
        # コース描画
        self.ax.plot(x_coords, y_coords, 'b-', linewidth=3, alpha=0.8, label='Course Path')
        
        # ウェイポイント表示（薄く）
        self.ax.scatter(x_coords[::5], y_coords[::5], c='lightblue', s=15, alpha=0.5, label='Waypoints (every 5th)')
        
        # スタート・ゴール地点
        self.ax.scatter(x_coords[0], y_coords[0], c='green', s=200, marker='o', 
                       label='START', edgecolors='darkgreen', linewidth=3)
        self.ax.scatter(x_coords[-1], y_coords[-1], c='red', s=200, marker='s', 
                       label='GOAL', edgecolors='darkred', linewidth=3)
        
        # 四半期ポイント表示（参考用）
        quarter_points = [0, len(x_coords)//4, len(x_coords)//2, 3*len(x_coords)//4]
        for i, idx in enumerate(quarter_points[1:-1], 1):  # START/GOALは除く
            self.ax.scatter(x_coords[idx], y_coords[idx], c='orange', s=100, marker='^',
                           edgecolors='darkorange', linewidth=2, alpha=0.7)
            self.ax.annotate(f'Quarter {i}', (x_coords[idx], y_coords[idx]), 
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=10, fontweight='bold', color='darkorange',
                           bbox=dict(boxstyle="round,pad=0.2", facecolor="orange", alpha=0.3))
        
        # 軸設定
        self.ax.set_xlabel('X coordinate [m]', fontsize=12)
        self.ax.set_ylabel('Y coordinate [m]', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right')
        self.ax.set_aspect('equal', adjustable='box')
        
        # マージン追加
        x_margin = (max(x_coords) - min(x_coords)) * 0.05
        y_margin = (max(y_coords) - min(y_coords)) * 0.05
        self.ax.set_xlim(min(x_coords) - x_margin, max(x_coords) + x_margin)
        self.ax.set_ylim(min(y_coords) - y_margin, max(y_coords) + y_margin)
        
        # コース情報表示
        course_info = (
            f'COURSE INFORMATION:\n'
            f'• Total waypoints: {len(self.waypoints)}\n'
            f'• Course dimensions: {max(x_coords)-min(x_coords):.0f}m × {max(y_coords)-min(y_coords):.0f}m\n'
            f'• Start: ({x_coords[0]:.0f}, {y_coords[0]:.0f})\n'
            f'• Goal: ({x_coords[-1]:.0f}, {y_coords[-1]:.0f})\n\n'
            f'INSTRUCTIONS:\n'
            f'1. Click Point 1: Where you will position the vehicle\n'
            f'2. Click Point 2: Where you will aim the vehicle\n'
            f'3. Direction: Point 1 → Point 2 becomes 0° reference\n'
            f'4. Choose clear, visible landmarks for accuracy'
        )
        
        self.ax.text(0.02, 0.98, course_info,
                    transform=self.ax.transAxes, 
                    verticalalignment='top',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.9),
                    fontsize=11, fontweight='normal')
        
        # マウスクリックイベント
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # 現在の選択状態表示エリア
        self.status_text = self.ax.text(0.02, 0.02, 'Status: Click Point 1',
                                       transform=self.ax.transAxes,
                                       fontsize=12, fontweight='bold', color='blue',
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
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
        colors = ['red', 'blue']
        markers = ['*', 'D']
        
        self.ax.scatter(click_x, click_y, c=colors[point_num-1], s=300, 
                       marker=markers[point_num-1], 
                       edgecolors='black', linewidth=3, zorder=10)
        
        # ラベル表示
        self.ax.annotate(f'Point {point_num}\n({click_x:.1f}, {click_y:.1f})', 
                        (click_x, click_y), xytext=(15, 15), 
                        textcoords='offset points',
                        fontsize=12, fontweight='bold', color=colors[point_num-1],
                        bbox=dict(boxstyle="round,pad=0.4", facecolor=colors[point_num-1], alpha=0.2))
        
        print(f"Selected Point {point_num}: ({click_x:.1f}, {click_y:.1f})")
        
        # ステータス更新
        if point_num == 1:
            self.status_text.set_text('Status: Click Point 2 (aim direction)')
        elif point_num == 2:
            self.show_calibration_result()
        
        self.fig.canvas.draw()
    
    def show_calibration_result(self):
        """2点選択完了時の結果表示"""
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
        
        # 方向線描画
        self.ax.annotate('', xy=(p2['x'], p2['y']), xytext=(p1['x'], p1['y']),
                        arrowprops=dict(arrowstyle='->', color='purple', lw=5, alpha=0.8))
        
        # 角度・距離表示
        mid_x = (p1['x'] + p2['x']) / 2
        mid_y = (p1['y'] + p2['y']) / 2
        
        direction_info = f'Direction: {angle_deg:.1f}°\nDistance: {distance:.1f}m'
        self.ax.text(mid_x, mid_y, direction_info, 
                   fontsize=14, fontweight='bold', color='purple',
                   ha='center', va='center',
                   bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9,
                           edgecolor='purple', linewidth=2))
        
        # 最終ステータス更新
        self.status_text.set_text('Status: 2 points selected - Ready for calibration!')
        self.status_text.set_color('green')
        
        # 結果サマリー表示
        result_summary = (
            f'CALIBRATION SETUP COMPLETE:\n\n'
            f'Position Setup:\n'
            f'• Place vehicle at Point 1: ({p1["x"]:.1f}, {p1["y"]:.1f})\n'
            f'• Aim vehicle toward Point 2: ({p2["x"]:.1f}, {p2["y"]:.1f})\n\n'
            f'Reference Direction:\n'
            f'• Angle: {angle_deg:.1f}° (from Point 1 to Point 2)\n'
            f'• Distance: {distance:.1f}m\n\n'
            f'Next Steps:\n'
            f'1. Close this window\n'
            f'2. Position vehicle at Point 1 facing Point 2\n'
            f'3. Run IMU calibration measurement\n'
            f'4. Save calibration with {angle_deg:.1f}° as reference'
        )
        
        # 結果ウィンドウ表示エリア
        self.ax.text(0.98, 0.98, result_summary,
                    transform=self.ax.transAxes,
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.95,
                             edgecolor='green', linewidth=2),
                    fontsize=11, fontweight='bold')
        
        print(f"\n{'='*50}")
        print(f"CALIBRATION POINTS SELECTED")
        print(f"{'='*50}")
        print(f"Point 1 (Vehicle Position): ({p1['x']:.1f}, {p1['y']:.1f})")
        print(f"Point 2 (Target Direction):  ({p2['x']:.1f}, {p2['y']:.1f})")
        print(f"Reference Angle: {angle_deg:.1f}°")
        print(f"Distance: {distance:.1f}m")
        print(f"{'='*50}")
        
        # 保存用データ
        self.calibration_setup = {
            'point1': p1,
            'point2': p2,
            'reference_angle': angle_deg,
            'distance': distance,
            'description': f'Custom calibration: Point ({p1["x"]:.1f},{p1["y"]:.1f}) → ({p2["x"]:.1f},{p2["y"]:.1f})'
        }
        
        return self.calibration_setup
    
    def get_calibration_setup(self):
        """選択された校正設定を取得"""
        if hasattr(self, 'calibration_setup'):
            return self.calibration_setup
        return None
    
    def run_interactive_selection(self):
        """インタラクティブ選択実行"""
        print("=== Custom Calibration Points Selection ===")
        print("Course map will be displayed.")
        print("Click 2 points on the course to define your calibration reference.")
        print()
        
        if not self.display_course_map():
            return False
        
        print("Map displayed. Click 2 points to select calibration reference.")
        print("Close the window when you're done selecting points.")
        
        try:
            plt.show()  # ブロッキング表示
        except Exception as e:
            print(f"Display error: {e}")
            return False
        
        # 選択結果確認
        setup = self.get_calibration_setup()
        if setup:
            print("\n✅ Calibration points selection completed!")
            return setup
        else:
            print("\n❌ No calibration points selected")
            return None

# 使用例とテスト
if __name__ == "__main__":
    print("=== Custom Calibration Points Selector ===")
    print("This tool helps you select your preferred 2-point calibration reference.")
    print()
    
    selector = CustomCalibrationSelector()
    
    # インタラクティブ選択実行
    result = selector.run_interactive_selection()
    
    if result:
        print("\n📋 Summary of your selection:")
        print(f"Position vehicle at: ({result['point1']['x']:.1f}, {result['point1']['y']:.1f})")
        print(f"Aim vehicle toward: ({result['point2']['x']:.1f}, {result['point2']['y']:.1f})")
        print(f"Reference angle: {result['reference_angle']:.1f}°")
        print(f"Distance: {result['distance']:.1f}m")
        print()
        print("Use these coordinates for your actual calibration procedure.")
    else:
        print("No calibration points were selected.")
    
    print("\nCalibration points selection completed!")