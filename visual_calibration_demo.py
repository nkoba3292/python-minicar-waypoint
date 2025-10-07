# visual_calibration_demo.py - マップ表示キャリブレーションのデモ
import json
import matplotlib.pyplot as plt
import numpy as np

def demo_visual_calibration():
    """ビジュアルキャリブレーションのデモンストレーション"""
    
    print("=== Visual Calibration Demo ===")
    
    # quarify.jsonからwaypointデータを読み込み
    try:
        with open('quarify.json', 'r') as f:
            waypoints = json.load(f)
        print(f"✓ Loaded {len(waypoints)} waypoints")
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        # デモ用のサンプルデータ作成
        waypoints = []
        for i in range(50):
            angle = i * 2 * np.pi / 50
            waypoints.append({
                'x': 10 * np.cos(angle),
                'y': 10 * np.sin(angle)
            })
        print("Using sample circular course")
    
    # マップ表示
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.suptitle('IMU Visual Calibration Demo\n(Click 2 points to simulate calibration)', fontsize=14)
    
    # Waypoint座標抽出
    x_coords = [wp['x'] for wp in waypoints]
    y_coords = [wp['y'] for wp in waypoints]
    
    # コース描画
    ax.plot(x_coords, y_coords, 'b-', linewidth=3, alpha=0.8, label='Course Path')
    ax.scatter(x_coords[::10], y_coords[::10], c='lightblue', s=30, alpha=0.7, label='Waypoints (every 10th)')
    
    # スタート・ゴール
    ax.scatter(x_coords[0], y_coords[0], c='green', s=150, marker='o', 
               label='START', edgecolors='darkgreen', linewidth=2)
    ax.scatter(x_coords[-1], y_coords[-1], c='red', s=150, marker='s', 
               label='GOAL', edgecolors='darkred', linewidth=2)
    
    # 軸設定
    ax.set_xlabel('X coordinate [m]')
    ax.set_ylabel('Y coordinate [m]')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_aspect('equal', adjustable='box')
    
    # 操作説明
    instruction_text = (
        "DEMO INSTRUCTIONS:\n"
        "1. Click on 2 points on the course\n"
        "2. Direction from Point 1 → Point 2 will be calculated\n"
        "3. This would be your calibration reference\n"
        "4. In real use, position vehicle in that direction\n"
        "5. Close window when done"
    )
    
    ax.text(0.02, 0.98, instruction_text,
            transform=ax.transAxes, 
            verticalalignment='top',
            bbox=dict(boxstyle="round,pad=0.5", facecolor="yellow", alpha=0.8),
            fontsize=10)
    
    # クリック処理用変数
    selected_points = []
    
    def on_click(event):
        if event.inaxes != ax or len(selected_points) >= 2:
            return
        
        x, y = event.xdata, event.ydata
        point_num = len(selected_points) + 1
        
        # 点を記録・表示
        selected_points.append({'x': x, 'y': y})
        ax.scatter(x, y, c='red', s=200, marker='*', 
                  edgecolors='darkred', linewidth=2)
        ax.annotate(f'Cal Point {point_num}', 
                   (x, y), xytext=(10, 10), 
                   textcoords='offset points',
                   fontsize=12, fontweight='bold', color='red',
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="red", alpha=0.3))
        
        print(f"Selected Point {point_num}: ({x:.1f}, {y:.1f})")
        
        # 2点選択完了時
        if len(selected_points) == 2:
            p1, p2 = selected_points[0], selected_points[1]
            
            # 方向ベクトル・角度計算
            dx = p2['x'] - p1['x']
            dy = p2['y'] - p1['y']
            angle_deg = np.degrees(np.arctan2(dy, dx))
            if angle_deg < 0:
                angle_deg += 360
            
            # 方向線描画
            ax.annotate('', xy=(p2['x'], p2['y']), xytext=(p1['x'], p1['y']),
                       arrowprops=dict(arrowstyle='->', color='red', lw=4))
            
            # 角度・距離表示
            mid_x = (p1['x'] + p2['x']) / 2
            mid_y = (p1['y'] + p2['y']) / 2
            distance = np.sqrt(dx**2 + dy**2)
            
            result_text = f"Angle: {angle_deg:.1f}°\nDistance: {distance:.1f}m"
            ax.text(mid_x, mid_y, result_text, 
                   fontsize=11, fontweight='bold', color='red',
                   ha='center', va='center',
                   bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.9))
            
            # 結果説明
            calibration_info = (
                f"CALIBRATION SETUP:\n"
                f"• Point 1: ({p1['x']:.1f}, {p1['y']:.1f})\n"
                f"• Point 2: ({p2['x']:.1f}, {p2['y']:.1f})\n"
                f"• Direction: {angle_deg:.1f}°\n"
                f"• Distance: {distance:.1f}m\n\n"
                f"Vehicle should face from Point 1 → Point 2\n"
                f"IMU measurement will be compared to {angle_deg:.1f}°"
            )
            
            ax.text(0.02, 0.02, calibration_info,
                   transform=ax.transAxes,
                   verticalalignment='bottom',
                   bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8),
                   fontsize=9, fontweight='bold')
            
            print(f"\n=== Calibration Setup Complete ===")
            print(f"Direction: Point 1 → Point 2 = {angle_deg:.1f}°")
            print(f"In real calibration:")
            print(f"1. Position vehicle at Point 1 ({p1['x']:.1f}, {p1['y']:.1f})")
            print(f"2. Face vehicle toward Point 2 ({p2['x']:.1f}, {p2['y']:.1f})")
            print(f"3. Measure IMU yaw angle")
            print(f"4. Offset = {angle_deg:.1f}° - measured_angle")
        
        fig.canvas.draw()
    
    # イベント接続
    fig.canvas.mpl_connect('button_press_event', on_click)
    
    plt.tight_layout()
    plt.show()
    
    return True

if __name__ == "__main__":
    print("Starting Visual Calibration Demo...")
    print("This demonstrates how map-based calibration would work.")
    print("In the real system, you would:")
    print("1. See your actual course map (from waypoints)")
    print("2. Click on clear landmarks or course features")  
    print("3. Position your vehicle accordingly")
    print("4. Measure and save the calibration")
    print()
    
    demo_visual_calibration()
    
    print("\nDemo completed!")
    print("The actual visual calibration system provides:")
    print("• Real waypoint course visualization")
    print("• Automatic landmark detection")
    print("• Interactive calibration point selection")
    print("• Real-time IMU measurement integration")
    print("• Automatic offset calculation and saving")