# waypoint_editor_multi_mode.py
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.widgets import Slider, Button, RadioButtons
import json
import math
import os
from cource_map import grid_matrix, world_to_grid, grid_to_world, start_pos, goal_pos, obstacles, start_lines, pylons
from cource_map import x_min, x_max, y_min, y_max, resolution
from skimage.draw import line
from scipy.interpolate import splprep, splev
import numpy as np

# 文字化け完全修正 - 英語表示に変更
plt.rcParams['font.family'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# --- 走行モード定義（英語表記で文字化け回避、短縮名） ---
DRIVING_MODES = {
    'qualifying': {'name': 'Qualifying', 'color': 'm', 'marker': 'o', 'file_suffix': '_qualifying'},
    'qualifying_backup': {'name': 'Qualifying BU', 'color': 'c', 'marker': 's', 'file_suffix': '_qualifying_backup'},
    'final': {'name': 'Final Race', 'color': 'r', 'marker': '^', 'file_suffix': '_final'},
    'final_backup': {'name': 'Final BU', 'color': 'orange', 'marker': 'D', 'file_suffix': '_final_backup'}
}

# --- 現在のモード ---
current_mode = 'qualifying'  # デフォルトは予選用

# --- モード別 waypoint リスト ---
waypoints_dict = {mode: [] for mode in DRIVING_MODES.keys()}

# --- 最大 waypoint 数（スライダー範囲固定） ---
MAX_WAYPOINTS = 200
DEFAULT_SPEED = 100.0  # 初期速度を 100 に変更

# --- 固定ベース名（入力不要） ---
base_save_path = "waypoints"

# --- Figure 作成（大きなプロットサイズ） ---
fig, ax = plt.subplots(figsize=(14, 10), dpi=100)  # より大きなプロットサイズ
plt.subplots_adjust(bottom=0.35, left=0.12, right=0.98, top=0.95)  # スライダー用余白確保

# 背景グリッド（正しいワールド座標で表示）
ax.imshow(grid_matrix, cmap="Greys", origin="lower", 
         extent=[x_min, x_max, y_min, y_max], alpha=0.7)

# 軸設定：正しいアスペクト比と範囲（ワールド座標）
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(x_min - 0.2, x_max + 0.2)  # ワールド座標範囲
ax.set_ylim(y_min - 0.2, y_max + 0.2)  # ワールド座標範囲

# 軸ラベル（ワールド座標表示）
ax.set_xlabel(f'X [meters] - Course Width: {x_max-x_min:.1f}m', fontsize=14, fontweight='bold')
ax.set_ylabel(f'Y [meters] - Course Height: {y_max-y_min:.1f}m', fontsize=14, fontweight='bold')
ax.set_title(f'Multi-Mode Waypoint Editor - Course: {x_max-x_min:.1f}m x {y_max-y_min:.1f}m', fontsize=16, fontweight='bold')

# 障害物描画（ワールド座標）
for obs in obstacles:
    x0, y0 = obs["start"]
    x1, y1 = obs["end"]
    left = min(x0, x1)
    bottom = min(y0, y1)
    width = abs(x1 - x0)
    height = abs(y1 - y0)
    rect = Rectangle((left, bottom), width, height, color="lightgreen", alpha=0.6, edgecolor='green', linewidth=2)
    ax.add_patch(rect)

# スタートライン描画（ワールド座標）
for line_def in start_lines:
    x0, y0 = line_def["start"]
    x1, y1 = line_def["end"]
    ax.plot([x0, x1], [y0, y1], "b-", linewidth=4, alpha=0.8, label='Start Lines')

# パイロン描画（ワールド座標）
for p in pylons:
    x, y = p["pos"]
    circ = Circle((x, y), radius=0.1, color="darkorange", alpha=0.9, edgecolor='red', linewidth=2)
    ax.add_patch(circ)

# --- waypoint 描画 (モード別) ---
wp_lines = {}
wp_points_plots = {}
selected_wps = {}
yaw_arrows = []  # 矢印オブジェクトを保持

# 各モードのプロット要素を初期化
for mode, config in DRIVING_MODES.items():
    wp_lines[mode], = ax.plot([], [], "--", lw=2, color=config['color'], 
                             label=f"{config['name']} Path", alpha=0.8)
    wp_points_plots[mode], = ax.plot([], [], config['marker'], color=config['color'], 
                                    markersize=6, alpha=0.8)
    selected_wps[mode], = ax.plot([], [], config['marker'], color='red', 
                                 markersize=10, markeredgecolor='black', markeredgewidth=2)

# --- モード選択 RadioButtons（位置調整） ---
ax_radio = plt.axes([0.02, 0.65, 0.11, 0.3])
mode_labels = [config['name'] for config in DRIVING_MODES.values()]
mode_radio = RadioButtons(ax_radio, mode_labels)
mode_radio.set_active(0)  # デフォルトは予選用

# --- waypoint スライダー ---
ax_slider = plt.axes([0.2, 0.25, 0.65, 0.03])
slider = Slider(ax_slider, "Waypoint Index", 0, MAX_WAYPOINTS-1, valinit=0, valstep=1)

# --- 速度スライダー (-100～100) ---
ax_speed = plt.axes([0.2, 0.18, 0.65, 0.03])
speed_slider = Slider(ax_speed, "Speed", -100.0, 100.0, valinit=DEFAULT_SPEED)

# --- プロット更新関数 (モード対応) ---
def update_plot(val=None):
    global yaw_arrows
    # 既存の矢印を削除
    for arr in yaw_arrows:
        arr.remove()
    yaw_arrows = []

    # 全モードのプロットを更新（選択モードのみ表示）
    for mode in DRIVING_MODES.keys():
        waypoints = waypoints_dict[mode]
        
        if mode == current_mode and waypoints:
            # 現在のモードのみ表示
            # グリッド座標をワールド座標に変換
            xs = [grid_to_world(wp["x"], wp["y"])[0] for wp in waypoints]  # ワールドX座標
            ys = [grid_to_world(wp["x"], wp["y"])[1] for wp in waypoints]  # ワールドY座標
            wp_points_plots[mode].set_data(xs, ys)

            # --- 曲線補間（purepursuit風） ---
            if len(xs) >= 3:
                try:
                    tck, u = splprep([xs, ys], s=0)
                    unew = np.linspace(0, 1, max(100, len(xs)*10))
                    out = splev(unew, tck)
                    wp_lines[mode].set_data(out[0], out[1])
                except:
                    wp_lines[mode].set_data(xs, ys)
            else:
                wp_lines[mode].set_data(xs, ys)
        else:
            # 現在のモード以外は非表示
            wp_lines[mode].set_data([], [])
            wp_points_plots[mode].set_data([], [])
    
    # 現在のモードの選択されたwaypointを表示
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    
    # 全モードの選択状態をリセット
    for mode in DRIVING_MODES.keys():
        selected_wps[mode].set_data([], [])
    
    if 0 <= idx < len(current_waypoints):
        # 現在のモードのみ選択表示（ワールド座標に変換）
        world_x, world_y = grid_to_world(current_waypoints[idx]["x"], current_waypoints[idx]["y"])
        selected_wps[current_mode].set_data([world_x], [world_y])
        speed_slider.set_val(current_waypoints[idx]["v"])
    else:
        speed_slider.set_val(DEFAULT_SPEED)

    # --- yaw矢印描画 (選択したモードのみ、ワールド座標) ---
    arrow_length = 0.5  # ワールド座標で50cm
    mode_waypoints = waypoints_dict[current_mode]
    mode_config = DRIVING_MODES[current_mode]
    
    for wp in mode_waypoints:
        if "yaw" in wp:
            # グリッド座標をワールド座標に変換
            world_x, world_y = grid_to_world(wp["x"], wp["y"])
            yaw_rad = math.radians(wp["yaw"])
            dx = arrow_length * math.cos(yaw_rad)
            dy = arrow_length * math.sin(yaw_rad)
            arr = ax.arrow(world_x, world_y, dx, dy, head_width=0.1, head_length=0.15, 
                          fc=mode_config['color'], ec=mode_config['color'], alpha=0.7)
            yaw_arrows.append(arr)

    fig.canvas.draw_idle()

# --- モード変更関数 ---
def change_mode(label):
    global current_mode
    # ラベルからモードを取得
    for mode, config in DRIVING_MODES.items():
        if config['name'] == label:
            current_mode = mode
            break
    
    print(f"Mode changed to: {DRIVING_MODES[current_mode]['name']}")
    update_plot()
    
mode_radio.on_clicked(change_mode)
slider.on_changed(update_plot)

# --- 速度スライダー更新 (モード対応) ---
def update_speed(val):
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        current_waypoints[idx]["v"] = speed_slider.val
        print(f"{DRIVING_MODES[current_mode]['name']} - Waypoint {idx} speed set to {speed_slider.val:.2f}")

speed_slider.on_changed(update_speed)

MIN_TURN_RADIUS = 0.7  # [m]

def calc_circle(x1, y1, x2, y2, x3, y3):
    # 3点から円の中心と半径を計算
    temp = x2**2 + y2**2
    bc = (x1**2 + y1**2 - temp) / 2.0
    cd = (temp - x3**2 - y3**2) / 2.0
    det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
    if abs(det) < 1e-6:
        return None, None, float('inf')  # 直線
    cx = (bc*(y2 - y3) - cd*(y1 - y2)) / det
    cy = ((x1 - x2)*cd - (x2 - x3)*bc) / det
    r = math.sqrt((cx - x1)**2 + (cy - y1)**2)
    return cx, cy, r

# --- マウスクリックで waypoint 追加 ---
def is_obstacle_or_wall(x, y):
    # グリッド座標が障害物や壁（値1）ならTrue
    if 0 <= y < grid_matrix.shape[0] and 0 <= x < grid_matrix.shape[1]:
        return grid_matrix[y, x] == 1
    return True  # 範囲外は障害物扱い

def is_pylon(x, y):
    # パイロン中心から半径2以内ならTrue
    for p in pylons:
        px, py = world_to_grid(*p["pos"])
        if math.hypot(x - px, y - py) <= 2:
            return True
    return False

def is_valid_segment(x1, y1, x2, y2):
    # 2点間の直線上に障害物・壁・パイロンがないか判定
    rr, cc = line(y1, x1, y2, x2)
    for xi, yi in zip(cc, rr):
        if is_obstacle_or_wall(xi, yi) or is_pylon(xi, yi):
            return False
    return True

def onclick(event):
    if event.inaxes != ax:
        return
    
    current_waypoints = waypoints_dict[current_mode]
    if len(current_waypoints) >= MAX_WAYPOINTS:
        print(f"Maximum {MAX_WAYPOINTS} waypoints reached for {DRIVING_MODES[current_mode]['name']}")
        return
    
    # ワールド座標からグリッド座標に変換
    world_x, world_y = event.xdata, event.ydata
    x_click, y_click = world_to_grid(world_x, world_y)

    # 障害物・壁・パイロン上は不可
    if is_obstacle_or_wall(x_click, y_click):
        print("Waypoint rejected: on obstacle or wall")
        return
    if is_pylon(x_click, y_click):
        print("Waypoint rejected: on pylon")
        return
    
    # 直前のwaypointから壁・障害物・パイロンをまたぐ場合も不可
    if current_waypoints:
        x_prev, y_prev = current_waypoints[-1]["x"], current_waypoints[-1]["y"]
        if not is_valid_segment(x_prev, y_prev, x_click, y_click):
            print("Waypoint rejected: crosses obstacle, wall, or pylon")
            return

    # 最小回転半径チェック
    if len(current_waypoints) >= 2:
        x1, y1 = current_waypoints[-2]["x"], current_waypoints[-2]["y"]
        x2, y2 = current_waypoints[-1]["x"], current_waypoints[-1]["y"]
        x3, y3 = x_click, y_click
        wx1, wy1 = grid_to_world(x1, y1)
        wx2, wy2 = grid_to_world(x2, y2)
        wx3, wy3 = grid_to_world(x3, y3)
        _, _, radius = calc_circle(wx1, wy1, wx2, wy2, wx3, wy3)
        if radius < MIN_TURN_RADIUS:
            print(f"Waypoint rejected: turn radius {radius:.2f}m < {MIN_TURN_RADIUS}m")
            return

    current_waypoints.append({"x": x_click, "y": y_click, "v": DEFAULT_SPEED})
    print(f"Added waypoint to {DRIVING_MODES[current_mode]['name']}: ({x_click}, {y_click})")
    update_plot()

cid = fig.canvas.mpl_connect('button_press_event', onclick)

# --- Clear ボタン (モード対応) ---
ax_clear = plt.axes([0.05, 0.05, 0.1, 0.04])
button_clear = Button(ax_clear, "Clear Mode")

def clear(event):
    current_waypoints = waypoints_dict[current_mode]
    current_waypoints.clear()
    slider.set_val(0)
    print(f"Cleared all waypoints for {DRIVING_MODES[current_mode]['name']}")
    update_plot()

button_clear.on_clicked(clear)

# --- Save ボタン (モード対応) ---
ax_save = plt.axes([0.17, 0.05, 0.1, 0.04])
button_save = Button(ax_save, "Save Mode")

def save_waypoints(event):
    current_waypoints = waypoints_dict[current_mode]
    if not current_waypoints:
        print(f"No waypoints to save for {DRIVING_MODES[current_mode]['name']}")
        return

    # yaw を計算して追加
    for i in range(len(current_waypoints)):
        if i < len(current_waypoints) - 1:
            dx = current_waypoints[i+1]["x"] - current_waypoints[i]["x"]
            dy = current_waypoints[i+1]["y"] - current_waypoints[i]["y"]
            yaw_deg = math.degrees(math.atan2(dy, dx))
        else:
            yaw_deg = current_waypoints[i-1]["yaw"] if i > 0 else 0.0
        current_waypoints[i]["yaw"] = yaw_deg

    # モード別ファイル名で保存
    save_path = f"{base_save_path}{DRIVING_MODES[current_mode]['file_suffix']}.json"
    with open(save_path, "w", encoding='utf-8') as f:
        json.dump(current_waypoints, f, indent=2, ensure_ascii=False)
    print(f"{DRIVING_MODES[current_mode]['name']}: {len(current_waypoints)} points saved to {save_path}")

button_save.on_clicked(save_waypoints)

# --- Delete ボタン ---
ax_delete = plt.axes([0.29, 0.05, 0.1, 0.04])
button_delete = Button(ax_delete, "Delete WP")

def delete_waypoint(event):
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        del current_waypoints[idx]
        slider.set_val(max(0, min(slider.val, len(current_waypoints)-1)))
        update_plot()
        print(f"{DRIVING_MODES[current_mode]['name']}: Waypoint {idx} deleted")

button_delete.on_clicked(delete_waypoint)

# --- Save All ボタン (全モード保存) ---
ax_save_all = plt.axes([0.41, 0.05, 0.12, 0.04])
button_save_all = Button(ax_save_all, "Save All")

def save_all_modes(event):
    total_saved = 0
    for mode in DRIVING_MODES.keys():
        waypoints = waypoints_dict[mode]
        if waypoints:
            # yaw 計算
            for i in range(len(waypoints)):
                if i < len(waypoints) - 1:
                    dx = waypoints[i+1]["x"] - waypoints[i]["x"]
                    dy = waypoints[i+1]["y"] - waypoints[i]["y"]
                    yaw_deg = math.degrees(math.atan2(dy, dx))
                else:
                    yaw_deg = waypoints[i-1]["yaw"] if i > 0 else 0.0
                waypoints[i]["yaw"] = yaw_deg
            
            # 保存
            save_path = f"{base_save_path}{DRIVING_MODES[mode]['file_suffix']}.json"
            with open(save_path, "w", encoding='utf-8') as f:
                json.dump(waypoints, f, indent=2, ensure_ascii=False)
            total_saved += len(waypoints)
            print(f"{DRIVING_MODES[mode]['name']}: {len(waypoints)} points -> {save_path}")
    
    print(f"All modes saved! Total waypoints: {total_saved}")

button_save_all.on_clicked(save_all_modes)

# --- Waypoints 読み込み ---
def load_waypoints():
    global waypoints_dict
    loaded_count = 0
    
    for mode, config in DRIVING_MODES.items():
        file_path = f"{base_save_path}{config['file_suffix']}.json"
        try:
            with open(file_path, "r", encoding='utf-8') as f:
                waypoints_dict[mode] = json.load(f)
            loaded_count += len(waypoints_dict[mode])
            print(f"{config['name']}: {len(waypoints_dict[mode])} points loaded from {file_path}")
        except FileNotFoundError:
            waypoints_dict[mode] = []
            print(f"{config['name']}: {file_path} not found, starting empty")
    
    update_plot()
    print(f"Total loaded waypoints: {loaded_count}")

# 起動時にロード
load_waypoints()

# === STARTUP INFORMATION AND FEATURES ===
print(f"\n{'='*60}")
print(f"    MULTI-MODE WAYPOINT EDITOR - FEATURES & USAGE")
print(f"{'='*60}")

print(f"\nCOURSE INFORMATION:")
print(f"   X range: {x_min:.1f}m - {x_max:.1f}m = {x_max-x_min:.1f}m")
print(f"   Y range: {y_min:.1f}m - {y_max:.1f}m = {y_max-y_min:.1f}m")
print(f"   Grid resolution: {resolution}m/pixel")
print(f"   Grid size: {grid_matrix.shape[1]} x {grid_matrix.shape[0]} pixels")

print(f"\nAVAILABLE DRIVING MODES:")
for mode, config in DRIVING_MODES.items():
    print(f"   {config['name']} ({config['color']} {config['marker']})")

print(f"\nCURRENT MODE: {DRIVING_MODES[current_mode]['name']}")

print(f"\nFEATURES & CONTROLS:")
print(f"   • MOUSE CLICK: Add waypoint to current mode")
print(f"   • RADIO BUTTONS: Switch between driving modes")
print(f"   • WAYPOINT SLIDER: Select waypoint for editing")
print(f"   • SPEED SLIDER: Adjust selected waypoint speed (-100 to 100)")
print(f"   • CLEAR MODE: Delete all waypoints in current mode")
print(f"   • SAVE MODE: Save current mode waypoints with auto-calculated yaw")
print(f"   • DELETE WP: Remove selected waypoint")
print(f"   • SAVE ALL: Save all modes simultaneously")

print(f"\nAUTO-VALIDATION FEATURES:")
print(f"   • Obstacle/Wall collision detection")
print(f"   • Pylon collision avoidance")
print(f"   • Path obstacle crossing prevention")
print(f"   • Minimum turn radius check ({MIN_TURN_RADIUS}m)")
print(f"   • Auto yaw calculation on save")

print(f"\nFILE MANAGEMENT:")
print(f"   • Auto-load existing waypoint files on startup")
print(f"   • Files: waypoints_[mode].json format")
print(f"   • JSON format: x, y (grid coords), v (speed), yaw (degrees)")

print(f"\nVISUAL ELEMENTS:")
print(f"   • World coordinates display (meters)")
print(f"   • Course obstacles (green rectangles)")
print(f"   • Pylons (orange circles)")
print(f"   • Start lines (blue lines)")
print(f"   • Mode-specific waypoint paths and markers")
print(f"   • Yaw direction arrows")
print(f"   • Selected waypoint highlighting")

print(f"\n{'='*60}")
print(f"Ready to use! Click on the course to add waypoints.")

plt.legend()
plt.show()