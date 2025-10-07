# waypoint_editor_with_speed_yaw_100.py
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.widgets import Slider, Button, RadioButtons
import json
import math
import os
from cource_map import grid_matrix, world_to_grid, grid_to_world, start_pos, goal_pos, obstacles, start_lines, pylons
from skimage.draw import line
from scipy.interpolate import splprep, splev
import numpy as np

# 文字化け対策
plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# --- 走行モード定義 ---
DRIVING_MODES = {
    'qualifying': {'name': 'Qualifying (予選用)', 'color': 'm', 'marker': 'o', 'file_suffix': '_qualifying'},
    'qualifying_backup': {'name': 'Qualifying Backup (予選予備用)', 'color': 'c', 'marker': 's', 'file_suffix': '_qualifying_backup'},
    'final': {'name': 'Final Race (本線用)', 'color': 'r', 'marker': '^', 'file_suffix': '_final'},
    'final_backup': {'name': 'Final Backup (本線予備用)', 'color': 'orange', 'marker': 'D', 'file_suffix': '_final_backup'}
}

# --- 現在のモード ---
current_mode = 'qualifying'  # デフォルトは予選用

# --- モード別 waypoint リスト ---
waypoints_dict = {mode: [] for mode in DRIVING_MODES.keys()}

# --- 最大 waypoint 数（スライダー範囲固定） ---
MAX_WAYPOINTS = 200
DEFAULT_SPEED = 100.0  # 初期速度を 100 に変更

# --- シナリオ名入力 ---
scenario_name = input("Enter base scenario name (modes will be saved as <name>_<mode>.json): ").strip()
base_save_path = scenario_name

# --- Figure 作成 ---
fig, ax = plt.subplots(dpi=120)
plt.subplots_adjust(bottom=0.55, left=0.15)  # モード選択用にスペースを確保

# 背景グリッド（ワールド座標で正しく表示）
from cource_map import x_min, x_max, y_min, y_max, resolution

# extentでワールド座標範囲を指定
ax.imshow(grid_matrix, cmap="Greys", origin="lower", 
         extent=[x_min, x_max, y_min, y_max], aspect='equal')

# 軸設定：ワールド座標での範囲設定
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_aspect('equal', adjustable='box')

# 軸ラベルをワールド座標で表示
ax.set_xlabel(f'X [m] (範囲: {x_min:.1f}～{x_max:.1f}m = {x_max-x_min:.1f}m)', fontsize=12)
ax.set_ylabel(f'Y [m] (範囲: {y_min:.1f}～{y_max:.1f}m = {y_max-y_min:.1f}m)', fontsize=12)
ax.set_title(f'Course Map - 実際サイズ: X={x_max-x_min:.1f}m × Y={y_max-y_min:.1f}m (解像度:{resolution}m/pixel)', fontsize=14, fontweight='bold')

# 障害物描画（ワールド座標で直接）
for obs in obstacles:
    x0, y0 = obs["start"]
    x1, y1 = obs["end"]
    left = min(x0, x1)
    bottom = min(y0, y1)
    width = abs(x1 - x0)
    height = abs(y1 - y0)
    # ワールド座標で直接描画
    rect = Rectangle((left, bottom), width, height, color="lightgreen", alpha=0.5)
    ax.add_patch(rect)

# スタートライン描画（ワールド座標で直接）
for line_def in start_lines:
    x0, y0 = line_def["start"]
    x1, y1 = line_def["end"]
    # ワールド座標で直接描画
    ax.plot([x0, x1], [y0, y1], "b-", lw=2)

# スタート・ゴール
#ax.plot(start_pos[0], start_pos[1], "go", markersize=8, label="Start")
#ax.plot(goal_pos[0], goal_pos[1], "ro", markersize=8, label="Goal")

# パイロン描画（ワールド座標で直接）
for p in pylons:
    x, y = p["pos"]
    # ワールド座標で直接描画（半径も実際のメートル単位）
    circ = Circle((x, y), radius=0.1, color="darkorange", alpha=0.9)  # 10cm半径
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

# --- モード選択 RadioButtons ---
ax_radio = plt.axes([0.02, 0.7, 0.12, 0.25])
mode_labels = [config['name'] for config in DRIVING_MODES.values()]
mode_radio = RadioButtons(ax_radio, mode_labels)
mode_radio.set_active(0)  # デフォルトは予選用

# --- waypoint スライダー ---
ax_slider = plt.axes([0.2, 0.4, 0.65, 0.03])
slider = Slider(ax_slider, "Waypoint Index", 0, MAX_WAYPOINTS-1, valinit=0, valstep=1)

# --- 速度スライダー (-100～100) ---
ax_speed = plt.axes([0.2, 0.33, 0.65, 0.03])
speed_slider = Slider(ax_speed, "Speed", -100.0, 100.0, valinit=DEFAULT_SPEED)

# --- プロット更新関数 ---
def update_plot(val=None):
    global yaw_arrows
    # 既存の矢印を削除
    for arr in yaw_arrows:
        arr.remove()
    yaw_arrows = []

    if waypoints:
        # グリッド座標をワールド座標に変換
        xs = [grid_to_world(wp["x"], wp["y"])[0] for wp in waypoints]  # ワールドX座標
        ys = [grid_to_world(wp["x"], wp["y"])[1] for wp in waypoints]  # ワールドY座標
        wp_points.set_data(xs, ys)

        # --- 曲線補間（purepursuit風） ---
        if len(xs) >= 3:
            tck, u = splprep([xs, ys], s=0)
            unew = np.linspace(0, 1, max(100, len(xs)*10))
            out = splev(unew, tck)
            wp_line.set_data(out[0], out[1])
        else:
            wp_line.set_data(xs, ys)

        idx = int(slider.val)
        if 0 <= idx < len(waypoints):
            # 選択されたwaypointもワールド座標に変換
            world_x, world_y = grid_to_world(waypoints[idx]["x"], waypoints[idx]["y"])
            selected_wp.set_data([world_x], [world_y])
            speed_slider.set_val(waypoints[idx]["v"])
        else:
            selected_wp.set_data([], [])
    else:
        wp_line.set_data([], [])
        wp_points.set_data([], [])
        selected_wp.set_data([], [])
        speed_slider.set_val(DEFAULT_SPEED)

    # --- yaw矢印描画（ワールド座標で） ---
    arrow_length = 0.4  # 40cm in world coordinates
    for wp in waypoints:
        if "yaw" in wp:
            # グリッド座標をワールド座標に変換
            world_x, world_y = grid_to_world(wp["x"], wp["y"])
            yaw_rad = math.radians(wp["yaw"])
            dx = arrow_length * math.cos(yaw_rad)
            dy = arrow_length * math.sin(yaw_rad)
            arr = ax.arrow(world_x, world_y, dx, dy, head_width=0.15, head_length=0.2, fc='r', ec='r', alpha=0.7)
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
    if len(waypoints) >= MAX_WAYPOINTS:
        print(f"Maximum {MAX_WAYPOINTS} waypoints reached")
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
    if waypoints:
        x_prev, y_prev = waypoints[-1]["x"], waypoints[-1]["y"]
        if not is_valid_segment(x_prev, y_prev, x_click, y_click):
            print("Waypoint rejected: crosses obstacle, wall, or pylon")
            return

    # 最小回転半径チェック
    if len(waypoints) >= 2:
        x1, y1 = waypoints[-2]["x"], waypoints[-2]["y"]
        x2, y2 = waypoints[-1]["x"], waypoints[-1]["y"]
        x3, y3 = x_click, y_click
        wx1, wy1 = grid_to_world(x1, y1)
        wx2, wy2 = grid_to_world(x2, y2)
        wx3, wy3 = grid_to_world(x3, y3)
        _, _, radius = calc_circle(wx1, wy1, wx2, wy2, wx3, wy3)
        if radius < MIN_TURN_RADIUS:
            print(f"Waypoint rejected: turn radius {radius:.2f}m < {MIN_TURN_RADIUS}m")
            return

    waypoints.append({"x": x_click, "y": y_click, "v": DEFAULT_SPEED})
    update_plot()

cid = fig.canvas.mpl_connect('button_press_event', onclick)

# --- Clear ボタン (モード対応) ---
ax_clear = plt.axes([0.05, 0.1, 0.1, 0.04])
button_clear = Button(ax_clear, "Clear Mode")
def clear(event):
    current_waypoints = waypoints_dict[current_mode]
    current_waypoints.clear()
    slider.set_val(0)
    print(f"Cleared all waypoints for {DRIVING_MODES[current_mode]['name']}")
    update_plot()
button_clear.on_clicked(clear)

# --- Save ボタン (モード対応) ---
ax_save = plt.axes([0.17, 0.1, 0.1, 0.04])
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
ax_delete = plt.axes([0.29, 0.1, 0.1, 0.04])
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
ax_save_all = plt.axes([0.41, 0.1, 0.12, 0.04])
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

# コース寸法情報とモード情報をコンソール出力
print(f"\n=== MULTI-MODE WAYPOINT EDITOR ===")
print(f"Course Dimensions:")
print(f"   X range: {x_min:.1f}m - {x_max:.1f}m = {x_max-x_min:.1f}m")
print(f"   Y range: {y_min:.1f}m - {y_max:.1f}m = {y_max-y_min:.1f}m")
print(f"   Grid resolution: {0.05}m/pixel")
print(f"   Grid size: {grid_matrix.shape[1]} x {grid_matrix.shape[0]} pixels")
print(f"\nDriving Modes:")
for mode, config in DRIVING_MODES.items():
    print(f"   {config['name']} ({config['color']} {config['marker']})")
print(f"\nCurrent mode: {DRIVING_MODES[current_mode]['name']}\n")

plt.legend()
plt.show()
