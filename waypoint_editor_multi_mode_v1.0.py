# waypoint_editor_multi_mode.py
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.widgets import Slider, Button, RadioButtons
import json
import math
import os
from course_map import grid_matrix, world_to_grid, grid_to_world, start_pos, goal_pos, obstacles, start_lines, pylons
from course_map import x_min, x_max, y_min, y_max, resolution
from scipy.interpolate import splprep, splev
import numpy as np
from skimage.draw import line

# 文字化け完全修正 - 英語表示に変更
plt.rcParams['font.family'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# --- 走行モード定義（新ルール: 予選1つ + 本選3レーン） ---
DRIVING_MODES = {
    'qualifying': {'name': 'Qualifying', 'color': 'm', 'marker': 'o', 'file_suffix': '_qualifying'},
    'final1': {'name': 'Final Lane 1', 'color': 'r', 'marker': '^', 'file_suffix': '_final1'},
    'final2': {'name': 'Final Lane 2', 'color': 'orange', 'marker': 's', 'file_suffix': '_final2'},
    'final3': {'name': 'Final Lane 3', 'color': 'c', 'marker': 'D', 'file_suffix': '_final3'}
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
# スクリプト所在ディレクトリを基準にファイル入出力を行う
base_dir = os.path.dirname(__file__)

# --- Figure 作成（大きなプロットサイズ） ---
fig, ax = plt.subplots(figsize=(14, 10), dpi=100)  # より大きなプロットサイズ
plt.subplots_adjust(bottom=0.35, left=0.12, right=0.98, top=0.95)  # スライダー用余白確保

# 背景グリッド（メートル単位で表示）
ax.imshow(grid_matrix, cmap="Greys", origin="lower", 
         extent=[x_min, x_max, y_min, y_max], alpha=0.7)

# 軸設定：メートル座標範囲
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(x_min - 0.2, x_max + 0.2)  # メートル座標範囲
ax.set_ylim(y_min - 0.2, y_max + 0.2)  # メートル座標範囲

# 軸ラベル（メートル表示）
ax.set_xlabel(f'X [meters] - Course Width: {x_max-x_min:.1f}m', fontsize=14, fontweight='bold')
ax.set_ylabel(f'Y [meters] - Course Height: {y_max-y_min:.1f}m', fontsize=14, fontweight='bold')
ax.set_title(f'Multi-Mode Waypoint Editor - Course: {x_max-x_min:.1f}m x {y_max-y_min:.1f}m (Resolution: {resolution}m/px)', fontsize=16, fontweight='bold')

# 障害物描画（メートル座標）
for obs in obstacles:
    x0, y0 = obs["start"]
    x1, y1 = obs["end"]
    left = min(x0, x1)
    bottom = min(y0, y1)
    width = abs(x1 - x0)
    height = abs(y1 - y0)
    rect = Rectangle((left, bottom), width, height, color="lightgreen", alpha=0.6, edgecolor='green', linewidth=2)
    ax.add_patch(rect)

# スタートライン描画（メートル座標）
for line_def in start_lines:
    x0, y0 = line_def["start"]
    x1, y1 = line_def["end"]
    ax.plot([x0, x1], [y0, y1], "b-", linewidth=4, alpha=0.8, label='Start Lines')

# パイロン描画（メートル座標）
for p in pylons:
    x, y = p["pos"]
    circ = Circle((x, y), radius=0.1, color="darkorange", alpha=0.9, edgecolor='red', linewidth=2)
    ax.add_patch(circ)

# --- waypoint 描画 (モード別) ---
wp_lines = {}
wp_points_plots = {}
wp_points_narrow = {}  # narrowポイント用（青色）
wp_points_checkpoint = {}  # チェックポイント用（緑色）
selected_wps = {}
yaw_arrows = []  # 矢印オブジェクトを保持

# ドラッグ状態管理
drag_data = {
    'dragging': False,
    'waypoint_idx': None
}

# 各モードのプロット要素を初期化
for mode, config in DRIVING_MODES.items():
    wp_lines[mode], = ax.plot([], [], "-", lw=3, color=config['color'], 
                             label=f"{config['name']} Path", alpha=0.9)
    wp_points_plots[mode], = ax.plot([], [], config['marker'], color=config['color'], 
                                    markersize=6, alpha=0.8)
    wp_points_narrow[mode], = ax.plot([], [], config['marker'], color='blue',
                                     markersize=6, alpha=0.8)  # narrowは青色
    wp_points_checkpoint[mode], = ax.plot([], [], config['marker'], color='green',
                                         markersize=8, alpha=0.9)  # チェックポイントは緑色
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
speed_slider = Slider(ax_speed, "Speed", -100.0, 100.0, valinit=DEFAULT_SPEED, valstep=10)

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
            # 現在のモードのみ表示（メートル座標）
            # チェックポイント、narrow、通常の優先順で色分け
            xs_checkpoint = [wp["x"] for wp in waypoints if wp.get("checkpoint") in ["start_line", "middle_mat"]]
            ys_checkpoint = [wp["y"] for wp in waypoints if wp.get("checkpoint") in ["start_line", "middle_mat"]]
            xs_narrow = [wp["x"] for wp in waypoints if wp.get("narrow", False) and not wp.get("checkpoint")]
            ys_narrow = [wp["y"] for wp in waypoints if wp.get("narrow", False) and not wp.get("checkpoint")]
            xs_normal = [wp["x"] for wp in waypoints if not wp.get("narrow", False) and not wp.get("checkpoint")]
            ys_normal = [wp["y"] for wp in waypoints if not wp.get("narrow", False) and not wp.get("checkpoint")]
            
            # 通常ポイント（元の色）
            wp_points_plots[mode].set_data(xs_normal, ys_normal)
            
            # narrowポイント（青色）
            wp_points_narrow[mode].set_data(xs_narrow, ys_narrow)
            
            # チェックポイント（緑色）
            wp_points_checkpoint[mode].set_data(xs_checkpoint, ys_checkpoint)
            
            # 全ポイントで曲線補間用
            xs = [wp["x"] for wp in waypoints]  # メートルX座標
            ys = [wp["y"] for wp in waypoints]  # メートルY座標

            # --- 曲線補間（purepursuit風） ---
            if len(xs) >= 3:
                try:
                    # 重複座標の確認
                    unique_points = list(set(zip(xs, ys)))
                    if len(unique_points) < len(xs):
                        print(f"[WARNING] {mode} has duplicate points: {len(xs)} total, {len(unique_points)} unique")
                    
                    # s=0で全点を通る補間、k=3で3次スプライン（滑らか）
                    tck, u = splprep([xs, ys], s=0, k=min(3, len(xs)-1))
                    unew = np.linspace(0, 1, max(100, len(xs)*10))
                    out = splev(unew, tck)
                    wp_lines[mode].set_data(out[0], out[1])
                except Exception as e:
                    # エラー時は直線で接続
                    print(f"[WARNING] Spline interpolation failed for {mode}: {e}")
                    print(f"  Points count: {len(xs)}, First 3 points: {list(zip(xs[:3], ys[:3]))}")
                    wp_lines[mode].set_data(xs, ys)
            elif len(xs) == 2:
                # 2点の場合は直線
                wp_lines[mode].set_data(xs, ys)
            else:
                wp_lines[mode].set_data(xs, ys)
        else:
            # 現在のモード以外は非表示
            wp_lines[mode].set_data([], [])
            wp_points_plots[mode].set_data([], [])
            wp_points_narrow[mode].set_data([], [])
            wp_points_checkpoint[mode].set_data([], [])
    
    # 現在のモードの選択されたwaypointを表示
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    
    # 全モードの選択状態をリセット
    for mode in DRIVING_MODES.keys():
        selected_wps[mode].set_data([], [])
    
    if 0 <= idx < len(current_waypoints):
        # 現在のモードのみ選択表示（メートル座標）
        meter_x = current_waypoints[idx]["x"]
        meter_y = current_waypoints[idx]["y"]
        selected_wps[current_mode].set_data([meter_x], [meter_y])
        speed_slider.set_val(current_waypoints[idx]["v"])
    else:
        speed_slider.set_val(DEFAULT_SPEED)

    # --- yaw矢印描画 (選択したモードのみ、メートル座標) ---
    arrow_length = 0.5  # メートル単位で50cm
    mode_waypoints = waypoints_dict[current_mode]
    mode_config = DRIVING_MODES[current_mode]
    
    for wp in mode_waypoints:
        if "yaw" in wp:
            # メートル座標で直接描画
            meter_x = wp["x"]
            meter_y = wp["y"]
            yaw_rad = math.radians(wp["yaw"])
            dx = arrow_length * math.cos(yaw_rad)
            dy = arrow_length * math.sin(yaw_rad)
            arr = ax.arrow(meter_x, meter_y, dx, dy, head_width=0.1, head_length=0.15, 
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
    
    # モード変更時のみ出力
    current_wp_count = len(waypoints_dict[current_mode])
    print(f"[MODE] {DRIVING_MODES[current_mode]['name']} ({current_wp_count} waypoints)")
    update_plot()
    
mode_radio.on_clicked(change_mode)
slider.on_changed(update_plot)

# --- 速度スライダー更新 (モード対応) ---
def update_speed(val):
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        current_waypoints[idx]["v"] = speed_slider.val
        # スライダー操作中は出力を抑制（静音モード）

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
    
    # クリック座標はメートル単位
    x_click = round(event.xdata, 3)  # メートル単位（mm精度）
    y_click = round(event.ydata, 3)
    
    # 範囲チェック
    if not (x_min <= x_click <= x_max and y_min <= y_click <= y_max):
        print("Waypoint rejected: out of course bounds")
        return
    
    # 障害物チェック用にグリッド座標に変換
    x_grid, y_grid = world_to_grid(x_click, y_click)

    # 障害物・壁・パイロン上は不可（グリッド座標でチェック）
    if is_obstacle_or_wall(x_grid, y_grid):
        print("Waypoint rejected: on obstacle or wall")
        return
    if is_pylon(x_grid, y_grid):
        print("Waypoint rejected: on pylon")
        return
    
    # 直前のwaypointから壁・障害物・パイロンをまたぐ場合も不可
    if current_waypoints:
        x_prev, y_prev = current_waypoints[-1]["x"], current_waypoints[-1]["y"]
        x_prev_grid, y_prev_grid = world_to_grid(x_prev, y_prev)
        if not is_valid_segment(x_prev_grid, y_prev_grid, x_grid, y_grid):
            print("Waypoint rejected: crosses obstacle, wall, or pylon")
            return

    # 最小回転半径チェック（メートル座標で計算）
    if len(current_waypoints) >= 2:
        x1, y1 = current_waypoints[-2]["x"], current_waypoints[-2]["y"]
        x2, y2 = current_waypoints[-1]["x"], current_waypoints[-1]["y"]
        x3, y3 = x_click, y_click
        _, _, radius = calc_circle(x1, y1, x2, y2, x3, y3)
        if radius < MIN_TURN_RADIUS:
            print(f"Waypoint rejected: turn radius {radius:.2f}m < {MIN_TURN_RADIUS}m")
            return

    current_waypoints.append({"x": x_click, "y": y_click, "v": DEFAULT_SPEED})
    print(f"[ADD] WP#{len(current_waypoints)-1} ({x_click:.2f}, {y_click:.2f})m")
    update_plot()

cid = fig.canvas.mpl_connect('button_press_event', onclick)

# --- ドラッグ機能の追加 ---
def on_press(event):
    """マウス押下時: 選択中のwaypointをドラッグ開始"""
    if event.inaxes != ax:
        return
    
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    
    if 0 <= idx < len(current_waypoints):
        # 選択中のwaypointの位置
        wp_x = current_waypoints[idx]["x"]
        wp_y = current_waypoints[idx]["y"]
        
        # クリック位置との距離チェック（0.3m以内ならドラッグ開始）
        if event.xdata and event.ydata:
            distance = ((event.xdata - wp_x)**2 + (event.ydata - wp_y)**2)**0.5
            if distance < 0.3:  # 30cm以内
                drag_data['dragging'] = True
                drag_data['waypoint_idx'] = idx
                # ドラッグ開始時は出力抑制

def on_motion(event):
    """マウス移動時: waypointをドラッグ"""
    if not drag_data['dragging'] or event.inaxes != ax:
        return
    
    if event.xdata is None or event.ydata is None:
        return
    
    current_waypoints = waypoints_dict[current_mode]
    idx = drag_data['waypoint_idx']
    
    if 0 <= idx < len(current_waypoints):
        # 新しい位置
        new_x = round(event.xdata, 3)
        new_y = round(event.ydata, 3)
        
        # 範囲チェック
        if x_min <= new_x <= x_max and y_min <= new_y <= y_max:
            # 障害物チェック（簡易版）
            x_grid, y_grid = world_to_grid(new_x, new_y)
            if not is_obstacle_or_wall(x_grid, y_grid) and not is_pylon(x_grid, y_grid):
                # 位置更新
                current_waypoints[idx]["x"] = new_x
                current_waypoints[idx]["y"] = new_y
                update_plot()

def on_release(event):
    """マウス離し時: ドラッグ終了"""
    if drag_data['dragging']:
        current_waypoints = waypoints_dict[current_mode]
        idx = drag_data['waypoint_idx']
        if 0 <= idx < len(current_waypoints):
            wp = current_waypoints[idx]
            print(f"[MOVE] WP#{idx} → ({wp['x']:.2f}, {wp['y']:.2f})m")
        
        drag_data['dragging'] = False
        drag_data['waypoint_idx'] = None

# マウスイベント接続
cid_press = fig.canvas.mpl_connect('button_press_event', on_press)
cid_motion = fig.canvas.mpl_connect('motion_notify_event', on_motion)
cid_release = fig.canvas.mpl_connect('button_release_event', on_release)

# --- Save ボタン (モード対応) ---
ax_save = plt.axes([0.05, 0.05, 0.1, 0.04])
button_save = Button(ax_save, "Save Mode")

def save_waypoints(event):
    current_waypoints = waypoints_dict[current_mode]
    if not current_waypoints:
        print(f"[SAVE] No waypoints to save")
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

    # モード別ファイル名で保存（スクリプトディレクトリ基準）
    save_path = os.path.join(base_dir, f"{base_save_path}{DRIVING_MODES[current_mode]['file_suffix']}.json")
    with open(save_path, "w", encoding='utf-8') as f:
        json.dump(current_waypoints, f, indent=2, ensure_ascii=False)
    
    update_plot()  # yaw矢印を更新
    filename = os.path.basename(save_path)
    print(f"[SAVE] {len(current_waypoints)} waypoints → {filename}")

button_save.on_clicked(save_waypoints)

# --- Delete ボタン ---
ax_delete = plt.axes([0.17, 0.05, 0.1, 0.04])
button_delete = Button(ax_delete, "Delete WP")

def delete_waypoint(event):
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        del current_waypoints[idx]
        slider.set_val(max(0, min(slider.val, len(current_waypoints)-1)))
        update_plot()
        print(f"[DELETE] WP#{idx}")

button_delete.on_clicked(delete_waypoint)

# --- Update Yaw ボタン (全Waypointのyaw再計算) ---
ax_update_yaw = plt.axes([0.29, 0.05, 0.12, 0.04])
button_update_yaw = Button(ax_update_yaw, "Update Yaw")

def update_yaw(event):
    """現在のモードの全waypointsのyawを再計算"""
    current_waypoints = waypoints_dict[current_mode]
    if len(current_waypoints) < 2:
        print(f"[YAW] Need at least 2 waypoints")
        return
    
    # yaw 計算
    for i in range(len(current_waypoints)):
        if i < len(current_waypoints) - 1:
            dx = current_waypoints[i+1]["x"] - current_waypoints[i]["x"]
            dy = current_waypoints[i+1]["y"] - current_waypoints[i]["y"]
            yaw_deg = math.degrees(math.atan2(dy, dx))
        else:
            yaw_deg = current_waypoints[i-1]["yaw"] if i > 0 else 0.0
        current_waypoints[i]["yaw"] = yaw_deg
    
    update_plot()
    print(f"[YAW] Updated {len(current_waypoints)} waypoints")

button_update_yaw.on_clicked(update_yaw)

# --- Toggle Narrow ボタン (選択中のwaypointのnarrowフラグを切り替え) ---
ax_narrow = plt.axes([0.43, 0.05, 0.12, 0.04])
button_narrow = Button(ax_narrow, "Toggle Narrow")

def toggle_narrow(event):
    """選択中のwaypointのnarrowフラグを切り替え"""
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        # narrowフラグがなければFalse扱い
        current_narrow = current_waypoints[idx].get("narrow", False)
        current_waypoints[idx]["narrow"] = not current_narrow
        new_state = "ON" if current_waypoints[idx]["narrow"] else "OFF"
        print(f"[NARROW] WP#{idx} → {new_state}")
        update_plot()

button_narrow.on_clicked(toggle_narrow)

# --- Set Start Line ボタン (選択中のwaypointをスタートラインに設定) ---
ax_start = plt.axes([0.57, 0.05, 0.12, 0.04])
button_start = Button(ax_start, "Start Line")

def set_start_line(event):
    """選択中のwaypointをstart_lineチェックポイントに設定"""
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        current_waypoints[idx]["checkpoint"] = "start_line"
        print(f"[CHECKPOINT] WP#{idx} → Start Line")
        update_plot()

button_start.on_clicked(set_start_line)

# --- Set Middle Mat ボタン (選択中のwaypointを中間地点に設定) ---
ax_middle = plt.axes([0.71, 0.05, 0.12, 0.04])
button_middle = Button(ax_middle, "Middle Mat")

def set_middle_mat(event):
    """選択中のwaypointをmiddle_matチェックポイントに設定"""
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        current_waypoints[idx]["checkpoint"] = "middle_mat"
        print(f"[CHECKPOINT] WP#{idx} → Middle Mat")
        update_plot()

button_middle.on_clicked(set_middle_mat)

# --- Clear Checkpoint ボタン (選択中のwaypointのチェックポイントを解除) ---
ax_clear_cp = plt.axes([0.85, 0.05, 0.12, 0.04])
button_clear_cp = Button(ax_clear_cp, "Clear CP")

def clear_checkpoint(event):
    """選択中のwaypointのチェックポイント設定を解除"""
    current_waypoints = waypoints_dict[current_mode]
    idx = int(slider.val)
    if 0 <= idx < len(current_waypoints):
        if "checkpoint" in current_waypoints[idx]:
            del current_waypoints[idx]["checkpoint"]
            print(f"[CHECKPOINT] WP#{idx} → Cleared")
            update_plot()

button_clear_cp.on_clicked(clear_checkpoint)

# --- Waypoints 読み込み ---
def load_waypoints():
    global waypoints_dict
    loaded_count = 0
    
    print("\n[LOAD] Loading waypoint files...")
    for mode, config in DRIVING_MODES.items():
        # スクリプトディレクトリ基準でファイルを探す
        file_path = os.path.join(base_dir, f"{base_save_path}{config['file_suffix']}.json")
        try:
            with open(file_path, "r", encoding='utf-8') as f:
                waypoints_dict[mode] = json.load(f)
            loaded_count += len(waypoints_dict[mode])
            filename = os.path.basename(file_path)
            print(f"  • {config['name']}: {len(waypoints_dict[mode])} pts ← {filename}")
        except FileNotFoundError:
            waypoints_dict[mode] = []
            # ファイルが見つからない場合は出力を抑制
    
    update_plot()
    print(f"[LOAD] Total: {loaded_count} waypoints loaded\n")

# 起動時にロード
load_waypoints()

# === STARTUP INFORMATION ===
print(f"\n{'='*60}")
print(f"  WAYPOINT EDITOR - Course: {x_max-x_min:.1f}m x {y_max-y_min:.1f}m")
print(f"{'='*60}")

print(f"\nMODES: ", end="")
mode_names = [config['name'] for config in DRIVING_MODES.values()]
print(" | ".join(mode_names))

print(f"\nCONTROLS:")
print(f"  Click    : Add waypoint")
print(f"  Drag     : Move selected waypoint (within 30cm)")
print(f"  Slider   : Select waypoint / Set speed")
print(f"  Buttons  : Save | Delete | Update Yaw | Toggle Narrow | Start Line | Middle Mat | Clear CP")

print(f"\nCOORDINATES: Meters (X: {x_min:.1f}~{x_max:.1f}m, Y: {y_min:.1f}~{y_max:.1f}m)")
print(f"VALIDATION: Obstacle/Wall/Pylon detection, Min turn radius {MIN_TURN_RADIUS}m")

print(f"\n{'='*60}")
print(f"Ready! Current mode: {DRIVING_MODES[current_mode]['name']}")
print(f"{'='*60}\n")

plt.legend()
plt.show()