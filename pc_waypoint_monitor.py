# pc_waypoint_monitor.py
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import threading
import time
import json
from matplotlib.animation import FuncAnimation
from scipy.interpolate import CubicSpline

from cource_map import grid_matrix, world_to_grid, pylons, start_pos, goal_pos

# --- Waypoints読み込み ---
WAYPOINT_FILE = "quarify.json"
with open(WAYPOINT_FILE, "r") as f:
    waypoints = json.load(f)

# --- 初期化 ---
current_pos = [0, 0]
status_text = "Waiting"

# --- Figure設定 ---
fig, ax = plt.subplots(dpi=120)
plt.subplots_adjust(bottom=0.2)
ax.imshow(grid_matrix, cmap="Greys", origin="lower")
ax.set_xlim(0, grid_matrix.shape[1])
ax.set_ylim(0, grid_matrix.shape[0])

# --- パイロン描画 ---
for p in pylons:
    x, y = world_to_grid(*p["pos"])
    circ = Circle((x, y), radius=2, color="darkorange", alpha=0.9)
    ax.add_patch(circ)

# --- Waypoint座標 ---
wp_x = [wp["x"] for wp in waypoints]
wp_y = [wp["y"] for wp in waypoints]

# --- Pure Pursuit 補間 ---
cs_x = CubicSpline(np.arange(len(wp_x)), wp_x)
cs_y = CubicSpline(np.arange(len(wp_y)), wp_y)
num_points = len(wp_x) * 20  # 補間点
interp_idx = np.linspace(0, len(wp_x)-1, num=num_points)
pp_x = cs_x(interp_idx)
pp_y = cs_y(interp_idx)

# 補間パス描画
wp_line, = ax.plot(pp_x, pp_y, "m--", lw=2, label="Pure Pursuit Path")
wp_points, = ax.plot(wp_x, wp_y, "mo", markersize=6)

# 現在位置描画
pos_dot, = ax.plot([], [], "ro", markersize=8, label="Current Position")
status_text_box = ax.text(0.02, -0.1, "", transform=ax.transAxes)

# --- 更新関数 ---
frame_idx = [0]  # リストでグローバルに扱えるように
def update(frame):
    if frame_idx[0] < len(pp_x):
        current_pos[0] = pp_x[frame_idx[0]]
        current_pos[1] = pp_y[frame_idx[0]]
        frame_idx[0] += 1
        status = f"Moving to WP {frame_idx[0]}/{len(pp_x)}"
    else:
        status = "Finished"
    pos_dot.set_data([current_pos[0]], [current_pos[1]])
    status_text_box.set_text(f"Status: {status}")
    return pos_dot, status_text_box

# --- 模擬ラズパイ送信スレッド ---
def mock_raspberrypi_thread():
    global current_pos, status_text
    idx = 0
    while idx < len(waypoints):
        current_pos[0] = wp_x[idx]
        current_pos[1] = wp_y[idx]
        status_text = f"Waypoint {idx+1}/{len(waypoints)}"
        time.sleep(0.05)  # WP速度に応じて調整
        idx += 1
    status_text = "Finished"

threading.Thread(target=mock_raspberrypi_thread, daemon=True).start()

# --- アニメーション開始 ---
ani = FuncAnimation(fig, update, interval=50)  # インターバル 50ms (20fps)
plt.legend()
plt.show()
