import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line, polygon, disk

# --- グリッド設定 ---
resolution = 0.05  # 5cm
x_min, x_max = -3.2, 7.0
y_min, y_max = -1.5, 5.0

width  = int((x_max - x_min) / resolution)
height = int((y_max - y_min) / resolution)
grid = np.zeros((height, width), dtype=np.uint8)

def world_to_grid(x, y):
    """ワールド座標 (m) → グリッド座標 (col,row)"""
    ix = int((x - x_min) / resolution)
    iy = int((y - y_min) / resolution)
    return ix, iy

def grid_to_world(ix, iy):
    """グリッド座標 (col,row) → ワールド座標 (m)"""
    x = ix * resolution + x_min
    y = iy * resolution + y_min
    return x, y

# --- 壁 ---
walls = [
    {"id":"wall_01", "start":(-3.084, -0.625), "end":(-3.084, 3.931)},
    {"id":"wall_02", "start":(-3.084, 3.931), "end":(-2.385, 4.608)},
    {"id":"wall_03", "start":(-2.385, 4.608), "end":(-0.590, 4.805)},
    {"id":"wall_04", "start":(-0.590, 4.805), "end":(5.848, 4.805)},
    {"id":"wall_05", "start":(5.848, 4.805), "end":(5.848, 4.805)},
    {"id":"wall_06", "start":(5.848, 4.805), "end":(6.544, 3.825)},
    {"id":"wall_07", "start":(6.544, 3.825), "end":(6.716, 2.027)},
    {"id":"wall_08", "start":(6.716, 2.027), "end":(6.716, 0.186)},
    {"id":"wall_09", "start":(6.716, 0.186), "end":(6.165, -0.625)},
    {"id":"wall_10", "start":(6.165, -0.625), "end":(2.441, -0.625)},
    {"id":"wall_11", "start":(2.441, -0.625), "end":(2.441, -1.128)},
    {"id":"wall_12", "start":(2.441, -1.128), "end":(-1.229, -1.128)},
    {"id":"wall_13", "start":(-1.229, -1.128), "end":(-1.229, -0.625)},
    {"id":"wall_14", "start":(-1.229, -0.625), "end":(-3.084, -0.625)},
    {"id":"wall_15", "start":(1.233, -1.128), "end":(1.233, -0.625)},
    {"id":"wall_16", "start":(0.000, -1.128), "end":(0.000, -0.625)},
    {"id":"wall_17", "start":(-3.084, 0.148), "end":(-2.339, -0.625)},
    {"id":"wall_18", "start":(2.150, 4.363), "end":(3.052, 4.363)},
    {"id":"wall_19", "start":(2.150, 3.916), "end":(3.052, 3.916)},
    {"id":"wall_20", "start":(4.925, 3.407), "end":(-0.643, 3.407)},
    {"id":"wall_21", "start":(-0.643, 3.407), "end":(-1.447, 2.947)},
    {"id":"wall_22", "start":(-1.447, 2.947), "end":(-1.447, 1.047)},
    {"id":"wall_23", "start":(-1.447, 1.047), "end":(-0.632, 0.625)},
    {"id":"wall_24", "start":(-0.632, 0.625), "end":(4.925, 0.625)},
    {"id":"wall_25", "start":(1.296, 2.079), "end":(2.230, 2.079)},
    {"id":"wall_26", "start":(2.230, 2.079), "end":(3.095, 2.311)},
    {"id":"wall_27", "start":(4.011, 2.273), "end":(5.876, 2.273)},
    {"id":"wall_28", "start":(5.876, 2.273), "end":(6.621, 3.123)},
    {"id":"wall_29", "start":(5.876, 2.273), "end":(6.716, 1.458)},
]
for w in walls:
    x1, y1 = world_to_grid(*w["start"])
    x2, y2 = world_to_grid(*w["end"])
    rr, cc = line(y1, x1, y2, x2)
    grid[rr, cc] = 1

# --- 障害物 ---
obstacles = [
    {"start": (-2.434, 3.481), "end": (-1.528, 2.965)},
    {"start": (-2.434, 1.180), "end": (-1.528, 0.639)},
    {"start": (-3.095, 2.445), "end": (-2.171, 1.721)},
]
for obs in obstacles:
    x1, y1 = world_to_grid(*obs["start"])
    x2, y2 = world_to_grid(*obs["end"])
    rr, cc = polygon([y1, y1, y2, y2], [x1, x2, x2, x1])
#    grid[rr, cc] = 1

# --- パイロン ---
pylons = [
    {"pos": (-0.063, 2.079)},
    {"pos": (-0.239, 2.079)},
]
for p in pylons:
    x, y = world_to_grid(*p["pos"])
    rr, cc = disk((y, x), radius=1)
    grid[rr, cc] = 1

# --- スタートライン ---
start_lines = [
    {"start": (0.000, -0.625), "end": (0.000, 0.625)},
    {"start": (1.855, -0.625), "end": (1.855, 0.625)},
    {"start": (3.720, -0.625), "end": (3.720, 0.625)},
]
for line_def in start_lines:
    x1, y1 = world_to_grid(*line_def["start"])
    x2, y2 = world_to_grid(*line_def["end"])
    rr, cc = line(y1, x1, y2, x2)
#    grid[rr, cc] = 1

# --- スタート・ゴール設定 (m単位で指定) ---
start_world = (0.0, 0.0)     # 原点
goal_world  = (-2, 0.5)     # 適当な右上
start_pos = world_to_grid(*start_world)
goal_pos  = world_to_grid(*goal_world)

# 外部利用用に export
grid_matrix = grid

# --- 動作確認用 ---
if __name__ == "__main__":
    plt.imshow(grid_matrix, cmap="Greys", origin="lower", interpolation='nearest', alpha=0.1)
    plt.gca().set_facecolor("white")  # 背景白
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")   # スタート
    plt.plot(goal_pos[0], goal_pos[1], "rx", label="Goal")     # ゴール
    plt.plot([start_pos[0], goal_pos[0]], [start_pos[1], goal_pos[1]], "b--", lw=1, label="Line")  # 青ライン

    # 壁（黒でプロット）
    for i, w in enumerate(walls):
        x1, y1 = world_to_grid(*w["start"])
        x2, y2 = world_to_grid(*w["end"])
        plt.plot([x1, x2], [y1, y2], "k-", lw=2, label="Wall" if i == 0 else "")

    # 障害物（緑で塗りつぶし長方形）
    for i, obs in enumerate(obstacles):
        x1, y1 = world_to_grid(*obs["start"])
        x2, y2 = world_to_grid(*obs["end"])
        rect_x = [x1, x2, x2, x1]
        rect_y = [y1, y1, y2, y2]
        plt.fill(rect_x, rect_y, color="green", alpha=0.5, label="Obstacle" if i == 0 else "")

    # パイロン（青でプロット）
    for i, p in enumerate(pylons):
        x, y = world_to_grid(*p["pos"])
        plt.plot(x, y, "bo", markersize=8, label="Pylon" if i == 0 else "")

    # スタートライン（青でプロット）
    for i, line_def in enumerate(start_lines):
        x1, y1 = world_to_grid(*line_def["start"])
        x2, y2 = world_to_grid(*line_def["end"])
        plt.plot([x1, x2], [y1, y2], "b-", lw=2, label="Start Line" if i == 0 else "")

    plt.legend()
    plt.show()