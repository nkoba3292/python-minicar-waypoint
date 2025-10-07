import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from heapq import heappush, heappop
from cource_map import grid_matrix, start_pos, goal_pos
from collections import deque

def is_reachable_bfs(grid, start, goal):
    """0=通過可能, 1=障害物 の grid に対して BFS で到達可能か判定"""
    h, w = grid.shape
    visited = np.zeros_like(grid, dtype=bool)
    q = deque([start])
    visited[start[1], start[0]] = True
    moves = [(1,0), (-1,0), (0,1), (0,-1)]
    while q:
        x, y = q.popleft()
        if (x, y) == goal:
            return True
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if 0 <= nx < w and 0 <= ny < h:
                if not visited[ny, nx] and grid[ny, nx] == 0:
                    visited[ny, nx] = True
                    q.append((nx, ny))
    return False

print("start_pos:", start_pos, "goal_pos:", goal_pos)
print("goal reachable by BFS?:", is_reachable_bfs(grid_matrix, start_pos, goal_pos))


# === A* 実装 ===
def heuristic(a, b):
    """マンハッタン距離"""
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    print(f"[DEBUG] Start={start}, Goal={goal}")
    h, w = grid.shape
    open_list = []
    heappush(open_list, (0+heuristic(start, goal), 0, start, [start]))
    visited = set()
    iter_count = 0

    while open_list:
        f, g, current, path = heappop(open_list)
        iter_count += 1

        if iter_count % 50 == 0:
            print(f"[DEBUG] Iter={iter_count}, current={current}, g={g}, f={f}")

        if current == goal:
            print(f"[DEBUG] Goal reached in {iter_count} iterations")
            return path

        if current in visited:
            continue
        visited.add(current)

        (r, c) = current
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < h and 0 <= nc < w and grid[nr, nc] == 0:
                new_pos = (nr, nc)
                if new_pos not in visited:
                    new_path = path + [new_pos]
                    heappush(open_list, (g+1+heuristic(new_pos, goal), g+1, new_pos, new_path))

    print("[DEBUG] A* failed: no path found")
    return None

# === 経路探索 ===
grid = np.array(grid_matrix, dtype=int)

print(f"[DEBUG] Grid shape: {grid.shape}")
print(f"[DEBUG] Start cell value={grid[start_pos[0], start_pos[1]]}")
print(f"[DEBUG] Goal  cell value={grid[goal_pos[0], goal_pos[1]]}")

path = astar(grid, start_pos, goal_pos)

if path is None:
    raise RuntimeError("経路が見つかりません。スタート・ゴールや障害物を確認してください。")

print("A* path length:", len(path))
print("A* path sample:", path[:10], "..." if len(path) > 10 else "")

# === アニメーション表示 ===
fig, ax = plt.subplots(figsize=(6,6))
ax.imshow(grid, cmap="gray_r")

# スタートとゴール
ax.plot(start_pos[1], start_pos[0], "bo", label="Start")
ax.plot(goal_pos[1], goal_pos[0], "ro", label="Goal")

# 経路全体（破線）
rr, cc = zip(*path)
ax.plot(cc, rr, "y--", linewidth=1, label="Planned Path")

# ロボットの点
robot_dot, = ax.plot([], [], "go", markersize=10)

def init():
    robot_dot.set_data([], [])
    return robot_dot,

def update(frame):
    r, c = path[frame]
    robot_dot.set_data([c], [r])  # ★ 修正: sequence 渡し
    return robot_dot,

ani = FuncAnimation(fig, update, frames=len(path), init_func=init,
                    interval=300, blit=True, repeat=False)

ax.legend()
plt.show()
