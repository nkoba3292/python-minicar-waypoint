import heapq
import numpy as np
from cource_map import grid_matrix, start_pos, goal_pos
from scipy.interpolate import splprep, splev

# --- 8方向A*探索 ---
def astar_8dir(grid, start, goal):
    h, w = grid.shape
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        dx = abs(a[0]-b[0])
        dy = abs(a[1]-b[1])
        return max(dx, dy)  # チェビシェフ距離

    directions = [
        (0,1),(0,-1),(1,0),(-1,0),
        (1,1),(1,-1),(-1,1),(-1,-1)
    ]
    move_cost = [1,1,1,1,np.sqrt(2),np.sqrt(2),np.sqrt(2),np.sqrt(2)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for (dx, dy), cost in zip(directions, move_cost):
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < w and 0 <= neighbor[1] < h:
                if grid[neighbor[1], neighbor[0]] != 0:
                    continue
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current
    return []

# --- スプラインで滑らか経路生成 ---
def smooth_path(path, num_points=500, s=1.0):
    if not path:
        return []
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    # スプライン補間
    tck, u = splprep([x, y], s=s)
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)
    return list(zip(x_new, y_new))

# --- 外部呼び出し ---
def compute_path():
    print("start_pos:", start_pos)
    print("goal_pos:", goal_pos)
    print("grid_matrix shape:", grid_matrix.shape)
    print("grid_matrix[start]:", grid_matrix[start_pos[1], start_pos[0]])
    print("grid_matrix[goal]:", grid_matrix[goal_pos[1], goal_pos[0]])
    path = astar_8dir(grid_matrix, start_pos, goal_pos)
    if not path:
        print("Path not found!")
        return []
    return smooth_path(path)

if __name__ == "__main__":
    path = compute_path()
    print("path length:", len(path))
