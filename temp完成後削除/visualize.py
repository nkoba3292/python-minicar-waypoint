import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
from cource_map import grid_matrix, world_to_grid, start_pos, goal_pos, obstacles, start_lines, pylons

def animate_path(path):
    fig, ax = plt.subplots(dpi=120)
    ax.imshow(grid_matrix, cmap="Greys", origin="lower")

    # スタート・ゴール
    ax.plot(start_pos[0], start_pos[1], "go", label="Start")
    ax.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")

    # 障害物（薄緑）
    for obs in obstacles:
        x0, y0 = world_to_grid(*obs["start"])
        x1, y1 = world_to_grid(*obs["end"])
        left, bottom = min(x0, x1), min(y0, y1)
        width, height = abs(x1 - x0), abs(y1 - y0)
        rect = Rectangle((left, bottom), width, height, color="lightgreen", alpha=0.5)
        ax.add_patch(rect)

    # パイロン（濃いオレンジ）
    for p in pylons:
        x, y = world_to_grid(*p["pos"])
        c = Circle((x, y), radius=1.5, color="#FF8C00", alpha=0.9)
        ax.add_patch(c)

    # スタートライン（青線）
    for line_def in start_lines:
        x0, y0 = world_to_grid(*line_def["start"])
        x1, y1 = world_to_grid(*line_def["end"])
        ax.plot([x0, x1], [y0, y1], "b-", lw=2)

    # 経路用ラインと点（破線）
    line, = ax.plot([], [], "b--", lw=2)
    point, = ax.plot([], [], "ro", markersize=6)

    def update(i):
        if i < len(path):
            xs = [p[0] for p in path[:i+1]]
            ys = [p[1] for p in path[:i+1]]
            line.set_data(xs, ys)
            point.set_data([xs[-1]], [ys[-1]])
        return line, point

    # intervalを33msにして1.5倍速に
    ani = animation.FuncAnimation(fig, update, frames=len(path), interval=33, blit=True)
    plt.legend()
    plt.show()
