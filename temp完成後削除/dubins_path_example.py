import numpy as np
import matplotlib.pyplot as plt

def circle(center, radius, start_angle, end_angle, n=100):
    """半径 radius の円弧を生成"""
    angles = np.linspace(start_angle, end_angle, n)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return x, y

def dubins_path_example():
    turning_radius = 1.0

    # 開始姿勢 (x, y, theta)
    x0, y0, theta0 = 0, 0, 0

    # 終了姿勢 (x, y, theta)
    x1, y1, theta1 = 4, 4, np.pi/2

    # --- 開始円弧（左回転） ---
    # 仮に中心を開始位置の左にとる
    center_start = (x0, y0 + turning_radius)
    x_arc1, y_arc1 = circle(center_start, turning_radius, -np.pi/2, 0)

    # --- 直線部分 ---
    x_line = np.linspace(x_arc1[-1], x1 - turning_radius, 50)
    y_line = np.linspace(y_arc1[-1], y1 - turning_radius, 50)

    # --- 終了円弧（右回転） ---
    center_end = (x1 - turning_radius, y1)
    x_arc2, y_arc2 = circle(center_end, turning_radius, np.pi, np.pi/2)

    # --- 描画 ---
    plt.plot(x_arc1, y_arc1, 'r', label='Start Arc')
    plt.plot(x_line, y_line, 'g', label='Straight')
    plt.plot(x_arc2, y_arc2, 'b', label='End Arc')
    plt.scatter([x0, x1], [y0, y1], c='k', marker='o', label='Start/Goal')
    plt.axis('equal')
    plt.legend()
    plt.title("Dubins-like Path Example")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

# 実行
dubins_path_example()
