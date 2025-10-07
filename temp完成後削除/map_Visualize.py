import numpy as np
import matplotlib.pyplot as plt

grid = np.zeros((120,200))
# 障害物セルを1に設定
grid[10:20,50:150] = 1

plt.imshow(grid, cmap='gray_r')
plt.show()
