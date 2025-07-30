# map.py

import numpy as np
import os

class Map:
    def __init__(self, config):
        self.width = config['grid_width']
        self.height = config['grid_height']
        self.grid = np.zeros((self.height, self.width), dtype=int)
        self._add_walls()

    def _add_walls(self):
        self.grid[0, :] = 1
        self.grid[-1, :] = 1
        self.grid[:, 0] = 1
        self.grid[:, -1] = 1

    def from_file(self, filename, obstacle_count):
        if os.path.exists(filename):
            self.grid = np.loadtxt(filename, dtype=int)
            self.height, self.width = self.grid.shape
            self._add_walls()
            print(f"Loaded map from '{filename}'.")
        else:
            self._generate_random_obstacles(obstacle_count)
            np.savetxt(filename, self.grid, fmt='%d')
            print(f"'{filename}' not found. Generated a new random map.")

    def _generate_random_obstacles(self, obstacle_count):
        for _ in range(obstacle_count):
            x, y = np.random.randint(2, self.width - 2), np.random.randint(2, self.height - 2)
            # if self.grid[y, x] == 0:   # CHECK
            self.grid[y, x] = 1

    def add_rectangular_obstacle(self, start_x, start_y, width, height):
        """지정된 시작점과 크기로 사각형 장애물을 그리드에 추가합니다."""
        x_start, y_start = int(start_x), int(start_y)
        w, h = int(width), int(height)

        for y in range(y_start, y_start + h):
            for x in range(x_start, x_start + w):
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.grid[y, x] = 1

    def is_obstacle(self, x, y):
        x_int, y_int = int(x), int(y)
        if not (0 <= x_int < self.width and 0 <= y_int < self.height):
            return True
        return self.grid[y_int, x_int] == 1