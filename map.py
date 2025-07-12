import numpy as np
import os

class Map:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)
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
            if self.grid[y, x] == 0:
                self.grid[y, x] = 1

    def add_obstacle(self, x, y):
        if 0 <= int(x) < self.width and 0 <= int(y) < self.height:
            self.grid[int(y), int(x)] = 1

    def is_obstacle(self, x, y):
        x_int, y_int = int(x), int(y)
        if not (0 <= x_int < self.width and 0 <= y_int < self.height):
            return True
        return self.grid[y_int, x_int] == 1
