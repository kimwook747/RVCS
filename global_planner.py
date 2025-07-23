# global_planner.py

import numpy as np

class GlobalPlanner:
    def __init__(self, global_map):
        self.map = global_map

    def generate_path(self, start_pos, goal_pos):
        path = [start_pos]
        scan_direction = 1
        for y in range(3, self.map.height - 1):
            row_path = []
            start_x = 1 if scan_direction == 1 else self.map.width - 2
            end_x = self.map.width - 1 if scan_direction == 1 else 0
            
            x_coords = range(start_x, end_x, scan_direction)
            for x in x_coords:
                 if not self.map.is_obstacle(x, y):
                    row_path.append((x, y))

            if not row_path: continue
            
            # If scan direction changed, connect the last point of the previous row
            if path and scan_direction == -1 and path[-1][1] < y:
                path.append((path[-1][0], y + 0.5))
            elif path and scan_direction == 1 and path[-1][1] < y:
                 path.append((row_path[0][0], y-0.5))

            path.extend(row_path)
            scan_direction *= -1
        
        path.append(goal_pos)
        return path