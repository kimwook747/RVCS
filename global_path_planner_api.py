import numpy as  np
import heapq
from collections import deque
from skimage.measure import label
from scipy.ndimage import distance_transform_edt, center_of_mass
from typing import List, Tuple, Dict

Point = Tuple[int, int]
Path = List[Point]

def astar_path(grid_map: np.ndarray, start: Point, end: Point) -> Path:
    height, width = grid_map.shape
    start_node, end_node = tuple(map(int, start)), tuple(map(int, end))

    if not (0<=start_node[0] < height and 0 <= start_node[1] < width and \
            0 <= end_node[0] < height and 0 <= end_node[1] < width) or \
        grid_map[start_node]==1 or grid_map[end_node] == 1:
            return []
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    open_heap = [(0,start_node)]
    came_from: Dict[Point, Point] = {}
    g_score: Dict[Point, float] = {start_node: 0}

    while open_heap:
        _, current_node = heapq.heappop(open_heap)
        if current_node == end_node:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start_node)
            return path[::-1]
        
        for i, j in neighbors:
            neighbor_node = (current_node[0] + i, current_node[1] + j)
            if not (0 <= neighbor_node[0] < height and 0 <= neighbor_node[1] < width) or grid_map[neighbor_node] == 1:
                continue

            if i != 0 and j != 0 and (grid_map[current_node[0]+i, current_node[1]]==1 and grid_map[current_node[0], current_node[1]+j]==1):
                continue
            tentative_g_score = g_score[current_node] + (1.414 if i != 0 and j != 0 else 1)
            if tentative_g_score < g_score.get(neighbor_node, float('inf')):
                came_from[neighbor_node] = current_node
                g_score[neighbor_node] = tentative_g_score
                heuristic = np.hypot(neighbor_node[0] - end_node[0], neighbor_node[1] - end_node[1])
                f_score = tentative_g_score + heuristic
                heapq.heappush(open_heap, (f_score, neighbor_node))
    return []

class IntelligentCoveragePlanner:
    def __init__(self, grid_map: np.ndarray):
        self.original_grid_map = grid_map
        self.height, self.width = grid_map.shape

        self.safety_margin = 1.0
        distance_from_obstacle = distance_transform_edt(self.original_grid_map == 0)
        self.grid_map = (distance_from_obstacle < self.safety_margin).astype(np.uint8)

        self.labeled_map: np.ndarray = None
        self.via_points: set[Point] = set()

    def plan(self) -> np.ndarray:
        if np.all(self.grid_map == 1):
            return np.array([])
        self._detect_via_points()
        coverage_path, visited_cells = self._generate_coverage_path()
        if not coverage_path:
            return np.array([])

        wall_following_path = self._generate_wall_following_path(coverage_path[-1])
        full_path = coverage_path + wall_following_path
        return np.array(full_path)[:, ::-1] if full_path else np.array([])

    def _detect_via_points(self):
        via_point_distance = 2.0
        min_distance_between_points = 5.0

        candidate_coords = np.argwhere(np.isclose(distance_transform_edt(self.grid_map == 0), via_point_distance, atol=0.5))

        if len(candidate_coords) > 0:
            candidate_coords = candidate_coords[np.lexsort((candidate_coords[:,1], candidate_coords[:,0]))]
            self.via_points.add(tuple(candidate_coords[0]))
            for coord in candidate_coords[1:]:
                point = tuple(coord)
                if all(np.linalg.norm(np.array(point) - np.array(p)) >= min_distance_between_points for p in self.via_points):
                    self.via_points.add(point)

    def _generate_coverage_path(self) -> Tuple[Path, set]:
        path = []
        visited_cells = set()
        
        free_space_points = np.argwhere(self.grid_map == 0)
        if free_space_points.size == 0: return [], set()

        current_pos = tuple(free_space_points[0])
        path.append(current_pos)
        visited_cells.add(current_pos)

        y_coords = sorted(list(set(p[0] for p in free_space_points)))
        direction = 1
        for y in y_coords:
            x_coords_on_line = sorted([p[1] for p in free_space_points if p[0]  == y])
            if not x_coords_on_line: continue
            if direction == -1: x_coords_on_line.reverse()
            for x in x_coords_on_line:
                target_pos = (y,x)
                if target_pos not in visited_cells:
                    segment = astar_path(self.grid_map, current_pos, target_pos)
                    if segment:
                        for pos in segment:
                            visited_cells.add(tuple(pos))
                        path.extend(segment[1:])
                        current_pos = segment[-1]
            direction *= -1
        return path, visited_cells

    def _generate_wall_following_path(self, start_pos: Point) -> Path:
        if not self.via_points: return []
        wall_tour_path = []
        current_pos = start_pos
        unvisited_points = set(self.via_points)

        while unvisited_points:
            next_target = min(unvisited_points, key=lambda p: np.linalg.norm(np.array(current_pos) - np.array(p)))
            unvisited_points.remove(next_target)
            path_to_target = astar_path(self.grid_map, current_pos, next_target)
            if path_to_target:
                wall_tour_path.extend(path_to_target[1:])
                current_pos = path_to_target[-1]
        return wall_tour_path
    
    def get_global_path(grid_map: np.ndarray) -> np.ndarray:
        planner = IntelligentCoveragePlanner(grid_map)
        final_path = planner.plan()
        if final_path.size == 0:
            return np.array([])
        return final_path

if __name__ == '__main__':
    map_width, map_height = 100, 100
    example_map = np.zeros((map_height, map_width), dtype=np.uint8)

    example_map[[0,-1],:] = 1
    example_map[:,[0,-1]] = 1

    np.random.seed(42)
    for _ in range(10):
        obs_w, obs_h = np.random.randint(5,15), p.random.randint(5,15)
        x, y = np.random.randint(1, map_width - obs_w -1), np.random.randint(1, map_height-obs_h-1)
        example_map[y:y+obs_h, x:x+obs_w] = 1

    final_path = get_global_path(example_map)

    if final_path.size > 0:
        try:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10,10))
            planner_for_viz = IntelligentCoveragePlanner(example_map)
            display_map = np.copy(planner_for_viz.original_grid_map).astype(float)
            display_map[planner_for_viz.grid_map == 1] = 0.5

            plt.imshow(display_map, cmap='gray_r', origin='upper', interpolation='none')
            plt.plot(final_path[:,0], final_path[:,1], 'b-', linewidth=1.2, alpha=0.9, label='GloablPath')
            plt.plot(final_path[0,0], final_path[0,1], 'go', marketsize=8, label='Start')
            plt.plot(final_path[-1:0], final_path[-1:1], 'ro', linewidth=8, label='End')
            plt.legent()
            plt.show()
        except ImportError:
            print("Matplotlib is not installed. Cannot visualize the path.")
    else:
            print("\nPath create failed")
