import numpy as np
from config import CONFIG

class LocalPlanner:
    def __init__(self, robot, global_map, local_map, global_path):
        self.robot = robot
        self.global_map = global_map
        self.local_map = local_map
        self.global_path = global_path
        self.current_global_idx = 0
        self.state = "PATH_FOLLOWING"
        self.detour_path = []
        self.pid_config = CONFIG['pid']
        self.detour_planned = False
        self.prev_error = 0.0

    def plan(self):
        self.robot.update_sensors(self.local_map)
        if self.state == "PATH_FOLLOWING":
            return self.path_following()
        elif self.state == "DETOURING":
            self.prev_error = 0.0
            return self.detour_obstacle()
        return 0, 0

    def path_following(self):
        if self.current_global_idx >= len(self.global_path):
            return 0, 0

        action_needed = self.check_and_manage_obstacles()

        if action_needed:
            self.state = "DETOURING"
            self.detour_planned = False
            return self.plan()

        if self.detour_planned:
            return 0, 0

        target_pos = np.array(self.global_path[self.current_global_idx], dtype=float)
        arrival_threshold = 0.6 * CONFIG.get('resolution', 1.0)
        
        if np.linalg.norm(self.robot.pos - target_pos) < arrival_threshold:
            self.current_global_idx += 1
            if self.current_global_idx >= len(self.global_path): return 0, 0
            target_pos = np.array(self.global_path[self.current_global_idx], dtype=float)
        
        v_base, w_path = self.calculate_vw_for_path(target_pos)
        w_avoid = self._calculate_pd_correction()

        final_w = (self.pid_config['path_weight'] * w_path) + (self.pid_config['avoid_weight'] * w_avoid)
        final_w = np.clip(final_w, -CONFIG['robot_max_w'], CONFIG['robot_max_w'])

        is_in_uturn_zone = (self.robot.pos[0] > CONFIG['grid_width'] - 2 * CONFIG.get('resolution', 1.0)) or \
                          (self.robot.pos[0] < 2 * CONFIG.get('resolution', 1.0))
        if is_in_uturn_zone:
            v_base *= 0.5

        return v_base, final_w
    
    def _calculate_pd_correction(self):
        sensors = self.robot.sensors.readings
        left_dist, right_dist = sensors[5], sensors[6]
        target_dist = self.pid_config['target_distance']
        
        error = 0.0
        if right_dist < CONFIG['sensors']['range']: error += (target_dist - right_dist)
        if left_dist < CONFIG['sensors']['range']: error -= (target_dist - left_dist)
        
        derivative = (error - self.prev_error) / CONFIG['dt']
        self.prev_error = error
        
        w_correction = (self.pid_config['Kp'] * error) + (self.pid_config['Kd'] * derivative)
        
        return w_correction

    def check_and_manage_obstacles(self):
        look_ahead_dist = CONFIG['detour_trigger_dist']
        check_pos_x = self.robot.pos[0] + look_ahead_dist * np.cos(self.robot.theta)
        check_pos_y = self.robot.pos[1] + look_ahead_dist * np.sin(self.robot.theta)
        is_new_obstacle = self.local_map.is_obstacle(check_pos_x, check_pos_y) and \
                              not self.global_map.is_obstacle(check_pos_x, check_pos_y)

        if is_new_obstacle:
            if not self.detour_planned:
                print(f"INFO: New obstacle detected at {look_ahead_dist / CONFIG.get('resolution', 1.0):.1f}m. Planning detour.")
                self.detour_planned = True
                obstacle_grid = (int(round(check_pos_x)), int(round(check_pos_y)))
                self.generate_detour_path(obstacle_grid)
            else: # Plan already exists, now we are close enough to act
                print(f"INFO: Obstacle still ahead. Executing planned detour.")
                return True
        
        return False
    
    def generate_detour_path(self, first_obstacle_grid):
        if self.current_global_idx >= len(self.global_path): return
        
        target_pos = np.array(self.global_path[self.current_global_idx], dtype=float)
        path_vec = target_pos - self.robot.pos
        if np.linalg.norm(path_vec) < 1e-6: path_vec = np.array([np.cos(self.robot.theta), np.sin(self.robot.theta)])

        if abs(path_vec[0]) > abs(path_vec[1]):
            dominant_axis_idx, perp_axis_idx = 0, 1
        else:
            dominant_axis_idx, perp_axis_idx = 1, 0
        scan_direction = np.sign(path_vec[dominant_axis_idx]) if path_vec[dominant_axis_idx] != 0 else 1
        
        last_obstacle_grid = list(first_obstacle_grid)
        scan_range = int(round(5 * CONFIG.get('resolution', 1.0)))
        for _ in range(scan_range):
            next_grid = list(last_obstacle_grid)
            next_grid[dominant_axis_idx] += scan_direction
            if not self.local_map.is_obstacle(next_grid[0], next_grid[1]):
                break
            last_obstacle_grid = next_grid
        
        self.detour_path = []
        for turn_dir in [1, -1]:
            temp_path = []
            clearance = int(round(CONFIG['robot_radius'] * 2.0))
            robot_grid = [int(round(self.robot.pos[0])), int(round(self.robot.pos[1]))]
            
            p1_grid = list(robot_grid)
            p1_grid[perp_axis_idx] += turn_dir * clearance
            if self.local_map.is_obstacle(first_obstacle_grid[0], p1_grid[1]): continue
            temp_path.append(tuple(p1_grid))
            
            p_exit_buffer_grid = list(last_obstacle_grid)
            p_exit_buffer_grid[dominant_axis_idx] += scan_direction * int(round(CONFIG['robot_radius'] * 2))
            p2_grid = list(p1_grid)
            p2_grid[dominant_axis_idx] = p_exit_buffer_grid[dominant_axis_idx]
            if self.local_map.is_obstacle(p2_grid[0], p2_grid[1]): continue
            temp_path.append(tuple(p2_grid))

            p3_grid = list(p2_grid)
            p3_grid[perp_axis_idx] = last_obstacle_grid[perp_axis_idx]
            if self.local_map.is_obstacle(p3_grid[0], p3_grid[1]): continue
            temp_path.append(tuple(p3_grid))

            self.detour_path = temp_path
            print(f"  - Detour path found (grid indices): {self.detour_path}")
            break
            
        if not self.detour_path: print("WARN: Could not find a clear detour path.")

    def detour_obstacle(self):
        if not self.detour_path:
            print("INFO: Detour complete. Resuming global path.")
            self.state = "PATH_FOLLOWING"
            self.detour_planned = False
            self.current_global_idx = self._find_closest_future_path_index()
            return self.plan()
            
        target_pos = np.array(self.detour_path[0], dtype=float)
        
        arrival_threshold = 0.7 * CONFIG.get('resolution', 1.0)
        if np.linalg.norm(self.robot.pos - target_pos) < arrival_threshold:
            self.detour_path.pop(0)
            if not self.detour_path: return self.plan()
        
        current_target_pos = np.array(self.detour_path[0], dtype=float) if self.detour_path else target_pos
        return self.calculate_vw_for_path(current_target_pos)

    def calculate_vw_for_path(self, target_pos):
        angle_to_target = np.arctan2(target_pos[1] - self.robot.pos[1], target_pos[0] - self.robot.pos[0])
        angle_diff = (angle_to_target - self.robot.theta + np.pi) % (2 * np.pi) - np.pi
        w = np.clip(angle_diff * 2.0, -CONFIG['robot_max_w'], CONFIG['robot_max_w'])
        v = CONFIG['robot_max_v']
        if abs(angle_diff) > np.deg2rad(45): v *= 0.5
        return v, w

    def _find_closest_future_path_index(self):
        if self.current_global_idx >= len(self.global_path) - 1: return self.current_global_idx
        min_dist = float('inf')
        best_idx = self.current_global_idx
        
        search_window = 15
        for i in range(self.current_global_idx, min(self.current_global_idx + search_window, len(self.global_path))):
            path_pos = np.array(self.global_path[i], dtype=float)
            dist = np.linalg.norm(self.robot.pos - path_pos)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        print(f"Path re-acquired at index {best_idx}")
        return best_idx