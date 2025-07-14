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
        # [수정] 변수명을 prev_error로 변경
        self.prev_error = 0.0

    def plan(self):
        self.robot.update_sensors(self.local_map)

        if self.state == "PATH_FOLLOWING":
            return self.path_following()
        elif self.state == "DETOURING":
            # [수정] 상태 변경 시 이전 오차 리셋
            self.prev_error = 0.0
            return self.detour_obstacle()
        return 0, 0

    def path_following(self):
        if self.current_global_idx >= len(self.global_path): return 0, 0

        if self.check_and_manage_obstacles():
            self.state = "DETOURING"
            if self.detour_planned:
                self.detour_planned = False
                return self.plan()
            else:
                print("STOP: Obstacle appeared suddenly. Planning and stopping.")
                return 0, 0
        
        target = self.global_path[self.current_global_idx]
        if np.linalg.norm(self.robot.pos - np.array(target)) < 0.6:
            self.current_global_idx += 1
            if self.current_global_idx >= len(self.global_path):
                return 0, 0
        
        v_base, w_path = self.calculate_vw_for_path(self.global_path[self.current_global_idx])
        w_avoid = self._calculate_pd_correction()

        # [수정] 조건부 벽면 회피 로직 제거. 항상 혼합 제어를 사용.
        final_w = (self.pid_config['path_weight'] * w_path) + \
                  (self.pid_config['avoid_weight'] * w_avoid)
        
        final_w = np.clip(final_w, -CONFIG['robot_max_w'], CONFIG['robot_max_w'])

        is_in_uturn_zone = (self.robot.pos[0] > CONFIG['grid_width'] - 2.0) or (self.robot.pos[0] < 2.0)
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
        
        # [수정] 변수명을 prev_error로 변경
        derivative = (error - self.prev_error) / CONFIG['dt']
        self.prev_error = error
        
        w_correction = (self.pid_config['Kp'] * error) + (self.pid_config['Kd'] * derivative)
        
        return w_correction

    def check_and_manage_obstacles(self):
        look_ahead_dist_far = 1.0
        check_point_x_far = self.robot.pos[0] + look_ahead_dist_far * np.cos(self.robot.theta)
        check_point_y_far = self.robot.pos[1] + look_ahead_dist_far * np.sin(self.robot.theta)
        is_new_obstacle_far = self.local_map.is_obstacle(check_point_x_far, check_point_y_far) and \
                              not self.global_map.is_obstacle(check_point_x_far, check_point_y_far)

        if is_new_obstacle_far and not self.detour_planned:
            print(f"INFO: New obstacle detected at {look_ahead_dist_far:.1f}m. Planning detour.")
            self.detour_planned = True
            self.generate_detour_path((check_point_x_far, check_point_y_far))
            return False

        look_ahead_dist_near = 0.8
        check_point_x_near = self.robot.pos[0] + look_ahead_dist_near * np.cos(self.robot.theta)
        check_point_y_near = self.robot.pos[1] + look_ahead_dist_near * np.sin(self.robot.theta)
        is_new_obstacle_near = self.local_map.is_obstacle(check_point_x_near, check_point_y_near) and \
                               not self.global_map.is_obstacle(check_point_x_near, check_point_y_near)
        
        if is_new_obstacle_near:
            if self.detour_planned:
                print(f"INFO: Obstacle at {look_ahead_dist_near:.1f}m. Executing planned detour.")
            else:
                print(f"WARN: Obstacle appeared suddenly at {look_ahead_dist_near:.1f}m. Planning detour now.")
                self.generate_detour_path((check_point_x_near, check_point_y_near))
            return True
        return False
    
    def generate_detour_path(self, first_obstacle_pos):
        if self.current_global_idx >= len(self.global_path): return
        target = self.global_path[self.current_global_idx]
        path_vec = np.array(target) - self.robot.pos
        if np.linalg.norm(path_vec) < 1e-6: path_vec = np.array([np.cos(self.robot.theta), np.sin(self.robot.theta)])

        if abs(path_vec[0]) > abs(path_vec[1]):
            dominant_axis_idx, perp_axis_idx = 0, 1
        else:
            dominant_axis_idx, perp_axis_idx = 1, 0
        scan_direction = np.sign(path_vec[dominant_axis_idx]) if path_vec[dominant_axis_idx] != 0 else 1
        
        last_obstacle_pos = list(first_obstacle_pos)
        for _ in range(10):
            next_pos = list(last_obstacle_pos)
            next_pos[dominant_axis_idx] += scan_direction * 0.5
            if not self.local_map.is_obstacle(next_pos[0], next_pos[1]):
                break
            last_obstacle_pos = next_pos
        
        self.detour_path = []
        for turn_dir in [1, -1]:
            temp_path = []
            clearance = CONFIG['robot_radius'] * 2.5
            
            p1 = list(self.robot.pos); p1[perp_axis_idx] += turn_dir * clearance
            if self.local_map.is_obstacle(p1[0], p1[1]): continue
            temp_path.append(tuple(p1))
            
            p_exit_buffer = list(last_obstacle_pos)
            p_exit_buffer[dominant_axis_idx] += scan_direction * (CONFIG['robot_radius'] + 0.5)
            p2 = list(p1)
            p2[dominant_axis_idx] = p_exit_buffer[dominant_axis_idx]
            if self.local_map.is_obstacle(p2[0], p2[1]): continue
            temp_path.append(tuple(p2))

            p3 = list(p2)
            p3[perp_axis_idx] = last_obstacle_pos[perp_axis_idx]
            if self.local_map.is_obstacle(p3[0], p3[1]): continue
            temp_path.append(tuple(p3))

            self.detour_path = temp_path
            print(f"  - Detour path found: {self.detour_path}")
            break
            
        if not self.detour_path: print("WARN: Could not find a clear detour path.")

    def detour_obstacle(self):
        if not self.detour_path:
            print("INFO: Detour complete. Resuming global path.")
            self.state = "PATH_FOLLOWING"
            self.detour_planned = False
            self.current_global_idx = self._find_closest_future_path_index()
            return self.plan()
            
        target = self.detour_path[0]
        if np.linalg.norm(self.robot.pos - np.array(target)) < 0.7:
            self.detour_path.pop(0)
            if not self.detour_path: 
                return self.plan()
        
        current_target = self.detour_path[0] if self.detour_path else target
        return self.calculate_vw_for_path(current_target)

    def calculate_vw_for_path(self, target):
        angle_to_target = np.arctan2(target[1] - self.robot.pos[1], target[0] - self.robot.pos[0])
        angle_diff = (angle_to_target - self.robot.theta + np.pi) % (2 * np.pi) - np.pi
        w = np.clip(angle_diff * 2.0, -CONFIG['robot_max_w'], CONFIG['robot_max_w'])
        v = CONFIG['robot_max_v']
        if abs(angle_diff) > np.deg2rad(45): v *= 0.5
        return v, w

    def _find_closest_future_path_index(self):
        if self.current_global_idx >= len(self.global_path) - 1:
            return self.current_global_idx
        min_dist = float('inf')
        best_idx = self.current_global_idx
        for i in range(self.current_global_idx, min(self.current_global_idx + 15, len(self.global_path))):
            dist = np.linalg.norm(self.robot.pos - np.array(self.global_path[i]))
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        print(f"Path re-acquired at index {best_idx}")
        return best_idx
