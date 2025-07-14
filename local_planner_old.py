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

        if self.state != "DETOURING":
            is_new_obs_center = self._is_new_obstacle_at_sensor(1)
            if is_new_obs_center and self.robot.sensors.readings[1] < CONFIG['detour_trigger_dist']:
                print("INFO: New obstacle CENTER. -> DETOURING")
                self.state = "DETOURING"
                obstacle_pos = self._get_sensor_hit_point(1)
                self.generate_detour_path(obstacle_pos)
                return 0, 0

        if self.state == "PATH_FOLLOWING":
            return self.path_following()
        elif self.state == "DETOURING":
            return self.detour_obstacle()
        
        return 0, 0

    def path_following(self):
        if self.current_global_idx >= len(self.global_path): return 0, 0

        target = self.global_path[self.current_global_idx]
        
        if np.linalg.norm(self.robot.pos - np.array(target)) < 0.6:
            self.current_global_idx += 1
            if self.current_global_idx >= len(self.global_path):
                return 0, 0
        
        # 1. 경로 추종을 위한 제어값 계산
        v_base, w_path = self.calculate_vw_for_path(self.global_path[self.current_global_idx])

        # 2. 측면 장애물 회피를 위한 제어값 계산
        w_avoid = self._calculate_p_correction()

        # 3. 두 제어값을 가중치를 적용하여 혼합
        final_w = (self.pid_config['path_weight'] * w_path) + \
                  (self.pid_config['avoid_weight'] * w_avoid)
        final_w = np.clip(final_w, -CONFIG['robot_max_w'], CONFIG['robot_max_w'])

                # [추가] U턴 구간(맵 가장자리)인지 확인
        is_in_uturn_zone = (self.robot.pos[0] > CONFIG['grid_width'] - 2.0) or \
                           (self.robot.pos[0] < 2.0)

        # [추가] U턴 구간이라면 속도를 50%로 줄임
        if is_in_uturn_zone:
            v_base *= 0.5

        return v_base, final_w

    def _calculate_p_correction(self):
        """
        [수정] 측면과의 거리를 유지하기 위한 P(비례) 제어기.
        Kd=0 이므로 사실상 PD가 아닌 P 제어기.
        """
        sensors = self.robot.sensors.readings
        left_dist, right_dist = sensors[5], sensors[6]
        target_dist = self.pid_config['target_distance']
        error = 0.0

        # 센서가 감지했을 때만 오차 계산
        # 오른쪽 벽과의 오차 (오른쪽이 가까우면 error는 양수 -> 좌회전)
        if right_dist < CONFIG['sensors']['range']:
            error += (target_dist - right_dist)

        # 왼쪽 벽과의 오차 (왼쪽이 가까우면 error는 음수 -> 우회전)
        if left_dist < CONFIG['sensors']['range']:
            error -= (target_dist - left_dist)
        
        # P 제어: 오차에 비례하여 회전 속도 결정
        w_correction = self.pid_config['Kp'] * error
        return w_correction

    def detour_obstacle(self):
        if not self.detour_path:
            print("INFO: Detour complete. Re-acquiring global path.")
            self.state = "PATH_FOLLOWING"
            self.current_global_idx = self._find_closest_future_path_index()
            return self.plan()
            
        target = self.detour_path[0]
        if np.linalg.norm(self.robot.pos - np.array(target)) < 0.7:
            self.detour_path.pop(0)
            if not self.detour_path: 
                return 0, 0
        
        current_target = self.detour_path[0] if self.detour_path else target
        return self.calculate_vw_for_path(current_target)

    def calculate_vw_for_path(self, target):
        angle_to_target = np.arctan2(target[1] - self.robot.pos[1], target[0] - self.robot.pos[0])
        angle_diff = (angle_to_target - self.robot.theta + np.pi) % (2 * np.pi) - np.pi
        
        dist_to_target = np.linalg.norm(self.robot.pos - np.array(target))
        
        if dist_to_target > 1.5:
            turn_gain = 2.0
        else:
            turn_gain = 1.2
        
        w = angle_diff * turn_gain
        v = CONFIG['robot_max_v']
        
        if abs(angle_diff) > np.deg2rad(90):
            v *= 0.2
        elif abs(angle_diff) > np.deg2rad(45):
            v *= 0.5
            
        return v, w
        
    def _is_new_obstacle_at_sensor(self, sensor_idx):
        if self.robot.sensors.readings[sensor_idx] < self.robot.sensors.range:
            hit_point = self._get_sensor_hit_point(sensor_idx)
            if self.local_map.is_obstacle(hit_point[0], hit_point[1]) and \
               not self.global_map.is_obstacle(hit_point[0], hit_point[1]):
                return True
        return False

    def _get_sensor_hit_point(self, sensor_idx):
        spec = self.robot.sensors.specs[sensor_idx]
        dist = self.robot.sensors.readings[sensor_idx]
        angle_offset, x_offset, y_offset = spec
        rotated_offset_x = x_offset * np.cos(self.robot.theta) - y_offset * np.sin(self.robot.theta)
        rotated_offset_y = x_offset * np.sin(self.robot.theta) + y_offset * np.cos(self.robot.theta)
        start_x = self.robot.pos[0] + rotated_offset_x
        start_y = self.robot.pos[1] + rotated_offset_y
        sensor_angle = self.robot.theta + angle_offset
        return (start_x + dist * np.cos(sensor_angle), start_y + dist * np.sin(sensor_angle))

    def _find_closest_future_path_index(self):
        if self.current_global_idx >= len(self.global_path) - 2:
            return self.current_global_idx
        p1 = np.array(self.global_path[self.current_global_idx])
        p2 = np.array(self.global_path[self.current_global_idx + 1])
        direction = p2 - p1
        if abs(direction[0]) > abs(direction[1]): main_axis = 0
        else: main_axis = 1
        for i in range(self.current_global_idx, len(self.global_path)):
            path_point = self.global_path[i]
            if np.sign(direction[main_axis]) * (path_point[main_axis] - self.robot.pos[main_axis]) > 0:
                best_i = i
                min_dist = float('inf')
                for j in range(i, min(i + 5, len(self.global_path))):
                    dist = np.linalg.norm(self.robot.pos - np.array(self.global_path[j]))
                    if dist < min_dist:
                        min_dist = dist
                        best_i = j
                print(f"Path re-acquired at index {best_i}")
                return best_i
        return self.current_global_idx

    def generate_detour_path(self, first_obstacle_pos):
        self.detour_path = []
        clearance = CONFIG['robot_radius'] * 3.0
        forward_direction = np.sign(np.cos(self.robot.theta))
        if forward_direction == 0: forward_direction = 1
        for turn_dir in [-1, 1]:
            temp_path = []
            robot_pos = self.robot.pos
            p1 = list(robot_pos)
            p1[1] += turn_dir * clearance
            if self.local_map.is_obstacle(p1[0], p1[1]): continue
            temp_path.append(tuple(p1))
            p2 = list(p1)
            p2[0] += forward_direction * 2.5 
            if self.local_map.is_obstacle(p2[0], p2[1]): continue
            temp_path.append(tuple(p2))
            p3 = list(p2)
            p3[1] -= turn_dir * clearance
            if self.local_map.is_obstacle(p3[0], p3[1]): continue
            temp_path.append(tuple(p3))
            self.detour_path = temp_path
            print(f"  - Detour path found, direction: {forward_direction}")
            break
        if not self.detour_path:
            print("WARN: Could not find a clear detour path. Robot will stop.")
