import numpy as np
from config import CONFIG

class Robot:
    class Sensor:
        def __init__(self):
            self.specs = CONFIG['sensors']['specs']
            self.range = CONFIG['sensors']['range']
            self.readings = [self.range] * len(self.specs)

        def update(self, robot_pose, robot_theta, local_map):
            self.readings = []
            for angle_offset, x_offset, y_offset in self.specs:
                rotated_offset_x = x_offset * np.cos(robot_theta) - y_offset * np.sin(robot_theta)
                rotated_offset_y = x_offset * np.sin(robot_theta) + y_offset * np.cos(robot_theta)
                
                start_x = robot_pose[0] + rotated_offset_x
                start_y = robot_pose[1] + rotated_offset_y
                sensor_angle = robot_theta + angle_offset
                
                dist = self._cast_ray(start_x, start_y, sensor_angle, local_map)
                self.readings.append(dist)
        
        def _cast_ray(self, x, y, angle, local_map):
            step_size = 0.1
            for d in np.arange(0, self.range, step_size):
                check_x = x + d * np.cos(angle)
                check_y = y + d * np.sin(angle)
                if local_map.is_obstacle(check_x, check_y):
                    return d
            return self.range

    def __init__(self, x, y):
        self.pos = np.array([x, y], dtype=float)
        self.theta = 0.0
        self.path_history = [self.pos.copy()]
        self.sensors = self.Sensor()

    def update_sensors(self, local_map):
        self.sensors.update(self.pos, self.theta, local_map)

    def move(self, v, w, dt):
        if abs(w) < 1e-6:
            self.pos[0] += v * np.cos(self.theta) * dt
            self.pos[1] += v * np.sin(self.theta) * dt
        else:
            R = v / w
            delta_theta = w * dt
            icc_x = self.pos[0] - R * np.sin(self.theta)
            icc_y = self.pos[1] + R * np.cos(self.theta)
            self.pos[0] = icc_x + R * np.sin(self.theta + delta_theta)
            self.pos[1] = icc_y - R * np.cos(self.theta + delta_theta)
            self.theta += delta_theta
            self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        self.path_history.append(self.pos.copy())
