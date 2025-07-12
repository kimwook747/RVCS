<<<<<<< HEAD
# visualizer.py

=======
>>>>>>> d097e19fe20eacddd8a2556d2c0595dad21618d6
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
from config import CONFIG

class Visualizer:
    def __init__(self, sim_map):
        self.map = sim_map
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(CONFIG['grid_width']/2.5, CONFIG['grid_height']/2.5))

<<<<<<< HEAD
    def draw(self, robot, global_path, local_planner, new_obstacle_info):
=======
    def draw(self, robot, global_path, local_planner, new_obstacle_pos):
>>>>>>> d097e19fe20eacddd8a2556d2c0595dad21618d6
        self.ax.cla()
        
        # Draw Map and Obstacles
        for y in range(self.map.height):
            for x in range(self.map.width):
                if self.map.grid[y, x] == 1:
                    is_wall = x==0 or x==self.map.width-1 or y==0 or y==self.map.height-1
                    color = CONFIG['colors']['wall'] if is_wall else CONFIG['colors']['static_obstacle']
                    rect = patches.Rectangle((x, y), 1, 1, facecolor=color)
                    self.ax.add_patch(rect)
<<<<<<< HEAD
        
        # [수정됨] 사각형 장애물을 그리는 로직
        if new_obstacle_info:
            x, y, w, h = new_obstacle_info
            rect = patches.Rectangle((x, y), w, h, facecolor=CONFIG['colors']['new_obstacle'])
=======
        if new_obstacle_pos:
            rect = patches.Rectangle(new_obstacle_pos, 1, 1, facecolor=CONFIG['colors']['new_obstacle'])
>>>>>>> d097e19fe20eacddd8a2556d2c0595dad21618d6
            self.ax.add_patch(rect)

        # Draw Paths
        g_path_arr = np.array(global_path)
        self.ax.plot(g_path_arr[:, 0], g_path_arr[:, 1], color=CONFIG['colors']['global_path_color'], linestyle='--', lw=1.5, label="Global Path")
        if local_planner.detour_path:
            d_path_arr = np.array([robot.pos] + local_planner.detour_path)
            self.ax.plot(d_path_arr[:, 0], d_path_arr[:, 1], color='cyan', linestyle='-', lw=2.0, label="Detour Path")
        
        # Draw Robot Trail
        robot_trail = np.array(robot.path_history)
        self.ax.plot(robot_trail[:, 0], robot_trail[:, 1], color=CONFIG['colors']['robot_trail'], lw=1.5, alpha=0.7)
        
        # Draw Sensors
        self._draw_sensors(robot)

        # Draw Robot
        robot_circle = patches.Circle(robot.pos, CONFIG['robot_radius'], facecolor=CONFIG['colors']['robot'])
        self.ax.add_patch(robot_circle)
        self.ax.arrow(robot.pos[0], robot.pos[1], 
                      CONFIG['robot_radius'] * 1.5 * np.cos(robot.theta), 
                      CONFIG['robot_radius'] * 1.5 * np.sin(robot.theta), 
                      head_width=0.4, color=CONFIG['colors']['robot_arrow'])

        # Plot Settings
        self.ax.set_title(f"State: {local_planner.state}")
        self.ax.set_xlim(0, self.map.width)
        self.ax.set_ylim(0, self.map.height)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.legend(fontsize='small', loc='upper right')
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _draw_sensors(self, robot):
        colors = CONFIG['colors']
        for i, dist in enumerate(robot.sensors.readings):
            spec = robot.sensors.specs[i]
            angle_offset, x_offset, y_offset = spec
            rotated_offset_x = x_offset * np.cos(robot.theta) - y_offset * np.sin(robot.theta)
            rotated_offset_y = x_offset * np.sin(robot.theta) + y_offset * np.cos(robot.theta)
            start_x, start_y = robot.pos[0] + rotated_offset_x, robot.pos[1] + rotated_offset_y
            end_x = start_x + dist * np.cos(robot.theta + angle_offset)
            end_y = start_y + dist * np.sin(robot.theta + angle_offset)
            color = colors['sensor_hit'] if dist < CONFIG['sensors']['range'] else colors['sensor_ray']
<<<<<<< HEAD
            self.ax.plot([start_x, end_x], [start_y, end_y], color=color, alpha=0.8, lw=1)
=======
            self.ax.plot([start_x, end_x], [start_y, end_y], color=color, alpha=0.8, lw=1)
>>>>>>> d097e19fe20eacddd8a2556d2c0595dad21618d6
