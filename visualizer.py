# visualizer.py

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
from config import CONFIG # config만 import하는 것은 괜찮습니다.

class Visualizer:
    def __init__(self, sim_map):
        self.map = sim_map
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(CONFIG['grid_width']/2, CONFIG['grid_height']/2))

    def draw(self, robot, global_path, local_planner, new_obstacle_info):
        self.ax.cla()
        colors = CONFIG['colors']

        # 격자선 추가
        self.ax.grid(True, which='both', color='lightgray', linestyle='--', linewidth=0.5, alpha=0.7)
        self.ax.set_xticks(np.arange(-0.5, self.map.width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, self.map.height, 1), minor=True)
        self.ax.tick_params(which='minor', size=0)
        self.ax.set_xticks(np.arange(0, self.map.width, 5))
        self.ax.set_yticks(np.arange(0, self.map.height, 5))

        # 맵과 장애물 그리기
        for y in range(self.map.height):
            for x in range(self.map.width):
                if self.map.grid[y, x] == 1:
                    is_wall = x==0 or x==self.map.width-1 or y==0 or y==self.map.height-1
                    color = colors['wall'] if is_wall else colors.get('static_obstacle', 'gray')
                    rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=color)
                    self.ax.add_patch(rect)
        
        if new_obstacle_info:
            x, y, w, h = new_obstacle_info
            rect = patches.Rectangle((x - 0.5, y - 0.5), w, h, facecolor=colors.get('new_obstacle', 'purple'))
            self.ax.add_patch(rect)

        # 경로 그리기 (경유지 표시)
        if global_path is not None and len(global_path) > 0:
            g_path_arr = np.array(global_path)
            self.ax.plot(g_path_arr[:, 0], g_path_arr[:, 1], color=colors['global_path_color'], 
                         linestyle=colors.get('global_path_style', '--'), lw=1.5, label="Global Path", 
                         marker='o', markersize=3)
        
        if local_planner.detour_path:
            detour_points = [robot.pos] + local_planner.detour_path
            d_path_arr = np.array(detour_points)
            self.ax.plot(d_path_arr[:, 0], d_path_arr[:, 1], color=colors['detour_path_color'], 
                         linestyle='-', lw=2.0, label="Detour Path", marker='x', markersize=4)

        # 로봇 궤적 그리기
        robot_trail = np.array(robot.path_history)
        self.ax.plot(robot_trail[:, 0], robot_trail[:, 1], color=colors['robot_trail'], lw=1.5, alpha=0.7)
        
        self._draw_sensors(robot)

        # 로봇 그리기
        robot_circle = patches.Circle(robot.pos, CONFIG['robot_radius'], facecolor=colors['robot'])
        self.ax.add_patch(robot_circle)
        self.ax.arrow(robot.pos[0], robot.pos[1], 
                      CONFIG['robot_radius'] * 1.5 * np.cos(robot.theta), 
                      CONFIG['robot_radius'] * 1.5 * np.sin(robot.theta), 
                      head_width=0.4, 
                      color=colors['robot_arrow'])

        # 플롯 설정
        self.ax.set_title(f"State: {local_planner.state}")
        self.ax.set_xlim(-0.5, self.map.width - 0.5)
        self.ax.set_ylim(-0.5, self.map.height - 0.5)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.legend(fontsize='small', loc='upper right')
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _draw_sensors(self, robot):
        colors = CONFIG['colors']
        for i, dist_cells in enumerate(robot.sensors.readings):
            spec = CONFIG['sensors']['specs'][i]
            angle_offset, x_offset_cells, y_offset_cells = spec
            
            rotated_offset_x = x_offset_cells * np.cos(robot.theta) - y_offset_cells * np.sin(robot.theta)
            rotated_offset_y = x_offset_cells * np.sin(robot.theta) + y_offset_cells * np.cos(robot.theta)

            start_x, start_y = robot.pos[0] + rotated_offset_x, robot.pos[1] + rotated_offset_y
            end_x = start_x + dist_cells * np.cos(robot.theta + angle_offset)
            end_y = start_y + dist_cells * np.sin(robot.theta + angle_offset)
            
            color = colors['sensor_hit'] if dist_cells < CONFIG['sensors']['range'] else colors['sensor_ray']
            self.ax.plot([start_x, end_x], [start_y, end_y], color=color, alpha=0.8, lw=1)