# Simulation.py

import numpy as np
import matplotlib.pyplot as plt
from Config import CONFIG
from Map import Map
from Robot import Robot
from GlobalPlanner import GlobalPlanner
from LocalPlanner import LocalPlanner
from Visualizer import Visualizer
from global_path_planner_api import IntelligentCoveragePlanner

class Simulation:
    def __init__(self):
        self.global_map = Map(CONFIG['grid_width'], CONFIG['grid_height'])
        self.global_map.from_file(CONFIG['map_filename'], CONFIG['obstacle_count'])

        self.local_map = Map(CONFIG['grid_width'], CONFIG['grid_height'])
        self.local_map.grid = np.copy(self.global_map.grid)
        self.local_map.height, self.local_map.width = self.local_map.grid.shape
        
        self.new_obstacle_info = self._setup_scenario()
        if self.new_obstacle_info:
            x, y, w, h = self.new_obstacle_info
            self.local_map.add_rectangular_obstacle(x, y, w, h)
        
        start_pos = (15.0, 40.0)

        self.robot = Robot(start_pos[0], start_pos[1])
        self.global_planner = GlobalPlanner(self.global_map)
        self.global_path = self.global_planner.generate_path(start_pos)
        # self.global_path = IntelligentCoveragePlanner.get_global_path(self.global_map.grid)
        self.local_planner = LocalPlanner(self.robot, self.global_map, self.local_map, self.global_path)
        self.visualizer = Visualizer(self.local_map)

    def _setup_scenario(self):
        print("\n--- Scenario: Add a new dynamic obstacle ---")
        try:
            x_str = input(f"Enter X for new obstacle's start (1-{CONFIG['grid_width']-2}) or 'n' for none: ")
            if x_str.lower() == 'n': return None
            
            y_str = input(f"Enter Y for new obstacle's start (1-{CONFIG['grid_height']-2}): ")
            x, y = int(x_str), int(y_str)

            w_str = input(f"Enter obstacle WIDTH: ")
            h_str = input(f"Enter obstacle HEIGHT: ")
            w, h = int(w_str), int(h_str)

            if self.global_map.is_obstacle(x,y) or \
               not (1 <= x < CONFIG['grid_width']-1 and 1 <= y < CONFIG['grid_height']-1) or \
               (x + w >= CONFIG['grid_width']-1 or y + h >= CONFIG['grid_height']-1) or w < 1 or h < 1:
                print("Invalid position or size. Placing obstacle at default (100, 50) with size 30x20.")
                return (100, 50, 30, 20)
            
            print(f"New obstacle will be placed at ({x}, {y}) with size {w}x{h}.")
            return (x, y, w, h)
        except (ValueError, IndexError):
            print("Invalid input. No new obstacle added.")
            return None

    def run(self):
        try:
            for i in range(8000):
                v, w = self.local_planner.plan()
                self.robot.move(v, w, CONFIG['dt'])
                
                if i % 5 == 0: 
                    self.visualizer.draw(self.robot, self.global_path, self.local_planner, self.new_obstacle_info)

                if self.local_planner.current_global_idx >= len(self.global_path) -1:
                    if np.linalg.norm(self.robot.pos - np.array(self.global_path[-1])) < 1.0 * CONFIG.get('resolution', 1):
                        print("Simulation Complete: Goal Reached!")
                        break
            else:
                 print("Simulation Timed Out.")
        except KeyboardInterrupt: 
            print("Simulation stopped by user.")
        finally:
            plt.ioff()
            self.visualizer.draw(self.robot, self.global_path, self.local_planner, self.new_obstacle_info)
            print("To close the window, please close the plot.")
            plt.show()