# simulation.py

import numpy as np
import matplotlib.pyplot as plt
from config import CONFIG
from map import Map
from robot import Robot
from local_planner import LocalPlanner
from visualizer import Visualizer
from global_path_planner_api import IntelligentCoveragePlanner

class Simulation:
    def __init__(self):
        self.map_config = CONFIG['map']
        self.robot_config = CONFIG['robot']
        self.sim_config = CONFIG['simulation']

        self.global_map = Map(self.map_config)
        self.global_map.from_file(self.map_config['filename'], self.map_config['obstacle_count'])

        self.local_map = Map(self.map_config)
        self.local_map.grid = np.copy(self.global_map.grid)
        
        # 이 라인이 _setup_scenario 메서드를 호출합니다.
        self.new_obstacle_info = self._setup_scenario()
        if self.new_obstacle_info:
            x, y, w, h = self.new_obstacle_info
            self.local_map.add_rectangular_obstacle(x, y, w, h)

        self.global_path = IntelligentCoveragePlanner.get_global_path(self.global_map.grid)        
        start_pos = np.array(self.global_path[0], dtype=float)       
        goal_pos = np.array(self.global_path[-1], dtype=float)

        self.robot = Robot(start_pos[0], start_pos[1])
        self.local_planner = LocalPlanner(self.robot, self.global_map, self.local_map, self.global_path)
        self.visualizer = Visualizer(self.local_map)

    # _setup_scenario 메서드가 __init__과 같은 레벨로 클래스 내부에 위치해야 합니다.
    def _setup_scenario(self):
        print("\n--- Scenario: Add a new dynamic obstacle ---")
        try:
            x_str = input(f"Enter X for new obstacle's start (1-{self.map_config['grid_width']-2}) or 'n' for none: ")
            if x_str.lower() == 'n': return None
            
            y_str = input(f"Enter Y for new obstacle's start (1-{self.map_config['grid_height']-2}): ")
            x, y = int(x_str), int(y_str)

            w_str = input(f"Enter obstacle WIDTH: ")
            h_str = input(f"Enter obstacle HEIGHT: ")
            w, h = int(w_str), int(h_str)

            if self.global_map.is_obstacle(x,y) or \
               not (1 <= x < self.map_config['grid_width']-1 and 1 <= y < self.map_config['grid_height']-1) or \
               (x + w >= self.map_config['grid_width']-1 or y + h >= self.map_config['grid_height']-1) or w < 1 or h < 1:
                print("Invalid position or size. Placing obstacle at default (10, 5) with size 3x2.")
                return (10, 5, 3, 2)
            
            print(f"New obstacle will be placed at ({x}, {y}) with size {w}x{h}.")
            return (x, y, w, h)
        except (ValueError, IndexError):
            print("Invalid input. No new obstacle added.")
            return None

    def run(self):
        try:
            for i in range(8000):
                v, w = self.local_planner.plan()
                self.robot.move(v, w, self.sim_config['dt'])
                
                if i % 5 == 0: 
                    self.visualizer.draw(self.robot, self.global_path, self.local_planner, self.new_obstacle_info)

                if self.local_planner.current_global_idx >= len(self.global_path) -1:
                    if np.linalg.norm(self.robot.pos - np.array(self.global_path[-1])) < 1.0:
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