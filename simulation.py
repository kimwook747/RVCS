import numpy as np
import matplotlib.pyplot as plt
from config import CONFIG
from map import Map
from robot import Robot
from global_planner import GlobalPlanner
from local_planner import LocalPlanner
from visualizer import Visualizer

class Simulation:
    def __init__(self):
        self.global_map = Map(CONFIG['grid_width'], CONFIG['grid_height'])
        self.global_map.from_file(CONFIG['map_filename'], CONFIG['obstacle_count'])

        self.local_map = Map(CONFIG['grid_width'], CONFIG['grid_height'])
        self.local_map.grid = np.copy(self.global_map.grid)
        
        self.new_obstacle_pos = self._setup_scenario()
        if self.new_obstacle_pos:
            self.local_map.add_obstacle(self.new_obstacle_pos[0], self.new_obstacle_pos[1])

        start_pos = (1.5, 3.5)
        goal_pos = (CONFIG['grid_width'] - 1.5, CONFIG['grid_height'] - 1.5)

        self.robot = Robot(start_pos[0], start_pos[1])
        self.global_planner = GlobalPlanner(self.global_map)
        self.global_path = self.global_planner.generate_path(start_pos, goal_pos)
        self.local_planner = LocalPlanner(self.robot, self.global_map, self.local_map, self.global_path)
        self.visualizer = Visualizer(self.local_map)

    def _setup_scenario(self):
        print("\n--- Scenario: Add a new dynamic obstacle ---")
        try:
            x_str = input(f"Enter X for new obstacle (1-{CONFIG['grid_width']-2}) or 'n' for none: ")
            if x_str.lower() == 'n': return None
            y_str = input(f"Enter Y for new obstacle (1-{CONFIG['grid_height']-2}): ")
            x, y = int(x_str), int(y_str)
            if self.global_map.is_obstacle(x,y) or not (1<=x<CONFIG['grid_width']-1 and 1<=y<CONFIG['grid_height']-1):
                print("Invalid position. Placing obstacle at default (10, 5).")
                return (10, 5)
            print(f"New obstacle will be placed at ({x}, {y}).")
            return (x, y)
        except (ValueError, IndexError):
            print("Invalid input. No new obstacle added.")
            return None

    def run(self):
        try:
            for i in range(8000):
                v, w = self.local_planner.plan()
                self.robot.move(v, w, CONFIG['dt'])
                
                if i % 5 == 0: 
                    self.visualizer.draw(self.robot, self.global_path, self.local_planner, self.new_obstacle_pos)

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
            self.visualizer.draw(self.robot, self.global_path, self.local_planner, self.new_obstacle_pos)
            print("To close the window, please close the plot.")
            plt.show()
