# Config.py

import numpy as np

RESOLUTION = 100

CONFIG = {
    'resolution' : RESOLUTION,
    'grid_width': 4 * RESOLUTION,
    'grid_height': 4 * RESOLUTION,
    'obstacle_count': 0,
    'map_filename': 'map.txt',
    'dt': 0.1,
    'robot_radius': 0.15 * RESOLUTION,
    'robot_max_v': 0.8 * RESOLUTION,
    'robot_max_w': np.deg2rad(80),
    'colors': {
        'wall': 'black',
        'static_obstacle': 'gray',
        'new_obstacle': 'purple',
        'robot': 'blue',
        'robot_arrow': 'white',
        'global_path_color': 'g',
        'global_path_style': '--',
        'detour_path_color': 'm',
        'robot_trail': 'r',
        'sensor_ray': 'orange',
        'sensor_hit': 'red',
    },
    'sensors': {
        'range': 2.0 * RESOLUTION,
        'specs': [
            (0, 0, 0.3 * RESOLUTION),
            (0, 0, 0),
            (0, 0, -0.3 * RESOLUTION),
            (np.deg2rad(45), 0, 0),
            (np.deg2rad(-45), 0, 0),
            (np.deg2rad(90), 0, 0),
            (np.deg2rad(-90), 0, 0),
        ]
    },
    'pid': {
        'target_distance': 0.7 * RESOLUTION,
        'Kp': 1.5,
        'Kd': 0.2,
        'path_weight': 1.2,
        'avoid_weight': 0.9,
    },
    'detour_trigger_dist': 1.0 * RESOLUTION
}