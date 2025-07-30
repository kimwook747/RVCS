# config.py

import numpy as np

# --- 1. 기본 해상도 및 물리적 상수 정의 ---
RESOLUTION = 1.0 # 1미터당 그리드 셀의 수

PHYSICAL_CONSTS = {
    'map_width_m': 20.0,
    'map_height_m': 15.0,
    'robot_radius_m': 0.3,
    'robot_max_v_ms': 0.8,       # m/s
    'sensor_range_m': 2.0,
    'pid_target_dist_m': 0.7,
    'detour_trigger_dist_m': 1.0,
    'sensor_center_offset_m': 0.3,
}

# --- 2. 해상도에 따라 시뮬레이션 파라미터 자동 계산 및 구조화 ---
CONFIG = {
    'simulation': {
        'dt': 0.1,
        'max_steps': 8000,
    },
    'map': {
        'resolution': RESOLUTION,
        'grid_width': int(PHYSICAL_CONSTS['map_width_m'] * RESOLUTION),
        'grid_height': int(PHYSICAL_CONSTS['map_height_m'] * RESOLUTION),
        'filename': 'map.txt',
        'obstacle_count': 0,
    },
    'robot': {
        'start_y_grid': 10,
        'goal_grid': (
            int((PHYSICAL_CONSTS['map_width_m'] - 1) * RESOLUTION),
            int((PHYSICAL_CONSTS['map_height_m'] - 1) * RESOLUTION)
        ),
        'radius': PHYSICAL_CONSTS['robot_radius_m'] * RESOLUTION,
        'max_v': PHYSICAL_CONSTS['robot_max_v_ms'] * RESOLUTION,
        'max_w': np.deg2rad(80),
    },
    'sensors': {
        'range': PHYSICAL_CONSTS['sensor_range_m'] * RESOLUTION,
        'specs': [
            (0, 0, PHYSICAL_CONSTS['sensor_center_offset_m'] * RESOLUTION),
            (0, 0, 0),
            (0, 0, -PHYSICAL_CONSTS['sensor_center_offset_m'] * RESOLUTION),
            (np.deg2rad(45), 0, 0),
            (np.deg2rad(-45), 0, 0),
            (np.deg2rad(90), 0, 0),
            (np.deg2rad(-90), 0, 0),
        ]
    },
    'pid': {
        'target_distance': PHYSICAL_CONSTS['pid_target_dist_m'] * RESOLUTION,
        'Kp': 0.4,
        'Kd': 0.1,
        'path_weight': 1.2,
        'avoid_weight': 0.9,
    },
    'detour_trigger_dist': PHYSICAL_CONSTS['detour_trigger_dist_m'] * RESOLUTION,
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
}