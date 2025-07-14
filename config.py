import numpy as np

CONFIG = {
    'grid_width': 20,
    'grid_height': 15,
    'obstacle_count': 5,
    'map_filename': 'map.txt',
    'dt': 0.1,
    'robot_radius': 0.3,
    'robot_max_v': 0.8,
    'robot_max_w': np.deg2rad(80),
    'colors': {
        'wall': 'black',
        'static_obstacle': 'gray',
        'new_obstacle': 'purple',
        'robot': 'blue',
        'robot_arrow': 'white',
        'global_path_color': 'g',
        'robot_trail': 'r',
        'sensor_ray': 'orange',
        'sensor_hit': 'red',
    },
    'sensors': {
        'range': 2.0,
        'specs': [
            (0, 0, 0.3),
            (0, 0, 0),
            (0, 0, -0.3),
            (np.deg2rad(45), 0, 0),
            (np.deg2rad(-45), 0, 0),
            (np.deg2rad(90), 0, 0),
            (np.deg2rad(-90), 0, 0),
        ]
    },
    # [수정] 혼합 제어를 위한 PID 파라미터 재설정
    'pid': {
        'target_distance': 0.7, # 측면 장애물과 유지할 목표 거리
        'Kp': 1.5,              # 비례 게인 (반응 강도)
        'Kd': 0.2,              # 미분 게인을 0으로 설정하여 진동 방지
        'path_weight': 1.2,     # 경로 추종 가중치
        'avoid_weight': 0.9,    # 장애물 회피 가중치
    },
    'detour_trigger_dist': 1.0
}