# GlobalPlanner.py

import numpy as np
from Config import CONFIG

class GlobalPlanner:
    def __init__(self, global_map):
        self.map = global_map

    def _resample_path(self, path, interval=1.0):
        """
        주어진 경로를 일정한 간격(interval)으로 재샘플링합니다.
        """
        if len(path) < 2:
            return path

        resampled_path = [path[0]]
        dist_covered = 0.0

        for i in range(len(path) - 1):
            p1 = np.array(path[i], dtype=float)
            p2 = np.array(path[i+1], dtype=float)
            segment_vec = p2 - p1
            segment_len = np.linalg.norm(segment_vec)
            if segment_len <= 1e-6: continue
            dir_vec = segment_vec / segment_len
            current_pos_on_segment = 0.0
            if dist_covered > 0:
                current_pos_on_segment = interval - dist_covered
                dist_covered = 0
            while current_pos_on_segment < segment_len:
                new_point = p1 + current_pos_on_segment * dir_vec
                if not resampled_path or not np.allclose(resampled_path[-1], new_point, atol=0.1):
                    resampled_path.append(tuple(new_point))
                current_pos_on_segment += interval
            dist_covered = segment_len - (current_pos_on_segment - interval)
        if not np.allclose(resampled_path[-1], path[-1]):
             resampled_path.append(path[-1])
        return resampled_path

    def generate_path(self, start_pos):
        """
        [최종 수정] 사용자께서 제공해주신 새로운 경로 생성 로직을 적용합니다.
        """
        print("Generating path with the new user-provided logic...")
        
        # --- 1. 제공된 로직의 상수 및 변수 초기화 ---
        y_increment = 30.0
        robot_clearance = 10.0

        x_coords = [0.0, 0.0, 0.0]
        x_coords[0] = 15.0
        x_coords[2] = self.map.width - 15.0 - 1
        x_coords[1] = (x_coords[0] + x_coords[2]) / 2

        current_y = start_pos[1]
        direction = 1
        
        path_skeleton = []
        current_x = 0 # current_x 변수 초기화

        # --- 2. 제공된 로직의 while 루프 구현 ---
        while self.map.height - current_y > robot_clearance:
            if(path_skeleton == []):
                path_skeleton.append((x_coords[0], current_y))
                path_skeleton.append((x_coords[1], current_y))
                path_skeleton.append((x_coords[2], current_y))
                current_x = 2
            else:
                if direction == 1:
                    path_skeleton.append((x_coords[1], current_y))
                    path_skeleton.append((x_coords[2], current_y))
                    current_x = 2
                else:
                    path_skeleton.append((x_coords[1], current_y))
                    path_skeleton.append((x_coords[0], current_y))
                    current_x = 0

            if self.map.height - (current_y+y_increment) > robot_clearance:
                current_y += y_increment
                path_skeleton.append((x_coords[current_x], current_y))
                direction *= -1
            else:
                break
            
        # --- 3. 시작점과 생성된 경로 골격을 연결 ---
        final_path = [start_pos]
        if path_skeleton:
            if not np.allclose(start_pos, path_skeleton[0]):
                final_path.append(path_skeleton[0])
            final_path.extend(path_skeleton[1:])
        
        # --- 4. 최종 경로 리샘플링 ---
        unique_path = [v for i, v in enumerate(final_path) if i == 0 or not np.allclose(v, final_path[i-1])]
        resampled_path = self._resample_path(unique_path, 1.0)
        
        print(f"INFO: Path created. Skeleton: {len(unique_path)} points, Resampled: {len(resampled_path)} points.")
        return resampled_path