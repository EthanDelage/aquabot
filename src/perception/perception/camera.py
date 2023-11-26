from typing import Optional

import numpy as np


class Camera:

    def __init__(self, projection_matrix: np.ndarray,
                 horizontal_fov: float,
                 resolution: tuple[int, int]) -> None:
        self.projection_matrix: np.ndarray = projection_matrix
        self.horizontal_fov: float = horizontal_fov
        self.width: int = resolution[0]
        self.height: int = resolution[1]
        self.lidar_translation_matrix: np.ndarray = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.lidar_roll_matrix: np.ndarray = np.array([
            [1, 0, 0, 0],
            [0, np.cos(0.26), -np.sin(0.26), 0],
            [0, np.sin(0.26), np.cos(0.26), 0],
            [0, 0, 0, 1]
        ])
        self.lidar_pitch_matrix: np.ndarray = np.array([
            [np.cos(np.pi / 2), 0, np.sin(np.pi / 2), 0],
            [0, 1, 0, 0],
            [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2), 0],
            [0, 0, 0, 1]
        ])
        self.lidar_rotation_matrix: np.ndarray = np.dot(self.lidar_roll_matrix,
                                                        self.lidar_pitch_matrix)
        self.lidar_transformation_matrix: np.ndarray = np.dot(
            self.lidar_translation_matrix, self.lidar_rotation_matrix)

    @staticmethod
    def is_inf_point(point: np.ndarray) -> bool:
        return any(np.isinf(point))

    def project_3d_to_2d(self, point_3d: np.ndarray) -> np.ndarray:
        point3d_homogeneous = np.array(
            [point_3d[0], point_3d[1], point_3d[2], 1])
        point2d_homogeneous = np.dot(self.projection_matrix,
                                     point3d_homogeneous)
        return point2d_homogeneous

    def project_lidar_point(self, lidar_point: np.ndarray) -> Optional[
            tuple[int, int]]:
        if self.is_inf_point(lidar_point):
            return None
        point3d_homogeneous = np.array(
            [lidar_point[0], lidar_point[2], lidar_point[1], 1])
        point3d_homogeneous = np.dot(self.lidar_transformation_matrix,
                                     point3d_homogeneous)
        point3d_homogeneous /= point3d_homogeneous[3]
        point2d_homogeneous = self.project_3d_to_2d(
            np.array([point3d_homogeneous[0],
                      point3d_homogeneous[1],
                      point3d_homogeneous[2]]))
        if point2d_homogeneous[2] >= 0:
            return None
        x = point2d_homogeneous[0] / point2d_homogeneous[2]
        y = point2d_homogeneous[1] / point2d_homogeneous[2]
        if not self.is_visible_point2d(np.array([x, y])):
            return None
        return int(x), int(y)

    def is_visible_point2d(self, point2d: np.ndarray) -> bool:
        return 0 <= point2d[0] < self.width and 0 <= point2d[1] < self.height
