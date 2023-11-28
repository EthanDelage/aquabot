from struct import unpack_from
from typing import Optional

from sensor_msgs.msg import PointCloud2
import numpy as np

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .camera import Camera


class LidarPoint:

    def __init__(self, position: np.ndarray, intensity: float,
                 ring: int) -> None:
        self.position: np.ndarray = position
        self.intensity: float = intensity
        self.ring: int = ring
        self.color: Optional[np.ndarray] = None
        self.image_position: Optional[tuple[int, int]] = None
        self.distance: Optional[float] = None

    def project_to_camera(self, camera: 'Camera', image: np.ndarray) -> bool:
        self.image_position = camera.project_lidar_point(self.position)
        if self.image_position is not None:
            color = image[self.image_position[1], self.image_position[0]]
            self.color = np.array([color[0], color[1], color[2]])
            return True
        return False

    def get_distance(self) -> float:
        if self.distance is None:
            translation_matrix = np.array([-1.6, 0, -2])
            point = self.position + translation_matrix
            self.distance = np.linalg.norm(point)
        return self.distance


class Lidar:

    def __init__(self):
        self.points: list[LidarPoint] = []
        self.visible_points: list[LidarPoint] = []

    def parse_points(self, lidar_message: PointCloud2) -> list[LidarPoint]:
        buffer = bytearray(lidar_message.data)
        x_offset = lidar_message.fields[0].offset
        intensity_offset = lidar_message.fields[3].offset
        ring_offset = lidar_message.fields[4].offset
        point_step = lidar_message.point_step

        self.points = []
        for i in range(0, len(buffer), point_step):
            position = np.frombuffer(buffer, dtype=np.float32, count=3,
                                     offset=i + x_offset)
            if any(np.isinf(position)):
                continue
            intensity = unpack_from('f', buffer,
                                    i + intensity_offset)[0]
            ring = unpack_from('H', buffer,
                               i + ring_offset)[0]
            lidar_point = LidarPoint(position,
                                     intensity,
                                     ring)
            self.points.append(lidar_point)
        return self.points

    def project_points_to_camera(self, camera: 'Camera', image: np.ndarray) -> \
            list[LidarPoint]:
        self.visible_points = [point for point in self.points if
                               point.project_to_camera(camera, image)]
        return self.visible_points