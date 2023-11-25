from typing import Optional, Tuple
import timeit
import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

from pprint import pprint
from .camera import Camera
from .lidar import Lidar

BoundingBox = Tuple[int, int, int, int, float]


class Perception(Node):

    def __init__(self) -> None:
        super().__init__('perception')
        self.image_subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/main_camera_sensor/image_raw',
            self.image_callback,
            10)
        self.camera_subscription = self.create_subscription(
            CameraInfo,
            '/wamv/sensors/cameras/main_camera_sensor/camera_info',
            self.camera_callback,
            10
        )
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            10
        )
        self.bridge: CvBridge = CvBridge()
        self.image: Optional[np.ndarray] = None
        self.enemy_bearing: Optional[float] = None
        self.lidar: Lidar = Lidar()

        self.camera: Optional[Camera] = None
        self.display_plt_graph: bool = False
        self.fig = None
        self.ax = None
        self.scatter_plot = None

        self.rgb_lower_red: np.ndarray = np.array([65, 6, 5])
        self.rgb_upper_red: np.ndarray = np.array([110, 16, 15])
        self.rgb_lower_green: np.ndarray = np.array([7, 50, 0])
        self.rgb_upper_green: np.ndarray = np.array([28, 81, 10])

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_red_boat()

    def camera_callback(self, msg):
        if self.camera is None:
            projection_matrix = msg.p.reshape(3, 4)
            resolution = (msg.width, msg.height)
            horizontal_fov = 1.3962634
            self.camera = Camera(projection_matrix, horizontal_fov, resolution)

    def lidar_callback(self, point_cloud_msg):
        # start = time.time()
        timer = timeit.Timer(lambda: self.lidar.parse_points(point_cloud_msg))
        time_exec = timer.timeit(number=1)
        pprint(time_exec * 1000)
        self.lidar.parse_points(point_cloud_msg)
        if self.camera is not None and self.image is not None:
            self.lidar.project_points_to_camera(self.camera, self.image)
            # self.draw_lidar_points_in_image()
            # self.draw_plt_graph()
            self.update_plot()
        # end = time.time()
        # pprint((end - start) * 1000)

    def detect_red_boat(self):
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        rgb_red_mask = cv2.inRange(rgb_image, self.rgb_lower_red,
                                   self.rgb_upper_red)
        rgb_green_mask = cv2.inRange(rgb_image, self.rgb_lower_green,
                                     self.rgb_upper_green)
        red_contours, _ = cv2.findContours(rgb_red_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(rgb_green_mask, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)
        largest_bounding_box: Optional[BoundingBox] = None
        for contour in red_contours:
            if self.is_enemy_contour(contour, green_contours):
                x, y, w, h = cv2.boundingRect(contour)
                size = w * h
                largest_bounding_box = self.max_bounding_box(
                    largest_bounding_box, (x, y, w, h, size))
                cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0),
                              1)
        self.calculate_enemy_bearing(largest_bounding_box)

    @staticmethod
    def is_enemy_contour(red_contour, green_contours):
        for green_contour in green_contours:
            if Perception.is_contour_inside_rect(red_contour, green_contour):
                return False
        return True

    @staticmethod
    def is_contour_inside_rect(contour1, contour2):
        rect1 = cv2.boundingRect(contour1)
        rect2 = cv2.boundingRect(contour2)
        x1, y1, w1, h1 = rect1
        x2, y2, w2, h2 = rect2
        # Check if rect1 (contour1) is inside rect2 (contour2)
        return x2 <= x1 <= x1 + w1 <= x2 + w2 and y2 <= y1 <= y1 + h1 <= y2 + h2

    @staticmethod
    def max_bounding_box(largest_bounding_box: BoundingBox,
                         current_bounding_box: BoundingBox) -> BoundingBox:
        if largest_bounding_box is None or current_bounding_box[4] > \
                largest_bounding_box[4]:
            return current_bounding_box
        return largest_bounding_box

    def calculate_enemy_bearing(self, largest_bounding_box: BoundingBox) -> \
            Optional[float]:
        if largest_bounding_box is not None and self.camera is not None:
            middle_x = largest_bounding_box[0] + (largest_bounding_box[2] / 2)
            dx = middle_x - (self.camera.width / 2)
            self.enemy_bearing = (dx / self.camera.width) * \
                                 self.camera.horizontal_fov
            return self.enemy_bearing

    def draw_lidar_points_in_image(self) -> None:
        if self.lidar.visible_points is not None:
            for point in self.lidar.visible_points:
                rate = point.get_distance() % 50
                if 0 <= point.get_distance() < 50:
                    rgb = [100 + rate * 3, 0, 0]
                elif 50 <= point.get_distance() < 100:
                    rgb = [0, 100 + rate * 3, 0]
                else:
                    rgb = [0, 0, 100 + rate * 3]
                self.image[
                    point.image_position[1], point.image_position[0]] = rgb
        cv2.imshow('Lidar Tracking', self.image)
        key = cv2.waitKey(1)

    def initialize_plot(self) -> None:
        if not self.display_plt_graph:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('X Label')
            self.ax.set_ylabel('Y Label')
            self.ax.set_zlabel('Z Label')
            self.display_plt_graph = True

    def update_plot(self) -> None:
        self.initialize_plot()
        xs, ys, zs, rs, gs, bs = [], [], [], [], [], []
        for point in self.lidar.visible_points:
            x, y, z = point.position
            color = point.color / 255
            xs.append(x)
            ys.append(y)
            zs.append(z)
            rs.append(color[2])
            gs.append(color[1])
            bs.append(color[0])
        colors = np.column_stack((rs, gs, bs))
        if self.scatter_plot is not None:
            self.scatter_plot.remove()
        self.scatter_plot = self.ax.scatter(xs, ys, zs, c=colors,
                                            cmap='viridis', s=1)
        plt.pause(0.01)

    def draw_plt_graph(self) -> None:
        self.initialize_plot()
        if not self.display_plt_graph:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            points = np.array(
                [point.position for point in self.lidar.visible_points])
            colors = np.array([point.color[::-1] / 255 for point in
                               self.lidar.visible_points])
            ax.scatter(*points.T, c=colors, cmap='viridis', s=1)
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            self.display_plt_graph = True
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    perception = Perception()
    rclpy.spin(perception)
    perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
