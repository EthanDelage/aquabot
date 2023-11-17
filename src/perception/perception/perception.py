import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from uuid import uuid4

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

import numpy as np
import matplotlib.pyplot as plt

import struct


class Perception(Node):

    def __init__(self):
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
        self.bridge = CvBridge()
        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_red_boat()
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindow()
            exit(0)
        elif key == ord('o'):
            cv.imwrite(
                'src/perception/resource/positive/{}.jpg'.format(uuid4()),
                self.image)
        elif key == ord('x'):
            cv.imwrite(
                'src/perception/resource/negative/{}.jpg'.format(uuid4()),
                self.image)

    def camera_callback(self, msg):
        projection_matrix = msg.p
        image_width = msg.width
        fx = projection_matrix[0]

        fov_horizontal = 2 * math.atan(image_width / (2 * fx))
        fov_horizontal_degrees = math.degrees(fov_horizontal)
        # print(fov_horizontal)
        # print(fov_horizontal_degrees)
        # print(projection_matrix)

    def lidar_callback(self, point_cloud_msg):
        if point_cloud_msg.data:
            # Assuming XYZ point cloud data with float32 encoding
            # Get the point cloud data as a bytearray
            data_buffer = bytearray(point_cloud_msg.data)

            # Define variables for the point cloud data
            points = []
            point_size = len(point_cloud_msg.fields)  # Number of fields per point (e.g., x, y, z)

            # Calculate the offset and stride for XYZ (assuming x, y, z are the first 3 fields)
            x_offset = point_cloud_msg.fields[0].offset
            y_offset = point_cloud_msg.fields[1].offset
            z_offset = point_cloud_msg.fields[2].offset
            point_step = point_cloud_msg.point_step

            # Iterate through the data and extract XYZ coordinates
            for i in range(0, len(data_buffer), point_step):
                x = struct.unpack_from('f', data_buffer, i + x_offset)[0]
                y = struct.unpack_from('f', data_buffer, i + y_offset)[0]
                z = struct.unpack_from('f', data_buffer, i + z_offset)[0]
                points.append([x, y, z])

            # Convert the list of points to a NumPy array
            parsed_points = np.array(points, dtype=np.float32)
            print(point_cloud_msg.is_dense)
            print(parsed_points)
            # Affichage des points dans un nuage 3D avec Matplotlib
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_zlim([0, 70])  # Remplacez min_z_value et max_z_value par les valeurs Z minimales et maximales que vous souhaitez afficher
            # Réduire la taille des marqueurs pour afficher des points moins hauts


# Séparation des coordonnées XYZ
            x = parsed_points[:, 0]
            y = parsed_points[:, 1]
            z = parsed_points[:, 2]

            # Affichage des points
            ax.scatter(x, y, z, c=z, cmap='viridis', s=1)  # La couleur est basée sur l'axe Z

            # Réglages d'affichage
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')

            plt.show()
            # return points_array
        return None





    @staticmethod
    def is_contour_inside_rect(contour1, contour2):
        rect1 = cv2.boundingRect(contour1)
        rect2 = cv2.boundingRect(contour2)

        x1, y1, w1, h1 = rect1
        x2, y2, w2, h2 = rect2

        # Check if rect1 (contour1) is inside rect2 (contour2)
        if x2 <= x1 <= x1 + w1 <= x2 + w2 and y2 <= y1 <= y1 + h1 <= y2 + h2:
            return True
        else:
            return False

    def detect_red_boat(self):
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        rgb_lower_red = np.array([65, 6, 5])
        rgb_upper_red = np.array([110, 16, 15])
        rgb_lower_green = np.array([7, 50, 0])
        rgb_upper_green = np.array([28, 81, 10])
        rgb_red_mask = cv2.inRange(rgb_image, rgb_lower_red, rgb_upper_red)
        rgb_green_mask = cv2.inRange(rgb_image, rgb_lower_green,
                                     rgb_upper_green)

        # Find contours in the binary mask
        red_contours, _ = cv2.findContours(rgb_red_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(rgb_green_mask, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected red regions
        for contour in red_contours:
            is_enemy = True
            for green_contour in green_contours:
                if self.is_contour_inside_rect(contour, green_contour):
                    is_enemy = False
                    break
            if is_enemy:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0),
                              1)
        # Display the result
        cv2.imshow('Enemy Tracking', self.image)


def main(args=None):
    print("start")
    rclpy.init(args=args)

    percep = Perception()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
