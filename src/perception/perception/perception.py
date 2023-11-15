import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from uuid import uuid4

from sensor_msgs.msg import Image


class Perception(Node):

    def __init__(self):
        super().__init__('perception')
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/main_camera_sensor/image_raw',
            self.image_callback,
            10)
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
            cv.imwrite('src/perception/resource/positive/{}.jpg'.format(uuid4()), self.image)
        elif key == ord('x'):
            cv.imwrite('src/perception/resource/negative/{}.jpg'.format(uuid4()), self.image)

    def check_contour_inside(self, contour, inside_contours):
        # Calculate the centroid of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0

        # Check if the centroid is inside any of the inside_contours
        for inside_contour in inside_contours:
            if cv2.pointPolygonTest(inside_contour, (cx, cy), False) >= 0:
                return True
        return False

    def detect_red_boat(self):
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        # rgb_lower_red = np.array([50, 7, 6])
        # rgb_upper_red = np.array([120, 20, 7])
        rgb_lower_red = np.array([65, 6, 5])
        rgb_upper_red = np.array([110, 16, 15])
        rgb_lower_green = np.array([7, 50, 0])
        rgb_upper_green = np.array([28, 81, 10])
        rgb_red_mask = cv2.inRange(rgb_image, rgb_lower_red, rgb_upper_red)
        rgb_green_mask = cv2.inRange(rgb_image, rgb_lower_green, rgb_upper_green)

        # Find contours in the binary mask
        red_contours, _ = cv2.findContours(rgb_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(rgb_green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected red regions
        for contour in red_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 1)
            for green_contour in green_contours:
                if self.check_contour_inside(contour, green_contour):
                    print("Alllyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
                # else:
                #     print("Enemyyyyyyyyyyyyyyyyyyyyyyyyyy")

        for contour in green_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 0, 0), 1)


        # Display the result
        cv2.imshow('Detected Red Objects', self.image)

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