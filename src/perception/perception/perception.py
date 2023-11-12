import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge

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
        height = msg.height
        width = msg.width
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv.imshow('test', self.image)
        cv.waitKey(1)
        print(f"Height: {height}\nWidth: {width}")


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