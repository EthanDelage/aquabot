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
        cv.imshow('test', self.image)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindow()
            exit(0)
        elif key == ord('o'):
            cv.imwrite('src/perception/resource/positive/{}.jpg'.format(uuid4()), self.image)
        elif key == ord('x'):
            cv.imwrite('src/perception/resource/negative/{}.jpg'.format(uuid4()), self.image)


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