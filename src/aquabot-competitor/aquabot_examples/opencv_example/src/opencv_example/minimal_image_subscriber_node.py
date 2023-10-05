#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class MninimalImageSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_image_subscriber')
        self.get_logger().info('Hello world from minimal_image_subscriber !')

        # Create a subscriber on the topic "random_image"
        self.subscriber = self.create_subscription(Image, 'random_image', self.image_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image !')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
    
        # Display image
        cv2.imshow("Image", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    minimal_image_subscriber = MninimalImageSubscriber()

    rclpy.spin(minimal_image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
