#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node_py')

        # Log that the node has succesfully started
        self.get_logger().info('Hello world from example_node_py!')

        # Create a publisher on the topic "status_string" that will publish a std_msgs::msg::String message
        self.publisher_ = self.create_publisher(String, 'status_string', 10)
        
        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Declare variables
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Node name : %s, Loop : %d' % (self.get_name(), self.i)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg) # Publish our String message (topic "status_string")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    example_node = ExampleNode()

    rclpy.spin(example_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
