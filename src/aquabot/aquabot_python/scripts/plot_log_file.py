#!/usr/bin/env python3

import rclpy

from aquabot_python.log_parser import plot_log_csv

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('plot_log_file')
    node.declare_parameter('obstacles', True)
    node.declare_parameter('filepath', '')
    obstacles = node.get_parameter('obstacles').get_parameter_value().bool_value
    filepath = node.get_parameter('filepath').get_parameter_value().string_value
    if len(filepath) <= 0:
        sefilepathed = "log.csv"
    plot_log_csv(filepath, obstacles)

if __name__ == "__main__":
    main()