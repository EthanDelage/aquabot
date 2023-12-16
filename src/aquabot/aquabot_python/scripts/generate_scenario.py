#!/usr/bin/env python3

import rclpy

from aquabot_python.scenario_generator import generate_scenario

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('generate_scenario')
    node.declare_parameter('seed', '')
    node.declare_parameter('name', '')
    node.declare_parameter('obstacles', True)
    seed = node.get_parameter('seed').get_parameter_value().string_value
    name = node.get_parameter('name').get_parameter_value().string_value
    obstacles = node.get_parameter('obstacles').get_parameter_value().bool_value
    if len(seed) <= 0:
        seed = None
    if len(name) <= 0:
        name = 'scenario'
    generate_scenario(name, seed, obstacles)
    node.destroy_node()
    rclpy.shutdown()

def generate_number_scenarios(number):
    for i in range(number):
        generate_scenario("scenario_%d" % i)

if __name__ == "__main__":
    main()