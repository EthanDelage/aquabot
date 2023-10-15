cmake_minimum_required(VERSION 3.8)
project(autoComplete)

include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/sensor_msgs)
include_directories(/opt/ros/humble/include/ros_gz_interfaces)

include_directories(src/sandbox/include)

add_executable(autoComplete
				src/sandbox/src/main.cpp
				src/sandbox/src/Sandbox.cpp
				src/sandbox/include/sandbox/Sandbox.hpp)