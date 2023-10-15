cmake_minimum_required(VERSION 3.8)
project(autoComplete)

include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/sensor_msgs)
include_directories(/opt/ros/humble/include/ros_gz_interfaces)

include_directories(src/navigation/include)

add_executable(autoComplete
		src/navigation/src/Navigation.cpp
		src/navigation/src/main.cpp
		src/navigation/include/Navigation.hpp)