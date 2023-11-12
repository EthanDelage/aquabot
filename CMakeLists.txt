cmake_minimum_required(VERSION 3.8)
project(autoComplete)

include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/sensor_msgs)
include_directories(/opt/ros/humble/include/ros_gz_interfaces)
include_directories(/opt/ros/humble/include/geometry_msgs)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/tf2_geometry_msgs)

include_directories(src/navigation/include)

add_executable(autoComplete
		src/navigation/src/Navigation.cpp
		src/navigation/src/main.cpp
		src/navigation/include/Navigation.hpp src/navigation/src/Pathfinding.cpp src/navigation/src/Graph.cpp src/navigation/include/Graph.hpp src/navigation/include/Pathfinding.hpp src/navigation/src/segmentCollision.cpp)