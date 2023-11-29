cmake_minimum_required(VERSION 3.8)
project(autoComplete)

include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/rcl_interfaces)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/sensor_msgs)
include_directories(/opt/ros/humble/include/ros_gz_interfaces)
include_directories(/opt/ros/humble/include/geometry_msgs)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/cv_bridge)
include_directories(/opt/ros/humble/include/tf2)

include_directories(/usr/include/opencv4)
include_directories(/usr/include/eigen3)

include_directories(src/navigation/include)
include_directories(src/pathfinding/include)
include_directories(src/perception/include)

add_executable(autoComplete
		src/navigation/src/Navigation.cpp
		src/navigation/src/main.cpp
		src/navigation/include/Navigation.hpp
		src/navigation/include/point.hpp
		src/pathfinding/src/Pathfinding.cpp
		src/pathfinding/src/Graph.cpp
		src/pathfinding/include/Graph.hpp
		src/pathfinding/include/Pathfinding.hpp
		src/pathfinding/src/collision.cpp
		src/pathfinding/include/point.hpp
		src/pathfinding/src/graphGeneration.cpp
		src/pathfinding/src/djikstra.cpp
		src/pathfinding/src/callback.cpp
        src/pathfinding/src/buoy.cpp
        src/pathfinding/src/parsing.cpp
        src/pathfinding/src/boat.cpp
    	src/pathfinding/src/buoy.cpp
    	src/pathfinding/src/parsing.cpp
    	src/pathfinding/src/boat.cpp
		src/perception/src/Perception.cpp
		src/perception/src/main.cpp
		src/perception/src/Lidar.cpp
		src/perception/include/Lidar.hpp
		src/perception/include/Camera.hpp
		src/perception/src/Camera.cpp
		src/perception/src/detection.cpp
        src/perception/src/computePosition.cpp)
