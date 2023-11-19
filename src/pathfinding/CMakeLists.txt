cmake_minimum_required(VERSION 3.8)
project(pathfinding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(pathfinding
        include/Graph.hpp
        include/Pathfinding.hpp
        include/point.hpp
        src/main.cpp
        src/Graph.cpp
        src/Pathfinding.cpp
        src/collision.cpp
        src/djikstra.cpp
        src/graphGeneration.cpp
        src/main.cpp)

ament_target_dependencies(pathfinding
        rclcpp
        sensor_msgs
        ros_gz_interfaces
        std_msgs
        geometry_msgs
        tf2_ros
        tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS pathfinding
        DESTINATION lib/${PROJECT_NAME})

ament_package()
