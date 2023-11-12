cmake_minimum_required(VERSION 3.8)
project(navigation)

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

add_executable(navigation
          src/main.cpp
          src/Navigation.cpp
          include/Navigation.hpp
          src/Graph.cpp
          include/Graph.hpp
          src/Pathfinding.cpp
          include/Pathfinding.hpp)

ament_target_dependencies(navigation
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

install(TARGETS navigation
        DESTINATION lib/${PROJECT_NAME})

ament_package()
