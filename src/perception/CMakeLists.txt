cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# OPENCV
# Specify the path to OpenCV using CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ~/.local/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)

add_executable(perception
        src/main.cpp
        src/Perception.cpp
        src/detection.cpp
        src/Lidar.cpp
        src/Camera.cpp)

# Include directories for the target
target_include_directories(perception PRIVATE ~/.local/include/opencv4/)

# Link OpenCV libraries
target_link_libraries(perception ${OpenCV_LIBS})
target_link_libraries(perception ${EIGEN3_LIBRARIES})

ament_target_dependencies(perception
        rclcpp
        sensor_msgs
        ros_gz_interfaces
        Eigen3
        cv_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS perception
        DESTINATION lib/${PROJECT_NAME})

ament_package()