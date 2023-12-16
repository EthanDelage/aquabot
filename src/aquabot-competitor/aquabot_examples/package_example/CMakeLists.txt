cmake_minimum_required(VERSION 3.8)
project(package_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# C++ executable installation
# Create our executable, add include directories and link dependencies (only for example_node)
add_executable(example_node_cpp src/example_node.cpp)
target_include_directories(example_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_node_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(example_node_cpp rclcpp std_msgs)

install(TARGETS example_node_cpp
  DESTINATION lib/${PROJECT_NAME})

# Python scripts installation
install(PROGRAMS scripts/example_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(
  package_example
  PACKAGE_DIR src/package_example
)
ament_package()
