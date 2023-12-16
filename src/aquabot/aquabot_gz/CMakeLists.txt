cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(aquabot_gz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools8 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat13 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(vrx_gz REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================
include_directories(
  ../../vrx/vrx_gz/src
)
link_directories(
  ../../../install/lib
)

# PatrolAndFollow scoring plugin
# include extenal directory
add_library(PatrolAndFollowScoringPlugin SHARED
  src/PatrolAndFollowScoringPlugin.cc
  ../../vrx/vrx_gz/src/WaypointMarkers.cc
)
target_link_libraries(PatrolAndFollowScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  ScoringPlugin
)
install(
  TARGETS PatrolAndFollowScoringPlugin
  DESTINATION lib)

ament_python_install_package(
  aquabot_gz
  PACKAGE_DIR src/aquabot_gz
)

#============================================================================
# Resources
install(DIRECTORY
  launch
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
