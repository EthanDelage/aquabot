cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(aquabot_python)
find_package(ament_cmake REQUIRED)

# Config
#install(DIRECTORY config/
#  DESTINATION share/${PROJECT_NAME}/config)

# Scripts
install(PROGRAMS scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME})
  
install(PROGRAMS scripts/generate_scenario.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/reopen_scenario.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/regenerate_scenario_file.py
  DESTINATION lib/${PROJECT_NAME})

  install(PROGRAMS scripts/plot_log_file.py
    DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(
  aquabot_python
  PACKAGE_DIR src/aquabot_python
)

install(DIRECTORY
  images
  DESTINATION share/${PROJECT_NAME})

ament_package()
