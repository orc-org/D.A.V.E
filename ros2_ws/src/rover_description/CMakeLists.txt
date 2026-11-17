cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

# install resource directories
install(DIRECTORY
  urdf
  launch
  rviz
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# install executable python scripts
install(PROGRAMS
  scripts/sim_drive_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
