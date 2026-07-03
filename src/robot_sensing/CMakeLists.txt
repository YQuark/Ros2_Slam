cmake_minimum_required(VERSION 3.5)
project(robot_sensing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scan_normalizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
