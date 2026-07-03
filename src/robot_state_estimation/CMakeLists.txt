cmake_minimum_required(VERSION 3.5)
project(robot_state_estimation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fake_base_odom.py
  scripts/wheel_odom_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
