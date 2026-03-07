cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/map_to_pointcloud_viz.py
  scripts/fake_base_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
