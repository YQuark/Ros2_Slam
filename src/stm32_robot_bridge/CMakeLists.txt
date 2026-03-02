cmake_minimum_required(VERSION 3.5)
project(stm32_robot_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  stm32_robot_bridge/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
