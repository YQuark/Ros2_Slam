cmake_minimum_required(VERSION 3.8)
project(robot_chassis_ops)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_chassis_ops)
install(PROGRAMS robot_chassis_ops/chassis_ops_node.py DESTINATION lib/${PROJECT_NAME}
  RENAME chassis_ops_node)
ament_package()
