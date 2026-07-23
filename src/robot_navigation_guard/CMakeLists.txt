cmake_minimum_required(VERSION 3.8)
project(robot_navigation_guard)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_navigation_guard)
install(PROGRAMS robot_navigation_guard/navigation_guard_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigation_guard_node)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_guard test/test_guard.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
