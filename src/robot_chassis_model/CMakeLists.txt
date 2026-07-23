cmake_minimum_required(VERSION 3.8)
project(robot_chassis_model)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_chassis_model)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_layout test/test_wheel_layout.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
