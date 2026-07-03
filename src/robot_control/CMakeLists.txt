cmake_minimum_required(VERSION 3.5)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robot_control)

install(PROGRAMS
  robot_control/cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 跨包引用：测试文件位于仓库顶层 tests/ 目录，由多个包共享
  ament_add_pytest_test(test_control_policy
    ../../tests/interface/test_control_policy.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_cmd_vel_mux test/test_cmd_vel_mux.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
