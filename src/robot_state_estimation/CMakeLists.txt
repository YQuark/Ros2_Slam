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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_odom_republisher
    ../../tests/interface/test_wheel_odom_republisher.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
