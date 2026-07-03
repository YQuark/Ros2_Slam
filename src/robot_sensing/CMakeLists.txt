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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scan_normalizer
    ../../tests/interface/test_scan_normalizer.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
