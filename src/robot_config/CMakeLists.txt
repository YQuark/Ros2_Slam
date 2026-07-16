cmake_minimum_required(VERSION 3.8)
project(robot_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robot_config)

install(PROGRAMS robot_config/compile_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compile_config
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
