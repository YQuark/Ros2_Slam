cmake_minimum_required(VERSION 3.5)
project(stm32_robot_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(stm32_robot_bridge)

install(PROGRAMS
  stm32_robot_bridge/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge_node
)
install(FILES
  stm32_robot_bridge/protocol_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_protocol_v2 test/test_protocol_v2.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_bridge_node test/test_bridge_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
