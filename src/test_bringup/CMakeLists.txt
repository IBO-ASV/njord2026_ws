cmake_minimum_required(VERSION 3.8)
project(test_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odometry_frame_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_layout test/test_layout.py)
endif()

ament_package()
