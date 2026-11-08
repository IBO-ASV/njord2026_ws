cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch config urdf runtime_params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS scripts/runtime_parameter_sync.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/agx_rosbag_record_manager.py
  DESTINATION lib/${PROJECT_NAME} RENAME agx_rosbag_record_manager)

ament_package()
