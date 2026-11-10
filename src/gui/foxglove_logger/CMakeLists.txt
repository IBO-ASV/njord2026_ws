cmake_minimum_required(VERSION 3.8)
project(foxglove_logger)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(njord_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(foxglove_logger_node src/foxglove_logger_node.cpp)
ament_target_dependencies(foxglove_logger_node
  action_msgs geometry_msgs nav_msgs njord_interfaces rclcpp rcl_interfaces sensor_msgs std_msgs)

install(TARGETS foxglove_logger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_foxglove_logger_contract test/test_foxglove_logger_contract.py)
endif()

ament_package()
