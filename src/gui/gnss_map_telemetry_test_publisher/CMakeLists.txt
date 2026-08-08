cmake_minimum_required(VERSION 3.8)
project(gnss_map_telemetry_test_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(gnss_map_telemetry_test_publisher src/gnss_map_telemetry_test_publisher.cpp)
ament_target_dependencies(
  gnss_map_telemetry_test_publisher
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros)

add_executable(norway_waypoint_publisher src/norway_waypoint_publisher.cpp)
ament_target_dependencies(
  norway_waypoint_publisher
  rclcpp
  sensor_msgs)

install(TARGETS
  gnss_map_telemetry_test_publisher
  norway_waypoint_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
