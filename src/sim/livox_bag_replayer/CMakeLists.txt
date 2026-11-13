cmake_minimum_required(VERSION 3.8)
project(livox_bag_replayer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

if(NOT "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(FATAL_ERROR "livox_bag_replayer requires ROS_DISTRO=jazzy")
endif()

add_library(bag_replayer_component SHARED src/bag_replayer_node.cpp)
target_compile_features(bag_replayer_component PUBLIC cxx_std_17)
ament_target_dependencies(
  bag_replayer_component
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)

rclcpp_components_register_node(bag_replayer_component
  PLUGIN "livox_bag_replayer::BagReplayerNode"
  EXECUTABLE bag_replayer_node
)

install(TARGETS bag_replayer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
