cmake_minimum_required(VERSION 3.8)
project(diagnostic_monitors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(njord_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(topic_heartbeat_monitor_component SHARED
  src/topic_heartbeat_monitor.cpp
)
target_include_directories(topic_heartbeat_monitor_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(topic_heartbeat_monitor_component
  diagnostic_msgs
  diagnostic_updater
  njord_interfaces
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(
  topic_heartbeat_monitor_component
  "njord::diagnostic_monitors::TopicHeartbeatMonitor"
)

add_library(health_state_aggregator_component SHARED
  src/health_state_aggregator.cpp
)
target_include_directories(health_state_aggregator_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(health_state_aggregator_component
  njord_interfaces
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(
  health_state_aggregator_component
  "njord::diagnostic_monitors::HealthStateAggregator"
)

add_executable(health_state_aggregator_node
  src/health_state_aggregator_node.cpp
)
target_link_libraries(health_state_aggregator_node
  health_state_aggregator_component
)
ament_target_dependencies(health_state_aggregator_node
  rclcpp
)

add_library(heartbeat_aggregator_component SHARED
  src/heartbeat_aggregator.cpp
)
target_include_directories(heartbeat_aggregator_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(heartbeat_aggregator_component
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(
  heartbeat_aggregator_component
  "njord::diagnostic_monitors::HeartbeatAggregator"
)

add_executable(heartbeat_aggregator_node
  src/heartbeat_aggregator_node.cpp
)
target_link_libraries(heartbeat_aggregator_node
  heartbeat_aggregator_component
)
ament_target_dependencies(heartbeat_aggregator_node
  rclcpp
)

add_executable(topic_heartbeat_monitor_node
  src/topic_heartbeat_monitor_node.cpp
)

add_executable(autonomy_supervisor_node
  src/autonomy_supervisor.cpp
)
ament_target_dependencies(autonomy_supervisor_node
  action_msgs
  diagnostic_msgs
  diagnostic_updater
  nav2_msgs
  nav_msgs
  njord_interfaces
  rclcpp
  rclcpp_action
  std_msgs
)
target_link_libraries(topic_heartbeat_monitor_node
  topic_heartbeat_monitor_component
)
ament_target_dependencies(topic_heartbeat_monitor_node
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS topic_heartbeat_monitor_component topic_heartbeat_monitor_node
          heartbeat_aggregator_component heartbeat_aggregator_node
          autonomy_supervisor_node health_state_aggregator_component
          health_state_aggregator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/heartbeat_inventory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
