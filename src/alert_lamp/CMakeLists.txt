cmake_minimum_required(VERSION 3.8)
project(alert_lamp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(njord_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AlertLampCommand.msg"
  DEPENDENCIES std_msgs
)

add_library(alert_lamp_core
  src/heartbeat_monitor.cpp
  src/status_evaluator.cpp
)
target_compile_features(alert_lamp_core PUBLIC cxx_std_17)
target_include_directories(alert_lamp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(alert_lamp_core rclcpp)

add_library(alert_lamp_nodes
  src/alert_lamp_manager.cpp
  src/alert_lamp_driver.cpp
)
target_compile_features(alert_lamp_nodes PUBLIC cxx_std_17)
target_include_directories(alert_lamp_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(alert_lamp_nodes
  rclcpp std_msgs nav_msgs sensor_msgs diagnostic_msgs diagnostic_updater njord_interfaces
)
target_link_libraries(alert_lamp_nodes alert_lamp_core)
rosidl_target_interfaces(alert_lamp_nodes ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(alert_lamp_manager_node src/alert_lamp_manager_node.cpp)
target_link_libraries(alert_lamp_manager_node alert_lamp_nodes)
ament_target_dependencies(alert_lamp_manager_node rclcpp)

add_executable(alert_lamp_driver_node src/alert_lamp_driver_node.cpp)
target_link_libraries(alert_lamp_driver_node alert_lamp_nodes)
ament_target_dependencies(alert_lamp_driver_node rclcpp)

install(TARGETS alert_lamp_core alert_lamp_nodes alert_lamp_manager_node alert_lamp_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_status_evaluator test/test_status_evaluator.cpp)
  target_link_libraries(test_status_evaluator alert_lamp_core)
  ament_add_gtest(test_heartbeat_monitor test/test_heartbeat_monitor.cpp)
  target_link_libraries(test_heartbeat_monitor alert_lamp_core)
  ament_target_dependencies(test_heartbeat_monitor rclcpp)
  ament_add_gtest(test_alert_lamp_manager test/test_alert_lamp_manager.cpp)
  target_link_libraries(test_alert_lamp_manager alert_lamp_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
