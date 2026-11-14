cmake_minimum_required(VERSION 3.8)
project(thruster_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(thruster_allocation SHARED src/allocation.cpp)
target_include_directories(thruster_allocation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(thruster_controller_component SHARED src/node.cpp)
target_include_directories(thruster_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thruster_controller_component
  rclcpp rcl_interfaces rclcpp_components geometry_msgs nav_msgs std_msgs)
target_link_libraries(thruster_controller_component thruster_allocation)

add_executable(thruster_controller_node src/main.cpp)
target_link_libraries(thruster_controller_node thruster_controller_component)
ament_target_dependencies(thruster_controller_node rclcpp)

install(TARGETS
  thruster_controller_component
  thruster_allocation
  thruster_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup thruster_controller_component)
  ament_target_dependencies(test_node_startup rclcpp rcl_interfaces geometry_msgs nav_msgs std_msgs)
  ament_add_gtest(test_velocity_control test/test_velocity_control.cpp)
  target_link_libraries(test_velocity_control thruster_allocation)
  ament_add_gtest(test_allocation test/test_allocation.cpp)
  target_link_libraries(test_allocation thruster_allocation)
endif()

ament_export_include_directories(include)
ament_export_libraries(thruster_allocation)

ament_package()
