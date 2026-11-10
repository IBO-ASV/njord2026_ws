cmake_minimum_required(VERSION 3.8)
project(critical_link)

if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(njord_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(critical_link_protocol
  src/io.cpp
  src/joy_codec.cpp
  src/operator_codec.cpp
  src/protocol.cpp
)
target_compile_features(critical_link_protocol PUBLIC cxx_std_17)
target_include_directories(critical_link_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(critical_link_protocol njord_interfaces sensor_msgs)
target_link_libraries(critical_link_protocol OpenSSL::Crypto)

add_executable(critical_link_sender src/sender_node.cpp)
target_compile_features(critical_link_sender PUBLIC cxx_std_17)
target_link_libraries(critical_link_sender critical_link_protocol)
ament_target_dependencies(critical_link_sender diagnostic_msgs njord_interfaces rclcpp sensor_msgs std_msgs)

add_executable(critical_link_receiver src/receiver_node.cpp)
target_compile_features(critical_link_receiver PUBLIC cxx_std_17)
target_link_libraries(critical_link_receiver critical_link_protocol)
ament_target_dependencies(critical_link_receiver diagnostic_msgs njord_interfaces rclcpp sensor_msgs std_msgs)

install(TARGETS
  critical_link_protocol
  critical_link_sender
  critical_link_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config firmware launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol critical_link_protocol)
  ament_target_dependencies(test_protocol sensor_msgs)
  ament_add_gtest(test_io test/test_io.cpp)
  target_link_libraries(test_io critical_link_protocol)
endif()

ament_package()
