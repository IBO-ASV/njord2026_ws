cmake_minimum_required(VERSION 3.8)
project(um982_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Boost (Asioなどのため)
find_package(Boost REQUIRED COMPONENTS system)

# This driver has no CPU-, CUDA-, or device-specific code.  Probe the actual
# installed tf2_geometry_msgs header instead of using platform detection (which
# can pull in optional GPU SDK configuration while building this CPU-only node).
# This keeps the package buildable on x86_64/amd64 and aarch64/arm64, including
# cross-compilation, independent of the ROS distribution name.
get_target_property(_um982_tf2_geometry_msgs_include_dirs
  tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
set(_um982_tf2_geometry_msgs_header_roots "")
foreach(_um982_include_dir IN LISTS _um982_tf2_geometry_msgs_include_dirs)
  # The exported target normally exposes <prefix>/include/tf2_geometry_msgs,
  # while the public include is addressed from <prefix>/include.
  get_filename_component(_um982_include_parent "${_um982_include_dir}" DIRECTORY)
  list(APPEND _um982_tf2_geometry_msgs_header_roots
    "${_um982_include_dir}" "${_um982_include_parent}")
endforeach()
find_file(UM982_TF2_GEOMETRY_MSGS_HPP_PATH
  NAMES tf2_geometry_msgs/tf2_geometry_msgs.hpp
  PATHS ${_um982_tf2_geometry_msgs_header_roots}
  NO_DEFAULT_PATH)
if(UM982_TF2_GEOMETRY_MSGS_HPP_PATH)
  set(UM982_TF2_GEOMETRY_MSGS_HEADER "tf2_geometry_msgs/tf2_geometry_msgs.hpp")
else()
  find_file(UM982_TF2_GEOMETRY_MSGS_H_PATH
    NAMES tf2_geometry_msgs/tf2_geometry_msgs.h
    PATHS ${_um982_tf2_geometry_msgs_header_roots}
    NO_DEFAULT_PATH)
  if(NOT UM982_TF2_GEOMETRY_MSGS_H_PATH)
    message(FATAL_ERROR
      "um982_driver requires tf2_geometry_msgs/tf2_geometry_msgs.hpp or .h")
  endif()
  set(UM982_TF2_GEOMETRY_MSGS_HEADER "tf2_geometry_msgs/tf2_geometry_msgs.h")
endif()

message(STATUS "um982_driver: tf2_geometry_msgs header=${UM982_TF2_GEOMETRY_MSGS_HEADER}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tf2_geometry_msgs_include.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/um982_driver/tf2_geometry_msgs_include.hpp
  @ONLY
)

include_directories(include)

# 2. ライブラリ（コンポーネント）の作成
add_library(um982_driver_component SHARED
  src/node.cpp
  src/parsing.cpp
  src/utils.cpp
)

target_include_directories(um982_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. 依存関係のリンク (ここを修正)
# ROS関連のパッケージは ament_target_dependencies でリンクします
ament_target_dependencies(um982_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

# ROS以外のライブラリ（Boostなど）は target_link_libraries を使います
target_link_libraries(um982_driver_component
  Boost::system
)

# 4. 実行ファイルの作成
add_executable(um982_driver_node src/main.cpp)
target_include_directories(um982_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 実行ファイルにコンポーネントをリンク
target_link_libraries(um982_driver_node
  um982_driver_component
)

# 5. インストール設定
install(TARGETS um982_driver_component um982_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_um982_utils test/test_utils.cpp)
  target_include_directories(test_um982_utils PRIVATE include)
  target_link_libraries(test_um982_utils um982_driver_component)
endif()

ament_package()
