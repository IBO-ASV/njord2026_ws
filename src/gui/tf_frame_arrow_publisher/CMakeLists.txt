cmake_minimum_required(VERSION 3.8)
project(tf_frame_arrow_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(arrow_publisher src/arrow_publisher.cpp)
ament_target_dependencies(arrow_publisher rclcpp visualization_msgs)

add_executable(circle_tf_broadcaster src/demo_tf_broad_caster.cpp)
ament_target_dependencies(circle_tf_broadcaster
  rclcpp tf2 tf2_ros geometry_msgs
)

add_executable(cmdveled_tf_broadcaster src/cmdveled_tf_broadcaster.cpp)
ament_target_dependencies(cmdveled_tf_broadcaster
  rclcpp tf2 tf2_ros geometry_msgs nav_msgs
)

add_executable(full_path_publisher src/full_path_publisher.cpp)
ament_target_dependencies(full_path_publisher
  rclcpp visualization_msgs tf2 tf2_ros geometry_msgs
)

add_executable(timed_path_publisher src/timed_path_publisher.cpp)
ament_target_dependencies(timed_path_publisher
  rclcpp visualization_msgs tf2 tf2_ros geometry_msgs
)

add_executable(planned_path_marker_publisher src/planned_path_marker_publisher.cpp)
ament_target_dependencies(planned_path_marker_publisher
  rclcpp visualization_msgs geometry_msgs nav_msgs
)

add_executable(ground_speed_publisher src/ground_speed_publisher.cpp)
ament_target_dependencies(ground_speed_publisher rclcpp nav_msgs std_msgs)

add_executable(cmd_vel_marker_publisher src/cmd_vel_marker_publisher.cpp)
ament_target_dependencies(cmd_vel_marker_publisher rclcpp geometry_msgs visualization_msgs)

install(TARGETS arrow_publisher circle_tf_broadcaster cmdveled_tf_broadcaster full_path_publisher timed_path_publisher planned_path_marker_publisher ground_speed_publisher cmd_vel_marker_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
