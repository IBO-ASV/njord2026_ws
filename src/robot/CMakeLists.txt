cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(localization_input_guard src/localization_input_guard.cpp)
ament_target_dependencies(localization_input_guard
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS localization_input_guard
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/runtime_parameter_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(pass_through_goal_checker SHARED
  src/pass_through_goal_checker.cpp
)
set_target_properties(pass_through_goal_checker PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION OFF
)
ament_target_dependencies(pass_through_goal_checker
  geometry_msgs nav2_core nav2_costmap_2d pluginlib rclcpp rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugin.xml)
install(TARGETS pass_through_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES goal_checker_plugin.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch config runtime_params urdf
  DESTINATION share/${PROJECT_NAME}
)

# Zenoh bridge configuration is shared by the three physical-machine
# bringups.  Keep the source-of-truth at the workspace-level config path,
# while installing it with robot so launch files work from an installed tree.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh
  DESTINATION share/${PROJECT_NAME}/config
)

# GLIM only accepts a single config directory containing a "config.json"
# that it reads at startup, so the CPU-backend variant needs its own
# directory (glim_config_cpu) rather than a config-file switch. Install a
# second copy of glim_config/ with config.json / config_ros.json swapped
# for their CPU-backend + headless-viewer counterparts.
install(DIRECTORY config/glim_config/
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  PATTERN "config.json" EXCLUDE
  PATTERN "config_ros.json" EXCLUDE
)

install(FILES config/glim_config/config_cpu.json
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  RENAME config.json
)

install(FILES config/glim_config/config_ros_cpu.json
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  RENAME config_ros.json
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_livox_config test/test_livox_config.py)
  ament_add_pytest_test(test_launch_roles test/test_launch_roles.py)
  ament_add_pytest_test(test_platform_manifest test/test_platform_manifest.py)
  ament_add_pytest_test(test_runtime_parameter_sync test/test_runtime_parameter_sync.py)
endif()

ament_package()
