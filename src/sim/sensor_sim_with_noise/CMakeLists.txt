cmake_minimum_required(VERSION 3.8)
project(sensor_sim_with_noise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gnss_noise_simulator src/gnss_noise_simulator.cpp)
target_compile_features(gnss_noise_simulator PRIVATE cxx_std_17)
ament_target_dependencies(gnss_noise_simulator
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
)

add_executable(imu_noise_simulator src/imu_noise_simulator.cpp)
target_compile_features(imu_noise_simulator PRIVATE cxx_std_17)
ament_target_dependencies(imu_noise_simulator
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
)

add_executable(um982_odometry_simulator src/um982_odometry_simulator.cpp)
target_compile_features(um982_odometry_simulator PRIVATE cxx_std_17)
ament_target_dependencies(um982_odometry_simulator
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    gnss_noise_simulator
    imu_noise_simulator
    um982_odometry_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
