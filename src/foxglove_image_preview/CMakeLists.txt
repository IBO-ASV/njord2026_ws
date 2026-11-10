cmake_minimum_required(VERSION 3.8)
project(foxglove_image_preview)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(preview_node src/preview_node.cpp)
target_compile_features(preview_node PUBLIC cxx_std_17)
ament_target_dependencies(preview_node rclcpp sensor_msgs)
target_link_libraries(preview_node ${OpenCV_LIBRARIES})
target_include_directories(preview_node PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS preview_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
