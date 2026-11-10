cmake_minimum_required(VERSION 3.8)
project(zed2i_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(njord_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree segmentation)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig QUIET)
include(CheckLanguage)

find_package(njord_platform REQUIRED)
njord_detect_platform()

# cv_bridge's C++ header was renamed from cv_bridge.h to cv_bridge.hpp partway
# through the Humble lifecycle (via a patch release), so a distro-name-based
# guess is not reliable here. Probe for whichever header actually exists.
njord_find_header(ZED2I_DRIVER_CV_BRIDGE_HEADER "${cv_bridge_INCLUDE_DIRS}"
  cv_bridge/cv_bridge.hpp
  cv_bridge/cv_bridge.h
)
message(STATUS "${PROJECT_NAME}: cv_bridge header=${ZED2I_DRIVER_CV_BRIDGE_HEADER}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cv_bridge_include.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/cv_bridge_include.hpp
  @ONLY
)

option(ZED2I_DRIVER_BUILD_GPU_PERCEPTION
  "Build CUDA/TensorRT buoy perception when all GPU dependencies are available" ON)

set(ZED2I_DRIVER_CUDA_VARIANT "${NJORD_CUDA_VARIANT}")
if(NJORD_HAS_CUDA)
  message(STATUS "${PROJECT_NAME}: detected CUDA Toolkit ${NJORD_CUDA_VERSION} (${ZED2I_DRIVER_CUDA_VARIANT})")
else()
  message(STATUS "${PROJECT_NAME}: CUDA Toolkit not found; SDK node will be disabled unless ZED SDK resolves without CUDA")
endif()

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  njord_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

add_library(${PROJECT_NAME}_common
  src/message_utils.cpp
  src/perception_logic.cpp
  src/lidar_fusion.cpp
  src/detection_message_utils.cpp
)
set_target_properties(${PROJECT_NAME}_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_common SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_common ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_common ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(zed2i_cpu_component SHARED
  src/cpu_stereo_node.cpp
)
target_include_directories(zed2i_cpu_component PRIVATE include)
ament_target_dependencies(zed2i_cpu_component ${DEPENDENCIES} rclcpp_components)
target_link_libraries(zed2i_cpu_component
  ${PROJECT_NAME}_common
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_node(zed2i_cpu_component
  PLUGIN "zed2i_driver::CpuStereoNode"
  EXECUTABLE zed2i_cpu_node
)

option(ZED2I_DRIVER_BUILD_SDK_NODE "Build the ZED SDK backed node when the Stereolabs SDK is available" ON)

# ZED SDK / CUDA / TensorRT availability is detected once by njord_platform's
# njord_detect_platform() (called above), including the "do not load
# zed-config.cmake with an incompatible CUDA toolkit" guard. Here we just
# combine that with this package's own build-time toggles.
set(ZED2I_DRIVER_USE_ZED_SDK OFF)
if(ZED2I_DRIVER_BUILD_SDK_NODE AND NJORD_HAS_ZED_SDK)
  set(ZED2I_DRIVER_USE_ZED_SDK ON)
endif()

set(ZED2I_DRIVER_HAS_CUDA OFF)
if(ZED2I_DRIVER_USE_ZED_SDK AND NJORD_HAS_CUDA)
  enable_language(CUDA)
  set(ZED2I_DRIVER_HAS_CUDA ON)
endif()

set(ZED2I_DRIVER_NVINFER_INCLUDE_DIR "${NJORD_TENSORRT_INCLUDE_DIR}")
set(ZED2I_DRIVER_NVINFER_LIBRARY "${NJORD_TENSORRT_LIBRARY}")

set(ZED2I_DRIVER_HAS_GPU_PERCEPTION OFF)
if(ZED2I_DRIVER_BUILD_GPU_PERCEPTION AND ZED2I_DRIVER_HAS_CUDA AND
   NJORD_HAS_TENSORRT AND ZED2I_DRIVER_USE_ZED_SDK)
  set(ZED2I_DRIVER_HAS_GPU_PERCEPTION ON)
endif()
if(ZED2I_DRIVER_HAS_GPU_PERCEPTION)
  message(STATUS "${PROJECT_NAME}: GPU perception enabled (ZED + CUDA compiler/toolkit + TensorRT)")
else()
  message(STATUS "${PROJECT_NAME}: GPU perception unavailable; requires ZED, CUDA compiler/toolkit, NvInfer.h, and libnvinfer")
endif()

if(ZED2I_DRIVER_USE_ZED_SDK)
  message(STATUS "${PROJECT_NAME}: ZED SDK found; building SDK node")
  if(NJORD_CUDA_VARIANT STREQUAL "cuda13")
    message(STATUS "${PROJECT_NAME}: using CUDA 13.x compatible ZED SDK build path")
  elseif(NJORD_CUDA_VARIANT STREQUAL "cuda12")
    message(STATUS "${PROJECT_NAME}: using CUDA 12.x compatible ZED SDK build path")
  endif()
  add_library(zed2i_sdk_component SHARED
    src/sdk_node_zed.cpp
  )
  target_include_directories(zed2i_sdk_component PRIVATE
    include
    ${ZED_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
  )
  target_compile_definitions(zed2i_sdk_component PRIVATE
    ZED2I_DRIVER_HAS_ZED_SDK=1
    ZED2I_DRIVER_CUDA_VARIANT="${ZED2I_DRIVER_CUDA_VARIANT}"
  )
  ament_target_dependencies(zed2i_sdk_component ${DEPENDENCIES} rclcpp_components)
  target_link_libraries(zed2i_sdk_component
    ${PROJECT_NAME}_common
    ${OpenCV_LIBRARIES}
    ${ZED_LIBRARIES}
  )
  # libnvjpeg is unusable on some Jetson Orin images (nvjpegCreate fails with
  # NVJPEG_STATUS_EXECUTION_FAILED because /dev/nvhost-nvjpg* is not exposed
  # to userspace there -- see the note at the top of ground_video_streamer.cu)
  # so this streamer only needs CUDA (for the GPU resize kernel) and
  # GStreamer's appsrc/nvjpegenc/rtpjpegpay elements to do the actual JPEG
  # encoding and RTP payloading; it does not link libnvjpeg at all.
  set(ZED2I_DRIVER_HAS_GROUND_VIDEO OFF)
  if(ZED2I_DRIVER_HAS_CUDA AND PkgConfig_FOUND)
    pkg_check_modules(GSTREAMER QUIET IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)
    if(GSTREAMER_FOUND)
      set(ZED2I_DRIVER_HAS_GROUND_VIDEO ON)
      target_sources(zed2i_sdk_component PRIVATE src/ground_video_streamer.cu)
      target_compile_definitions(zed2i_sdk_component PRIVATE ZED2I_DRIVER_HAS_GROUND_VIDEO=1)
      target_link_libraries(zed2i_sdk_component CUDA::cudart PkgConfig::GSTREAMER)
    endif()
  endif()
  if(ZED2I_DRIVER_HAS_GROUND_VIDEO)
    message(STATUS "${PROJECT_NAME}: GPU-resize + GStreamer nvjpegenc ground-video streaming enabled")
  else()
    message(STATUS "${PROJECT_NAME}: ground-video streaming unavailable; requires ZED, CUDA, and GStreamer appsrc")
  endif()
  if(ZED2I_DRIVER_HAS_GPU_PERCEPTION)
    target_sources(zed2i_sdk_component PRIVATE
      src/tensor_rt_detector.cpp
      src/cuda_preprocess.cu
      src/cuda_depth_median.cu
    )
    target_compile_definitions(zed2i_sdk_component PRIVATE ZED2I_DRIVER_HAS_GPU_PERCEPTION=1)
    target_include_directories(zed2i_sdk_component PRIVATE ${ZED2I_DRIVER_NVINFER_INCLUDE_DIR})
    target_link_libraries(zed2i_sdk_component ${ZED2I_DRIVER_NVINFER_LIBRARY} CUDA::cudart)
  endif()
else()
  if(NOT ZED2I_DRIVER_BUILD_SDK_NODE)
    message(STATUS "${PROJECT_NAME}: SDK node disabled by ZED2I_DRIVER_BUILD_SDK_NODE=OFF; building SDK-mode stub")
  else()
    message(STATUS "${PROJECT_NAME}: ZED SDK not found; building SDK-mode stub")
  endif()
  add_library(zed2i_sdk_component SHARED
    src/sdk_node_stub.cpp
  )
  target_include_directories(zed2i_sdk_component PRIVATE include)
  ament_target_dependencies(zed2i_sdk_component ${DEPENDENCIES} rclcpp_components)
endif()

# The USB-camera senders are intentionally independent of the ZED SDK/CUDA-only
# ground-video sender above. JPEG uses CPU encoding; H.26x uses VA-API on the miniPC.
if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER_APP QUIET IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)
endif()
if(GSTREAMER_APP_FOUND)
  add_executable(back_cam_ground_video_streamer src/back_cam_ground_video_streamer.cpp)
  add_executable(back_cam_h26x_ground_video_streamer src/back_cam_h26x_ground_video_streamer.cpp)
  target_include_directories(back_cam_ground_video_streamer PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/include
  )
  target_include_directories(back_cam_h26x_ground_video_streamer PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/include
  )
  ament_target_dependencies(back_cam_ground_video_streamer rclcpp sensor_msgs cv_bridge)
  ament_target_dependencies(back_cam_h26x_ground_video_streamer rclcpp sensor_msgs cv_bridge)
  target_link_libraries(back_cam_ground_video_streamer ${OpenCV_LIBRARIES} PkgConfig::GSTREAMER_APP)
  target_link_libraries(back_cam_h26x_ground_video_streamer ${OpenCV_LIBRARIES} PkgConfig::GSTREAMER_APP)
else()
  message(STATUS "${PROJECT_NAME}: back-camera ground-video sender unavailable; requires GStreamer appsrc")
endif()

rclcpp_components_register_node(zed2i_sdk_component
  PLUGIN "zed2i_driver::SdkNode"
  EXECUTABLE zed2i_sdk_node
)

install(TARGETS
  ${PROJECT_NAME}_common
  zed2i_cpu_component
  zed2i_sdk_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TARGET back_cam_ground_video_streamer)
  install(TARGETS back_cam_ground_video_streamer back_cam_h26x_ground_video_streamer
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/ground_receiver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ground_receiver
)

# ground_h26x_receiver keeps its filename when installed; the JPEG receiver
# above retains the historical ground_receiver executable name.
install(PROGRAMS scripts/ground_h26x_receiver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ground_h26x_receiver
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_logic test/test_perception_logic.cpp)
  if(TARGET test_perception_logic)
    target_link_libraries(test_perception_logic ${PROJECT_NAME}_common)
    target_include_directories(test_perception_logic PRIVATE include)
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
