cmake_minimum_required(VERSION 3.8)
project(njord_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/NjordPlatform.cmake)

njord_detect_platform()

if(NJORD_HAS_CUDA)
  set(NJORD_HAS_CUDA_DEFINE 1)
else()
  set(NJORD_HAS_CUDA_DEFINE 0)
endif()
if(NJORD_HAS_TENSORRT)
  set(NJORD_HAS_TENSORRT_DEFINE 1)
else()
  set(NJORD_HAS_TENSORRT_DEFINE 0)
endif()
if(NJORD_HAS_ZED_SDK)
  set(NJORD_HAS_ZED_SDK_DEFINE 1)
else()
  set(NJORD_HAS_ZED_SDK_DEFINE 0)
endif()
if(NJORD_HAS_GLIM)
  set(NJORD_HAS_GLIM_DEFINE 1)
else()
  set(NJORD_HAS_GLIM_DEFINE 0)
endif()
if(NJORD_HAS_LIVOX)
  set(NJORD_HAS_LIVOX_DEFINE 1)
else()
  set(NJORD_HAS_LIVOX_DEFINE 0)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/njord_platform/platform_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/njord_platform/platform_config.hpp
  @ONLY
)

install(DIRECTORY include/
  DESTINATION include
  PATTERN "*.in" EXCLUDE
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/njord_platform/platform_config.hpp
  DESTINATION include/njord_platform
)

ament_export_include_directories(include)

njord_write_platform_manifest()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No copyright headers in this package yet (same as the other packages here).
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "cmake/NjordPlatform.cmake")
