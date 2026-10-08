#
# NjordPlatform.cmake
#
# Shared platform-detection macros for the Jetson / miniPC split build.
#
# Usage from a downstream package's CMakeLists.txt:
#
#   find_package(njord_platform REQUIRED)
#   njord_detect_platform()
#
# after which NJORD_* variables described below are available in the
# calling scope.
#
# These macros are intentionally implemented with macro() (not function())
# so that all detected variables are set directly in the caller's scope
# without needing PARENT_SCOPE plumbing.

include(CheckLanguage)

# _njord_bool_to_yaml(<in_var> <out_var>)
#
# Internal helper: converts a CMake ON/OFF (or TRUE/FALSE) value into the
# lowercase "true"/"false" strings expected by YAML.
macro(_njord_bool_to_yaml var_in var_out)
  if(${var_in})
    set(${var_out} "true")
  else()
    set(${var_out} "false")
  endif()
endmacro()

# njord_detect_platform()
#
# Detects the ROS distro, CUDA/TensorRT/ZED SDK/glim/livox availability, and
# picks an overall NJORD_PLATFORM_PROFILE ("jetson" or "minipc").
macro(njord_detect_platform)
  # ---------------------------------------------------------------------
  # ROS distro
  # ---------------------------------------------------------------------
  if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not set. Please build inside a ROS 2 environment.")
  endif()

  string(TOLOWER "$ENV{ROS_DISTRO}" NJORD_ROS_DISTRO)

  set(NJORD_ROS_DISTRO_IS_FOXY OFF)
  set(NJORD_ROS_DISTRO_IS_HUMBLE OFF)
  set(NJORD_ROS_DISTRO_IS_JAZZY OFF)
  if(NJORD_ROS_DISTRO STREQUAL "foxy")
    set(NJORD_ROS_DISTRO_IS_FOXY ON)
  elseif(NJORD_ROS_DISTRO STREQUAL "humble")
    set(NJORD_ROS_DISTRO_IS_HUMBLE ON)
  elseif(NJORD_ROS_DISTRO STREQUAL "jazzy")
    set(NJORD_ROS_DISTRO_IS_JAZZY ON)
  endif()

  # ---------------------------------------------------------------------
  # CUDA
  # ---------------------------------------------------------------------
  find_package(CUDAToolkit QUIET)
  check_language(CUDA)

  # check_language(CUDA) relies on nvcc being on PATH (or CUDACXX being
  # set). On Jetson/JetPack, /usr/local/cuda/bin is not on PATH by default,
  # so check_language(CUDA) can fail to set CMAKE_CUDA_COMPILER even though
  # find_package(CUDAToolkit) already located the same nvcc via the CUDA
  # Toolkit's own search logic (CUDAToolkit_NVCC_EXECUTABLE). Fall back to
  # that path so NJORD_HAS_CUDA reflects reality instead of PATH quirks.
  # This does not change behavior on a real miniPC: without CUDA installed,
  # CUDAToolkit_FOUND is false and CUDAToolkit_NVCC_EXECUTABLE is unset, so
  # NJORD_HAS_CUDA stays OFF and the stub fallback is preserved.
  if(NOT CMAKE_CUDA_COMPILER AND CUDAToolkit_FOUND AND CUDAToolkit_NVCC_EXECUTABLE)
    set(CMAKE_CUDA_COMPILER "${CUDAToolkit_NVCC_EXECUTABLE}")
    message(STATUS
      "njord_platform: check_language(CUDA) did not find nvcc on PATH; using "
      "CUDAToolkit_NVCC_EXECUTABLE=${CUDAToolkit_NVCC_EXECUTABLE}")
  endif()

  set(NJORD_HAS_CUDA OFF)
  if(CUDAToolkit_FOUND AND CMAKE_CUDA_COMPILER)
    set(NJORD_HAS_CUDA ON)
  endif()

  set(NJORD_CUDA_VERSION "")
  set(NJORD_CUDA_VARIANT "none")
  if(CUDAToolkit_FOUND)
    set(NJORD_CUDA_VERSION "${CUDAToolkit_VERSION}")
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "13.0")
      set(NJORD_CUDA_VARIANT "cuda13")
    elseif(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.0")
      set(NJORD_CUDA_VARIANT "cuda12")
    else()
      set(NJORD_CUDA_VARIANT "legacy")
    endif()
  endif()

  if(NJORD_HAS_CUDA)
    message(STATUS "njord_platform: CUDA found (version=${NJORD_CUDA_VERSION}, variant=${NJORD_CUDA_VARIANT})")
  else()
    if(NOT CUDAToolkit_FOUND)
      message(STATUS "njord_platform: CUDA unavailable; CUDAToolkit not found")
    elseif(NOT CMAKE_CUDA_COMPILER)
      message(STATUS "njord_platform: CUDA unavailable; no CUDA compiler (nvcc) found")
    endif()
  endif()

  # ---------------------------------------------------------------------
  # TensorRT
  # ---------------------------------------------------------------------
  find_path(NJORD_TENSORRT_INCLUDE_DIR NvInfer.h)
  find_library(NJORD_TENSORRT_LIBRARY nvinfer)

  set(NJORD_HAS_TENSORRT OFF)
  if(NJORD_TENSORRT_INCLUDE_DIR AND NJORD_TENSORRT_LIBRARY)
    set(NJORD_HAS_TENSORRT ON)
  endif()

  if(NJORD_HAS_TENSORRT)
    message(STATUS "njord_platform: TensorRT found (NvInfer.h=${NJORD_TENSORRT_INCLUDE_DIR}, libnvinfer=${NJORD_TENSORRT_LIBRARY})")
  else()
    if(NOT NJORD_TENSORRT_INCLUDE_DIR)
      message(STATUS "njord_platform: TensorRT unavailable; NvInfer.h not found")
    elseif(NOT NJORD_TENSORRT_LIBRARY)
      message(STATUS "njord_platform: TensorRT unavailable; libnvinfer not found")
    endif()
  endif()

  # ---------------------------------------------------------------------
  # ZED SDK
  #
  # Stereolabs installs a lower-case package configuration file
  # (``/usr/local/zed/zed-config.cmake``), not ``ZEDConfig.cmake``.  Do not
  # try to load it with an incompatible CUDA toolkit: the SDK config calls
  # ``find_package(CUDA <SDK-major> REQUIRED)`` internally and would
  # otherwise make a CPU/stub fallback impossible to configure.  So we only
  # call find_package(zed ...) when the SDK's recorded CUDA major version
  # matches the detected CUDAToolkit major version.
  # ---------------------------------------------------------------------
  set(NJORD_HAS_ZED_SDK OFF)
  set(_njord_zed_config "/usr/local/zed/zed-config.cmake")
  if(DEFINED ZED_DIR AND EXISTS "${ZED_DIR}/zed-config.cmake")
    set(_njord_zed_config "${ZED_DIR}/zed-config.cmake")
  endif()

  set(_njord_zed_cuda_major "")
  if(EXISTS "${_njord_zed_config}")
    file(STRINGS "${_njord_zed_config}" _njord_zed_cuda_version_line
      REGEX "^[ \\t]*SET\\(ZED_CUDA_VERSION[ \\t]+[0-9]+\\)")
    string(REGEX MATCH "[0-9]+" _njord_zed_cuda_major "${_njord_zed_cuda_version_line}")
  endif()

  if(_njord_zed_cuda_major AND CUDAToolkit_FOUND AND
    CUDAToolkit_VERSION_MAJOR EQUAL _njord_zed_cuda_major)
    get_filename_component(ZED_DIR "${_njord_zed_config}" DIRECTORY)
    find_package(zed QUIET CONFIG PATHS "${ZED_DIR}" NO_DEFAULT_PATH)
    if(ZED_FOUND)
      set(NJORD_HAS_ZED_SDK ON)
    endif()
  endif()

  if(NJORD_HAS_ZED_SDK)
    message(STATUS "njord_platform: ZED SDK found (${_njord_zed_config}, CUDA ${_njord_zed_cuda_major}.x)")
  else()
    if(NOT EXISTS "${_njord_zed_config}")
      message(STATUS "njord_platform: ZED SDK unavailable; ${_njord_zed_config} not found")
    elseif(NOT _njord_zed_cuda_major)
      message(STATUS "njord_platform: ZED SDK unavailable; could not parse ZED_CUDA_VERSION from ${_njord_zed_config}")
    elseif(NOT CUDAToolkit_FOUND)
      message(STATUS "njord_platform: ZED SDK unavailable; requires CUDA ${_njord_zed_cuda_major}.x but no CUDA Toolkit found")
    elseif(NOT CUDAToolkit_VERSION_MAJOR EQUAL _njord_zed_cuda_major)
      message(STATUS
        "njord_platform: ZED SDK unavailable; requires CUDA ${_njord_zed_cuda_major}.x "
        "but detected CUDA Toolkit is ${CUDAToolkit_VERSION}")
    else()
      message(STATUS "njord_platform: ZED SDK unavailable; find_package(zed) did not succeed")
    endif()
  endif()

  # ---------------------------------------------------------------------
  # glim_ros / livox_ros_driver2
  # ---------------------------------------------------------------------
  find_package(glim_ros QUIET)
  set(NJORD_HAS_GLIM OFF)
  if(glim_ros_FOUND)
    set(NJORD_HAS_GLIM ON)
  endif()
  if(NJORD_HAS_GLIM)
    message(STATUS "njord_platform: glim_ros found")
  else()
    message(STATUS "njord_platform: glim_ros unavailable")
  endif()

  find_package(livox_ros_driver2 QUIET)
  set(NJORD_HAS_LIVOX OFF)
  if(livox_ros_driver2_FOUND)
    set(NJORD_HAS_LIVOX ON)
  endif()
  if(NJORD_HAS_LIVOX)
    message(STATUS "njord_platform: livox_ros_driver2 found")
  else()
    message(STATUS "njord_platform: livox_ros_driver2 unavailable")
  endif()

  # ---------------------------------------------------------------------
  # Overall platform profile
  #
  # Decision order:
  #   1. cache variable NJORD_PLATFORM_PROFILE, if explicitly set
  #   2. environment variable NJORD_PROFILE, if set
  #   3. jetson if NJORD_HAS_CUDA AND NJORD_HAS_ZED_SDK, else minipc
  # ---------------------------------------------------------------------
  if(DEFINED NJORD_PLATFORM_PROFILE AND NOT NJORD_PLATFORM_PROFILE STREQUAL "")
    message(STATUS "njord_platform: profile forced via NJORD_PLATFORM_PROFILE cache variable")
  elseif(DEFINED ENV{NJORD_PROFILE} AND NOT "$ENV{NJORD_PROFILE}" STREQUAL "")
    set(NJORD_PLATFORM_PROFILE "$ENV{NJORD_PROFILE}")
    message(STATUS "njord_platform: profile taken from NJORD_PROFILE environment variable")
  elseif(NJORD_HAS_CUDA AND NJORD_HAS_ZED_SDK)
    set(NJORD_PLATFORM_PROFILE "jetson")
  else()
    set(NJORD_PLATFORM_PROFILE "minipc")
  endif()

  message(STATUS "njord_platform: ---- detection summary ----")
  message(STATUS "njord_platform: ros_distro=${NJORD_ROS_DISTRO}")
  message(STATUS "njord_platform: profile=${NJORD_PLATFORM_PROFILE}")
  message(STATUS "njord_platform: has_cuda=${NJORD_HAS_CUDA} (variant=${NJORD_CUDA_VARIANT}, version=${NJORD_CUDA_VERSION})")
  message(STATUS "njord_platform: has_tensorrt=${NJORD_HAS_TENSORRT}")
  message(STATUS "njord_platform: has_zed_sdk=${NJORD_HAS_ZED_SDK}")
  message(STATUS "njord_platform: has_glim=${NJORD_HAS_GLIM}")
  message(STATUS "njord_platform: has_livox=${NJORD_HAS_LIVOX}")
  message(STATUS "njord_platform: -----------------------------")
endmacro()

# njord_distro_header(<out_var> <pkg>/<stem>)
#
# Generalization of the um982_driver tf2_geometry_msgs header shim: picks
# ".h" for foxy and ".hpp" for humble/jazzy.
macro(njord_distro_header out_var pkg_stem)
  if(NOT DEFINED NJORD_ROS_DISTRO)
    message(FATAL_ERROR "njord_distro_header() called before njord_detect_platform()")
  endif()

  if(NJORD_ROS_DISTRO STREQUAL "foxy")
    set(${out_var} "${pkg_stem}.h")
  elseif(NJORD_ROS_DISTRO STREQUAL "humble" OR NJORD_ROS_DISTRO STREQUAL "jazzy")
    set(${out_var} "${pkg_stem}.hpp")
  else()
    message(FATAL_ERROR "Unsupported ROS_DISTRO: ${NJORD_ROS_DISTRO}. Expected one of: foxy, humble, jazzy.")
  endif()
endmacro()

# njord_find_header(<out_var> <hint_dirs> <candidate1> [<candidate2> ...])
#
# Probes for the first candidate header (relative include path, e.g.
# "cv_bridge/cv_bridge.hpp") that actually exists under any of the given
# hint directories (typically a package's *_INCLUDE_DIRS) plus the default
# system include search paths, and sets <out_var> to that candidate string
# (suitable for direct use in a generated #include).
#
# This exists because distro-name-based guessing (see njord_distro_header)
# is not always sufficient: some headers were added mid-way through a
# distro's lifetime via patch releases, so the only reliable way to know
# whether a header is available is to actually probe for it.
#
# If none of the candidates are found, aborts with FATAL_ERROR listing the
# candidates and the search paths that were tried.
macro(njord_find_header out_var hint_dirs)
  if(NOT DEFINED NJORD_PLATFORM_PROFILE)
    message(FATAL_ERROR "njord_find_header() called before njord_detect_platform()")
  endif()

  set(_njord_find_header_candidates ${ARGN})
  set(_njord_find_header_found "")
  set(_njord_find_header_found_candidate "")

  # A package's exported <pkg>_INCLUDE_DIRS does not always contain the
  # directory holding its own headers -- cv_bridge 3.2.1 on Humble, for
  # example, exports only its dependencies' include dirs.  So always search
  # the include/ directory of every ament prefix on top of the caller's
  # hints.
  # AMENT_PREFIX_PATH is colon-separated; turn it into a CMake list first.
  set(_njord_find_header_search ${hint_dirs})
  string(REPLACE ":" ";" _njord_find_header_prefixes "$ENV{AMENT_PREFIX_PATH}")
  foreach(_njord_find_header_prefix ${_njord_find_header_prefixes})
    if(_njord_find_header_prefix)
      list(APPEND _njord_find_header_search "${_njord_find_header_prefix}/include")
      # Newer ROS packaging installs headers under include/<pkg>/, older
      # packaging puts them directly under include/.  Cover both.
      file(GLOB _njord_find_header_subdirs
        LIST_DIRECTORIES true "${_njord_find_header_prefix}/include/*")
      foreach(_njord_find_header_subdir ${_njord_find_header_subdirs})
        if(IS_DIRECTORY "${_njord_find_header_subdir}")
          list(APPEND _njord_find_header_search "${_njord_find_header_subdir}")
        endif()
      endforeach()
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _njord_find_header_search)

  foreach(_njord_find_header_candidate ${_njord_find_header_candidates})
    unset(_njord_find_header_result CACHE)
    find_path(_njord_find_header_result
      NAMES "${_njord_find_header_candidate}"
      PATHS ${_njord_find_header_search}
    )
    if(_njord_find_header_result)
      set(_njord_find_header_found "${_njord_find_header_result}")
      set(_njord_find_header_found_candidate "${_njord_find_header_candidate}")
      unset(_njord_find_header_result CACHE)
      break()
    endif()
    unset(_njord_find_header_result CACHE)
  endforeach()

  if(NOT _njord_find_header_found_candidate)
    message(FATAL_ERROR
      "njord_find_header: none of the candidate headers were found: "
      "${_njord_find_header_candidates} (searched: ${_njord_find_header_search})")
  endif()

  set(${out_var} "${_njord_find_header_found_candidate}")
  message(STATUS "njord_platform: njord_find_header found ${_njord_find_header_found_candidate} in ${_njord_find_header_found}")
endmacro()

# njord_write_platform_manifest()
#
# Writes platform.yaml and platform.env from the NJORD_* variables set by
# njord_detect_platform(), and installs them to share/njord_platform/ so
# that launch files (Stage B) can read the build-time detection result at
# runtime via ament_index_python.
macro(njord_write_platform_manifest)
  if(NOT DEFINED NJORD_PLATFORM_PROFILE)
    message(FATAL_ERROR "njord_write_platform_manifest() called before njord_detect_platform()")
  endif()

  _njord_bool_to_yaml(NJORD_HAS_CUDA _njord_yaml_has_cuda)
  _njord_bool_to_yaml(NJORD_HAS_TENSORRT _njord_yaml_has_tensorrt)
  _njord_bool_to_yaml(NJORD_HAS_ZED_SDK _njord_yaml_has_zed_sdk)
  _njord_bool_to_yaml(NJORD_HAS_GLIM _njord_yaml_has_glim)
  _njord_bool_to_yaml(NJORD_HAS_LIVOX _njord_yaml_has_livox)

  set(_njord_manifest_dir "${CMAKE_CURRENT_BINARY_DIR}/njord_platform_manifest")
  file(MAKE_DIRECTORY "${_njord_manifest_dir}")

  set(_njord_yaml_path "${_njord_manifest_dir}/platform.yaml")
  file(WRITE "${_njord_yaml_path}"
"# Generated by njord_platform at configure time. Do not edit by hand.
ros_distro: ${NJORD_ROS_DISTRO}
profile: ${NJORD_PLATFORM_PROFILE}
has_cuda: ${_njord_yaml_has_cuda}
cuda_version: \"${NJORD_CUDA_VERSION}\"
cuda_variant: ${NJORD_CUDA_VARIANT}
has_tensorrt: ${_njord_yaml_has_tensorrt}
has_zed_sdk: ${_njord_yaml_has_zed_sdk}
has_glim: ${_njord_yaml_has_glim}
has_livox: ${_njord_yaml_has_livox}
")

  set(_njord_env_path "${_njord_manifest_dir}/platform.env")
  file(WRITE "${_njord_env_path}"
"# Generated by njord_platform at configure time. Do not edit by hand.
NJORD_ROS_DISTRO=${NJORD_ROS_DISTRO}
NJORD_PROFILE=${NJORD_PLATFORM_PROFILE}
NJORD_HAS_CUDA=${_njord_yaml_has_cuda}
NJORD_CUDA_VERSION=${NJORD_CUDA_VERSION}
NJORD_CUDA_VARIANT=${NJORD_CUDA_VARIANT}
NJORD_HAS_TENSORRT=${_njord_yaml_has_tensorrt}
NJORD_HAS_ZED_SDK=${_njord_yaml_has_zed_sdk}
NJORD_HAS_GLIM=${_njord_yaml_has_glim}
NJORD_HAS_LIVOX=${_njord_yaml_has_livox}
")

  install(FILES
    "${_njord_yaml_path}"
    "${_njord_env_path}"
    DESTINATION share/njord_platform
  )
endmacro()
