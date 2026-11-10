#!/usr/bin/env bash
# njord_env.sh
#
# Source this script to detect and export the Jetson/miniPC platform
# environment for this workspace. It does NOT build anything and does NOT
# set up RMW config files (fastdds XML / zenoh json5) -- those are added in
# Stage B. It only decides and exports environment variables, plus toggles
# COLCON_IGNORE on the Jetson-only sensor packages depending on the profile.
#
# Usage:
#   source scripts/njord_env.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this script so the exported environment remains in the current shell:" >&2
  echo "  source scripts/njord_env.sh" >&2
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[njord_env] ERROR: ROS_DISTRO is not set. Source your ROS 2 setup.bash first." >&2
  return 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# ---------------------------------------------------------------------------
# 1. Hardware / SDK detection
# ---------------------------------------------------------------------------
NJORD_ARCH="$(uname -m)"

NJORD_DETECTED_HAS_CUDA=0
if command -v nvcc >/dev/null 2>&1; then
  NJORD_DETECTED_HAS_CUDA=1
elif compgen -G "/usr/local/cuda*" >/dev/null 2>&1; then
  NJORD_DETECTED_HAS_CUDA=1
fi

# ---------------------------------------------------------------------------
# 1b. Make sure nvcc is actually reachable on PATH, and that CMake's
#     find_package(CUDAToolkit)/check_language(CUDA) can locate it too.
#
#     Jetson/JetPack installs CUDA under /usr/local/cuda (a symlink to
#     /usr/local/cuda-<version>), but does not add its bin/ to PATH by
#     default. Without nvcc on PATH, CMake's check_language(CUDA) fails even
#     though find_package(CUDAToolkit) succeeds, which desyncs the shell's
#     detection (NJORD_DETECTED_HAS_CUDA=1, profile=jetson) from CMake's
#     NJORD_HAS_CUDA (OFF). Fix that up here so both agree.
# ---------------------------------------------------------------------------
if ! command -v nvcc >/dev/null 2>&1; then
  NJORD_CUDA_BIN_DIR=""
  if [[ -x "/usr/local/cuda/bin/nvcc" ]]; then
    NJORD_CUDA_BIN_DIR="/usr/local/cuda/bin"
  else
    for _njord_cuda_dir in /usr/local/cuda-*; do
      if [[ -x "${_njord_cuda_dir}/bin/nvcc" ]]; then
        NJORD_CUDA_BIN_DIR="${_njord_cuda_dir}/bin"
        break
      fi
    done
    unset _njord_cuda_dir
  fi

  if [[ -n "${NJORD_CUDA_BIN_DIR}" ]]; then
    case ":${PATH}:" in
      *":${NJORD_CUDA_BIN_DIR}:"*) ;;
      *) export PATH="${NJORD_CUDA_BIN_DIR}:${PATH}" ;;
    esac
    export CUDACXX="${NJORD_CUDA_BIN_DIR}/nvcc"
    NJORD_DETECTED_HAS_CUDA=1
    echo "[njord_env] added ${NJORD_CUDA_BIN_DIR} to PATH and set CUDACXX=${CUDACXX}"
  fi
  unset NJORD_CUDA_BIN_DIR
fi

NJORD_DETECTED_HAS_ZED_SDK=0
if [[ -f "/usr/local/zed/zed-config.cmake" ]]; then
  NJORD_DETECTED_HAS_ZED_SDK=1
fi

# ---------------------------------------------------------------------------
# 2. Decide NJORD_PROFILE (respect a pre-set value, do not overwrite it)
# ---------------------------------------------------------------------------
if [[ -z "${NJORD_PROFILE:-}" ]]; then
  if [[ "${NJORD_DETECTED_HAS_CUDA}" -eq 1 && "${NJORD_DETECTED_HAS_ZED_SDK}" -eq 1 ]]; then
    NJORD_PROFILE="jetson"
  else
    NJORD_PROFILE="minipc"
  fi
fi
export NJORD_PROFILE

# ---------------------------------------------------------------------------
# 4. NJORD_ENABLE_GPU_SENSORS: 1 on jetson, UNSET (not 0) on minipc
# ---------------------------------------------------------------------------
if [[ "${NJORD_PROFILE}" == "jetson" ]]; then
  export NJORD_ENABLE_GPU_SENSORS=1
else
  unset NJORD_ENABLE_GPU_SENSORS
fi

# ---------------------------------------------------------------------------
# 5. NJORD_ROLE (defaults to NJORD_PROFILE if unset)
# ---------------------------------------------------------------------------
if [[ -z "${NJORD_ROLE:-}" ]]; then
  export NJORD_ROLE="${NJORD_PROFILE}"
else
  export NJORD_ROLE
fi

# ---------------------------------------------------------------------------
# 6. Toggle COLCON_IGNORE on packages that are not built on the miniPC.
#    zed2i_driver is intentionally excluded: its CMake auto-falls-back to a
#    CPU/stub build when the ZED SDK is absent, so it must remain buildable
#    (and its launch files present) on both profiles.
# ---------------------------------------------------------------------------
NJORD_LIVOX_SDK2_DIR="${WS_DIR}/src/driver/lidar/Livox-SDK2"
NJORD_LIVOX_ROS_DRIVER2_DIR="${WS_DIR}/src/driver/lidar/livox_ros_driver2"
NJORD_PCL_SEGMENTATION_DIR="${WS_DIR}/src/detection/pcl_segmentation"

if [[ "${NJORD_PROFILE}" == "minipc" ]]; then
  for _njord_dir in \
    "${NJORD_LIVOX_SDK2_DIR}" \
    "${NJORD_LIVOX_ROS_DRIVER2_DIR}" \
    "${NJORD_PCL_SEGMENTATION_DIR}"; do
    if [[ -d "${_njord_dir}" && ! -f "${_njord_dir}/COLCON_IGNORE" ]]; then
      touch "${_njord_dir}/COLCON_IGNORE"
      echo "[njord_env] placed COLCON_IGNORE in ${_njord_dir}"
    fi
  done
else
  for _njord_dir in \
    "${NJORD_LIVOX_SDK2_DIR}" \
    "${NJORD_LIVOX_ROS_DRIVER2_DIR}" \
    "${NJORD_PCL_SEGMENTATION_DIR}"; do
    if [[ -f "${_njord_dir}/COLCON_IGNORE" ]]; then
      rm -f "${_njord_dir}/COLCON_IGNORE"
      echo "[njord_env] removed COLCON_IGNORE from ${_njord_dir}"
    fi
  done
fi
unset _njord_dir

# ---------------------------------------------------------------------------
# 7. RMW selection.
#
#    fastrtps (default): points FASTRTPS_DEFAULT_PROFILES_FILE at
#    config/dds/fastdds_profile.xml. Used for Humble<->Jazzy interop
#    between the two machines.
#
#    zenoh (opt-in): points ZENOH_ROUTER_CONFIG_URI / ZENOH_SESSION_CONFIG_URI
#    at config/zenoh/*.json5. See those files for important caveats
#    (Humble needs a source build of rmw_zenoh_cpp, and both machines must
#    run matching rmw_zenoh_cpp/zenoh-protocol revisions) before relying on
#    this for a real run.
# ---------------------------------------------------------------------------
NJORD_RMW="${NJORD_RMW:-fastrtps}"
NJORD_FASTDDS_PROFILE_FILE="${WS_DIR}/config/dds/fastdds_profile.xml"
NJORD_ZENOH_ROUTER_CONFIG_FILE="${WS_DIR}/config/zenoh/router_config.json5"
NJORD_ZENOH_SESSION_CONFIG_FILE="${WS_DIR}/config/zenoh/session_config.json5"

case "${NJORD_RMW}" in
  fastrtps)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    if [[ -f "${NJORD_FASTDDS_PROFILE_FILE}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${NJORD_FASTDDS_PROFILE_FILE}"
    else
      echo "[njord_env] WARNING: ${NJORD_FASTDDS_PROFILE_FILE} not found; FASTRTPS_DEFAULT_PROFILES_FILE not set" >&2
    fi
    ;;
  zenoh)
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    if [[ -f "${NJORD_ZENOH_ROUTER_CONFIG_FILE}" ]]; then
      export ZENOH_ROUTER_CONFIG_URI="${NJORD_ZENOH_ROUTER_CONFIG_FILE}"
    else
      echo "[njord_env] WARNING: ${NJORD_ZENOH_ROUTER_CONFIG_FILE} not found; ZENOH_ROUTER_CONFIG_URI not set" >&2
    fi
    if [[ -f "${NJORD_ZENOH_SESSION_CONFIG_FILE}" ]]; then
      export ZENOH_SESSION_CONFIG_URI="${NJORD_ZENOH_SESSION_CONFIG_FILE}"
    else
      echo "[njord_env] WARNING: ${NJORD_ZENOH_SESSION_CONFIG_FILE} not found; ZENOH_SESSION_CONFIG_URI not set" >&2
    fi
    echo "[njord_env] NOTE: run exactly one 'ros2 run rmw_zenoh_cpp rmw_zenohd' router on the miniPC before starting any zenoh sessions."
    ;;
  *)
    echo "[njord_env] ERROR: unsupported NJORD_RMW='${NJORD_RMW}'. Expected 'fastrtps' or 'zenoh'." >&2
    return 1
    ;;
esac
export NJORD_RMW

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ---------------------------------------------------------------------------
# 8. Report
# ---------------------------------------------------------------------------
echo "[njord_env] ROS_DISTRO=${ROS_DISTRO}"
echo "[njord_env] arch=${NJORD_ARCH} detected_cuda=${NJORD_DETECTED_HAS_CUDA} detected_zed_sdk=${NJORD_DETECTED_HAS_ZED_SDK}"
echo "[njord_env] NJORD_PROFILE=${NJORD_PROFILE}"
echo "[njord_env] NJORD_ROLE=${NJORD_ROLE}"
echo "[njord_env] NJORD_ENABLE_GPU_SENSORS=${NJORD_ENABLE_GPU_SENSORS:-<unset>}"
echo "[njord_env] NJORD_RMW=${NJORD_RMW} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
# Only report the config vars belonging to the selected RMW. Reporting the
# other implementation's variables is misleading: they may hold stale values
# left over from the ambient shell that have no effect on this session.
if [[ "${NJORD_RMW}" == "fastrtps" ]]; then
  echo "[njord_env] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
else
  echo "[njord_env] ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI:-<unset>}"
  echo "[njord_env] ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI:-<unset>}"
fi
echo "[njord_env] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
