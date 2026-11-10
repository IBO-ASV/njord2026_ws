#!/usr/bin/env bash
# Task 2 Jetson preflight check.
#
# Verifies the Jetson software stack needed by the real-vessel Task 2 launch:
# JetPack/L4T, Ubuntu, ROS 2, CUDA, TensorRT, Python ML stack (system python
# and, optionally, a venv passed as $1), cv_bridge, PCL, livox_ros_driver2,
# disk/RAM, and a short tegrastats sample.
#
# Usage:
#   ./task2_jetson_preflight.sh [/path/to/venv]
#
# Every probe is best-effort (|| true): the script always reaches the summary.
set -euo pipefail

VENV_PATH="${1:-}"

PASS_COUNT=0
WARN_COUNT=0
RESULTS=()

report() {
    # report <PASS|WARN> <label> <detail>
    local status="$1"
    local label="$2"
    local detail="$3"

    if [ "${status}" = "PASS" ]; then
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        WARN_COUNT=$((WARN_COUNT + 1))
    fi

    RESULTS+=("[${status}] ${label}: ${detail}")
    printf '[%s] %-28s %s\n' "${status}" "${label}" "${detail}"
}

echo "=== Task 2 Jetson preflight ($(date)) ==="
echo

# ------------------------------------------------------------------
# JetPack / L4T
# ------------------------------------------------------------------
if [ -f /etc/nv_tegra_release ]; then
    L4T="$(head -n1 /etc/nv_tegra_release 2>/dev/null || true)"
    report PASS "L4T (/etc/nv_tegra_release)" "${L4T}"
else
    report WARN "L4T (/etc/nv_tegra_release)" "file not found (not a Jetson?)"
fi

if command -v jetson_release >/dev/null 2>&1; then
    JR="$(jetson_release -s 2>/dev/null | tr '\n' ' ' || true)"
    report PASS "jetson_release" "${JR:-available}"
else
    report WARN "jetson_release" "not installed (pip install jetson-stats)"
fi

# ------------------------------------------------------------------
# Ubuntu / ROS
# ------------------------------------------------------------------
UBUNTU="$( (lsb_release -ds 2>/dev/null || grep PRETTY_NAME /etc/os-release 2>/dev/null | cut -d= -f2) | tr -d '"' || true)"
if [ -n "${UBUNTU}" ]; then
    report PASS "Ubuntu" "${UBUNTU}"
else
    report WARN "Ubuntu" "could not determine release"
fi

if [ -n "${ROS_DISTRO:-}" ]; then
    report PASS "ROS_DISTRO" "${ROS_DISTRO}"
else
    report WARN "ROS_DISTRO" "not set (source /opt/ros/<distro>/setup.bash)"
fi

# ------------------------------------------------------------------
# CUDA / TensorRT
# ------------------------------------------------------------------
if command -v nvcc >/dev/null 2>&1; then
    CUDA_VER="$(nvcc --version 2>/dev/null | grep -o 'release [0-9.]*' | head -n1 || true)"
    report PASS "CUDA (nvcc)" "${CUDA_VER:-found}"
elif [ -d /usr/local/cuda ]; then
    CUDA_VER="$(cat /usr/local/cuda/version.txt 2>/dev/null || readlink /usr/local/cuda || true)"
    report PASS "CUDA (/usr/local/cuda)" "${CUDA_VER:-present, nvcc not on PATH}"
else
    report WARN "CUDA" "nvcc not found and /usr/local/cuda missing"
fi

TRT_VER="$(dpkg -l 2>/dev/null | awk '/tensorrt|nvinfer/ {print $2"="$3}' | head -n3 | tr '\n' ' ' || true)"
if [ -n "${TRT_VER}" ]; then
    report PASS "TensorRT (dpkg)" "${TRT_VER}"
else
    TRT_PY="$(python3 -c 'import tensorrt; print(tensorrt.__version__)' 2>/dev/null || true)"
    if [ -n "${TRT_PY}" ]; then
        report PASS "TensorRT (python)" "${TRT_PY}"
    else
        report WARN "TensorRT" "not found via dpkg or python3"
    fi
fi

# ------------------------------------------------------------------
# Python ML stack
# ------------------------------------------------------------------
check_python_stack() {
    # check_python_stack <label-prefix> <python-binary>
    local prefix="$1"
    local py="$2"

    if ! command -v "${py}" >/dev/null 2>&1 && [ ! -x "${py}" ]; then
        report WARN "${prefix} python" "${py} not found"
        return 0
    fi

    local pyver
    pyver="$("${py}" --version 2>&1 || true)"
    report PASS "${prefix} python" "${py} (${pyver})"

    local mod
    for mod in torch torchvision ultralytics numpy cv2; do
        local ver
        ver="$("${py}" -c "import ${mod}; print(getattr(${mod}, '__version__', 'unknown'))" 2>/dev/null || true)"
        if [ -n "${ver}" ]; then
            report PASS "${prefix} ${mod}" "${ver}"
        else
            report WARN "${prefix} ${mod}" "import failed"
        fi
    done

    local cuda_ok
    cuda_ok="$("${py}" -c "import torch; print(torch.cuda.is_available())" 2>/dev/null || true)"
    if [ "${cuda_ok}" = "True" ]; then
        report PASS "${prefix} torch.cuda" "available"
    else
        report WARN "${prefix} torch.cuda" "not available (got: '${cuda_ok:-import failed}')"
    fi
}

check_python_stack "system" "python3" || true

if [ -n "${VENV_PATH}" ]; then
    check_python_stack "venv" "${VENV_PATH}/bin/python" || true
fi

# cv_bridge (ROS <-> OpenCV)
if python3 -c "import cv_bridge" >/dev/null 2>&1; then
    report PASS "cv_bridge" "import OK"
else
    report WARN "cv_bridge" "import failed (source ROS + install ros-\${ROS_DISTRO}-cv-bridge)"
fi

# ------------------------------------------------------------------
# PCL / Livox
# ------------------------------------------------------------------
PCL_VER="$(dpkg -l 2>/dev/null | awk '/libpcl-dev|libpcl-common/ {print $2"="$3}' | head -n1 || true)"
if [ -n "${PCL_VER}" ]; then
    report PASS "PCL" "${PCL_VER}"
else
    PCL_CFG="$(pkg-config --modversion pcl_common 2>/dev/null || true)"
    if [ -n "${PCL_CFG}" ]; then
        report PASS "PCL (pkg-config)" "${PCL_CFG}"
    else
        report WARN "PCL" "not found via dpkg or pkg-config"
    fi
fi

LIVOX_PREFIX="$(ros2 pkg prefix livox_ros_driver2 2>/dev/null || true)"
if [ -n "${LIVOX_PREFIX}" ]; then
    report PASS "livox_ros_driver2" "${LIVOX_PREFIX}"
else
    report WARN "livox_ros_driver2" "ros2 pkg prefix failed (workspace not sourced or driver missing)"
fi

# ------------------------------------------------------------------
# Disk / RAM
# ------------------------------------------------------------------
DISK="$(df -h / 2>/dev/null | awk 'NR==2 {print $4" free of "$2" ("$5" used)"}' || true)"
if [ -n "${DISK}" ]; then
    DISK_AVAIL_KB="$(df -k / 2>/dev/null | awk 'NR==2 {print $4}' || echo 0)"
    if [ "${DISK_AVAIL_KB:-0}" -ge 10485760 ]; then  # >= 10 GiB
        report PASS "Disk (/)" "${DISK}"
    else
        report WARN "Disk (/)" "${DISK} - less than 10 GiB free"
    fi
else
    report WARN "Disk (/)" "df failed"
fi

RAM="$(free -h 2>/dev/null | awk '/^Mem:/ {print $7" available of "$2}' || true)"
if [ -n "${RAM}" ]; then
    report PASS "RAM" "${RAM}"
else
    report WARN "RAM" "free failed"
fi

# ------------------------------------------------------------------
# tegrastats sample (nvidia-smi is typically unavailable on Jetson)
# ------------------------------------------------------------------
if command -v tegrastats >/dev/null 2>&1; then
    echo
    echo "--- 3-second tegrastats sample ---"
    TEGRA_OUT="$(timeout 4 tegrastats --interval 1000 2>/dev/null | head -n3 || true)"
    if [ -n "${TEGRA_OUT}" ]; then
        echo "${TEGRA_OUT}"
        report PASS "tegrastats" "3-second sample captured"
    else
        report WARN "tegrastats" "installed but produced no output"
    fi
else
    report WARN "tegrastats" "not found (note: nvidia-smi is typically unavailable on Jetson; tegrastats is the Jetson equivalent)"
fi

# ------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------
echo
echo "=== Summary: ${PASS_COUNT} PASS / ${WARN_COUNT} WARN ==="
for line in "${RESULTS[@]}"; do
    echo "  ${line}"
done

if [ "${WARN_COUNT}" -gt 0 ]; then
    echo
    echo "Preflight finished with ${WARN_COUNT} warning(s). Review before on-water testing."
fi

exit 0
