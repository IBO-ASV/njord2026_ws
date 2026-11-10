#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"

# shellcheck disable=SC1091
source "${WS_DIR}/export_python_path.sh"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_DIR}/install/setup.bash"
fi

python3 - <<'PY'
import sys
import numpy as np

try:
    import cv_bridge  # noqa: F401
except Exception as exc:
    print(f"[ERROR] cv_bridge import failed: {exc}", file=sys.stderr)
    raise SystemExit(2)

if int(np.__version__.split('.')[0]) >= 2:
    print(f"[ERROR] numpy={np.__version__} is unsupported for ROS cv_bridge", file=sys.stderr)
    raise SystemExit(3)

print(f"[INFO] Python={sys.version.split()[0]} numpy={np.__version__}")
PY

exec ros2 launch yolo yolo.launch.py "$@"
