#!/usr/bin/env bash
# Reproducible CPU or JetPack 7.2 CUDA 13 environment; leaves system Python alone.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-jetson}"
if [[ $# -gt 0 ]]; then shift; fi
cd "${workspace_dir}"
case "${profile}" in
  jetson)
    /usr/bin/python3 - <<'PY'
import json, platform
from pathlib import Path
release = Path('/etc/nv_tegra_release')
if platform.machine() != 'aarch64' or not release.is_file() or not release.read_text().startswith('# R39'):
    raise SystemExit('jetson profile is validated for ARM64 JetPack 7.2 / L4T R39; use cpu on other hosts')
version = json.loads(Path('/usr/local/cuda/version.json').read_text())['cuda']['version']
if version.split('.')[0] != '13':
    raise SystemExit(f'CUDA 13 is required for this Jetson profile; found {version}')
PY
    device='cuda:0'
    extra_checks=(--triton)
    ;;
  cpu)
    device='cpu'
    extra_checks=()
    ;;
  *) echo 'Usage: bash scripts/setup_python_uv.sh [jetson|cpu] [runtime-check options]' >&2; exit 2 ;;
esac
# ARM torchvision CPU/CUDA wheels share version 0.24.1. Reinstall it when
# selecting a profile so a same-version binary from another index cannot remain.
uv sync --frozen --extra "${profile}" --no-managed-python --link-mode hardlink --reinstall-package torchvision
.venv/bin/python scripts/yolo/normalize_cusparselt_metadata.py
uv pip check --python .venv/bin/python
.venv/bin/python scripts/yolo/check_jetson_runtime.py --device "${device}" \
  "${extra_checks[@]}" --output log/python_runtime_check.json "$@"
echo 'Environment ready. For ROS YOLO: source /opt/ros/jazzy/setup.bash; source install/setup.bash; source export_python_path.sh'
