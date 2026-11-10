#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this script so the activated environment remains in the current shell:" >&2
  echo "  source ./export_python_path.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Creating virtual environment at ${VENV_DIR}..."
  if command -v uv >/dev/null 2>&1; then
    uv venv --python "$(command -v python3)" "${VENV_DIR}" || return 1
  else
    python3 -m venv --prompt njord2026_ws "${VENV_DIR}" || return 1
  fi
fi

if [[ ! -f "${VENV_DIR}/bin/activate" ]]; then
  echo "Virtual environment is incomplete: ${VENV_DIR}/bin/activate not found." >&2
  return 1
fi

if [[ "${VIRTUAL_ENV:-}" != "${VENV_DIR}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
fi

# Prevent ~/.local packages (notably NumPy 2.x) from overriding the
# ROS/OpenCV-compatible packages selected for this workspace.
export PYTHONNOUSERSITE=1

PYVER=$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
export PYTHONPATH="${VIRTUAL_ENV}/lib/python${PYVER}/site-packages:/usr/local/lib/python${PYVER}/dist-packages:/usr/local/lib/python3/dist-packages:/usr/lib/python${PYVER}/dist-packages:/usr/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
