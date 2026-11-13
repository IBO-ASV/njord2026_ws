#!/usr/bin/env bash
# Install and optionally start the correct persistent base bringup service.
# Run this once on each vehicle computer after building the workspace.
set -euo pipefail

usage() {
  printf '%s\n' \
    'Usage: scripts/operations/install_bringup_service.sh --role <agx|jetson|minipc> [options]' \
    '' \
    'Options:' \
    '  --user USER          Account that owns the ROS processes (default: invoking user)' \
    '  --ros-distro DISTRO  ROS distribution installed under /opt/ros (auto-detected)' \
    '  --domain-id ID       ROS_DOMAIN_ID (126 for AGX, 5 for miniPC, 6 for Jetson)' \
    '  --rmw NAME           fastrtps (default) or zenoh; AGX uses cyclonedds' \
    '  --output-dir DIR    Generate a unit only; do not install, enable or start' \
    '  --user-service      Register with the invoking user systemd manager' \
    '  --no-start           Install and enable, but do not start the service now' \
    '  -h, --help           Show this help' >&2
  exit "${1:-64}"
}

ROLE=""
RUN_AS_USER="${SUDO_USER:-${USER:-}}"
ROS_DISTRO_NAME="${ROS_DISTRO:-}"
ROS_DOMAIN=""
NJORD_RMW_NAME="${NJORD_RMW:-fastrtps}"
START_NOW=1
OUTPUT_DIR=""
USER_SERVICE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --role) ROLE="${2:-}"; shift 2 ;;
    --user) RUN_AS_USER="${2:-}"; shift 2 ;;
    --ros-distro) ROS_DISTRO_NAME="${2:-}"; shift 2 ;;
    --domain-id) ROS_DOMAIN="${2:-}"; shift 2 ;;
    --rmw) NJORD_RMW_NAME="${2:-}"; shift 2 ;;
    --output-dir) OUTPUT_DIR="${2:-}"; shift 2 ;;
    --user-service) USER_SERVICE=1; shift ;;
    --no-start) START_NOW=0; shift ;;
    -h|--help) usage 0 ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

case "${ROLE}" in agx|jetson|minipc) ;; *) echo "--role is required" >&2; usage ;; esac
case "${ROLE}" in
  agx) EXPECTED_ROS_DOMAIN=126; NJORD_RMW_NAME=cyclonedds ;;
  minipc) EXPECTED_ROS_DOMAIN=5 ;;
  jetson) EXPECTED_ROS_DOMAIN=6 ;;
esac
if [[ -z "${ROS_DOMAIN}" ]]; then
  ROS_DOMAIN="${EXPECTED_ROS_DOMAIN}"
elif [[ "${ROS_DOMAIN}" != "${EXPECTED_ROS_DOMAIN}" ]]; then
  echo "--domain-id must be ${EXPECTED_ROS_DOMAIN} for role ${ROLE}" >&2
  exit 64
fi
case "${NJORD_RMW_NAME}" in fastrtps|zenoh|cyclonedds) ;; *) echo "--rmw must be fastrtps, zenoh or cyclonedds" >&2; exit 64 ;; esac
[[ "${ROS_DOMAIN}" =~ ^[0-9]+$ ]] || { echo "--domain-id must be a non-negative integer" >&2; exit 64; }
[[ -n "${RUN_AS_USER}" && "${RUN_AS_USER}" != "root" ]] || { echo "Specify a non-root --user" >&2; exit 64; }
id "${RUN_AS_USER}" >/dev/null

if [[ -z "${ROS_DISTRO_NAME}" ]]; then
  mapfile -t DISTROS < <(find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort)
  if [[ ${#DISTROS[@]} -ne 1 ]]; then
    echo "Could not uniquely detect ROS distro; pass --ros-distro." >&2
    exit 64
  fi
  ROS_DISTRO_NAME="${DISTROS[0]}"
fi
[[ "${ROS_DISTRO_NAME}" == jazzy ]] || { echo "This workspace requires ROS Jazzy" >&2; exit 64; }
[[ -r "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || { echo "ROS distro not installed: ${ROS_DISTRO_NAME}" >&2; exit 1; }

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
TEMPLATE="${SCRIPT_DIR}/systemd/njord-bringup.service.in"
RUNNER="${SCRIPT_DIR}/run_bringup_service.sh"
[[ -r "${WORKSPACE_DIR}/install/setup.bash" ]] || { echo "Build the workspace before installing the service." >&2; exit 1; }
[[ -r "${TEMPLATE}" && -x "${RUNNER}" ]] || { echo "Service template or runner is missing." >&2; exit 1; }

escape_sed() { printf '%s' "$1" | sed 's/[\\&|]/\\&/g'; }
WORKSPACE_ESCAPED="$(escape_sed "${WORKSPACE_DIR}")"
USER_ESCAPED="$(escape_sed "${RUN_AS_USER}")"
ROLE_ESCAPED="$(escape_sed "${ROLE}")"
DISTRO_ESCAPED="$(escape_sed "${ROS_DISTRO_NAME}")"
DOMAIN_ESCAPED="$(escape_sed "${ROS_DOMAIN}")"
RMW_ESCAPED="$(escape_sed "${NJORD_RMW_NAME}")"
UNIT_NAME="njord-${ROLE}-bringup.service"

if [[ "${USER_SERVICE}" == 1 && "${RUN_AS_USER}" != "$(id -un)" ]]; then
  echo "--user-service must run as the service owner" >&2
  exit 64
fi
GENERATED_UNIT=$(mktemp)
trap 'rm -f "$GENERATED_UNIT"' EXIT
sed \
  -e "s|@WORKSPACE@|${WORKSPACE_ESCAPED}|g" \
  -e "s|@USER@|${USER_ESCAPED}|g" \
  -e "s|@ROLE@|${ROLE_ESCAPED}|g" \
  -e "s|@ROS_DISTRO@|${DISTRO_ESCAPED}|g" \
  -e "s|@ROS_DOMAIN_ID@|${DOMAIN_ESCAPED}|g" \
  -e "s|@NJORD_RMW@|${RMW_ESCAPED}|g" \
  "${TEMPLATE}" > "${GENERATED_UNIT}"

if [[ "${USER_SERVICE}" == 1 ]]; then
  sed -i '/^User=/d; s/WantedBy=multi-user.target/WantedBy=default.target/' "${GENERATED_UNIT}"
fi
if [[ -n "${OUTPUT_DIR}" ]]; then
  install -d "${OUTPUT_DIR}"
  install -m 0644 "${GENERATED_UNIT}" "${OUTPUT_DIR}/${UNIT_NAME}"
  echo "Generated ${OUTPUT_DIR}/${UNIT_NAME}"
  exit 0
fi
if [[ "${USER_SERVICE}" == 1 ]]; then
  UNIT_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/systemd/user"
  install -d "${UNIT_DIR}"
  install -m 0644 "${GENERATED_UNIT}" "${UNIT_DIR}/${UNIT_NAME}"
  systemctl --user daemon-reload
  systemctl --user enable "${UNIT_NAME}"
  if [[ "${START_NOW}" == 1 ]]; then systemctl --user restart "${UNIT_NAME}"; fi
  echo "Registered user service ${UNIT_NAME}; starts with the user manager."
  exit 0
fi
sudo install -d -m 0755 /etc/systemd/system
sudo install -m 0644 "${GENERATED_UNIT}" "/etc/systemd/system/${UNIT_NAME}"

sudo systemctl daemon-reload
sudo systemctl enable "${UNIT_NAME}"
if [[ "${START_NOW}" -eq 1 ]]; then
  sudo systemctl restart "${UNIT_NAME}"
fi

echo "Installed ${UNIT_NAME} for user ${RUN_AS_USER}."
echo "Status: sudo systemctl status ${UNIT_NAME}"
echo "Logs:   journalctl -u ${UNIT_NAME} -f"
