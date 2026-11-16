"""Static contract checks for the thruster-ID-check startup wrapper."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPOSITORY_ROOT / "scripts/operations/run_thruster_id_check.sh"


def test_thruster_id_check_wrapper_matches_jetson_bringup_network_defaults():
    contents = SCRIPT.read_text()

    assert contents.startswith("#!/usr/bin/env bash\n")
    assert "set -euo pipefail" in contents
    assert 'export NJORD_PROFILE="${NJORD_PROFILE:-jetson}"' in contents
    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"' in contents
    assert 'set +u\n# shellcheck disable=SC1091\nsource "${ROS_SETUP}"' in contents
    assert 'source "${SCRIPT_DIR}/../setup/njord_env.sh"' in contents
    assert 'exec ros2 launch thruster_tools thruster_id_test.launch.py "$@"' in contents
