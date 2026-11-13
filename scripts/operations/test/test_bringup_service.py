"""Render units without modifying systemd or starting vessel hardware."""
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
INSTALLER = ROOT / "scripts/operations/install_bringup_service.sh"


@pytest.mark.parametrize("role,domain", [("agx", 126), ("jetson", 6), ("minipc", 5)])
def test_role_unit_uses_new_runner(tmp_path, role, domain):
    subprocess.run([
        str(INSTALLER), "--role", role, "--ros-distro", "jazzy",
        "--output-dir", str(tmp_path),
    ], check=True, capture_output=True, text=True)
    unit = (tmp_path / f"njord-{role}-bringup.service").read_text()
    assert f"Environment=ROS_DOMAIN_ID={domain}" in unit
    assert f"ExecStart={ROOT}/scripts/operations/run_bringup_service.sh {role} jazzy" in unit
    assert "WantedBy=multi-user.target" in unit
    if role == "agx":
        assert "Environment=NJORD_RMW=cyclonedds" in unit


def test_agx_user_unit_has_no_system_user_directive(tmp_path):
    subprocess.run([
        str(INSTALLER), "--role", "agx", "--ros-distro", "jazzy",
        "--user-service", "--output-dir", str(tmp_path),
    ], check=True, capture_output=True, text=True)
    unit = (tmp_path / "njord-agx-bringup.service").read_text()
    assert "\nUser=" not in unit
    assert "WantedBy=default.target" in unit
    subprocess.run(["systemd-analyze", "--user", "verify",
                    str(tmp_path / "njord-agx-bringup.service")], check=True)


def test_agx_rejects_nano_domain_before_installing(tmp_path):
    result = subprocess.run([
        str(INSTALLER), "--role", "agx", "--domain-id", "6",
        "--output-dir", str(tmp_path),
    ], capture_output=True, text=True)
    assert result.returncode == 64
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("role", ["agx", "jetson", "minipc"])
def test_unsupported_ros_distro_is_rejected_before_install(tmp_path, role):
    result = subprocess.run([
        str(INSTALLER), "--role", role, "--ros-distro", "humble",
        "--output-dir", str(tmp_path),
    ], capture_output=True, text=True)
    assert result.returncode == 64
    assert "requires ROS Jazzy" in result.stderr
    assert not list(tmp_path.iterdir())
