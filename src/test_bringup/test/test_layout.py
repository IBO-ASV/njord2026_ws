from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_expected_test_launches_are_present():
    launch_names = {path.name for path in (PACKAGE_ROOT / "launch").glob("*.launch.py")}
    assert launch_names == {
        "critical_link_loopback.launch.py",
        "nav2_mintest.launch.py",
        "soft_emg_alert_lamp_test.launch.py",
        "um982_ekf_bag_playback.launch.py",
    }


def test_nav2_configs_exist_for_supported_distros():
    for distro in ("humble", "jazzy"):
        assert (PACKAGE_ROOT / "config" / f"nav2_mintest_{distro}.yaml").is_file()


def test_playback_launch_uses_test_bringup_relay():
    source = (PACKAGE_ROOT / "launch" / "um982_ekf_bag_playback.launch.py").read_text()
    assert 'package="test_bringup"' in source
    assert 'FindPackageShare("robot")' in source


def test_network_test_guide_is_linked_from_readme():
    guide = PACKAGE_ROOT / "about_networktest.md"
    assert guide.is_file()
    assert "about_networktest.md" in (PACKAGE_ROOT / "README.md").read_text()


def test_critical_link_route_templates_are_present():
    config_dir = PACKAGE_ROOT / "config" / "critical_link_test_config"
    assert {path.name for path in config_dir.glob("*.yaml")} == {
        "e1_ground.yaml",
        "e1_minipc.yaml",
        "u1_ground.yaml",
        "u1_minipc.yaml",
        "u2_ground.yaml",
        "u2_minipc.yaml",
        "v1_ground.yaml",
        "v1_minipc.yaml",
    }
