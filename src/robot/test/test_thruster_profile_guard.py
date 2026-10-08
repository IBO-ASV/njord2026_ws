"""Tests for the three-file physical thruster profile startup guard."""

import importlib.util
from pathlib import Path
import unittest


_GUARD_PATH = Path(__file__).parents[1] / "launch" / "thruster_profile_guard.py"
_SPEC = importlib.util.spec_from_file_location("thruster_profile_guard", _GUARD_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
validate_thruster_profile_combination = _MODULE.validate_thruster_profile_combination


class TestThrusterProfileGuard(unittest.TestCase):
    def test_legacy_and_matching_three_wheel_sets_are_accepted(self):
        validate_thruster_profile_combination(
            "true", "config.yaml", "robot.urdf_modified.urdf", "esc4_force.yaml"
        )
        validate_thruster_profile_combination(
            "true", "omni_md10c3.yaml", "omni_3wheel.urdf", "omni_md10c3.yaml"
        )
        validate_thruster_profile_combination(
            "true",
            "omni_md10c3_calibration.yaml",
            "omni_3wheel.urdf",
            "omni_md10c3_calibration.yaml",
        )

    def test_partial_three_wheel_profile_is_rejected(self):
        for selected in (
            ("omni_md10c3.yaml", "robot.urdf_modified.urdf", "esc4_force.yaml"),
            ("config.yaml", "omni_3wheel.urdf", "esc4_force.yaml"),
            ("config.yaml", "robot.urdf_modified.urdf", "omni_md10c3.yaml"),
        ):
            with self.subTest(selected=selected):
                with self.assertRaisesRegex(RuntimeError, "matching normal or calibration"):
                    validate_thruster_profile_combination("true", *selected)

    def test_disabled_thruster_allows_profile_staging(self):
        validate_thruster_profile_combination(
            "false", "omni_md10c3.yaml", "robot.urdf_modified.urdf", "esc4_force.yaml"
        )

    def test_minipc_runs_the_guard_before_actuator_nodes(self):
        minipc = Path(__file__).parents[1] / "launch" / "minipc_bringup.launch.py"
        source = minipc.read_text(encoding="utf-8")
        self.assertIn("OpaqueFunction(function=validate_thruster_profile_action)", source)
        self.assertLess(source.index("thruster_profile_guard,"), source.index("thruster_launch,"))
        self.assertLess(source.index("thruster_profile_guard,"), source.index("thruster_serial,"))


if __name__ == "__main__":
    unittest.main()
