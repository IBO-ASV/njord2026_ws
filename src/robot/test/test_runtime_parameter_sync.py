import importlib.util
from pathlib import Path

import rclpy


MODULE = Path(__file__).parents[1] / "scripts" / "runtime_parameter_sync.py"
SPEC = importlib.util.spec_from_file_location("runtime_parameter_sync", MODULE)
SYNC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SYNC)


def test_role_parameters_accepts_runtime_values_and_rejects_nested_values():
    assert SYNC.role_parameters({"minipc": {"/joy_converter": {"scale.linear_x": 0.3}}}, "minipc") == {
        "/joy_converter": {"scale.linear_x": 0.3}
    }
    try:
        SYNC.role_parameters({"minipc": {"/node": {"bad": {"nested": True}}}}, "minipc")
    except ValueError:
        return
    assert False, "nested parameter mappings must be rejected"


def test_sync_can_start_without_overwriting_node_clients_property():
    rclpy.init()
    try:
        node = SYNC.RuntimeParameterSync()
        assert node.parameter_clients == {}
        node.destroy_node()
    finally:
        rclpy.shutdown()
