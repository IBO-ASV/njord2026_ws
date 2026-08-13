"""Keep miniPC Python nodes safe to stop with Ctrl-C."""

import ast
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[3]
_NODES = (
    "src/control/control_manager/control_manager/mode_manager_node.py",
    "src/control/control_manager/control_manager/command_arbiter_node.py",
    "src/control/control_manager/control_manager/safety_supervisor_node.py",
    "src/mission/mission_manager/mission_manager/mission_manager_node.py",
    "src/mission/mission_manager/mission_manager/ground_link_return_node.py",
    "src/mission/mission_manager/mission_manager/operator_dispatcher_node.py",
    "src/mission/mission_manager/mission_manager/runtime_manager_node.py",
    "src/navigation/buoy_obstacle_publisher/buoy_obstacle_publisher/"
    "buoy_obstacle_publisher_node.py",
    "src/navigation/buoy_obstacle_publisher/buoy_obstacle_publisher/"
    "cardinal_wall_publisher_node.py",
    "src/navigation/buoy_obstacle_publisher/buoy_obstacle_publisher/"
    "field_boundary_publisher_node.py",
    "src/localization/um982_feedback_filter/um982_feedback_filter/window_feedback_node.py",
)


def _call_name(call):
    if isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name):
        return f"{call.func.value.id}.{call.func.attr}"
    return ""


def test_minipc_rclpy_mains_handle_sigint_and_cleanup():
    for relative_path in _NODES:
        module = ast.parse((_ROOT / relative_path).read_text(encoding="utf-8"))
        main = next(
            node
            for node in module.body
            if isinstance(node, ast.FunctionDef) and node.name == "main"
        )
        guarded_spin = next(node for node in ast.walk(main) if isinstance(node, ast.Try))
        exception_names = {
            element.id
            for handler in guarded_spin.handlers
            if isinstance(handler.type, ast.Tuple)
            for element in handler.type.elts
            if isinstance(element, ast.Name)
        }
        cleanup_calls = {
            _call_name(node)
            for statement in guarded_spin.finalbody
            for node in ast.walk(statement)
            if isinstance(node, ast.Call)
        }
        assert {"KeyboardInterrupt", "ExternalShutdownException"} <= exception_names, relative_path
        assert {"node.destroy_node", "rclpy.try_shutdown"} <= cleanup_calls, relative_path
