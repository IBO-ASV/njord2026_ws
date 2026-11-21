"""Keep miniPC Python nodes safe to stop with Ctrl-C."""

import ast
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[4]
_NODES = (
    "src/control/motion_supervisor/motion_supervisor/mode_manager_node.py",
    "src/control/motion_supervisor/motion_supervisor/command_arbiter_node.py",
    "src/control/motion_supervisor/motion_supervisor/safety_supervisor_node.py",
    "src/mission/mission_manager/mission_manager/mission_manager_node.py",
    "src/mission/mission_manager/mission_manager/ground_link_return_node.py",
    "src/operator/operator_gateway/operator_gateway/operator_dispatcher_node.py",
    "src/system/runtime_manager/runtime_manager/runtime_manager_node.py",
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


def test_shutdown_paths_execute_cleanup_and_preserve_unexpected_errors():
    import types

    class ExternalShutdownException(Exception):
        pass

    paths = (
        "src/mission/mission_manager/mission_manager/course_editor_node.py",
        "src/bringup/robot/scripts/runtime_parameter_sync.py",
        "src/detection/task2_perception/task2_perception/buoy_selector_node.py",
    )
    for relative_path in paths:
        module = ast.parse((_ROOT / relative_path).read_text(encoding="utf-8"))
        main = next(n for n in module.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        constructor = next(
            n.value.func.id for n in main.body
            if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call)
            and isinstance(n.value.func, ast.Name)
        )
        for error in (KeyboardInterrupt, ExternalShutdownException, ValueError):
            calls = []
            node = types.SimpleNamespace(destroy_node=lambda: calls.append("destroy"))
            def spin(_node):
                raise error()
            rclpy = types.SimpleNamespace(
                init=lambda **kwargs: None, spin=spin,
                try_shutdown=lambda: calls.append("shutdown"),
                ok=lambda: True,
                executors=types.SimpleNamespace(ExternalShutdownException=ExternalShutdownException),
            )
            namespace = {"rclpy": rclpy, constructor: lambda: node,
                         "ExternalShutdownException": ExternalShutdownException}
            exec(compile(ast.Module(body=[main], type_ignores=[]), relative_path, "exec"), namespace)
            try:
                namespace["main"]()
            except ValueError:
                assert error is ValueError
            else:
                assert error is not ValueError
            assert calls == ["destroy", "shutdown"], relative_path
