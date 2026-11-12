"""A rejected path must cancel even a FollowPath acceptance still in flight."""

from concurrent.futures import Future
from types import SimpleNamespace

from asv_trajectory_planner.follow_path_client_node import FollowPathClientNode


def test_path_permission_loss_cancels_active_goal_and_discards_path():
    calls = []
    handle = SimpleNamespace(cancel_goal_async=lambda: calls.append("cancel"))
    node = SimpleNamespace(
        path_permitted=True,
        latest_path=object(),
        goal_generation=3,
        goal_handle=handle,
        goal_active=True,
    )
    FollowPathClientNode.path_permission_callback(node, SimpleNamespace(data=False))
    assert calls == ["cancel"]
    assert node.latest_path is None
    assert node.goal_generation == 4
    assert not node.goal_active and not node.path_permitted


def test_late_goal_acceptance_is_canceled_after_gate_generation_changes():
    calls = []
    handle = SimpleNamespace(
        accepted=True, cancel_goal_async=lambda: calls.append("cancel")
    )
    future = Future()
    future.set_result(handle)
    node = SimpleNamespace(goal_generation=4)
    FollowPathClientNode.goal_response_callback(node, future, 3)
    assert calls == ["cancel"]
