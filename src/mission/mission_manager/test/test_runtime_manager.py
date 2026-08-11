import threading

from mission_manager.runtime_manager_node import RuntimeManager
from njord_interfaces.msg import Nav2RuntimeStatus


class _ExitedProcess:
    def poll(self):
        return 1


def test_exited_runtime_is_reported_as_failed():
    manager = object.__new__(RuntimeManager)
    manager._lock = threading.Lock()
    manager._process = _ExitedProcess()
    published = []
    manager._publish = lambda state, message: published.append((state, message))

    manager._monitor_process()

    assert manager._process is None
    assert published == [(Nav2RuntimeStatus.FAILED, "Nav2 runtime exited with code 1")]
