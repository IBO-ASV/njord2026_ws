"""Installed nodes must locate resources through ament, not site-packages."""
from pathlib import Path
from types import SimpleNamespace

from mission_manager import mission_manager_node as module


def test_registry_default_uses_installed_share(monkeypatch, tmp_path):
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(tmp_path / name))
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=''))
    assert module.MissionManager._registry_path(node) == tmp_path / 'mission_manager' / 'config' / 'task_registry.yaml'


def test_registry_explicit_path_is_preserved(monkeypatch, tmp_path):
    def unexpected_lookup(name):
        raise AssertionError('explicit paths must not require package lookup')
    monkeypatch.setattr(module, 'get_package_share_directory', unexpected_lookup)
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=str(tmp_path / 'custom.yaml')))
    assert module.MissionManager._registry_path(node) == tmp_path / 'custom.yaml'


def test_waypoint_default_uses_installed_share(monkeypatch, tmp_path):
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(tmp_path / name))
    assert module.MissionManager._waypoint_share_path() == tmp_path / 'waypoint_publisher'
