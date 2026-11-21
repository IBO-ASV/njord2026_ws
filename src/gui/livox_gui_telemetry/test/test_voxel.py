from types import SimpleNamespace

from livox_gui_telemetry.voxel import cloud_points, voxelize


def test_voxelize_keeps_one_per_cell_and_honours_limit():
    assert len(voxelize([(0.1, 0.1, 0.1, 1), (0.2, 0.2, 0.2, 2), (1.1, 0, 0, 3)], 1.0, 10)) == 2
    assert len(voxelize([(float(i), 0, 0, 0) for i in range(10)], .1, 3)) == 3


def test_cloud_points_checks_required_names_against_field_mapping():
    message = SimpleNamespace(
        fields=[SimpleNamespace(name=name, offset=offset) for name, offset in (("x", 0), ("y", 4), ("z", 8))],
        point_step=12, is_bigendian=False, data=b"\0" * 12,
    )
    assert cloud_points(message) == [(0.0, 0.0, 0.0, 0.0)]


def test_downsampler_preserves_near_and_hull_points_by_default():
    from geometry_msgs.msg import TransformStamped
    from livox_gui_telemetry.nodes import Downsampler, make_cloud

    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    sent = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)),
        last=0, period=0.5,
        tf=SimpleNamespace(lookup_transform=lambda *args, **kwargs: transform),
        min_range=0.0, max_range=0.0, exclude_hull=False,
        voxel=0.05, limit=20000, pub=SimpleNamespace(publish=sent.append),
    )
    from builtin_interfaces.msg import Time
    stamp = Time(sec=1, nanosec=0)
    scan = make_cloud(stamp, 'livox_frame', [(0.1, 0.2, 0.3, 1.0), (0.11, 0.21, 0.31, 1.0), (2.0, 0.0, 0.0, 2.0)])
    from unittest.mock import patch
    from livox_gui_telemetry.nodes import transform_point
    with patch('livox_gui_telemetry.nodes.transform_point', wraps=transform_point) as transform_mock:
        Downsampler.callback(node, scan)
        assert transform_mock.call_count == 2
    assert len(sent) == 1
    assert sent[0].header.frame_id == 'base_link'
    assert len(cloud_points(sent[0])) == 2
