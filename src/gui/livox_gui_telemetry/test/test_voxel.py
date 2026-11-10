from types import SimpleNamespace

from livox_gui_telemetry.voxel import cloud_points, voxelize


def test_voxelize_keeps_one_per_cell_and_honours_limit():
    assert len(voxelize([(0.1, 0.1, 0.1, 1), (0.2, 0.2, 0.2, 2), (1.1, 0, 0, 3)], 1.0, 10)) == 2
    assert len(voxelize([(float(i), 0, 0, 0) for i in range(10)], .1, 3)) == 3


def test_cloud_points_accepts_field_dict_keys():
    message = SimpleNamespace(
        fields=[SimpleNamespace(name=name, offset=offset) for name, offset in (("x", 0), ("y", 4), ("z", 8))],
        point_step=12, is_bigendian=False, data=b"\0" * 12,
    )
    assert cloud_points(message) == [(0.0, 0.0, 0.0, 0.0)]
