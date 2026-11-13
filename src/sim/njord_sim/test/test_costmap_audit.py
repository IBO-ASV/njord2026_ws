from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import MapMetaData, OccupancyGrid
from njord_sim.costmap_audit import CostmapAudit
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class Observer:
    def create_subscription(self, *args):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000))


def full(stamp, width, data):
    return OccupancyGrid(
        header=Header(stamp=Time(sec=stamp)),
        info=MapMetaData(width=width, height=1, resolution=1.0),
        data=data,
    )


def test_audit_does_not_reapply_stale_patches_after_a_new_full_grid():
    audit = CostmapAudit(Observer())
    audit.full("local_costmap", full(10, 3, [0, 0, 0]))
    patch = OccupancyGridUpdate(
        header=Header(stamp=Time(sec=9)), x=1, y=0, width=1, height=1, data=[100]
    )
    audit.update("local_costmap", patch)
    assert not audit.grids["local_costmap"]["data"].any()
    patch.header.stamp.sec = 11
    audit.update("local_costmap", patch)
    assert audit.grids["local_costmap"]["data"][0, 1] == 100
    audit.full("local_costmap", full(12, 2, [0, 0]))
    audit.update("local_costmap", patch)
    assert not audit.grids["local_costmap"]["data"].any()


def test_residual_report_separates_observed_free_space_from_unknown_shadow():
    audit = CostmapAudit(Observer())
    for name in ["global_costmap", "local_costmap"]:
        audit.full(name, full(10, 5, [100, 0, 100, 0, 100]))
    audit.points = np.array([[0.5, 0.5]])
    audit.scan = LaserScan(
        header=Header(stamp=Time(sec=10)),
        angle_min=-np.pi,
        angle_increment=np.pi / 2,
        ranges=[4.0, 4.0, 4.0, 4.0],
    )
    audit.poses[10_000_000_000] = (0.0, 0.0, 0.0)
    snapshot = audit.snapshot()
    assert snapshot["ready"]
    for grid in snapshot["maps"].values():
        assert grid["lethal_cells"] == 3
        assert grid["residual_cells"] == 2
        assert grid["visible_residual_cells"] == 1
        assert grid["occluded_or_unobserved_residual_cells"] == 1


def test_snapshot_uses_last_matched_pair_if_new_scan_precedes_its_odometry():
    audit = CostmapAudit(Observer())
    for name in ["global_costmap", "local_costmap"]:
        audit.full(name, full(10, 1, [0]))
    audit.points = np.empty((0, 2))
    for stamp in [10, 11]:
        audit.laser(
            LaserScan(
                header=Header(stamp=Time(sec=stamp)),
                angle_min=-np.pi,
                angle_increment=np.pi / 2,
                ranges=[4.0] * 4,
            )
        )
    audit.poses[10_000_000_000] = (0.0, 0.0, 0.0)
    snapshot = audit.snapshot()
    assert snapshot["ready"]
    assert snapshot["scan_pose_stamp_ns"] == 10_000_000_000
    assert snapshot["scan_pose_lag_ms"] == 1000.0
