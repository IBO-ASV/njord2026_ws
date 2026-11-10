"""Tests 1-5: rigid transform (upside-down mount), water removal, self crop.

numpy only — no rclpy import anywhere in this suite.
"""

import numpy as np

from task2_perception import cloud_ops

LIDAR_XYZ = np.array([0.5, 0.0, 0.8])  # URDF lidar_joint translation


def make_mount_transform(roll=np.pi, pitch=0.0, yaw=0.0, xyz=LIDAR_XYZ):
    """base_link <- livox_frame transform as published from the URDF."""
    rot = cloud_ops.rpy_to_rotation_matrix(roll, pitch, yaw)
    return cloud_ops.make_transform(rot, xyz)


class TestFrameTimestampDecision:
    def test_first_frame_is_accepted(self):
        assert cloud_ops.frame_timestamp_decision(100, None, 200) == (True, False)

    def test_frame_inside_rate_period_is_dropped(self):
        assert cloud_ops.frame_timestamp_decision(250, 100, 200) == (False, False)

    def test_bag_loop_timestamp_rollback_is_accepted_and_reported(self):
        assert cloud_ops.frame_timestamp_decision(100, 1_000, 200) == (True, True)


class TestInvertedMountTransform:
    """Test 1: equivalence check of the URDF roll=pi correction."""

    def test_roll_pi_flips_y_and_z(self):
        transform = make_mount_transform()
        # A point 10 m ahead, 2 m left, 1 m up in the (upside-down) sensor frame.
        pts = np.array([[10.0, 2.0, 1.0]])
        out = cloud_ops.apply_transform(pts, transform)
        # roll=pi about x: y -> -y, z -> -z, then the mount offset is added.
        expected = np.array([[10.0 + 0.5, -2.0, -1.0 + 0.8]])
        np.testing.assert_allclose(out, expected, atol=1e-9)

    def test_water_surface_maps_below_base_link(self):
        # With the sensor upside-down, a return from the water surface appears
        # at positive z in the raw sensor frame (sensor z axis points down).
        # After the URDF correction it must land BELOW base_link.
        transform = make_mount_transform()
        sensor_height_above_water = 1.1  # nominal: lidar_z 0.8 + draft margin
        water_hit = np.array([[5.0, 0.0, sensor_height_above_water]])
        out = cloud_ops.apply_transform(water_hit, transform)
        assert out[0, 2] < 0.0
        np.testing.assert_allclose(
            out[0, 2], 0.8 - sensor_height_above_water, atol=1e-9)

    def test_round_trip_identity(self):
        transform = make_mount_transform()
        inv = np.linalg.inv(transform)
        pts = np.random.default_rng(1).normal(size=(50, 3)) * 10.0
        back = cloud_ops.apply_transform(
            cloud_ops.apply_transform(pts, transform), inv)
        np.testing.assert_allclose(back, pts, atol=1e-9)

    def test_intensity_column_is_preserved(self):
        transform = make_mount_transform()
        pts = np.array([[1.0, 2.0, 3.0, 42.0], [4.0, 5.0, 6.0, 7.0]])
        out = cloud_ops.apply_transform(pts, transform)
        np.testing.assert_allclose(out[:, 3], [42.0, 7.0])


class TestWaterRemoval:
    """Tests 2-4: water plane removed; quay wall and ship cluster survive."""

    WATERLINE = -0.3

    def _scene(self):
        rng = np.random.default_rng(42)
        # Water surface: horizontal plane at z = waterline with ripple noise.
        water_xy = rng.uniform([-30.0, -30.0], [30.0, 30.0], size=(600, 2))
        water = np.column_stack(
            [water_xy, self.WATERLINE + rng.normal(0, 0.015, 600)])
        # Vertical quay wall: x in [10, 30] at y = 12, z from above the removal
        # band up to +3 m.
        wall_x = rng.uniform(10.0, 30.0, 300)
        wall_z = rng.uniform(self.WATERLINE + 0.3, 3.0, 300)
        wall = np.column_stack(
            [wall_x, np.full(300, 12.0) + rng.normal(0, 0.03, 300), wall_z])
        # Opponent ship cluster: blob above the water around (15, -5).
        ship = np.column_stack([
            15.0 + rng.normal(0, 1.5, 200),
            -5.0 + rng.normal(0, 0.8, 200),
            rng.uniform(0.2, 1.5, 200),
        ])
        return water, wall, ship

    def _run(self, pts):
        return cloud_ops.water_removal(
            pts,
            waterline_z_m=self.WATERLINE,
            band_min_m=-0.15,
            band_max_m=0.1,
            use_ransac=True,
            distance_threshold_m=0.05,
            normal_z_min=0.9,
            max_height_error_m=0.3,
            iterations=200,
            rng=np.random.default_rng(7),
        )

    def test_water_plane_is_removed(self):
        """Test 2: the horizontal plane near the waterline is removed."""
        water, wall, ship = self._scene()
        pts = np.vstack([water, wall, ship])
        kept, rejected = self._run(pts)
        # Nearly all water points are gone.
        assert rejected.shape[0] >= 0.9 * len(water)
        assert all(abs(z - self.WATERLINE) < 0.2 for z in rejected[:, 2])
        # RANSAC alone also finds a horizontal plane at the right height.
        mask, plane = cloud_ops.fit_horizontal_plane_ransac(
            pts, distance_threshold_m=0.05, normal_z_min=0.9,
            waterline_z_m=self.WATERLINE, max_height_error_m=0.3,
            max_iterations=200, rng=np.random.default_rng(7))
        assert plane is not None
        assert plane[2] >= 0.9
        assert abs(-plane[3] / plane[2] - self.WATERLINE) < 0.1

    def test_vertical_wall_survives(self):
        """Test 3: quay wall points are NOT removed by the water filters."""
        water, wall, ship = self._scene()
        pts = np.vstack([water, wall, ship])
        kept, _ = self._run(pts)
        # Count survivors near the wall plane y ~ 12.
        wall_kept = np.sum(np.abs(kept[:, 1] - 12.0) < 0.5)
        assert wall_kept >= 0.95 * len(wall)
        # A wall-only scene must never yield a "water" plane (normal guard).
        mask, plane = cloud_ops.fit_horizontal_plane_ransac(
            wall, distance_threshold_m=0.05, normal_z_min=0.9,
            waterline_z_m=self.WATERLINE, max_height_error_m=0.3,
            max_iterations=200, rng=np.random.default_rng(8))
        assert plane is None
        assert not mask.any()

    def test_ship_cluster_survives(self):
        """Test 4: the above-water ship blob survives water removal."""
        water, wall, ship = self._scene()
        pts = np.vstack([water, wall, ship])
        kept, _ = self._run(pts)
        ship_kept = np.sum(
            (np.abs(kept[:, 0] - 15.0) < 6.0)
            & (np.abs(kept[:, 1] + 5.0) < 4.0)
            & (kept[:, 2] > 0.0))
        assert ship_kept >= 0.95 * len(ship)


class TestSelfCropAndBasicFilters:
    """Test 5: self crop-box removal, plus NaN and range filters."""

    def test_self_crop(self):
        self_min = [-1.2, -0.8, -0.5]
        self_max = [1.2, 0.8, 1.5]
        inside = np.array([[0.0, 0.0, 0.0], [1.1, -0.7, 1.4], [-0.5, 0.3, -0.4]])
        outside = np.array([[5.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]])
        pts = np.vstack([inside, outside])
        out = cloud_ops.crop_box_remove(pts, self_min, self_max)
        np.testing.assert_allclose(out, outside)

    def test_object_height_band_keep(self):
        pts = np.array([[1.0, 0.0, -1.0], [1.0, 0.0, 0.5], [1.0, 0.0, 5.0]])
        out = cloud_ops.crop_box_keep(
            pts, [-np.inf, -np.inf, -0.2], [np.inf, np.inf, 4.0])
        np.testing.assert_allclose(out, [[1.0, 0.0, 0.5]])

    def test_nan_removal(self):
        pts = np.array([[1.0, 2.0, 3.0],
                        [np.nan, 0.0, 0.0],
                        [0.0, np.inf, 0.0],
                        [4.0, 5.0, 6.0]])
        out = cloud_ops.remove_nonfinite(pts)
        np.testing.assert_allclose(out, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    def test_range_band(self):
        pts = np.array([[0.1, 0.0, 0.0], [10.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        out = cloud_ops.range_filter(pts, 0.5, 60.0)
        np.testing.assert_allclose(out, [[10.0, 0.0, 0.0]])

    def test_voxel_downsample(self):
        pts = np.array([[0.01, 0.01, 0.01, 1.0],
                        [0.02, 0.02, 0.02, 3.0],
                        [5.0, 5.0, 5.0, 7.0]])
        out = cloud_ops.voxel_downsample(pts, 0.1)
        assert out.shape == (2, 4)
        # Disabled passthrough.
        np.testing.assert_allclose(cloud_ops.voxel_downsample(pts, 0.0), pts)


class TestRansacGuards:
    """Test 2 (RANSAC path) and test 9 (deck plane survives)."""

    def _water(self, z, n=400, noise=0.04, seed=11):
        rng = np.random.default_rng(seed)
        xy = rng.uniform([-20.0, -20.0], [20.0, 20.0], size=(n, 2))
        return np.column_stack([xy, z + rng.uniform(-noise, noise, n)])

    def _ship(self, n=120, seed=12):
        rng = np.random.default_rng(seed)
        return np.column_stack([
            15.0 + rng.normal(0, 1.5, n),
            5.0 + rng.normal(0, 0.8, n),
            rng.uniform(0.3, 2.0, n),
        ])

    def test_band_only_path_removes_water_at_waterline(self):
        """Test 2 (band-only path): the z band alone removes a flat water
        plane at the waterline, keeping the ship."""
        water = self._water(z=0.0, noise=0.03)
        ship = self._ship()
        pts = np.vstack([water, ship])
        kept, rejected = cloud_ops.water_removal(
            pts, waterline_z_m=0.0, band_min_m=-0.3, band_max_m=0.15,
            use_ransac=False)
        assert rejected.shape[0] == len(water)
        assert kept.shape[0] == len(ship)
        assert kept[:, 2].min() > 0.15

    def test_ransac_path_removes_water_the_band_misses(self):
        """Test 2 (RANSAC path): a rippled water plane wider than a NARROW
        z band is only fully removed when the RANSAC path is enabled."""
        water = self._water(z=0.0)
        ship = self._ship()
        pts = np.vstack([water, ship])
        band_only, _ = cloud_ops.water_removal(
            pts, waterline_z_m=0.0, band_min_m=-0.01, band_max_m=0.01,
            use_ransac=False)
        assert band_only.shape[0] > len(ship)  # the band alone is insufficient
        kept, rejected = cloud_ops.water_removal(
            pts, waterline_z_m=0.0, band_min_m=-0.01, band_max_m=0.01,
            use_ransac=True, distance_threshold_m=0.05, iterations=200,
            rng=np.random.default_rng(13))
        assert kept.shape[0] == len(ship)
        assert rejected.shape[0] == len(water)

    def test_deck_plane_two_meters_up_survives(self):
        """Test 9: a horizontal deck plane 2 m above the waterline fails the
        height guard and is never removed by the RANSAC path."""
        deck = self._water(z=2.0, noise=0.02, seed=14)
        kept, rejected = cloud_ops.water_removal(
            deck, waterline_z_m=0.0, band_min_m=-0.3, band_max_m=0.15,
            use_ransac=True, max_height_error_m=0.3, iterations=200,
            rng=np.random.default_rng(15))
        assert kept.shape[0] == len(deck)
        assert rejected.shape[0] == 0
        # Direct check: the plane fit itself must refuse the deck.
        mask, plane = cloud_ops.fit_horizontal_plane_ransac(
            deck, waterline_z_m=0.0, max_height_error_m=0.3,
            max_iterations=200, rng=np.random.default_rng(16))
        assert plane is None
        assert not mask.any()
