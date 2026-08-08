# task2_perception

Task 2 (collision avoidance) LiDAR perception glue for the Njord ASV.
Bridges the raw Livox Mid-360 cloud into the reused `pcl_segmentation`
submodule pipeline, and bridges the submodule tracker output into the MPPI
planner.

岸壁認識はTask 2の対象外となったため、feature/quay-perceptionブランチへ保存し、task2-experimentの認識・計画・制御ループから除外した。

## Full chain

```
/livox/lidar (PointCloud2, livox_frame, ~10 Hz, xyz+intensity)
    |
    v
task2_cloud_filter                              [this package]
    TF livox_frame -> base_link (URDF owns the upside-down correction)
    nonfinite -> range (1.5-40 m; own-hull radius excluded) -> self crop-box
    -> object height band
    -> water removal (z band + guarded RANSAC horizontal plane)
    |
    v
/task2/points_filtered (base_link)
    |
    v
pcl_segmentation submodule (reused AS-IS, zero modification):
    ros2 launch ship_perception_bringup classical_pipeline.launch.py \
        lidar_topic:=/task2/points_filtered \
        ego_odom_topic:=/odometry/filtered/local
    preprocessing_node (Task 2 ROI, voxel 0.15 m, no frame accumulation)
        -> /pcl/preprocessed
    ground_remover_node -> /pcl/nonground
    cluster_node -> /pcl/cluster_centroids (PoseArray)
    ship_tracker_node (EKF, Hungarian association)
        -> /tracked_objects
           (ship_perception_msgs/TrackedObjectArray,
            pose base_link-relative, twist RELATIVE and
            expressed in the object body frame — the
            tracker's ego compensation is a no-op stub)
        -> TF base_link -> tracked_obj_{id}
    |
    v
opponent_selector                              [this package]
    gate (confirmed / distance / size /
    points / staleness) -> rank ("nearest"
    | "min_tcpa" | "track_id") -> top-1
    body twist -> base axes -> + ego twist
    -> map frame -> low-pass + spike gate
    |
    v
/other_ship/twist (TwistStamped, map frame, ABSOLUTE ground velocity)
+ TF map -> opponent_vessel
    |
    v
MPPI planner (asv_trajectory_planner/planner_node.py,
    other_twist_is_relative=False, require_other_ship=False:
    no opponent published -> plans a straight path — safe degradation)
```

## Nodes

| Node (executable) | In | Out |
|---|---|---|
| `task2_cloud_filter` (`task2_cloud_filter_node`) | `/livox/lidar`, TF | `/task2/points_filtered`, `/task2/points_filtered_visual` (display only), `/task2/self_vessel_marker`, `/task2/debug/*` (opt.) |
| `opponent_selector` (`opponent_selector_node`) | `/tracked_objects`, `/odometry/filtered/local`, TF map->base_link | `/other_ship/twist`, TF map->opponent_vessel |

認識単体のデバッグでは、以下の2つを起動します。

```
ros2 launch task2_perception task2_perception.launch.py
ros2 launch ship_perception_bringup classical_pipeline.launch.py \
  lidar_topic:=/task2/points_filtered
```

前者はTask 2用の点群フィルタと他船選択、後者はクラスタリングと追跡を担当します。
実機の自動運転では両方とも `robot/task2_autonomy.launch.py` に含まれるため、別途起動しません。

rosbagでこの一連の認識を確認する場合は、`robot/task2_bag_perception.launch.py` を使います。
これはbag再生と必要なTFを補い、認識・追跡だけを起動するオフライン検証用です。

## Design notes

- **Inverted LiDAR**: the Mid-360 is mounted upside-down. The nominal
  roll = pi correction lives ONLY in the URDF `lidar_joint`
  (`src/robot/urdf/robot.urdf.xacro`); all nodes consume TF. The
  `lidar_inverted` / `lidar_*_deg` parameters of the cloud filter are an
  emergency manual pre-rotation for URDF-less setups and warn loudly when
  active. **HUMAN: measure the real mount angles before trusting
  perception/GLIM.**
- **Water removal** cannot eat walls or decks: the RANSAC plane is accepted
  only when its normal is near-vertical (`water_plane_normal_z_min`) AND its
  height is within `water_plane_max_height_error_m` of `waterline_z_m`.
- **Velocities**: the tracker EKF already estimates velocity; nothing here
  numerically differentiates positions. The selector rotates the body-frame
  twist into base_link axes, adds the ego twist (`/odometry/filtered/local`,
  child frame base_link), rotates into map, then applies the same low-pass +
  spike rejection style as `opponent_twist_from_tf_node.py`.
- **Safe degradation**: no valid confirmed track -> `/other_ship/twist` and
  the opponent TF simply stop (MPPI plans straight).

## Pure modules / tests

`cloud_ops.py`, `tracking_glue.py`, `smoothing.py` are
ROS-free (numpy only) and covered by pytest:

```
python3 -m pytest src/detection/task2_perception/test/ -q
```

## Parameters that MUST be measured on the vessel

See the unified `config/task2_params.yaml`, marked `HUMAN: measure on vessel`:
`waterline_z_m`, `min_range_m` (own-hull exclusion radius), the `self_crop_*`
box, and the URDF LiDAR mount
angles/position.

`/task2/self_vessel_marker` is a visualization-only pentagonal outline of the
configured `self_crop_*` footprint. Its pointed bow faces `+X` (forward).
Add it as a Marker in Foxglove with `base_link` (or its parent frame) as the
fixed frame to see the own-vessel reference against the point cloud. It is
disabled by the real-vessel autonomy overlay.

`/task2/points_filtered_visual` is another visualization-only topic: it keeps
x/y (the horizontal azimuth) unchanged and mirrors only z. Use it in a
Foxglove panel instead of `/task2/points_filtered` when inspecting the
upside-down LiDAR; do not connect it to perception or planning.
