# Detection 実験メモ

Njord のブイ検出実験で、LiDAR 前処理、YOLO、Camera-LiDAR fusion を動かす前に確認する内容をまとめる。

## 全体の流れ

```text
rosbag / Livox
  -> /livox/lidar
  -> pcl_preprocessing
  -> /pcl/lidar_aligned
  -> /pcl/preprocessed

ZED image
  -> YOLO
  -> /yolo/detections
  -> /yolo/debug_image

/pcl/preprocessed + /yolo/detections + /zed/zed_node/rgb/camera_info
  -> camera_lidar_fusion
  -> /fusion/buoy_points
  -> /fusion/buoy_markers
  -> /fusion/camera_frustum
```

## 実験前チェック

### 1. build

```bash
cd ~/workspace/njord2026_ws
colcon build --symlink-install --packages-select pcl_preprocessing camera_lidar_fusion yolo
source install/setup.bash
```

別ターミナルを開いたら毎回:

```bash
cd ~/workspace/njord2026_ws
source install/setup.bash
```

### 2. rosbag 再生

rosbag の時刻を使うため `--clock` を付ける。

```bash
ros2 bag play <bag_dir> --clock --read-ahead-queue-size 10000
```

ゆっくり確認したい場合:

```bash
ros2 bag play <bag_dir> --clock --rate 0.05 --read-ahead-queue-size 10000
```

`<bag_dir>` は `.db3` ファイルそのものではなく、`.db3` が入っているディレクトリ。

### 3. Foxglove

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Foxglove は `ws://localhost:8765` に接続する。

3D panel の Fixed Frame は、基本 `base_link` を使う。

表示するとよい topic:

```text
/pcl/preprocessed
/fusion/buoy_points
/fusion/buoy_markers
/fusion/camera_frustum
```

画像確認:

```text
/yolo/debug_image
```

## 起動コマンド

### LiDAR 前処理

```bash
ros2 launch pcl_preprocessing preprocessing.launch.py \
  input_topic:=/livox/lidar \
  aligned_topic:=/pcl/lidar_aligned \
  output_topic:=/pcl/preprocessed
```

### YOLO

```bash
ros2 launch yolo yolo.launch.py
```

YOLO の model、device、入力画像 topic、画像 ROI は `src/detection/yolo/config/yolo_params.yaml` で調整する。

### Camera-LiDAR fusion

```bash
ros2 launch camera_lidar_fusion fusion.launch.py \
  use_sim_time:=true \
  use_colorizer:=false \
  lidar_topic:=/pcl/preprocessed \
  info_topic:=/zed/zed_node/rgb/camera_info \
  detections_topic:=/yolo/detections
```

カメラ中心の物理位置を上書きする場合:

```bash
ros2 launch camera_lidar_fusion fusion.launch.py \
  use_sim_time:=true \
  use_colorizer:=false \
  lidar_topic:=/pcl/preprocessed \
  info_topic:=/zed/zed_node/rgb/camera_info \
  detections_topic:=/yolo/detections \
  fusion_camera_x:=0.263 \
  fusion_camera_y:=0.00 \
  fusion_camera_z:=0.67
```

`fusion_camera_x/y/z` は、補正後の `base_link` から見たカメラ中心位置 [m]。

```text
x: 前方が +
y: 左が +、右が -
z: 上が +
```

## YAML の役割

### `pcl_preprocessing/config/preprocessing_params.yaml`

LiDAR 点群を `base_link` 基準に合わせ、不要点を落とす。

主な項目:

```text
roll_deg / pitch_deg / yaw_deg
  反転取付された Mid-360s の姿勢補正。

translation_x / translation_y / translation_z
  LiDAR raw 原点から補正後 base_link への平行移動。

roi_min_x ... roi_max_z
  base_link 基準の大まかな点群範囲。

remove_rear_points / rear_cut_min_x
  後方点の除去。

remove_floor_points / floor_cut_min_z
  床・水面・低すぎる点の除去。

voxel_leaf_x/y/z
  点群密度を落とす VoxelGrid。
```

ここで出る `/pcl/preprocessed` の `header.frame_id` は `base_link`。

### `yolo/config/yolo_params.yaml`

YOLO 推論と画像内 ROI を調整する。

主な項目:

```text
model_path
  空文字なら yolo/config/best.pt を使う。

device
  "cuda:0" または "cpu"。

camera_topic
  ZED の入力画像 topic。

use_image_roi / roi_*_ratio
  YOLO をかける画像範囲。画像上部を見ない、などの調整に使う。

inference_hz / debug_image_hz
  推論周期と debug image publish 周期。

detections_topic
  YOLO bbox の出力。通常は /yolo/detections。
```

### `camera_lidar_fusion/config/fusion_params.yaml`

YOLO bbox と LiDAR 点を対応づけ、ブイ候補点と marker を出す。

主な項目:

```text
camera_frame
  LiDAR 点を投影するカメラ optical frame。
  現在は fusion_zed_left_camera_optical_frame を使う。

association_mode
  "bbox": YOLO bbox 内に投影された LiDAR 点だけを使う。
  "bearing": YOLO bbox 中心の横方向だけを見る。

bbox_margin_px
  bbox を何 pixel 広げるか。

use_nearest_range_gate / nearest_range_window
  bbox 内の一番手前の距離帯だけを残す。
  奥の壁や別物体を拾うときに効く。

use_z_gate / z_min / z_max
  base_link 基準の高さで大外れを落とす。

publish_camera_frustum / frustum_depth
  カメラ画角を /fusion/camera_frustum に描画する。
```

## 座標系で見るべきこと

### LiDAR 側

`/pcl/preprocessed` を Fixed Frame `base_link` で見る。

確認:

```text
船体前方が +X
左が +Y
上が +Z
ブイが真正面にあるなら +X 方向に点群が出る
```

### カメラ側

ZED wrapper が出す `zed_left_camera_optical_frame` は自己位置推定で動くことがある。fusion では固定の専用 frame を使う。

確認:

```bash
ros2 run tf2_ros tf2_echo base_link fusion_zed_left_camera_optical_frame
```

値が固定であることを見る。

`/fusion/camera_frustum` の黄色線がカメラ中心方向。ブイが画像中心に映っているなら、黄色線方向にブイ点群があるのが期待値。

## 確認コマンド

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /pcl/preprocessed
ros2 topic hz /yolo/detections
ros2 topic hz /fusion/buoy_points
ros2 topic echo /fusion/buoy_markers --once
ros2 topic echo /zed/zed_node/rgb/camera_info --once
```

TF:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link fusion_zed_left_camera_optical_frame
```

## よくある症状

### YOLO は合っているが、奥の点をブイにする

`camera_lidar_fusion/config/fusion_params.yaml` を確認する。

```yaml
association_mode: "bbox"
use_nearest_range_gate: true
nearest_range_window: 0.5
```

`nearest_range_window` を小さくすると、bbox 内の手前点を優先する。

### 床や高い点を拾う

```yaml
use_z_gate: true
z_min: -0.3
z_max: 0.6
```

ブイが消える場合は `z_max` を広げる。

### カメラ画角が LiDAR 点群と合わない

`fusion_camera_x/y/z` と `fusion_camera_yaw/pitch/roll` を調整する。

まず位置:

```bash
fusion_camera_x:=...
fusion_camera_y:=...
fusion_camera_z:=...
```

次に向き:

```bash
fusion_camera_yaw:=...
fusion_camera_pitch:=...
fusion_camera_roll:=...
```

`/fusion/camera_frustum` を見ながら、黄色線が実際のカメラ正面を向くように合わせる。
