# Task 2 パラメータリファレンス (task2-experiment)

> **注意**: §7の `task2_real.launch.py` 引数は旧構成であり、現在は使用しません。現在の起動構成は [`task2_launch_guide.md`](task2_launch_guide.md) を参照してください。

- 作業日: 2026-07-19
- 各表の記号: 「人が調整」 ◎=実測/実機調整**必須**(公称値のまま信用しない)、○=実機で調整が想定される、−=通常は触らない。
- 「Jetson確認」「実機確認」: ✓=その段階で値の妥当性を確認すべき項目(手順は `Docs/task2_jetson_validation.md`)。**現時点ではいずれも未確認**。
- デフォルト値はすべてコード/YAML から転記(静的確認済み)。
- 岸壁認識はTask 2の対象外となったため、feature/quay-perceptionブランチへ保存し、task2-experimentの認識・計画・制御ループから除外した。

---

## 1. 人が調整する値(要注意リスト)

各ノードの詳細表は §2 以降。まず優先度の高い「人が決める値」を集約する:

| 区分 | パラメータ | 場所 | 公称値 | 理由 |
|---|---|---|---|---|
| ◎ 実測必須 | URDF LiDAR 取付角 `lidar_roll/pitch/yaw`・位置 `lidar_x/y/z` | `src/robot/urdf/robot.urdf.xacro` | roll=π, z=0.8 m | 上下逆さま補正の唯一の場所。実測せずに認識/GLIM を信用しない |
| ◎ 実測必須 | `waterline_z_m` | task2_params.yaml | 0.0 m | 喫水と LiDAR 高さに依存 |
| ◎ 実測必須 | `self_crop_min/max_x/y/z`(6 値) | 同上 | x±1.2, y±0.8, z −0.5〜1.5 m | 船体・マスト実寸で決める |
| ◎ 配置必須 | YOLO11s 重み(`model_path` / `yolo_model_path`) | yolo11s_params.yaml / launch 引数 | リポジトリ非含有 | 学習済み .pt / .engine を人が配置 |
| ◎ 現地確定 | `task2_waypoints.yaml` の x/y(map 座標・オフライン事前計算) | waypoint_publisher/config | プール C→A 対角 約46 m | 実験水域の GPS で再生成 |
| ○ 実機調整 | MPPI `mppi.*` 全般(特に target_speed, horizon, num_samples, CRM ゲイン) | mppi_params.yaml | 旧ハードコード値 | 変更時は値+日付+理由を YAML に記録する運用 |
| ○ 実機調整 | YOLO [TUNE] 群(conf/iou/imgsz/inference_hz/class_names/buoy_class_names) | yolo11s_params.yaml | 下表 | 実コース映像で調整 |
| ○ 実機調整 | opponent_selector ゲート群・平滑化 | task2_params.yaml | 下表 | 実データでの追跡品質次第 |
| ○ 実機調整 | Nav2 速度上限・controller ゲイン | nav2_params_task2.yaml | 下表 | 実艇挙動で確認(read-only 参照 §8) |

これ以外(topic 名・frame 名・出力トピック等)は「コードから決定できる値」であり、システム構成を変えない限り変更不要。

---

## 2. task2_cloud_filter(パッケージ: task2_perception)

設定ファイル: `src/detection/task2_perception/config/task2_params.yaml`

| パラメータ名 | 型 | 単位 | デフォルト | 説明 | 人が調整 | Jetson確認 | 実機確認 |
|---|---|---|---|---|---|---|---|
| `input_topic` | string | − | `/livox/lidar` | 入力点群 topic | − | ✓ | |
| `output_topic` | string | − | `/task2/points_filtered` | 出力点群 topic | − | ✓ | |
| `output_frame` | string | − | `base_link` | 出力 frame(TF 変換先) | − | ✓ | |
| `min_range_m` | double | m | 0.5 | これより近い点を除去(近接ノイズ) | ○ | | ✓ |
| `max_range_m` | double | m | 60.0 | これより遠い点を除去 | ○ | | ✓ |
| `voxel_leaf_size_m` | double | m | 0.0 | voxel 間引き。0=オフ(下流が 0.1 m で間引くため**二重適用しない**) | − | | |
| `accumulation_frames` | int | frame | 1 | フレーム蓄積。1=オフ(下流が 3 フレーム蓄積) | − | | |
| `lidar_inverted` | bool | − | false | 緊急用手動反転(TF 前に適用)。**URDF が正: 通常 false のまま**。有効時は起動警告 | − | | |
| `lidar_roll_deg` / `lidar_pitch_deg` / `lidar_yaw_deg` | double | deg | 0.0 | 緊急用手動プリ回転。既定 no-op | − | | |
| `waterline_z_m` | double | m | 0.0 | base_link 系での水面高さ。**実測必須** | ◎ | | ✓ |
| `water_remove_min_z_m` | double | m | −0.3 | 水面除去帯の下端(waterline 相対) | ○ | | ✓ |
| `water_remove_max_z_m` | double | m | 0.15 | 水面除去帯の上端(waterline 相対) | ○ | | ✓ |
| `water_plane_distance_threshold_m` | double | m | 0.05 | 水面 RANSAC 平面インライア距離 | ○ | | ✓ |
| `water_plane_normal_z_min` | double | − | 0.9 | 法線 z 下限(ほぼ水平の面のみ水面候補 → 岸壁を食わない) | − | | ✓ |
| `water_plane_max_height_error_m` | double | m | 0.3 | 平面高さの waterline からの許容誤差(デッキ等を残す) | ○ | | ✓ |
| `use_water_plane_ransac` | bool | − | true | RANSAC 水面除去の有効化 | − | | ✓ |
| `self_crop_min_x` / `self_crop_max_x` | double | m | −1.2 / 1.2 | 自船クロップ AABB(内側を除去)。**実測必須** | ◎ | | ✓ |
| `self_crop_min_y` / `self_crop_max_y` | double | m | −0.8 / 0.8 | 同上 | ◎ | | ✓ |
| `self_crop_min_z` / `self_crop_max_z` | double | m | −0.5 / 1.5 | 同上 | ◎ | | ✓ |
| `object_min_z_m` / `object_max_z_m` | double | m | −0.2 / 4.0 | 物体高さバンド(この範囲のみ keep) | ○ | | ✓ |
| `publish_debug` | bool | − | false | `/task2/debug/*` 4 topic を出力 | − | ✓ | |

## 3. opponent_selector(パッケージ: task2_perception)

| パラメータ名 | 型 | 単位 | デフォルト | 説明 | 人が調整 | Jetson確認 | 実機確認 |
|---|---|---|---|---|---|---|---|
| `tracked_objects_topic` | string | − | `/tracked_objects` | サブモジュール tracker 出力 | − | ✓ | |
| `ego_odom_topic` | string | − | `/odometry/filtered/local` | 自船 odometry(twist は base_link) | − | ✓ | |
| `twist_topic` | string | − | `/other_ship/twist` | MPPI への出力 topic | − | ✓ | |
| `map_frame` / `base_frame` | string | − | `map` / `base_link` | frame 名 | − | ✓ | |
| `opponent_frame` | string | − | `opponent_vessel` | 出力 TF child frame(MPPI の `other_ship_frame` と一致必須) | − | ✓ | |
| `confirmed_only` | bool | − | true | CONFIRMED トラックのみ採用 | − | | ✓ |
| `max_distance_m` | double | m | 60.0 | 水平距離ゲート | ○ | | ✓ |
| `selection_policy` | string | − | `nearest` | `nearest` \| `min_tcpa` \| `track_id` | ○ | | ✓ |
| `target_track_id` | int | − | −1 | `track_id` policy 時のみ使用 | − | | |
| `min_length_m` / `max_length_m` | double | m | 0.5 / 30.0 | トラック長さ(dimensions.x)ゲート | ○ | | ✓ |
| `min_point_count` | int | 点 | 5 | クラスタ点数ゲート | ○ | | ✓ |
| `stale_timeout_sec` | double | s | 2.0 | これより古いトラックは無視 | ○ | | ✓ |
| `publish_rate_hz` | double | Hz | 10.0 | 出力レート | − | | |
| `twist_lowpass_alpha` | double | − (0..1) | 0.3 | 一次 IIR ローパス係数 | ○ | | ✓ |
| `max_speed_mps` | double | m/s | 5.0 | スパイク棄却ゲート(超過サンプルは破棄) | ○ | | ✓ |
| `min_absolute_speed_mps` | double | m/s | 0.3 | ego 補償後の絶対速度がこの値未満の物体を他船候補から除外 | ○ | | ✓ |

## 3.1 MPPI ブイ外側コスト（パッケージ: asv_trajectory_planner）

通常は `/buoy_detections` の LiDAR 融合位置を用い、GPS 直線に対して各ブイの外側へ出る軌道にコストを与える。`use_virtual_buoys` は旧来の GPS 由来仮想ブイへ戻すための明示的なフォールバックで、既定値は `false`。

| Parameter | Default | Meaning |
| --- | --- | --- |
| `use_detected_buoys` | `true` | `/buoy_detections` を MPPI のブイ座標として使う |
| `use_virtual_buoys` | `false` | GPS 直線上の仮想ブイを使う旧方式へ戻す |
| `buoy_detection_timeout_sec` | `1.0` | この時間を超えた検出を破棄 |
| `buoy_merge_distance_m` | `1.0` | 同一ブイとみなして統合する距離 |

(コード内定数: スパイクゲートの yaw rate 上限 `max_yaw_rate_rps` = 1.5 rad/s、`TwistSmoother` の既定値でパラメータ非公開)

## 4. 岸壁認識(Task 2 対象外)

岸壁認識のパラメータ群は feature/quay-perception ブランチに保存されており、本リファレンスの対象外(§冒頭の注記参照)。

## 5. yolo11_detector = yolo11_node(パッケージ: yolo)

設定ファイル: `src/detection/yolo/config/yolo11s_params.yaml`([TUNE] 付き = 実映像で調整)

| パラメータ名 | 型 | 単位 | デフォルト | 説明 | 人が調整 | Jetson確認 | 実機確認 |
|---|---|---|---|---|---|---|---|
| `model_path` | string | − | `""` | 重みパス。空= `<yolo share>/config/yolo11s.pt`(backend=tensorrt 時 .engine)。**重みは非同梱・欠如時 fatal 終了** | ◎ | ✓ | |
| `device` | string | − | `cuda:0` | 推論デバイス(`cpu` / `cuda:0`)。.engine では無視 | − | ✓ | |
| `backend` | string | − | `pytorch` | 期待形式 `pytorch`(.pt)/ `tensorrt`(.engine)。実際は拡張子優先・不一致は警告 | − | ✓ | |
| `imgsz` | int | px | 640 | 入力サイズ。.engine は**エクスポート時と一致必須** | ○ | ✓ | ✓ |
| `conf_threshold` | double | − | 0.25 | 検出信頼度しきい値 | ○ | | ✓ |
| `iou_threshold` | double | − | 0.45 | NMS IoU しきい値 | ○ | | ✓ |
| `max_det` | int | 件 | 20 | 1 フレーム最大検出数 | ○ | | |
| `classes` | int[] | − | `[]` | クラス ID フィルタ。空=全クラス | ○ | | |
| `use_half` | bool | − | true | PyTorch backend の FP16(CUDA のみ有効。.engine では無視) | − | ✓ | |
| `inference_hz` | double | Hz | 5.0 | 推論レート上限(超過フレームは破棄・キューしない)。0=毎フレーム | ○ | ✓ | ✓ |
| `debug_image_hz` | double | Hz | 2.0 | デバッグ画像レート | − | ✓ | |
| `publish_debug_image` | bool | − | true | false で描画処理を全省略 | − | ✓ | |
| `use_roi_crop` | bool | − | false | 画像 ROI クロップ有効化 | ○ | | ✓ |
| `roi_x_min` / `roi_x_max` / `roi_y_min` / `roi_y_max` | double | 分率 0–1 | 0.0 / 1.0 / 0.0 / 1.0 | ROI 範囲(空・自船体カット用)。bbox は全画像座標へ戻る | ○ | | ✓ |
| `draw_roi_rect` | bool | − | true | デバッグ画像に ROI 枠を描画 | − | | |
| `camera_topic` | string | − | `/camera/image_raw` | 入力画像 topic | − | ✓ | |
| `camera_info_topic` | string | − | `/camera/camera_info` | CameraInfo(現状ログ用途のみ) | − | | |
| `detections_topic` | string | − | `/yolo/detections` | 検出出力 topic | − | ✓ | |
| `buoy_roi_topic` | string | − | `/buoy_roi` | BuoyRoi 出力 topic | − | ✓ | |
| `roi_frame_id` | string | − | `base_link` | BuoyRoi header frame | − | | |
| `fixed_range_m` | double | m | 5.0 | 単眼のため固定レンジ提案(main.py と同一仕様) | ○ | | ✓ |
| `roi_range_half` | double | m | 2.0 | BuoyRoi.r_range(探索半幅) | ○ | | |
| `camera_fov_deg` | double | deg | 90.0 | 方位換算に使う水平 FOV | ○ | | ✓ |
| `roi_theta_min_deg` | double | deg | 2.0 | BuoyRoi.theta_range の下限 | − | | |
| `enable_color_estimation` | bool | − | true | bbox 内 HSV による色推定 | − | | ✓ |
| `class_names` | string[] | − | `[""]` | クラス ID→名前の上書き。`[""]`=モデル内蔵名を使用 | ○ | | ✓ |
| `buoy_class_names` | string[] | − | `[""]` | BuoyRoi 対象クラス名(小文字完全一致)。`[""]`=全クラス | ○ | | ✓ |
| `color_class_map` | string[] | − | red/green/yellow/black/white/blue/orange | クラス名に含まれる色キーワード(一致時 HSV 推定をスキップ) | − | | |

## 6. planner_node(パッケージ: asv_trajectory_planner)

設定ファイル: `src/navigation/path_generator/mppi/config/mppi_params.yaml`。
**全 `mppi.*` デフォルト = 旧ハードコード値**(変更時は値・日付・理由を YAML に記録する運用)。

### 6.1 MPPI ハイパーパラメータ(`mppi.*`)

| パラメータ名 | 型 | 単位 | デフォルト | 説明(コード対応) | 人が調整 | Jetson確認 | 実機確認 |
|---|---|---|---|---|---|---|---|
| `mppi.horizon` | int | step | 225 | 予測ホライズン(×dt=22.5 s 先読み) | ○ | ✓ | ✓ |
| `mppi.dt` | double | s | 0.1 | 予測刻み(`pred_dt`) | ○ | ✓ | |
| `mppi.num_samples` | int | 本 | 5000 | サンプル制御列数(`nb_sample`)。計算時間に直結 | ○ | ✓ | ✓ |
| `mppi.lambda` | double | − | 12.0 | MPPI 温度(`_lambda`) | ○ | | ✓ |
| `mppi.control_noise_sigma` | double[2] | [deg/s, m/s²] | [35.0, 0.0] | 制御ノイズ σ [舵角速度, 加速度] | ○ | | ✓ |
| `mppi.target_speed` | double | m/s | 1.0288888888888889 | 目標速度(2 kn = 2×1852/3600) | ○ | | ✓ |
| `mppi.path_cost_weight` | double | − | 150.0 | GPS 基準線からの逸脱コスト(`div_cost`) | ○ | | ✓ |
| `mppi.collision_cost_min` / `mppi.collision_cost_max` | double | − | 10.0 / 50.0 | CRM 衝突コストの再スケール範囲(`col_cost_min/max`)。単一重みは存在しない | ○ | | ✓ |
| `mppi.gate_cost_weight` | double | − | 3.0 | 仮想ゲート誘引コスト | ○ | | ✓ |
| `mppi.buoy_cost_weight` | double | − | 120.0 | 仮想ブイ回廊コスト | ○ | | ✓ |
| `mppi.speed_cost_weight` | double | − | 0.1 | 目標速度二乗誤差コスト | ○ | | |
| `mppi.control_cost_weight` | double | − | 0.2 | 入力逸脱コスト(`norm_cost`) | ○ | | |
| `mppi.loa` | double | m | 2.0 | CRM スケーリング用 LOA(自船/他船) | ○ | | ✓ |
| `mppi.safe_distance_right_loa` / `left` / `fore` / `aft` | double | ×LOA | 3.2 / 1.6 / 6.4 / 1.6 | CRM バンパ楕円ゲイン(crm_torch RIGHT/LEFT/FORE/AFT_AX_GAIN)。単一 safe distance は存在しない | ○ | | ✓ |
| `mppi.gate_half_width_m` | double | m | 4.0 | 仮想ゲート半幅(ブイ間隔=2 倍) | ○ | | ✓ |
| `mppi.buoy_margin_m` | double | m | 1.0 | 仮想ブイ外側マージン | ○ | | |
| `mppi.buoy_longitudinal_sigma_m` | double | m | 8.0 | ブイコストの縦方向ゲート σ | ○ | | |

### 6.2 planner のその他パラメータ

デフォルト列はノード宣言値。「実機 launch 値」は `planner_real.launch.py` が上書きする値。

| パラメータ名 | 型 | 単位 | デフォルト | 実機 launch 値 | 説明 | 人が調整 |
|---|---|---|---|---|---|---|
| `own_odom_topic` | string | − | `/own_ship/odom` | `/odom`(launch 引数) | 自船 odometry | − |
| `other_ship_twist_topic` | string | − | `/other_ship/twist` | 同左 | 他船速度入力 | − |
| `waypoint1_topic` / `waypoint2_topic` | string | − | `/waypoint1_pose` / `/waypoint2_pose` | 同左 | GPS 基準線 | − |
| `path_topic` | string | − | `/planned_path` | `/planned_path_pruned` | 出力 Path(pruner バイパス配線) | − |
| `own_frame` / `other_ship_frame` | string | − | `base_link` / `other_ship_base_link` | `base_link` / `opponent_vessel` | TF frame | − |
| `frame_id` | string | − | `map` | `map` | 出力 frame | − |
| `planning_frequency` | double | Hz | 2.0 | 2.0 | 計画ループ周期 | ○ |
| `point_spacing` | double | m | 0.5 | 0.5 | 経路点間隔 | − |
| `avoid_radius` / `avoid_offset` | double | m | 2.0 / 3.0 | 2.0 / 3.0 | (旧幾何回避系) | − |
| `require_other_ship` | bool | − | true | **false** | false: 他船なしでも直進 Path を出す(安全縮退) | − |
| `other_twist_is_relative` | bool | − | true | **false** | false: `/other_ship/twist` は map 絶対速度 | − |
| `opponent_use_distance_m` | double | m | 20.0 | 20.0 | 他船を考慮する距離 | ○ |
| `opponent_passed_margin_m` | double | m | 2.0 | 10.0 | 追い越し判定マージン | ○ |
| `reconnect_line_distance_m` | double | m | 1.0 | 1.0 | 基準線復帰判定 | ○ |
| `reconnect_ahead_length_m` | double | m | 8.0 | 5.0 | 復帰先読み長 | ○ |
| `straight_path_spacing_m` / `straight_path_length_m` | double | m | 2.0 / 60.0 | 2.0 / 60.0 | 直進 Path の間隔・長さ | − |
| `mppi_smoothing_window` | int | 点 | 5 | 3 | 出力経路平滑化窓 | ○ |

### 6.3 task2_waypoint_pose_publisher(同パッケージ)

| パラメータ名 | 型 | デフォルト | 説明 | 人が調整 |
|---|---|---|---|---|
| `config_package` / `config_file` / `config_key` | string | `waypoint_publisher` / `task2_waypoints.yaml` / `task2_config` | 共有 waypoint YAML の参照先(map 座標 x/y は事前計算済み・lat/lon はメタデータ) | −(YAML の中身は◎ 現地確定) |
| `waypoint1_topic` / `waypoint2_topic` | string | `/waypoint1_pose` / `/waypoint2_pose` | 出力 topic(start→wp1, goal→wp2) | − |
| `frame_id` | string | `map` | 出力 frame | − |
| `publish_frequency` | double [Hz] | 2.0 | 配信周期 | − |

## 7. task2_real.launch.py の引数(パッケージ: robot)

| 引数 | デフォルト | 説明 | 人が調整 |
|---|---|---|---|
| `enable_camera` | true | ZED2i 起動(real_bringup `enable_zed2i` へ) | ○ |
| `enable_yolo` | true | yolo11s.launch.py 起動。**venv 必須**(なければ `enable_yolo:=false`) | ○ |
| `enable_lidar` | true | Mid360 + task2_perception 起動 | ○ |
| `enable_ship_tracking` | true | サブモジュール classical_pipeline + opponent_selector 起動 | ○ |
| `enable_mppi` | true | planner_real.launch.py 起動 | ○ |
| `enable_nav2` | true | Task2 Nav2 スタック起動 | ○ |
| `enable_thrusters` | true | 三重ゲートの第 1 条件(単独ではスラスタは起動しない) | − |
| `enable_debug_topics` | false | diagnostics.launch.py(profile localization)起動 | ○ |
| `lidar_model` | `mid360s` | `mid360` \| `mid360s` | − |
| `yolo_backend` | `pytorch` | `pytorch`(.pt)\| `tensorrt`(.engine)。`use_tensorrt:=true` で強制上書き | ○ |
| `yolo_model_path` | `<yolo share>/config/best.pt` | 重みパス。**YOLO11s 重みで上書きすること(非同梱)** | ◎ |
| `use_tensorrt` | false | true で backend を `tensorrt` に強制 | ○ |
| `use_int8` | false | **予約・未使用**(INT8 パススルー用に宣言のみ) | − |
| `dry_run` | **true** | true の間 thruster_driver / serial_writer は決して起動しない | ◎(実出力時のみ false) |
| `send_thruster_commands` | **false** | 実出力には明示的に true が必要(dry_run:=false と併用) | ◎(実出力時のみ true) |

スラスタ起動条件: `enable_thrusters ∧ ¬dry_run ∧ send_thruster_commands`。

## 8. 参照値(read-only: 変更は今回のスコープ外)

### 8.1 thruster_driver(`src/driver/micon/thruster_driver/config/config.yaml`、test07089 と同一)

| パラメータ | 値 | 備考 |
|---|---|---|
| `input_mode` | `cmd_vel` | |
| `control.p.surge` / `sway` / `yaw` | 28.29 / 28.29 / 12.17 | P ゲイン |
| `max_surge_wrench` / `max_sway_wrench` / `max_yaw_wrench` | 6.0 / 6.0 / 3.0 N(·m) | wrench クランプ |
| `control.rate_hz` | 50.0 Hz | |
| `feedback_timeout_sec` / `stop_on_feedback_timeout` | 0.5 s / true | フェイルセーフ |
| `safety.watchdog_timeout_sec` | 0.5 s | |
| `thrusters.force_per_duty` | [40.0, 40.0, 40.0, 40.0] N | `/thruster_command` [N] → duty 換算基準 |
| `duty_resolution` | 1000 | |
| `static_map.wheels.forward_gain/reverse_gain` | 1.0 ×4 | mppi ブランチの 3.5 は**意図的に不採用** |

### 8.2 nav2_params_task2.yaml の主要値

| 項目 | 値 | 備考 |
|---|---|---|
| controller | RegulatedPurePursuitController | `use_collision_detection: false` |
| `desired_linear_vel` | 1.10 m/s | |
| `lookahead_dist`(min/max) | 3.0(1.2 / 6.0)m | velocity-scaled |
| velocity_smoother `max_velocity` | [1.20, 0.0, 0.18] | [m/s, m/s, rad/s] |
| velocity_smoother `min_velocity` | [0.0, 0.0, −0.18] | 後進なし |
| velocity_smoother `max_accel` / `max_decel` | [1.50, 0, 0.25] / [−1.50, 0, −0.25] | |
| local costmap | 30×30 m, 0.1 m, global_frame odom | rolling |
| global costmap | 100×60 m, 0.2 m, origin (−20, −30), global_frame map | |
| obstacle_layer sources | `pointcloud`(/pointcloud, clearing+marking) | 両 costmap 共通 |
| inflation_layer | cost_scaling_factor 2.5, inflation_radius 1.2 m | |
| goal_checker | xy 1.0 m, yaw 0.6 rad | |
| bt_navigator `odom_topic` | `/odometry/filtered/global` | |

### 8.3 サブモジュール pcl_segmentation(db84af9・無変更)

パラメータは各パッケージの `config/*.yaml`(preprocessing_params / segmentation_params / tracker_params)を参照。今回の統合では launch 引数 `lidar_topic:=/task2/points_filtered`、`ego_odom_topic:=/odometry/filtered/local` のみ指定し、**サブモジュール内の値は一切変更していない**。
