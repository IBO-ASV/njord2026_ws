# Task 2 変更ファイルマップ (task2-experiment)

> **履歴資料**: ファイル作成時点の記録です。記載される `task2_real.launch.py` は削除済みです。現在の起動構成は [`task2_launch_guide.md`](task2_launch_guide.md) を参照してください。

- 基準: `origin/test07089` tip `834f433` → `task2-experiment` tip `1647689`
- 取得コマンド: `git diff --name-status 834f433 HEAD`(計 74 ファイル、+10576 / −44 行)
- 区分: A=新規、M=変更。「対象」: 実機 / シミュ / 両方 / 開発(Mac 等の開発環境用)。
- サブモジュール `src/detection/pcl_segmentation` はポインタ `db84af9` のまま**変更なし**(diff に現れない)。
- 岸壁認識はTask 2の対象外となったため、feature/quay-perceptionブランチへ保存し、task2-experimentの認識・計画・制御ループから除外した。

---

## 1. 実機統合 launch・robot パッケージ

| パス | 状態 | 役割 | 変更理由 | 対象 | 依存パッケージ | 関連 launch / YAML |
|---|---|---|---|---|---|---|
| `src/robot/launch/task2_real.launch.py` | A | 実機 Task 2 統合エントリポイント。三重ゲート dry-run 実装 | 実機用統合 launch が未存在だった | 実機 | robot, yolo, task2_perception, ship_perception_bringup, asv_trajectory_planner | real_bringup / yolo11s / task2_perception / classical_pipeline / planner_real / navigation_launch_task2 / diagnostics |
| `src/robot/launch/navigation_launch_task2.py` | A | Task 2 専用 Nav2 起動(nav2_bringup navigation_launch.py 相当。velocity_smoother 出力を `/cmd_vel_thruster` にリマップ) | 実機用 `nav2.launch.py` を無変更に保つため分離 | 両方 | nav2_* 各種 | task2_real / task2_sim、`nav2_params_task2.yaml` |
| `src/robot/config/nav2_params_task2.yaml` | M | Task 2 用 Nav2 パラメータ | mppi ブランチ版へ更新(obstacle layer ソースは `pointcloud` のみ) | 両方 | nav2_costmap_2d ほか | navigation_launch_task2.py |
| `src/robot/urdf/robot.urdf.xacro` | M | ロボットモデル | `lidar_joint` に roll=π(公称・**要実測**)を設定 — 上下逆さま Mid360 補正の唯一の場所 | 両方 | robot_state_publisher | localization.launch.py |
| `src/robot/urdf/robot.urdf_modified.urdf` | M | 展開済み URDF(シミュが直接読む) | xacro と同じ roll=π を反映 | シミュ | robot_state_publisher | task2_sim.launch.py |
| `src/robot/CMakeLists.txt` | M | ビルド定義 | `test_task2_real_launch_static` を pytest テストとして登録 | 開発 | ament_cmake_pytest | − |
| `src/robot/test/test_task2_real_launch_static.py` | A | 静的テスト 14 件(launch 構文 / シミュ専用ノード非混入 / 安全既定値 / 三重ゲート式 / 岸壁認識の非再混入) | dry-run 安全性を CI 的に固定 | 開発 | pytest | task2_real.launch.py |

## 2. task2_perception(新規パッケージ)

| パス | 状態 | 役割 | 対象 | 依存 | 関連 |
|---|---|---|---|---|---|
| `src/detection/task2_perception/package.xml` / `setup.py` / `setup.cfg` / `resource/task2_perception` / `task2_perception/__init__.py` | A | パッケージ定義(ament_python)。2 executable を登録 | 実機 | rclpy, sensor_msgs, tf2_ros, ship_perception_msgs(サブモジュール) | task2_perception.launch.py |
| `src/detection/task2_perception/README.md` | A | パッケージ設計・チェーン・要実測パラメータの説明 | − | − | − |
| `src/detection/task2_perception/config/task2_params.yaml` | A | Task 2全ノードのパラメータ（用途・単位コメント付き） | 実機 | − | Task 2 launch群 |
| `src/detection/task2_perception/launch/task2_perception.launch.py` | A | 2 ノード起動(enable_cloud_filter / enable_opponent_selector) | 実機 | − | task2_real.launch.py から include |
| `src/detection/task2_perception/task2_perception/cloud_filter_node.py` | A | `task2_cloud_filter`: /livox/lidar → TF 変換+フィルタ → /task2/points_filtered | 実機 | tf2_ros, sensor_msgs_py | 上記 YAML |
| `src/detection/task2_perception/task2_perception/opponent_selector_node.py` | A | `opponent_selector`: /tracked_objects → ゲート/選択/ego 補償/平滑化 → /other_ship/twist + TF | 実機 | ship_perception_msgs | 上記 YAML |
| `src/detection/task2_perception/task2_perception/cloud_ops.py` | A | 点群操作の純 numpy モジュール(変換・クロップ・水面除去 RANSAC 等) | 実機 | numpy のみ | pytest |
| `src/detection/task2_perception/task2_perception/tracking_glue.py` | A | Track データクラス・body→base 回転・ego 補償・CPA/TCPA・選択ロジック(純 Python) | 実機 | numpy のみ | pytest |
| `src/detection/task2_perception/task2_perception/smoothing.py` | A | TwistSmoother(IIR ローパス+スパイク棄却。opponent_twist_from_tf_node の移植) | 実機 | − | pytest |
| `src/detection/task2_perception/test/`(`__init__.py`, `conftest.py`, `test_cloud_ops.py`, `test_tracking_glue.py`) | A | 合成データによる pytest 26 件(ROS 不要、Mac で全通過) | 開発 | pytest, numpy | task2_static_check.sh |

## 3. YOLO11s(既存 yolo パッケージへ並置追加)

| パス | 状態 | 役割 | 対象 | 依存 | 関連 |
|---|---|---|---|---|---|
| `src/detection/yolo/yolo/yolo11_node.py` | A | `yolo11_node`: 標準 Ultralytics による YOLO11s 検出ノード(既存 main.py は無変更)。.pt / .engine 両対応、重み欠如で fatal 終了 | 実機 | ultralytics, cv_bridge, vision_msgs, njord_interfaces | yolo11s.launch.py, yolo11s_params.yaml |
| `src/detection/yolo/config/yolo11s_params.yaml` | A | yolo11_node 全パラメータ([TUNE] 注記付き) | 実機 | − | yolo11s.launch.py |
| `src/detection/yolo/launch/yolo11s.launch.py` | A | yolo11_node 起動(venv ガード付き) | 実機 | − | task2_real.launch.py から include |
| `src/detection/yolo/requirements_jetson_yolo11.txt` | A | Jetson venv 用最小パッケージ(ultralytics==8.3.237, numpy<2。torch は NVIDIA wheel を別途) | 実機(Jetson) | − | Docs/yolo11s_jetson_setup.md |
| `src/detection/yolo/package.xml` | M | 依存宣言追加(vision_msgs 等) | 実機 | − | − |
| `src/detection/yolo/setup.py` | M | entry point `yolo11_node = yolo.yolo11_node:main` 追加 | 実機 | − | − |

## 4. MPPI(asv_trajectory_planner、mppi ブランチ由来+今回の追加)

| パス | 状態 | 役割 | 対象 | 関連 |
|---|---|---|---|---|
| `src/navigation/path_generator/mppi/package.xml` / `setup.py` / `setup.cfg` / `resource/` / `asv_trajectory_planner/__init__.py` | A | パッケージ定義(mppi マージで取込+依存宣言追加) | 両方 | − |
| `.../asv_trajectory_planner/planner_node.py` | A | MPPI プランナノード。**今回 `mppi.*` パラメータ化(デフォルト=旧ハードコード値)** | 両方 | mppi_params.yaml |
| `.../asv_trajectory_planner/mppi_torch.py` | A | MPPI コア(torch)。パラメータ注入対応(デフォルト挙動不変を 25/25 テストで確認) | 両方 | − |
| `.../asv_trajectory_planner/crm_torch.py` | A | CRM 衝突リスクモデル(バンパ楕円) | 両方 | − |
| `.../asv_trajectory_planner/trajectory_generator.py` | A | planner_node と MPPIPlanner の仲介(直進経路・復帰・平滑化) | 両方 | − |
| `.../asv_trajectory_planner/vessel_state.py` | A | 船体状態データクラス | 両方 | − |
| `.../asv_trajectory_planner/task2_waypoint_pose_publisher.py` | A | **新規**: 実機用 waypoint 供給(共有 task2_waypoints.yaml の start/goal を 2 Hz 配信) | 実機 | planner_real.launch.py |
| `.../asv_trajectory_planner/path_pruner_node.py` | A | 経路プルーナ(現配線では実質バイパス) | 両方 | − |
| `.../asv_trajectory_planner/follow_path_client_node.py` | A | Nav2 FollowPath action クライアント(1 Hz 再送) | 両方 | − |
| `.../asv_trajectory_planner/opponent_twist_from_tf_node.py` | A | **シミュ専用** TF→twist ブリッジ | シミュ | planner_with_follow_path.launch.py |
| `.../asv_trajectory_planner/task2_gps_waypoint_publisher.py` | A | **シミュ専用** /sim/task2_gps_markers→waypoint | シミュ | 同上 |
| `.../asv_trajectory_planner/my_planning_algorithm.py` | A | 未接続の代替案(幾何迂回)。どの launch からも起動されない | − | − |
| `.../config/mppi_params.yaml` | A | **新規**: MPPI 全ハイパーパラメータ(デフォルト=旧ハードコード値。spec 名→コード名対応表付き) | 実機 | planner_real.launch.py |
| `.../launch/planner_real.launch.py` | A | **新規**: 実機用 planner 系 launch(シミュブリッジなし) | 実機 | task2_real.launch.py から include |
| `.../launch/planner_with_follow_path.launch.py` | A | シミュ用 planner 系 launch(ブリッジ 2 ノード込み) | シミュ | task2_sim.launch.py から include |
| `.../test/test_copyright.py` / `test_flake8.py` / `test_pep257.py` | A | ament 標準 lint テスト雛形 | 開発 | − |

## 5. waypoint_publisher

| パス | 状態 | 役割 | 対象 | 関連 |
|---|---|---|---|---|
| `src/navigation/path_generator/waypoint_publisher/waypoint_publisher/task2_gate_midpoint_publisher.py` | A | ゲート中点 publisher(mppi ブランチ由来の**未接続**代替案) | − | − |
| `src/navigation/path_generator/waypoint_publisher/setup.py` | M | 上記 entry point 追加 | − | − |

## 6. シミュレーション(task2_sim)

| パス | 状態 | 役割 | 変更理由 | 対象 |
|---|---|---|---|---|
| `src/sim/task2_sim/launch/task2_sim.launch.py` | M | シミュ統合 launch | アダプタ追加・`topic_thruster_command:=/sim/thruster_duty`・Nav2 を navigation_launch_task2.py 直接 include(MPPI 統合時の変更のまま) | シミュ |
| `src/sim/task2_sim/task2_sim/sim_thruster_command_adapter.py` | A | /thruster_command [N] → /sim/thruster_duty [duty] 変換(**シミュ専用**) | 実機コマンド形式とシミュ動力学の型不一致解消 | シミュ |
| `src/sim/task2_sim/task2_sim/opponent_vessel.py` | M | 相手船シミュ | mppi ブランチの更新(初期位置・運動設定) | シミュ |
| `src/sim/task2_sim/config/task2_opponent_sim.yaml` | M | 相手船シナリオ設定 | mppi ブランチの更新 | シミュ |
| `src/sim/task2_sim/scripts/set_opponent_initial_from_collision_point.py` | A | 衝突点から相手船初期値を逆算するツール | mppi ブランチ由来 | シミュ(ツール) |
| `src/sim/task2_sim/setup.py` / `package.xml` | M | entry point・exec_depend(asv_trajectory_planner)追加 | − | シミュ |

## 7. スクリプト・ドキュメント・その他

| パス | 状態 | 役割 | 対象 |
|---|---|---|---|
| `scripts/task2/task2_jetson_preflight.sh` | A | Jetson 環境検査(L4T/ROS/CUDA/TensorRT/Python スタック/tegrastats。**nvidia-smi は Jetson に無いため tegrastats を使用**) | 実機(Jetson) |
| `scripts/task2/task2_static_check.sh` | A | ROS 不要の静的検査一括実行(py_compile / YAML / XML / pytest) | 開発 |
| `scripts/task2/benchmark_mppi.py` | A | ROS 不要の MPPI ソルバベンチマーク(実構成 horizon 225×5000 samples。Mac CPU 実測 ≈9.25 Hz) | 開発+Jetson |
| `scripts/task2/benchmark_yolo11s.py` | A | YOLO11s 推論ベンチマーク(--tegrastats 対応) | 実機(Jetson) |
| `scripts/task2/record_task2_bag.sh` | A | Task 2 関連 topic の rosbag 記録(--all で全 topic) | 実機 |
| `scripts/yolo/export_yolo11s_tensorrt.py` | A | .pt → .engine エクスポート(**Jetson 上でのみ実行**。FP16 既定、INT8 は --calib-data 必須。環境情報を sidecar JSON に記録) | 実機(Jetson) |
| `Docs/task2_mppi_integration_report.md` | A | MPPI 統合レポート(前段作業の記録) | − |
| `Docs/yolo11s_jetson_setup.md` | A | Jetson venv 戦略・重み配置・エクスポート手順 | − |
| `.gitignore` | M | 両ブランチの ignore 統合(build/ install/ log/, rosbag2*, *.bak*) | 開発 |
