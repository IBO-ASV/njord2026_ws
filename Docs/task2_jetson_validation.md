# Task 2 Jetson/実機 段階検証手順 (task2-experiment)

> **履歴資料**: この手順内の `task2_real.launch.py` は削除済みです。実機の現在の入口は [`task2_launch_guide.md`](task2_launch_guide.md) の `task2_autonomy.launch.py` です。

- 作業日: 2026-07-19
- 前提: 本ブランチのコードは **Mac 上で静的確認のみ**。以下の 18 段階を順に実施し、各段階の合格条件を満たしてから次へ進む。
- 記法: `<ws>` = ワークスペースルート。ROS 2 環境は毎シェルで `source /opt/ros/<distro>/setup.bash && source install/setup.bash`。
- 段階 14 までは水上に出さない。段階 15 以降は係留または安全確保された水域で行う。
- 岸壁認識はTask 2の対象外となったため、feature/quay-perceptionブランチへ保存し、task2-experimentの認識・計画・制御ループから除外した。

---

## 段階 1: 環境確認

```bash
./scripts/task2/task2_jetson_preflight.sh ~/venvs/yolo11
```

- **確認**: スクリプトのサマリ出力(PASS/WARN 一覧)。
- **合格条件**: L4T / Ubuntu / ROS_DISTRO / CUDA / TensorRT / venv 内 torch(`cuda.is_available()==True`)/ numpy 1.26.x / cv_bridge / livox_ros_driver2 がすべて PASS。
- **失敗時**: `Docs/yolo11s_jetson_setup.md` に従い venv を再構築。torch が CPU 版になっていたら NVIDIA wheel を再インストール(`pip install torch` を実行していないか確認)。

## 段階 2: ビルドと静的検査

```bash
cd <ws>
colcon build --symlink-install --packages-up-to \
    task2_perception yolo asv_trajectory_planner robot task2_sim \
    ship_perception_bringup ship_perception_msgs
./scripts/task2/task2_static_check.sh
colcon test --packages-select robot task2_perception && colcon test-result --verbose
```

- **合格条件**: colcon build エラーなし(C++ サブモジュール含む)。static check の FAIL 0 件。pytest 40 件(perception 26 + launch 静的 14)全通過。
- **失敗時**: `ship_perception_msgs` を先にビルドしたか(`opponent_selector` の import 失敗は大抵これ)。rosdep 未解決(torch は pip 管理)。

## 段階 3: LiDAR 単体

```bash
ros2 launch robot task2_real.launch.py \
    enable_camera:=false enable_yolo:=false enable_ship_tracking:=false \
    enable_mppi:=false enable_nav2:=false
# 別シェル
ros2 topic hz /livox/lidar
ros2 topic echo /task2/points_filtered --field header.frame_id --once
```

- **確認 topic**: `/livox/lidar`(~10 Hz)、`/task2/points_filtered`(frame_id=`base_link`)、TF `base_link→livox_frame`。
- **合格条件**: `/task2/points_filtered` が入力と同程度のレートで出る。RViz(fixed frame `base_link`)で点群の**天地が正しい**(岸壁が上に伸びる、水面が z≈waterline)。
- **失敗時**: 天地逆・傾き → URDF `lidar_roll/pitch/yaw` を実測値に修正(ノード側の `lidar_inverted` は緊急用。恒久対応は URDF)。TF エラーのログ → robot_state_publisher 起動確認。

## 段階 4: 水面除去

```bash
# 段階 3 の構成に加え、cloud filter のデバッグ出力を有効化
ros2 param set /task2_cloud_filter publish_debug true   # または YAML を書き換えて再起動
ros2 topic echo /task2/debug/rejected_water --once
```

- **確認 topic**: `/task2/debug/water_removed`, `/task2/debug/rejected_water`, `/task2/debug/self_filtered`。
- **合格条件**: rejected_water が水面リターンのみを含み、岸壁・他船・デッキが water_removed 側に残る。自船リターンが self crop で消えている。
- **失敗時**: `waterline_z_m` を実測して設定(◎)。岸壁が消える → `water_plane_normal_z_min` を上げる。自船が残る → `self_crop_*` を実測拡大。**調整値は YAML に日付・理由付きで記録**。

## 段階 5: 他船追跡

```bash
ros2 launch robot task2_real.launch.py \
    enable_camera:=false enable_yolo:=false enable_mppi:=false enable_nav2:=false
# 別シェル(他船役のボートまたは移動物体を用意)
ros2 topic echo /tracked_objects
ros2 topic echo /other_ship/twist
ros2 run tf2_ros tf2_echo map opponent_vessel
```

- **確認 topic**: `/pcl/preprocessed` → `/pcl/nonground` → `/pcl/cluster_centroids` → `/tracked_objects` → `/other_ship/twist` + TF `map→opponent_vessel`。
- **合格条件**: 移動物体が CONFIRMED トラックになり、`/other_ship/twist` が **map 系の絶対対地速度**として妥当(静止物体で ≈ 0 m/s、移動物体で実速度と符号・大きさが一致)。物体を隠すと 2 s 以内に出力が沈黙する。
- **失敗時**: 速度の向き・大きさが不自然 → **ship_tracker の twist frame 規約(物体 body frame・相対)の実データ確認**が必要(既知の要確認事項)。`/tracked_objects` が空 → 段階 4 のフィルタで消えすぎていないか `/task2/points_filtered` を確認。ゲートで落ちる → `opponent_selector` の distance/size/point ゲートを緩める。

## 段階 6: 岸壁認識(対象外)

別Taskへ移行したため対象外。

## 段階 7: カメラ単体

```bash
ros2 launch robot task2_real.launch.py \
    enable_yolo:=false enable_lidar:=false enable_ship_tracking:=false \
    enable_mppi:=false enable_nav2:=false
ros2 topic hz /camera/image_raw
```

- **合格条件**: `/camera/image_raw` が安定配信(ZED2i)。
- **失敗時**: ZED SDK / USB 帯域を確認。**注**: ZED2i の CPU フォールバック vs USB カメラの選択は上流で未解決(既知の課題)。カメラを替える場合は `camera_topic` を合わせる。

## 段階 8: YOLO PyTorch backend

```bash
# 重みを配置してから (リポジトリ非同梱)
cp <学習済み>/yolo11s.pt src/detection/yolo/config/yolo11s.pt
colcon build --packages-select yolo
source ~/venvs/yolo11/bin/activate   # venv ガード必須
ros2 launch robot task2_real.launch.py \
    enable_lidar:=false enable_ship_tracking:=false \
    enable_mppi:=false enable_nav2:=false \
    yolo_model_path:=$(pwd)/src/detection/yolo/config/yolo11s.pt
# 別シェル
ros2 topic hz /yolo/detections
python3 scripts/task2/benchmark_yolo11s.py --model src/detection/yolo/config/yolo11s.pt --tegrastats
```

- **合格条件**: `/yolo/detections` が `inference_hz`(5 Hz)で出る。`yolo/debug_image` でブイ bbox と色ラベルが妥当。ベンチマークで実効 FPS ≥ inference_hz。
- **失敗時**: fatal「model file not found」→ 重みパス確認。ultralytics import 失敗 → venv 未活性(launch は venv ガードで即死する。venv なしなら `enable_yolo:=false`)。

## 段階 9: YOLO TensorRT FP16

```bash
source ~/venvs/yolo11/bin/activate
python3 scripts/yolo/export_yolo11s_tensorrt.py \
    --weights src/detection/yolo/config/yolo11s.pt --imgsz 640 \
    --output src/detection/yolo/config/yolo11s.engine
ros2 launch robot task2_real.launch.py ... use_tensorrt:=true \
    yolo_model_path:=$(pwd)/src/detection/yolo/config/yolo11s.engine
python3 scripts/task2/benchmark_yolo11s.py --model src/detection/yolo/config/yolo11s.engine --tegrastats
```

- **合格条件**: エクスポート成功(sidecar .json 生成)。検出結果が .pt と同等(同一シーンで比較)。FPS が .pt より向上。
- **失敗時**: エクスポートは **Jetson 上のみ**(CUDA なしでは拒否される)。imgsz 不一致に注意(engine は 640 固定)。JetPack/TensorRT 更新後は再エクスポート。

## 段階 10: YOLO INT8(任意)

```bash
python3 scripts/yolo/export_yolo11s_tensorrt.py \
    --weights src/detection/yolo/config/yolo11s.pt --imgsz 640 --int8 \
    --calib-data <キャリブレーション画像ディレクトリ> \
    --output src/detection/yolo/config/yolo11s_int8.engine
```

- **合格条件**: FP16 と比較して精度劣化が許容範囲(実映像で bbox/クラスを目視比較)であること。**キャリブレーションなし INT8 はスクリプトが拒否する(仕様)**。
- **注**: launch 引数 `use_int8` は**予約・未使用**。INT8 engine は `yolo_model_path` に直接渡す。

## 段階 11: MPPI 単体(ROS 不要ベンチ+ノード起動)

```bash
python3 scripts/task2/benchmark_mppi.py --device cuda --tegrastats   # Jetson CUDA
python3 scripts/task2/benchmark_mppi.py --device cpu -n 20           # 比較用
ros2 launch asv_trajectory_planner planner_real.launch.py
ros2 topic echo /waypoint1_pose --once
```

- **合格条件**: CUDA での 1 solve が `planning_frequency` 2 Hz に対し十分速い(< 500 ms、参考: Mac CPU ≈ 9.25 Hz ≈ 108 ms/solve)。`/waypoint1_pose` `/waypoint2_pose` が 2 Hz で出る。odom/waypoint 投入で `/planned_path_pruned` に Path が出る(他船なし → 直進)。
- **失敗時**: torch が CPU にフォールバックしていないか(`torch.cuda.is_available()`)。task2_waypoints.yaml が実験水域用に更新済みか。

## 段階 12: GLIM + LiDAR 統合

```bash
ros2 launch robot task2_real.launch.py \
    enable_camera:=false enable_yolo:=false enable_mppi:=false enable_nav2:=false
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odom /task2/points_filtered /tracked_objects
```

- **合格条件**: GLIM が TF `map→odom→base_link` を安定発行し、`/odom` がドリフトなく追従。**GLIM と task2_cloud_filter が同じ /livox/lidar を購読しても処理落ちしない**(CPU/RAM を tegrastats で確認)。EKF が TF を出していない(`ros2 run tf2_ros tf2_monitor` で発行者確認)。
- **失敗時**: TF 二重発行 → ekf_local/global の `publish_tf: false` を確認。GLIM 発散 → URDF LiDAR 取付角の実測(段階 3)へ戻る。

## 段階 13: GLIM + YOLO + MPPI + Nav2 同時(フルスタック・スラスタなし)

```bash
source ~/venvs/yolo11/bin/activate
ros2 launch robot task2_real.launch.py enable_debug_topics:=true \
    yolo_model_path:=<重みパス>
ros2 topic hz /planned_path_pruned /cmd_vel_thruster
```

- **合格条件**: 全ノード同時稼働で `/planned_path_pruned`(≈2 Hz)と `/cmd_vel_thruster` が連続して出る。tegrastats で CPU/GPU/RAM に余裕(サーマルスロットリングなし)。ノード落ちなし(30 分連続)。
- **失敗時**: リソース不足 → `inference_hz` 低減、`mppi.num_samples` 削減(値変更を YAML に記録)。topic 途絶 → `enable_debug_topics:=true` の diagnostics 出力で特定。

## 段階 14: dry-run 検証(三重ゲート)

既定値がすでに dry-run であることを確認する(**引数なし = 安全**):

```bash
ros2 launch robot task2_real.launch.py   # dry_run:=true, send_thruster_commands:=false (既定)
ros2 node list | grep -E "thruster_driver|serial_writer"   # → 何も出ないこと
ros2 topic hz /cmd_vel_thruster                            # → 出ること
ros2 topic hz /thruster_command                            # → 出ないこと
```

さらに三重ゲートの各辺を確認:

| 引数 | 期待 |
|---|---|
| (既定) | スラスタ系ノードなし |
| `dry_run:=false`(のみ) | スラスタ系ノードなし(send_thruster_commands が false) |
| `send_thruster_commands:=true`(のみ) | スラスタ系ノードなし(dry_run が true) |
| `dry_run:=false send_thruster_commands:=true enable_thrusters:=false` | スラスタ系ノードなし |
| `dry_run:=false send_thruster_commands:=true` | thruster_driver + serial_writer 起動 |

- **合格条件**: 上表どおり。dry-run 中は `/thruster_command` が存在せず ESP32 に通電コマンドが届かない。
- **失敗時**: launch の三重ゲート式を確認(静的テスト `test_thruster_condition_references_all_three_gates` が守っているはず — 差異があればコード改変を疑う)。

## 段階 15: 低出力スラスタ試験(陸上/係留)

```bash
# プロペラ周囲の安全確認・係留状態で
ros2 launch robot task2_real.launch.py \
    dry_run:=false send_thruster_commands:=true \
    enable_mppi:=false enable_nav2:=false      # 自動航行なし・手動確認から
ros2 topic echo /thruster_command
```

- **合格条件**: `/thruster_command`(Float32MultiArray、4 要素 [N]、FR/FL/RR/RL)が妥当な小値。E-stop(`/emg` およびハードウェア)で即停止。wrench クランプ(6/6/3)を超える指令が出ない。
- **失敗時**: serial_writer のポート(`/dev/serial/by-id/usb-Silicon_Labs_CP2102N…`)確認。feedback timeout(0.5 s)で停止するか確認。

## 段階 16: 実験水域での統合試験

```bash
source ~/venvs/yolo11/bin/activate
ros2 launch robot task2_real.launch.py \
    dry_run:=false send_thruster_commands:=true \
    yolo_model_path:=<重みパス> enable_debug_topics:=true
```

- 事前に: `task2_waypoints.yaml` を実験水域の GPS で再生成し、直進コース(他船なし)→ 他船ありの順で実施。
- **合格条件**: 他船なしで基準線を target_speed ≈ 1.03 m/s で直進。他船接近で MPPI が回避経路を生成し、CRM 楕円(右 3.2 LOA 等)を尊重した回避。他船喪失時に直進へ安全に復帰。
- **失敗時**: 即 E-stop → rosbag(段階 17)を解析。MPPI パラメータ調整はすべて `mppi_params.yaml` に記録。

## 段階 17: rosbag 記録

```bash
./scripts/task2/record_task2_bag.sh            # Task 2 主要 topic
./scripts/task2/record_task2_bag.sh --all      # 問題調査時は全 topic
```

- **合格条件**: 記録 topic(/livox/lidar, /odom, /task2/points_filtered, /tracked_objects, /other_ship/twist, /pcl/nonground, /planned_path_pruned, /cmd_vel_thruster, /thruster_command, /tf ほか)がすべて含まれ、ディスク残量警告が出ない。
- **注**: 既定リストにない topic の調査時は `--all` を使うこと。

## 段階 18: E-stop 総合確認

- 手順: 各動作状態(dry-run、係留低出力、水上)で ①ハードウェア E-stop ②ソフト E-stop(`/emg`)③ `ros2 launch` の Ctrl-C ④ serial 切断(USB 抜去)をそれぞれ試す。
- **合格条件**: いずれの場合もスラスタが 0.5 s(watchdog/feedback timeout)以内に停止。再投入で正常復帰。
- **失敗時**: これが通らない限り**自動航行試験を行わないこと**。

---

## 記録テンプレート

各段階の結果は以下を残す: 実施日 / 実施者 / commit SHA / 変更したパラメータ(値・理由)/ rosbag パス / 合否。パラメータ調整は該当 YAML のコメントにも転記する(`mppi_params.yaml` の「record any change」運用)。
