# Task 2 MPPI 統合レポート (task2-experiment)

- 作業日: 2026-07-19
- 作業環境: Mac (ROS 2 環境なし。静的確認のみ。push は未実施)
- 参照: `task2_repository_investigation.md` (2026-07-18 調査報告)

---

## 1. 作成したブランチ

| ブランチ | 起点 |
|---|---|
| `task2-experiment` | `origin/test07089` (tip `834f433`) |

## 2. マージしたブランチ

`origin/mppi` (tip `c5163a9`) を `task2-experiment` へマージ (マージコミット `17ed890`)。

## 3. 発生した競合

Git 上のテキスト競合は **`.gitignore` のみ**。
ただし自動マージで以下 2 件の**意味的な退行**が混入したため、手動で test07089 側へ戻した:

1. `src/driver/micon/thruster_driver/config/config.yaml` — mppi 側の `static_map.wheels.forward_gain/reverse_gain = 3.5` が自動マージで混入
2. `src/robot/launch/nav2.launch.py` — mppi 側の「無条件で `navigation_launch_task2.py` を include する」変更が自動マージで混入(実機 bringup が常に Task 2 用 Nav2 になってしまう)

また、mppi ブランチに混入していた Task 2 と無関係のゴミを除外した:

- `src/driver/camera/zed-ros2-wrapper` / `zed-ros2-wrapper_backup` — `.gitmodules` に登録のない野良 gitlink(サブモジュール参照だけがコミットされた状態)
- `src/driver/camera/zed-ros2-wrapper-foxy-humble-v3.8.2_backup/` — ベンダリングされたバックアップツリー(117 ファイル)

## 4. 競合の解消方法

| ファイル | 解消 |
|---|---|
| `.gitignore` | 両ブランチの内容を統合(`build/ install/ log/`, `rosbag2*`, `*.bak*` すべて保持) |
| `thruster_driver/config/config.yaml` | `git checkout origin/test07089 --` で test07089 版に復元(gain 1.0 のまま) |
| `src/robot/launch/nav2.launch.py` | test07089 版に復元。Task 2 用 Nav2 は `task2_sim.launch.py` 側から `navigation_launch_task2.py` を直接 include する方式に変更 |
| ZED 野良 gitlink・backup ツリー | マージ時に削除(`git rm --cached`) |
| `task2_sim.launch.py` | 自動マージ成功(mppi の構成 + test07089 の `transport_mode` 削除が両立)。その後 §8, §9 の修正を追加 |

## 5. test07089 側を維持したファイル(実機系 — mppi の旧実装へ戻していない)

- `src/driver/micon/thruster_driver/` 一式(`Float32MultiArray` ニュートン出力・`config.yaml` ゲイン)
- `src/driver/micon/micon_driver_fd/`(`serial_writer`、ESP32 USB シリアル、E-stop)
- `src/robot/launch/real_bringup.launch.py`(変更なし・シミュノード混入なし)
- `src/robot/launch/nav2.launch.py`(標準 `nav2_bringup/navigation_launch.py` を使う実機構成)
- `src/robot/config/nav2_params.yaml` ほか実機 Nav2 設定
- GLIM / Livox / INS / GNSS / `/dev/serial/by-id/` 関連の設定一式
- `.gitmodules` とサブモジュールポインタ 5 件すべて(`pcl_segmentation` = `db84af9` 等、変更なし)

確認: `git diff origin/test07089 HEAD -- src/robot src/driver` の差分は
`nav2_params_task2.yaml` と `navigation_launch_task2.py` の**追加 2 件のみ**。

## 6. mppi 側から取り込んだファイル

- `src/navigation/path_generator/mppi/` 一式(新規パッケージ `asv_trajectory_planner`)
  - MPPI 計算部: `mppi_torch.py`, `crm_torch.py`, `trajectory_generator.py`, `vessel_state.py`
  - ノード: `planner_node.py`, `path_pruner_node.py`, `follow_path_client_node.py`
  - シミュ専用ノード: `opponent_twist_from_tf_node.py`, `task2_gps_waypoint_publisher.py`
  - 未接続の代替案: `my_planning_algorithm.py`
  - `launch/planner_with_follow_path.launch.py`
- `src/robot/launch/navigation_launch_task2.py`(新規)
- `src/robot/config/nav2_params_task2.yaml`(更新)
- `src/sim/task2_sim/`: `task2_opponent_sim.yaml`, `opponent_vessel.py`, `task2_sim.launch.py`, `scripts/set_opponent_initial_from_collision_point.py`
- `src/navigation/path_generator/waypoint_publisher/`: `task2_gate_midpoint_publisher.py` + setup.py エントリ
  (GPS アンカー済み `task2_waypoints.yaml` は test07089/mppi 両方に取込済のため差分なしでそのまま有効)

## 7. 追加または修正したファイル(マージ後の新規変更)

| ファイル | 内容 |
|---|---|
| `src/sim/task2_sim/task2_sim/sim_thruster_command_adapter.py` | **新規**・シミュ専用変換ノード(§8) |
| `src/sim/task2_sim/setup.py` | 上記のエントリポイント追加 |
| `src/sim/task2_sim/launch/task2_sim.launch.py` | ①アダプタノード追加 ②`dutyed_tf_pub_with_disturbance` の `topic_thruster_command` を `/sim/thruster_duty` に変更(パラメータのみ・ノード本体無改造) ③Nav2 include を `robot/nav2.launch.py` → `navigation_launch_task2.py` 直接に変更 |
| `src/navigation/path_generator/mppi/package.xml` | 不足依存の宣言追加(`visualization_msgs`, `python3-numpy`) |
| `src/sim/task2_sim/package.xml` | `asv_trajectory_planner` への exec_depend 追加 |
| `.gitignore` | 両ブランチ統合 |

MPPI アルゴリズム本体(ホライズン・サンプル数・コスト重み・CRM・目標速度・ノイズ)は**一切変更していない**。

## 8. スラスタ指令形式への対応方法

**選択: 方法2「シミュレーション専用変換ノードを追加」**

- test07089 の `thruster_driver` は `/thruster_command` を `Float32MultiArray`(4 要素・ニュートン)で出力
- シミュレータ `dutyed_tf_pub_with_disturbance` は `Int16MultiArray`(duty カウント)を要求
- 新ノード `sim_thruster_command_adapter`(task2_sim パッケージ)が
  `/thruster_command` [N] → `duty = round(N / force_per_duty × duty_resolution)` → `/sim/thruster_duty` [Int16] に変換
  (`force_per_duty = 40.0`、`duty_resolution = 1000` は実機 config / シミュ config と一致させたパラメータ)

**理由**: 既存コード変更が最小(実機側 0 行、シミュ動力学ノード 0 行。変更は launch のパラメータ 1 個と新規ファイルのみ)。方法1 はシミュ C++ ノードの改修が必要、方法3 は型不一致自体を解決しない。将来実機で不要なノードが混ざる余地もない(task2_sim パッケージ内・sim launch のみで起動)。

実機チェーンは無変更で維持:
```
Nav2 controller → cmd_vel → velocity_smoother → /cmd_vel_thruster
→ thruster_driver (test07089) → /thruster_command (Float32MultiArray, N)
→ serial_writer → ESP32
```
シミュのみ末尾が分岐:
```
… → /thruster_command → sim_thruster_command_adapter → /sim/thruster_duty (Int16MultiArray)
→ dutyed_tf_pub_with_disturbance → /odom + TF
```

## 9. シミュレーション用と実機用の分離方法

- シミュ専用ノード(`opponent_twist_from_tf_node`, `task2_gps_waypoint_publisher`, `opponent_vessel_node`, `ideal_lidar_pointcloud_node`, `dutyed_tf_pub_with_disturbance`, `sim_thruster_command_adapter`)はすべて `task2_sim.launch.py` / `planner_with_follow_path.launch.py` からのみ起動。`real_bringup.launch.py` は無変更(grep で混入なしを確認)
- `robot/nav2.launch.py` は実機用のまま。Task 2 用 Nav2 (`navigation_launch_task2.py` + `nav2_params_task2.yaml`) はシミュ launch が明示的に include
- 他船認識入力は現状シミュの TF ブリッジ(`opponent_twist_from_tf_node`)。`planner_node` の入力はトピック `/other_ship/twist` + TF `map→opponent_vessel` に閉じているため、後日 LiDAR 認識ノードが同じインターフェースを出力すれば無改造で差し替え可能
- 注意: `planner_with_follow_path.launch.py` はシミュブリッジ 2 ノードを内包している。**実機接続時はこの launch をそのまま使わず**、planner/pruner/follow_path_client のみを起動する実機用 launch を別途作る(次段作業)

## 10. ビルドまたは構文確認結果

Mac に ROS 2 / colcon が無いため `colcon build` は未実施。静的確認の結果:

| 確認 | 結果 |
|---|---|
| Python 構文 (`py_compile`、変更・取込全 .py) | ✅ すべて OK |
| launch ファイル AST 確認(`generate_launch_description` 存在) | ✅ 5 ファイル OK |
| YAML 構文(nav2_params_task2 ほか 5 件) | ✅ OK |
| package.xml XML 妥当性 | ✅ OK(不足依存 2 件は追加済) |
| ROS 2 パッケージ名重複 | ✅ 重複なし |
| MPPI コア import(torch 2.7.0 + numpy で実 import) | ✅ OK(ROS 非依存を確認) |
| トピック/型の静的整合(§8 チェーン、`/odom`, `/pointcloud`, `/sim/task2_gps_markers`, `/other_ship/twist`, FollowPath action) | ✅ 発行側と購読側が一致 |

**環境起因で未確認(Jetson/Ubuntu で要実施)**: `colcon build`(C++ パッケージ含む)、`ros2 launch` の実行時解決、rosdep 解決(torch は rosdep キーなし・pip 管理)、CUDA での MPPI 実行。**コード起因の問題は検出なし**。

## 11. Task 2 シミュレーション起動方法(Jetson / Ubuntu ROS 2 環境)

```bash
cd <ws>
colcon build --symlink-install --packages-up-to task2_sim asv_trajectory_planner robot
source install/setup.bash

# 統合launch一発起動(Nav2 + MPPI + シミュ動力学 + 相手船を内部タイマで順次起動)
ros2 launch task2_sim task2_sim.launch.py
```

- 一つの統合 launch で起動可能。内部の `TimerAction` が
  ①動力学+アダプタ+thruster_driver+EKF (0s) → ②Task2 Nav2 (2s) → ③MPPI プランナ系 (3s) → ④相手船+擬似 LiDAR (10s) の順で起動する
- 主な引数: `use_nav2`, `use_mppi`, `use_dynamics`(default true)、`opponent_delay` など
- 動作確認ポイント: `/planned_path_pruned` に Path が出る → `/cmd_vel_thruster` → `/thruster_command`(Float32, N)→ `/sim/thruster_duty`(Int16)→ `/odom` が動く

## 12. Jetson 上で確認が必要な項目

1. `colcon build`(特に C++: thruster_driver, micon_driver_fd, dutyed_tf_pub_with_disturbance)と `ros2 launch task2_sim task2_sim.launch.py` の実行
2. torch の Jetson 用 wheel で MPPI(サンプル 5000 × horizon 225)の 1 ステップ計算時間、CUDA 自動選択の動作
3. `nav2_params_task2.yaml` の速度上限(max_velocity [1.20, 0, 0.18])・controller ゲインが test07089 実機ゲイン(P: 28.29/12.17, wrench clamp 6/6/3N)と整合するか実挙動で確認
4. アダプタ変換値の妥当性(duty ±1000 クランプに張り付かないか、`/sim/thruster_duty` を echo で確認)
5. 実機構成へのリグレッションがないこと: `real_bringup.launch.py` 起動 → serial_writer/ESP32 疎通・E-stop 動作(コード無変更だが最終確認)
6. ROS 2 ディストリビューションの確定(調査報告 §15-1 のまま未確定)

## 13. 次の作業へ引き継ぐ注意事項

- **MPPI パラメータはすべてハードコードのまま**(horizon 225×0.1s, サンプル 5000, λ=12, σ=[35,0], 目標速度 2kn)。ROS パラメータ化は次段作業
- **実機用 Task 2 launch は未作成**。`planner_with_follow_path.launch.py` はシミュブリッジ 2 ノード込みなので、実機では planner/pruner/follow_path_client のみを起動する launch を新設し、`/odom` は GLIM、waypoint は `waypoint_publisher`(GPS アンカー済 task2_waypoints.yaml)、`/other_ship/twist` は LiDAR 認識グルー(未実装)に接続する
- **YOLO 軽量化・LiDAR 他船認識・岸壁認識は未実装・未着手**(今回のスコープ外。関連コード無変更)
- **pcl_segmentation サブモジュールは test07089 のポインタ(db84af9)のまま**。輝度ブイ/YOLOfusion 系(0cdf3df)との統合はサブモジュール側で別途必要
- mppi ブランチ由来の `my_planning_algorithm.py`(幾何迂回)と `task2_gate_midpoint_publisher.py`(ゲート中点)は取り込んだが**どの launch にも接続されていない**別系統の代替案
- mppi の thruster gain 3.5 は意図的に捨てた(旧 duty 構成向けの値)。シミュで推力不足に見えた場合はこの経緯を思い出すこと
- GitHub へは push していない。push 前にユーザー確認が必要
