# ブイ仮想壁のオフライン検証

水上試験の前に、赤・緑 lateral mark の仮想壁を次の順で確認する。対象は
`zed2i_driver` が publish する `/virtual_obstacles` (`sensor_msgs/msg/PointCloud2`) である。
Task 2 はこのトピックを costmap に接続しない設計なので、通常 Nav2 / Task 3 相当の
構成でのみ評価する。

## 1. 決定的な単体テスト

GPU、カメラ、ROS グラフなしで、壁の形状を確認できる。

```bash
cmake --build build/zed2i_driver --target test_perception_logic -j2
ctest --test-dir build/zed2i_driver --output-on-failure -R test_perception_logic
```

テストは次を検証する。

- 東向き航路で、赤ブイの壁が北（左外側）、緑ブイの壁が南（右外側）だけに出る。
- 同色ブイを航路方位順に接続する。
- `same_color_wall_max_gap_m` を超える間隔は接続しない。
- `channel_heading_rad` が未設定なら同色連結壁を作らない。
- 2個ずつの赤・緑ブイから、半円壁84点と連結壁10点を含む94点の
  `/virtual_obstacles` 点群が生成される。

## 2. 録画データの再生試験

実海域で取得済みのカメラ・TF・LiDAR rosbag がある場合は、同じ config で再生する。
`wall_frame=map` への TF と `channel_heading_rad` を与え、RViz で
`/virtual_obstacles` と costmap を重ねて確認する。

確認基準は次の通り。

- 赤・緑の間に航行可能な帯が残る。
- 各色列の外側は、半円と連結壁で連続している。
- 一時的な検出欠落では、最大間隔を越える誤接続をしない。
- 誤検出を入れた bag でも、別列・遠方の同色ブイへ壁をつながない。

## 3. シミュレーションと costmap

Task 3 simulator の既知のブイ配置を入力に使い、`/virtual_obstacles` を Nav2 が
購読する costmap に接続した構成で確認する。ゴールを航路内に置いた場合に、計画経路が
赤・緑列の間に残ること、航路外側へ抜ける候補が lethal cost で除外されることを確認する。

```bash
colcon build --packages-select foxglove_logger task3_sim
source install/setup.bash
ros2 launch task3_sim task3_sim.launch.py enable_diagnostics:=false
```

`task3_orchestrator` は `b31_*` を東向き、`b32_*` を西向きのコースとして扱い、
各コースの色列から `/virtual_obstacles` を map frame で 10 Hz に publish する。Task 3
の local/global costmap はこのトピックを `virtual_wall` obstacle source として購読する。
起動後は次で確認できる。

```bash
ros2 topic hz /virtual_obstacles
ros2 topic echo /virtual_obstacles --once
```

`enable_virtual_wall=true` だけでは不十分で、`channel_heading_rad` が有限値であり、
対象 costmap の obstacle layer が `/virtual_obstacles` を購読していることを必ず確認する。

## 4. 実装上の限界

この実装はフレーム内の色と位置だけで壁を作る。ブイ ID の追跡やゲートの赤緑ペアリングは
行わない。実機適用前に、時系列トラッキング、誤検出抑制、壁の有効期限、および
safe-stop 条件を別途実装・検証する。
