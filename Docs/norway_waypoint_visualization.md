# ノルウェー waypoint 可視化ガイド

ノルウェー会場を想定した Task 1〜3 の waypoint を、Foxglove 上で確認するための
**検討・可視化専用**の仕組みである。実機の Nav2 ゴール、GNSS 入力、推進指令は変更
しない。実海域で自律航行に使う値ではない。

## どの文書と実装を参照するか

- 座標の基準、Task ごとの配置意図: [norway_task_waypoint_geometry.md](norway_task_waypoint_geometry.md)
- 基準 GPS 1→2 の距離・方位の算出: [norway_waypoint_plan.md](norway_waypoint_plan.md)
- 実際に公開する点とトピック名: `src/gui/gnss_map_telemetry_test_publisher/src/norway_waypoint_publisher.cpp`

`norway_task_waypoint_proposal.md` は初期の運用検討案であり、現在の publisher と点名・
Task 構成が一致しない。可視化の確認にはこの文書と
`norway_task_waypoint_geometry.md` を正本とする。

## 座標系と基準線

開始点 GPS 1 は `63.4420936, 10.4242793`、終点 GPS 2/6 は
`63.4416044, 10.4238816` である。二点間は約 57.88 m、真方位は約 199.98° である。

waypoint は開始点を原点として、次のコース座標で定義する。

- `s`: GPS 1→2 の進行方向の距離 [m]
- `d`: 進行方向に対して左への横オフセット [m]。左が正、右が負

publisher はこの `(s, d)` を ENU に変換し、球面近似で `NavSatFix` の緯度・経度へ
変換している。`s, d` は現地測量値ではないため、GNSS の map 原点、方位、障害物・
安全域を確認せずに実船の目標へ流用してはならない。

## 表示されるコース

| Task | 可視化する点 | 意図 |
| --- | --- | --- |
| Task 1 | GPS 1→1.1〜1.10→GPS 2、GPS 3→3.1〜3.3→GPS 4 | ジグザグと S→N→S の方位標識通過案 |
| Task 2 | GPS 5、赤・緑のゲート端点と各中心、GPS 6 | 5 m 幅・20 m 間隔の二つのゲート。船の参照点は中心線で、回避は MPPI が担う |
| Task 3.1 | GPS 7、回廊、GPS 8 ゲート、接岸口・バース・離岸口 | 通常接岸の仮レイアウト |
| Task 3.2 | GPS 9、回廊、ゲート、接岸口・バース、GPS 10 | 別走行の点対称・平行接岸レイアウト |

ポイントは `/visualization/norway_waypoints/` 配下で公開する。たとえば
`task1/wp_1_1`、`task2/gate1_center`、`task3_1/berth` はそれぞれ Task 1 の
ジグザグ点、Task 2 の第1ゲート中心、Task 3.1 のバースを表す。全トピック名は
publisher の `waypoints` 配列を参照する。

## 起動と確認

ROS 2 ワークスペースで次を実行する。

```bash
colcon build --packages-select gnss_map_telemetry_test_publisher
source install/setup.bash
ros2 launch gnss_map_telemetry_test_publisher norway_waypoints.launch.py
```

既定では各点を `sensor_msgs/msg/NavSatFix` として 1 Hz で公開する。頻度は
`publish_rate_hz` で変更できる。

```bash
ros2 launch gnss_map_telemetry_test_publisher norway_waypoints.launch.py \
  publish_rate_hz:=2.0
```

Foxglove では GNSS Map Telemetry 拡張を読み込み、Map パネルで
`/visualization/norway_waypoints/*` を追加する。拡張は Task 1 を黄、Task 2 を赤、
Task 3.1 を水色、Task 3.2 を紫、開始点を白で表示する。publisher は
transient-local QoS と周期再送を使うため、Foxglove を後から接続しても点を取得できる。

## 実機へ移す前の手順

1. 現地でブイ、ゲート、桟橋、浅瀬、立入禁止域を測量する。
2. GNSS と `map` の原点・ENU 軸・ヨー角の定義を照合する。
3. 船体寸法、潮流、測位誤差を含む安全余裕を設定する。
4. 低速の手動監視試験で表示位置とログを確認する。
5. 承認済みの値だけを、別途 Nav2/経路計画の設定へ反映する。

この publisher 自体を実機航行の入力に接続しないこと。可視化用トピックと実機 GNSS
トピックは分離したままにする。
