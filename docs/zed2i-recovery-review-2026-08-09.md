# ZED2i 復旧レビュー（2026-08-09）

対象は `65a031c fix(camera): bypass ZED startup self-calibration`。設定値
`disable_self_calibration: true` は `SdkNode` を経て
`sl::InitParameters::camera_disable_self_calib` に設定されるため、今回の
`POTENTIAL_CALIBRATION_ISSUE` 回避としては経路が通っている。

## 実機前に直す／確認する項目

1. **camera heartbeat の要求周波数が不一致**
   `jetson.yaml` は `/zed2i/left/image_rect` に 30 Hz を要求するが、標準の
   ZED 起動は 15 Hz で、README にある実測も約 13.7 Hz。現状ではカメラが正常でも
   health が異常になる。heartbeat を実測に合う値へ下げるか、ZED の実効周波数を
   30 Hz に揃えること。

2. **陸上映像の既定送信先が運用手順と矛盾**
   `jetson_bringup.launch.py` は `.local` ホスト名を既定値にする一方、README は
   `ground_video_host` に実 IP が必要としている。実機では受信側の有線 IP を明示して
   起動すること。既定値を残すなら README の制約も一致させること。

3. **自己キャリブレーションの切替を launch から指定できない**
   `zed2i.launch.py` は `disable_self_calibration` を宣言・転送していない。既定 YAML を
   編集すれば変更できるが、現地で `:=false` に戻して比較テストできない。launch 引数を
   追加して component parameter へ転送すること。

4. **回帰テストが現行 heartbeat 実装と不一致**
   `test_launch_roles.py` は廃止済みの `/heartbeat/driver/camera/front` と
   `/heartbeat/driver/lidar` を `heartbeat.launch.py` に期待して失敗する。役割別 inventory
   を読む現在の実装に合わせ、テストを更新すること。

## 最小実機確認

```shell
ros2 launch robot jetson_bringup.launch.py \
  ground_video_host:=<ground-pcの有線IPv4>
ros2 param get /zed2i/zed2i disable_self_calibration
ros2 topic hz /zed2i/left/image_rect
ros2 topic hz /zed2i/depth/image
ros2 node list | grep zed2i
```

期待値は、`disable_self_calibration` が `true`、ZED コンテナが 1 個だけ、画像・深度が
継続発行されること。heartbeat の修正前は 15 Hz 前後の画像レートを正常として扱わない。
