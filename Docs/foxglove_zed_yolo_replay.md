# FoxgloveでZED YOLOを確認する

記録時は、ZED GPU認識を有効にした状態で次をrecordする。`ros2 bag record`が購読するため、ZEDの購読者オンデマンド出力も有効になる。

```bash
ros2 bag record -o zed_yolo_$(date +%Y%m%d_%H%M%S) \
  /zed2i/left/image_rect /zed2i/left/camera_info \
  /zed2i/debug/detections_image /zed2i/points/detections \
  /tf /tf_static
```

再生はFoxgloveでMCAPを直接開く。`/zed2i/debug/detections_image`をImage panelに、`/zed2i/points/detections`を3D panelに追加する。前者にはboat BB・信頼度・ZED距離、後者には同じフレームのBB内だけのZED点群が出る。

認識率（recall/precision）を数値化するには正解アノテーションが必要で、BB表示だけからは算出できない。
