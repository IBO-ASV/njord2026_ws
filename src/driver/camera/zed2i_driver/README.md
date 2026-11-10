# zed2i_driver

ZED 2i カメラ用ドライバ。以下の2モードを提供する。

- `sdk`(既定): ZED SDK + CUDA を使用する本線モード。
- `cpu`: OpenCV ステレオマッチングによるフォールバックモード。

## 起動方法

SDK モード(既定):

```
ros2 launch zed2i_driver zed2i.launch.py
```

CPU モード:

```
ros2 launch zed2i_driver zed2i.launch.py mode:=cpu
```

パラメータファイルを差し替える場合:

```
ros2 launch zed2i_driver zed2i.launch.py params_file:=/path/to.yaml
```

既定のパラメータファイルは `config/zed2i_jetson_orin_nano.yaml`。

SDK/CPUノードはいずれもROS 2 componentとして登録され、上記launchでは
`component_container_mt`内に`use_intra_process_comms=true`でロードされる。
大容量のImage/PointCloud2は`UniquePtr`でpublishするため、同じcontainerへC++の
購読componentを追加した場合はintra-process通信を利用できる。Pythonノードなど
container外の購読者との通信は従来どおりDDS経由になる。

## 露出・ゲイン（ZED SDKモード）

SDKモードでは、ZED SDKの自動露出・自動ゲイン（AEC/AGC）を使う。既定では画面下半分を
測光ROIにするため、水面を優先して調整し、空などによるブイ周辺の白飛びを抑える。
起動ログに `ZED automatic exposure/gain enabled with ROI ...` と出れば設定済みである。

`config/zed2i_jetson_orin_nano.yaml` の以下は、すべて左画像に対する0.0--1.0の比率である。

- `aec_agc_enable`: 自動露出・自動ゲインの有効化（既定 `true`）
- `aec_agc_roi_enable`: ROI測光の有効化（既定 `true`）
- `aec_agc_roi_x_ratio` / `aec_agc_roi_y_ratio`: ROI左上（既定 `0.0` / `0.5`）
- `aec_agc_roi_width_ratio` / `aec_agc_roi_height_ratio`: ROIサイズ（既定 `1.0` / `0.5`）

例えば水面の位置がさらに下寄りなら、`aec_agc_roi_y_ratio: 0.6` と
`aec_agc_roi_height_ratio: 0.4` にする。手動露出に切り替える必要があるときだけ
`aec_agc_enable: false` とする。

全画面測光（比較用）は、設定ファイルを編集せず次で起動できる。

```shell
ros2 launch zed2i_driver zed2i.launch.py mode:=sdk aec_agc_roi_enable:=false
```

下半分測光（既定）は次である。

```shell
ros2 launch zed2i_driver zed2i.launch.py mode:=sdk \
  aec_agc_roi_y_ratio:=0.5 aec_agc_roi_height_ratio:=0.5
```

## 陸上映像伝送(ground video)の正しい起動手順

Image トピックを DDS で流すと 1 本あたり約 442 Mbps になるため、陸上 PC で映像を見る場合は
GPU JPEG → RTP/UDP の専用経路を使う。送信は本パッケージの `src/ground_video_streamer.cu`
(Jetson)、受信は `launch/ground_video_receiver.launch.py`(陸上 PC)。

### 0. ソースを更新したら必ず再ビルドする(最重要)

```shell
# Jetson 上で
cd ~/njord2026_ws
source /opt/ros/$ROS_DISTRO/setup.bash   # Jetson 実機は jazzy
colcon build --symlink-install --packages-select zed2i_driver
source install/setup.bash
```

再ビルドを忘れると**古い `.so` がロードされ**、
`Ground-video streaming disabled: nvjpegCreateSimple failed (nvJPEG status 6)`
で映像が出ない。これは解決済みの旧実装(libnvjpeg 版)が残っているだけで、
ハード・ドライバの故障ではない。判別:

```shell
ls -l --time-style=long-iso ~/njord2026_ws/build/zed2i_driver/libzed2i_sdk_component.so
strings ~/njord2026_ws/build/zed2i_driver/libzed2i_sdk_component.so | grep -c nvjpegenc  # 1 なら新実装
```

### 1. 受信側(陸上 PC)を先に起動する。ポートごとに 1 プロセスだけ

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5600   # ZED 2i left
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5601   # back cam(別端末)
```

引数は `port`(既定 `5600`) と `jitter_latency_ms`(既定 `50`)。

同じポートを二重に掴んではいけない。UDP unicast は `SO_REUSEADDR` で bind しても
片方のソケットにしか配送されず、後から起動した側が奪って**先の受信ウィンドウが止まる**。

```shell
ss -lunp | grep -E '5600|5601'   # 各ポート 1 行だけであること
```

### 2. 受信側の実 IP を確認して `ground_video_host` に渡す

```shell
ip -4 -o addr show scope global | awk '{print $2, $4}'
```

Jetson と共有しているサブネットの IP を選ぶ(有線 `192.168.1.x` 系が本線)。
ホスト名や `localhost` は不可。既定の空文字のままでは送信が有効にならない。

### 3. 送信側(Jetson)を起動する

```shell
ros2 launch zed2i_driver zed2i.launch.py \
  enable_ground_video:=true \
  ground_video_host:=192.168.1.2 \
  ground_video_port:=5600 \
  ground_video_fps:=3.0
```

bringup 経由の場合は `ros2 launch robot jetson_bringup.launch.py enable_ground_video:=true
ground_video_host:=<陸上 PC の IP> ground_video_port:=5600`。fps と解像度も
`ground_video_width` / `ground_video_height` / `ground_video_fps` で上書きできる。
JPEG 品質は `config/zed2i_jetson_orin_nano.yaml` を編集する。

### ground video 関連の launch 引数

| 引数 | 既定 | 備考 |
|---|---|---|
| `enable_ground_video` | `false` | |
| `ground_video_host` | `""` | 空だと無効。受信 PC の実 IP を渡す |
| `ground_video_port` | `5600` | back cam は 5601 |
| `ground_video_width` / `ground_video_height` | `360` / `240` | |
| `ground_video_fps` | `3.0` | |
| `ground_video_jpeg_quality` | `70` | |
| `ground_video_max_pending_frames` | `1` | latest-wins |
| `ground_video_mtu` | `1200` | |

`ground_video_codec` は `jpeg` のみ対応(config 側パラメータ)。他の値だと
`Ground-video streaming disabled: only JPEG is supported` になる。

### 起動成功の判定

送信側ログに以下が出れば Tegra NVJPG ハードウェアエンコーダが動作している:

```
NvMMLiteBlockCreate : Block : BlockType = 1
```

この経路は **libnvjpeg を使わない**(GStreamer の `nvjpegenc` 経由)。Jetson Orin Nano Super の
L4T 39.2 / CUDA 13.2 イメージでは `nvjpegCreateSimple()` が
`NVJPEG_STATUS_EXECUTION_FAILED`(status 6)を返し `/dev/nvhost-nvjpg*` も存在しないため、
libnvjpeg にはリンクしていない(`CMakeLists.txt` と `ground_video_streamer.cu` 冒頭のコメント参照)。

送信パイプラインは
`appsrc ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvjpegenc ! jpegparse ! rtpjpegpay ! udpsink`。
I420(4:2:0)固定なのは `rtpjpegpay`(RFC 2435)の制約で、4:4:4 JPEG を payload すると
`Invalid component` で全フレームが落ちるため。

正常時の RTP は `payload=26 / type=1(4:2:0) / Q=255(量子化表インライン) / 360x240`、
1 フレーム約 27.7 KB = 24 パケット(mtu 1200)、最終パケットに marker bit。

### ZED はプロセス排他。二重起動しない

ZED 2i は 1 プロセスからしか open できない。既に起動済みだと
`CAMERA STREAM FAILED TO START` になる。`Failed to grab a ZED frame` を連発する状態に
陥ったノードはカメラを掴んだまま復帰しないので、止めてから起動し直す:

```shell
pgrep -af zed2i_container
kill -INT <pid>   # 落ちなければ kill -9
```

### 受信レートの実測値(既知の挙動)

`ground_video_fps:=5.0` では**実測 4.07〜4.13 fps**だった(2026-08-01、640x360、約 920 kbps、
フレーム間隔 平均 245 ms / min 199 ms / max 272 ms)。不具合ではない:

- 送信側は「前回送信から 1/fps 秒未満なら捨てる」ゲート方式
  (`ground_video_streamer.cu` の `now - last_selected_ < period_`)
- `/zed2i/left/image_rect` の実測は `framerate:=15` 指定でも約 13.7 Hz
- 73 ms 周期の列に 200 ms ゲートを掛けると採用は 3 フレームおき(219 ms)が上限で、
  grab のジッタで 4 フレームおき(267 ms)が混ざり平均 245 ms = 約 4.1 fps になる

RTP タイムスタンプ側は一定の 5 fps(+18000)を刻むので送信レートとずれる。受信は必ず
`sync=false`(`ground_video_receiver.launch.py` は既定でそう)。`sync=true` では再生が徐々に遅れる。

### 切り分けコマンド

```shell
ros2 topic hz /zed2i/left/image_rect              # カメラ側の実レート(Jetson)
ros2 param get /zed2i/zed2i ground_video_host     # 実際に渡った値の確認
sudo tcpdump -ni any udp port 5600 -c 20          # 受信 PC。ビューアを止めてから
```

## miniPC back_cam の H.264/H.265 陸上伝送

miniPC (UM870 Slim) の背面カメラは VA-API ハードウェアエンコーダを用いる別経路で送る。
既定は H.264、640x480、5 fps、800 kbps、port 5601。`codec:=h265` で H.265 を選べる。
送信側は `minipc_bringup.launch.py` に既定で含まれるが、送信先を指定するまで無効である。

```shell
# miniPC (送信)
ros2 launch robot minipc_bringup.launch.py \
  back_cam_ground_video_host:=192.168.1.2

# 陸上 PC (受信)
ros2 launch zed2i_driver ground_h26x_receiver.launch.py port:=5601 codec:=h264
```

送信機に `vaapih264enc` / `vaapih265enc` が無い場合は起動を失敗させる。ソフトウェア
エンコードへ黙ってフォールバックしないため、CPU負荷が増えた状態で運用されることはない。
受信トピックは復号済みの `sensor_msgs/Image` (`/ground_video/back_cam/image_raw`) である。

## 発行トピック(SDKモードで確認済み)

- `/zed2i/left/image_rect`
- `/zed2i/right/image_rect`
- `/zed2i/left/camera_info`
- `/zed2i/right/camera_info`
- `/zed2i/depth/image`
- `/zed2i/points`

## config について

`left_device` / `right_device` は **CPUモード専用**のパラメータで、SDKモードでは無視される。

安定運用のため、`/dev/videoN` の番号ずれを避けて by-id パスを使用する。

- 左右とも: `/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0`

この ZED 2i は左右画像を横並びにした単一の UVC 映像として公開する。`index1` は
V4L2 メタデータノードであり、映像デバイスとして指定してはならない。CPU ノードは
左右に分割して各トピックへ発行する。

## 前提条件(重要)

SDKモードは**動作する GPU/CUDA が必須**。現在このホストは iGPU がドライバに未登録で `NO GPU DETECTED` により SDK ノードが起動不可(ホスト側ブロッカー)。

詳細と復旧手順は `.ai/escalations/2026-07-12-zed-gpu-cuda-blocker.md` を参照。

## CPUモードの既知の制約

CPU モードは左右画像を分割して `StereoSGBM` で深度を計算する暫定フォールバックである。
精密なキャリブレーションや SDK の機能が必要な運用では SDK モードを使用する。
