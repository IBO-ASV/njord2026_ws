this pkg is for launch and visualization

## ロール別 bringup(Jetson / miniPC 2台構成)

構成の全体像・ネットワーク設定・ビルド手順は `Docs/two_machine_split.md` を参照。

| launch | 動かす端末 | 内容 |
|---|---|---|
| `ground_pc.launch.py` | Ground PC | joy、前後映像受信、Foxglove bridge、ground-station heartbeat、実軌跡マーカー、CycloneDDS、critical-link sender |
| `jetson_bringup.launch.py` | Jetson | MID360S + ZED 2i + GPU camera/LiDAR buoy detection、Zenoh bridge。GLIMと単体PCL検出は既定OFF |
| `minipc_bringup.launch.py` | miniPC | micon、UM982、localization、スラスタ、back camera、Foxglove logger、Zenoh bridge、critical-link receiver。Drogger/WIT IMUノードは起動対象外 |
| `standalone_bringup.launch.py` | Jetson 1台 | Jetson用とminiPC用bringupを両方includeする回帰用 |

```
# Jetson
ros2 launch robot jetson_bringup.launch.py

# miniPC
ros2 launch robot minipc_bringup.launch.py

# Ground PC (Foxglove is opt-in)
ros2 launch robot ground_pc.launch.py enable_foxglove_bridge:=true

# AGX vessel + Ground PC direct CycloneDDS pair (domain 126, ops-LAN multicast)
./scripts/run_agx_pair.sh vessel
./scripts/run_agx_pair.sh ground

# miniPC: task selection uses the persistent Mission Manager
njord-task check task1
njord-task start task1 --auto
```

minipc_bringup.launch.py / jetson_bringup.launch.py は各端末で一度だけ起動する。
タスクの開始・停止・状態確認は njord-task（Mission Manager の typed API）を使い、
起動時は MANUAL / IDLE のまま自動目標を送らない。

task1.launch.py、task2.launch.py、task3.launch.py は非推奨の回帰比較用
compatibility wrapper である。既定では role bringup、Nav2、waypoint publisher の
いずれも起動しないため、常駐bringupとの二重起動を起こさない。歴史的な一括グラフを
明示的に再現するときだけ、start_role_bringup:=true と
start_legacy_task_nodes:=true を指定する。

task1-1.launch.py、task1-2.launch.py、task2-1.launch.py、task3-1.launch.py、
task3-2.launch.py も同様に enable_legacy_graph:=true を指定した場合だけ旧実験
グラフを起動する。

Jetson/miniPC bringup は既定で `zenoh-bridge-ros2dds` を起動する。bridgeは端末ごとの
JSON5設定でROS domainを選ぶため、親の`ROS_DOMAIN_ID`はbridgeプロセスへ引き継がない。
Ground PCでは`critical_link_sender`、miniPCでは`critical_link_receiver`も既定で起動する。
必要に応じて、Zenoh bridgeとcritical linkはそれぞれ`enable_zenoh_bridge:=false`、
`enable_critical_link:=false`で無効化できる。`ground_pc.launch.py`はZenoh bridgeを既定で起動しない。
`run_agx_pair.sh` の vessel/Ground 構成はdomain 126のCycloneDDS multicastで直接通信し、Zenohを起動しない。
`standalone_bringup.launch.py`は1台で両ロールを含む回帰用のため、両方とも既定で無効である。

miniPCは既定でUM982とAdvanced Navigation Spatialの両方をGround PCのNTRIP caster
（`192.168.1.72:2101`、mountpoint `RTCM3`）へ接続する。
casterの既定クライアント資格情報は `test:test` である。UM982 RTKは
`enable_um982_rtk:=false`、Spatialは `enable_spatial:=false` を指定すれば
個別に無効化できる。Spatialの起動引数の既定値は安全のため `false`。

miniPCのlocal odometryは既定でUM982 feedback EKFを使う。
`use_ekf_local:=true` のときだけ、これを停止してLivox IMU入力のlocal EKFへ
排他的に切り替える。両方のEKFが同時に
`/odometry/filtered/local` と `odom -> base_link` を出すことはない。

## GLIM local + UM982 map EKF（実験構成）

dual EKFの入力を相互に混ぜず、TFの責務だけを分離して確認するためのlaunch:

```bash
ros2 launch robot glim_um982_localization.launch.py
```

このlaunchはUM982 driverも起動する。別プロセスですでに起動している場合は、driver側で
`publish_feedback_odometry:=true feedback_frame_id:=map`を指定したうえで、次のようにする。

```bash
ros2 launch robot glim_um982_localization.launch.py enable_um982_driver:=false
```

TFとtopicの責務は次のとおり。

```text
GLIM /odom -> GLIM local EKF -> odom -> base_link
UM982 first-fix ENU -> UM982 map EKF -> map -> odom
```

GLIM自身の`/tf`は外部へ出さないこと。既存のJetson bringupはGLIMのTFを
`/glim/tf_unused`へ隔離している。このlaunchと従来の`localization.launch.py`または
UM982 feedback EKFを同時に起動するとTFが競合するため、排他的に使用する。

driver heartbeatは実データの鮮度から階層的に生成する。Jetsonが前カメラと
LiDARのleaf heartbeatを生成し、miniPCがback camera、GNSS、Miconと合わせて
`/heartbeat/driver` へ集約する。local/global odometryも
`/heartbeat/localization` へ集約される。

Ground PCではNTRIP casterも既定起動する。Drogger RWS/ETHMの接続設定は
`192.168.1.72:2101`、mountpoint `RTCM3`、SOURCE password `osw` の
NTRIP v1 SOURCE方式とする。
Casterは `src/driver/gnss/ntripcaster` のsubmoduleをcolconでビルドし、
workspaceのinstall spaceから起動する。既定のSOURCE/client資格情報は公開済みの
試験値なので、閉じた実験LAN以外ではconfigを差し替えること。

`glim_backend`(`gpu` / `cpu`、既定 `gpu`)で GLIM の設定ディレクトリを切り替えられる。`cpu` は `config/glim_config_cpu/` を使い、OpenGL ビューアを外したヘッドレス構成になる。

## back_cam(背面USBカメラ)

Adesso CyberTrack H7 を使用。

```
ros2 launch robot back_cam.launch.py
```

発行トピック(実測):

- `/back_cam/image_raw`(~26-30Hz)
- `/back_cam/camera_info`

フォーマットは MJPG 640x480 @30fps。

`video_device` は既定で by-id 安定パスを指定している。launch 起動時に `os.path.realpath` で実際の `/dev/videoN` に解決してから `usb_cam` に渡している(`usb_cam` が by-id シンボリックリンクを直接開けないため)。

上書きする場合:

```
ros2 launch robot back_cam.launch.py video_device:=/dev/videoN
```

## real_bringup(実機一括起動)

```
ros2 launch robot real_bringup.launch.py
```

`enable_*` 引数(`enable_imu` のみ既定 `false`、それ以外は `true`):

- `enable_mid360`
- `enable_zed2i`
- `enable_back_cam`
- `enable_um982`
- `enable_drogger_rzs`
- `enable_imu`
- `enable_localization`
- `enable_thruster`
- `enable_nav2`(既定 `true`)

その他の引数:

- `lidar_model`(既定 `mid360s`、`mid360` または `mid360s`)
- `serial_port`(既定 `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c82421728a9aef118808b29061ce3355-if00-port0`)
- `baud`(既定 `115200`)
- `um982_protocol`(既定 `uart`)
- `enable_glim`(既定 `true`)
- `glim_backend`(既定 `gpu`、`gpu` または `cpu`)
- `enable_pcl_buoy_detection`(既定 `false`)
- `enable_gpu_perception`(既定 `false`)
- `gpu_perception_engine_path`(既定は空)
- `camera_resolution`(既定 `HD720`) / `camera_framerate`(既定 `15`)
- `enable_ground_video`(既定 `false`) / `ground_video_host` / `ground_video_port`(既定 `5600`)
- `imu_port`(既定 `/dev/ttyUSB0`) / `imu_baud`(既定 `9600`)
- `um982_port`(既定 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`)
- `enable_um982_rtk`(既定 `false`)
- `drogger_rzs_port`(既定 `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ACCQg146B12-if00-port0`)
- `drogger_rzs_baud`(既定 `115200`)
- `drogger_rzs_fix_topic`(既定 `/gnss/fix`)
- `device`(back_cam の by-id パス)

使用例(back_cam のみ起動):

```
ros2 launch robot real_bringup.launch.py enable_mid360:=false enable_zed2i:=false enable_um982:=false enable_imu:=false enable_localization:=false enable_thruster:=false
```

注意: `zed2i` を含める場合、SDKモードの GPU 前提が満たされている必要がある(詳細は `src/driver/camera/zed2i_driver/README.md` を参照)。

## ZED 2i 陸上映像伝送(ground video)の起動手順

bringup 経由で有効化する場合の手順。**受信側(陸上 PC)を先に起動し、次に Jetson**の順。

```shell
# 1. 陸上 PC(受信)。ポートごとに 1 プロセスだけ
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5600   # ZED 2i left
ros2 launch zed2i_driver ground_h26x_receiver.launch.py port:=5601 codec:=h264 # back cam

# 2. 受信側の実 IP を確認(ホスト名や localhost は不可)
ip -4 -o addr show scope global | awk '{print $2, $4}'

# 3. Jetson。ソースを更新したら必ず先に再ビルドする
colcon build --symlink-install --packages-select zed2i_driver && source install/setup.bash
ros2 launch robot jetson_bringup.launch.py \
  enable_ground_video:=true \
  ground_video_host:=192.168.1.2 \
  ground_video_port:=5600


  ros2 launch robot jetson_bringup.launch.py \
  enable_ground_video:=true \
  ground_video_host:=100.82.157.125 \
  ground_video_port:=5600
```

`jetson_bringup.launch.py` は `enable_ground_video` / `ground_video_host` /
`ground_video_port` に加え、`ground_video_width` / `ground_video_height` /
`ground_video_fps` も `zed2i_driver` へ転送する。既定は JPEG 480x360・4 fps。

miniPCのback_camはH.264・VA-API送信と、UDP 5602 の JPEG互換送信が既定で含まれる。
送信先を指定して有効化する:

```shell
ros2 launch robot minipc_bringup.launch.py back_cam_ground_video_host:=192.168.1.2
```

`back_cam_ground_video_codec:=h265` でH.265を選択できる。陸上PCはH.264/H.265をUDP 5601、
JPEG互換映像をUDP 5602で受信する。空のhostでは送信ノードは安全に無効化される。

つまずきやすい点:

- **再ビルド忘れ**で古い `.so` がロードされると
  `nvjpegCreateSimple failed (nvJPEG status 6)` になり映像が出ない(故障ではない)。
- **同一ポートの受信を二重起動しない**(`ss -lunp | grep 5600` が 1 行だけか確認)。
- **ZED はプロセス排他**。二重起動で `CAMERA STREAM FAILED TO START`。
- 起動成功のサインは送信側ログの `NvMMLiteBlockCreate : Block : BlockType = 1`。
- `ground_video_fps:=5.0` でも実測は 4.1 fps 程度(仕様どおりの挙動)。

詳細は `src/driver/camera/zed2i_driver/README.md` の同名セクションを参照。
