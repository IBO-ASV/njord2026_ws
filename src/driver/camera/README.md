# Camera Drivers

This directory contains camera-related ROS 2 packages used by the workspace.

## ZED 2i 陸上映像伝送(ground video)の起動手順

映像を陸上 PC で見る場合、Image トピックを DDS で流さず(1 本約 442 Mbps)
GPU JPEG → RTP/UDP の専用経路を使う。**受信側を先に起動し、次に送信側**の順。

```shell
# 1. 陸上 PC(受信)。ポートごとに 1 プロセスだけ
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5600   # ZED 2i left
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5601   # back cam

# 2. 受信側の実 IP を確認
ip -4 -o addr show scope global | awk '{print $2, $4}'

# 3. Jetson(送信)。ソースを更新したら必ず先に再ビルドすること
colcon build --symlink-install --packages-select zed2i_driver && source install/setup.bash
ros2 launch zed2i_driver zed2i.launch.py \
  enable_ground_video:=true ground_video_host:=192.168.1.2 \
  ground_video_port:=5600 ground_video_fps:=5.0
```

つまずきやすい点(すべて実際に踏んだもの):

- **再ビルド忘れ**: 古い `.so` が残っていると
  `nvjpegCreateSimple failed (nvJPEG status 6)` で映像が出ない。旧 libnvjpeg 実装が
  動いているだけで故障ではない。
- **`ground_video_host` は受信 PC の実 IP**。空文字(既定)・ホスト名・`localhost` は不可。
- **同一ポートの受信を二重起動しない**。UDP unicast は後から bind した側が奪う。
  `ss -lunp | grep 5600` で 1 行だけか確認する。
- **ZED はプロセス排他**。二重起動すると `CAMERA STREAM FAILED TO START`。
- 起動成功のサインは送信側ログの `NvMMLiteBlockCreate : Block : BlockType = 1`。
- `ground_video_fps:=5.0` でも実測は 4.1 fps 程度(仕様どおりの挙動)。

詳細・原因・実測値は `zed2i_driver/README.md` の同名セクションを参照。

## USB Web Camera

Use the ROS 2 `usb_cam` package from apt instead of maintaining a local USB
camera driver package in this workspace.

Install it for the active ROS 2 distribution:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam
```

Check that Linux can see the USB camera:

```bash
ls /dev/video*
v4l2-ctl --list-devices
```

If `v4l2-ctl` is not installed:

```bash
sudo apt install v4l-utils
```

Source ROS 2 and this workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

Start the camera node:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0
```

In another terminal, confirm that image topics are published:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 topic list | grep image
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

## Quick rqt Image Test

Install the image viewer if needed:

```bash
sudo apt install ros-$ROS_DISTRO-rqt-image-view
```

Run the viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/image_raw` from the topic dropdown. If the camera is working, the live
USB camera image should appear in the viewer.

If no image appears, verify the device path with `v4l2-ctl --list-devices` and
retry with the correct `video_device`, such as `/dev/video1`.
