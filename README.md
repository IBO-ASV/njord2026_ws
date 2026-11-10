# GitHub運用上の重要ルール

このワークスペースに関するGitHubへの書き込みは、`IBO-ASV` または `oswystk15662` がownerのリポジトリに限定します。

- 他ownerのリポジトリへpush、Pull Request、Issue、コメント、レビューなどを送らないでください。
- `IBO-ASV`／`oswystk15662` 配下のforkから、外部upstreamへPull Requestを作ることも禁止です。
- 外部コードの変更が必要な場合は、許可owner配下へforkし、そのfork内のbranchへpushするところまでにしてください。
- clone、fetch、参照などのread-only操作は対象外です。

# 環境について

## 端末構成（Jetson / miniPC 2台）

このワークスペースは **Jetson（ZED 2i + Livox MID360S の処理のみ）** と **miniPC（それ以外すべて）** の 2 台構成を前提にしている。同一ソースツリーが両方でビルドできるよう、CUDA / TensorRT / ZED SDK / ROS distro の有無をビルド時に自動検出して依存とリンク先を切り替える。

ビルド前に必ず以下を source すること:

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
source scripts/njord_env.sh      # プロファイル判定と環境変数の export
colcon build --symlink-install
```

`scripts/build.sh` は上記をまとめたラッパ。

minIPC プロファイルでは `scripts/njord_env.sh` が Livox 関連と
`pcl_segmentation` に `COLCON_IGNORE` を自動配置するため、通常の
`colcon build` でこれらはビルドされない。Jetson では同ファイルを自動削除する。

配線図・ノード配置・帯域の注意・ネットワーク設定（FastRTPS / Zenoh）・時刻同期・検証手順は **`Docs/two_machine_split.md`** にまとめてある。

## 環境構築
まずはros2 humbleが入っているubuntu22.04を起動し、以下のコマンドでこのワークスペースをクローンしてください。

```shell
# githubのsshを通していない場合
git clone https://github.com/oswystk15662/njord2026_ws.git --recursive

# githubのsshを通している場合
git clone git@github.com:oswystk15662/njord2026_ws.git --recursive
```

正常にクローンできたか確認するために、vscodeでこのワークスペースディレクトリを開き、./src/driver/imu/内のwit_node_ros2の中身があるか確認してください。

ない場合は以下のコマンドを叩いてください

```shell
# ワークスペースディレクトリにいる状態で以下を打つ
cd ./src/driver/imu/wit_node_ros2 && git submodule update --init --remote

# ワークスペースディレクトリに戻る
cd ../../../../
```

クローンできたら、colcon buildしてください

```shell
cd njord2026_ws && colcon build --symlink-install
```

おそらく以下のaptで入れるパッケージがないというエラーが出るので、入れましょう。
でなくてもlaunchするときにエラーが出ると思うので入れましょう。

```shell
sudo apt install ros-humble-ecl* ros-humble-robot-localization
```

ビルドが成功したら環境構築はうまく行っていると思います。

## Advanced Navigation Spatial v8.0

GNSSアンテナを接続した Spatial v8.0 は `src/driver/ins/ros2-driver`
の `adnav_driver` で扱います。現物確認では `/dev/ttyUSB0` を
`ttyUSB0` として指定し、115200 baud で Device Information packet を取得できました。
`/dev/` prefix は driver 側が内部で付けるため、設定ファイルには入れません。

現在の設定は `src/robot/config/adnav_spatial.yaml` です。packet 20
（filtered INS/GNSS state）と packet 28（raw sensors）を 20 Hz で要求します。
115200 baud で高い packet rate にすると `SERIAL PORT DATA OVERFLOW` が出るため、
この rate に落としています。

引き継ぎと実機確認結果は `Docs/spatial_v8_handoff.md` を参照してください。

## ディレクトリ構成について

colcon buildしたあとは以下のようなディレクトリ構造になっていると思います。

```shell
$ tree -d
.
├── bag_files/  # rosbagを保存する場所。なおrosbag自体をgitで管理するわけではなく、入れるところっていうだけ
├── build/      # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
├── install/    # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
├── log/        # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
└── src
    ├── bt      # behavior tree周りの諸々（action nodeなど）を入れる。
    ├── ditection   # 物体認識周り
    │   ├── pcl_det # Point CLoud DETection
    │   └── yolo    # You Look Only Once
    │
    ├── driver
    │   ├── camera
    │   │   ├── usb_camera_driver   # logicool c270用
    │   │   └── zed-ros2-wrapper/ # zed 2i用
    │   │       ├── zed_ros2
    │   │       └── zed_wrapper/
    │   ├── gnss
    │   │   ├── drogger_bt  # drogger gnss moduleとblue toothで接続する用
    │   │   ├── drogger_rws # drogger gnss moduleの中身がublox ZED-F9Pのやつ
    │   │   ├── drogger_rzs # drogger gnss moduleの中身がseptentorio mozaicのやつ
    │   │   └── um982_driver # unicore um982用
    │   └── imu
    │       ├── sonota # その他（）
    │       └── wit_node_ros2 # witmotion mpu9250用
    │           ├── doc
    │           ├── wit_driver
    │           ├── wit_msgs
    │           └── wit_node
    ├── path_follower # 経路追従アルゴリズム・ノードを入れるところ。
    ├── path_generator # 経路生成アルゴリズム・ノードを入れるところ。避航も含む
    └── robot
        ├── config # ここには最終的にlaunchするときに使う設定ファイルとかを入れる。単体試験は各パッケージlaunch書いてください。
        ├── launch # ここには最終的にlaunchするときに使うlaunchファイルを入れる。単体試験用launchを呼び出す形で書くと良いとされている
        ├── meshes # 表示用STLファイルを入れるところ
        └── urdf # tf設定・表示用のurdfを入れるところ
```

colcon buildによって生まれた、build・installl・logディレクトリは.gitignoreによって、gitが変更を追跡しないようにしているため、薄い灰色になっています。
中見ればわかりますが、非常に大量のファイルが生まれ、変更差分が分かりづらくなるため、無視するのが一般的です。
bag_filesの中身も同様に追跡しないようになっています。

**重要**
カスタムメッセージをむやみに生やさないでください。
基本的にros側で適するものが用意されています。

## visualizerについて
fox gloveを推奨します。
https://foxglove.dev/download

ros2ネットワークにそのままアクセスできるようにするため、
```shell
sudo apt install ros-humble-foxglove-bridge
```
をしてください。

## ZED 2i 陸上映像伝送（ground video）の正しい起動手順

Image トピックを DDS で流すと 1 本あたり約 442 Mbps になるため、陸上 PC で映像を見る場合は
DDS ではなく **GPU JPEG → RTP/UDP** の専用経路を使う。送信は Jetson の `zed2i_driver`
（`src/driver/camera/zed2i_driver/src/ground_video_streamer.cu`）、受信は陸上 PC の
`ground_video_receiver.launch.py`。

### 手順（この順序で起動する）

**0. Jetson 側のソースを更新したら必ず再ビルドする（最重要）**

```shell
# Jetson 上で
cd ~/njord2026_ws
source /opt/ros/$ROS_DISTRO/setup.bash   # Jetson の実機は jazzy
colcon build --symlink-install --packages-select zed2i_driver
source install/setup.bash
```

`git pull` やソース修正だけして再ビルドしないと **古い `.so` がロードされ**、
`Ground-video streaming disabled: nvjpegCreateSimple failed (nvJPEG status 6)`
で映像が出ない。これは既に解決済みの旧実装（libnvjpeg 版）が動いているだけで、
ハードやドライバの故障ではない。判別方法:

```shell
ls -l --time-style=long-iso ~/njord2026_ws/build/zed2i_driver/libzed2i_sdk_component.so
strings ~/njord2026_ws/build/zed2i_driver/libzed2i_sdk_component.so | grep -c nvjpegenc  # 1 なら新実装
```

**1. 受信側（陸上 PC）を先に起動する。ポートごとに 1 プロセスだけ**

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# ZED 2i left camera
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5600

# back camera（別端末）
ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5601
```

同じポートを掴むプロセスを二重に起動してはいけない。UDP の unicast は
`SO_REUSEADDR` で bind しても片方のソケットにしか配送されないため、
後から起動した側が奪って**先の受信ウィンドウが黒画面のまま止まる**。確認:

```shell
ss -lunp | grep -E '5600|5601'   # 各ポートに 1 行だけであること
```

**2. 受信側の IP を確認する。`ground_video_host` はここで見えた実 IP を渡す**

```shell
ip -4 -o addr show scope global | awk '{print $2, $4}'
```

Jetson と共有しているサブネット側の IP を選ぶ（有線 `192.168.1.x` 系が本線）。
ホスト名や `localhost` は不可。空文字のままだと送信そのものが無効になる。

**3. 送信側（Jetson）を起動する**

単体で確認する場合:

```shell
# Jetson 上で
ros2 launch zed2i_driver zed2i.launch.py \
  enable_ground_video:=true \
  ground_video_host:=192.168.1.2 \
  ground_video_port:=5600 \
  ground_video_fps:=5.0
```

実運用の bringup 経由:

```shell
ros2 launch robot jetson_bringup.launch.py \
  enable_ground_video:=true \
  ground_video_host:=192.168.1.2 \
  ground_video_port:=5600
```

注意: `jetson_bringup.launch.py` が転送するのは `enable_ground_video` /
`ground_video_host` / `ground_video_port` の 3 つだけ。fps・解像度・JPEG 品質を変えたい場合は
`src/driver/camera/zed2i_driver/config/zed2i_jetson_orin_nano.yaml` 側を編集する。

### 起動が成功したかの判定

送信側ログに次の行が出れば Tegra の NVJPG ハードウェアエンコーダが動いている:

```
NvMMLiteBlockCreate : Block : BlockType = 1
```

`nvjpeg` 関連の ERROR が出ていないことも確認する。この経路は **libnvjpeg を使わない**
（GStreamer の `nvjpegenc` エレメント経由）。Jetson Orin Nano Super の
L4T 39.2 / CUDA 13.2 イメージでは `nvjpegCreateSimple()` が
`NVJPEG_STATUS_EXECUTION_FAILED`（status 6）を返し `/dev/nvhost-nvjpg*` も存在しないため、
libnvjpeg には依存させていない。詳細は `ground_video_streamer.cu` 冒頭のコメント。

### ZED はプロセス排他。二重起動しない

ZED 2i は 1 プロセスからしか open できない。既に別端末で起動していると
`CAMERA STREAM FAILED TO START` になる。また `Failed to grab a ZED frame` を
連発する状態に陥ったノードはカメラを掴んだまま復帰しないので、止めてから起動し直す:

```shell
pgrep -af zed2i_container
kill -INT <pid>   # 落ちなければ kill -9
```

### 受信レートの実測値（既知の挙動）

`ground_video_fps:=5.0` を指定しても**実測は 4.07〜4.13 fps** になる（2026-08-01 実測、640x360、約 920 kbps）。
不具合ではなく間引きロジックの結果:

- 送信側は「前回送信から 1/fps 秒未満なら捨てる」ゲート方式（`ground_video_streamer.cu`）
- `/zed2i/left/image_rect` の実測は `framerate:=15` 指定でも約 13.7 Hz
- 73 ms 周期の列に 200 ms ゲートを掛けると採用は 3 フレームおき（219 ms）が上限で、
  grab のジッタで 4 フレームおき（267 ms）が混ざり平均 245 ms = 約 4.1 fps に落ちる

RTP タイムスタンプ側は一定の 5 fps（+18000）を刻むため送信レートとずれる。
受信パイプラインは必ず `sync=false`（`ground_video_receiver.launch.py` は既定でそうなっている）。
`sync=true` にすると再生が徐々に遅れていく。

### 切り分けコマンド

```shell
# カメラ側の実レート（Jetson）
ros2 topic hz /zed2i/left/image_rect

# 受信側に本当にパケットが来ているか（陸上 PC。受信ビューアを止めてから実行する）
sudo tcpdump -ni any udp port 5600 -c 20
```

## YOLOについて
Jetson Nano + Ubuntu 20.04 + ROS2 Foxy では、ZED/ROS系とYOLO系を同じPython環境に混ぜないでください。
`numpy` の競合で `cv_bridge` が壊れることがあります。

YOLOノードが使用する ROS メッセージ定義は別途必要である。Humble 環境では、初回に
次を実行する。

```shell
sudo apt install ros-humble-vision-msgs
```

運用ルール:
* ROS2/ZED ノードはシステム環境（apt + colcon）で実行する
* YOLOノードは `.venv` で実行する
* Jetson では `torch` / `torchvision` を `pip` で上書きしない

### Jetson用セットアップ

```shell
cd ~/njord2026_ws

# venvが未導入の場合
sudo apt update
sudo apt install -y python3-venv

# YOLO専用venv
# .venvがなければuv venv（uv未導入時はpython3 -m venv）で自動作成される
source ./export_python_path.sh
# Jetson専用requirementsのみ使う
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install -r requirements_jetson_nano_yolo.txt
```

### スモークテスト

```shell
python3 -c "import numpy, cv2, ultralytics; print('numpy=', numpy.__version__, 'opencv=', cv2.__version__, 'ultralytics=', ultralytics.__version__)"
python3 -c "import cv_bridge; print('cv_bridge import OK')"
```

YOLOノード起動は以下のラッパーを使うと安全です。

```shell
./run_yolo_jetson.sh
```

launchファイルを直接使う場合も、`export_python_path.sh`をsourceして
`.venv`の有効化、Python pathの設定、ユーザーsite-packagesの除外を行ってください。

```shell
source ./export_python_path.sh
ros2 launch yolo yolo.launch.py
```

### よくあるNG

* Jetsonで `pip install -r requirements.txt` を実行する
* Jetsonで `pip install torch torchvision` を実行する
* `numpy>=2.0` に上がった状態で ROS Python ノードを起動する

Foxy + ZED 既知の注意点として、`image_transport` は `v3.0.0` 系を使ってください（`zed_wrapper` の README 参照）。


#　座標系について
機体前方をxとし、鉛直地面から空向きをz正とするようにyをとります。
基本的にすべてENUで考えるために、NEDで出てきがちなimuはだいたい回しています。
**センサの座標はdatasheetを信用せず、dataを見てやってください**


# aptで入れるパッケージ
## Jetson Orin Nano Super + ROS Jazzy の OpenCV 注意点

Jetson Orin Nano Super 上で ROS Jazzy を使う場合、OpenCV の apt パッケージが NVIDIA Jetson repo 版と Ubuntu noble 版で混在すると、`cv_bridge` の `find_package` 時に以下のような CMake エラーが出ることがあります。

```shell
The imported target "opencv_core" references the file
  "/usr/lib/libopencv_core.so.4.8.0"
but this file does not exist.
```

確認時点では、`libopencv-dev` だけが NVIDIA Jetson repo の `4.8.0-3-g6ef37b4` で、実体ライブラリは Ubuntu noble の `libopencv-core406t64 4.6.0+dfsg-13.1ubuntu1` になっていました。この状態では `/usr/lib/cmake/opencv4/OpenCVModules-release.cmake` が OpenCV 4.8 の `.so` を参照しますが、実体が存在しないため `camera_lidar_fusion` などのビルドが失敗します。

当面は ROS Jazzy / Ubuntu noble 側との整合を優先し、OpenCV を Ubuntu 4.6 系に揃えてください。

```shell
sudo apt install -s libopencv-dev=4.6.0+dfsg-13.1ubuntu1 libopencv-core-dev=4.6.0+dfsg-13.1ubuntu1

# simulation の内容に問題がなければ実行
sudo apt install libopencv-dev=4.6.0+dfsg-13.1ubuntu1 libopencv-core-dev=4.6.0+dfsg-13.1ubuntu1
sudo apt-mark hold libopencv-dev libopencv-core-dev

cd ~/njord2026_ws
rm -rf build install log
colcon build
```

NVIDIA 版 OpenCV 4.8 に揃えるのは、`cv::cuda` など OpenCV の CUDA 機能を明示的に使う必要があり、runtime と dev パッケージを同じ系列で揃えられることを確認してからにしてください。通常の `cv_bridge`、`cv::Mat`、PCL/カメラ/LiDAR fusion のビルド安定性は Ubuntu 4.6 系に揃える方を優先します。

for wit motion
* ros-${ROS_DISTRO}-sophus
* ros-${ROS_DISTRO}-tf2-geometry-msgs
* ros-${ROS_DISTRO}-ecl*

for usb camera
* opencv(ubuntu22.04にあったものを入れてください)
* ros-humble-cv-bridge
* ros-humble-image-pipeline
* gstreamer系の諸々(mjpg経由は動くけどgstreamer経由はうまく動かない可能性が高いが、依存の中に入れてしまってるので入れてください。すみません)

for zed 2i
* CUDA 12.x/13.x に対応する Stereolabs ZED SDK（colcon build 時に CMake が検出します）
* CUDA 13.1 環境では CUDA 13.x 対応の ZED SDK を入れてから rebuild してください
* ROS2 Foxyなら [setup_foxy_image_transport.sh](setup_foxy_image_transport.sh) で image_transport v3 系を source から固定する

for localization
* ros-humble-robot-localization

for navigation
* ros-humble-navigation2
* ros-humble-nav2-bringup

for simulation
* ros-humble-tf-transformations

about topic network



大澤個人的ToDo

* waypoint生成
    * [ ] ダイクストラ理解・python実装・pyqtでのGUI
    * [ ] A*理解・cpp実装・rqtとかで表示
    * [ ] theta*理解・ros2 cpp実装・o3de表示
* 補間
    * [ ] スプライン理解・python実装・pyqt表示
    * [ ] beta spline理解・python実装・pyqt表示
    * [ ] NURBS理解・cpp実装・いい感じ表示
    * [ ] クロソイド理解・ros2 cpp実装・o3de表示
* 

Nav2（ROS 2 Navigation Stack）を中心とした実装方針、非常に良い選択です。Njordの自律船（ASV）タスクにおいて、拡張性と堅牢性を確保できます。

Task 1（ウェイポイント追従 + 方位標識による回避）をNav2で実現するための、具体的な実装ステップを4段階で解説します。

---

###ステップ1：座標系と位置推定の確立 (Localization)Nav2を動かすには、「ロボットが地図（地球）上のどこにいるか」を正確に定義する必要があります。

1. **TFツリーの構築:**
ロボットの形状定義（URDF）と座標変換（TF）を作成します。
* `base_link`: 船の中心
* `gnss_link`, `imu_link`, `lidar_link`: 各センサの取り付け位置


2. **GPS + IMU フュージョン (`robot_localization`):**
船はオドメトリ（エンコーダ）がないため、GPSとIMUを融合して `map` 座標系と `odom` 座標系を作ります。
* `ekf_node`: IMUの加速度・角速度から、短期間の滑らかな位置変化（`odom` -> `base_link`）を計算。
* `navsat_transform_node`: GPSの緯度経度を `map` 座標系のXY平面に変換（`map` -> `odom`）。



**実装すること:**

* `robot_localization` パッケージのインストールとパラメータ設定（`ekf.yaml`）。

---

###ステップ2：Nav2 の基本設定 (Configuration)まずは「障害物がない海面を、指定したGPS座標まで直進する」部分（Task 1の前半）を作ります。

1. **Costmap（コストマップ）の設定:**
海上の障害物を管理するマップです。
* **Global Costmap:** 全体経路計画用。GPS座標と地図のサイズ（十分大きく、例えば 2000m x 2000m）を設定。
* **Local Costmap:** 直近の回避用。Rolling Window（ロボット中心に追従する設定）をTrueにします。
* **Layer:** `ObstacleLayer` を入れ、LiDAR (`LaserScan` or `PointCloud2`) を入力ソースに指定します。これで「ブイ」が障害物として認識されます。


2. **Controller (Local Planner) の選択:**
船の挙動に合ったコントローラを選びます。
* **推奨:** **`Regulated Pure Pursuit`**
* 理由: レガシーコードでも Pure Pursuit が使われており、挙動が直感的です。中〜高速での経路追従に優れています。


* **次点:** **`MPPI Controller`**
* 理由: GPU（Jetson等）があるならこちらが最強です。動的な障害物回避能力が非常に高いです。





**実装すること:**

* `nav2_params.yaml` の作成（Planner, Controller, Costmapの設定）。
* `gps_waypoint_follower` (Nav2のサンプル) を実行し、船がGPS点へ動くか確認。

---

###ステップ3：方位標識（Cardinal Marks）の対応ロジックここがTask 1後半の肝です。「北方位標識が見えたら、北側を通る（＝南側に行ってはいけない）」というルールをNav2に守らせます。

**アプローチ：「仮想壁（Virtual Wall）」の生成**

Nav2の標準機能だけでは「北を通れ」という指示は理解できません。そこで、**「南側に嘘の障害物（壁）を作る」** ことで、Plannerに強制的に北側のルートを引かせます。

**実装の流れ:**

1. **認識ノード (`yolo_to_marker_node`):**
* YOLOでブイを検出。
* LiDARと突き合わせて、ブイの正確な3次元位置 (x, y) を特定。
* ブイの種類（例：North Cardinal）を判別。


2. **仮想障害物生成:**
* もし **North Cardinal** が (x, y) にあったら、その **南側** 半径 R メートルの範囲に、点群（PointCloud2）を生成して配信します。
* トピック名例: `/virtual_obstacles`


3. **Nav2連携:**
* Costmapの `ObstacleLayer` に `/virtual_obstacles` トピックを追加します。
* これでNav2は「標識の南側は壁だらけだ」と認識し、自動的に北側を通るルートを再計算（Replanning）します。



**Python擬似コード:**

```python
def callback(self, detection):
    if detection.label == "North_Mark":
        # 北標識の南側に仮想の壁を作る
        fake_points = []
        buoy_x, buoy_y = detection.position
        
        # ブイの南側(yマイナス方向)に半円状に点を配置
        for theta in range(180, 360): 
            rad = math.radians(theta)
            px = buoy_x + radius * math.cos(rad)
            py = buoy_y + radius * math.sin(rad)
            fake_points.append([px, py, 0.0])
        
        self.publish_point_cloud(fake_points) # Costmapがこれを読み込む

```

---

###ステップ4：Behavior Tree (BT) の構築最後に、これらを統合します。Task 1用のシンプルなBT XMLを作成します。

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <NavigateThroughPoses goals="{gps_waypoints}" />
    </Sequence>
  </BehaviorTree>
</root>

```

* **動的な回避:**
ステップ3の「仮想壁」の仕組みがあれば、BT側で特別な分岐（if North then...）を書く必要はありません。Nav2の `MapsThroughPoses` が実行されている裏で、Costmapが更新されれば、自動的に経路が修正されるからです。

---

###まとめ：開発ロードマップ1. **足回り:** `robot_localization` で GPS/IMU を統合し、TF (`map` -> `base_link`) を確立する。
2. **基本航行:** Nav2 を `Regulated Pure Pursuit` で設定し、障害物のない海でGPS走行できるようにする。
3. **認識統合:** YOLOとLiDARを統合し、ブイの位置特定ノードを作る。
4. **ルール適用:** 「仮想壁生成ノード」を作り、方位標識の特定方向をNav2上で通行止めにする。

まずは **ステップ1と2**（GPSでの自律航行）を最優先で完了させることをお勧めします。これだけでTask 1の半分はクリアできます。

## UM982 GNSS live-check notes

UM982 driver live-check work is in branch `codex/um982-live-check`.
The likely USB serial device observed on the test machine was `/dev/ttyUSB0`, also available as
`/dev/serial/by-id/usb-FTDI_USB_HS_SERIAL_CONVERTER_FTRTKA7O-if00-port0`.

At test time `/dev/ttyUSB0` produced data at 115200 bps, but it was a binary stream rather than
NMEA/ASCII `$GNGGA` or `#UNIHEADINGA`. The driver therefore applies volatile startup configuration
each time it opens the receiver and intentionally does not send `SAVECONFIG`:

```text
UNLOG
MODE ROVER
GPGGA <fix_period>
UNIHEADINGA <heading_period>
GPTHS <heading_period>
```

`$GNGGA` and `$GPGGA` are both accepted as fix input. `#UNIHEADINGA` remains the primary dual-antenna
heading source, with `$GNTHS`/`$GPTHS` accepted as a fallback heading sentence.

### 2026-07-07 update: binary-only heading, revert-on-shutdown

Live-checked against the actual UM982 (this time over a CH340 USB-serial adapter; the kernel
lacked a `ch341` driver, which was built out-of-tree from upstream `drivers/usb/serial/ch341.c`
and installed). Confirmed end-to-end at 20 Hz through the ROS driver:

- `$GNGGA`/`$GPGGA` (position fix) — ASCII NMEA only. The receiver rejects `GPGGAB`
  (`PARSING FAILED NO MATCHING FUNC`), so there is no binary counterpart; GPGGA stays ASCII.
- `UNIHEADING` (dual-antenna heading) — the receiver accepts **both** `UNIHEADINGA` (ASCII) and
  `UNIHEADINGB` (binary). Per policy, the driver now requests `UNIHEADINGB` only and parses the
  Unicore binary frame directly (sync `0xAA 0x44 0xB5`, 24-byte header, message ID 972, CRC-32 per
  the Unicore Reference Commands Manual). `GPTHS` has no binary counterpart (`GPTHSB` is also
  rejected) and remains as the ASCII fallback heading sentence.
- Serial reads are no longer line-delimited only: since binary frames can contain `\n` bytes,
  `um982_driver_node` now reads raw chunks and scans the buffer for either an ASCII `$`/`#` line or
  a binary sync+length+CRC frame.
- On shutdown (SIGINT or the `shutdown` control command), the driver now sends `UNLOG` to revert
  the volatile logging configuration it applied at startup, so the receiver doesn't keep streaming
  our custom config (previously the cause of a stray binary stream from an earlier, un-reverted
  session). Verified live: after node shutdown, the serial port goes silent.

## 2026-07-17 実機フルスタック試験（Drogger / WIT IMUを除く）

`test07089`上でZED2i、back_cam、MID360S、UM982、Advanced Navigation
Spatial、ESP32、前後カメラ用YOLO、点群ブイ検出、local/global EKF、NavSat
Transform、Task1 Nav2を同時起動して確認した。試験機のROS 2はJazzyである。
DroggerとWIT IMUは意図的に起動していない。Nav2へ目標は送信せず、推進指令も
送信していない。試験後は関連プロセスをすべて停止した。

### 入れた変更

- `origin/40-advanced-navigation-spatial-ins`を`test07089`へmergeした。
  merge commitは`05e9b1f`。
- `src/detection/yolo/yolo/main.py`の画像subscriberを
  `qos_profile_sensor_data`へ変更した。ZED2iはBest Effortで画像をpublishするため、
  変更前のReliable subscriberではQoS不一致となり、前方YOLOが画像を受信できなかった。
- `src/sim/task1_sim/config/task1_nav2_params.yaml`のTheta* plugin名を
  `nav2_theta_star_planner::ThetaStarPlanner`へ変更した。
- 同ファイルからHumble用の`plugin_lib_names`明示リストを削除した。JazzyではBT
  pluginが自動ロードされ、明示リストがあると`ComputePathToPose already registered`
  でBT Navigatorが停止したため。

変更後のビルド・構文確認:

```bash
cd /home/ibo_asv/njord2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-up-to yolo pcl_det \
  --event-handlers console_direct+
colcon build --symlink-install \
  --packages-select natural_cubic_spline \
  --event-handlers console_direct+

python3 -m py_compile src/detection/yolo/yolo/main.py
git diff --check
```

### 起動コマンド

センサ、ESP32、Spatial、点群ブイ検出は以下で起動した。Spatialは
`localization.launch.py`がGLIM検索時に停止した後も起動済みだったため、二重起動は
していない。

```bash
cd /home/ibo_asv/njord2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot lidar.launch.py \
  lidar_model:=mid360s enable_buoy_detection:=true
ros2 launch zed2i_driver zed2i.launch.py mode:=sdk
ros2 launch robot back_cam.launch.py
ros2 launch um982_driver um982.launch.py \
  uart_or_tcp:=uart gnss_port:=/dev/ttyUSB2 rtk_enable:=false
ros2 run micon_driver_fd serial_writer --ros-args \
  -p serial_port:=/dev/ttyUSB1 -p baud:=115200
ros2 launch robot localization.launch.py
```

背面カメラの解像度・フレームレート・`pixel_format`・コントラストは
`src/robot/config/back_cam.yaml` で設定する。`contrast` はカメラが受け付ける
範囲内の整数に変更してから、launch を再起動する。範囲は
`v4l2-ctl --device=/dev/videoN --list-ctrls` で確認できる。

白飛びを抑えるには、同ファイルの `autoexposure: false` を維持し、
`exposure` を小さくする（Adesso CyberTrack H7 は `1`--`5000`）。まず `50` を
基準にし、白飛びする場合は `20`、暗すぎる場合は `80` のように調整する。
`back_cam.launch.py` はこの値を V4L2 の `exposure_time_absolute` へ適用してから
`usb_cam` を起動する。

別の設定ファイルを使う場合は、次のように指定できる。

```bash
ros2 launch robot back_cam.launch.py params_file:=/path/to/back_cam.yaml
```

この PC の内蔵カメラで試すには、`back_cam_pc.yaml` を用いる。

```bash
ros2 launch robot back_cam.launch.py \
  params_file:=$(ros2 pkg prefix robot)/share/robot/config/back_cam_pc.yaml
```

`enable_buoy_detection:=true`ではLivox driverと点群ブイ検出を同じ
`component_container_mt`へロードし、`/livox/lidar`区間でintra-process通信を使う。
個別起動が必要な場合は従来どおり`pcl_bouy_det.launch.py`も使用できる。

`localization.launch.py`は`glim_ros`がインストールされていないためlaunch全体を完遂
できなかった。残りのlocal/global EKFとNavSat Transformは、同launchファイルと同じ
parameter/remapで個別起動した。

```bash
ros2 run robot_state_publisher robot_state_publisher \
  src/robot/urdf/robot.urdf_modified.urdf

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id um982_link

ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node_local \
  --params-file src/robot/config/ekf_local.yaml \
  -r odometry/filtered:=odometry/filtered/local

ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node_global \
  --params-file src/robot/config/ekf_global.yaml \
  -r odometry/filtered:=odometry/filtered/global

ros2 run robot_localization navsat_transform_node --ros-args \
  -r __node:=navsat_transform_node \
  -p frequency:=10.0 \
  -p magnetic_declination_radians:=0.0 \
  -p yaw_offset:=0.0 \
  -p zero_altitude:=true \
  -p broadcast_utm_transform:=true \
  -p publish_filtered_gps:=true \
  -p use_odometry_yaw:=true \
  -p wait_for_datum:=false \
  -r gps/fix:=/sensor/vehicle_gnss/fix/raw \
  -r odometry/filtered:=odometry/filtered/local

ros2 run robot_localization navsat_transform_node --ros-args \
  -r __node:=spatial_navsat_transform_node \
  -p frequency:=10.0 \
  -p magnetic_declination_radians:=0.0 \
  -p yaw_offset:=0.0 \
  -p zero_altitude:=true \
  -p broadcast_utm_transform:=false \
  -p publish_filtered_gps:=false \
  -p use_odometry_yaw:=false \
  -p wait_for_datum:=false \
  -r imu:=/adnav_driver/imu \
  -r gps/fix:=/adnav_driver/nav_sat_fix \
  -r odometry/filtered:=odometry/filtered/local \
  -r odometry/gps:=/odometry/gps/spatial
```

CUDA YOLOはJetson上で動作確認済みのvenvを使用し、前方ZED2iとback_camの2本を
起動した。前方だけ`/virtual_obstacles`生成を有効にした。現在の実装では後方カメラ
ROIと前方MID360S点群の幾何対応を定義していないため、点群ブイ検出へ渡すROIは
前方の`/buoy_roi`のみである。

```bash
cd /home/ibo_asv/njord2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source _worktrees/yolo-best-2026-smoke/.venv/bin/activate
source _worktrees/yolo-best-2026-smoke/export_python_path.sh
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

ros2 run yolo yolo_cuda_node --ros-args \
  -r __node:=front_yolo -r __ns:=/zed2i \
  -p camera_topic:=/zed2i/left/image_rect \
  -p enable_roi:=true -p roi_topic:=/buoy_roi \
  -p enable_virtual_wall:=true

ros2 run yolo yolo_cuda_node --ros-args \
  -r __node:=back_yolo -r __ns:=/back_cam \
  -p camera_topic:=/back_cam/image_raw \
  -p enable_roi:=true -p roi_topic:=/back_buoy_roi \
  -p enable_virtual_wall:=false
```

Jazzy標準の`navigation_launch.py`はHumbleのTask1設定に存在しないRoute Server、
Collision Monitor、Docking Serverも無条件起動する。この試験ではTask1で使用する
従来のNav2ノードだけを、同じparameter fileで個別起動した。BT XMLはリポジトリ内の
ファイルを実行時指定した。

```bash
PARAMS=src/sim/task1_sim/config/task1_nav2_params.yaml
BT=/home/ibo_asv/njord2026_ws/src/robot/config/navigate_to_pose_w_replanning_and_recovery.xml

ros2 run nav2_controller controller_server --ros-args \
  --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav
ros2 run nav2_smoother smoother_server --ros-args --params-file "$PARAMS"
ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS"
ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file "$PARAMS" -p default_nav_to_pose_bt_xml:="$BT"
ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
  --params-file "$PARAMS"
ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
  --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -r __node:=lifecycle_manager_navigation \
  -p autostart:=true \
  -p "node_names:=['controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother']"
```

上記は説明のため1行ずつ記載している。実測時は各コマンドを別terminal相当の
background processとして同時実行した。

### 測定コマンド

`ros2 topic hz`を多数同時起動すると測定自体の負荷が大きくなるため、1つのrclpy
nodeから全topicを15秒間購読してcountと実効Hzを算出した。

```bash
cd /home/ibo_asv/njord2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 - <<'PY'
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

topics = {
    '/livox/lidar': 'sensor_msgs/msg/PointCloud2',
    '/livox/imu': 'sensor_msgs/msg/Imu',
    '/zed2i/left/image_rect': 'sensor_msgs/msg/Image',
    '/zed2i/depth/image': 'sensor_msgs/msg/Image',
    '/zed2i/points': 'sensor_msgs/msg/PointCloud2',
    '/back_cam/image_raw': 'sensor_msgs/msg/Image',
    '/adnav_driver/imu': 'sensor_msgs/msg/Imu',
    '/adnav_driver/nav_sat_fix': 'sensor_msgs/msg/NavSatFix',
    '/sensor/vehicle_gnss/fix/raw': 'sensor_msgs/msg/NavSatFix',
    '/zed2i/yolo/debug_image': 'sensor_msgs/msg/Image',
    '/back_cam/yolo/debug_image': 'sensor_msgs/msg/Image',
    '/virtual_obstacles': 'sensor_msgs/msg/PointCloud2',
    '/buoy_roi': 'njord_interfaces/msg/BuoyRoi',
    '/back_buoy_roi': 'njord_interfaces/msg/BuoyRoi',
    '/buoy_detections': 'geometry_msgs/msg/PointStamped',
    '/odometry/filtered/local': 'nav_msgs/msg/Odometry',
    '/odometry/filtered/global': 'nav_msgs/msg/Odometry',
    '/odometry/gps': 'nav_msgs/msg/Odometry',
    '/odometry/gps/spatial': 'nav_msgs/msg/Odometry',
    '/local_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/global_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
}

rclpy.init()
node = Node('full_stack_rate_probe')
counts = {topic: 0 for topic in topics}
first = {}
last = {}

def make_callback(topic):
    def callback(_msg):
        now = time.monotonic()
        counts[topic] += 1
        first.setdefault(topic, now)
        last[topic] = now
    return callback

subscriptions = [
    node.create_subscription(
        get_message(type_name), topic, make_callback(topic),
        qos_profile_sensor_data)
    for topic, type_name in topics.items()
]

end = time.monotonic() + 15.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

for topic in topics:
    count = counts[topic]
    span = last.get(topic, 0.0) - first.get(topic, 0.0)
    hz = (count - 1) / span if count > 1 and span > 0.0 else 0.0
    print(f'{topic}\tcount={count}\thz={hz:.2f}')

node.destroy_node()
rclpy.shutdown()
PY
```

追加確認には以下を使用した。

```bash
ros2 node list | sort
ros2 topic list -t | sort
ros2 topic info /virtual_obstacles -v
ros2 topic info /buoy_roi -v
ros2 topic info /odometry/filtered/local -v
ros2 topic echo /adnav_driver/nav_sat_fix --once
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
tegrastats --interval 1000
```

### 測定結果

| 系統 / topic | 設定値 | 15秒実測 | 結果 |
|---|---:|---:|---|
| MID360S `/livox/lidar` | 10 Hz | 9.68 Hz | 達成 |
| MID360S `/livox/imu` | 約200 Hz | 200.07 Hz | 達成 |
| Spatial `/adnav_driver/imu` | 20 Hz | 20.55 Hz | 達成 |
| Spatial `/adnav_driver/nav_sat_fix` | 20 Hz | 20.55 Hz | 達成 |
| UM982 `/sensor/vehicle_gnss/fix/raw` | 20 Hz | 1.00 Hz | 未達 |
| ZED2i `/zed2i/left/image_rect` | 15 Hz | 1.63 Hz | 未達 |
| ZED2i `/zed2i/depth/image` | 15 Hz | 2.59 Hz | 未達 |
| ZED2i `/zed2i/points` | 15 Hz | 1.46 Hz | 未達 |
| back_cam `/back_cam/image_raw` | 30 Hz | 8.48 Hz | 未達 |
| 前方YOLO `/zed2i/yolo/debug_image` | 指定なし | 1.76 Hz | 推論動作 |
| 後方YOLO `/back_cam/yolo/debug_image` | 指定なし | 6.06 Hz | 推論動作 |
| Nav2障害物 `/virtual_obstacles` | YOLO依存 | 3.77 Hz | 出力あり |
| local EKF `/odometry/filtered/local` | 30 Hz | 0 Hz | GLIM入力なし |
| global EKF `/odometry/filtered/global` | 30 Hz | 0 Hz | local EKF入力なし |
| local/global costmap | 2 / 1 Hz | 0 Hz | odom TFなし |

`/buoy_roi`、`/back_buoy_roi`、`/buoy_detections`は検出時のみpublishされる
event-driven topicである。試験時は画角内に認識対象のブイがなく、いずれも0件だった。
前後YOLOのdebug imageが出力されたため、実画像に対する推論実行自体は確認できた。

フル負荷時のJetsonはGPU使用率最大99%、RAM約5.9 / 7.3 GiBだった。ZED2iの
深度・点群とCUDA YOLO 2本の同時実行で計算資源が飽和しており、ZED2iとback_camが
設定周波数を満たさない主因と考えられる。

今後の軽量化項目と実装状況は
[`Docs/sensor_pipeline_performance_roadmap.md`](Docs/sensor_pipeline_performance_roadmap.md)
を参照する。

UM982は`/dev/ttyUSB2`を開いて連続受信できているため、USBハブ帯域が直接の原因とは
考えにくい。driverはGPGGA/UNIHEADINGを0.05秒周期に設定したと記録しているが、実出力は
1 Hzで、binary message ID 972のCRC mismatchも1件記録された。受信機がvolatileな
20 Hz設定を受理していない可能性を追加調査する。

Spatialはtopic rateを満たしたが、`NavSatFix.status.status`は`-1`で実fixではない。
UM982も試験時の座標値は0だった。そのためGNSS由来のglobal localizationは未成立である。

最大のblockerは`glim_ros`未導入である。`src/robot/config/glim_config`は存在するが、
実行パッケージはworkspace、`/opt/ros/jazzy`、APT cacheのいずれにもない。このため
GLIMが担当する`/odom`と`map -> odom -> base_link`が生成されず、EKFはtopicをadvertise
するだけで出力できない。Task1 Nav2はcontroller、Natural Cubic Spline smoother、
Theta* planner、BT Navigatorまでconfigureできたが、controller activate時に
`odom -> base_link`を待って停止した。

### GLIM導入後の再確認（2026-07-17）

上記試験後に`ros-jazzy-glim-cuda13.1`と
`ros-jazzy-glim-ros-cuda13.1` 1.2.2を導入し、GLIMを含めて同じ構成を再確認した。
試験時のnvpmodelは15Wである。GLIMの設定は確認時点ですでにodometry、sub mapping、
global mappingのすべてがGPU版JSONを選択していたため、CPU設定からの切り替えは
不要だった。

GLIM 1.2.2ではROS executable名が`glim_rosnode`、odometry出力が
`/glim_node/odom`である。このため`localization.launch.py`のexecutable名を更新し、
`/glim_node/odom`を既存EKF入力の`/odom`へremapした。1.2.2で追加された
`config_logging`、`compute_covs`、`validate_imu`も既定値と同じ値を明示した。

低負荷のMID360S＋localization試験では、GLIMが以下のGPU moduleをロードし、
Livox PointCloud2とIMUを購読して初期IMU state estimationまで完了した。

```text
libodometry_estimation_gpu.so
libsub_mapping.so (VGICP_GPU)
libglobal_mapping.so (VGICP_GPU)
```

別の低負荷試験ではGLIMの`/glim_node/odom`を約9.9 Hzで実測した。remap後は
`/odom`にGLIM publisherが1、local EKF、Nav2 controller、BT Navigatorのsubscriberが
計3接続されることも確認した。LivoxのPointCloud2が持つFLOAT64 nanosecond単位の
per-point timestampはGLIMが検出し、相対時刻へ変換して取り込んでいる。

15Wでのフルスタック再確認では以下を確認できたため、起動確認はOKとした。

- GLIMはGPU backendで起動し、`/odom`およびGLIMのpose、aligned points、map系topicを
  advertiseした。`/livox/lidar`にはGLIMと点群ブイ検出のsubscriberが接続した。
- ZED2iはcamera openに成功し、left image、depth、pointsをadvertiseした。
- 前後CUDA YOLOは初期化し、両方のdebug image、前方の`/virtual_obstacles`、前後ROI
  topicをadvertiseした。
- UM982、ESP32、back_cam、点群ブイ検出、local/global EKF、NavSat Transformが起動し、
  各topicをadvertiseした。
- Task1 Nav2はcontroller、Natural Cubic Spline smoother、Theta* planner、behavior、
  BT Navigator、waypoint follower、velocity smootherとlocal/global costmapを起動し、
  各topicをadvertiseした。

フル負荷時はGPU 95--98%、RAM約5.86 / 7.49 GiBで、処理遅延が大きかった。GLIMは
点群とIMUを受信したがTF生成まで追いつかず、Nav2 controllerのactivateは
`odom -> base_link`待ちでtimeoutした。またMID360S driverはPointCloud2/IMUの
publish開始後にsegfaultした。いずれも各nodeの起動、入力接続、必要topicの出現までは
確認できており、本確認の合格基準（負荷で周波数が低下しても、起動してtopicが出れば
成功）を満たす。試験後は関連プロセスをすべて停止した。
