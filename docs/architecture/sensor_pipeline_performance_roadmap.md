# センサパイプライン性能改善ロードマップ

ZED 2iとLivox MID-360SをJetson上で同時使用する際の、CPU/GPU負荷、メモリコピー、
ROS 2通信量を削減するための優先項目をまとめる。

## 現状

- MID-360Sは`/livox/lidar`を約10 Hzで配信できている。
- ZED 2iは15 Hz設定に対し、フル負荷時の実測が画像約1.6 Hz、depth約2.6 Hz、
  点群約1.5 Hzまで低下した。
- Python YOLOとGLIMはC++ component container外で動作するため、これらとの通信は
  DDS経由のままである。

## 優先項目

### 1. ZEDのlazy publishing

状態: 実装完了・実機評価待ち

購読者がいないright image、depth、PointCloud2は、publishだけでなくZED SDKの
`retrieveImage`、`retrieveMeasure`とROSメッセージ変換自体を実行しない。
`get_subscription_count()`と個別のpublish有効化パラメータを併用する。

SDK/CPUモードとも、Image、CameraInfo、depth、PointCloud2のsubscriber数を周期ごとに
確認する。必要な出力がない場合はframe取得を省略し、depth/points未購読時はdepth取得・
ステレオ計算・点群変換を行わない。SDKモードでは左右画像も個別にretrieveする。

完了条件:

- left imageだけを購読した場合、right/depth/pointsの取得・変換時間が発生しない。
- 購読開始・終了に追従して必要な出力だけが生成される。

### 2. 設定値をZED SDKモードへ反映

状態: 未実装

現在のSDKモードはHD720固定で、点群生成もstride=1固定である。YAMLの
`image_width`、`image_height`、`pointcloud_stride`をSDK初期化と点群生成へ反映する。
必要に応じてdepth modeも実行時パラメータ化する。

完了条件:

- `pointcloud_stride:=2`で生成点数が概ね1/4になる。
- 選択した解像度、FPS、depth modeが起動ログと実出力へ反映される。

### 3. YOLOを最新フレーム優先にする

状態: 実装完了・実機評価待ち

画像QoSを`KeepLast(1)`にし、推論中に届いた古いフレームを蓄積しない。
画像callbackと推論処理を分離し、常に最新画像だけを推論する。debug imageは無効化
パラメータまたは購読者数に応じて生成を省略する。

購読QoSをBest Effort、`KeepLast(1)`とし、ROS callbackはsingle-slot bufferの最新画像を
置換するだけにした。専用workerが推論中でもcallbackは次の最新画像を受け取れる。
debug image未購読時は矩形・文字・仮想壁線の描画と画像publishを省略する。

完了条件:

- 推論速度より入力FPSが高い場合も遅延が累積しない。
- debug image未購読時に描画・変換処理が発生しない。

### 4. ローカルコピーの削減

状態: 部分完了

実装済み:

- ZED SDKバッファを非所有`cv::Mat` viewとして扱い、`clone()`を廃止した。
- ImageとPointCloud2を`UniquePtr`で確保してmove publishする。
- ZED点群は一時点群vectorを介さず、最終`PointCloud2::data`へ直接構築する。
- Livox点群も一時vectorと最終一括memcpyを廃止し、最終バッファへ直接構築する。

残作業:

- 実機でメモリ帯域、CPU使用率、publish周期を変更前後比較する。
- 必要ならメッセージ用バッファの再利用・pool化を検討する。

### 5. component化とintra-process通信

状態: 部分完了

実装済み:

- ZED SDK/CPUノードをROS 2 componentとして登録した。
- PCLブイ検出をcomponentとして登録した。
- Livox driverとPCLブイ検出を同じ`component_container_mt`へロードできる。
- 対象componentで`use_intra_process_comms=true`を使用する。

起動例:

```bash
ros2 launch robot lidar.launch.py \
  lidar_model:=mid360s enable_buoy_detection:=true
```

残る境界:

- Python YOLOは同じC++ component containerへロードできない。
- GLIMが独立processで動作する構成では、LivoxからGLIMまではDDS経由になる。
- ZED componentと同一containerで動作するC++画像購読componentがない間は、
  ZED画像のintra-process効果は得られない。

### 6. ZEDから推論までGPU上で接続

状態: 未実装・長期項目

Python、ROS Image、cv_bridgeを経由せず、ZED SDKのGPUバッファをTensorRTや
NVIDIA Isaac ROS NITROS等へ渡す構成を検討する。実装規模は大きいが、画像の
CPU readback、色変換、DDS転送をまとめて削減できる可能性がある。

完了条件:

- カメラ取得から推論入力まで不要なCPUコピーがないことを計測で確認する。
- 現行YOLOモデルと同等の検出精度を維持する。
- フルスタック時のGPU使用率、RAM、入力FPS、推論遅延を記録する。

## 評価指標

各項目の変更前後で、最低限以下を同じnvpmodel・同じ入力条件で記録する。

- `ros2 topic hz`によるleft image、depth、ZED points、Livox pointsの実効周期
- カメラtimestampから検出結果publishまでのend-to-end遅延
- `tegrastats`のCPU、GPU、RAM、EMC使用率
- component内／外subscriber数と使用している通信経路
- 10分以上の連続運転でのdrop、segfault、メモリ増加

追記

確認できた範囲では、対象コードは commit `27de65a...` にあります。指定された `test07089` branch は GitHub API上では見つからなかったため、検索で取得できた該当実装を基に判断します。

## 結論

**Fast DDSをZenohへ変えるより先に、ZED→CPU→ROS Image→Python/CvBridge→GPUという往復をなくすべきです。**

現在は概ね次の経路です。

```text
ZED USB
  ↓
ZED SDK / GPUでdepth生成
  ↓ retrieveImage/retrieveMeasure
CPUメモリ
  ↓ clone()
cv::Mat
  ↓ memcpy()
sensor_msgs/Image
  ↓ DDSシリアライズ・コピー
Python rclpy
  ↓ CvBridge
NumPy / OpenCV CPU画像
  ↓ H2D転送
Ultralytics / PyTorch / TensorRT
GPU推論
```

この構成で右画像・depth・point cloudを追加すると、推論そのものより**メモリコピーとCPU処理**が先に限界になる可能性が高いです。

---

## 現在の実装で重い箇所

### 1. 現状は実質的にcomposable nodeではありません

`SdkNode` は `rclcpp::Node` を継承していますが、末尾では普通に `main()` から生成してspinしています。`RCLCPP_COMPONENTS_REGISTER_NODE` もなく、component libraryとして登録されていません。つまり「クラスになった」だけで、YOLOと同一プロセスのintra-process通信にはなっていません。

ROS 2のintra-process zero-copyを使うには、基本的には同一process内にcomponentとして載せ、intra-processを有効にし、対応する所有権形式でメッセージを渡す必要があります。単にcomposable形式に書き換えただけではGPUメモリのzero-copyにはなりません。([ROS Docs][1])

### 2. ZED画像をCPUへ取得後、明示的にcloneしています

```cpp
mat.getPtr(... sl::MEM::CPU)
return view.clone();
```

左・右・depthのすべてでフレーム全体をコピーしています。

HD720の場合、1フレーム当たり概算で、

* BGRA左: `1280 × 720 × 4 ≈ 3.7 MB`
* BGRA右: 約3.7 MB
* depth float32: 約3.7 MB

合計約11 MBです。15 FPSなら、cloneだけで少なくとも約166 MB/s相当のメモリ書き込みが生じます。

### 3. ROS Image化でもさらにmemcpyしています

`mat_to_image_msg()` 内でメッセージ用vectorをresizeして、その後にフレーム全体を`memcpy()`しています。

したがって現在は少なくとも、

```text
ZED GPU → CPU
CPU sl::Mat → cv::Mat clone
cv::Mat → ROS msg memcpy
ROS/DDS → subscriber
ROS msg → CvBridge / NumPy
NumPy → CUDA
```

という多段コピーです。

### 4. YOLOノードがPythonです

YOLO側では、受信したROS Imageを`CvBridge`でBGRの`cv::Mat`相当へ変換してから、Ultralyticsの`model.predict()`へ渡しています。

さらに毎フレーム、

* Python callback
* `CvBridge`
* BGR変換
* Ultralytics前処理
* NumPyからGPUへの転送
* boxごとの`.cpu().numpy()`
* デバッグ画像描画
* デバッグ画像再publish

が走ります。特に各boxの結果をGPUからCPUへ戻す部分と、デバッグ画像publishは、検出数や解像度次第で無視できません。

### 5. PointCloud生成がCPU全画素ループです

depthをpoint cloudへ変換する処理は、現在`stride=1`で全画素をCPU上で二重ループしています。HD720なら約92万画素を毎フレーム走査します。

しかも一度`std::vector<cv::Vec3f>`を作り、その後PointCloud2へ再度要素単位でコピーしています。

これはかなり大きなボトルネック候補です。

---

# 推奨する改善順序

## 優先度1：YOLOには左画像だけを入れ、depthは検出後に参照する

右・depthをYOLOの入力としてそれぞれ推論する必要は通常ありません。

推奨構成は、

```text
左画像
  ↓
YOLO推論
  ↓
bbox / mask
  ↓
同一timestampのdepth画像から
bbox中心・中央値・下端付近などのdepthを取得
  ↓
3D位置算出
```

です。

右画像はZED SDKがdepthを計算するために内部利用しています。ユーザー側YOLOへ右画像をもう一度入れても、同じ物体を少し異なる視点から二重検出するだけになりやすいです。

必要なのは「右画像もYOLO」ではなく、

* 左画像の検出結果
* depth map
* camera intrinsics
* 必要なら左右整合性・depth信頼度

の組み合わせです。

ブイ一個の距離推定なら、bbox内のdepthすべてをCPUへ持ってくる必要すらなく、GPU上で有効depthの中央値や下位百分位を計算できます。

---

## 優先度2：まずpoint cloud常時生成を止める

現在のコードでは`publish_pointcloud`のデフォルトが`true`です。

YOLOとブイの距離推定が目的なら、全画面point cloudを毎フレーム作る必要はありません。

まず以下にします。

```yaml
publish_pointcloud: false
```

point cloudが必要な場合も、

* 毎フレームではなく2～5 Hz
* `stride=2`または`4`
* ROI内だけ生成
* ZED SDKのGPU point cloudを必要時のみ利用

とするべきです。

`stride=4`なら計算対象画素数は約1/16になります。

---

## 優先度3：デバッグ画像を通常時はpublishしない

現在は毎フレーム必ずbboxを描画して、`yolo/debug_image`へ再publishしています。

パラメータを追加して、

```yaml
publish_debug_image: false
```

を通常運用にしてください。

さらに、デバッグ画像が必要な場合も、

* subscriberが存在するときだけ生成
* 1～5 FPSへ間引く
* 実験時のみ有効化

がよいです。

C++ならpublisherのsubscription countを見て、購読者がいなければ画像描画自体を省略できます。

---

## 優先度4：YOLOをTensorRT engine化する

現在はUltralyticsのモデルをロードして`model.predict()`しています。モデルが`.pt`なら、Jetson上ではPyTorch経由よりTensorRT engineを直接使う方が適しています。

候補は以下です。

```text
.pt
 ↓ export
FP16 TensorRT engine
```

さらに精度が許せば、

```text
INT8 TensorRT engine
```

です。

設定としては、

* 固定input shape
* batch size 1
* FP16
* TensorRT engineを起動時に一度だけロード
* CUDA streamを再利用
* 入出力bufferを事前確保
* 推論ごとのmalloc/freeを禁止
* resize、normalize、BGRA→RGBをGPU上で実施

が重要です。

YOLOXへの変更自体より、**どのモデルでもTensorRTへ落とし、GPU上の前処理から推論まで一続きにすること**の方が効きます。YOLOXを採用しても、CPU OpenCV→ROS→Python→GPUの経路を残すと根本解決になりません。

---

# 最終的に目指す構成

## 案A：C++ component + TensorRT + CUDA

最も性能を追求するならこれです。

```text
ComposableNodeContainer
├── ZedGpuComponent
│    ├── camera.grab()
│    ├── retrieveImage(..., MEM_GPU)
│    └── retrieveMeasure(..., MEM_GPU)
│
└── YoloTensorRtComponent
     ├── CUDA BGRA→RGB
     ├── CUDA resize/letterbox/normalize
     ├── TensorRT inference
     ├── CUDA NMS
     └── depth ROI reduction
```

ZED SDKは画像・depth・point cloudをGPUメモリへ直接取得できます。`retrieveMeasure(..., MEM_GPU)`が公式に提供されています。([StereoLabs Docs][2])

コードイメージは、

```cpp
sl::Mat left_gpu;
sl::Mat depth_gpu;

camera_.retrieveImage(left_gpu, sl::VIEW::LEFT, sl::MEM::GPU);
camera_.retrieveMeasure(depth_gpu, sl::MEASURE::DEPTH, sl::MEM::GPU);

auto* left_ptr = left_gpu.getPtr<sl::uchar1>(sl::MEM::GPU);
auto* depth_ptr = depth_gpu.getPtr<sl::float1>(sl::MEM::GPU);
```

です。

そのポインタをそのままCUDA kernelまたはTensorRT前処理へ渡します。

ここまで行けば、メイン推論経路では`cv::Mat`も`sensor_msgs::Image`も不要です。

ROSには軽量な結果だけを出します。

```text
Detection2DArray
BuoyRoi
Pose / PointStamped
diagnostics
```

画像はFoxglove表示や録画が必要なときだけ、低FPSで別経路にpublishします。

---

## 案B：Isaac ROS NITROSを使う

自前CUDA/TensorRT componentを全面的に書く負担を減らしたい場合は、Isaac ROSのNITROS系パイプラインも候補です。

NITROSは、対応node間でGPU bufferを受け渡し、通常のCPU中心のROS Image経路におけるコピーを減らすための仕組みです。特にJetson上の画像前処理・DNN推論パイプラインとの相性があります。([StereoLabs Docs][2])

ただし、現在の独自ZED driverとPython Ultralytics nodeをそのままNITROS化できるわけではありません。対応type adapterやGXF/NITROS側の構成が必要なので、実装量との比較になります。

既存ZED ROS wrapper、Isaac ROS DNN image encoder、TensorRT nodeなどを組み合わせられるなら有力です。

---

# 「カーネル操作」で改善できるところ

ここでいうカーネルがLinux kernelなら、優先度は低いです。USB driverやschedulerを調整しても、現在の多段コピー構造は解消しません。

CUDA kernelという意味なら大いに効果があります。

GPU上でまとめるべき処理は、

```text
BGRA → RGB
resize / letterbox
uint8 → FP16
normalization
HWC → CHW
depth ROI filtering
median / percentile reduction
NMS
```

です。

特に、

```text
ZED GPU buffer
 → CUDA preprocessing kernel
 → TensorRT input tensor
```

を同じCUDA stream上でつなげれば、CPUへの画像転送を避けられます。

さらに高度には、

* CUDA Graphsで毎フレームのlaunch overhead削減
* TensorRT custom pluginで前処理やNMS統合
* pinned host memory
* buffer pool
* asynchronous execution
* double/triple buffering
* CUDA eventsによる同期
* Jetson VICを使ったresize/color conversion

があります。

ただし、最初からcustom kernel最適化へ行くより、まずCPU往復とPython経路を消す方が大きく効きます。

---

# Composable化についての注意

C++ driverだけcomponent化して、Python YOLO nodeが別processのままでは、そこはDDS通信になります。

また、ROS 2 intra-processによるzero-copyは主にCPUメモリ上のROS message所有権移動です。GPU上の`sl::Mat`やCUDA bufferを自動でzero-copy転送するものではありません。

したがって段階は、

1. `SdkNode`をcomponent libraryとして登録
2. YOLOもC++ component化
3. 同一containerへロード
4. `use_intra_process_comms(true)`を設定
5. ROS Image経由を暫定的にintra-process化
6. 最終的にはGPU handle/type adapterまたは内部APIでGPU bufferを渡す

となります。

暫定版でも、現在の`publish()`へ一時オブジェクトを渡すより、loaned messageや`std::unique_ptr<Image>`を検討できます。ただし可変長`std::vector<uint8_t>`を含む`Image`は、DDS shared memoryやloaned messageの制約を受けやすく、「これだけで完全zero-copy」とは考えない方がよいです。

---

# Fast DDSとZenohの比較

## 同一Jetson内のZED→YOLOにはZenohはほぼ効きません

Zenohは特に、

* Jetsonと地上PC間
* Wi-Fi越し
* 複数ロボット
* DDS discovery traffic削減
* topicごとの帯域制御
* 遠隔監視

では有効です。

一方、同じJetson内で大容量画像をdriverからYOLOへ渡す処理については、

```text
GPU bufferをCPUへ落としてROS Image化する
```

こと自体が問題です。Fast DDSからZenohへ変えても、このGPU→CPUコピーやCvBridge、Python→GPU転送は消えません。

さらに`zenoh-bridge-ros2dds`はDDSとZenohのbridgeであり、ローカルROS 2 node側ではDDSを使用します。公式説明でも、bridgeはローカルDDS entityを発見してZenohへrouteする構成です。([GitHub][3])

したがって用途は分離すべきです。

```text
Jetson内部の画像処理:
  composition / GPU zero-copy / NITROS / CUDA

Jetson ↔ 地上PC:
  Zenoh bridge
```

地上PCへはrawの左右画像とfloat depthを常時送らず、

* 検出結果
* diagnostics
* 必要時のみ圧縮画像
* depth可視化画像
* 低密度point cloud

を送る設計がよいです。

---

# すぐ実行できる現実的な第一段階

大規模改修前に、以下だけでも効果を確認できます。

1. `publish_pointcloud:=false`
2. ZED depth modeを`QUALITY`から`PERFORMANCE`または用途に適した軽量モードへ変更
3. debug imageを無効化
4. YOLO入力を640×640またはそれ以下に固定
5. TensorRT FP16 engineを使用
6. YOLO推論を毎カメラフレームではなく10 FPS程度に制限
7. callbackが詰まったら古い画像を捨て、常に最新フレームだけを推論
8. QoS depthを1、best effortに統一
9. depthはbbox周辺だけ参照
10. 右画像topicは必要なsubscriberがいる場合だけretrieve/publish

現在driverは、subscriberがいなくても毎回左・右・depthを取得し、ROS messageを作っています。

ここを、

```cpp
if (right_image_pub_->get_subscription_count() > 0) {
  camera_.retrieveImage(...);
  publish(...);
}
```

のようにするだけでも、未使用データのコピーを削れます。

depthもYOLO componentから内部的に使うなら、外部topic用のdepth message作成はsubscriberがいる場合だけで構いません。

---

## 推奨する実装ロードマップ

**まず1日程度の変更範囲**

```text
pointcloud off
debug image off
QoS depth=1
latest-frame-only
TensorRT FP16
不要topicのlazy publish
```

**次の段階**

```text
zed2i_driverを真のcomponent化
YOLOをC++ TensorRT node化
同一component container化
```

**最終段階**

```text
ZED MEM_GPU
CUDA前処理
TensorRT
depth ROI処理
ROSには検出結果だけpublish
```

この最終構成なら、左画像推論に加えてdepthを利用しても、現在より軽くできる可能性があります。右画像はZED SDKのdepth生成に任せ、別YOLO推論は原則不要です。Zenoh導入はその後、地上PCとの画像・可視化通信を整理する目的で行うのが適切です。

[1]: https://docs.ros.org/en/jazzy/Tutorials/Demos/Intra-Process-Communication.html?utm_source=chatgpt.com "Setting up efficient intra-process communication — ROS 2 Documentation: Jazzy documentation"
[2]: https://docs.stereolabs.com/docs/development/zed-sdk/modules/depth-sensing/using-the-api?utm_source=chatgpt.com "Using the Depth Sensing API | StereoLabs"
[3]: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?utm_source=chatgpt.com "GitHub - eclipse-zenoh/zenoh-plugin-ros2dds: A Zenoh plug-in for ROS2 with a DDS RMW. See https://discourse.ros.org/t/ros-2-alternative-middleware-report/ for the advantages of using this plugin over other DDS RMW implementations. · GitHub"
