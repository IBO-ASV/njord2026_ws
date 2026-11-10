# ZED 2i / YOLOv10 / MID-360S GPU統合知覚 実装仕様書

## 1. この文書の目的

この文書は、`zed2i_driver` とYOLO周辺の性能改善を実装する担当者向けの、
判断済み事項を含む実装仕様である。実装担当者は、ここに記載された公開interface、
データフロー、fallback、失敗時挙動を変更せずに実装すること。

最終目標は次のとおり。

- ZED左画像をROS `Image`へ変換せず、ZED SDKのGPU bufferからTensorRTへ渡す。
- ZED depthもGPU上で参照し、YOLO bbox内の距離を算出する。
- bboxはcomponent内部だけで使用し、通常運用では`/buoy_roi`をpublishしない。
- MID-360S点群でZED由来の3D位置を精密化または補完する。
- ROSへは最終3D検出配列と通常Nav2用の仮想壁だけをpublishする。
- Python YOLOと既存PCL ROI経路は比較・fallback用として残すが、実機の既定経路にはしない。

この文書でいう「ROSを介さない」は、ZED画像、ZED depth、bboxの主推論経路を
ROS messageにしないという意味である。MID-360Sは既存driverが`PointCloud2`を出すため、
同一process内のROS 2 intra-process通信を使用する。

Task 2は他船回避に特化した独立構成とし、このブイ検知経路の対象外とする。Task 2では
旧Python YOLO、YOLO11s、新GPUブイ検知のいずれも起動せず、`/buoy_roi`、
`/yolo/detections`、`/buoy_detections_3d`、`/virtual_obstacles`をブイ検知目的でpublishしない。
Task 2のMID-360S cloud filter、他船tracking、opponent selector、MPPIは維持する。

## 2. 現状と制約

### 2.1 現行実装

- `src/driver/camera/zed2i_driver/src/sdk_node_zed.cpp`
  - ZED SDK nodeは既にROS 2 componentである。
  - subscriberがない画像、depth、点群は取得・変換しないlazy publishingがある。
  - SDK取得はCPU memoryであり、TensorRT推論はない。
- `src/detection/yolo/yolo/main.py`
  - Python、CvBridge、Ultralyticsを使用する。
  - 入力画像は`KeepLast(1)`相当のsingle-slot bufferで最新フレーム優先になっている。
  - `/buoy_roi`、`/virtual_obstacles`、購読時だけdebug imageをpublishする。
- commit `77199fcf09a01234c265dab6e2fa9fcb0a16a112`では、Task 2用に
  `src/detection/yolo/yolo/yolo11_node.py`も追加されている。
  - 外部配置したYOLO11s `.pt`または`.engine`をUltralytics経由で実行する。
  - `/yolo/detections`、`/buoy_roi`、debug imageをpublishするが、Task 2制御ループには
    接続されていない。
  - 本仕様のYOLOv10 TensorRT engineとは別artifactであり、相互流用しない。
- `src/detection/pcl_det/src/pcl_bouy_det_node.cpp`
  - `/buoy_roi`を使いLivox点群を角度・距離で絞り、`/buoy_detections`をpublishする。
- 学習済みmodelは`src/detection/yolo/config/best.pt`にある。
  - architecture: YOLOv10s
  - class順: `green`, `red`, `north`, `east`, `south`, `west`

### 2.2 Task 2との境界

commit `77199fcf09a01234c265dab6e2fa9fcb0a16a112`のTask 2実機構成には、YOLO11s、
`task2_cloud_filter`、classical ship tracking、`opponent_selector`、MPPI、Task 2 Nav2が含まれる。
このうちブイ検知に属するYOLO11sだけをTask 2構成から除外する。LiDARによる他船認識と
回避制御は別機能なので削除、停止、または新GPUブイ検知へ置換してはならない。

Task 2 Nav2のobstacle sourceは`/pointcloud`であり、`/virtual_obstacles`を購読しない。
またTask 2のcontroller設定ではcostmap上のブイ仮想壁を回避制御へ伝搬する契約がない。
したがって、本仕様の`/virtual_obstacles`は通常Nav2専用とし、Task 2 launchや
`nav2_params_task2.yaml`へ接続しない。

### 2.3 開発端末

この文書作成時の端末はx86_64、GeForce RTX 4070 Laptop GPU、CUDA Toolkit 12.6である。
ZED SDK、ZED 2i本体、TensorRT header/libraryはない。このため以下を分離する。

- この端末で常時可能:
  - TensorRT/ZEDなしのcompile path
  - 純粋ロジックのunit test
  - message、launch、設定ファイルの検査
- TensorRT導入後にこの端末で可能:
  - x86用engineの生成
  - CUDA前処理、TensorRT出力decode、静止画推論比較
- Jetson Orin Nano Super + ZED 2i + MID-360Sでのみ完了可能:
  - ZED `MEM_GPU`接続
  - Jetson用engine
  - camera/LiDAR融合
  - `tegrastats`、Nsight Systems、連続運転評価

TensorRT engineは原則として生成したplatform、TensorRT version、GPUに依存する。
x86 RTX用engineをJetsonへコピーして使用してはならない。engineが不一致の場合は
再生成すること。

## 3. 対象範囲

### 3.1 今回実装するもの

1. 新しい3Dブイ検出message。
2. YOLOv10を静的ONNXへexportするtool。
3. 対象GPU上でFP16 TensorRT engineを事前生成するtool。
4. C++ TensorRT runtime wrapper。
5. CUDA画像前処理。
6. ZED GPU depthのbbox内中央値算出。
7. ZED SDK nodeへの最新フレーム優先GPU推論worker。
8. MID-360S点群の投影、cluster選択、3D位置の精密化・補完。
9. 6クラス対応の仮想壁生成。
10. ZED componentとLivox componentを同じcontainerへ載せるbringup。
11. unit test、GPU test、実機評価手順、関連文書更新。

### 3.2 今回実装しないもの

- INT8量子化、calibration dataset整備。
- DLA対応。
- Isaac ROS NITROSへの移行。
- CUDA Graphs、TensorRT custom pluginによる追加最適化。
- Python YOLO package、`BuoyRoi`、既存PCL node自体の削除。ただしTask 2 launchから
  ブイ検知nodeと専用argumentを除去することは移行作業に含む。
- `/virtual_obstacles`生成の別node化。
- MID-360S driver自体のGPU化。
- ブイtracking、同一ブイIDの時系列保持。Task 2の既存他船trackingは独立経路として維持する。

## 4. 完成時のデータフロー

```text
ZED 2i
  |
  | camera.grab()
  v
ZED SDK GPU buffers (double buffer)
  |-- left BGRA8 -------------------------------+
  |                                             |
  |     CUDA BGRA->RGB / letterbox / FP16       |
  |                    |                        |
  |                    v                        |
  |              TensorRT YOLOv10               |
  |                    |                        |
  |          decoded boxes/classes/scores       |
  |                    |                        |
  |-- depth float32 ---+--> center ROI median   |
  |                              |              |
  |                              v              |
  |                         ZED 3D candidate     |
  |                                             |
MID-360S -> Livox component -> PointCloud2      |
                  intra-process |               |
                                v               |
                    project into internal bbox  |
                                |
                    cluster and select target   |
                                |
                LiDAR-refined/fallback 3D point |
                                |               |
                                +---------------+
                                |
                  +-------------+-------------+
                  |                           |
                  v                           v
       /buoy_detections_3d          /virtual_obstacles
       BuoyDetectionArray           PointCloud2 (map, 通常Nav2専用)
```

通常運用では次を推論のためにpublishしてはならない。

- `/zed2i/left/image_rect`
- `/zed2i/depth/image`
- `/zed2i/points`
- `/buoy_roi`
- bboxを描画したdebug image

ただしZED driverの既存画像・depth topic自体は削除しない。subscriberが明示的に存在する
場合だけ、既存lazy publishing経路でCPU readbackしてpublishしてよい。

## 5. 公開ROS interface

### 5.1 `BuoyDetection.msg`

`src/njord_interfaces/msg/BuoyDetection.msg`を次の内容で追加する。

```text
uint8 CLASS_GREEN=0
uint8 CLASS_RED=1
uint8 CLASS_NORTH=2
uint8 CLASS_EAST=3
uint8 CLASS_SOUTH=4
uint8 CLASS_WEST=5

uint8 POSITION_NONE=0
uint8 POSITION_ZED_DEPTH=1
uint8 POSITION_LIDAR_FUSED=2

uint8 class_id
float32 confidence
geometry_msgs/Point position
uint8 position_source
```

規則:

- `class_id`はTensorRT modelのclass indexをそのまま格納する。
- `confidence`は0.0以上1.0以下。
- 3D位置を得られない2D検出もmessageから捨てない。
- 位置不明時は`position.x/y/z`をすべてquiet NaN、`position_source=POSITION_NONE`とする。
- ZEDだけで位置が得られた場合は`POSITION_ZED_DEPTH`。
- 有効なLiDAR clusterを採用した場合は、ZED depthの有無にかかわらず
  `POSITION_LIDAR_FUSED`とする。

### 5.2 `BuoyDetectionArray.msg`

`src/njord_interfaces/msg/BuoyDetectionArray.msg`を次の内容で追加する。

```text
std_msgs/Header header
njord_interfaces/BuoyDetection[] detections
```

規則:

- topic名は`/buoy_detections_3d`。
- 1回のcamera推論につき1 messageをpublishする。
- `header.stamp`は対応するZED grab直後に採ったROS時刻。
- `header.frame_id`は既定で`base_link`。
- 検出0件でも空配列をpublishする。consumerが古い検出を保持し続けるのを防ぐためである。
- bbox座標はこの公開messageへ含めない。

### 5.3 interface build更新

`src/njord_interfaces/CMakeLists.txt`の`msg_files`へ2ファイルを追加する。
`geometry_msgs`と`std_msgs`は既にinterface依存に含まれているため再利用する。

### 5.4 既存topicとの関係

| Topic | Type | 新C++経路 | 旧`main.py`経路 | YOLO11s経路 | Task 2 |
|---|---|---:|---:|---:|---:|
| `/buoy_detections_3d` | `BuoyDetectionArray` | publish | publishしない | publishしない | publishしない |
| `/virtual_obstacles` | `PointCloud2` | publish | publish | publishしない | publishしない |
| `/yolo/detections` | `Detection2DArray` | publishしない | publishしない | publish | publishしない |
| `/buoy_roi` | `BuoyRoi` | publishしない | publish | publish | publishしない |
| `/buoy_detections` | `PointStamped` | publishしない | publishしない | publishしない | publishしない |

新C++経路と旧`main.py`経路を同時起動すると`/virtual_obstacles`のpublisherが競合するため、
launchで禁止する。YOLO11sは同topicをpublishしないが、ZED画像のCPU readback、重複推論、
GPU資源競合を発生させるため、新C++経路と同時起動しない。Task 2では三経路すべてを
起動しない。

## 6. packageとファイル構成

新しいruntimeは`zed2i_driver`内へ実装する。ZED GPU pointerをROS messageや公開ABIで
別componentへ渡さず、同じSDK nodeから直接呼び出すためである。

推奨する追加ファイル:

```text
src/driver/camera/zed2i_driver/
  include/zed2i_driver/
    perception_types.hpp
    tensor_rt_detector.hpp
    lidar_fusion.hpp
    virtual_wall.hpp
  src/
    tensor_rt_detector.cpp
    cuda_preprocess.cu
    cuda_depth_median.cu
    lidar_fusion.cpp
    virtual_wall.cpp
  test/
    test_detection_decode.cpp
    test_depth_policy.cpp
    test_lidar_fusion.cpp
    test_virtual_wall.cpp
    test_message_conversion.cpp

src/detection/yolo/
  tools/
    export_yolov10_onnx.py
    build_tensorrt_engine.sh
    inspect_tensorrt_engine.py
```

実装を過度に1ファイルへ集約しない。`sdk_node_zed.cpp`はcamera、ROS parameter、thread、
publisherの組み立てを担当し、推論、融合、壁geometryのアルゴリズムは上記classへ委譲する。

## 7. build構成と依存関係

### 7.1 必須依存

`zed2i_driver/package.xml`とCMakeへ次を追加する。

- `geometry_msgs`
- `njord_interfaces`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `pcl_conversions`
- `pcl_ros`
- test時の`ament_cmake_gtest`

PCLはbboxへ投影した後の少数点だけをcluster化するために使用する。入力全点群を
`pcl::fromROSMsg()`でコピーしないこと。`sensor_msgs::PointCloud2ConstIterator<float>`で
xyzを読み、bbox候補点だけを`pcl::PointCloud<pcl::PointXYZ>`へ追加する。
CMakeでは`find_package(PCL REQUIRED COMPONENTS common kdtree segmentation)`を使い、
必要componentを明示する。

### 7.2 optional GPU build

CMake optionを追加する。

```cmake
option(ZED2I_DRIVER_BUILD_GPU_PERCEPTION
  "Build CUDA/TensorRT buoy perception when dependencies are available" ON)
```

検出条件:

1. `ZED_FOUND`
2. `CUDAToolkit_FOUND`
3. `NvInfer.h`
4. `libnvinfer`
5. CUDA compiler

すべて揃う場合だけCUDA/TensorRT sourceをcompileし、
`ZED2I_DRIVER_HAS_GPU_PERCEPTION=1`をdefineする。一つでも不足する場合:

- CPU node、SDK stub、GPU知覚以外の既存targetはbuild可能にする。
- CMake configure時に不足依存を明示する。
- `enable_gpu_perception=false`なら既存SDK nodeを使用可能にする。
- `enable_gpu_perception=true`でGPU知覚未compileなら、node constructorでfatal logを出して
  起動失敗させる。暗黙のPython fallbackは禁止する。

### 7.3 CUDA architecture

- Jetson用buildではJetPackが提供する既定architectureを使用する。
- repositoryへ特定のdesktop GPU向け`CMAKE_CUDA_ARCHITECTURES`を固定しない。
- CIや開発端末では明示overrideを許可する。
- CUDA sourceはC++17と互換にする。

## 8. model exportとengine生成

### 8.1 artifact方針

- source of truth: `src/detection/yolo/config/best.pt`
- 生成物: `best_yolov10s_640.onnx`
- 対象機生成物: `best_yolov10s_640_fp16.engine`
- `.onnx`と`.engine`はGit管理しない。
- `.gitignore`へ最低限`*.engine`と、生成先directoryの`*.onnx`を追加する。
- engineは起動時に自動生成しない。

commit `77199fcf09a01234c265dab6e2fa9fcb0a16a112`でもtracked `best.pt`のGit blobは
基点`834f433`から変わっていない。本仕様では引き続き6クラスYOLOv10sをsource of truthとする。
Task 2向けYOLO11s重みはrepositoryに含まれない別modelであり、YOLO11s用export script、
manifest、engineをこのruntimeへ入力してはならない。将来modelをYOLO11sへ移行する場合は、
class順、tensor契約、Python比較oracle、artifact名をまとめて再仕様化する。

### 8.2 ONNX export仕様

`export_yolov10_onnx.py`は次を固定する。

- input size: 640 x 640
- batch: 1
- dynamic shape: false
- opset: 17
- simplify: true
- precision: FP32 ONNX。FP16化はTensorRT build時に行う。
- class数: 6
- class順: green, red, north, east, south, west
- outputはdecode済み検出。1検出は`x1,y1,x2,y2,confidence,class_id`の6値。

export後にONNX modelを読み、次を検査して不一致ならnon-zero終了する。

- inputが1個でshape `[1, 3, 640, 640]`
- outputが1個
- outputの1検出あたり要素数が6
- class metadataが上記6クラスと一致

Ultralytics versionはtoolのrequirementsまたは文書に固定し、export logへversion、model SHA-256、
ONNX SHA-256を出す。

### 8.3 TensorRT engine build仕様

`build_tensorrt_engine.sh`はJetson上で`trtexec`を呼ぶ薄いwrapperとする。

- input ONNXとoutput engineを必須引数にする。
- `--fp16`
- `--minShapes`, `--optShapes`, `--maxShapes`をすべて`1x3x640x640`にする。
- build logをengineと同じdirectoryへ保存する。
- build後に`inspect_tensorrt_engine.py`を実行する。
- model/ONNX SHA-256、TensorRT version、CUDA version、GPU名をmanifest JSONへ保存する。

runtimeはmanifestと現在環境を検査する。engine deserialize失敗、binding不一致、manifest不在、
model hash不一致のいずれかなら起動失敗する。自動再buildはしない。

### 8.4 runtime tensor契約

TensorRT wrapperはbinding名そのものをhard-codeせず、入出力数、dtype、shapeから検査する。

- input: 1個、FP16またはFP32、`[1,3,640,640]`
- output: 1個、FP16またはFP32
- output layoutは`[1,N,6]`または`[1,6,N]`だけを許可する。
- `N`は正の固定値。
- それ以外はfatal error。

outputが`[1,6,N]`の場合はdecoder内でindexだけを読み替え、大きなtranspose bufferを作らない。
confidence threshold適用後、confidence降順で最大32件までを採用する。

## 9. C++内部型

`perception_types.hpp`にはROS非依存の型を置く。

```cpp
struct Detection2D {
  int class_id;
  float confidence;
  float x1;
  float y1;
  float x2;
  float y2;
};

enum class PositionSource : uint8_t {
  kNone = 0,
  kZedDepth = 1,
  kLidarFused = 2,
};

struct PositionedDetection {
  Detection2D detection;
  std::array<float, 3> position_base;
  PositionSource source;
};
```

不明位置は`position_base`の3要素すべてをNaNにする。`std::optional`だけに依存せず、
公開messageへ変換する直前にもsourceとfinite性の整合をassertまたは検査する。

## 10. ZED GPU frame処理

### 10.1 buffer所有権

最低2slotの永続bufferを用意する。各slotは次を保持する。

- `sl::Mat left_gpu`
- `sl::Mat depth_gpu`
- frame stamp
- CUDA completion event
- state: `FREE`, `PENDING`, `RUNNING`

slotの`sl::Mat`はcamera解像度確定後に一度確保する。frameごとにconstruct/destructしない。

### 10.2 grab thread

既存timer callbackは次の順序にする。

1. 外部topic subscriberと`enable_gpu_perception`を確認する。
2. 何も必要なければ`camera.grab()`自体を省略する。
3. `camera.grab()`成功直後に`stamp=now()`を保存する。
4. GPU知覚が有効なら書込み可能slotを確保する。
5. `retrieveImage(slot.left_gpu, LEFT, MEM_GPU)`を実行する。
6. `retrieveMeasure(slot.depth_gpu, DEPTH, MEM_GPU)`を実行する。
7. slotを`PENDING`としてworkerへ通知する。
8. 外部subscriberがあるtopicだけ、既存CPU取得・message化経路を別途実行する。

workerが追いつかず`PENDING`が既にある場合は古いpending frameを捨て、最新frameで置換する。
`RUNNING` slotは上書きしない。空slotがなければそのcamera frameのGPU知覚をdropし、
drop counterを増やす。callbackを待たせて遅延を蓄積してはならない。

### 10.3 inference worker

- nodeにつきworker threadは1本。
- TensorRT execution contextはworker専用で共有しない。
- CUDA streamは起動時に1本作り再利用する。
- ZEDによるslot書込み完了を保証してからCUDA前処理をenqueueする。
- 前処理、TensorRT enqueue、depth中央値処理、小さいoutput copyを同じstream上で順序付ける。
- 終了時はstop flag、condition variable通知、worker join、CUDA event/stream破棄、camera closeの順。
- ROS shutdown時に無期限joinしない。

## 11. CUDA画像前処理

inputはZEDのBGRA8 GPU pointerとpitchである。次を1回または少数kernelで行う。

1. 1280x720等の元画像から640x640へletterbox resize。
2. BGRAからRGBへchannel変換。
3. `uint8`からengine input dtypeへ変換。
4. 0..1 normalize。
5. HWCからNCHWへ変換。

letterbox padding値は114。scaleと左右上下paddingをframe metadataへ保存し、engine出力bboxを
元ZED画像座標へ戻すときに使用する。bboxは画像範囲へclipし、幅または高さが1 pixel未満なら
破棄する。

frameごとのhost/device allocation、OpenCV CUDA、CvBridge、NumPyは禁止する。

## 12. ZED depthによる3D化

### 12.1 対象ROI

各bboxの中央50%を使用する。

```text
roi_width  = bbox_width  * depth_center_ratio
roi_height = bbox_height * depth_center_ratio
```

既定`depth_center_ratio=0.5`。bbox中心を維持して縮小し、ZED元解像度へmapする。

### 12.2 有効depth

次をすべて満たすfloatだけを有効とする。

- finite
- `depth_min_m <= depth <= depth_max_m`
- ZED SDKが返すinvalid sentinelではない

既定で2 pixelおきにsampleする。sample数が`min_valid_depth_samples=16`未満なら無効。

### 12.3 GPU中央値

- ROIから有効depthを事前確保bufferへcompactするkernelを用意する。
- countはdevice memory上で管理する。
- CUB `DeviceRadixSort`等を使いfloatをsortし中央要素を得る。
- 最大ROI用のdata bufferとCUB workspaceは起動時に確保する。
- detectionごとに処理してよいが、frameごとのallocationは禁止する。
- 最大検出数32を超える候補はdepth処理前にconfidence順で切る。

### 12.4 pixelからcamera 3Dへの変換

median depthを`z`、bbox中心pixelを`u,v`としてZED optical frame上で次を計算する。

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = median_depth
```

ZED optical frameのx右、y下、z前という向きを独自に並べ替えない。TFへそのまま渡し、
URDF/static TFで`base_link`へ変換する。

## 13. MID-360S融合

### 13.1 点群cache

- topic: `/livox/lidar`
- QoS: `rclcpp::SensorDataQoS()`
- 通常ブイbringupではGLIMとGPU融合が同topicを購読する。Task 2ではGPU融合を起動せず、
  GLIMと`task2_cloud_filter`が購読する。両モードを混在させず、それぞれのsubscriber数と
  serialization/copyを負荷試験で記録する。
- GPUブイ融合はraw cloudを使用し、`/task2/points_filtered`へ切り替えない。同topicは他船認識用に
  水面、自船、rangeを除去したcloudであり、ブイ融合の入力契約とは分離する。
- callbackは`PointCloud2::ConstSharedPtr`とstampを最大5件のdequeへ保持する。
- mutex保持中に投影やclusterを実行しない。選択したshared pointerだけをcopyしてlockを外す。
- camera stampとの差の絶対値が最小のcloudを選ぶ。
- 差が`lidar_max_age_sec=0.15`を超えたcloudは使用しない。
- stampが0のcamera/cloudは融合に使用しない。

### 13.2 投影

camera stampで`cloud.header.frame_id -> ZED optical frame`のTFを取得する。
TF timeout既定値は0.05秒。TFを取得できない場合はLiDAR融合だけをskipし、ZED結果は残す。
MID-360Sの上下反転や取付姿勢を融合code内で補正してはならない。URDFの
`livox_frame -> base_link`を唯一の補正元とし、Task 2由来の緊急manual pre-rotationも適用しない。
commit `77199fcf09a01234c265dab6e2fa9fcb0a16a112`のroll=piと取付位置は公称値なので、実機で
roll/pitch/yaw/x/y/zを実測し、GLIM、Task 2 cloud、ブイ投影の全経路で同じTFを使用する。

各finite xyz点について:

1. camera optical frameへ変換する。
2. `z <= 0`を除外する。
3. `u = fx*x/z + cx`, `v = fy*y/z + cy`で画像へ投影する。
4. 画像外を除外する。
5. bbox中央ROI内に入る点を、そのdetectionの候補へ追加する。

重なるbboxへ入る点は、正規化したbbox中心距離が最小のdetection一つだけへ割り当てる。
同一点を複数のブイ位置に使用しない。

### 13.3 clustering

detectionごとの候補点だけをPCL Euclidean clusteringへ渡す。

既定値:

- `lidar_cluster_tolerance_m=0.15`
- `lidar_min_cluster_points=5`
- `lidar_max_cluster_points=5000`

各clusterについてcentroid、cameraからのrange、bbox中心rayとの角度差を計算する。

### 13.4 cluster選択

ZED位置が有効な場合:

1. ZED rangeとの差が最小のclusterを選ぶ。
2. 差が`fusion_max_range_delta_m=2.0`以下の場合だけ採用する。
3. 採用したLiDAR centroidを最終位置とする。
4. 条件を満たすclusterがなければZED位置を維持する。

ZED位置が無効な場合:

1. bbox中心rayとの角度差が最小のclusterを選ぶ。
2. 同差ならcameraに近いclusterを選ぶ。
3. 角度差がbbox中央ROIの半角以内の場合だけ採用する。
4. clusterがなければNaN位置をpublishする。

LiDAR centroidはcamera frameから`base_link`へcamera stampで変換する。TF失敗時は採用しない。

## 14. 6クラス仮想壁

この章の仮想壁は通常Nav2専用である。Task 2では生成も購読もせず、Task 2のMPPI、
controller、costmapへ接続しない。

### 14.1 共通規則

- output topic: `/virtual_obstacles`
- type: `sensor_msgs/msg/PointCloud2`
- frame: `map`
- 各点はfloat32 x/y/z、z=0.0。
- `position_source=NONE`の検出は使用しない。
- `base_link -> map`を検出stampでlookupする。
- TFがない場合、そのframeの壁は生成せずthrottled warningを出す。
- 検出0件または全位置無効でも空PointCloud2をpublishする。
- `enable_virtual_wall=false`の場合はpublisher自体を作らない。

### 14.2 cardinal mark

`map`をENU、x東、y北とする。安全方位中心角は次のとおり。

| Class | 安全方位角 |
|---|---:|
| north | `pi/2` |
| east | `0` |
| south | `-pi/2` |
| west | `pi` |

安全方位の±45度を空け、それ以外の270度円弧へ点を置く。

```text
start = safe_angle + pi/4
end   = safe_angle + 7*pi/4
point = buoy_position + wall_radius * [cos(angle), sin(angle)]
```

### 14.3 red/green lateral mark

IALA Region Aを前提とする。`channel_heading_rad`はmap x軸から反時計回りに測った、
航路へ進入する向きと定義する。launch/configから必須で与える。

- red: 航路左舷境界。危険側中心角は`channel_heading_rad + pi/2`。
- green: 航路右舷境界。危険側中心角は`channel_heading_rad - pi/2`。

危険側中心角の±90度に半円弧を生成する。

同色の lateral mark が2個以上ある場合は、`channel_heading_rad` への投影順で
隣接するブイ同士を直線の点群で結ぶ。これにより赤列・緑列それぞれが航路外側の
連続した仮想壁になる。`same_color_wall_max_gap_m` より離れた点は結ばないため、
欠落や誤検出をまたいだ壁は作らない。方位が未設定の場合は、半円壁・同色連結壁の
どちらも生成しない。

### 14.4 既定geometry

- `wall_radius_m=2.0`
- `wall_points_per_full_circle=40`
- `connect_same_color_buoys=true`
- `same_color_wall_max_gap_m=12.0`
- `same_color_wall_point_spacing_m=0.2`
- 270度arcは31点、180度arcは21点。両端を含める。
- 同じframe内で複数検出があれば全点を1つのPointCloud2へまとめる。
- NaN、infの点をmessageへ入れない。

## 15. ROS parameter

Jetson用YAMLへ以下を追加する。値は特記がない限り実装初期値にも合わせる。

```yaml
zed2i:
  ros__parameters:
    enable_gpu_perception: true
    engine_path: ""
    detection_topic: /buoy_detections_3d
    output_frame: base_link
    confidence_threshold: 0.25
    max_detections: 32

    depth_center_ratio: 0.5
    depth_sample_stride: 2
    min_valid_depth_samples: 16

    lidar_topic: /livox/lidar
    lidar_max_age_sec: 0.15
    lidar_tf_timeout_sec: 0.05
    lidar_cluster_tolerance_m: 0.15
    lidar_min_cluster_points: 5
    lidar_max_cluster_points: 5000
    fusion_max_range_delta_m: 2.0

    enable_virtual_wall: true
    virtual_wall_topic: /virtual_obstacles
    wall_frame: map
    channel_heading_rad: .nan
    wall_radius_m: 2.0
    wall_points_per_full_circle: 40

    publish_pointcloud: false
```

validation規則:

- GPU知覚有効時、`engine_path`が空、読めない、manifest不一致なら起動失敗。
- 仮想壁有効かつred/greenを処理する場合、`channel_heading_rad`がfiniteでなければ
  そのクラスの壁だけをskipしてerror diagnosticを出す。camera推論自体は止めない。
- ratioは`0 < value <= 1`。
- stride、sample数、cluster点数、最大検出数は正数。
- timeout、距離、半径は非負。
- runtime中のengine、解像度、frame名変更は対応しない。parameter callbackでrejectする。

## 16. launchとbringup

### 16.1 standalone launch

既存`zed2i.launch.py`は維持する。新しいlaunch argumentを追加する。

- `enable_gpu_perception`
- `engine_path`

引数はparameter fileより優先させる。CPU modeで`enable_gpu_perception=true`は明示的に
起動失敗させる。

### 16.2 unified sensor container

`robot` packageに、Livox driverとZED SDK componentを1つの
`component_container_mt`へ載せる実機用launchを追加する。

```text
sensor_perception_container
  - livox_ros::DriverNode
  - zed2i_driver::SdkNode
```

両componentへ`use_intra_process_comms=true`を設定する。

既存`lidar.launch.py`との設定重複を避けるため、Livox component descriptionを生成する
Python helperへ共通化し、standalone lidar launchとunified launchの両方から使う。

`real_bringup.launch.py`では次の選択にする。

- 新GPU経路有効: unified sensor containerを起動。旧Python YOLOと旧PCL buoy detectorは起動しない。
- 旧経路選択: 従来のZED launch、Livox launch、Python YOLO、PCL detectorを使用可能。
- 新旧経路の同時有効化はlaunch setupでRuntimeErrorにする。

新GPU経路を実機既定とする。ただしengine pathはrobot固有configから必ず与える。

### 16.3 Task 2 launch

commit `77199fcf09a01234c265dab6e2fa9fcb0a16a112`由来の`task2_real.launch.py`から、
ブイ検知だけをpurgeする。

- `yolo11s.launch.py`のincludeを削除する。
- `enable_yolo`、`yolo_backend`、`yolo_model_path`、`use_tensorrt`、`use_int8`を削除する。
- 旧`main.py`、旧PCL buoy detector、新GPUブイ検知を代替起動しない。
- `/buoy_roi`、`/yolo/detections`、`/buoy_detections_3d`、`/virtual_obstacles`のpublisherを
  Task 2 launch treeへ含めない。
- `enable_camera`はブイ検知専用argumentとしては削除する。他用途でZEDが必要になった場合は、
  その用途とconsumerを別仕様で定義してから再追加する。
- `task2_cloud_filter`、classical ship tracking、`opponent_selector`、GLIM/EKF、MPPI、Task 2 Nav2、
  dry-runとスラスタ三重gateは維持する。
- `nav2_params_task2.yaml`へ`/virtual_obstacles`を追加しない。

Task 2のstatic launch testでは、上記ブイnode、ブイtopic、YOLO専用argumentがlaunch treeに
存在しないことを検査する。通常Nav2のGPUブイbringupとは別entry pointとして保つ。

## 17. loggingとdiagnostics

起動時にinfo logへ次を1回出す。

- ZED SDK version
- CUDA runtime version
- TensorRT version
- GPU名
- engine pathとmanifest SHA-256
- input/output tensor shapeとdtype
- camera解像度、FPS、depth mode
- class順
- 各output topicとframe
- Livox融合parameter

最低限次のcounterを保持し、10秒ごとにthrottled infoまたはdiagnosticとして出す。

- grabbed frames
- inference submitted/completed/dropped
- TensorRT failures
- detections total
- ZED depth valid/invalid
- LiDAR clouds received/stale
- LiDAR fused/fallback/no-cluster
- TF failures by transform
- published detection arrays
- published virtual wall points

frameごとのinfo logは禁止する。回復不能なCUDA/TensorRT errorはcomponentをerror状態にし、
不正な結果をpublishし続けない。

## 18. 実装順序

以下の順序を守る。各段階でbuild/testを通してから次へ進む。

### Step 1: interface

1. 2つのmsgを追加。
2. `njord_interfaces`のCMakeを更新。
3. message生成buildと簡単なimport/compile testを実行。

### Step 2: ROS非依存ロジック

1. 内部検出型を追加。
2. TensorRT output decodeとletterbox逆変換を実装。
3. depth ROI計算をCPU referenceとして実装。
4. LiDAR cluster選択を純粋関数化。
5. 仮想壁geometryを純粋関数化。
6. gtestを追加。

CPU reference depth実装はCUDA版とのtest oracleに使い、本番frame経路からは呼ばない。

### Step 3: model tools

1. export script。
2. ONNX contract検査。
3. engine build wrapper。
4. engine/manifest inspector。
5. artifact ignore設定と利用手順。

### Step 4: TensorRT/CUDA library

1. optional CMake検出。
2. RAII TensorRT wrapper。
3. CUDA preprocess。
4. GPU depth compact/sort/median。
5. 静止画像とsynthetic depthによるconditional GPU test。

### Step 5: ZED node統合

1. parameter追加。
2. persistent double buffer。
3. latest-frame worker。
4. GPU取得からTensorRTまで接続。
5. ZED-only 3D結果とmessage publish。
6. 外部topicのlazy publishingが退行していないことを確認。

### Step 6: Livox融合

1. point cloud cache。
2. TF、投影、bbox点割当。
3. clusteringと選択。
4. source/NaN規則をmessageへ反映。

### Step 7: 仮想壁

1. base位置をmapへ変換。
2. 6クラスarc生成。
3. 空点群clear。
4. channel heading validation。

### Step 8: launch、文書、実機評価

1. unified container。
2. real bringup切替。
3. Task 2 launchからブイ検知node、topic、argumentをpurgeし、static testを追加。
4. README、buoy pipeline、performance roadmapに加え、Task 2 architecture、parameter reference、
   Jetson validation、integration reportを更新。
5. x86 conditional test。
6. Jetson実機testと測定結果記録。

## 19. test仕様

### 19.1 unit test

最低限次を自動test化する。

#### Decode / letterbox

- paddingなし、左右padding、上下paddingのbbox逆変換。
- 画像端clip。
- 負サイズbbox破棄。
- confidence threshold前後。
- 33件以上の入力から上位32件だけを選ぶ。
- 6クラスindexを変更しない。
- `[1,N,6]`と`[1,6,N]`が同じ結果になる。

#### Depth

- 奇数個、偶数個の中央値。
- NaN、inf、範囲外除去。
- sample数15は無効、16は有効。
- bboxが画像外へはみ出す場合のclip。
- central ratio 0.5のpixel範囲。
- CPU referenceとCUDA結果がfloat tolerance内で一致。

#### Fusion

- timestamp差0.15秒以内を採用、超過を拒否。
- ZED有効かつ整合clusterあり: LiDAR採用。
- ZED有効だが距離差2m超: ZED維持。
- ZED無効かつray上clusterあり: LiDAR補完。
- 両方無効: xyz NaN、source NONE。
- bbox重複時に1点を複数検出へ割り当てない。
- TF失敗時にZED fallbackが残る。

#### Virtual wall

- north/east/south/westの安全sectorへ点を置かない。
- redはchannel heading左、greenは右へ壁を置く。
- invalid positionを無視。
- 検出0件で空PointCloud2。
- map TF欠落で壁なし。

#### Thread / lifecycle

- pending frameの置換。
- running slotを上書きしない。
- shutdown時にworkerが終了する。
- 推論例外後に壊れたmessageをpublishしない。

### 19.2 開発端末での基本コマンド

workspaceの通常環境に合わせてsetupをsourceした後、少なくとも以下を実行する。

```bash
colcon build --packages-up-to njord_interfaces zed2i_driver pcl_det robot yolo \
  --cmake-args -DZED2I_DRIVER_BUILD_SDK_NODE=OFF

colcon test --packages-select njord_interfaces zed2i_driver pcl_det yolo
colcon test-result --verbose
```

TensorRTを導入した端末では追加でGPU test targetを実行する。GPU testは依存がないCIで
skipされるようlabelまたはCMake conditionを付ける。単なるtest失敗の握りつぶしは禁止する。

Task 2関連commitを統合したtreeでは`test_task2_real_launch_static.py`も実行し、Task 2 launch treeに
YOLO executable、PCL buoy detector、GPUブイ知覚、ブイtopic、YOLO専用launch argumentがないこと、
および他船tracking、MPPI、Nav2、スラスタ三重gateが残ることを検査する。

### 19.3 Python modelとの一致確認

固定画像setに対してPython `best.pt`とC++ TensorRTを比較する。

- classが一致すること。
- 対応bbox IoU >= 0.95。
- confidence絶対差 <= 0.05。
- dataset全体のmAP低下 <= 1 percentage point。

FP16差でthreshold近辺だけ不一致になる場合は、該当sample、score、原因を記録する。
許容値を黙って緩めない。

## 20. Jetson実機受入試験

### 実装後の検証状態（2026-07-21）

この変更はx86_64開発端末で、TensorRT/ZED GPU runtimeを有効化しない
`njord_interfaces` / `zed2i_driver` のbuildと、ROS非依存の知覚ロジックunit testを
完了している。ZED GPU buffer、TensorRT engine、MID-360S、実際のTFを必要とする検証は
この端末では実行できないため、**Jetson Orin Nano Super実機で実施予定**とする。

Jetsonでは、JetPackと互換なZED SDK・TensorRTを導入し、Jetson自身で生成したengineを
`engine_path`へ設定してから、以下の受入試験と性能測定を実行する。結果（環境version、
engine manifest、実行コマンド、pass/fail、計測値）はこの節またはintegration reportへ
追記するまで未完了と扱う。

### 20.1 機能

- Jetson自身で生成したFP16 engineをdeserializeできる。
- ZED 2i左画像から6クラスを検出できる。
- ZED depth有効時に3D位置が出る。
- depthを意図的に欠損させた対象をMID-360Sで補完できる。
- 両方無効時も検出自体はNaN位置で配列に残る。
- `/buoy_roi`は新経路からpublishされない。
- `/buoy_detections_3d`は検出0件を含めcamera推論ごとに更新される。
- 6クラスの壁がmap座標とchannel headingに従う。
- 通常Nav2が空`/virtual_obstacles`で古い壁を消せる。
- Task 2 launchではブイ検知nodeと4つのブイ関連topic publisherが存在しない。
- Task 2の他船tracking、MPPI、Nav2、dry-runとスラスタ三重gateが維持される。

### 20.2 性能

同一nvpmodel、同一解像度、同一FPS、同一sceneで旧経路と比較する。

- `tegrastats`: CPU、GPU、RAM、EMC。
- camera grab rate。
- inference completion rate。
- camera stampから`/buoy_detections_3d` publishまでのp50/p95 latency。
- dropped frame数。
- external ZED Image/depth/pointsのpublish rate。
- 通常ブイ構成のGLIM+GPU融合とTask 2構成のGLIM+cloud filterについて、`/livox/lidar`の
  subscriber数、serialization/copy、CPU/RAM/EMC差分。
- 10分以上の連続運転でsegfault、CUDA error、memory増加がないこと。

目標:

- 推論遅延が蓄積せず、常に最新frameへ追従する。
- 15 FPS camera入力時、検出出力10 Hz以上を維持する。
- end-to-end p95を150 ms以下にする。
- ZED画像/depthにsubscriberがない通常運用で、それらのROS publish rateは0 Hz。
- Nsight Systems上で主推論経路にZED全画面画像/depthのGPUからhostへのcopyがない。

性能目標を満たせない場合でも、frame queueを増やして見かけのFPSを上げない。kernel、engine、
ZED depth mode、GPU/EMC使用率を記録し、次の最適化項目として切り分ける。

## 21. error handling一覧

| 条件 | 動作 |
|---|---|
| engine pathなし/読めない | node起動失敗 |
| engine/manifest非互換 | node起動失敗、再生成手順を表示 |
| tensor contract不一致 | node起動失敗 |
| CUDA/TensorRT初期化失敗 | node起動失敗 |
| 単発ZED grab失敗 | frame skip、throttled warning |
| 回復不能CUDA enqueue失敗 | publish停止、error状態 |
| ZED depth無効 | LiDAR補完を試行 |
| LiDAR stale/TF失敗 | ZED位置を維持 |
| ZED/LiDAR両方無効 | NaN位置をpublish |
| map TF失敗 | detectionはpublish、壁だけskip |
| channel heading未設定 | red/green壁だけskip、error diagnostic |
| subscriberなしのdebug topic | CPU readbackもmessage生成もしない |

## 22. compatibilityと移行

- Python `yolo` package、`BuoyRoi.msg`、`pcl_det` packageを削除しない。
- 旧launchを壊さない。
- 新旧経路は明示launch argumentで排他的に選ぶ。
- Task 2だけは互換対象の旧ブイlaunchではなく、ブイ検知を持たない独立entry pointとする。
- Task 2の他船認識は`/task2/points_filtered`、`/tracked_objects`、`/other_ship/twist`を使用し、
  本仕様のブイmessageや仮想壁へ依存させない。
- `/buoy_detections`の型を変更しない。新配列は`/buoy_detections_3d`を使う。
- 新経路のconsumerはNaNと`position_source`を必ず確認する。
- `best.pt`のclass順を変更する再学習modelを投入する場合、export検査とmessage定数も同時に
  更新しなければならない。
- 既存の`Docs/sensor_pipeline_performance_roadmap.md`には未コミット編集があるため、更新時に
  内容を上書きまたは破棄しない。現在のdiffへ必要箇所だけ追記・修正する。

## 23. Definition of Done

次をすべて満たした時点で実装完了とする。

- 新messageが生成され、全対象packageがbuildできる。
- TensorRT/ZEDなしの環境で既存buildを壊さない。
- TensorRT/ZEDありのJetsonで新componentが起動する。
- ZED GPU bufferからTensorRTまでROS Image/CvBridgeを通らない。
- bboxと`/buoy_roi`を新経路でpublishしない。
- ZED depth中央値、MID-360S精密化・補完、NaN fallbackが仕様どおり動く。
- `/buoy_detections_3d`と6クラスの`/virtual_obstacles`が仕様どおり出る。
- `/virtual_obstacles`のconsumerが通常Nav2に限定され、Task 2へ接続されていない。
- 新旧経路がlaunchで排他的に選択できる。
- Task 2 launchから全ブイ検知経路、関連publisher、YOLO専用argumentがpurgeされ、他船tracking、
  MPPI、Task 2 Nav2、安全gateが維持される。
- unit test、conditional GPU test、Jetson受入試験の結果が文書化される。
- 10分連続運転でcrash、処理待ちqueueの増加、継続的memory増加がない。
