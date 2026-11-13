# ZED2i 地上局向け低負荷映像転送計画

## 1. 結論

実現可能である。ただし、この機体は **Jetson Orin Nano Super** であり、Orin Nanoには
NVENCがない。そのため、TensorRTと同じようにGPU上でH.264/H.265をハードウェア圧縮する
構成は採用できない。

この機体での第一候補は、左カメラの既存GPUバッファから分岐し、送信対象フレームだけを
GPU上で縮小・色変換・JPEG圧縮して、**圧縮後の小さいbitstreamだけをCPUへ1回コピー**し、
RTP/UDPで地上局へ送る構成である。

```text
ZED 2i
  |
  | camera.grab()
  v
ZED SDK left BGRA8 (GPU、推論と共有する入力)
  |                         |
  | 毎フレーム              | 例: 15 Hz中5 Hzだけ選択
  v                         v
TensorRT推論             CUDA resize + BGRA->RGB/NV12
                            |
                            v
                    nvJPEG / nvjpegenc (GPU)
                            |
                            | D2H 1回、圧縮済み数十～数百KBのみ
                            v
                    pinned host GstBuffer
                            |
                            v
                    RTP/JPEG over UDP
                            |
                            v
                         地上局
```

フレームレート削減は圧縮後や地上局側ではなく、GPU圧縮経路へ投入する前に行う。これにより、
送信しないフレームには縮小、圧縮、CPU readbackの費用を発生させない。

なお、ネットワーク送信ではユーザー空間からカーネルへの受け渡しなどは残る。「1回コピー」
は、アプリケーションで行うGPU-to-CPU readbackを圧縮bitstreamの1回だけにする、という意味で
定義する。ネットワークスタックを含む完全な物理zero-copyは本計画の目標外とする。

## 2. 現状調査

### 2.1 実機環境（2026-07-21確認）

| 項目 | 確認結果 | 設計への影響 |
|---|---|---|
| 機体 | Jetson Orin Nano Engineering Reference Developer Kit Super | NVENCなし |
| Jetson Linux | L4T `39.2.0` | 実装時はこの版で再検証する |
| CUDA | `nvidia-smi`表示 `13.2` | 既存CUDA経路を利用可能 |
| TensorRT engine | manifest上 `10.16.2.10` | 推論と同一GPUを共有する |
| `nvv4l2h264enc` | GStreamer elementなし | H.264 HW encodeは選択不可 |
| `nvv4l2h265enc` | GStreamer elementなし | H.265 HW encodeは選択不可 |
| `nvjpegenc` | あり。NVMMのI420/NV12入力に対応 | GPU JPEG経路の第一候補 |
| `nvvidconv` | あり。NVMM、GPU/VIC resizeに対応 | 色変換・縮小のPoCに使用可能 |
| software encoder | `x264enc`、`openh264enc`、`x265enc`あり | JPEG不適合時のfallback |

NVIDIAの公式資料にも、Orin NanoはNVENCを持たずH.264はsoftware encodeを使うと明記されて
いる: [Software Encode in Orin Nano](https://docs.nvidia.com/jetson/archives/r35.6.0/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html)。

### 2.2 現在のZEDドライバ

対象は `src/driver/camera/zed2i_driver/src/sdk_node_zed.cpp` である。

- 推論時は `retrieveImage(..., sl::MEM::GPU)` で左BGRA画像をGPUへ取得している。
- TensorRT入力へはGPU pointerとpitchを直接渡している。
- 通常の `/zed2i/left/image_rect` にsubscriberがいると、別途CPU memoryへ画像を取得し、
  `sensor_msgs/Image`へコピーする。
- 現実装はtimer callback内で推論まで同期実行し、GPU画像bufferは1枚である。
- 設計文書にはdouble bufferとworker threadが記載されているが、現コードにはまだ反映されて
  いない。この差を映像転送の実装前に解消する必要がある。

地上局映像をraw ROS Imageのsubscriberとして実現すると、1280x720 BGRA、15 Hzの場合、画像
payloadだけで約 `1280 * 720 * 4 * 15 = 55.3 MB/s` となり、CPU readback、ROS message生成、DDS
serialization、無線帯域のすべてに負担がかかる。既存raw topicとは別の圧縮経路にする。

## 3. 採用アーキテクチャ

### 3.1 推奨: GPU JPEG + RTP/UDP

新しい `GroundVideoStreamer` を `zed2i_driver` の内部クラスとして実装する。公開GPU pointerを
ROS messageに載せず、ZED SDKと同じcomponent内で処理する。

1. `camera.grab()`後、推論用に左画像をGPUへ1回取得する。
2. stream周期に該当するフレームだけ、stream専用の空きGPU slotへ投入する。
3. CUDA kernelまたはNVMM/VICで、例えば1280x720 BGRAを640x360 RGB/NV12へ縮小・変換する。
4. `nvJPEG` C APIを第一候補としてGPU上でJPEG化する。
5. encoder完了event後、圧縮済みbitstreamだけを再利用可能なpinned host bufferへコピーする。
6. `GstBuffer`はhost bufferを追加コピーせずwrapし、`rtpjpegpay ! udpsink`へ渡す。
7. 地上局はGStreamerで受信、jitter buffer、decode、表示する。

`nvjpegenc`を使うGStreamer-only PoCも先に試すが、任意の`sl::Mat` CUDA pointerを`appsrc`から
NVMMへ安全に渡す部分が複雑、または隠れたhost copyが生じる場合は、色変換と圧縮を直接CUDA/
nvJPEG APIで実装する。採否はNsight Systemsとcopy counterで決める。

### 3.2 bufferとthread

camera callback、推論、映像送信を互いに待たせない。

```text
grab thread
  ├─ perception slot: latest-wins、最低2 slots
  └─ stream slot:     latest-wins、2～3 slots

perception worker
  └─ CUDA preprocess -> TensorRT -> small result D2H

stream worker
  └─ CUDA resize/color -> JPEG -> compressed D2H -> RTP send
```

- `FREE / PENDING / RUNNING`状態とCUDA eventでbuffer寿命を管理する。
- queueが埋まったら古い`PENDING`映像を捨てる。カメラ、推論、操船を待たせない。
- `RUNNING` slotは上書きしない。
- frameごとのGPU/host allocationは禁止し、GPU buffer、encoder state、pinned bufferを起動時に
  確保して再利用する。
- 推論と圧縮は別CUDA streamとし、優先度は推論を高くする。
- ネットワーク停止時も推論を継続し、送信queueだけをdropする。
- raw image subscriberの有無はこの圧縮経路と分離する。圧縮配信中も
  `/zed2i/left/image_rect`のCPU取得を自動的に有効化しない。

### 3.3 転送方式

初期実装は **RTP/JPEG over UDP unicast** とする。

- ROS/DDSのrawまたは`CompressedImage`をWAN相当の無線映像transportとして使わない。
- DDS discoveryやreliable再送で遅延を蓄積しない。
- JPEGは各フレームが独立しているため、packet loss後に次のフレームで復帰しやすい。
- 送信先IPとportを明示し、大会ネットワーク以外へ送らない。

UDPのlossが大きく1枚のJPEG欠損が頻発する場合は、短いjitter buffer、RTP packet size、FECの
順に評価する。SRT/WebRTCは、暗号化、NAT越え、輻輳制御など大会要件で必要になった場合だけ
第2段階で検討する。

### 3.4 初期parameter案

```yaml
zed2i:
  ros__parameters:
    enable_ground_video: false
    ground_video_codec: jpeg
    ground_video_width: 640
    ground_video_height: 360
    ground_video_fps: 5.0
    ground_video_jpeg_quality: 70
    ground_video_host: ""
    ground_video_port: 5600
    ground_video_max_pending_frames: 1
    ground_video_mtu: 1200
```

`host`が空、解像度/FPS/quality/portが範囲外、encoder初期化失敗の場合は、操船や推論まで停止
させず映像配信だけをERRORにする。ただし大会規定上「映像なしで出走不可」であれば、bringup
側に別のpreflight gateを設ける。

640x360、5 FPS、quality 70はPoC開始値であり、確定値ではない。海面反射や波はJPEG sizeを
増やすため、実際の海上映像と大会無線で測って決める。

## 4. 実装計画

### Phase 0: 大会要件と帯域budgetの確定

実装前に次を大会規定または運営へ確認し、計測可能な受け入れ条件へ変換する。

- 必須映像のcamera、画角、カラー/白黒、最低解像度、最低FPS。
- 最大許容遅延と、映像が連続である必要があるか。
- 許可codec/transport、指定viewer、overlayや時刻表示の要否。
- 使用可能な周波数・ネットワーク、送信先、最大bitrate、multicast可否。
- 一時的な映像欠損時の扱い。

未確定の間は `640x360 @ 5 FPS`、片道遅延300 ms以下、映像帯域3 Mbit/s以下を暫定目標とする。
JPEGは内容依存なので、bitrate上限を保証できない点に注意する。

### Phase 1: 実機PoCと方式選定

1. 保存済みZED海上映像または実カメラを入力にする。
2. `nvvidconv ! nvjpegenc`で640x360、5 FPS、quality 50/60/70/80を測る。
3. 同条件で直接nvJPEG APIを測る。
4. 各方式で以下を記録する。
   - encode p50/p95時間。
   - CUDA/VIC使用率、CPU core別使用率、消費電力、温度。
   - 平均/p95/最大frame sizeとbitrate。
   - host-device copy回数とbytes。
   - 画質、文字・ブイ・航路標識の視認性。
5. 隠れたfull-frame D2Hがなく、既存TensorRT推論への影響が小さい方式を採用する。

成果物は再現可能なPoCコマンド、計測表、採用理由とする。このgateを通るまで本体へ組み込まない。

### Phase 2: 非同期buffer基盤

1. 現在の単一`left_gpu_`と同期timer推論を、設計済みのslot + worker構造へ変更する。
2. buffer所有権、latest-wins drop、CUDA event待ち、shutdown順序をunit test可能な状態機械に分離
   する。
3. 推論だけを有効にした状態で、変更前後のFPSとlatencyが同等であることを確認する。
4. ThreadSanitizerが利用可能なCPU部分と、連続起動停止testでrace/use-after-freeがないことを
   確認する。

このPhaseを映像圧縮実装より先に行い、推論中のZED bufferをencoderが参照し続ける事故を防ぐ。

### Phase 3: GPU圧縮component

1. `GroundVideoStreamer` interface、RAII resource、parameter validationを実装する。
2. 起動時にstream専用GPU slots、CUDA stream/event、nvJPEG encoder、pinned host buffersを確保
   する。
3. resizeとBGRA変換を1 kernelに融合し、中間bufferを最小化する。
4. timestamp accumulatorで入力15 Hzから指定5 Hzを選択する。`frame_count % 3`だけに依存せず、
   入力FPS変更やgrab dropがあっても平均送信FPSを維持する。
5. 圧縮bitstreamのsizeを取得してから、必要bytesだけD2Hする。
6. 異常時はstream workerだけを停止し、camera/TensorRTは継続する。

### Phase 4: RTP送信と地上局receiver

1. wrapしたhost bufferを`appsrc ! jpegparse ! rtpjpegpay ! udpsink`へ渡す。
2. timestampをRTP PTSへ引き継ぐ。
3. 地上局用receiver launch/scriptを追加し、指定NIC、port、jitter latencyを設定可能にする。
4. 地上局に受信FPS、bitrate、packet loss、最終受信時刻を表示する。
5. sender/receiverの終了、地上局再起動、Wi-Fi切断・復帰を試験する。

### Phase 5: bringupと運用

1. `zed2i_jetson_orin_nano.yaml`へparameterを追加し、既定は無効にする。
2. `real_bringup.launch.py`に明示的なenable、host、port引数を追加する。
3. 大会用configだけで有効化し、開発時の意図しない映像送信を防ぐ。
4. preflight checkでreceiver到達性ではなく、encoder起動、frame生成、送信counter増加を確認する。
5. 操縦画面には「最終映像受信からの経過時間」を常時表示する。
6. 映像配信の失敗が自律・手動操船のcallbackやexecutorをblockしないことをlaunch testで確認する。

### Phase 6: 実機受け入れ試験

以下をすべて満たしてから大会configを固定する。

- raw image subscriberなしでfull-resolution imageのD2Hが発生しない。
- stream frameあたり、アプリケーション上のD2Hは圧縮bitstream 1回だけである。
- 映像無効時に既存推論性能が変更前と同等である。
- 映像有効時の推論出力rate低下が5%以内、推論p95 latency増加が10%以内である。
- 暫定目標として、640x360 @ 5 FPS、glass-to-glass p95 300 ms以下を満たす。
- 大会距離で平均/p95 bitrateが無線budget内に収まる。
- 5%、10%、20% packet lossと帯域制限下で、遅延を蓄積せず最新映像へ復帰する。
- 30分以上の同時運転でthermal throttling、OOM、encoder停止がない。
- 地上局停止中もcamera grab、TensorRT推論、操船が継続する。
- プロセス再起動を10回行ってCUDA/GStreamer resource leakやport残留がない。

5%/10%の性能budgetは初期値であり、Phase 0で安全系・大会要件に合わせて確定する。

## 5. diagnostics

10秒ごと、または`diagnostic_msgs`で次を公開する。frameごとのinfo logは禁止する。

- selected / encoded / sent / dropped frames。
- drop理由: no slot、encoder busy、oversize、network error。
- encode時間、D2H時間、送信queue滞留時間のp50/p95。
- JPEG bytes/frame、瞬間/平均bitrate。
- 最終成功encode時刻、最終成功send時刻。
- encoder再初期化回数と直近error。
- 現在の解像度、FPS、quality、送信先。

推論側のsubmitted/completed/dropped counterと同時に取得し、圧縮が推論を圧迫していないことを
確認できるようにする。

## 6. fallbackと将来拡張

### 6.1 JPEGが帯域に収まらない場合

優先順位は次の通りとする。

1. FPS、解像度、JPEG qualityを下げる。
2. GPUで縮小・NV12化し、縮小後frameだけをCPUへD2Hして`x264enc`/`openh264enc`を
   `zerolatency`設定で使う。
3. 外付けH.264 encoderを追加する。
4. NVENCを持つJetson Orin NX/AGX Orinへ変更する。

software H.264はbitrate効率がよい反面、圧縮前の縮小画像をD2Hするため、本計画の「圧縮後だけ
1回D2H」という目標から外れる。CPU負荷が操船・ROS executorへ影響しないことを別途検証する。

### 6.2 NVENC搭載機へ移行した場合

`GroundVideoStreamer`のbackendを差し替え、次の経路にする。

```text
ZED GPU BGRA -> CUDA/VIC NV12 -> NvBufSurface/NVMM
             -> nvv4l2h264enc / nvv4l2h265enc
             -> encoded bitstream D2H -> RTP/UDP
```

送信・buffer・diagnostics interfaceは維持し、codec backendだけを変更できる設計にする。

## 7. 非目標

- depth画像、右画像、point cloudの常時地上転送。
- raw ROS ImageをWi-Fi越しに配信すること。
- 映像に推論bboxを焼き込むこと。必要ならGPU overlayとして別Phaseで追加する。
- 映像送信失敗を理由に、推論や操船threadをblockすること。
- 大会規定が未確認のまま解像度、FPS、codecを固定すること。

## 8. 完了条件

Phase 0で確定した大会要件を満たし、Phase 6の実機試験に合格し、次の2経路が独立して動くことを
もって完了とする。

```text
安全・自律経路: ZED GPU -> TensorRT -> detection/control
監視経路:       ZED GPU -> rate limit -> GPU JPEG -> compressed D2H -> ground station
```

地上局の接続品質が低下しても、安全・自律経路のrateとlatencyを維持することを最優先とする。
