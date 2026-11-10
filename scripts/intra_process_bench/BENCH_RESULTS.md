# intra-process 認識パイプライン 実機ベンチ結果（GLIM 稼働版）

計測日: 2026-08-01 / 端末: Jetson Orin Nano Super (6-core, ROS 2 Jazzy) / branch: `test07089`

前回のベンチ（`glim`・`pcl_det` の出力が無い状態）に対して、**GLIM を実際に publish させた状態**で取り直したもの。
`pcl_det` はパイプラインから外したため、LiDAR 側の負荷は GLIM のみ。

計測ハーネス: `scripts/intra_process_bench/run_bench.sh` + `combined_bench.launch.py`
（warmup 45s → 計測 60s、tegrastats 1Hz）

---

## 1. 結論

- **GLIM は載る。** 実機 MID360S で `/odom` は 10 Hz にきっちり張り付き（3 ラン全て 9.999〜10.000 Hz、std 約 10 ms）、
  CPU 余力 53〜59 %・GPU 平均 62〜68 % を残す。
- **GLIM の GUI viewer（`libstandard_viewer.so`）は載らない。** ON にすると GPU 平均が 65 %→87 % に上がり、
  ZED の出力が半分以下（11.7→5.9 Hz）に落ちる。運用は viewer OFF、デバッグ時のみ ON。
- **デバッグ用トピックは publish 不要どころか、既に実質コストゼロ。** subscriber が居なければ処理ごとスキップされる。
  前回の計測は `ros2 topic hz` が subscriber を作ったせいで、本来存在しない負荷を自分で発生させていた。

---

## 2. 実機計測（MID360S 実機 + ZED2i 実機、GLIM viewer OFF）

計測対象は実消費者のいる運用トピックのみ。

| topic | run1 (yaml修正前) | run2 (yaml修正後) | run3 (yaml修正後·再現確認) |
|---|---|---|---|
| `/livox/lidar` | 10.001 Hz | 10.000 Hz | 10.001 Hz |
| `/livox/imu` | 199.996 Hz | 199.994 Hz | 199.995 Hz |
| `/odom` (GLIM) | 9.999 Hz (std 12.0 ms) | 9.999 Hz (std 9.9 ms) | 10.000 Hz (std 10.4 ms) |
| `/buoy_detections_3d` | 14.513 Hz | 12.603 Hz | 13.867 Hz |
| CPU 余力 | 55 % (peak busy 86 %) | 53 % (peak 87 %) | 59 % (peak 84 %) |
| GPU GR3D 平均 | 62 % (peak 99 %) | 64 % (peak 99 %) | 68 % (peak 99 %) |
| RAM peak | 3.72 GB / 7.5 GB | 3.76 GB | 3.77 GB |

`/livox/lidar`・`/livox/imu`・`/odom` は 3 ラン全てで一致しており、**GLIM は実機 LiDAR でも取りこぼしなく 10 Hz を維持**している。

### `/buoy_detections_3d` のばらつきについて

14.51 / 12.60 / 13.87 Hz と最大 1.9 Hz ばらつくが、これは**設定差ではなくラン間変動**。
同一設定同士（run2 vs run3）でも 1.3 Hz 動いている。ZED 融合検出のレートは視界内のクラスタ数に依存するため、
実機カメラの前の状況で変わる。CPU 余力 53〜59 %・GPU 62〜68 % のレンジも同様。

---

## 3. bag 再生での viewer ON/OFF 比較

GLIM の GUI viewer のコストを切り分けるための計測（LiDAR は bag 再生、デバッグトピックも購読した状態）。

| topic | 前回 (GLIM無音·pcl_det有) | GLIM稼働 + GUI viewer | GLIM稼働 + viewer OFF |
|---|---|---|---|
| `/livox/lidar` | 10.00 Hz | 10.001 Hz | 10.000 Hz |
| `/livox/imu` | 200.0 Hz | 200.020 Hz | 200.019 Hz |
| `/odom` (GLIM) | **無音** | 10.238 Hz | 10.000 Hz |
| `/glim_node/points` | **無音** | 10.238 Hz | 9.998 Hz |
| `/zed2i/left/image_rect` | 13.0 Hz | 5.929 Hz | 11.660 Hz |
| `/zed2i/points` | 10.8 Hz | 3.579 Hz (最大ギャップ 1.40 s) | 10.416 Hz |
| `/buoy_detections_3d` | 14.1 Hz | 11.008 Hz | 13.834 Hz |
| CPU 余力 | 83 % | 34 % (peak busy 98 %) | 54 % (peak 98 %) |
| GPU GR3D 平均 | 48 % | 87 % (peak 99 %) | 65 % (peak 99 %) |
| RAM peak | 3.9 GB | 4.16 GB | 4.06 GB |

- GLIM 本体の追加コストは GPU +17pt（48→65 %）程度。ZED 側は 13.0→11.7 Hz とほぼ維持。
- **GUI viewer を足すとさらに GPU +22pt**（65→87 %）、ZED が 11.7→5.9 Hz、点群が 10.4→3.6 Hz に落ちる。

---

## 4. 修正した不具合

### 4.1 GLIM が起動するのに ROS 上で完全に無音だった

**原因**: `glim_config_headless/config_ros.json` の `extension_modules` から `librviz_viewer.so` を外していた。

名前に反して rviz_viewer は GUI モジュールではなく、**GLIM の ROS publisher 本体**
（`~/odom` → `/odom`、`~/points`、`~/pose`、TF を publish する拡張）。
これを外すと GLIM は正常に動作し続けたまま ROS 上では何も出さない。
OpenGL GUI は別モジュールの `libstandard_viewer.so` の方。

**さらに**: rviz_viewer は publish 前に `livox_frame → base_link` の TF lookup を行い、
失敗するとそのフレームを捨てる。`base_link` は URDF 由来なので、ベンチにも
`robot_state_publisher`（`task1.launch.py` と同じもの）が必要だった。

両方入れて lookup 失敗 0 件、`/odom` 10 Hz を確認。

### 4.2 ZED2i の params yaml が丸ごと無視されていた

**原因**: ノードの FQN は `/zed2i/zed2i`（`zed2i.launch.py` が namespace `/zed2i` に name `zed2i` でロード）
なのに、`zed2i_jetson_orin_nano.yaml` のキーが `zed2i:` だったためマッチせず、
**全パラメータが `declare_parameter()` の default に落ちていた**。

実行中ノードへの問い合わせで確認（修正前）:

```
publish_pointcloud   = True              ← yaml は false
pointcloud_stride    = Parameter not set ← yaml に 2 と記載あり
depth_max_m          = 20.0              ← コード default と同値
confidence_threshold = 0.25              ← コード default と同値
```

キーを `/zed2i/zed2i:` に修正後、`publish_pointcloud = False` になることを確認。
`enable_gpu_perception` は launch 引数の `true` が yaml の `false` を正しく上書きしており、優先順位も期待通り。

**ただし実効的な挙動変化は `publish_pointcloud: true→false` の 1 個だけ**だった。
他の yaml 値は全てコード default と同値で、以下は SDK ノードが宣言しておらず不活性（v4l ノード側のパラメータ）:
`pointcloud_stride` / `fx` / `fy` / `cx` / `cy` / `image_width` / `image_height` /
`num_disparities` / `block_size` / `fusion_max_range_delta_m` / `left_device` / `right_device`

**注意**: この yaml キーは launch の `namespace` 引数（`/zed2i`）に依存する。
namespace を変えると再び黙って無視されるので、堅牢性を優先するなら `/**:` の方が安全。

---

## 5. デバッグ用トピックについて

`/zed2i/left/image_rect`・`/zed2i/points`・`/glim_node/points` は publish する必要がなく、
**かつ既に実質コストゼロ**。3 つとも subscriber 数でゲートされている:

- `/zed2i/*` — `sdk_node_zed.cpp` が全 publisher の subscriber 数を見て、
  誰も居なければ `sl::Camera::grab` 後の retrieve すらスキップして `return` する
- `/glim_node/points` — `librviz_viewer.so` も `get_subscription_count` を参照する

設定変更は不要で、購読しないだけで消える。
逆に言えば **`ros2 topic hz` で観測した瞬間に負荷が発生する**ので、計測時は注意が必要。

---

## 6. 未解決 / 次アクション

### ground_video (JPEG エンコード) が動かない

`nvjpegCreateSimple failed (nvJPEG status 6)` で毎回無効化される。
インストールされている nvJPEG が CUDA toolkit の **sbsa（server-class ARM）版**で、Tegra iGPU では
`ARCH_MISMATCH` を返す:

```
/usr/local/cuda-13.2/targets/sbsa-linux/lib/libnvjpeg.so.13   (libnvjpeg-13-2)
```

L4T multimedia API の NvJPEG に置き換えないとこの経路は動かない。
**本レポートの数値には JPEG エンコード負荷は含まれていない**（前回計測も同条件なので比較自体は公平）。

### その他

- GLIM が起動直後に `large per-point timestamp is found after time conversion!! front=1.500170` を 1 回吐く。
  その後 `/odom` は 10 Hz で安定するため実害は確認されていないが、実機 MID360S の初回フレーム由来と思われる。
- 実機は **MID360S (192.168.1.114)** が接続されている（MID360 の 192.168.1.151 は不応答）。
  `lidar.launch.py` の `lidar_model` default は `mid360` なので、実機起動時は `mid360s` の指定が必要。
  `combined_bench.launch.py` は default を `driver` / `mid360s` にしてある。
- 役割分担: 本ベンチはこの端末が担う intra-process 認識部のみを検証している。
  Nav2 / GNSS-EKF / 上位 behavior を miniPC へオフロードする場合の境界（トピック / DDS 越え）と
  ネットワーク負荷は別途検証が必要。

---

## 7. 再現方法

```bash
# 実機 MID360S + ZED2i、GLIM viewer OFF、運用トピックのみ
BENCH_TOPICS="/livox/lidar /livox/imu /odom /buoy_detections_3d" \
BENCH_LAUNCH_ARGS="lidar_source:=driver lidar_model:=mid360s glim_headless:=true" \
  bash scripts/intra_process_bench/run_bench.sh 45 60

# GLIM GUI viewer を ON にする場合は glim_headless:=false
# bag 再生に切り替える場合は lidar_source:=bag
```

生ログ・サマリは `scripts/intra_process_bench/out/` に出力される（git 管理外）。
本レポートの元データ: `real_viewer_off.txt` / `real_yamlfix.txt` / `real_yamlfix_rep2.txt` /
`glim_viewer_on.txt` / `glim_viewer_off.txt`
