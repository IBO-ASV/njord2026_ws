# current task — sim↔real ブリッジ実装 (branch: test07089)

最終更新: 2026-07-12 / 記録者: メインClaude(司令塔)

## 背景

test07089 では急ピッチで「実センサ・実スラスタを使えるようにする」変更を進めている。
sim で成立しているハイレイヤ(cmd_vel / task / nav2 / localization)と現実(ESP32・センサ)を
つなぐ部分の実装が抜けていたため、その接続層を整備する。

前提(ユーザ指示):
- ESP32 とは **USB serial のみ**で通信する。CAN は使わない。
- `can_msgs` pkg / uros(micro-ROS)は廃止してよい。
- mid360s / zed2i / red hill web cam を使えるようにする。
- スラスタの id check など実運用を想定した実装を進める。

## 現状把握(コード調査結果)

- USB serial ブリッジは既存: `micon_driver_fd/serial_writer`
  - sub `/thruster_command` (`std_msgs/Float32MultiArray`, 先頭4要素) + `/emg /red /yellow /green` (`Bool`)
  - 50ms 周期で `4×float32 (native LE) + flag byte` を書き込み。flag: bit3=emg,bit2=green,bit1=yellow,bit0=red
  - README は「normalized [-1,1] を送る」と記載。
- `thruster_driver`: cmd_vel + odom → P(+DOB) → wrench → 4スラスタ配分。
  - sim 出力は `/thruster_command` に **Newton値** (`command * force_per_duty`, 最大40N) を publish → **serial_writer の期待(normalized)と単位不整合**。
  - transport_mode に `can` / `mros_usb` / `both` などが残存(CAN=`can_msgs/Frame`, mros=per-thruster `UInt16`)。
- スラスタ id check: `simple_manual/thruster_id_test.cpp` が3秒ごとに各スラスタへ順番に thrust を出す簡易版が既存。
- CAN 依存箇所: `can_msgs`(pkg本体), `thruster_driver`(package.xml/CMake/node.cpp), `bms`(package.xml/CMake/node.cpp)。
- uros/micro-ROS: 実コードは未検出(`thruster_driver` の命名 "mros" のみ)。
- センサ launch:
  - `zed2i_driver/launch/zed2i.launch.py`, `zed2i_cpu.launch.py`
  - `robot/launch/back_cam.launch.py`(`usb_cam`, /dev/video0)= red hill web cam 候補
  - `um982_driver/launch/um982.launch.py`
  - livox/mid360 driver 本体は存在するが unified launch から未起動
  - `robot/launch/localization.launch.py` は glim/ekf/navsat を起動するが **実 lidar/gnss/imu ドライバは起動していない**
  - `robot/launch/all.launch.py` は localization+navigation のみ

## 要求(Requirements)

### R1. スラスタ実駆動パス(USB serial)の一貫性確保 [必須]
- ハイレイヤ cmd_vel → `thruster_driver` → `/thruster_command` → `serial_writer` → ESP32 が
  **単位・スケールが破綻せず**通ること。
- serial_writer と thruster_driver の間で送る値の意味(normalized か Newton か)を統一する。
- ESP32 は 1台につき 1 ESC、UInt16 指令 1 個という前提(specific.md)と矛盾しないこと。

### R2. CAN / uros の廃止 [必須]
- `thruster_driver` から CAN transport (`can_msgs/Frame`)と mros(micro-ROS想定)分岐を除去。
- `bms` から CAN 依存を除去(BMS 値は USB serial 経由に一本化 or 明示的に別線と定義)。
- `can_msgs` pkg を削除。ビルド対象・依存から外す。
- 除去後も sim 経路(`/thruster_command` Float32MultiArray)は維持。

### R3. スラスタ id check(実運用向け) [必須]
- どの ESP32/ESC が FR/FL/RR/RL のどれに対応するかを実機で確認できる手段。
- 既存 `thruster_id_test` を実運用要件(安全停止・対象選択・出力単位が実パスと一致)に合わせて整備。

### R4. センサ bringup [必須]
- mid360(livox lidar), zed2i, red hill web cam(usb_cam back_cam)を
  実機 bringup launch から起動できること。
- GNSS(um982) / IMU / localization(glim,ekf) と統合した「実機起動」launch を用意 or 整備。

### R5. 実運用を想定した安全・診断 [推奨]
- watchdog / feedback timeout / emg 停止が実パスで機能すること。
- センサ・スラスタの死活が diagnostics で見えること(既存 diagnostic_monitors 活用)。

## 受け入れ条件(Acceptance Criteria)

- AC1: `colcon build` が対象パッケージ(thruster_driver, micon_driver_fd, bms, robot, simple_manual, 各driver)で成功する。
- AC2: `can_msgs` への参照がビルドグラフから消え、grep で実コード参照が残らない(README等の記述除く)。
- AC3: 既存ユニットテスト(`thruster_driver` allocation, `micon_driver_fd` serial_writer)が pass。
       単位統一に関する回帰を守るテストを追加。
- AC4: 実機 bringup launch が **ドライラン(ros2 launch --show-args / node graph)**で
       正しいノード構成・topic 接続になっている(実 HW なしで検証可能な範囲)。
- AC5: cmd_vel → serial パケットの数値変換が期待通り(単位・neutral・span・クランプ)であることを
       テストまたは明示的な計算で確認。
- AC6: id check が実パス(serial_writer 経由)と同じ単位・安全挙動で1スラスタずつ駆動する。
- AC7: 変更が sim 経路(task*_sim / manual_control)を壊さない。

## 確定事項(2026-07-12 ユーザ判断)

- D1(完了ライン): **コード/launch/config 完成 + colcon build + テスト + ドライラン検証**まで。実HW通電検証は別工程(艇上)。
- D2(単位規約): **Newton のまま送る**。`/thruster_command`(Float32MultiArray)は Newton 値。serial_writer は 4×float32(native LE, Newton)+flag byte をそのまま送信。ESP32 側で thrust→PWM 換算。
  → thruster_driver の sim 出力(Newton)と serial_writer の生 float 送信が既に一致。README の "normalized" 記述が誤りなので Newton に訂正。id_test/manual も Newton 一貫に。
- D3(pkg整理): **can_msgs 削除 / kinematics 削除 / bms は serial 一本化して残す**。thruster_driver は CAN・mros 分岐を除去し sim(Float32MultiArray)経路のみ。
- D4(webcam): red hill web cam = **back_cam**(usb_cam, /dev/video0)。実機 bringup に統合。

### D3 に伴う新規設計論点(→ Sol レビュー対象)
- bms を serial 一本化 = ESP32→ホストの **BMS 受信経路が現状存在しない**(micon_driver_fd は送信専用)。
  ESP32 が同一 USB serial 上で BMS(4セル電圧)を返す前提で、受信・デコード・publish の設計が必要。
  想定アーキ: **単一 Micon/ESP32** が USB serial 1本で 4スラスタ(+flag)を受け、BMS 等を返す
  (specific.md の "ESP32 1台=1ESC + per-thruster mros topic" 案は D2/D3 で破棄)。

## 進め方(CLAUDE.md ルーティング)

1. 本要求整理(完了) → ユーザにオープン論点を確認。
2. 確定後、必要なら Codex Sol に設計レビュー依頼(単位規約・パケット互換・pkg削除影響)。
3. 実装担当を luna/terra/sonnet から作業規模に応じ1つ選択(現時点想定=terra: 複数pkg横断の中規模)。
4. 実装 → 別モデルでレビュー(.ai/reviews/)。
5. テスト失敗は同一修正を最大2回、以降 Sol→(昇格条件で)Fable(分析・計画のみ)。
6. 各段階を .ai/tasks/current.md, .ai/reviews/, .ai/escalations/ に記録。

## 確定設計(Sol レビュー + 司令塔裁定, 詳細は .ai/reviews/2026-07-12-sol-design-review.md)

- 単一 serial 所有ノード = `micon_driver_fd`(双方向化, executable 名 `serial_writer` 維持)。他ノードは tty を open しない。
- TX: 現行 raw v1(`4×float32 LE + flag byte`)維持。ESP32 ファーム変更は本タスク**スコープ外**。
- RX: BMS を framed+CRC でホスト側実装 + loopback ユニットテスト(実 BMS 通電はHW工程)。
- bms pkg は残す: 所有ノードの復号済み topic を購読し `bms/cell_voltages` 標準化 + diagnostics。tty は open しない。
- thruster_driver: CAN/mros/transport_mode 除去、非sim指定は起動エラー、`/thruster_command`=Newton。pkg は残す。
- can_msgs / kinematics: 削除。thruster_config.yaml/各README/task1_sim.launch.py 等の参照残を掃除。
- real_bringup.launch.py 新設: mid360/zed2i/back_cam/um982/imu/localization/thruster+serial を enable_* 引数で統合、use_sim_time=false。
- 安全: 起動時0N/cmd timeout0N/feedback timeout0N/serial disconnect0N。実機 bringup は stop_on_feedback_timeout=true 固定。

## 実装担当の選択

- 選択: **codex-terra**。理由: micon 複数pkg横断 + launch新設 + 削除掃除 + テスト付きリファクタ = 通常〜中規模、既存構造準拠。
- レビュー: 実装後に別モデル(Sol もしくは sonnet-coder)で実施。実装者≠レビュアー維持。

## ログ

- 2026-07-12: コード調査・要求整理・.ai 構築(メインClaude)。オープン論点をユーザ確認→D1〜D4 確定。
- 2026-07-12: Codex 一時利用不可(gpt-5.6-sol 要CLI更新)→ model=gpt-5.5 回避で復帰(.ai/escalations/)。
- 2026-07-12: Codex Sol 設計レビュー実施→設計確定(.ai/reviews/)。実装担当=terra を選択。
- 2026-07-12: Codex 書込 sandbox 不可(bwrap)→ ユーザ承認で danger-full-access 実行。Terra 実装完了。
  環境は ROS2 Jazzy と判明(Humble 未導入)。Jazzy で build/test/dry-run 成功。
- 2026-07-12: 実装レビュー(Opus, 実装者と別モデル)= **合格**。独立に build + 44 tests/0 fail を再確認。
  残課題: SW E-stop 未接続(ユーザ確認事項)/ 実 HW 一括起動は艇上工程。→ .ai/reviews/2026-07-12-review-terra-impl.md
- ステータス: **実装+レビュー完了**。テスト失敗ゼロのため Sol/Fable 昇格なし。

---

# サブタスク2 — カメラ実機bringup (zed2i / back_cam) 正常化

最終更新: 2026-07-12 / 記録者: メインClaude(司令塔)

## 背景
実機bringupで zed2i と back_cam を「ちゃんと使える」状態にする。ユーザ仮説「両方USB2.0認識で本体設定が悪い」の検証を含む。

## 現状把握(計測結果 2026-07-12)
- USB実速度: **ZED 2i 映像=USB3.0 (5000M, Bus002) で正常**。HID制御IFは12M(正常)。
  back_cam(Adesso CyberTrack H7)=USB2.0 (480M)。**Adessoは仕様上USB2.0のUVC機器**なので480Mは正常。
  → **USB速度に関する本体設定不良は無し。sudoでのUSB修正は不要**(ユーザ仮説を計測で否定)。
- 権限: user は video(44)/dialout(20) 所属済。ZED udev `/etc/udev/rules.d/99-slabs.rules` 存在。usb_cam 導入済。ZED SDK `/usr/local/zed` 存在。→ 権限系の追加sudoも不要。
- 電源モード: nvpmodel=15W。Orin Nano Super は上位モード(MAXN 等)可 → **性能面の任意sudoのみ**。
- デバイス実割当: ZED=`/dev/video0,1`, back_cam(Adesso)=`/dev/video2,3`。
- **根本バグ(デバイス割当が逆)**:
  - `robot/launch/back_cam.launch.py` 既定 `/dev/video0` = ZED左目(誤)。
  - `robot/launch/real_bringup.launch.py` `device` 既定 `/dev/video0`(誤, back_camへ渡る)。
  - `zed2i_driver/config/zed2i_cpu.yaml` / `zed2i_jetson_orin_nano.yaml` = `/dev/video2,3` = Adesso(誤)。
  - `/dev/videoN` は挿抜/再起動で不安定 → by-id 安定パスへ。
- cpu_stereo_node は `cv::VideoCapture.open(std::string, CAP_ANY)` = パス文字列OK → by-id 使用可。SDKモードは /dev/video を使わない。
- Adesso 対応フォーマット: MJPG 640x480@30 対応 → `pixel_format: mjpeg2rgb` 妥当。

### 確定 by-id パス
- ZED 左: `/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0`
- ZED 右: `/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index1`
- back_cam: `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0`

## 要求(Requirements)
- CR1[必須]: back_cam は Adesso(背面webカメラ)を開く。ZEDを掴まない。
- CR2[必須]: zed2i cpu系config は ZED センサを指す。Adessoを掴まない。
- CR3[必須]: 両カメラのデバイス指定を `/dev/v4l/by-id/` 安定パス既定にする(引数で上書き可)。
- CR4[必須]: real_bringup が zed2i(sdkモード)+back_cam を正しい device で配線。
- CR5[必須]: back_cam の pixel_format/解像度/fps が Adesso 対応値。
- CR6[推奨]: by-id を使う理由の短いコメント/メモを残す。

## 受け入れ条件(Acceptance)
- CAC1: `colcon build --packages-select robot zed2i_driver` 成功(config/launch install)。
- CAC2: `ros2 launch robot back_cam.launch.py --show-args` と real_bringup の device 既定が Adesso by-id。
- CAC3: zed2i cpu/orin yaml の left/right_device が ZED by-id。
- CAC4: sim 経路(task*_sim / manual_control)非破壊。
- CAC5: 実HW通電(実映像トピック確認)は艇上工程 or 別途(本タスク完了ラインはcode/build/dryrunまで)。

## sudo必要リスト(ユーザ実行・任意)
- カメラ動作に**必須のsudoは無し**(権限/udev/導入すべて充足)。
- 任意(性能): `sudo nvpmodel -m 0`(MAXN) + `sudo jetson_clocks`。ZED SDK/深度演算・点群の余裕確保。
  ※電源/放熱の許容内で。艇上運用の消費電力方針に依存するためユーザ判断。

## ルーティング判断
- Sol設計レビュー: **省略**(config/launchのデバイスパス修正のみで設計論点が無い軽微タスク)。
- 実装担当: 環境制約(Codex書込=bwrap不可, danger-full-access要再承認)を踏まえ、急ピッチ優先で **sonnet-coder**(native書込, 文脈保持)。規模的にはterra相当だが環境ブロッカー回避。
- レビュー: 実装者と別モデル(Codex Sol read-only もしくは Opus)。

## ログ
- 2026-07-12: 計測・要求整理(メインClaude)。USB仮説を否定、根本原因=デバイス割当逆+/dev/videoN不安定 と特定。sonnet-coder へ実装依頼。
- 2026-07-12: sonnet-coder 実装完了(4ファイル: back_cam.launch.py / real_bringup.launch.py / zed2i_cpu.yaml / zed2i_jetson_orin_nano.yaml を by-id 化)。colcon build(robot, zed2i_driver)成功、--show-args で既定値一致。
- 2026-07-12: Codex Sol(gpt-5.5, read-only)レビュー = **合格**(ブロッカーなし, Low指摘2=SN0001一意性/SDKモードyaml未使用)。司令塔が差分独立確認。→ .ai/reviews/2026-07-12-review-camera-sonnet-impl.md
- ステータス: **サブタスク2 実装+レビュー完了**。昇格なし。CAC5(実映像通電)=艇上工程。任意sudo(nvpmodel MAXN)はユーザ判断待ち。

### 実機起動確認 (2026-07-12, ユーザ指示)
- **back_cam: 合格**。by-id 既定で usb_cam が `/dev/../../video2` 化して起動失敗するバグを発見 → sonnet-coder が launch時 os.path.realpath 解決で修正 → 司令塔独立確認で `/back_cam/image_raw` ~26-30Hz 配信。**完了**。
- **ZED SDK(既定): ブロッカー**。GPU/CUDA 未検出でクラッシュ(`NO GPU DETECTED`)。nvidia-smi も `No devices found`、`/proc/driver/nvidia/gpus/` 空、`/dev/nvgpu/igpu0` に power ノードのみ、nvidia(open)+nvgpu(legacy)混在。**ホスト側GPUドライバ問題=要sudo/ホスト作業**。→ .ai/escalations/2026-07-12-zed-gpu-cuda-blocker.md
- **ZED CPU: 不適**。ZED は単一UVCデバイスのため cpu_stereo_node(左右別デバイス前提)で right open 失敗。node改修は別タスク。SDK が本線。
- README 整備を sonnet-coder に依頼中(zed2i_driver / robot)。
- **ユーザ待ち**: escalation 記載の sudo 手順1(dmesg採取)+手順2(reboot)→ 出力共有で次手判断。
- 2026-07-12: dmesg採取で根本原因確定 = **nvgpu ACR bootstrap failure**(iGPU firmware セキュア初期化失敗, 毎ブート決定的)。Codex Sol 相談 → 「再起動だけでは直らない/非sudo・コード側は対処不能/打ち切り妥当」。是正は L4T再導入→reboot、最悪reflash(要sudo・ホスト工程)。→ escalation 更新。
- ステータス: **カメラ実装+レビュー+READMEは完了しコミット/push済(eae1e97)**。ZED実配信のみGPU firmwareブロッカーで保留。ユーザがsudo是正+reboot後にZED確認を再開。
- 2026-07-12 夕: **ZED完全復旧・完了**。(1)ユーザのL4T reinstall+reboot でGPU(ACR)復旧、(2)calibファイル `SN34432991.conf` を `/usr/local/zed/settings/` に配置(URL param は `SN=`)、(3)メインClaudeが `ros2 launch zed2i_driver zed2i.launch.py` で left/right/depth/points 実配信(~6-8Hz)確認。詳細=escalation更新。
- Qiita備忘録: sonnet-coder が `qiita_draft.md` 作成(GPU復旧手順)。司令塔が内容検証済(実測値のみ・推測明記)。
- **最終ステータス: back_cam・ZED ともに実機配信確認済み=完了。** 任意改善: ZED depth NEURAL化 / nvpmodel MAXN で fps 向上。

---

# サブタスク3 — Livox Mid-360 / Mid-360s 両対応

最終更新: 2026-07-12 / 記録者: メインClaude(司令塔)
ステータス: **設計フェーズ（実装前・Sol合意待ち）**

## 要求
Livox Mid-360 と Mid-360s の両方を robot bringup で使用可能にする。

## 前提の想定（要確認）
- 「両対応」= 搭載された1台を機種選択して使う（selectable）。同時2台運用は想定外。
  根拠: 単一ASVでLivoxは1台構成。

## 現状把握（コード調査）
- SDK (Livox-SDK2) は両機種を既にサポート。
  - device_type: `kLivoxLidarTypeMid360 = 9`, `kLivoxLidarTypeMid360s = 35` (include/livox_lidar_def.h)
  - 機種判定は driver config の **トップレベルJSONキー**。`parse_cfg_file.cpp` の
    `dev_type_map = {"HAP":.., "MID360":9, "Mid360s":35}`。
  - ポートは両機種同一 (comm/define.h の kMid360* と kMid360s* 同値)。
- ROS driver config `config/MID360_config.json`:
  - `lidar_summary_info.lidar_type: 8`(=kLivoxLidarType, driverのSDK系フラグ, 機種非依存)
  - net-infoブロックキー `"MID360"`
  - `lidar_configs` 配列(extrinsic, 機種非依存)
- Launch `launch_ROS2/msg_MID360_launch.py` は configパスをモジュール変数でハードコード, launch引数を受けない。
- `real_bringup.launch.py` が `enable_mid360` 条件で include。CMakeは config/launch_ROS2 を丸ごと INSTALL_TO_SHARE。

## 初期設計案（最小・加算的）
1. `config/MID360S_config.json` 新規: `MID360_config.json` のコピーで
   net-infoブロックキー `"MID360"` → `"Mid360s"` のみ変更(lidar_type=8, lidar_configs, ports, IP据え置き)。
2. 機種選択を launch 引数化。案: (A)vendored launch複製 / (B)msg_MID360_launch.pyを引数化 /
   (C)robot側に lidar.launch.py 新設しconfig切替(vendored無改変)。暫定推奨=(C)。
3. `real_bringup.launch.py` に `lidar_model`(mid360既定|mid360s)引数を追加しconfig選択。

## リスク／未解決
- Mid-360s 実機デフォルトIPが .151 と異なる可能性 → config可変で吸収。
- 実機テスト不可 → launch/JSON妥当性のスモークテストで代替。
- frame_id/topic 両機種 `livox_frame` 共有(selectable前提で競合なし)。

## 確定設計（案C修正版・レビュー2で合意）
- config を submodule でなく **robot 側** `src/robot/config/livox/{MID360_config.json, MID360S_config.json}` に配置。
  MID360S は net-info ブロックキー "MID360"→"Mid360s" のみ変更。lidar_type=8/lidar_configs/ports/IP 据え置き。
- `src/robot/launch/lidar.launch.py` 新設: vendored `livox_ros_driver2_node` 直接起動。
  `lidar_model`(mid360既定|mid360s, choices検証) で user_config_path 切替。9 param 完全転記。
- `real_bringup.launch.py`: `lidar_model` 引数追加、include を lidar.launch.py へ差替、enable_mid360 コメント。
- `robot/package.xml` に exec_depend livox_ros_driver2 追加。
- `robot/test/test_livox_config.py`(pytest) で config 妥当性を担保。
- 詳細: docs/ai-discussion.md, .ai/reviews/2026-07-12-review1/2-mid360s-design.md

## 実装担当の選択
- 選択: **sonnet-coder**（Codex 上限のため。native 書込・文脈保持。robot 単一pkg内の加算的変更＝中規模）。
- レビュー: 実装後に Opus（司令塔）で独立検証。Codex 復帰後に Sol 追認。

## 進行ログ
- 2026-07-12: コード調査・初期案作成(メインClaude)。
- 2026-07-12: Codex 利用上限(~20:48復帰) → ユーザ判断で Claude系暫定レビュー+Sol後追認に切替(.ai/escalations/)。
- 2026-07-12: 設計レビュー R1(fable代役)→H1(submodule)/H2(param漏れ)/M1(lidar_type)ほか全採用。司令塔が submodule/glim/NIC を独立検証。案C修正版へ(.ai/reviews/review1)。
- 2026-07-12: 設計レビュー R2(修正版)→**合意**。Med3点(choices/exec_depend/pytest)追記(.ai/reviews/review2)。往復2回で終了。docs/ai-discussion.md 保存。
- 2026-07-12: sonnet-coder 実装完了（robot pkg 内6ファイル、submodule 無改変）。
- 2026-07-12: 実装レビュー(Opus, 実装者と別)= **合格**。独立に build + 53 tests/0 failure + dry-run(show-args/fail-fast/config解決) 再確認(.ai/reviews/review-mid360s-impl.md)。
- ステータス: **実装+レビュー完了**。テスト失敗ゼロで昇格なし。残: Codex 復帰後の Sol 設計追認(~20:48)。実HW通電は艇上工程。

## 追加調査 — upstream取り込み + 実機検証(2026-07-12 夕)
- ユーザ指示: 「公式最新版に対してbehindなら取り込め」「同様エラーのweb事例調査」。
- Livox-SDK2: fork=公式v1.3.1と機能的に完全同一(パッケージング差分のみ)。**更新不要**。
- livox_ros_driver2: fork は公式1.2.4相当、**1.2.6(commit 13eb05e)にbehind**。
  実質差分= `src/comm/pub_handler.cpp` 1行(Mid-360sのline_num処理漏れ)。**submoduleに適用済み(未commit, ユーザ承認済み)**。
- Web調査: GitHub Issue #240(Livox-SDK/livox_ros_driver2)が同一症状("Init lds lidar successfully!"だがトピック無し)を報告、公式回答が1.2.6修正で解決と明言。
- 実機検証: 実機は config のプレースホルダIP(.151)と異なる `192.168.1.114` に存在すると判明(ハンドル値をIPデコードして特定)。
  pub_handler.cpp修正 + 実IP補正の一時テストconfigで **完全解消**: `Can not get index`=0件、`/livox/lidar` `/livox/imu` トピック生成、`ros2 topic hz /livox/lidar`=**平均10.0Hz**実測。実機Mid-360s相当機とのe2e疎通を確認。
- 詳細: `.ai/reviews/2026-07-12-mid360s-upstream-fix-and-live-verify.md`
- **完了(ユーザ指示反映)**: MID360=`.151`既定維持、MID360S=`.114`既定へ変更。
  livox_ros_driver2 submodule: masterをfast-forward後 `pub_handler.cpp` 修正をcommit(`e0382eb`)し
  `KeioRoboticsAssociation/livox_ros_driver2` master へ push 済み(`95d96d3..e0382eb`)。
  親repo側 submodule ポインタ更新は未commit(次のcommit時にまとめて)。
  rebuild(`robot`,`livox_ros_driver2`)成功、robotテスト53件/0failure再確認。
