# 統合方針と残課題

現行の優先事項を先に示し、その下に過去の統合記録・提案を保存します。
GitHubのIssue・PR状態は再照会していません。過去の `Pending` は当時の統合管理状態であり、
現在の機能実装の有無とは一致しません。現行構成はパッケージREADMEと検証記録を優先してください。

## 現行の残作業（2026-11-13更新）

実機前のソフトウェア作業:

- [x] `ntripcaster`の入れ子submoduleを初期化し、system libev未導入時の同梱版fallbackを追加。
  通常のcolconで`ntripcaster`・`robot`・`njord_sim`のビルドを確認。
- [x] suiteからMMG・外乱・初期pose offsetを指定できるようにする。
- [x] Task2の他船加減速・周期的な認識欠落fixtureを追加。真値と物理障害物は保持する。
- [x] Foxgloveの重複costmap表示を既定OFFにし、航跡の静止中再送を抑制。
  航跡は保持配信と履歴間引きに対応し、全履歴の送信量を制限する。
- [x] 古いTODOと現行の受入記録を分離。
- [x] 全タスクMMG truth matrixを実行（5/6合格）。
- [x] ノイズ・外乱・初期offset・Task2加減速/認識欠落の追加matrixを実行（8/9合格）。
  不合格も保存し、下記のソフトウェア残課題へ反映。

- [ ] **ソフトウェア優先課題:** MMGのTask2 head_onで接触（他船間隔0 m）。
  実速度・旋回/減速応答を考慮した避航予測と追従の調整・再試験。
- [ ] **追加条件のTask2 starboard:** ブイ接触判定（余裕 -0.0058 m）と45秒進展なし。
  初期pose offset・外乱・他船加減速・認識欠落の複合条件を切り分け、
  ゲート経路・追従余裕・停止後の再計画を修正して再試験。

追加matrixの結果と残る調整は [simulation acceptance](../validation/simulation_acceptance.md) を参照。
試験機能の追加を、すべての条件での合格や実機適合と同一視しない。

実機で優先する作業:

- [ ] 手動操縦、推進方向、停止、非常停止、通信・センサ欠測時の安全動作。
- [ ] 屋外GNSS/RTK・Spatial初期化、GLIM/EKF・heading・TFの実測と共分散調整。
- [ ] 水上認識精度と遅延、カメラ/LiDAR/GLIM同時処理、10分以上の連続負荷測定。
- [ ] 地上局の実画面操作、映像・通信・復帰、状態灯の実配線。
- [ ] 低速Task1.2→Task1→停泊→Task2→Task4。実測船体応答をMMGと制御へ反映。

GUI、Task1 sim、diagnosticのコードはすでに存在するため、下記の古い項目を
そのまま「未着手一覧」として使わない。GUIは操作性と実回線、diagnosticは実センサ欠測と
配線を確認する。性能ロードマップの未実装記載も履歴を含み、現行GPU経路は
`src/robot/README.md`とcamera packageを優先する。

---

以下は過去の記録です。枝分け・Issue状態を推測で完了扱いにはしていません。

---

# Branch Decomposition Tracking

This document tracks the decomposition of the historical `4-task1` and
`test0506` branches into reviewable changes for `master`.

GitHub tracking issue: [#11](https://github.com/IBO-ASV/njord2026_ws/issues/11)

## Source preservation

| Source branch | Archived tip | Archive tag |
|---|---:|---|
| `4-task1` | `c020411` | `archive/4-task1-before-decomposition-20260615` |
| `test0506` | `6f8b986` | `archive/test0506-before-decomposition-20260615` |

The source branches must not be merged directly. Each accepted change is
reconstructed from the latest `master`, scoped to one topic, and reviewed
independently.

## Integration order

| Order | Topic | Planned branch | Source commits | Status |
|---:|---|---|---|---|
| 1 | CI baseline | `chore/ci-baseline`, `fix/ci-ros-container` | new | Merged in [#12](https://github.com/IBO-ASV/njord2026_ws/pull/12) and [#25](https://github.com/IBO-ASV/njord2026_ws/pull/25) |
| 2 | Dependency and submodule cleanup ([#14](https://github.com/IBO-ASV/njord2026_ws/issues/14)) | `chore/dependency-submodules` | `791ddf0`, `82e42c7`, `6f8b986` | Pending review |
| 3 | GLIM, EKF, and localization ([#13](https://github.com/IBO-ASV/njord2026_ws/issues/13)) | `feature/glim-ekf-localization` | `791ddf0`, `4bde855`, `e31390c`, `c020411` | Merged in [#24](https://github.com/IBO-ASV/njord2026_ws/pull/24) |
| 4 | Manual control and Micon agent ([#15](https://github.com/IBO-ASV/njord2026_ws/issues/15)) | `feature/manual-control-micon-agent` | `791ddf0`, `bf12ca9`, `82e42c7`, `919d5cf`, `8dc48fa` | Pending |
| 5 | X4 omni thruster control ([#16](https://github.com/IBO-ASV/njord2026_ws/issues/16)) | `feature/x4-omni-thruster-control` | `8dc48fa`, `18ab245`, `e4464f1`, `7d8634f`, `c020411` | Pending |
| 6 | Buoy perception pipeline ([#17](https://github.com/IBO-ASV/njord2026_ws/issues/17)) | `feature/buoy-perception-pipeline` | `09bc35e`, `528f308`, `82e42c7` | Merged directly in `26c6076` |
| 7 | Nav2 waypoint and smoothing ([#18](https://github.com/IBO-ASV/njord2026_ws/issues/18)) | `feature/nav2-waypoint-smoothing` | `58c545b`, `9f8d770`, `4bde855`, `f481f46`, `c020411` | Merged directly in `51a543a` |
| 8 | Task1 simulation ([#19](https://github.com/IBO-ASV/njord2026_ws/issues/19)) | `feature/task1-simulation` | `31ba23e`, `28eb2ad`, `82e42c7`, `09bc35e` | Pending |
| 9 | Task3 navigation and simulation ([#20](https://github.com/IBO-ASV/njord2026_ws/issues/20)) | `feature/task3-navigation-simulation` | Task3 commits through `c020411` | Merged directly in `4b25e34` |
| 10 | Visualization and documentation ([#21](https://github.com/IBO-ASV/njord2026_ws/issues/21)) | `docs/architecture-simulation` | `a3d180c`, `18ab245`, `8e6d589`, `c020411` | Pending |
| 11 | `test0506`-only changes ([#22](https://github.com/IBO-ASV/njord2026_ws/issues/22)) | Investigation issues only | `6f8b986` | Hold |

## Explicitly held or excluded

- `.antigravity/rules`
- empty `Docs/about_*.md` files
- empty `src/robot/launch/task2_all.launch.py` and `task3_all.launch.py`
- incomplete `manual_with_odom` package
- empty `cubic_spline` package
- empty `src/ros2.repos`
- `test0506` emergency-stop inversion and manual thrust gain increase
- `test0506` GNSS status semantic change
- unrelated formatting-only changes

## Required checks

Every integration branch must:

1. be based on the latest accepted `master`;
2. contain only its documented topic;
3. pass `git diff --check`;
4. pass changed XML, YAML, and Python syntax checks;
5. build and test affected ROS packages;
6. record unavailable hardware or simulation validation explicitly.

Hardware-dependent changes remain on hold until their hardware behavior is
confirmed.

## Known validation debt

- `simple_sim` was an obsolete lightweight experiment for `/cmd_vel`, `/odom`,
  and path-following checks. It was removed instead of restoring its old lint
  test coverage because the maintained simulation work now lives under
  `src/sim`.


---

# master へ段階的に取り込むブランチ案

`origin/master` から作成する統合用ブランチ名の候補です。大きな機能単位で切り、依存する土台を先に取り込みます。各ブランチでは現ブランチ全体を merge せず、対象機能のコミットを cherry-pick する想定です。

```text
integrate/
├── platform/
│   ├── agx-bringup
│   ├── ground-station-links
│   ├── dds-zenoh-network
│   ├── agx-rosbag-service-ui
│   └── runtime-parameter-sync
├── localization/
│   ├── um982-driver-config
│   ├── glim-livox-imu
│   └── ekf-tf-heading
├── control/
│   ├── manual-sbus-arbitration
│   ├── soft-emergency-stop
│   ├── critical-link
│   └── thruster-live-tuning
├── perception/
│   ├── task1-onboard-detection
│   ├── task2-vessel-tracking
│   └── foxglove-visualization
├── missions/
│   ├── task1-waypoints
│   ├── task2-opponent-selection
│   └── task4-dolly-missions
├── simulation/
│   ├── task1-sensor-parity
│   ├── task2-kinematics
│   └── task3-virtual-walls
└── tooling/
    ├── sensor-bringup
    ├── experiment-validation
    └── environment-lock
```

## 取り込み順の目安

1. `platform/dds-zenoh-network` と `platform/ground-station-links`
2. `platform/agx-bringup`、`platform/runtime-parameter-sync`、`platform/agx-rosbag-service-ui`
3. `localization/*`
4. `control/*`
5. `perception/*` と `missions/*`（特に Task 2 は localization と perception の依存を確認）
6. `simulation/*` と `tooling/*`

ブランチ名は候補です。実際の分割では、共通ファイルの重複変更やコミット間依存を確認し、複数項目にまたがる変更は依存先のブランチに寄せてください。Foxglove の可視化・実験記録・環境ロックは個別機能の動作に必須か確認してから取り込みます。


---

### 過去のTODOメモ（現行状況は冒頭を参照）
* [ ] 1~5 knot checker : memo
* [ ] GUI
* [ ]

センサやtopic通信が途絶えていないかをチェックするためのdiagnosticを発行したいです。
/home/osw/njord2026_ws/src/diagnostic
の下に、
* USB to CANの接続状況（socket CANで通信し、Heatbeatを出します）。具体的な仕様を決めてないので、一旦pkgだけ作ってください
* LiDARの接続状況mid360をつなぎます。EtherがUPかどうか、もしくはpointcloudが出続けているかどうかなどから判断します。
* miniPC 2 PC間通信（GUIの更新が止まるので一瞬でわかるとは思いますが、一応）
* IMUの接続、dev/ttyUSB~でつながるのを見てもいいし、topicを見てもいいです。
*  CAMEraの接続、dev/camera0?でつながるのを見てもいいし、topicを見てもいいです。
* GNSSの接続、bluetoothか有線のどちらかです。それぞれ別で作ってください。

---

## `simple_manual/manual_control.launch.py` を使う動作確認の優先順位

`manual_control.launch.py` は操縦系だけでなく、MID360S、ZED2i、後方カメラ、
UM982、Drogger、Localization（Spatial、GLIM、local/global EKF）も起動する。
Spatial用のGNSSアンテナが不足しているため、**既存のUM982/Droggerのアンテナを
接続したまま確認できる項目を先に完了し、最後にDroggerまたはUM982のアンテナを
Spatialへ付け替える**。これにより、付け替えはSpatial試験開始時と原状復帰時だけで
済む。

| 優先度 | 確認する系統 | 実施内容・完了条件 | アンテナ |
| --- | --- | --- | --- |
| P0 | 安全・起動前確認 | 船体を安全に固定し、推進器の周囲を無人にする。Jetson・地上PCを同一Wi-Fiへ接続し、コントローラーをBluetooth接続する。 | 変更なし |
| P1 | 操縦・推進（最優先） | 地上PCで`joy_node`を起動して接続メッセージを確認する。Jetsonで`ros2 launch simple_manual manual_control.launch.py`を起動し、`/joy`、joy_converterの出力、`/thruster_command`を`ros2 topic echo`で確認する。ボタン割り当て、前後・左右・ヨーの向き、停止時に推力指令が0になることを確認する。マイコンと通信できない場合は`micon_driver_fd`/`serial_writer`を先に切り分ける。 | 変更なし |
| P2 | 常設センサ・認識・Localizationの起動 | 同じ起動のままMID360S、ZED2i、後方カメラ、前後YOLO、UM982、Drogger、Spatial、GLIM、EKFのノード・topicが起動することを確認する。まず起動失敗、USB/serial競合、画像/点群未配信を解消する。点群ブイ検出は現在launch内でコメントアウトされているため、この起動だけでは確認対象外とする。 | UM982/Droggerは現状のまま。SpatialはアンテナなしでもROS通信・IMU topicのみ確認 |
| P3 | UM982・Droggerの単体GNSS | 屋外・天空視界ありで、両者が現在のアンテナ構成でfixを取得すること、座標が妥当であること、UM982の`/sensor/vehicle_gnss/fix/raw`とDroggerの設定topic（既定`/gnss/fix`）が継続配信されることを確認する。RTKは通常fixが安定してから実施する。 | 変更なし |
| P4 | SpatialのGNSS/INS（アンテナ付け替え後） | launchを停止してから、原則として**Droggerのアンテナを外してSpatialへ接続**する（コネクタや試験目的の都合でUM982を外す場合は、外した機器と復旧先を記録する）。屋外で再起動し、SpatialのGNSS fix、Navigation Filter初期化、`/odometry/gps/spatial`、global EKFの安定性を確認する。単体数分間の位置ばらつきを記録し、EKF共分散設定の根拠にする。最後にRTK対応を試験する。 | DroggerまたはUM982からSpatialへ1本移設 |
| P5 | 原状復帰・最終確認 | Spatial試験を記録した後、アンテナを元の機器へ戻す。UM982/Droggerの両方が再び認識されることと、P1の停止操作（0推力）を再確認して終了する。 | 元の構成へ復帰 |

### Spatial試験（P4）の判定

- `/adnav_driver/system_status` に `GNSS FAILURE`、`GNSS ANTENNA FAULT`、
  `SERIAL PORT DATA OVERFLOW` がない。
- `/adnav_driver/filter_status` で `Navigation Filter Initialised` と
  `Internal GNSS Enabled` を確認する。
- `/adnav_driver/nav_sat_fix` の `status.status` が `-1` ではなく、
  `/odometry/gps/spatial` と `/odometry/filtered/global` が不自然な飛びなく配信される。
- Spatialで取得した単体測位の分散・RTK有無の結果を記録してから、
  `ekf_global.yaml` のGNSS共分散を調整する。

> アンテナの抜き差しは、対象GNSSと`manual_control.launch.py`を停止してから行う。
> 付け替え前後の接続先・時刻・fix状態を記録し、P5で確実に元へ戻す。

スラスター関連
joy_nodeは地上PCで立ち上げる。コントローラーをbluetoothで繋いで、joy_nodeを起動すると、OPEN ~~ Joystickみたいなメッセージが流れる
* joy_converterのボタン割り当てをmanual_control.launch.pyに書いてあるので、ros2 topic echoして割り当てが正しそうか確認する
* thruster_controllerのconfigファイルに速度制御パラメータが入っている。反転・最大推力などそこで変えられる
* manual_control.launch.pyからの起動はthruster_controllerのlaunchファイルを呼び出してるだけのはず。
* micon_driver_fd。呼び出さなくていいはずだが、マイコンと通信できてなさそうだったらそこが悪い

GNSS関連

* spatialにアンテナを繋げて上手くいくか.
* シンプルに単体で数分置いたとき、精度がどれくらいでるか調べないといけない。調べたらekfの共分散行列の対角行列に入れる.
* RTK対応試験

GLIM

* robot/config/glim_config/〜でコメントアウトで複数設定用意するので、精度と動作周期、htopの食い具合どんな感じか.
* glimとyoloをcomposableに出来るか.

ekf

* mid360sのimuを使ってたら、船体に取り付けた状態で分散とらないと
* RTK有無でGNSSの分散変えられるように
* 出来ればekfの有無でどれくらい精度変わるか

yolo

* 画質（VGAかHP720）とモデルの組み合わせで浮かべて、認識精度・周波数確認
* ZED 2i driverが調光関連をちゃんとやってるか


その他

* Alert_lamp pkgの動確が出来なかったのでお願いします.
* 地上通信とGUI
* external aruco calibration.
