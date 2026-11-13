# task1-1 当日動作確認チェックリスト

目的は、Task 1.1（測線追従）の**安全な手動航行、GNSS の絶対位置、LiDAR
オドメトリ、TF、経路追従**を、段階ごとに切り分けて確認することである。
今日は共分散の最適化よりも、各系統が一貫した座標系・時刻で動くことを完了条件にする。

## 結論（先に決めること）

- **UM982/Drogger のEKF用共分散合わせ込みは、今日の必須条件ではない。**
  両者のメッセージに妥当な covariance が入り、単体測位と融合後の軌跡に大きな跳び・
  持続的な振動がなければ、まずはその値で Task 1.1 の疎通・低速走行を行う。
- ただし、covariance がゼロ・NaN・極端に小さい、または GNSS がGLIM推定を不自然に
  引っ張る場合は、そのまま自律航行に進まない。GNSS融合を一旦止めるか、実測に基づく
  値へ直す。
- **TFの第一発行者はGLIMのままにする。** 現行構成ではGLIMが
  `map -> odom -> base_link`、EKFは `/odometry/filtered/local` と
  `/odometry/filtered/global` のtopicだけを発行する設計である。
- 「GLIMのTFが出なければEKFの `publish_tf` をtrue」は今日の即時回避策にしない。
  現行 `ekf_local.yaml` はGLIM odom入力がコメントアウトされ、LiDAR IMUだけを入力して
  いる。その状態のEKF TFには平面位置の根拠がない。またGLIMが一部のTFを発行している
  場合に同じ動的edgeを二重発行する危険がある。

## 0. 安全と記録

- [ ] 船体を係留または陸上で固定し、プロペラ周囲を立入禁止にする。
- [ ] 非常停止・停止操作を確認し、最初は `enable_thruster:=false` で起動する。
- [ ] Jetson/地上PCの時刻、同一ROS_DOMAIN_ID、ネットワークを確認する。
- [ ] 日時、場所、天候、アンテナ接続先、使用branch/commit、起動コマンドを記録する。
- [ ] 以降の各段階で、成功/失敗、topic周波数、警告ログ、bag名を記録する。

## 1. センサ単体（推進を有効にしない）

起動例（必要なデバイス名は実機に合わせて明示する）。

```bash
source install/setup.bash
ros2 launch simple_manual manual_control.launch.py \
  enable_thruster:=false enable_nav2:=false lidar_model:=mid360s
```

- [ ] MID360S: `/livox/lidar` と `/livox/imu` が継続配信される。
- [ ] GLIM: `/odom` が継続配信され、静止時に大きく流れず、短い低速移動に追従する。
- [ ] UM982: `/sensor/vehicle_gnss/fix/raw` が屋外で継続配信される。
- [ ] Drogger: `/gnss/fix` が屋外で継続配信される（現状のEKFはこのtopicを融合していないので、
  単体確認として記録する）。
- [ ] GNSSの `status`、緯度経度、covariance が妥当で、RTKの有無・fix種別を記録する。
- [ ] `/diagnostics` と各ノードログにserial競合、時刻逆行、TF lookup failureがない。

よく使う観測コマンド：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
ros2 topic echo --once /sensor/vehicle_gnss/fix/raw
ros2 topic echo --once /gnss/fix
ros2 topic echo --once /diagnostics
```

## 2. TFを最優先で切り分ける

GLIMへ点群・IMUが入っていることと、GLIMのTFが成立していることは別の確認項目である。
TFがない場合は、Nav2や経路追従の試験へ進まない。

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 topic info /tf -v
ros2 topic echo --once /odom
```

合格条件：

- [ ] `base_link -> livox_frame` は `robot_state_publisher` から静的に存在する。
- [ ] `map -> odom` と `odom -> base_link` が連続して取得できる。
- [ ] `/tf` の各動的edgeに発行者が一つだけである。
- [ ] `/odom` の `header.frame_id` と `child_frame_id` が期待する `odom` と `base_link` であり、
  TFと矛盾しない。

### GLIM TFが出ないときの順序

1. `view_frames` と `tf2_echo` で、**どのedgeが欠けているか**を記録する。RVizの表示だけで
   TF欠落と判断しない。
2. `/odom` のframe名、GLIMログ、`config_ros.json` の `base_frame_id`、`odom_frame_id`、
   `map_frame_id` を照合する。現設定は `base_link`、`odom`、`map` である。
3. `base_link -> livox_frame` がない場合はURDF/robot_state_publisherを直す。GLIM/EKFを
   変更して補わない。
4. `/odom` はあるが `map -> odom` または `odom -> base_link` がない場合は、GLIMの起動ログ・
   GLIM版・設定読込先・frame名不一致を先に解消する。点群topicだけが正常でも合格ではない。
5. `/odom` も出ない場合は、LiDAR/IMU時刻、GLIM初期化、CPU/GPU負荷、入力QoSを確認する。

### 2026-07-23の `tf2_monitor` 画面に対応する判定

共有された画面では、`base_link`、`livox_frame`、`odom` と各センサframeは見えている一方で、
**`map` frameが表示されていない**。したがって、少なくともこの実行時点では
`map -> odom` はTFツリーに存在せず、`map` を使うTask 1経路・Nav2は開始不可である。
まずGLIMが `odom -> base_link` だけを出しているのか、TFを全く出していないのかを次で確定する。

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /tf --qos-reliability best_effort
ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable
ros2 node info /glim_node
```

| 観測結果 | 考えられる原因 | 対処・次の判定 |
| --- | --- | --- |
| `odom -> base_link` は取れるが `map -> odom` が取れない（今回の最有力） | GLIMがodometryモードだけで動作している、mapping初期化/設定読込に失敗している、またはGLIMが実装上 `map -> odom` を出さない | GLIM起動ログで実際に読んだ `config_path` とmappingの有効化を確認する。`config_ros.json` の `enable_local_mapping`/`enable_global_mapping` が有効な設定を読んでいることを確認する。GLIM版の仕様で `map -> odom` を出さないと判明した場合に限り、GLIMのdynamic TFを止めた隔離環境でEKFを唯一のTF発行者にする設計へ変更する。 |
| `odom -> base_link` も取れず、`/odom` は出る | `/odom` のframe名とGLIM設定のframe名が不一致、GLIMのTF publicationが無効、または別ROS domain/namespaceを見ている | `ros2 topic echo --once /odom` で `header.frame_id`/`child_frame_id` を記録し、`config_ros.json` の `odom_frame_id: odom` と `base_frame_id: base_link` に合わせる。GLIMが読込んだ設定ファイルをログで確認してから再起動する。 |
| `base_link -> livox_frame` が取れない | URDF/xacro、robot_state_publisher、または静的TFのQoS/起動順の問題 | `ros2 node info /robot_state_publisher` と `/tf_static` を確認する。URDF側を修正し、GLIM/EKFで代替しない。 |
| `tf2_echo` が Extrapolation into the past/future を出す、または `odom` の遅延が継続して1秒超 | LiDAR/IMUの時刻とPC時刻のずれ、GLIM処理遅延、CPU/GPU飽和、入力キュー滞留 | `/livox/imu`、`/livox/lidar`、`/odom`、`/tf` のheader stampと現在時刻を比較する。JetsonとセンサをNTP/PTP等で同期し、GLIMの処理負荷・入力rate・キューを確認する。低速/自律試験は、動的TF遅延が安定して十分小さくなるまで行わない。 |

画面の `published by <no authority available>` は、**それだけでは発行ノード不在の証拠ではない**。
`tf2_monitor` が受け取った静的TFや既存キャッシュではauthorityを復元できないことがあるため、
原因判定には上の `tf2_echo`、`/tf`・`/tf_static` の中身、`ros2 node info` を使う。

画面の `gnss_link`、`livox_frame`、`spatial_link`、各thruster frame、`um982_link` などの
静的frameにある約876秒のdelayは、起動時に一度だけ送られた `/tf_static` を `tf2_monitor` が
現在時刻との差として表示している値であり、単独では故障ではない。`base_link` は動的edgeの
childになるべきなので、上記の `tf2_echo odom base_link` で個別に確認する。対して `odom` の
delayが約2〜11秒と出ている点は要調査である。再起動直後の一時値でなく、10秒以上の観測で
継続する場合は時刻/処理遅延として扱う。

## 3. EKFのTF発行へ切り替える条件

以下を**すべて満たしてから**、別試験としてEKFをTF発行者に切り替える。

- [ ] GLIMの問題が、設定読込・frame不一致・入力停止ではなく、GLIM自身のTF発行に限定されている。
- [ ] GLIM odometry（`/odom`）が安定し、`odom`/`base_link` frameで一貫して配信される。
- [ ] GLIMの動的TF発行を明示的に停止できる、またはGLIMを停止してEKFへodometry入力を
  与える手順が確立している。
- [ ] local EKFの `odom0: /odom` と `odom0_config` を有効化し、EKF出力が静止・低速移動で
  妥当であることを確認済みである。現在はこの入力がコメントアウトされているため、ここを
  満たさない。
- [ ] `map -> odom` と `odom -> base_link` のそれぞれについて、**GLIMかEKFのどちらが
  発行するかを一意に決めた**。二者同時発行は不可。

切替試験の合格条件は、EKFを発行者にした後に `view_frames` で動的edgeの発行者がEKFだけで、
`tf2_echo`・`/odometry/filtered/*`・Nav2のTF lookupが数分間安定することである。これを
満たさなければ `publish_tf` をfalseへ戻し、GLIM側の原因調査へ戻る。

> 注意: `publish_tf: true` にする対象と `world_frame` により、EKFが発行するedgeは変わる。
> YAMLを一行だけ変えて本番起動へ入れず、GLIMを止めた隔離試験・bag記録付きで実施する。

### `glim_base_link` を介してEKFをTF発行者にする案

この案は、**GLIMがTFを発行しないなら可能**であり、以前に使われていた「EKFがTFを発行する」
構成に近い。ただし、次のようなツリーは不可である。

```text
map -> glim_base_link -> base_link       # GLIM等がこの動的TFを発行
odom -> base_link                        # local EKFも発行
```

`base_link` に親が二つ（`glim_base_link` と `odom`）でき、TFツリーが壊れるためである。また
「GLIMはTFを発行しない」のに `map -> glim_base_link` が存在することもあり得ない。そのedgeを
発行する別ノードを明示する必要がある。

EKFを唯一の動的TF発行者にするなら、成立する形は以下である。

```text
map  -> odom             # global EKF
odom -> base_link        # local EKF
base_link -> glim_base_link  # 静的TF（GLIMの推定原点が船体原点と異なる場合だけ）
base_link -> livox_frame     # URDFの静的TF
```

この場合、GLIMは `/odom` を**measurement topicとしてだけ**出し、`/tf` には
`map -> glim_base_link`、`odom -> glim_base_link`、`odom -> base_link` を出さない。GLIMの
`/odom` の `child_frame_id` が `glim_base_link` でも、`base_link -> glim_base_link` の静的TFが
あればrobot_localizationはmeasurementを `base_link` 基準へ変換できる。GLIMの推定原点と
`base_link` が同一なら、余計な `glim_base_link` は作らず `base_link` を使う方が簡単である。

切替時の実施順：

1. GLIMのTF発行を無効化できることを確認する（設定がなければ、GLIMをTF無効で起動できる版・
   パラメータを確認するまで切り替えない）。
2. `glim_base_link` を使う必要がある場合だけ、実測した剛体変換をURDFへ
   `base_link -> glim_base_link` として追加する。ゼロ値を仮置きして航行試験しない。
3. local EKFで `odom0: /odom` を有効化し、`odom0_config`、frame名、covarianceを確認する。
   現行設定はここがコメントアウト中である。
4. local EKFだけ `publish_tf: true`、`world_frame: odom` として `odom -> base_link` を出す。
5. global EKFも `publish_tf: true`、`world_frame: map` として `map -> odom` を出す。両者の
   `base_link_frame` は同じ `base_link` にする。
6. `view_frames` と `tf2_echo map base_link` で、各edgeの発行者が一つだけであることを確認する。

履歴上はcommit `791ddf0`でlocal/global EKFがともに `publish_tf: true` だった。その後
`e31390c`でTF競合回避のためfalseに変更され、現行設定もfalseである。履歴に
`glim_base_link` は見つからないため、記憶にある構成はEKF TF発行そのもの、または未commitの
実験設定である可能性が高い。

## 4. GNSS・共分散の当日判定

今日の走行を進めてよい条件：

- [ ] UM982の `/sensor/vehicle_gnss/fix/raw` が安定し、測位点が実際の場所から明らかに外れない。
- [ ] `position_covariance` が有限値かつ正で、RTK/非RTKの状態と整合する。
- [ ] `/odometry/gps` と `/odometry/filtered/global` が連続配信され、GNSS更新のたびに数m級で
  跳ばない。
- [ ] 静止2〜5分と低速直進で、GNSS単体・GLIM・global EKFの軌跡をbagへ記録した。

今日中に共分散を調整すべき条件：

- [ ] covarianceがゼロ、NaN、未設定、または実際のばらつきより明らかに小さい。
- [ ] GNSS更新ごとにglobal EKFが大きく跳ぶ、またはGLIMとの位置差が持続的に増える。
- [ ] RTK fix/float/singleが混在するのに同じ過信した共分散を使っている。

その場合は値を推測で固定せず、まずGNSS融合を外したGLIM/local系とGNSS単体系を比較して記録し、
静止ログから水平位置の標準偏差を求めてから `ekf_global.yaml` 側の融合設定を調整する。

## 5. Task 1.1 経路・Nav2（TFと位置が合格してから）

```bash
ros2 launch waypoint_publisher waypoint_publisher.launch.py task_type:=task1
```

- [ ] `map` frameでTask 1 waypointが表示され、GPS 1を原点とする経路の向きが実水域と一致する。
- [ ] 船首方向を少し動かしたとき、`base_link`、GLIM odom、global位置、RViz表示の向きが一致する。
- [ ] Nav2を有効化する前に、手動・低速で1〜2 m動かし、TF欠落・自己位置の跳び・操舵符号反転が
  ないことを確認する。
- [ ] 最初の自律試験は短い1区間、十分低い速度、監視者と即時停止可能な操縦者を置いて行う。
- [ ] 失敗時は推進を停止し、bagとログを保存してから設定を変更する。

## 6. bagに残す最小セット

```bash
ros2 bag record -o task1_1_$(date +%Y%m%d_%H%M%S) \
  /tf /tf_static /odom /odometry/filtered/local /odometry/filtered/global \
  /odometry/gps /sensor/vehicle_gnss/fix/raw /gnss/fix \
  /livox/imu /livox/lidar /diagnostics
```

推進有効時は、これに `/joy`、joy_converter出力、`/thruster_command`、実際の推進器feedbackを
追加する。個人情報・位置情報の取り扱い範囲はチームのルールに従う。

## 終了判定

今日の最低限の完了は、(1) 安全に手動停止できる、(2) MID360S→GLIM→単一のTF連鎖が成立する、
(3) UM982のGNSSとglobal EKFが連続配信する、(4) Task 1経路が `map` で妥当に見える、の4点とする。
共分散の数値最適化とEKFのTF切替は、上記ログを根拠にする次段階の作業としてよい。
