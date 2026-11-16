# RTK GNSS + GLIM ライブ融合試験（2026-11-15 EST）

## 結果

実機MID360の点群・SI単位IMUとRTK FIXのUM982で、既存のlocal/global分離EKFをライブ実行した。
GLIM単独より静止位置のばらつきは小さくなったが、同時刻のRTK単独より改善したとは言えない。
今回の完了条件は融合の連続出力、座標変換・TF整合、静止時の比較。動的精度や操船性能は未評価。

作業開始前に既存4 commitを143ed28cまでorigin/masterへpushした。
ユーザーの既存のスラスター設定・ntripcasterサブモジュール変更は今回のcommit対象外。

## 構成・実機条件

```text
MID360(.151) + IMU(SI) → GPU GLIMの生センサpose
 → glim_base_odom_adapter（取付け補正・body速度）
 → local EKF → odom→base_link、/odometry/filtered/local

RTK FIX GNSS位置 → navsat_map → GNSS位置(map)
GNSS絶対ENU方位 ───────────────┐
local EKFのbody速度 ───────────┼→ global EKF
GNSS位置(map) ─────────────────┘  → map→odom、/odometry/filtered/global
```

- AGX、ROS domain 5、CycloneDDSは実在するsrc/bringup/robot/config/cyclonedds_agx.xmlを使用。
- 補正経路は前回のntrip1.bizstation.jp:2101/3C66B072→AGX内RTCM3→UM982を継続。
- main=(-0.3,+0.3)、sub=(+0.7,-0.3) mの仮座標、テープ除去後・静止条件を継続。
- GNSSドライバはframe修正を反映するため単体で再起動した。FIX継続を確認。
- LiDARは前回許可されたゼロ推力の給電方法を使用。CP2102Nのserial_writerを
  専用/fusion_test/zero_thrustだけに接続し、4要素ゼロと/soft_emg=falseを継続送信。
  スラスターミキサー・操縦・Nav2・ミッションは起動していない。
- MID360は192.168.1.151、AGX end0は192.168.1.5。MID360S(.114)は使っていない。
- IMU scaler後の/livox/imuを使い、glim_acc_scale=1.0。
- GNSSの補正済み絶対方位は姿勢のみのIMUへ変換し、global EKFへ渡す。
- global推定位置はlocal GLIM/EKFへ戻さない。GLIMのTFは/task2/glim/private_tfへ隔離。
- GLIMアダプターの共分散は既存の仮定値。LiDAR–IMU・船体取付け外部パラメータの校正は未実施。

## 修正

1. UM982のGGA位置のframeをbase_linkからgnss_linkへ修正。
   GGAはmainアンテナ位置なので、navsat_transformがURDFの実際の水平レバーアームを扱えるようにした。
2. task2_glim_localizationの実行時設定でbase_frame_idを空にし、GLIMは生センサposeを出す。
   後段アダプターが取付け変換を担当し、GLIM側との二重適用を避ける。
   実機でraw child=livox_frame、変換後child=base_linkを確認。
3. global EKFのprint_diagnosticsをtrueにした。
   実出力があるのに周波数診断がNo events recordedと出る事象を修正した。
   同パラメータがfalseだと周波数診断を登録しながらtickしない実装に対応する。
   参考: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/ros_filter.cpp
   記録後にglobal EKFだけ再起動し、15秒で300出力、診断level=0を確認。

## 記録と静止比較

評価bagは約184秒。最初の30秒を除いた約154秒を比較した。
初期疎通bagはfusion_bag、正式な評価はfusion_eval_bagを使用。
初期のPython観測スクリプトはROS MessageInfoのAPI差で停止したため、正式値は保存済みMCAPを直接解析した。
TF発行元の数は別のC++プローブでpublisher GIDを15秒間収集して確認した。

| 指標 | GLIM→local EKF | GNSS位置(map投影・レバー補正) | global融合後 |
| --- | ---: | ---: | ---: |
| 評価サンプル数 | 1539 | 1540 | 3064 |
| 実際の出力周期 | 10.000 Hz | 10.000 Hz | 19.903 Hz |
| 平均位置周りの水平RMS | 10.56 mm | 5.52 mm | 5.61 mm |
| 平均位置からの95%半径 | 18.06 mm | 8.82 mm | 9.09 mm |
| 隣接出力間の最大水平差 | 23.64 mm | 7.57 mm | 8.12 mm |
| 静止時の水平速度95%値 | 0.0370 m/s | 位置のみ | 0.0317 m/s |
| 観測時刻−header時刻の95%値 | 0.152 s | 0.052 s | 0.0115 s |

GLIM生センサposeの水平RMSは12.05 mm、船体変換後は11.86 mm。
表の各系統はそれぞれの平均位置を中心としている。localとmapの座標値を直接一致させた比較ではない。
遅延はbag記録時刻とheader時刻の差で、単体の計算時間ではない。
local EKF設定は30 Hzだが、実際の新しいstampを持つ出力は入力に対応して約10 Hzだった。

- 融合後の平均ENU yaw: -159.22618度（北0度時計回りの方位では約249.22618度）。
- 融合後yaw標準偏差: 0.09053度。
- 時刻を合わせたGNSS方位との差: RMS 0.002634度、絶対差95%値0.005119度。
- map上のGNSS投影位置との差: RMS 1.724 mm、距離95%値3.482 mm。
- 対象poseの非有限値0、header時刻の逆行・同時刻0。
- IMU約200 Hz、GLIM約10 Hz、GNSS約20 Hzが継続。
- 評価bagのゼロ推力3585件は全件[0,0,0,0]。Soft EMG=false、安全状態0のみ。
- 動的TFはmap→odom、odom→base_linkの両方で発行元1つ。global EKF再起動後にも再確認。

RTK単独の静止位置が既に安定しているため、この測定から融合の絶対精度向上は主張しない。
GNSS方位を入力しているので、GNSS方位との一致も独立した方位精度の証明ではない。
静止中も速度推定に残差があり、速度制御用のゲイン調整や動的性能確認は別途必要。

## 稼働状態と再現

試験後も、ユーザーが確認できるようGNSS・補正中継・LiDAR・融合系・Bridgeを稼働させている。
ゼロ推力専用publisherとserial_writerも稼働中。通常の操縦ノードは起動していない。
常駐サービスへの登録は行っていない。

既存GNSS/補正が稼働し、前回のゼロ推力給電が成立している状態で使った主要コマンド:

```bash
source install/setup.bash
export CYCLONEDDS_URI=file:///home/ibo/njord2026_ws/src/bringup/robot/config/cyclonedds_agx.xml
ros2 launch robot lidar.launch.py lidar_model:=mid360 lidar_config_role:=agx \
  enable_glim:=false enable_buoy_detection:=false
ros2 launch robot onboard_localization.launch.py glim_acc_scale:=1.0
```

二つのlaunchは別ターミナルで実行する。既存ノードとの二重起動は避ける。
今回のglobal EKFは診断設定反映のためlaunch内の元プロセスを止め、同名の単体プロセスで再起動済み。
新規起動時は上記launchだけで修正済み設定を読む。
終了時は融合とLiDARを止め、給電用publisherからゼロ推力とSoft EMG=trueを送って確認してからserial_writerを止める。

GUI接続は引き続きws://10.42.0.233:8765。
融合結果は/odometry/filtered/global、GLIMローカル結果は/odometry/filtered/local。
GNSS Map Telemetryの地図マーカーは引き続き生GNSS位置・GNSS方位を表示するため、
そのマーカーだけで融合結果の改善を判定しない。

## 検証と保存先

- um982_driver・robotビルド成功。
- 起動構成・GLIM取付け変換・実行時profileのpytest計90件合格。
- UM982 gtest 8件合格。新しいprofileテストはCTest経由でも合格。
- YAML構文、diffチェック合格。

/home/ibo/rosbags/gnss_glim_fusion_20261115_2345/:
- fusion_eval_bag/: 正式なMCAP評価bag（LiDAR生点群は含めず、GLIM生odom・EKF入力/出力等を記録）。
- fusion_summary.json、fusion_observations.json: 再集計結果・時系列。
- fusion_stationary.png: 比較図。
- tf_authority.txt、tf_authority_final.txt: 動的TF発行元の確認。
- final_check.json: 診断修正後の実機確認。
- glim_runtime_config/: 実行時GLIM設定。
- analysis/: 再集計・図作成・C++プローブのソース。

GNSSの生ログは/home/ibo/rosbags/gnss_heading_check_20261115/gnss_glim_fusion.raw。
位置情報を含むbag・図・時系列はGitに追加していない。
