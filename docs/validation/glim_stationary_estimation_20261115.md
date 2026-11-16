# GLIM静止bag推定評価（2026-11-15 EST）

## 結論

約5分間の静止データでGPU版GLIMが10 Hzの推定を継続した。位置は数cmの範囲に収まり、継続的に大きく流れる挙動は見られなかった。一方、静止中も速度出力に数cm/sの残差がある。静止安定性の確認であり、移動精度や絶対精度の合格判定ではない。

## 入力・実行条件

- ホスト: ibo-agx-orin。GLIM / glim_ros 1.2.2（CUDA13.1版）、GPU odometry backend。
- 入力: `/home/ibo/rosbags/glim_static_20261115_2324/input_bag`。船体静止の条件で採取した約299.12秒、点群2,992件、SI単位IMU59,822件。
- ROS domain 84、CycloneDDS loopback限定。再生は点群とSI単位IMUのみ、1倍速で1回、`use_sim_time=true`。
- `glim_config_headless`を評価先へ実体コピー。IMU topicは`/livox/imu`、acc_scale=1.0。点群timestampは絶対ナノ秒、scale=1e-9を維持。
- base_frame_idを空にしてセンサ基準の生出力を評価。map_frame_id=map、ローカル・グローバルmapping無効、GLIMのTFはprivate topicへremap。
- GNSS融合、EKF、センサ・シリアルドライバ、制御系は起動していない。UM982/NTRIPの設定・プロセスを操作していない。
- LiDAR–IMU外部パラメータは既存値`[0.006,-0.012,0.008,0,0,0,1]`を維持。今回の実機に対する校正は未実施。

## 結果

| 指標 | 全出力区間 | 最初の出力から30秒を除外 |
|---|---:|---:|
| 出力数 | 2,981 | 2,680 |
| 出力の時刻幅 | 298.000秒 | 267.901秒 |
| 開始点から終了点の水平変位 | 0.42 cm | 1.32 cm |
| 開始点からの最大水平変位 | 4.31 cm | 3.85 cm |
| 水平位置の95%半径（区間内中央値中心） | 2.15 cm | 2.17 cm |
| 最初と最後の各10秒平均位置の水平差 | 0.089 cm | 0.472 cm |
| yawの最大−最小 | 0.182° | 0.182° |
| roll / pitchの最大−最小 | 0.206° / 0.246° | 0.166° / 0.202° |
| 水平速度の中央値 / 95%値 | 0.0392 / 0.0821 m/s | 0.0394 / 0.0831 m/s |
| 3D速度の中央値 / 95%値 | 0.0479 / 0.0889 m/s | 0.0481 / 0.0893 m/s |

- header時刻基準の平均出力周期は9.99999 Hz、最大間隔0.102456秒。逆行・同時刻は0件、非有限値は0件、quaternion normの最大誤差は約1.2e-15。
- 最初の出力は最初の点群より約1.10秒後。最後の入力点群まで出力があり、初期化後の出力は10 Hzで連続している。
- 再生clockと出力stampの差は、30秒除外後に中央値0.121秒、95%値0.131秒、最大0.151秒。これは再生・通信・推定を含む観測遅れの近似であり、GLIM単体の計算時間ではない。
- GLIM、bag player、recorderはすべてexit 0。ログに推定失敗・時刻異常・クラッシュは見られなかった。評価プロセスはすべて終了し、UM982 driver / NTRIP casterの稼働を確認。

## 解釈と制限

静止中の位置は数cmの揺れに留まり、今回の環境で入力・初期化・連続推定が成立した。小さい終端差だけを精度と解釈せず、95%半径や最大変位も併記した。
静止時の水平速度95%値は約8.3 cm/sで、速度を直接使う制御・相対速度補償ではこの残差を考慮する必要がある。今回はフィルタ調整やEKFへの統合は行っていない。
出力pose covarianceは全件ゼロであり、その値から推定誤差や信頼度を評価できない。
独立した測量真値、GNSS比較、移動データ、外部パラメータ校正はなく、絶対位置精度・動的精度・水上性能は未評価。

## 再現用成果物

`/home/ibo/rosbags/glim_static_20261115_2324/glim_eval/`:

- `run_manifest.json`: 実行コマンド、終了コード、domain、IMU倍率。
- `config/`、`cyclonedds_loopback.xml`: 実際の評価設定。
- `glim.log`、`play.log`、`record.log`、`output_bag/`: ログと生odometry・clock。
- `summary.json`、`odometry.csv`、`stationary_drift.png`: 集計、時系列、図。
- `analyze.py`: `python3 analyze.py`で保存済みoutput_bagから再集計できる。

関連: [静止記録と給電条件の確認](glim_stationary_precheck_20261115.md)。
