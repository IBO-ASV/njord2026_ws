# GLIM静止記録の事前確認（2026-11-15 EST）

## 結果

初回はLiDAR点群・IMUを受信できず記録を見送った。その後、ユーザーの許可を受けてゼロ推力で非常停止を解除し、約5分間の静止bag記録に成功した。詳細は末尾の再試験結果を参照。

## 現状確認

- ホスト: ibo-agx-orin。通常ROS domainは5、RMWはCycloneDDS。
- 開始時にUM982ドライバ、NTRIP caster/relay、GNSS静止観測プロセスが稼働。
- UM982側のプロセス、シリアル、補正、設定、ネットワーク設定は変更していない。
- 通常domainで確認できたセンサtopicはGNSS fix/compass。LiDAR/IMUの既存配信はなかった。共有TF topicも一覧になかった。
- end0は192.168.1.5/24、LOWER_UP。設定上のLiDAR IP 192.168.1.114および192.168.1.151はping無応答、neighbor状態FAILED。
- メモリ約54 GiB空き、load average約0.82/0.55/1.75。保存先の空きは2.7 GB（使用率95%）。
- 継承環境のCYCLONEDDS_URIは存在しない旧パスを参照。調査コマンドだけで解除、または現行設定パスを指定した。永続設定は未変更。

## 受信確認

ROS_DOMAIN_ID=83、ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST、CYCLONEDDS_URI未指定で、既存のrobot/lidar.launch.pyを起動。
enable_glim:=false、enable_buoy_detection:=false、lidar_config_role:=agxとし、MID360S設定、続いてMID360設定を逐次確認した。

両設定ともドライバとIMU scalerのロードは成功したが、/livox/lidarは生成されなかった。
/livox/imuはscalerのpublisherが存在するだけで、実メッセージを確認できなかった。
GLIM、EKF、TF publisher、制御系は起動していない。確認用ドライバは停止した。

起動ログ:

- /home/ibo/.ros/log/2026-11-15-23-11-01-342204-ibo-agx-orin-92312
- /home/ibo/.ros/log/2026-11-15-23-11-40-727673-ibo-agx-orin-92789

最初のドライバはSIGINT/SIGTERMで終了せず、launchの終了処理によりSIGKILLで停止した。

## 次に必要なこと

LiDARの電源表示、電源・LAN配線、実機型番を現地で確認する。通信復旧後、点群とIMUの実メッセージ・周期を確認してから記録を開始する。
記録前に容量を再確認し、必要なら別の保存先または短い記録時間を選ぶ。

## Soft EMGを含む再確認（同日）

現地から、LiDAR給電にSoft EMG側の電源操作が必要との情報を受けた。
domain 83でCP2102Nのby-idポートに専用serial_writerを起動した（UM982のCH340ポートは使用していない）。
推力入力を`/glim_test/unused_thrust`に限定し、`/soft_emg=true`を5 Hzで送信した。
`/safety/emergency_stop=1`と`/micon/relay_active=true`の応答を確認したが、MID360Sの点群・IMU実データと192.168.1.114へのping応答は得られなかった。

firmware/firm1ではtrueは非常停止要求で、FETをLOWにする。電源投入を意味する「ON」とROS Boolの極性が逆になる可能性があるため、falseへの切り替え（非常停止解除）はユーザーに確認した上で実施する。
この段階ではbag記録は未実施。前回の無応答だけではLiDAR故障やネットワーク不良とは判断できず、給電条件が未確認だった点を補足する。
確認待ちのため、試験用のSoft EMG publisher、serial_writer、LiDARドライバを終了した。serial_writerの終了処理はゼロ推力・非常停止要求を送信する。

## ゼロ推力で非常停止解除後の再試験

ユーザーの明示的な許可を受け、domain 83で`/glim_test/zero_thrust`に`[0,0,0,0]`を約20 Hzで送信し、`/soft_emg=false`を継続送信した。推力入力を専用topicに限定し、制御・自律航行ノードは起動していない。
`/safety/emergency_stop=0`、`/micon/relay_active=false`となり、192.168.1.151へのpingが復帰した（2/2応答）。192.168.1.114は引き続き無応答。
`lidar_model:=mid360 lidar_config_role:=agx`へ切り替えると、SDKのNormalモード移行とIMU有効化が成功し、点群・IMUを受信した。

- ライブ点群周期: 約9.99 Hz。
- ライブSI単位IMU周期: 約200.0 Hz。GLIMに入力する際のacc_scaleは1.0。
- 20秒間のIMU観測: 4,001件、平均加速度`[0.18652, 0.01521, 9.78040]` m/s²、各軸標準偏差`[0.04399, 0.05438, 0.05819]` m/s²。
- 点群サンプル: frameはlivox_frame、20,064点、point_step=26。x/y/z/intensity/tag/line/timestampフィールドを確認。
- 保存先: `/home/ibo/rosbags/glim_static_20261115_2324/`。
- 記録対象: `/livox/lidar`、`/livox/imu`、`/livox/imu_raw`、`/micon/relay_active`、`/safety/emergency_stop`、`/soft_emg`、`/glim_test/zero_thrust`。
- MCAP zstd_fast、250 MBで分割。空き900 MBを下回れば早期終了する条件で、300秒記録を開始。

UM982のCH340ポート、driver、NTRIP補正には操作していない。GLIM/EKFと共有TF publisherは起動していない。

### 保存データの検証と終了状態

- 記録コマンドは300.45秒で正常終了（exit 0）。bag内の実データ時間幅は299.122秒。
- bag: `/home/ibo/rosbags/glim_static_20261115_2324/input_bag`、MCAP 3ファイル、528.0 MiB。
- 点群: 2,992件、header時刻基準10.0000 Hz、最大間隔0.102456秒。
- SI単位IMU: 59,822件、200.0042 Hz、最大間隔0.008631秒。raw IMUは59,824件。
- 点群・IMUともheader時刻の逆行・同時刻は0件。IMUの加速度・角速度の非有限値は0件。
- ゼロ推力指令5,658件を全件検査し、4要素すべてゼロ。記録中のSoft EMGはfalse、emergency_stopは0、relay_activeはfalseのみ。
- 詳細結果: 同じ保存先の`record_summary.json`、`input_observation.json`、`bag_validation.json`、`record.log`。
- 記録終了後、ゼロ推力送信を止め、`/soft_emg=true`を明示送信。`/safety/emergency_stop=1`を確認してからserial_writerを終了。試験用LiDARドライバとpublisherも終了した。
- 終了時の空きは約2.2 GiB。UM982 driverとNTRIP casterの稼働を確認した。

今回確認したのは静止時の入力配信・保存の健全性であり、GLIM自体の推定精度や静止ドリフトはまだ評価していない。
