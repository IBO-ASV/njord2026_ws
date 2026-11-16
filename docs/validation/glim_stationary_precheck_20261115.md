# GLIM静止記録の事前確認（2026-11-15 EST）

## 結果

静止bag記録は未実施。LiDAR点群・IMUの実データを受信できず、空のbagは作成しなかった。

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
