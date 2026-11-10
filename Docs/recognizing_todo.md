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
