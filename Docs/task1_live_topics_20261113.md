# Task1 sim topic監査（2026-11-13）

稼働中のROS domain 89を確認した。Missionの直近結果はTask1成功（state=8）、
現在のMission状態はIDLE、Controlの実効出力源はZERO、`/cmd_vel` は全軸ゼロだった。
実行中のsim・GUI接続を止めず、購読による確認だけを行った。

## 一覧と受信状況

全topic一覧： [topics.csv](topic_audit_20261113/topics.csv)

採取データ： [snapshot.json](topic_audit_20261113/snapshot.json)

Action内部topicを含め135 topic。publisherあり120、観測中に受信69、
publisherなしのsubscriber-onlyが15。Foxglove bridgeが実際に購読するtopicは28だった。

| topic | 確認結果 |
| --- | --- |
| `/control/state`, `/control/requirements` | 個別確認で約20 Hz。適用中の条件とAUTO阻害理由を受信 |
| `/cmd_vel` | 個別確認で約20 Hz、surge/sway/yawは全て0 |
| `/mission/status` | 約10 Hz。IDLE、直近メッセージは経路完了 |
| `/mission/last_result` | 保持されたTask1成功結果を受信 |
| `/mission/plan` | 保持された10 poseの経路を受信 |
| `/odometry/filtered/local`, `/odom`, `/tf` | 約18–20 Hzで受信 |
| `/sensor/vehicle_gnss/fix/raw`, compass | 約9–10 Hzで受信 |
| `/health/state` | 約8–10 Hzで受信 |
| `/buoy_detections_3d` | 受信あり。現在の検出数は0 |
| `/virtual_obstacles`, `/virtual_obstacle_markers` | 2 Hzで受信。現在は0点・0 marker |
| `/task1/cardinal_wall_enable` | false。Task1終了後なので壁が空なのは正常 |
| `/sim/estimated_path` | 2 Hz、2,400 poseの履歴を送信 |
| `/tf_static` | 保持された静的TFを受信。周期送信でない |

全体測定はdiscovery待ち8秒後に約12秒、必須topicは別途15秒観測した。
QoSはpublisherに合わせ、保持データも受信した。CSVのHzは観測サンプル数/秒であり、
設定上のpublish頻度を保証しない。保持データ1件の0.08 Hz表示は周期送信を意味しない。
Action feedback、Nav2経路・lookahead、状態遷移などはIDLE中に無送信でも異常とは限らない。
監査用subscriberは測定終了時に破棄し、一覧のsubscriber数・名前から除外した。

## subscriber-onlyの15 topic

| 分類 | topic |
| --- | --- |
| Task2用 | `/planned_path_pruned`, `/task2/points_filtered`, `/mission/task2/goal_reached` |
| 停泊補正用 | `/docking/berth1_corrected`, `/docking/berth2_corrected` |
| runtime切替・failsafe用 | `/runtime/nav2/status`, `/system/configure/_action/feedback`, `/system/configure/_action/status`, `/mission/failsafe/return_home` |
| 手動入力用 | `/cmd_vel_manual`, `/cmd_vel_sbus` |
| Nav2の任意入力 | `/goal_pose`, `/speed_limit`, `/global_costmap/footprint`, `/local_costmap/footprint` |

Task1で使わない汎用ノードの入力や、外部から与える任意入力が含まれる。
publisherがなければtopicデータは送られず、登録だけで大量の帯域を使うわけではない。
削除するなら各機能で必要になる時だけ購読を作る方法があるが、現時点で動作不良を示してはいない。

Foxgloveのtopic一覧に広告されることと、実際に購読されることも別である。
[公式bridgeの実装](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp)では、
clientのsubscribe要求を受けてROS subscriptionを作り、購読者を参照カウントしている。

## 実際の負荷と整理候補

監査の大量購読がない別の約5秒間で、Windows接続へのTCP送信は約0.814 MB/s（6.51 Mbps）、
bridge CPUは1コア換算49.6%、RSSは142 MiBだった。短時間の観測値であり、
実機のカメラ・LiDAR運用や別レイアウトの負荷を保証するものではない。

| 主な送信候補 | 観測と推奨 |
| --- | --- |
| `/sim/estimated_path` | ROS payloadだけで約0.384 MB/s。静止後も2,400 pose全体を2 Hz送る。位置変化時の追加・終了後の保持・間引きが最優先の改善候補 |
| `/global_costmap/costmap` と `/global_costmap/obstacle_layer` | 両方のupdateを購読。各updateは概算0.177 MB/s。通常監視は統合costmapだけに絞れる |
| local costmapとそのobstacle layer | 同様に重複表示を整理できる |
| `/received_global_plan`, `/lookahead_collision_arc` | 現在は無送信。追従デバッグ時だけ購読する候補 |
| `/mission/plan`, `/mission/last_result`, 状態・センサ概要 | 操作・判断に必要なので残す |

各topicのpayload概算は最初の受信サイズ×観測頻度で、変動するupdateの正確な平均帯域ではない。
保持データの初回配信も含む。TCP値は実際の接続のbytes_sent差分で測定した。

結論：subscriber-onlyの数より、実際に送っている大きなデータの重複が整理対象。
Task1専用レイアウトでcostmapの重複購読を外し、航跡の送信を軽くする順がよい。
航跡はTask1の理想経路との比較に必要なので、単に非表示にするより送信方法を改善する。
今回、実行中の設定・topic・レイアウト・ナビゲーションコードは変更していない。
