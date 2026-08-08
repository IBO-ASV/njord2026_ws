このpkgはnjord2026のTask 2衝突回避シミュレーションをするpkgです。

`ros2 launch task2_sim task2_sim.launch.py` は、他船を**認識済みと仮定**して
相手船TFから `/other_ship/twist` を生成し、MPPI → Nav2 FollowPath →
`/cmd_vel` を理想追従する運動モデルまでを検証します。スラスタドライバ・
推力配分・船体力学は起動しません。実LiDAR認識器も起動しません。

このpkgは、
* task2_simのための以下のlaunchを含みます
  * `/cmd_vel` 理想追従のキネマティック運動モデル
  * nav2
  * configに緯度経度で指定されたGPSpoint2つ
  * marker vessel the otter of Njordの投影範囲
  * 下記自分のnode
* yoloの代替として、時々刻々と微妙に位置が変わる赤と緑のブイのtfをnav2に渡す。
* start（初期位置を与える必要があれば初期位置も）とgoal（こちらはGPSpoint4の半径~mに入ったら）を出せるようにする。
などtask2_simのorchestratorを担当します。
