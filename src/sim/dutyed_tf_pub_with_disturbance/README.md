このpkgはthruster_controllerが発行するduty比を受け取り、それにしたがって、base_link frameを移動させるnodeを含みます。

tf treeは
* world（gnssがmapへの変換を発行）
* map（lidarがodomへの変換を発行）
* odom（ekf nodeがbase_linkへの変換を発行）
* base_link（tf static publisherで各センサなどへの変換を発行）
となります。
ただし、simulationではworld→map・map→odomは固定です。

simulationのためのモデルとしては、
* T200の公称duty・thrustモデル
* MMG model
* 加わるベクトルが連続して変化するが変化の仕方がランダムなpose2dの二階微分に対する外乱
を用いて計算する。
これらのモデルはpure c++ libとして実装すること

MMG modelに必要な定数はRobotXの定数を参照するか、njord challengeで一般的っぽそうな値を適当に決めてください。
node_config.yamlにまとめてください。

