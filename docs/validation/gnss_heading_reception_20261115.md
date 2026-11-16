# GNSS・方位の受信確認（2026-11-15）

AGX実機でUM982ドライバのみ起動。推進・Nav2・ミッションは起動していない。
アンテナは未固定のため、船首方位との一致・精度・取付け補正は評価対象外。
NTRIPは無効。受信機設定はvolatileのみで、終了時にUNLOGされた。

## 実施

22:54:20〜22:55:08 EST付近。ROS観測時間30秒。
CYCLONEDDS_URIの既存値は旧src/robot/configパスを指し起動に失敗したため、
実行プロセスのみsrc/bringup/robot/config/cyclonedds_agx.xmlを指定した。
install/setup.bashをsourceし、以下を実行して観測後にSIGINTで停止した。

```bash
ros2 launch um982_driver um982.launch.py rtk_enable:=false \
  log_file_name:=/tmp/gnss_heading_check_20261115.raw
```

## 結果

- ROS位置600件、方位601件（約20 Hz）。
- 生ログのチェックサム正常データ: GNGGA 929件、GNGST 928件、GNTHS 928件。
- GGA qualityは全件1（単独測位）、使用衛星数24〜28。
- GNTHSは全件有効フラグA。受信機方位279.2529〜279.995度。
- ROS方位のENU yawは-28.37895〜-27.76645度。これは取付け補正等を含み得る値で、受信機方位そのものや検証済み船首方位ではない。
- チェックサム不一致4行あり。今回は原因・発生時刻を切り分けていない。
- **ROS NavSatFix.statusは600件すべて-2（STATUS_UNKNOWN）。生データの測位状態と不整合。**
  現在のsrc/node.cppはquality=1をSTATUS_FIXに設定しており、実行済みinstall成果物との不一致が疑われる。
  原因確定・再ビルドは未実施。自律航行用GNSS入力の合格とはしない。
- ROS yaw covarianceは固定値のため、実測精度の証拠にはしない。

結論: 受信機の位置・有効な方位出力とROSトピック配信を確認。
アンテナ固定後の方位校正・精度確認、ROS status不整合の解消は残る。

## 保存先

生ログと30秒間のROS観測JSONを以下へ保存（位置情報を含むためGitには追加していない）。

- /home/ibo/rosbags/gnss_heading_check_20261115/gnss_heading_check_20261115.raw
- /home/ibo/rosbags/gnss_heading_check_20261115/gnss_heading_check_20261115_observer.json

既存のthruster設定変更とntripcasterサブモジュール変更は本作業に含めていない。

## 続報: UNKNOWNの原因

後続調査で、node.cppの当該実装は `#if 0` 内にあり、実際にはparsing.cppが使われていることを確認。
parsing.cppのparse_ggaがstatusを設定していなかったことが原因であり、上記のinstall成果物不一致という疑いは取り下げる。
後続の静止ばらつき測定で修正と実機検証を実施する。
