# GNSS位置・方位の静止ばらつき（2026-11-15）

## 条件

AGX、UM982ドライバ単体。推進・Nav2は起動せず、終了後にドライバを停止。
23:00〜23:03 ESTに180秒記録。アンテナは未固定で、ユーザー指定の概略座標を仮採用した。
main=(-0.3, +0.3) m、sub=(+0.7, -0.3) m（船体x前方/y左方向）。
main→sub=(1.0, -0.6) m、基線長1.16619 m、基線角-0.5404195002705842 rad。
船首ENU yaw = 90度 - 受信機THS方位 - 基線角。
NTRIP無効、単独測位。高さは未指定のためURDFの既存値を維持した。

実測の基準位置・方位はないため、これは絶対精度ではなく、静止していたと仮定した時間方向のばらつき。
アンテナの実際の動きやマルチパス、測位ドリフトは分離していない。
写真による取付け補正の確認・固定後の再評価が必要。

## 結果

ROS位置3599件、方位3599件。位置statusは全件0（STATUS_FIX、単独測位相当）。
位置19.99994 Hz、方位20.00138 Hz。最大受信間隔は位置55.3 ms、方位69.4 ms。

| 指標 | mainアンテナ | 船体原点換算 |
| --- | ---: | ---: |
| 東西標準偏差 | 0.15760 m | 0.15747 m |
| 南北標準偏差 | 0.23124 m | 0.23143 m |
| 平均位置周りの水平RMS | 0.27981 m | 0.27988 m |
| 平均位置からの距離の95パーセンタイル | 0.45620 m | 0.45619 m |
| 平均位置からの最大距離 | 0.54971 m | 0.54967 m |

mainアンテナ位置の開始点→終了点は0.17358 m、出力高度の標準偏差0.31738 m。
船体原点は、方位をunwrapして位置サンプル時刻へ線形補間し、mainの水平レバーアームを回転して引いた。
平均位置を原点とした局所WGS84接平面で集計。95%半径は真値に対する誤差限界や将来の保証ではない。

| 方位指標 | 値 |
| --- | ---: |
| 補正後の船首方位の円平均（北0度・時計回り） | 248.29573度 |
| 円平均周りの角度標準偏差 | 0.13853度 |
| 円平均からの絶対偏差の95パーセンタイル | 0.27601度 |
| 最大−最小 | 0.90190度 |

生ログ全体（ROS観測の前後も含む）はGGA quality=1が3865件、THS有効Aが3864件、使用衛星24〜28。
チェックサム不一致4行あり、原因は未切り分け。ROS出力covarianceの固定方位精度0.5度ではなく、
上記は実測サンプルから算出した統計値。

## 修正と検証

実際にコンパイルされるsrc/parsing.cppでNavSatFix.status未設定だったためUNKNOWNになっていた。
quality=1/3はSTATUS_FIX、2/4/5はSTATUS_GBAS_FIX、それ以外はSTATUS_NO_FIXへ変換。
測位不可はfeedback odometryに流さない。ROS標準NavSatStatusにSINGLEという列挙値はなく、
単独測位はSTATUS_FIX=0で表す。RTK fixed/floatの区別はこの標準statusだけでは表せない。

launch既定値と3つのURDFで指定座標・基線角を更新。ドライバを再ビルド・install済み。
ドライバgtest 8件合格。円周跨ぎ統計・レバーアーム換算・URDF座標の軽量検証、diffチェックも合格。
既存の未使用関数finite_quaternionのビルド警告あり。

## 再現と保存

ROS環境をsourceし、CYCLONEDDS_URIを実在するsrc/bringup/robot/config/cyclonedds_agx.xmlへ設定。

```bash
ros2 launch um982_driver um982.launch.py rtk_enable:=false \
  heading_baseline_yaw_rad:=-0.5404195002705842 \
  log_file_name:=/home/ibo/rosbags/gnss_heading_check_20261115/stationary_fixed_offsets.raw
python3 scripts/operations/gnss_stationary_check.py --seconds 180 --output /新しい保存先
```

保存先: /home/ibo/rosbags/gnss_heading_check_20261115/
- stationary_fixed_offsets.raw: 受信機生ログ
- stationary_fixed_offsets/samples.json: ROS時系列
- stationary_fixed_offsets/summary.json: 統計（船体原点換算を含む）

位置情報を含む時系列はGitに含めていない。既存のスラスター設定・ntripcaster変更は本作業に含めていない。
