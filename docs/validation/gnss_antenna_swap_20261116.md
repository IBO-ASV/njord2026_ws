# main/sub反転後のGNSS・融合再試験（2026-11-16 EST）

## 変更と条件

ユーザーがmainとsubの位置が逆になったことを確認したため、次を反映した。

- main=(+0.7,-0.3)、sub=(-0.3,+0.3) m。高さは未指定のため既存値1.0 mを維持。
- main→sub=(-1.0,+0.6) m、取付け角atan2(0.6,-1.0)=2.601173153319209 rad。
- UM982 launchの方位補正既定値、xacroと2種類の展開済みURDF、静止評価スクリプトのmain位置既定値を更新。
- main/sub反転で受信機の方位が180度変わっても、補正後の船首方位が変わらない回帰テストを追加。
- 3種類のURDFのmain/sub座標と方位補正の整合を検査するテストを追加。

ホストAGX、ROS domain 5。受信機は同じUM982、同じ3C66B072の補正を使用した。
NTRIPはntrip1.bizstation.jp:2101/3C66B072→AGX内RTCM3→UM982。
GNSS単体の評価区間はLiDAR・推進・融合系を起動せず取得した。
静止条件を前提とし、今回は独立した測量真値や取付け位置の実測校正を用意していない。

## GNSS単体

- ドライバ起動は00:13:48 EST付近。
- 最初のGGAは05:13:49.00 UTC、最初のFIX（quality=4）は05:13:52.80 UTC。
  出力開始から約3.8秒で再FIX。受信機の電源状態や内部状態を前回と統制していないため、コールドスタート性能の比較ではない。
- 生方位は約100度となり、反転前の約280度との差約180度と整合する。
  ROSパラメータ2.601173153319209 radが実際に読み込まれていることを確認。
- FIXを180秒以上連続維持。固定スナップショットの最後の180秒を正式評価とした。

正式区間: 05:14:52.25〜05:17:52.25 UTC（00:14:52.25〜00:17:52.25 EST）、GGA3601件、すべてquality=4。

| 指標 | mainアンテナ | 船体原点換算 |
| --- | ---: | ---: |
| 東西標準偏差 | 2.317 mm | 2.334 mm |
| 南北標準偏差 | 2.014 mm | 2.384 mm |
| 平均位置周りの水平RMS | 3.069 mm | 3.336 mm |
| 平均位置からの95%半径 | 5.286 mm | 5.660 mm |
| 平均位置からの最大距離 | 8.006 mm | 10.109 mm |

- 補正後平均船首方位: 248.98501度（北0度・時計回り）。
- 方位標準偏差: 0.14051度。絶対偏差の95%値: 0.27824度。
- GGA平均海面高度の標準偏差: 8.391 mm。
- GST推定標準偏差の中央値: 東西10 mm、南北11 mm、高さ28 mm。
- 別途ROSで180秒観測し、位置3599件・方位3598件、各約20 Hz。
  位置statusは全件2。標準NavSatStatusではFloat/FIXを区別できないので、生GGAでFIXを判定した。
- GNSS単体スナップショット全体のチェックサム不一致は4行。集計ではチェックサム正常なNMEAのみ使用した。

前回（主副交換前、別の180秒区間）の船体原点RMSは4.786 mm、95%半径8.251 mm、方位標準偏差0.16556度。
今回の値は小さいが、衛星配置・時刻・設置誤差・内部状態が変わっており、アンテナ交換の因果効果とは断定しない。
上記は静止時の時間方向のばらつきであり、絶対位置・絶対方位の誤差保証ではない。

## 融合側の取付け位置確認

GNSS単体のスナップショット保存後、前回許可されたゼロ推力の給電方法でMID360と融合系を起動した。
serial_writerは/antenna_swap/zero_thrustだけを入力にし、[0,0,0,0]と/soft_emg=falseを継続送信。
推進ミキサー・操縦・Nav2・ミッションは起動していない。

```bash
ros2 launch robot lidar.launch.py lidar_model:=mid360 lidar_config_role:=agx \
  enable_glim:=false enable_buoy_detection:=false
ros2 launch robot onboard_localization.launch.py glim_acc_scale:=1.0
```

- 実行中の静的TFでbase_link→gnss_link=(0.7,-0.3,1.0)、base_link→gnss_sub_link=(-0.3,0.3,1.0)を確認。
- GNSS fixのframeはgnss_link、融合出力はmap/base_link。
- map→odom、odom→base_linkは、それぞれ15秒のpublisher GID観測で発行元1つ。
- 約99秒のMCAPを保存し、最初の30秒を除外した約69秒の短時間評価を行った。

| 指標 | GNSS投影位置(map) | global融合後 |
| --- | ---: | ---: |
| 評価サンプル数 | 689 | 1370 |
| 実出力周期 | 10.000 Hz | 19.883 Hz |
| 平均位置周りの水平RMS | 2.498 mm | 2.630 mm |
| 平均位置からの95%半径 | 4.410 mm | 4.607 mm |

- 融合後のGNSS方位との差はRMS0.01657度、絶対差95%値0.005887度。
- 融合後の静止時水平速度95%値は0.0432 m/sで、速度出力の残差は残る。
- 診断の非正常levelなし、融合位置の非有限値・header時刻逆行なし。
- 記録中の推力指令は全件[0,0,0,0]、Soft EMG=false、安全状態0のみ。
- GNSS単体の正式180秒区間とは別時刻・別時間長の測定であり、両表の数値を融合による改善として直接比較しない。

## 終了状態

融合試験後、ゼロ推力とSoft EMG=trueを送り、/safety/emergency_stop=1を確認した。
LiDAR、GLIM/EKF・融合系、給電用publisher、serial_writerは停止した。
CP2102Nシリアルポートが解放されていることを確認。
GNSS・NTRIP caster/relay・Foxglove Bridgeは確認用に稼働を継続している。
GUI接続先はws://10.42.0.233:8765。GNSS Map Telemetry 0.4.1で補正済みGNSS方位を表示できる。

## 検証・保存

- um982_driver・robotビルド成功。
- UM982 gtest9件（反転補正のテスト含む）合格。
- URDF/方位補正整合、GLIM sensor pose契約、起動構成のpytest計88件合格。
- Python構文、XML値、diffチェック合格。

保存先: /home/ibo/rosbags/gnss_antenna_swap_20261116_0013/
- receiver.raw: 継続中の生ログ。
- gnss_only_evaluation.raw、gnss_only_evaluation.summary.json: GNSS単体の固定スナップショットと統計。
- ros_observation/: 180秒間のROS時系列・統計。
- rtk_fix_precision.png: GNSS単体評価図。
- fusion_eval_bag/、fusion_summary.json、fusion_observations.json: 融合試験の記録と集計。
- fusion_frame_check.json、tf_authority.txt: 実行時取付け位置とTF発行元の確認。
- analyze_rtk_fix.py、analyze_fusion_bag.py、plot_rtk_fix.py、fusion_power.py: 使用スクリプト。

位置情報を含むログ・図・時系列はGitに追加していない。
既存のthruster設定とntripcasterサブモジュールの変更は今回のcommitに含めない。
