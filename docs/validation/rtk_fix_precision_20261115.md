# RTK FIX成立と静止ばらつき・Foxglove地図方位

## 実施条件と収束

2026-11-15 EST夜、AGXのローカルNTRIPキャスターを使用。
ユーザーは船体・アンテナが静止していることを確認した。
別途GLIM試験が行われており、そのプロセス・設定・TFには変更を加えていない。
推進・Nav2・ミッションは起動していない。
main=(-0.3,+0.3)、sub=(+0.7,-0.3) mを仮採用。基線角はatan2(-0.6,1.0)。

- 23:20:15 EST付近: geortk.jp:2101/tamtamを再開。約5分待ってもFloat。
- 23:25:20 EST付近: ntrip1.bizstation.jp:2101/3C66B072へ切り替え。
  ドライバは継続起動し、ローカルキャスターの再起動に追従して再接続した。
- **最初のGGA quality=4: 2026-11-16 04:32:23.35 UTC（23:32:23.35 EST）。**
  比較局への切り替えから約7分、今回の最初の起動から約12分でFIXを確認。
- ユーザーから上面の養生テープを剥がしたとの報告を受けた。
  作業そのものの正確な時刻は不明。除去完了報告後のデータを評価に使用した。
  時間経過による収束と除去の効果は分離できず、テープが原因とは断定しない。
- 除去報告後の監視で180秒以上の連続FIXを確認した。

## 最後の180秒の実測値

固定スナップショット `rtk_fix_evaluation.raw` の最長連続FIX区間から最後の180秒を抽出。
評価区間は04:33:12.20〜04:36:12.20 UTC、GGA 3601件、すべてquality=4。
約20 Hz。WGS84の局所接平面へ変換し、サンプル平均からの偏差を集計。
方位は有効フラグAのTHSをアンテナ取付け角で補正し、円平均周りの偏差を計算。
船体原点はサンプルごとの方位でmainの水平レバーアームを回転し、アンテナ位置から引いた。

| 指標 | mainアンテナ位置 | 船体原点換算 |
| --- | ---: | ---: |
| 東西標準偏差 | 2.529 mm | 2.452 mm |
| 南北標準偏差 | 3.973 mm | 4.111 mm |
| 平均位置周りの水平RMS | 4.709 mm | 4.786 mm |
| 平均位置からの距離の95パーセンタイル | 8.514 mm | 8.251 mm |
| 平均位置からの最大距離 | 13.543 mm | 12.716 mm |

- GGAの平均海面高度の標準偏差: 6.476 mm。
- 補正後の平均船首方位（北0度・時計回り）: 249.32967度。
- 方位標準偏差: 0.16556度。絶対偏差の95パーセンタイル: 0.30858度。
- 受信機GSTの推定標準偏差の中央値: 東西0.012 m、南北0.014 m、高さ0.035 m。
  これは上記のサンプルから求めた実測ばらつきとは別の値。
- スナップショット全体にはチェックサム不一致4行がある。正常なNMEAのみを解析した。

**これは180秒間の静止再現性であり、真の位置・方位に対する絶対精度ではない。**
測量済み真値がなく、基準局座標の誤差・アンテナ取付け誤差・長時間変動は評価できていない。
GNSS出力は時間相関があるため、3601件を独立な試行として精度保証に使わない。

## 追加で得た2局の情報

Droggerの公開ソーステーブルから候補を探し、各20秒間接続した。
CRC24Q正常な1005基準局座標と観測データを両局から受信できた。
距離は現在位置の概略値からの直線距離で、1005内のECEF座標をWGS84緯度経度へ変換した値に基づく。
座標の国家座標との整合・絶対精度は検証していない。

| 接続先 | マウントポイント | 1005からの緯度・経度 | 楕円体高 | 距離 | 配信確認 |
| --- | --- | --- | ---: | ---: | --- |
| ntrip1.bizstation.jp:2101 | 3C66B072 | 35.75215522, 139.70054634 | 69.817 m | 18.83 km | 1005/1075/1085/1095/1115/1125/1230、今回FIX成立 |
| ntrip1.bizstation.jp:2101 | 40F1B18A | 35.51190582, 139.48521149 | 112.970 m | 18.46 km | 1005/1075/1095/1115/1125、受信機でのFIX試験は未実施 |

元のtamtamはgeortk.jp:2101、公開座標で約8.9 km。今回は約5分間ではFIX未成立。
同時に複数局の補正を混ぜず、受信機へ渡す局を一つずつ選択する。

提供元の利用案内（移動局クライアントの制限・認証）:
https://drogger.hatenadiary.jp/entry/ntripCaster
国家座標についての注意:
https://www.bizstation.jp/ja/drogger/man/use_drg_ntrip_caster.html
RTCM 1005のフィールド位置の照合先:
https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtcm3.c

## Foxglove

ユーザーの依頼でFoxglove Bridgeを起動し、その後Bridgeだけ再起動してAGXのLAN IP `10.42.0.233:8765` にbindした。
接続先は **`ws://10.42.0.233:8765`**。GNSS・中継・GLIMは再起動していない。

```bash
source install/setup.bash
export CYCLONEDDS_URI=file:///home/ibo/njord2026_ws/src/bringup/robot/config/cyclonedds_agx.xml
ros2 launch robot foxglove_bridge.launch.py foxglove_address:=10.42.0.233 foxglove_port:=8765
```

Bridge 3.6.0でSDK WebSocketのhandshake、位置/方位のchannel広告、両topicの購読とbinary payload受信を確認。
実際のFoxglove GUI画面での描画確認はユーザー側で行う。

拡張GNSS Map Telemetry 0.4.1を作成し、地図の矢印は補正済み絶対ENU方位
`/sensor/vehicle_gnss/compass/raw` を優先するよう変更した。
GNSS方位を一度受信した後は、2秒超の欠測時にローカルTFへ置換せず矢印を非表示。
GNSS方位を一度も受信していない既存構成ではTFの挙動を維持する。
地図上の緯度経度はmainアンテナの生位置で、上記船体原点換算値とは異なる。

7件の方位選択/北東変換/欠測/無効quaternion検証に合格（GJSで実行）。
拡張のパッケージ作成、ZIP整合性検査、git diff --checkに合格。

## 保存と稼働状態

/home/ibo/rosbags/gnss_heading_check_20261115/ 配下:
- rtk_fix_wait.raw: 継続中の受信ログ
- rtk_fix_evaluation.raw: 今回の固定スナップショット
- rtk_fix_evaluation.summary.json: 統計
- rtk_fix_precision.png: 評価図
- rtk_fix_evaluation_marker.json: テープ除去報告後の評価開始時刻
- alternative_stations.json: 2局の配信件数と1005フレーム・座標
- rtk_fix_analysis/: 再集計と図作成、WebSocket確認用スクリプト

位置情報を含むログ・図はGitへ追加していない。
ユーザーがGUI上で確認するため、GNSSドライバ・比較局の中継/ローカルキャスター・Bridgeを稼働させたままにしている。
常駐サービス登録はしていない。GLIM試験は変更していない。
