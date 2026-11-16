# 無料局→AGX内キャスター→UM982の実機試験

2026-11-15 23:09〜23:11 EST付近。AGX上で次の経路を実行した。

```text
geortk.jp:2101/tamtam
  → robot/ntrip_relay.py（CRC検証）
  → AGX内ntripcaster:2101/RTCM3
  → um982_driver（NTRIP接続先127.0.0.1）
  → USBシリアル→UM982
```

地上PCは使用しない。スラスター・Nav2・ミッションは起動しない。
アンテナ座標main=(-0.3,0.3)、sub=(0.7,-0.3) mの前回設定を利用。

## 結果

- ローカルキャスターへのSOURCE/CLIENT接続が成立し、両方に同じ転送バイト数を確認。
- CRC正常なRTCM 1005、1077、1087、1097、1117、1127を中継。
- 受信機GGA qualityはSingle(1)→DGPS(2)→RTK Float(5)に変化。
- 生ログ上、最初のSingleは04:09:08.00 UTC、DGPSは04:09:10.60、Floatは04:09:44.15。
- 120秒のROS観測: 位置2387件、方位2387件。約20 Hz。
- NavSatFix.statusは0が31件、2が2356件。標準statusではDGPS/RTK Float/FIXを区別できない。
- **この観測ではGGA quality=4（RTK FIX）は確認できなかった。** 補正利用とFloat成立までを合格とする。
- 補正投入中の遷移を含むため、この区間の位置分散は前回Single静止精度との比較には使わない。
- 観測後にUM982ドライバとキャスター/中継を停止。常駐登録は行っていない。

補正情報が使われることを確認できたが、FIXに至らない原因やアンテナ条件は未切り分け。
基準局の絶対座標精度・国家座標との整合も本試験では未確認。

## ソフトウェア検証

- robotビルド成功。
- launch引数表示成功（ground_pc、ntrip_corrections）。
- 起動構成テストと中継プロトコルテスト: 合計92件合格。
- CTest経由でも中継テスト6件合格。
- Python構文検査、git diff --check合格。

上流切断時は5秒後に再接続する実装。今回の実機試験で切断を強制する検証は行っていない。
地上PC上の全bringupは停止中のため実行しておらず、共通launchをAGXで実行して補正経路を検証した。

## 保存先

- /home/ibo/rosbags/gnss_heading_check_20261115/rtk_local_caster.raw
- /home/ibo/rosbags/gnss_heading_check_20261115/rtk_local_caster/samples.json
- /home/ibo/rosbags/gnss_heading_check_20261115/rtk_local_caster/summary.json

位置情報を含むログはGitに追加していない。
起動方法は docs/operations/free_rtk_ntrip_relay.md を参照。
既存のthruster設定変更とntripcasterサブモジュールの更新は本変更へ含めない。
