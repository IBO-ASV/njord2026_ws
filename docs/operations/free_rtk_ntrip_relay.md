# AGXのRTK補正と基準局の選択

`./scripts/operations/run_agx_pair.sh vessel` はUM982とローカルNTRIPキャスターを起動し、既定で `geortk.jp:2101` の稼働中sourcetableから近い固定基準局を選ぶ。地図側EKFには `config/gnss_map_covariance.yaml` の共分散下限が適用される。UM982の有効なGGAを待ってから、座標のあるRTCM 3固定局を距離順に選ぶ。既定の距離上限は50 kmで、局接続やRTCMの受信に失敗すると次の候補を試す。候補がなければ5秒ごとに再確認する。局名と概算距離は `ntrip_relay` のログに出る。運用中は選択した局を維持し、切断時に再選択する。

```bash
# 既定: 現在位置の近くの固定局を自動選択
./scripts/operations/run_agx_pair.sh vessel

# 明示的に固定局を選択
./scripts/operations/run_agx_pair.sh vessel ntrip_upstream_mountpoint:=tamtam

# 距離上限を変更（局の座標が概算の場合あり）
./scripts/operations/run_agx_pair.sh vessel ntrip_max_distance_km:=30.0

# 別キャスターと手動マウントポイント
./scripts/operations/run_agx_pair.sh vessel \
  ntrip_upstream_host:=example.org ntrip_upstream_port:=2101 \
  ntrip_upstream_mountpoint:=MOUNTPOINT
```

## VRS / NEAR

自動選択は固定局のみ。VRS・NEARや認証の必要なサービスは、提供元の接続情報と契約を確認して手動指定する。`ntrip_require_gga:=true` は有効なUM982の生GGAがないと接続せず、GGAが10秒以上届かないか無効になると中継を切断して再試行する。新鮮なGGAは接続要求直後と以後約5秒ごとに上流へ送られる。

```bash
./scripts/operations/run_agx_pair.sh vessel \
  ntrip_upstream_host:=caster.example.org ntrip_upstream_port:=2101 \
  ntrip_upstream_mountpoint:=VRS_MOUNT \
  ntrip_upstream_user:=USERNAME ntrip_upstream_password:=PASSWORD \
  ntrip_send_gga:=true ntrip_require_gga:=true
```

認証情報はコマンド履歴やプロセス引数から見えるため、共有環境では適切に管理する。GGAには位置情報が含まれ、指定した上流に送られる。GGA送信を不要にする場合は `ntrip_send_gga:=false`、RTK自体を無効にする場合は `enable_um982_rtk:=false`。TLSは未対応。サービス側のNTRIP仕様・認証方式によっては利用できない。

## 指定可能な局の一覧

[GEORTK公式一覧](https://geortk.jp/mountpoint)には登録局が掲載されるが、登録されていても停止中の局がある。実際に稼働中とキャスターが報告し、RTCM 3・固定局・座標あり・認証不要という自動選択条件を満たす局は次で表示できる。緯度・経度はsourcetableの概算値。

```bash
python3 src/bringup/robot/scripts/ntrip_relay.py --list-stations
# 別のcasterの固定局一覧
python3 src/bringup/robot/scripts/ntrip_relay.py --list-stations --upstream-host HOST --upstream-port 2101
```

2026-11-21取得時の例: `tamtam`（横浜、35.55, 139.58）、`sugamafuji2`（35.95, 139.49）、`funabashi_sasaki`（35.73, 140.06）、`ONCT_M2`（36.32, 139.84）。その他の局は上のコマンドまたは公式一覧を参照する。手動指定は一覧に座標がない局やVRSも可能だが、配信の可否・認証・座標精度は提供元に確認する。基準局の座標系や運用状態は自動的に保証されない。

## 処理と確認

受信したHTTP通常形式またはchunked形式を判別し、CRC24Q正常なRTCM3フレームのみをローカルcasterへ送る。有効なフレームが15秒届かない場合は再接続する。`/sensor/vehicle_gnss/gga/raw` はUM982が出したチェックサム正常なGGAを、`/sensor/vehicle_gnss/gga_quality` はそのqualityを公開する。quality 1=Single、2=DGPS、5=RTK Float、4=RTK FIX。中継接続、CRC正常な補正、UM982の補正利用、RTK FIXは別々に確認する。

地上PCで従来の構成を使う場合、`ros2 launch robot ground_pc.launch.py` は従来通り固定局 `tamtam` が既定。AGX専用の単体中継は `ros2 launch robot ntrip_corrections.launch.py` で起動でき、同launchにも上記の `ntrip_upstream_*`・`ntrip_max_distance_km`・`ntrip_send_gga`・`ntrip_require_gga` 引数がある。
