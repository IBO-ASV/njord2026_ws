# Critical-link network test guide

## 目的と対象

Ground PCからminiPCへ、次のcritical topicを冗長配送できることを確認する。

- Joy入力: `/critical_link/input/joy` → `/joy`
- Ground heartbeat: `/critical_link/input/heartbeat` →
  `/heartbeat/ground_station`

対象経路はUSB Wi-Fi、内蔵Wi-Fi、ESP-NOW、スマートフォン回線上のVPNである。
通常の映像、LiDAR、可視化telemetryやZenoh全体の帯域試験は本書の対象外とする。

receiverは到着時点で既知の最新sequenceだけをpublishする。同一frameを複数経路で
受けた場合は最初の正常な1個だけを採用し、同じsequenceの複製や後着した古いframeを
破棄する。このため、経路の固定優先順位や待ち時間は設けない。

## 安全条件

実機で`/joy`をpublishする試験では、最初に推進電源を切る。推進電源が必要な試験は
船体を確実に拘束し、非常停止担当者を置いて最後に実施する。試験用loopbackは
`/critical_link/test/output/*`へ出力するため、推進系へ接続されない。

次も試験開始前に記録する。

- Ground PC、miniPC、Jetsonのホスト名、ROS distro、`ROS_DOMAIN_ID`
- 各NICのinterface名、IPv4、SSIDまたは接続先、UDP port
- ESP32のSTA MAC、ESP-NOW channel、firmware commit、USB serial by-id
- VPN製品、VPN内IPv4、使用したスマートフォン回線
- 試験日時、場所、距離、見通し、天候、電源状態

## 事前確認

両PCで時刻とinterfaceを確認する。critical-linkの判定自体はPC間の時計同期に依存しない。

```bash
date --iso-8601=seconds
ip -br -4 address
ip route
```

Ground PCでは、設定した送信元IPから各宛先へ意図したinterfaceが選ばれることを確認する。

```bash
ip route get <VESSEL_USB_WIFI_IP> from <GROUND_USB_WIFI_IP>
ip route get <VESSEL_BUILTIN_WIFI_IP> from <GROUND_BUILTIN_WIFI_IP>
ip route get <VESSEL_VPN_IP> from <GROUND_VPN_IP>
```

`ground_sender.yaml`の各要素は次の形式である。

```text
name|Ground側bind IPv4|miniPC側宛先IPv4|UDP port
```

`vessel_receiver.yaml`は次の形式で、送信側とportを一致させる。

```text
name|miniPC側bind IPv4|UDP port
```

経路別の空テンプレートは`config/critical_link_test_config/`にある。
`u1_*`がUSB Wi-Fi、`u2_*`が内蔵Wi-Fi、`e1_*`がESP-NOW serial、`v1_*`がスマートフォン
VPNである。IPやserial deviceは空欄のままcommitしてあり、実機試験時だけ各端末の設定を
埋めて`params_file`へ渡す。テンプレートの`udp_paths`は空なので、未設定のまま起動しても
意図しない宛先へ送信しない。

ESP-NOWには両側で同じchannel、PMK、LMKを設定し、相手のSTA MACを指定する。example keyは
実機で使用しない。USB deviceは`/dev/ttyUSB*`ではなく`/dev/serial/by-id/...`を使う。

## ビルドと自動試験

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select critical_link test_bringup --symlink-install
source install/setup.bash
colcon test --packages-select critical_link test_bringup
colcon test-result --test-result-base build/critical_link --verbose
colcon test-result --test-result-base build/test_bringup --verbose
```

`critical_link`のprotocol、CRC、serial stream再同期、sequence wrap、session切替、Joy codec、
UDP設定parserが全件合格することを確認する。

## 1台でのUDP loopback

端末Aで安全なloopbackを起動する。

```bash
source install/setup.bash
ros2 launch test_bringup critical_link_loopback.launch.py
```

端末BからJoyとheartbeatを入力する。

```bash
ros2 topic pub --once /critical_link/input/joy sensor_msgs/msg/Joy \
  "{axes: [0.25, -0.5], buttons: [0, 1]}"
ros2 topic pub --once /critical_link/input/heartbeat std_msgs/msg/Empty "{}"
```

端末Cで出力を確認する。

```bash
ros2 topic echo --once /critical_link/test/output/joy
ros2 topic echo --once /critical_link/test/output/heartbeat
ros2 topic echo /diagnostics
```

axes、buttonsが一致し、Joyの`frame_id`が`critical_link`になること、receiver aggregateの
`accepted_joy`と`accepted_heartbeat`が増えることを合格条件とする。

## 2台での実経路試験

実機設定をsource treeへ直接書く必要はない。試験ごとに設定をコピーし、絶対pathを
`params_file`へ渡す。まずminiPCのreceiver、次にGround PCのsenderを起動する。

```bash
# miniPC
ros2 launch critical_link vessel_receiver.launch.py \
  params_file:=/absolute/path/to/vessel_receiver_test.yaml

# Ground PC
ros2 launch critical_link ground_sender.launch.py \
  params_file:=/absolute/path/to/ground_sender_test.yaml
```

最初は設定の`udp_paths`を1本だけにして、次の順に単独試験する。

| 試験 | 有効にする経路 | 主な確認事項 |
|---|---|---|
| U1 | USB Wi-Fi | bind先、受信数、Joy連続性、抜線時の停止 |
| U2 | 内蔵Wi-Fi | U1と別subnetであること、受信数、再接続 |
| E1 | ESP-NOW serialのみ | encrypted unicast、ACK再送、USB再接続 |
| V1 | スマートフォンVPN | VPN内IPのみ使用、NAT再接続、遅延変動 |

各UDP経路では両端でpacketを観測できる。`-i any`だけで済ませず、経路固有interfaceでも
取得して、想定外のinterfaceへ流れていないことを確認する。

```bash
sudo tcpdump -ni <INTERFACE> udp port <PORT>
ss -lunp
ros2 topic hz /joy
ros2 topic hz /heartbeat/ground_station
ros2 topic echo /diagnostics
```

UDP sender側の`send()`成功は相手への到達を保証しない。合否はminiPC receiverの
path別`received`、canonical topic、packet captureで判定する。

## 冗長化、逆順、復旧試験

単独試験後に全経路を有効にし、実ジョイスティックを一定周期で動かす。同一frameが
複数経路から届くため、canonical `/joy`は経路数倍に増えず、diagnosticsの
`duplicate_or_old`が増えることが正常である。

専用の試験interfaceにだけ遅延と逆順を加える場合は、既存qdiscがないことを先に確認する。
次は例であり、interface名は実機に合わせる。

```bash
tc qdisc show dev <TEST_INTERFACE>
sudo tc qdisc add dev <TEST_INTERFACE> root netem delay 500ms 100ms \
  distribution normal reorder 25% 50%
```

正常経路を残した状態で、遅延経路から古いframeが後着してもJoyが過去の値へ戻らず、
`duplicate_or_old`へ計上されることを確認する。試験で追加したqdiscだけを終了時に外す。

```bash
sudo tc qdisc del dev <TEST_INTERFACE> root
```

続いて、USB Wi-Fi、内蔵Wi-Fi、ESP-NOW、VPNを1本ずつ抜線または切断する。各操作後も
最低1経路を残し、Joyとheartbeatが継続することを確認する。切断した経路を復旧しても
古いframeが採用されないこと、senderまたはreceiver再起動後に新sessionで復帰することも
確認する。最後に全経路を切断し、Joyが止まってcommand freshness timeoutにより推力指令が
ゼロになることを、推進電源OFFで確認する。

## ESP-NOW固有試験

Ground/Vessel両方の起動logでrole、STA MAC、channelを確認する。両ESP32を通常Wi-Fiの
stationとして併用するとAPにchannelを拘束されるため、critical-link専用ESP32を用いる。

次を順に確認する。

1. ESP-NOW以外の`udp_paths`を空にしてJoyとheartbeatが通る。
2. 一時的な遮蔽や距離増加でpacket lossを発生させ、application ACKによる再送で復帰する。
3. 連続Joy中は古いframeの再送より同streamの新しいframeが優先される。
4. Ground側USB、Vessel側USBを個別に抜き差しし、by-id pathで自動再openする。
5. channel、LMK、peer MACのいずれかを意図的に不一致にすると受信しない。

最後のnegative test後は正しい暗号設定へ戻し、設定値そのものを試験記録やpacket captureへ
残さない。

## スマートフォン／Internet固有試験

public InternetへUDP portやZenoh 7447を公開せず、WireGuard、Tailscale等のVPN内IPv4を
使用する。まずWi-Fi環境でVPN疎通を確認し、次にモバイル回線へ切り替える。

- USB tetherの抜き差し、機内モードON/OFF、圏外からの復帰を試す。
- VPN addressとrouteが復旧し、receiverのVPN path受信数が再び増えることを確認する。
- 高遅延のVPN frameがUSB/内蔵Wi-Fiの新しいJoyを巻き戻さないことを確認する。
- tether端末の充電、発熱、データ容量制限、OSの省電力設定を確認する。

VPNは追加の第4経路であり、携帯回線だけを通常時の唯一の操縦経路にしない。

## ROS、Zenoh、heartbeatの確認

Ground側のJoyとheartbeat publisherはcanonical topicではなくcritical-link入力topicへ
remapする。miniPCでは`critical_link_receiver`だけがcanonical topicをpublishする。

```bash
ros2 topic info --verbose /joy
ros2 topic info --verbose /heartbeat/ground_station
```

miniPCでpublisherがそれぞれ1つであることを確認する。Ground側でcanonical publisherを
残すとZenoh経由との二重化が起き得るため不合格とする。bridge起動logではGround PCと
Jetsonがdomain 6、miniPCがdomain 5になっていることも確認する。

経路probeとGround heartbeatは別物である。heartbeat sourceだけを停止し、通信経路が
生きたままでもreceiverがheartbeatを代理生成しないことを確認する。大会規定の最終試験は
推進電源OFFで行い、最後のGround heartbeatから60秒で
`ground_station_heartbeat_timeout_sec: 60.0`の安全動作へ入ることを確認する。

## 合格基準と記録

次をすべて満たした場合に合格とする。

- 4経路がそれぞれ単独でJoyとGround heartbeatを配送できる。
- 任意の複数経路断でも、最低1経路が生きていれば最新Joyが継続する。
- 重複、CRC不正、古いsequence、退役済みsessionをcanonical topicへpublishしない。
- 遅い経路の復帰でcommandが巻き戻らない。
- link probeからGround heartbeatを生成しない。
- 全経路断でcommand freshnessが作動し、Ground heartbeat停止後60秒の規定動作を保つ。
- canonical `/joy`と`/heartbeat/ground_station`のpublisherがminiPC receiverだけである。
- Internet経路がVPN内に閉じ、public portを公開していない。

試験結果には使用config、Git commit、成功／失敗時刻、diagnostics、topic rate、packet loss、
遅延条件、切断・復旧時間を残す。PMK、LMKやVPN秘密鍵は記録へ含めない。
