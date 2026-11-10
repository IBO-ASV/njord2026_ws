# Ground critical-link redundancy plan

## Scope

Ground PCからminiPCへ送る次の2 topicを、通常の可視化・sensor通信から独立して
冗長化する。

- `/joy`
- `/heartbeat/ground_station`

Jetson Jazzy、Ground PC Jazzy、miniPC HumbleのDDS domainは直接混在させず、
既存の`zenoh-bridge-ros2dds`を役割別に使用する。

- `config/zenoh/bridge_groundpc.json5`: Ground PCのDDS domainとZenohの境界
- `config/zenoh/bridge_jetson.json5`: JetsonのDDS domainとZenohの境界
- `config/zenoh/bridge_minipc.json5`: miniPCのDDS domainとZenohの境界

`zenoh-bridge-ros2dds` 1.9.0では、process環境に`ROS_DOMAIN_ID`が設定されていると
JSON5中の`plugins/ros2dds/domain`より環境変数が優先される。bridge起動時は必ず
`ROS_DOMAIN_ID`をunsetし、JSON5のdomainを唯一の設定源にする。これによりminiPC
bridgeは5、Ground PCとJetson bridgeは6になる。起動logの`ROS2 plugin Config`で
実効domainを確認する。bridgeをsystemdから起動する場合も、unitに
`Environment=ROS_DOMAIN_ID=...`を入れず、`ExecStart=/usr/bin/env -u ROS_DOMAIN_ID ...`
とする。

映像は従来どおりRTP/UDPを使用し、critical linkには流さない。

## Paths

critical linkは次の4経路を扱える構造にする。

1. USB Wi-Fi上のunicast UDP
2. 内蔵Wi-Fi上のunicast UDP
3. USB接続ESP32間のESP-NOW
4. スマートフォン回線上のVPN内unicast UDP

Internet経路ではminiPCやZenohのportをpublic Internetへ直接公開しない。
Ground PCと船載スマートフォン側gatewayをWireGuard等のVPNへ参加させ、
VPN内アドレスを第4経路として扱う。携帯回線はNAT、遅延変動、再接続があるため
主回線とはせず、同じsequence規則に従う追加経路とする。

## Sender and receiver

Ground PCの`joy_node`とground-station heartbeat publisherは、canonical topicではなく
critical-link入力topicへpublishする。senderは共通frameへ変換し、利用可能な全経路へ
同報する。

miniPCのreceiverだけがcanonical `/joy`と`/heartbeat/ground_station`をpublishする。
Ground/miniPC/JetsonのZenoh bridgeのallow listには、この2 topicを入れない。
これによりDDS、Zenoh、複数無線から同じtopicが直接流入する構成を避ける。

frameには少なくともprotocol version、sender session ID、stream ID、sequence、
source monotonic timestamp、payload length、CRCを持たせる。sequenceはstreamごとに
単調増加させ、sender再起動時はsession IDを変更する。

receiverの採用規則は「最初に届いた経路」ではなく、次のとおりとする。

1. 現在採用済みのsequenceより新しい正常frameを採用する。
2. 同じsequenceの複製は、最初に検証を通過した1個だけを採用する。
3. 採用済みsequenceより古いframeは、後から到着しても破棄する。
4. session ID変更時は再起動として扱い、古いsessionのframeを再採用しない。

真の最新frameが将来到着するかは待たず、到着時点で既知の最新sequenceを即座に
publishする。これにより経路優先待ちによる操縦遅延を作らない。

## Heartbeats and safety

経路監視用probe/ACKと`/heartbeat/ground_station`は別物として扱う。通信経路が
生きていてもGround heartbeat sourceが停止していれば、receiverはheartbeatを
代理生成しない。

`command_arbiter`のcommand freshness timeoutは現在の0.5秒を維持し、古いJoyで
推進を継続しない。大会規定に基づき、`thruster_serial`の
`ground_station_heartbeat_timeout_sec`は60.0秒のまま維持する。

## Rollout

ROS 2 sender/receiver、共通frame、UDP/serial transport、ESP-NOW firmware、loopback
bringupまでは`critical_link`パッケージとして実装済みである。以下は実機IP、USB device、
ESP MAC/keyを設定して投入する順序を示す。

1. 共通frame、sequence比較、重複排除をloopback unit testで実装する。
2. USB Wi-Fiと内蔵Wi-Fiの2経路を、送信元IPへ明示的にbindして試験する。
3. ESP-NOW gatewayを追加し、ACK、再送、CRC、古いJoyの破棄を試験する。
4. VPN内のInternet経路を追加し、NAT再接続、長遅延、逆順到着を試験する。
5. 各単独経路、任意の複数経路断、全経路断、sender/receiver再起動を試験する。
6. receiver以外にcanonical topicのpublisherが存在しないことを確認する。

合格条件は、いずれか1経路で最新JoyとGround heartbeatを受信できること、古い経路の
復帰でcommandが巻き戻らないこと、全経路断でcommand freshness timeoutにより推力が
ゼロになること、heartbeat timeoutが規定の60秒で作動することである。
