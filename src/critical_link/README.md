# critical_link

Ground PCからminiPCへ`/joy`と`/heartbeat/ground_station`を冗長配送するROS 2
パッケージです。USB Wi-Fi、内蔵Wi-Fi、VPN内InternetはIPv4 unicast UDP、
ESP-NOWは専用ESP32を介したUSB serialとして扱います。

## Semantics

senderはROS messageをESP-NOW v1でも収まる最大234 byteの共通frameへ変換し、設定済みの
全経路へ同報します。receiverはCRCとpayloadを検証してから、stream別sequenceが現在値
より新しい場合だけcanonical topicへpublishします。同一sequenceの複製と、後着した古い
sequenceは破棄します。sender再起動はsession IDで識別し、退役済みsessionへ戻りません。

Joy payloadは最大32 axesと64 buttonsです。NaN/Inf、壊れたbutton、過大payloadは拒否
します。heartbeatは空payloadのみ受理し、link probeからGround heartbeatを代理生成する
ことはありません。

## Configuration

UDP sender path:

```text
name|ground_bind_ipv4_or_hostname|vessel_destination_ipv4_or_hostname|port
```

UDP receiver path:

```text
name|vessel_bind_ipv4_or_hostname|port
```

NIC名やdefault routeに依存せず、各socketを経路固有の送信元/受信先IPv4へbindします。Avahi
（mDNS）が使える環境では`.local` hostnameも指定できます。
船載スマートフォンはminiPCへUSB tetherし、WireGuard/Tailscale等のVPN内IPv4を4本目の
pathに指定してください。public InternetへUDP portやZenoh 7447を直接公開しません。

[ground_sender.yaml](config/ground_sender.yaml)と
[vessel_receiver.yaml](config/vessel_receiver.yaml)へ実機アドレスと安定した
`/dev/serial/by-id/...`を設定します。

## ROS integration

Ground側ではcanonical topicを直接ネットワークへ出さず、publisherを次へremapします。

```text
joy_node:                         /joy -> /critical_link/input/joy
ground_station_heartbeat_node:   /heartbeat/ground_station
                                  -> /critical_link/input/heartbeat
```

miniPCではreceiverだけが`/joy`と`/heartbeat/ground_station`をpublishします。
既存の`joy_converter`、0.5秒command freshness、規定の60秒Ground heartbeat timeoutは
変更しません。

```bash
ros2 launch critical_link ground_sender.launch.py
ros2 launch critical_link vessel_receiver.launch.py
```

実機へ切り替える前に`test_bringup`のloopback launchで確認できます。

## ESP-NOW

[firmware/critical_link_espnow](firmware/critical_link_espnow/README.md)にGround/Vessel共通の
Arduino sketchがあります。Ground側は同じstreamの新しいframeが来ると古い再送を中止し、
船側はCRC検証とserial出力後にapplication ACKを返します。PMK/LMKを設定したencrypted
unicastのみを使用します。
