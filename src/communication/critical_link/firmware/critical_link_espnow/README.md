# critical_link_espnow

ESP32をUSB serialとESP-NOWの間のcritical-link gatewayとして動作させる
Arduino sketchです。Arduino-ESP32 3.xとESP32-C3を基準にしています。

1. `critical_link_config.example.h`を`critical_link_config.h`へコピーする。
2. Ground側は`CRITICAL_LINK_ROLE_GROUND=1`、船側は`0`にする。
3. `CRITICAL_LINK_PEER_MAC`へ相手側ESP32のSTA MACを設定する。
4. 両方に同じchannel、PMK、LMKを設定し、example keyを必ず交換する。
5. sketchを書き込み、起動logのrole、MAC、channelを確認する。

Ground側はserialから受けた最新frameをESP-NOW unicastで送り、application ACKが
来なければ設定回数まで再送します。同一streamの新しいframeが来た場合、古いframeの
再送を中止して新しいframeを優先します。船側はCRC検証後にserialへ出力してからACKを
返します。

ESP-NOWと通常Wi-Fiを同じESP32で共有するとchannelがAP側に拘束されるため、
critical link専用ESP32を使用してください。
