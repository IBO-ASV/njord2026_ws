# Foxglove起動と推力指令watchdog（2026-11-13）

AGXの `agx_bringup.launch.py`（Task1/Task2ラッパーを含む）と
`njord_sim/mission_sim.launch.py` は、Foxglove bridgeを標準起動する。
共通定義は `robot/launch/foxglove_bridge.launch.py`。

- 接続先は `ws://<AGX-IP>:8767`。既定listen addressは `0.0.0.0`。
- `foxglove_address:=127.0.0.1` なら従来のSSHトンネル方式。
- 無効化：AGXは `enable_foxglove_bridge:=false`、simは `foxglove:=false`。
- `foxglove_port:=8768` などで変更可能。既存の地上局bridgeの8765とは別。
- 自動試験runnerは明示的にbridgeを無効にし、並列起動時の競合を避ける。
- WebSocketデータサーバーであり、HTTPアクセスだけで表示するWeb GUIではない。

標準インストールは `sudo apt install ros-jazzy-foxglove-bridge`。
このAGXでは最初sudoを使えず、下記fallbackでworkspace内に配置していた。
現在はapt版が導入されたため、旧fallbackを `~/.local/share/njord/` 配下に退避し、
通常の `source /opt/ros/jazzy/setup.bash` と `source install/setup.bash` で
`ros2 pkg prefix foxglove_bridge` が `/opt/ros/jazzy` を返す構成へ移行した。
launchはパッケージ名による解決なので、apt版のための実行パス指定は不要。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/install_foxglove_bridge_local.py
source install/setup.bash
```

apt版が既にある場合、このinstallerは何も配置しない。既存prefixも上書きしない。
通常のシステム依存関係は別途必要で、これはaptパッケージを展開する
runtime用fallbackであり、source buildするためのSDK導入ではない。

## GUIとAUTO条件

Mission Controlに自己位置・GNSS受信・driver heartbeatの状態、age、周波数、監視メッセージを表示。
`/control/requirements` にSafetySupervisorが公開する現在のpolicyと禁止条件ON/OFFも表示する。
古い状態や未登録の信号は、正常として表示しない。
GUI用の現在のpolicyは実行中Supervisorの設定であり、policyファイル変更後はSupervisorの再起動が必要。

ユーザーがテスト用に無効化していたpolicyの既定値は変更していない。
現在の `gnss` healthは更新頻度監視でRTK FIXの品質判定ではなく、`driver_heartbeat` の集約信号も
inventoryに未定義だからである。正しい品質・driver信号を実装してからこれらのgateを有効化する。
既存の自己位置gateをONにしたテスト設定では、GUIにONとSTALE、AUTO阻害理由が揃うことを検証した。

## serial writer

推力指令をsteady clockで監視し、0.5秒の受信途絶・未受信・4要素でない配列・非有限値の場合は
ゼロ推力、software emergency stop、赤のパケットを送信する。
地上局・SBUS・LEDの通信が継続しても推力指令の期限は更新しない。
正常な推力指令で復帰するが、他の非常停止条件は独立して維持する。

状態は `/micon/thruster_command_status`（String）と `/micon/thruster_command_fresh`（Bool）で公開。
statusには独立したsoftware stop、地上局タイムアウト、relay入力、ポート未接続も併記する。
これは指令受信状態であり、物理出力やリレーの実測確認を代用しない。

## 検証

- 擬似TTYで実際のCOBS/CRC付き送信パケットを検証するgtest：8件合格。
- Mission / Control / simのPythonテスト：62件合格（うちControl Manager 17件）。ONにした入力の欠落でAUTOが禁止され、同じ条件がGUI用topicへ出る。
- Mission Controlの状態・policy ON/OFF・age・serial watchdog理由表示：Nodeテスト合格。
- simの既定起動で8767のlisten、WebSocket接続、型付きタスク一覧取得を確認。
- 実機のシリアルデバイスや推進器は使っていない。AGX実機launchはハードウェアを起動せず構成を確認。
