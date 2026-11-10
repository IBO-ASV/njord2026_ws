手順

1. Jetsonで`ros2 launch robot jetson_bringup.launch.py`、miniPCで
   `ros2 launch robot minipc_bringup.launch.py`を各1回起動する。
2. 陸上PCで`ros2 launch robot ground_pc.launch.py enable_foxglove_bridge:=true`を
   実行する。ゲームパッド、ground-station heartbeat、前方JPEG-RTP受信（UDP 5600）、
   後方H.264/H.265-RTP受信（UDP 5601）とFoxglove bridgeが起動する。
3. Foxgloveを開き、layoutを読み込んでbridgeのWebSocketへ接続する。bridgeが不要な
   運用では`enable_foxglove_bridge:=false`（既定値）のままにする。

タスクの開始・停止・状態確認はFoxgloveのMission操作または同じ typed API を使う
`njord-task` CLIから行う。例:

```bash
njord-task check task1
njord-task start task1 --auto
njord-task status
njord-task stop
```

起動直後はMANUAL / IDLEで、自動目標は送信されない。
