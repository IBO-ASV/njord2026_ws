手順

1. Jetsonで`ros2 launch robot jetson_bringup.launch.py`、miniPCで
   `ros2 launch robot minipc_bringup.launch.py`を各1回起動する。
2. 陸上PCで`ros2 launch robot ground_pc.launch.py`を
   実行する。ゲームパッド、ground-station heartbeat、前方JPEG-RTP受信（UDP 5600）、
   後方H.264/H.265-RTP受信（UDP 5601）が起動する。
3. Foxgloveを使う場合は別途bridgeを起動し、layoutを読み込んでそのWebSocketへ接続する。

タスクの開始・停止・状態確認はFoxgloveのMission操作または同じ typed API を使う
`njord-task` CLIから行う。例:

```bash
ros2 run mission_manager njord-task check task1
ros2 run mission_manager njord-task start task1 --auto
ros2 run mission_manager njord-task status
ros2 run mission_manager njord-task stop
```

起動直後はMANUAL / IDLEで、自動目標は送信されない。
