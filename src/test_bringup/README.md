# test_bringup

Njordの実機試験とオフライン再生にだけ使用するbringupを、運用用の
`robot`パッケージから分離したパッケージです。通常の実機bringupから
これらのlaunchをincludeしないでください。

critical-linkのloopback、4経路単独、冗長化、断線、ESP-NOW、スマートフォンVPN、
heartbeat試験の手順は[about_networktest.md](about_networktest.md)を参照してください。

## Launch files

| Launch | Purpose |
|---|---|
| `critical_link_loopback.launch.py` | critical-linkのUDP sender/receiverを安全なtest topicでloopbackする |
| `nav2_mintest.launch.py` | Nav2を最小構成で起動する。`ROS_DISTRO`に応じてHumble/Jazzy設定を選ぶ |
| `soft_emg_alert_lamp_test.launch.py` | joy、soft-EMG、Micon、alert lampの実機試験 |
| `um982_ekf_bag_playback.launch.py` | UM982とglobal EKFをrosbagの時刻で再生検証する |

Examples:

```bash
ros2 launch test_bringup nav2_mintest.launch.py
ros2 launch test_bringup soft_emg_alert_lamp_test.launch.py serial_port:=/dev/ttyUSB0
ros2 launch test_bringup um982_ekf_bag_playback.launch.py
ros2 launch test_bringup critical_link_loopback.launch.py
```

soft-EMG試験では推進電源を切るか、船体を確実に拘束してください。

critical-link loopbackでは、次のように入力と出力を確認できます。

```bash
ros2 topic pub -r 10 /critical_link/input/joy sensor_msgs/msg/Joy \
  "{axes: [0.1, -0.2], buttons: [0, 1]}"
ros2 topic echo /critical_link/test/output/joy
```

パッケージ単体の確認:

```bash
colcon build --packages-up-to test_bringup
colcon test --packages-select test_bringup
colcon test-result --verbose
```
