通常操作手順
1. コントローラーと陸上PCをつなぎ、```ros2 run joy joy_node```
2. jetsonと陸上PCを同じwifiにつないでいることを確認し、jetson側で```ros2 launch manual_bringup manual_control.launch.py```
3. コントローラー左スティックで前後左右、R/L1?でyaw
4. もし回転速度が早すぎたら、```src/control/thruster_controller/config/config.yaml```のmax_thrustを下げる

thruster_id_check手順
1. jetsonで ```ros2 launch manual_bringup thruster_id_test.launch.py```
2. 10secおきに、RR, RL, FR, FLの順で正転するか確認

できなかったときの確認
- [ ] コントローラーをBluetoothでつないでいるか
- [ ] ```echo $RMWIMPLEMENTATION ; echo $ROS_DOMAIN_ID ; echo $ROS_LOCALHOST_ONLY``` で陸上PCとjetson両方同じになっているか。両端のRMWと通信設定を起動プロファイルに合わせる（2台構成はFastRTPS、AGX/GroundはCycloneDDS）
- [ ] 陸上PCで```ros2 run joy joy_node```を実行し、```ros2 topic echo /joy```で値が取れるか
- [ ] jetsonと陸上PCは同じwifiにつながっているか
- [ ] 緊急停止を上げているか、esp32に電源が入っているか、緊急停止スイッチを押して解除したらﾄｩﾙﾙﾝするか
