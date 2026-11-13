通常操作手順
1. コントローラーと陸上PCをつなぎ、```ros2 run joy joy_node```
2. jetsonと陸上PCを同じwifiにつないでいることを確認し、jetson側で```ros2 launch simple_manual manual_control.launch.py```
3. コントローラー左スティックで前後左右、R/L1?でyaw
4. もし回転速度が早すぎたら、```src/driver/micon/thruster_driver/config/config.yaml```のmax_thrustを下げる

thruster_id_check手順
1. jetsonで ```ros2 launch simple_manual thruster_id_test.launch.py```
2. 10secおきに、RR, RL, FR, FLの順で正転するか確認

できなかったときの確認
- [ ] コントローラーをBluetoothでつないでいるか
- [ ] ```echo $RMWIMPLEMENTATION ; echo $ROS_DOMAIN_ID ; echo $ROS_LOCALHOST_ONLY``` で陸上PCとjetson両方同じになっているか。特にfastrtpsになってないとubuntu22.04,24.04間で通信できないので注意
- [ ] 陸上PCで```ros2 run joy joy_node```を実行し、```ros2 topic echo /joy```で値が取れるか
- [ ] jetsonと陸上PCは同じwifiにつながっているか
- [ ] 緊急停止を上げているか、esp32に電源が入っているか、緊急停止スイッチを押して解除したらﾄｩﾙﾙﾝするか
