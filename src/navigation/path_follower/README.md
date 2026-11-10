nav2のpure pursuitを使います。渋かったらMPCとかに変えます

$ tree -d
.
├── bag_files
└── src
    ├── bt
    │   └── tree
    ├── detection
    │   ├── pcl_det // poiintcloudでperception系ができれば
    │   ├── scan_matching
    │   │   ├── icp_base
    │   │   └── ndt_base
    │   └── yolo
    ├── driver
    │   ├── camera
    │   │   ├── README.md
    │   │   └── zed-ros2-wrapper
    │   ├── gnss
    │   │   ├── drogger_bt
    │   │   ├── drogger_rws // septentorio
    │   │   ├── drogger_rzs // ublox　と違うので分けたほうがいいのかなと思って分けてるがbtでまとめれる気がしている。
    │   │   └── um982_driver
    │   ├── imu
    │   │   ├── sonota
    │   │   └── wit_node_ros2
    │   │       ├── doc
    │   │       ├── wit_driver
    │   │       ├── wit_msgs
    │   │       └── wit_node
    │   └── micon
    │       ├── bms //battery management sys。マイコンから送られる電圧情報を受け取ってdiagnoseとかで出すだけ。
    │       ├── thruster_driver // cmd_vel から4スラスタ推力を生成
    │       └── micon_driver_fd //file discriptorのfd。micon通信部分。std_msgs IntMultiArrayとかを受け付け、適当に変換してマイコンに送るdaemon的な
    ├── path_follower //勉強のため自前で書いていきたいが、一旦nav2で収まり切らないカスタムaction nodeだけ入れる
    ├── path_generator//勉強のため自前で書いていきたいが、一旦nav2で収まり切らないカスタムaction nodeだけ入れ
    └── robot
        ├── config
        │   └── yolo_model
        ├── launch
        ├── meshes
        └── urdf
