# master へ段階的に取り込むブランチ案

`origin/master` から作成する統合用ブランチ名の候補です。大きな機能単位で切り、依存する土台を先に取り込みます。各ブランチでは現ブランチ全体を merge せず、対象機能のコミットを cherry-pick する想定です。

```text
integrate/
├── platform/
│   ├── agx-bringup
│   ├── ground-station-links
│   ├── dds-zenoh-network
│   ├── agx-rosbag-service-ui
│   └── runtime-parameter-sync
├── localization/
│   ├── um982-driver-config
│   ├── glim-livox-imu
│   └── ekf-tf-heading
├── control/
│   ├── manual-sbus-arbitration
│   ├── soft-emergency-stop
│   ├── critical-link
│   └── thruster-live-tuning
├── perception/
│   ├── task1-onboard-detection
│   ├── task2-vessel-tracking
│   └── foxglove-visualization
├── missions/
│   ├── task1-waypoints
│   ├── task2-opponent-selection
│   └── task4-dolly-missions
├── simulation/
│   ├── task1-sensor-parity
│   ├── task2-kinematics
│   └── task3-virtual-walls
└── tooling/
    ├── sensor-bringup
    ├── experiment-validation
    └── environment-lock
```

## 取り込み順の目安

1. `platform/dds-zenoh-network` と `platform/ground-station-links`
2. `platform/agx-bringup`、`platform/runtime-parameter-sync`、`platform/agx-rosbag-service-ui`
3. `localization/*`
4. `control/*`
5. `perception/*` と `missions/*`（特に Task 2 は localization と perception の依存を確認）
6. `simulation/*` と `tooling/*`

ブランチ名は候補です。実際の分割では、共通ファイルの重複変更やコミット間依存を確認し、複数項目にまたがる変更は依存先のブランチに寄せてください。Foxglove の可視化・実験記録・環境ロックは個別機能の動作に必須か確認してから取り込みます。
