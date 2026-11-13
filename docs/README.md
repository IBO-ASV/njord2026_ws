# ドキュメント

現行の対応環境は **ROS 2 Jazzyのみ**です。旧ディストリビューションとの混在運用手順は廃止しました。

| 用途 | 入口 |
|---|---|
| 環境構築・端末別運用 | [2台構成](operations/two_machine_split.md)、[AGX起動](operations/agx_ground_jetson_startup.md) |
| launch・設定の使い分け | [robot](../src/robot/README.md) |
| 制御・ミッション設計 | [責務と検証要求](architecture/control_mission.md) |
| 統合方針・残課題 | [backlog](architecture/integration_backlog.md) |
| 自己位置推定 | [TF構成](architecture/localization_tf_architecture.md) |
| 検証 | [受入基準](validation/simulation_acceptance.md)、[記録結果](validation/recorded_results.md)、[GLIM](validation/glim_experiments.md)、[Task 2 costmap](validation/task2_costmap.md) |
| 通信仕様 | [スラスターUART](reference/PROTOCOL.md) |
| ファームウェア | [firm1](../firmware/firm1/README.md)、[BMS](../firmware/njord_BMS_v0/README.md)、[SBUS](../firmware/sbus/README.md) |
| スクリプト | [用途別一覧](../scripts/README.md) |

`operations/` は運用手順、`architecture/` は設計、`validation/` は検証条件と結果、
`reference/` は機器資料です。パッケージ固有の仕様は各パッケージREADMEを正本とします。
過去の日付付き検証値や設計案は、現行の既定設定・実機での保証値ではありません。
生成物は `artifacts/`（Git対象外）へ保存してください。
