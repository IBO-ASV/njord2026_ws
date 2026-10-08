# test07089 から master への段階的統合計画

## 監査の基準

- 比較範囲: `origin/master` (`27de65a`) .. `origin/test07089`
  (`0144e6e`)、共通祖先は `27de65a`。
- 差分規模: 692 commit。制御、センサ、TF/EKF、Task 1--4、知覚、シミュレーション、
  Zenoh/ネットワーク、firmware資料、vendor/modelが同時に含まれる。
- 方針: test07089を1本のPRとしては扱わない。各sliceはmasterから作り、依存する
  sliceだけを順にrebase/mergeする。masterへの直接push・force push・履歴改変はしない。

## 先に除外・仕分けするもの

| 区分 | 対象例 | 処置 |
| --- | --- | --- |
| ローカル/生成物 | `CMakeFiles/`, `.vscode/`, `.ai/`, `.claude/`, `.codex/` | 原則として統合しない。必要な開発規約だけを別PRで提案する。 |
| vendor/submodule | `ultralytics`, Livox, GNSS/INS submodule | upstream SHA、ライセンス、再取得手順、CI容量を確認して独立PRにする。 |
| モデル/大容量成果物 | YOLOモデル、画像、PDF、bag相当 | Git LFS/配布先・ライセンス・再現方法が決まるまで機能PRから除外する。 |
| 実機固有情報 | 配線、個人端末名、IP、測量データ | 非公開設定または運用記録へ分離し、公開Issue/PR本文へ載せない。 |

## 推奨sliceと依存順

| 順 | slice | 主な範囲 | 前提 | 受入条件 |
| ---: | --- | --- | --- | --- |
| 0 | CI・リポジトリ衛生 | `.github/`, build script、lint、不要生成物の除外 | なし | master向けPRで静的検査と最小ROS buildが再現する。 |
| 1 | 共通インターフェース/基盤 | `njord_platform`, `njord_interfaces` | 0 | 下流packageが同じmessage/service定義でビルド・単体試験できる。 |
| 2 | 安全・手動制御・駆動通信 | `simple_manual`, `control_manager`, `critical_link`, `alert_lamp`, `micon_*`, `thruster_driver` | 1 | watchdog/soft stop/通信異常の単体・疑似通信試験、既存水上profile回帰。 |
| 3 | GNSS・LiDAR・TF/EKF | UM982、Drogger、GLIM設定、`robot` のlocalization/URDF | 1 | `map -> odom -> base_link` の発行者が一意、GNSS/odometryのbagまたは再生試験。 |
| 4 | Task 1最小経路 | Task 1 waypoint、Nav2設定、`task1_sim`、診断 | 2, 3 | 停止状態のroute/TF確認、simulation test、実機は別チェックリストで承認。 |
| 5 | mission/runtime・Task 3/4 | `mission_manager`, waypoint transform、Task 3/4設定 | 2, 3, 4 | mission遷移・失敗時停止・シミュレーションを個別に検証。 |
| 6 | Task 2航法・知覚統合 | task2 planner、buoy/ship tracking、ZED/LiDAR bridge | 3, 5 | replay入力で座標系・検出freshness・fallback routeをテスト。 |
| 7 | 検出器/重い依存 | YOLO、PCL、ZED CUDA/TensorRT、モデル | 0, 1 | CPU最小buildとGPU runnerでの明示的な分離、モデル入手手順・ライセンス確認。 |
| 8 | 運用/可視化/ネットワーク | Zenoh、Foxglove、systemd、ネットワークscript、BMS資料 | 2, 3 | 非機密設定で起動し、二重起動・誤接続・権限をレビュー。 |

## PRの作り方

1. `test07089` のcommitをそのままcherry-pickしない。masterからbranchを作り、sliceの
   必要ファイルだけを移植する。
2. PR本文に「含める機能」「含めない差分」「設定変更」「安全影響」「テスト結果」を記す。
3. firmware/実機設定はソース・通信仕様・host設定・mock testを1組として扱う。ただし
   flash・モータ駆動はPR検証に含めず、別の実機チェックリストで承認する。
4. interface変更は下流更新を同PRへ無制限に混ぜず、互換layerか依存PR順序を明示する。
5. PRはdraftで開始し、CI・review・運用担当の確認を満たしてからreadyにする。

## 現在の3輪オムニslice

`codex/tri-omni-task1-1` は上表のslice 2へ入る候補であり、水上機の型`0x01`を
維持したまま、3輪MD10C型`0x02`を別profileに分離する。この文書はbranch先端commitを
統合基準にしない。master向けsliceでは、レビュー済みの固定commitをPR本文に明示し、
`codex/tri-omni-task1-1` のHEADを暗黙に基準にしない。host/firmwareの出力lockは未校正の
まま既定で有効である。

受入にはROS CI、hostのpseudo-TTY試験、firmware compile、輪別極性確認、外部停止確認が
必要である。極性・配線・物理速度校正が未確認の間は実機出力を有効化しない。

## GitHub Project

Projectのownerは `IBO-ASV`。現在のCLI tokenには `read:project` と `project` がないため、
Projectの読取・作成は保留中である。権限が付与されるまでは、このplanのsliceをMarkdownと
draft PRで追跡する。
