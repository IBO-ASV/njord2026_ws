# 設計レビュー2（修正版・最終）— Sol代役=fable-escalation

- 日時: 2026-07-12
- 対象: 案C修正版（config を robot 側へ / lidar.launch.py 新設 / real_bringup 差替）
- 結論: **合意**（High 級欠陥なし）。下記 Med 3点を実装計画に反映することを条件に確定。

## 残リスク3点への回答（レビュアー検証）
1. `user_config_path`: FindPackageShare('robot')+PathJoin は installed share の絶対パス→ node は fopen するだけで cwd 非依存。**設計どおりで可**。
2. `exec_depend`: robot/package.xml に livox_ros_driver2 依存が現状無し（既存負債）。**追加すべき(Med)**。
3. real_bringup 差替: msg_MID360_launch.py の include は real_bringup 1箇所のみ。他ノードと引数非共有。**副作用なし**。

## 追加指摘（全採用）
- [Med-A] `DeclareLaunchArgument("lidar_model", choices=["mid360","mid360s"])` で fail-fast。typo は無トピックのサイレント失敗になるため。→ **採用**。
- [Med-B] `<exec_depend>livox_ros_driver2</exec_depend>` を robot/package.xml に追加。→ **採用**（司令塔も package.xml 未依存を独立確認済）。
- [Med-C] robot に config 検証 pytest 追加: 両config (a)JSON parse (b)MID360S は "Mid360s" 有 "MID360" 無 (c)lidar_type==8 (d)lidar_configs/host_net_info 必須field。dry-run は JSON を読まないため代替担保。→ **採用**。
- [Low-D] MID360 config 二重管理: robot 側を canonical と明記、vendored 側は「bringup 未使用」と記録。→ 採用(doc)。
- [Low-E] 非 symlink ビルドでは config 編集は再ビルドまで反映されない。→ 採用(運用メモ)。
- [Low-F] Mid360s 実機IP(SN依存)/機種ミス時サイレント無点群は設計で解消不可 → 艇上チェックリストへ転記。→ 採用。

## 実装計画への反映（確定）
- lidar.launch.py に choices 検証を入れる（Med-A）。
- robot/package.xml に exec_depend 追加（Med-B）。
- robot に `test/test_livox_config.py`（pytest）追加（Med-C）。
- docs/ai-discussion.md に canonical 明記 + 艇上チェックリスト（Low-D/E/F）。
