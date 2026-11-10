# 設計レビュー1 (Sol代役=fable-escalation) + 司令塔裁定

- 日時: 2026-07-12
- 対象: サブタスク3 Livox Mid-360 / Mid-360s 両対応 初期設計案
- レビュアー: fable-escalation (Codex Sol 上限のため暫定。Sol は復帰後に追認予定)
- 実装者: sonnet-coder (予定, reviewer≠implementer)

## レビュー指摘と裁定

| # | 重大度 | 指摘 | 裁定 | 根拠/対応 |
|---|--------|------|------|-----------|
| H1 | High | livox_ros_driver2 は git submodule。config を submodule 内に置く案1は「vendored無改変」と矛盾 | **採用** | 司令塔が独立検証: `.gitmodules` に登録済/親repoが config を tracking せず。→ config を `src/robot/config/livox/` に移し submodule 不触に。 |
| H2 | High | 案C で vendored node 直接起動時のパラメータ再現漏れ(xfer_format, frame_id 等)で下流(glim/pcl)が沈黙 | **採用** | 9パラメータ全転記チェックリスト必須。param 名は `output_data_type`/`cmdline_input_bd_code` に注意。frame_id='livox_frame' 必須(glim が `livox_frame` 依存を独立確認)。 |
| M1 | Med | lidar_type=8 は機種非依存(SDK系フラグ)。35/9 に直すと `& 8` 偽で driver 初期化されない | **採用** | 8 固定。docに「lidar_type(=8) と device_type(9/35) は別空間」明記。JSONキーのみで機種決定。 |
| M2 | Med | s-config の ip=192.168.1.151 は個体SN依存でMid360s実機では別値の可能性 | **採用** | プレースホルダ明記+艇上チェック項目化。 |
| M3 | Med | 機種選択ミスはサイレント無点群 | **採用** | 運用手順に `ros2 topic hz /livox/lidar` + driver ログ dev_type 突合。 |
| M4 | Med | node起動スモークは host 192.168.1.5 NIC が要る(bind失敗で偽陰性) | **採用(方式調整)** | 司令塔検証: 現環境に192.168.1.5 無し。→ 一次担保は **launch dry-run(node起動せずbindしない)** に限定。node起動スモークは NIC alias 前提の任意手順。 |
| L1 | Low/Med | `enable_mid360=true` + `lidar_model=mid360s` の命名ねじれ | **保留→部分採用** | 互換維持で `enable_mid360` 残置+コメントで「Livox LiDAR起動可否」の意に読替。`enable_lidar` へのリネームは未確定事項として繰越。 |
| L2 | Low | 両モデルを1 JSON に同居させない | **採用** | 1モデル=1ファイル(selectable)を doc に固定。 |
| L3 | Low | HW無テストの担保範囲/非担保を明文化 | **採用** | doc に担保(build/dry-run/JSON lint) と非担保(実機dev_type=35応答/ファーム互換/実IP/点群品質)を明記しリスク受容記録。 |

## 反論・保留（司令塔判断）
- 反論なし。全 High/Med を採用。L1 のみリネームを繰越(互換優先)。
- レビュアーが指摘した fork の Mid360s 対応が実機実績不明な点(L3内)= 本タスク完了ライン(code/build/dry-run)の外。**リスク受容**として明記し艇上bench testで確認。

## 結論
- 採用アーキ: **案C 修正版** = config を `src/robot/config/livox/` に配置し、robot 側 `lidar.launch.py` から vendored `livox_ros_driver2_node` を直接起動。submodule(Livox-SDK2 / livox_ros_driver2)は一切改変しない。
