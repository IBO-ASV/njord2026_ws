# AI 設計討議録 — Livox Mid-360 / Mid-360s 両対応

- 日付: 2026-07-12
- ブランチ: test07089
- 議長/アーキテクト: メイン Claude (Opus 4.8)
- 設計レビュアー: fable-escalation（Sol 代役／Codex 利用上限のため。Codex 復帰後に GPT-5.6 Sol が追認予定）
- 実装担当（予定）: sonnet-coder（実装者≠レビュアーを維持）

## 1. 要求

ROS2 Jazzy の ASV プロジェクト `njord2026_ws` で、Livox **Mid-360** と **Mid-360s** の
両方を robot bringup から使用可能にする（1 台を機種選択して使う selectable 方式。同時 2 台は非対象）。
完了ライン = code / build / dry-run まで。実 HW 通電は艇上工程。

## 2. 事実確認（コード調査）

- vendored（**git submodule**）: `Livox-SDK2`, `livox_ros_driver2`（KeioRoboticsAssociation fork）。
  親リポジトリは submodule 内ファイルを直接 tracking しない。
- SDK `parse_cfg_file.cpp`: `dev_type_map = {"HAP":.., "MID360":9, "Mid360s":35}`。
  **JSON トップレベルキーで device_type が決まる**。旧形式(host_net_info=オブジェクト)も有効。
- SDK: Mid360 と Mid360s のポートは全て同値。機種別分岐は params_check のポート検証のみ。
- ROS driver `MID360_config.json`: `lidar_summary_info.lidar_type:8`(=`kLivoxLidarType`,
  **ROS driver 側の SDK 世代フラグ。機種非依存**。`& 8` でマスク判定。9/35 に変えると初期化されない)、
  `"MID360"` net-info ブロック、`lidar_configs`(extrinsic, ip=192.168.1.151)。
- ROS driver の JSON parse は `lidar_summary_info` と `lidar_configs` のみ読む（ブロックキー名は SDK 用）。
- glim は `livox_frame`（imu/lidar_frame_id）と `/livox/lidar` `/livox/imu` に固定依存。
- 現開発環境に host IP 192.168.1.5 を持つ NIC は無い。

## 3. 合意した設計（案 C 修正版）

vendored submodule を**一切改変せず**、robot パッケージ側で完結させる。

1. **config を robot 側に配置**
   - `src/robot/config/livox/MID360_config.json`（既存 vendored のコピー）
   - `src/robot/config/livox/MID360S_config.json`（コピーで net-info ブロックキー
     `"MID360"` → `"Mid360s"` のみ変更。`lidar_summary_info.lidar_type:8`・`lidar_configs`・
     ポート・IP は据え置き）
   - robot CMake は既に `install(DIRECTORY launch config urdf ...)` なので追加不要。
2. **robot 側 launch 新設** `src/robot/launch/lidar.launch.py`
   - vendored `livox_ros_driver2` の `livox_ros_driver2_node` を直接起動。
   - `lidar_model` 引数（`mid360` 既定 / `mid360s`）で `user_config_path` を上記 config に切替。
   - **9 パラメータを msg_MID360_launch.py から完全転記**（下記チェックリスト）。
3. **real_bringup.launch.py 改修**
   - 既存 `enable_mid360` を残置（コメントで「Livox LiDAR 起動可否」の意に読替）。
   - `lidar_model` 引数を追加し、include 先を vendored の msg_MID360_launch.py から
     robot の `lidar.launch.py` へ差し替え、`lidar_model` を渡す。

### パラメータ転記チェックリスト（H2 対策・必須）
| param | 値 | 備考 |
|---|---|---|
| xfer_format | 1 | CustomMsg。0 にすると PointCloud2 で下流沈黙 |
| multi_topic | 0 | |
| data_src | 0 | |
| publish_freq | 10.0 | |
| output_data_type | 0 | ※param 名は output_data_type（launch 変数名 output_type ではない） |
| frame_id | 'livox_frame' | glim/TF 依存。欠落で TF 断 |
| lvx_file_path | '/home/livox/livox_test.lvx' | |
| user_config_path | lidar_model により切替 | 本タスクの肝 |
| cmdline_input_bd_code | 'livox0000000001' | ※param 名は cmdline_input_bd_code |

## 4. 意見が割れた部分 / 保留

- **enable_mid360 のリネーム**（`enable_lidar` 化）: 命名がねじれるが後方互換優先で**保留**。
  コメント補足のみ。将来の未確定事項として繰越。
- **fork の Mid360s 対応の実機実績**: fork commit 履歴上、対応途上の可能性。
  本タスク完了ライン外のため**リスク受容**として記録。艇上 bench test で確認。

## 5. 採用した判断と理由（要点）

- 案 C を選んだ理由: submodule を触らず（コミット/ポインタ更新の運用コスト回避）、
  艇固有のネットワーク/extrinsic 設定を robot 側に集約でき、upstream リベース面が最小。
- config を submodule でなく robot 側に置く理由: H1（submodule 改変矛盾）の解消。
- lidar_type=8 据え置きの理由: M1（ROS driver 世代フラグで機種非依存。変更で初期化失敗）。

## 6. 実装手順（レビュー2の Med 反映済み）

1. `src/robot/config/livox/MID360_config.json` 追加（vendored コピー。**robot 側を canonical とする**）。
2. `src/robot/config/livox/MID360S_config.json` 追加（net-info ブロックキーを `"Mid360s"` に変更）。
   ip はプレースホルダである旨コメント（JSON はコメント不可のため doc/PR に明記）。
3. `src/robot/launch/lidar.launch.py` 新設（node 直接起動 + `lidar_model` で config 切替 + 9 param 転記）。
   - `DeclareLaunchArgument("lidar_model", default_value="mid360", choices=["mid360","mid360s"])`（**Med-A: fail-fast**）。
   - 冒頭コメントに「config の canonical は robot 側。vendored config/launch は bringup 未使用」（**Low-D**）。
4. `src/robot/launch/real_bringup.launch.py`: `lidar_model` 引数追加、include を lidar.launch.py に差替、
   `enable_mid360` にコメント。
5. `src/robot/package.xml`: `<exec_depend>livox_ros_driver2</exec_depend>` 追加（**Med-B**）。
6. `src/robot/test/test_livox_config.py` 追加（**Med-C**、下記 T2 の pytest 化）。CMake の BUILD_TESTING に登録。
7. `colcon build --packages-select robot` → `colcon test --packages-select robot` → dry-run 検証。

## 7. テスト項目（HW 無しで担保）

- T1: `colcon build --packages-select robot` 成功（config/launch install 反映）。
- T2（pytest, Med-C）: 両 config について
      (a) JSON parse 可能、(b) MID360S はトップキー `"Mid360s"` 有・`"MID360"` 無、
      (c) `lidar_summary_info.lidar_type == 8`、(d) `lidar_configs` と host 側 net-info 必須 field 存在。
- T3: `ros2 launch robot real_bringup.launch.py --show-args` に `lidar_model` が出る。
- T4: `ros2 launch robot lidar.launch.py lidar_model:=mid360 --print`（および `mid360s`）で
      node パラメータ 9 個が正しく解決（特に xfer_format=1, frame_id=livox_frame,
      user_config_path が該当 config を指す）。不正値（例 `mid360S`）は choices で launch が fail-fast。
- T5: sim 経路（task*_sim / manual_control）非破壊（include 変更が real_bringup 限定であること）。

### 担保できない項目（非担保・明記）
- 実機 Mid360s が detection で device_type=35 を返すこと。
- fork の Mid360sCommandHandler 経路のファーム互換。
- 実 IP（SN 依存）、点群/IMU データ品質。
- node 起動スモークでの SDK ソケット bind（host 192.168.1.5 NIC 前提。任意手順）。

## 8. 運用手順（艇上・ドキュメント化）

- 起動後 `ros2 topic hz /livox/lidar` で点群配信を確認（機種選択ミスはサイレント無点群のため）。
- driver ログの検出 dev_type と選択 `lidar_model` を突合。
- Mid360s 実機の IP を SN から確認し `MID360S_config.json` の `lidar_configs.ip` を更新。

## 9. 討議往復ログ

- R1: fable-escalation が初期案（config を submodule 内に置く前提）を批判
  → H1(submodule)/H2(param 漏れ)/M1(lidar_type)ほか。司令塔が H1/glim/NIC を独立検証し全採用。
- R1 裁定: 案 C 修正版（config を robot 側へ）で確定。詳細 `.ai/reviews/2026-07-12-review1-mid360s-design.md`。
- R2: 修正版を再レビュー → **合意**（High 級欠陥なし）。Med 3点（choices 検証 / exec_depend / config pytest）を
  実装計画に反映。詳細 `.ai/reviews/2026-07-12-review2-mid360s-design.md`。往復 2 回で終了。
- 次: sonnet-coder が実装（実装者≠レビュアー）。Codex 復帰後に GPT-5.6 Sol が本設計を追認予定。

## 10. 艇上チェックリスト（HW 工程・非担保項目）
- Mid360s 実機の SN を確認し `MID360S_config.json` の `lidar_configs.ip` を実 IP に更新（既定 .151 は個体依存プレースホルダ）。
- 起動後 `ros2 topic hz /livox/lidar` で点群配信を確認（機種選択ミスはサイレント無点群）。
- driver ログの検出 dev_type と選択 `lidar_model`（mid360=9 / mid360s=35）を突合。
- config を編集したら robot を再ビルド（非 symlink ビルドでは installed share に即反映されない）。
