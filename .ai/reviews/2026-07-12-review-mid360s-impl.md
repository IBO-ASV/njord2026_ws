# 実装レビュー — Livox Mid-360 / Mid-360s 両対応

- 日時: 2026-07-12
- 実装者: sonnet-coder
- レビュアー: メイン Claude (Opus 4.8, 司令塔) ※実装者≠レビュアー
- 結論: **合格**。テスト失敗ゼロ → Sol/Fable 昇格なし。

## 変更ファイル（すべて robot pkg 内、submodule 無改変）
- `src/robot/config/livox/MID360_config.json`（vendored と **完全一致**を diff で確認）
- `src/robot/config/livox/MID360S_config.json`（vendored コピーから **1行のみ** 差分: `"MID360"`→`"Mid360s"`。lidar_type=8/lidar_configs/ports/IP 据え置き）
- `src/robot/launch/lidar.launch.py`（新設。OpaqueFunction で dict 引き、9 param 完全転記、choices 検証、canonical コメント）
- `src/robot/launch/real_bringup.launch.py`（include を lidar.launch.py へ差替、lidar_model 引数追加、enable_mid360 コメント）
- `src/robot/package.xml`（exec_depend livox_ros_driver2 追加）
- `src/robot/test/test_livox_config.py` + `CMakeLists.txt`（ament_add_pytest_test 登録）

## 独立検証（司令塔が再実行）
- submodule 不触: `git status src/driver/lidar/` にライブラリ差分なし。✅
- config 正当性: robot MID360 == vendored MID360（IDENTICAL）、MID360S は単一キー差のみ。✅
- **T1 build**: `colcon build --packages-select robot` 成功。✅
- **T2 pytest**: `colcon test` → 53 tests, 0 errors, **0 failures**, 7 skipped（livox config 8 テスト含む）。✅
- **T3 show-args**: `real_bringup --show-args` に `lidar_model` と `enable_mid360` 両方出現。✅
- **T4 config 解決**: installed share に MID360_config.json / MID360S_config.json 存在、
  `ros2 pkg executables livox_ros_driver2` = `livox_ros_driver2_node` 発見可、dict マッピング決定的。
  **不正値 `mid360S` は実 launch で EXIT=1・node 起動前に即エラー**（fail-fast 確認）。✅
- **T5 sim 非破壊**: 本タスクで src/sim・simple_manual に新規差分なし。✅

## 非担保（設計どおり・艇上工程）
- 実 node 起動スモークは host 192.168.1.5 NIC 不在のため bind でハングし省略（設計で任意判定済）。
- 実機 Mid360s の device_type=35 応答 / fork ファーム互換 / 実 IP / 点群品質。

## 未決（本タスク外）
- Codex 復帰後の **GPT-5.6 Sol による設計追認**（利用上限で保留、~20:48 復帰）。
