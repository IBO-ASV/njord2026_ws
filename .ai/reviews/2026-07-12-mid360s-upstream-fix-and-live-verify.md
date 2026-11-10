# livox_ros_driver2 upstream取り込み + 実機検証（追加調査）

- 日時: 2026-07-12
- 発端: ユーザ指示「公式最新版に対してbehindなら取り込め」+「同様のエラー事例をweb調査」
- 担当: メインClaude（調査・修正・検証すべて実施。差分が1行の明確なバグ修正のため直接対応。ユーザ承認済み）

## 調査結果

### Livox-SDK2
- fork(HEAD)は公式 tag `v1.3.1` と機能的に完全同一（`git diff --stat` の差分はCMake/package.xmlのROS2パッケージング化のみ）。
- **更新不要**。

### livox_ros_driver2
- fork ベースは公式 `1.2.4` 相当（+独自13コミット）。公式は **`1.2.6`(commit `13eb05e`)** でMid-360s対応を修正済みで、fork未取り込み（1コミット behind）。
- 実質差分は `src/comm/pub_handler.cpp` の1行（Mid-360sがpoint cloud line_num計算で`kLineNumberDefault`に落ちていた）:
  ```cpp
  // 修正前
  } else if (dev_type == kLivoxLidarTypeMid360) { packet.line_num = kLineNumberMid360; }
  // 修正後(公式)
  } else if (dev_type == kLivoxLidarTypeMid360 || dev_type == kLivoxLidarTypeMid360s) { packet.line_num = kLineNumberMid360; }
  ```
  他はinclude順整理・バージョン番号のみで機能差分なし。
- **fork submodule に同内容を適用済み**（未commit, working tree のみ。ユーザ承認済み）。

## Web調査

- **GitHub Issue #240** (Livox-SDK/livox_ros_driver2) "Init lds lidar successfully! but no topic Livox Mid360s":
  今回我々が実機で踏んだ症状（`Init lds lidar successfully!`は出るがトピック無し）と一致する報告多数。
  公式メンテナ回答: 「1.2.6でMid-360s対応修正済み。Livox SDK2もv1.3.1以降に更新すること。config/MID360s_config.json・launch_ROS2/msg_MID360s_launch.pyを使うこと」。
- **GitHub Issue #30** (Livox-SDK/livox_ros_driver2) "MID360:Init lds lidar success,but can not get index":
  同一エラーメッセージだが解決策はケーブル/電源/リンク確認。汎用的な接続不良の症状としても既知。

## 実機検証（本セッション）

1. 修正前 mid360s 起動 → `Can not get index, the livox lidar type:8, handle:1912711360` が即座に洪水（422件/8秒）。
2. ハンドル値をIPへデコード: 登録済み(config記載 `192.168.1.151`) = `2533468352` ≠ 着信 `1912711360` = **`192.168.1.114`**。
   → **実機の実IPが config のプレースホルダ(.151)と異なっていた**（設計レビューの Low-F 指摘が実際に発生）。
3. `pub_handler.cpp` 修正 + IP を実機値 `.114` に補正したテスト用config（robot pkg外, `git status`に影響なし）で再検証:
   - `Can not get index` = **0件**。
   - ログに `livox/imu publish use imu format` / `livox/lidar publish use livox custom format`（初回データ受信でのみ出力）。
   - `ros2 topic hz /livox/lidar` = **平均 9.998〜10.007 Hz**（`publish_freq:=10.0`設定と一致）。
   - `/livox/imu` `/livox/lidar` ともトピック生成確認。
   → **実機Mid-360s相当機との end-to-end 疎通を実際のデータレートで確認**。

## 結論
- `pub_handler.cpp` の1行修正は正しい upstream 取り込みであり、保持すべき（無くても今回のIP修正だけでフラッドは消えたが、修正しないとpoint cloudのline_num処理が誤ったままになる既知バグ）。
- 今回のフラッドの主因は **config の実機IPプレースホルダ不一致**であり、コード側の別バグではなかった。艇上運用時は実機SNからIPを確認し `MID360S_config.json` を更新する運用手順（既にdocs/ai-discussion.md §10に記載済み）が必須である点を再確認。
- 検証に使ったIP `192.168.1.114` はこの開発機ネットワーク上の実機の値。

## 完了（ユーザ承認・指示により実施, 2026-07-12 夜）
- ユーザ指示: 「mid360は151, sは114をデフォルトとし、KeioRoboticsAssociationへのpushまで」。
- MID360_config.json の ip は `192.168.1.151` のまま既定（変更なし）。
- MID360S_config.json の ip を `192.168.1.114` に既定変更。
- livox_ros_driver2 submodule: ローカル `master` を現行pin(`95d96d3`)にfast-forward後、
  `pub_handler.cpp` の1行修正を commit(`e0382eb "Set correct point cloud line_num for Mid-360s"`)し、
  `KeioRoboticsAssociation/livox_ros_driver2` の `master` へ push 済み(`95d96d3..e0382eb`)。
- 親リポジトリ側 submodule ポインタは `95d96d3..e0382eb` への更新差分として認識(未commit, 親側で`git add`要)。
- `colcon build --packages-select robot livox_ros_driver2` 成功、`colcon test --packages-select robot` = 53 tests/0 failure 再確認。
