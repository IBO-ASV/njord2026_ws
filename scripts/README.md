# ワークスペースのスクリプト

コマンド例はワークスペースルートから実行します。旧配置の互換wrapperはありません。

| ディレクトリ | 用途 |
|---|---|
| `setup/` | ビルド、環境設定、依存導入 |
| `operations/` | AGX・bringup・Zenoh起動、サービス、録画、診断 |
| `network/` | Wi-Fi・有線LAN・Livox IP設定 |
| `calibration/` | 速度追従・推力ペア補正・MMG同定（実機への適用は自動で行わない） |
| `simulation/` | mission/suite実行、観測、汎用評価 |
| `visualization/` | Foxglove preview・clip・replay |
| `yolo/` | runtime検証、モデルexport・benchmark、YOLO起動 |
| `benchmark/` | MPPI・intra-process計測 |

```bash
source scripts/setup/njord_env.sh
scripts/setup/build.sh
scripts/operations/run_agx_pair.sh vessel
python3 scripts/simulation/run_sim_suite.py --help
```

生成物の保存先は `artifacts/` とします。simulationは `--output artifacts/<新規実験名>` を指定します。
既存systemdサービスは旧スクリプトパスを保持しているため、移行後に
`scripts/operations/install_bringup_service.sh` で再生成してください。
今回の整理では稼働中のサービスは更新・再起動していません。
ROSパッケージ専用・CI専用スクリプトは各パッケージ・`.github/scripts/` に残します。
8月10日専用スクリプトの処理条件・取得元は [GLIM検証](../docs/validation/glim_experiments.md) を参照してください。

## AGX専用サービス

Orin Nanoの `jetson` ロールとは別に、`agx` ロールを使用します。
Jazzy・domain 126・CycloneDDSで `run_agx_pair.sh vessel` を起動します。

```bash
# ユーザーサービスとして登録・有効化（いまは起動しない）
scripts/operations/install_bringup_service.sh --role agx --ros-distro jazzy --user-service --no-start
systemctl --user status njord-agx-bringup.service
# 実機を起動する際
systemctl --user start njord-agx-bringup.service
```

ユーザーサービスはユーザーマネージャー起動時（通常はログイン時）に起動します。
ログイン前のOS起動時から必要な場合は、管理者権限で `--user-service` を外して
システムサービスを登録してください。同時に両方を有効化しないでください。
`--output-dir artifacts/systemd` はunitの生成だけを行い、登録・起動は行いません。

## 関連テスト

package shareを参照するため、ROSとビルド済みworkspaceをsourceしてから実行します。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="src/sim/njord_sim:${PYTHONPATH:-}" \
  python3 -m pytest -p no:cacheprovider -q \
  src/bringup/robot/test/test_launch_roles.py src/bringup/robot/test/test_platform_manifest.py \
  src/bringup/robot/test/test_livox_config.py scripts/yolo/test/test_cusparselt_metadata.py \
  src/sim/njord_sim/test/test_suite_reporting.py src/sim/njord_sim/test/test_dependencies.py \
  scripts/operations/test/test_bringup_service.py
```
