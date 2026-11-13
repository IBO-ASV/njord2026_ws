# ワークスペースのスクリプト

コマンド例はワークスペースルートから実行します。旧配置の互換wrapperはありません。

| ディレクトリ | 用途 |
|---|---|
| `setup/` | ビルド、環境設定、依存導入 |
| `operations/` | AGX・bringup・Zenoh起動、サービス、録画、診断 |
| `network/` | Wi-Fi・有線LAN・Livox IP設定 |
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
