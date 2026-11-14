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


## AGX velocity tuning

With the AGX bringup running from this workspace, start the idle velocity tuner in another AGX terminal with `scripts/operations/run_agx_velocity_tuner.sh`. The script applies the same ROS domain 126 and CycloneDDS profile as `run_agx_pair.sh vessel`; the tuner publishes no movement command until started from Mission Control.
詳細な画面操作・試験結果の見方は `docs/operations/velocity_gain_autotuning.md` を参照してください。

### AGX / Ground PCをDDSで直接接続する

`operations/run_agx_pair.sh vessel`と`operations/run_agx_pair.sh ground`は
ROS_DOMAIN_ID=126、CycloneDDSで起動し、既定でcritical linkを無効にします。
Ground側は`/joy`と`/heartbeat/ground_station`をDDSで直接送信します。
critical linkを明示的に有効化した場合は専用入力トピックへ切り替わります。

Ground側のDDS NICは`ip -4 route get 192.168.1.5`の送信元IPv4から選択します。
AGXの接続先が異なる場合は`NJORD_AGX_ADDRESS=<AGXのLAN IP>`を指定してください。
この変数はNIC選択に使います。DDSの固定peer設定やVPN経由の探索は追加しません。
独自DDS設定を使う場合は`NJORD_GROUND_CYCLONEDDS_URI=/absolute/path/config.xml`を指定できます。
AGX側は従来の`cyclonedds_agx.xml`を使います。

```bash
# launch変更後はrobotパッケージを再ビルドする
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot --symlink-install
scripts/operations/run_agx_pair.sh ground
```

`ground`は従来の全部入りGround PC launchであり、joyとheartbeat以外に映像受信、
地図・経路表示用ノード、NTRIP caster、Ground側Foxglove bridgeなども起動します。
Foxglove自体はAGXの`ws://192.168.1.5:8767`へ直接接続できます。
Ground側のbridgeとNTRIP casterが不要なら、次のように無効化できます。

```bash
scripts/operations/run_agx_pair.sh ground enable_foxglove_bridge:=false enable_ntrip_caster:=false
```

Ground PCに`192.168.1.0/24`のNICがない状態でAGX固定のDDS XMLを使うと、
`192.168.1.0: does not match an available interface`でROSノードの起動に失敗します。
NICを選べても通信できるとは限らないため、AGXへのルートとDDSの探索・疎通を別途確認してください。
