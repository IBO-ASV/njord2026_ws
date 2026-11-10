# GNSS・IMU・LiDARを載せたROS 2船で、EKFとTFを破綻させない設計

屋外を走る（航行する）ロボットでは、GNSSの絶対位置とIMU・LiDAR odometryの短期的な連続性を同時に使いたくなります。ここで重要なのは、EKFの精度以前に、各フレームと各ノードの責務を一意にすることです。本記事は `robot_localization` を使う際の、`map`、`odom`、`base_link`、`navsat_transform_node` の設計をまとめます。

対象は、UM982のようなRTK GNSS + デュアルアンテナ方位、Livox MID-360のIMU、LiDAR odometry/SLAMを搭載する船体を想定しています。

## まず結論

推奨するTFの責務は次のとおりです。

```text
map ── global EKF ──> odom ── local EKF / continuous odometry ──> base_link
                                                                  └─> sensor frames
```

| 要素 | 役割 | ジャンプしてよいか |
|---|---|---|
| `odom -> base_link` | 短期的な自己位置。局所制御・局所経路計画の基準 | いいえ |
| `map -> odom` | GNSS、AMCL、SLAM等によるグローバル補正 | はい |
| `map -> base_link` | 上記二つを合成した現在地 | 結果として補正を反映する |
| `base_link -> sensor_link` | センサ取付位置・取付姿勢 | 静的TF |

`base_link`に親を二つ持たせてはいけません。したがって、local EKFとglobal EKFがそれぞれ`odom -> base_link`と`map -> base_link`を同時にpublishする構成は不正です。

## なぜ `world_frame` でEKFのTF出力が変わるのか

`robot_localization` のEKFは、状態を常に「`world_frame`に対する`base_link`の姿勢」として推定します。

```text
world_frame: odom なら T_odom_base_link を推定する
world_frame: map  なら T_map_base_link  を推定する
```

local EKFがすでに`odom -> base_link`を所有している場合、global EKFが推定値そのままの`map -> base_link`を出すと、TF木の`base_link`に`map`と`odom`の二つの親ができます。そこでglobal EKFはTFからlocal側の姿勢を取得し、次の関係で`map -> odom`をpublishします。

```text
T_map_odom = T_map_base_link × inverse(T_odom_base_link)
```

設定は両方のEKFでフレーム名をそろえ、`world_frame`だけを分けます。

```yaml
# local EKF: IMU、wheel/visual/LiDAR odometryなど連続な観測を融合
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true
```

```yaml
# global EKF: 連続な観測 + GNSS/AMCL/SLAMなど絶対位置を融合
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: map
publish_tf: true
```

`map_frame`や`odom_frame`は名前の定義です。どちらの動的edgeを担当するかは`world_frame`と、既存のTF責務で決まります。

## センサごとの置き場所

| センサ・推定器 | 性質 | 主な置き場所 |
|---|---|---|
| Livox IMUの角速度・加速度 | 高レートだが、yawは積分すると漂う | local EKF |
| wheel / thrust model / visual odometry / LiDAR odometry | 短期的に連続、原点は漂う | local EKF |
| UM982 RTK位置 | 絶対位置、RTK状態やマルチパスで不連続になり得る | `navsat_transform_node`経由でglobal EKF |
| UM982デュアルアンテナ方位 | 地球基準の絶対yawになり得る | local EKFのyaw観測、またはnavsatの方位入力 |
| AMCL | 地図に対するグローバル位置。再収束時に補正する | global EKFへの`map`基準観測 |
| SLAMの`map`姿勢 | グローバル補正を含むことがある | global側、または`map -> odom`の唯一の所有者 |

AMCLは「連続odometry」ではありません。local EKFの情報源にすると、再localizationのジャンプを`odom -> base_link`へ持ち込んでしまいます。

## UM982 compassとLivox IMUはどう融合するか

「GNSS compassのyawを混ぜたIMUメッセージ」を自作するより、EKFに別観測として入力します。

```text
Livox gyro z ────────────────> local EKF ──> odom -> base_link
UM982 absolute yaw ──────────┘
```

これは、ジャイロ積分の短期的な滑らかさを使いつつ、絶対yawで長期ドリフトを抑える普通の設計です。ただし有効にする前に、次を実測で確認します。

- UM982 yawがENU/FLUのROS座標系に合うこと（NEDなら変換する）。
- 取付yawオフセット、アンテナ基線の向き、RTK状態とyaw covarianceが正しいこと。
- Livox IMUのframeと`base_link`の静的TFが正しいこと。
- RTK未解・低速・アンテナ遮蔽時に、compass yawのcovarianceを十分大きくすること。

位置とyawが同じGNSS装置に由来する場合、通常のEKFは両者の誤差相関を表現できません。位置を同じEKFに重複して投入しないこと、yawだけを必要な共分散で使うことが保守的な方針です。

## `navsat_transform_node` は何をするか

`NavSatFix`の緯度・経度・高度を、EKFのworld座標系で使える`nav_msgs/Odometry`へ変換するノードです。初期化時に次の三つから、GPS/UTM座標とローカル座標の対応を求めます。

1. `gps/fix`の緯度・経度
2. 地球基準の方位
3. `odometry/filtered`の現在姿勢

その後、変換済みの`odometry/gps`をglobal EKFへ入力します。`navsat_transform_node`自身が通常`map -> odom`を出すわけではありません。`map -> odom`の所有者はglobal EKFです。

### `use_odometry_yaw` の意味

これは**navsat_transform_nodeのパラメータ**です。

```yaml
# navsat_transform_node
use_odometry_yaw: true
```

| 設定 | navsatが方位として使うもの |
|---|---|
| `true` | `odometry/filtered`入力のyaw。地球基準の絶対yawでなければならない。 |
| `false` | `imu`入力のorientation yaw。ENUで、東向きがyaw=0となる規約に合わせる。 |

Livox IMUだけでは絶対yawがないため、単に`false`へ変えるのは不適切です。UM982 compassをlocal EKFのyawに正しく融合できているなら、local EKF出力をnavsatへ入力して`true`にする選択には根拠があります。

## 公式dual-EKF例をそのまま適用できる条件

公式例ではnavsatの`odometry/filtered`に**global EKF出力**を接続します。navsatの出力Odometryは入力Odometryのworldフレームを引き継ぐため、この構成ならGPS Odometryも`map`基準になります。

```text
continuous sensors ─┬─> local EKF  ──> odom -> base_link
                    └─> global EKF ──> map -> odom
GPS + heading + global EKF odometry ──> navsat_transform ──> global EKF
```

これはglobal EKFにも、GNSSとは独立した連続位置・速度・方位の観測が入る場合に有効です。global EKFが起動直後から妥当な姿勢、とくに方位を出せるためです。公式例もwheel odometryとIMUをlocal/globalの双方へ投入しています。

一方、global EKFがGNSS位置だけを融合し、navsatが`use_odometry_yaw: true`である構成では、global出力をnavsatへつなぐだけではいけません。global側に有効なyawがなく、GPS座標をローカル座標へ固定する初期方位がゼロなどの不正値になるからです。この場合は、以下のどちらかを先に満たします。

- global EKFへGNSSと独立した連続odometryと絶対yawを入力する。
- navsatへENU準拠の絶対方位IMUを与え、`use_odometry_yaw: false`にする。

条件未達なら、公式例へ機械的に変更せず、local EKFの絶対yawをnavsatに使う現在の設計を検証しながら維持します。

### 本リポジトリの入力防御

UM982ドライバは、GGA/THSの数値が不正、NaN/Inf、または無効fixなら通常topicへpublishしません。quality=0はSTATUS_NO_FIXとしてdebug topicだけに流します。予期しないパース例外が発生しても次のGNSS readを先に再登録するため、ノードだけ生存して入力が永久停止する状態を避けます。

localization_input_guardはcompass、navsat変換後Odometry、Livox IMU、GLIM Odometryを検査し、全体が有限値でquaternionが正規化済みなmessageだけをglobal EKFへ中継します。global EKFにはGNSSと独立したLivoxのyaw rateと平面加速度も連続入力します。

## `odom`が大きく飛ぶとき

望ましいのは`odom -> base_link`が連続で、GNSS補正により`map -> odom`だけが変化する状態です。

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

- 1行目が飛ぶ: local odometryのリセット、局所推定器の不連続、またはTF二重publishを疑う。
- 2行目が飛ぶ: GNSS/AMCL/SLAMによるグローバル補正としてあり得る。
- 3行目だけ見て正常と判断しない: `map -> odom`がlocal側の飛びを相殺している場合がある。

特に、local EKF、UM982 feedback EKF、GLIM、SLAMのうち二つ以上が同じ動的edgeをpublishしていないかを`view_frames`と`/tf`のpublisher一覧で確認します。

## 実装前チェックリスト

- [ ] 動的TFの各edgeにpublisherが一つだけである。
- [ ] `odom -> base_link`が静止時・低速航行時に連続である。
- [ ] GNSSアンテナの`base_link -> gnss_link`静的TFが実測値である。
- [ ] GNSS compassのyaw軸・符号・取付オフセット・covarianceを屋外で確認した。
- [ ] navsatに渡すyawが地球基準である。
- [ ] GPSをglobal EKFに`differential: false`で入力している。
- [ ] GNSS由来の位置を同一EKFへ二重投入していない。

## 参考

- [robot_localization: Integrating GPS Data](https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html)
- [robot_localization: dual EKF + navsat example](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/launch/dual_ekf_navsat_example.launch.py)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
