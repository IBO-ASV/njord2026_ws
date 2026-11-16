# 無料RTK補正を既存NTRIP経路で受ける

地上PCで固定基準局のRTCMを受け、既存キャスターの `RTCM3` へ中継する。
AGXの既存NTRIP設定（地上PC:2101、RTCM3、test/test）をそのまま利用できる。
補正配信元は既定で `geortk.jp:2101/tamtam`（横浜）。今回の試験場所の約9 km先であり、
他地域でも適した基準局とは限らない。現地の近い稼働局を指定すること。

提供元: https://www.geosense.co.jp/news_ntripcaster_open/
公開局一覧: https://geortk.jp/mountpoint

無料公開局の座標と国家座標の整合・常時稼働は別途確認する。
RTK FIXだけで絶対位置の正確さを保証するものではない。

## 地上PC

```bash
git pull --ff-only
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot ntripcaster --symlink-install
source install/setup.bash
ros2 launch robot ground_pc.launch.py
```

`enable_ntrip_caster:=true` と `enable_ntrip_relay:=true` が既定。
中継の接続先は次の引数で変更する。

```bash
ros2 launch robot ground_pc.launch.py \
  ntrip_upstream_host:=geortk.jp \
  ntrip_upstream_port:=2101 \
  ntrip_upstream_mountpoint:=tamtam
```

自前基準局が既存のRTCM3へ送信する場合は `enable_ntrip_relay:=false`。
同じマウントポイントへ二つの送信元を同時に接続しない。
`enable_ntrip_caster:=false` なら地上PCのキャスター・中継の両方を起動しない。
キャスターは既存設定の0.0.0.0:2101を使用する。

この中継は固定局用であり、GGAを要求するVRS/NEAR局には非対応。
上流NTRIP認証はスクリプト引数で利用可能だが、launchが公開する引数は無料固定局用。
HTTPS・HTTP chunked転送は非対応。HTTPエラーや局一覧を補正データとして転送しない。
CRC24Qが正常なRTCM3フレームのみ転送し、30秒ごとにメッセージ別件数を出す。
接続切断または15秒間有効フレームがない場合は、両側を閉じ5秒後に再接続する。
ローカルSOURCE認証は既存のcaster JSONから読むため、サブモジュール内の設定変更は不要。

## AGXだけで試験

地上PCのGUI・映像・操作ノードを起動せず、補正経路だけを起動できる。
同じホストの2101番ポートに別キャスターがないことを確認する。

```bash
source install/setup.bash
ros2 launch robot ntrip_corrections.launch.py
```

別ターミナルでUM982ドライバ単体を起動する（他のUM982ドライバと同時起動しない）。
DDS設定は実在する `src/bringup/robot/config/cyclonedds_agx.xml` を指定する。

```bash
source install/setup.bash
export CYCLONEDDS_URI=file:///home/ibo/njord2026_ws/src/bringup/robot/config/cyclonedds_agx.xml
ros2 launch um982_driver um982.launch.py \
  rtk_enable:=true ntrip_server:=127.0.0.1 \
  ntrip_port:=2101 ntrip_mountpoint:=RTCM3 \
  log_file_name:=/新しい保存先/um982.raw
```

上記2つはセンサ/推進全体のbringupを呼ばず、スラスターを起動しない。
終了はそれぞれCtrl-C。キャスター/中継を常駐サービスへ登録する処理は行わない。

GGA quality: 1=Single、2=DGPS、5=RTK Float、4=RTK FIX。
ROS標準NavSatStatusはRTK FloatとFIXを区別できないので、生GGAも記録する。
中継接続成功、CRC正常な受信、UM982の補正利用、RTK FIX成立は別々に確認する。
