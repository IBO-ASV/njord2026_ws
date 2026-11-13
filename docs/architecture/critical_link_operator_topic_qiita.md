# ROS 2 Humble/Jazzy 間で Service/Action が通らないとき、認証済み UDP topic 経路を作った

Ground PC は ROS 2 Jazzy、船載 miniPC は ROS 2 Humble という構成で、Zenoh bridge を介してミッション操作を試した。

`/mission/list_tasks` のような Service は graph 上では見えるのに応答しない。`/mission/run_task` Action も miniPC 側で不正な request として破棄された。topic の publish/subscribe は通るため、操作要求だけを認証済み UDP transport に載せ、船側で既存の Mission Manager API を呼ぶ構成にした。

## 結論

Ground PC が Mission Manager を直接操作しない。

```text
Ground PC
  njord-critical
    -> /critical_link/input/operator_command
    -> authenticated UDP
miniPC
  /critical_link/operator_command
    -> operator_dispatcher
    -> /mission/start_task などのローカル API
    -> OperatorResponse
    -> authenticated UDP reply
Ground PC
  /critical_link/output/operator_response
```

これで Zenoh の Service/Action proxy に依存せず、既存の Mission Manager を唯一の実行者として残せる。

## 発生した問題

Zenoh bridge の allow list に Service/Action を入れても、異なる ROS 2 distribution 間では次のような状態になった。

- `ros2 service list` には Service 名が出る
- `wait_for_service()` は timeout する
- Action request が miniPC 側で `less than 20 bytes` として破棄される

名前の discovery と request/response の相互運用性は別物だった。

## topic 化で注意したこと

最初は `rclcpp::Serialization<OperatorCommand>` の結果を UDP に入れた。しかし Jazzy 側で直列化したバイト列を Humble 側で復号できず、receiver は `invalid_payload` を増やした。

そのため operator command/response だけは ROS の内部シリアライズを使わない固定 wire format にした。

```text
OperatorCommand
  request_id: u64
  command: u8
  target: u8
  requested_mode: u8
  task_id: u16 length + UTF-8 bytes

OperatorResponse
  request_id: u64
  result_code: u8
  message: u16 length + UTF-8 bytes
  mission/control/runtime fields
```

UDP frame 自体は既存の `critical_link` protocol を利用する。frame には session ID、sequence、timestamp、CRC、HMAC が含まれる。

## 実装上の修正

今回、次の問題も修正した。

1. miniPC receiver の VPN bind address が Ground PC の IP になっていた
2. operator command の stream ID は 4 だが、sender の sequence 配列は index 0--3 しかなく SIGSEGV した
3. receiver が command を受けても response を送信元へ返していなかった

## 鍵の配置

鍵は 32 byte のランダム値を 64 桁 hex で保存する。内容をログや記事へ載せない。

```bash
umask 077
openssl rand -hex 32 > /tmp/critical_link.key
sudo install -d -m 710 -o root -g <runtime-user> /etc/njord
sudo install -m 640 -o root -g <runtime-user> \
  /tmp/critical_link.key /etc/njord/critical_link.key
rm /tmp/critical_link.key
```

Ground PC と miniPC には同じ鍵を配置する。表示せずに確認するなら hash を比較する。

```bash
sudo sha256sum /etc/njord/critical_link.key
```

## 動作確認

まず sender と receiver を起動する。

```bash
# Ground PC
ros2 launch critical_link ground_sender.launch.py

# miniPC
ros2 launch critical_link vessel_receiver.launch.py
```

Ground PC から dry-run を送る。

```bash
ros2 run mission_manager njord-critical task check move_to_exam_field
```

期待する結果は、Ground PC に `OperatorResponse` が返り、miniPC の Mission Manager が `/fromLL` の有無を含めて dry-run を評価すること。

## 現在の検証状況

UDP frame の受信、HMAC 検証、固定 command codec、返信処理はローカルビルドまで確認した。実機2台での request/response 往復は、両端へ同じ commit を反映して sender/receiver を再起動してから確認する。

## まとめ

異なる ROS 2 distribution 間で Service/Action が不安定なら、topic に置き換えるだけでは足りない。ROS middleware のシリアライズも distribution 依存になり得る。

操作対象を小さな固定 wire format に限定し、船側で既存 API を呼ぶ dispatcher に集約すると、transport の問題をミッション実装へ持ち込まずに済む。
