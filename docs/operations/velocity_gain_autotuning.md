# 速度ゲイン自動調整の使い方

この手順では、AGX上の調整器を起動し、FoxgloveのMission Controlから前後速度のFF・P・Dゲインを調整します。**開始ボタンを押すと船が動きます。** ここに書かれた手順は操作担当者が試験条件を決めて実行してください。

## 起動

AGXの新しい作業場所は `/home/ibo/njord2026_ws_autotune`、ブランチは `feat/velocity-autotuning-clean` です。古い作業場所から起動したbringupと二重に動かさないでください。

1. AGXの端末で、通常のAGX bringupを新しい作業場所から起動します。

   ```bash
   cd /home/ibo/njord2026_ws_autotune
   scripts/operations/run_agx_pair.sh vessel
   ```

2. AGXの別端末で、待機状態の速度調整器を起動します。起動しただけでは速度指令は出ません。

   ```bash
   cd /home/ibo/njord2026_ws_autotune
   scripts/operations/run_agx_velocity_tuner.sh
   ```

3. 新しいMission Control拡張をFoxgloveへ読み込みます。Windows PowerShellでAGXからファイルをコピーし、Foxgloveへドラッグしてください。

   ```powershell
   scp ibo@100.73.137.72:/home/ibo/njord2026_ws_autotune/src/gui/foxglove_extensions/mission_control/mission-control-0.1.0.foxe "$env:USERPROFILE\Downloads\"
   ```

   FoxgloveはAGXの通常のFoxglove接続先へ接続してください。Mission Controlに「速度ゲイン調整」が表示され、状態が`IDLE`になれば準備できています。

## 試験

1. 目標速度、速度上限、スラスター1基あたりの推力上限を入力します。速度上限は目標速度以上にします。数値は試験場所と船に合わせて担当者が決めてください。
2. `/control/state`相当の表示で、AUTO要求・AUTO許可・非常停止状態を確認します。AUTO許可がない場合は開始できません。条件を迂回せず、表示された安全上の理由を解消してください。
3. **「自動調整を開始」**を押します。調整器はFF、P、Dの順に候補を1つずつ試します。各試行の終了後は調整指令をゼロにして`WAITING_NEXT`で待機し、自動では次へ進みません。
4. 試行間で船を止めたり位置を直したりする場合は、**「MANUALへ切替」**を押すか、操作者が手動コントローラーでMANUALに切り替えます。調整指令はゼロです。手動操作が終わったらGUIの**「次の試行を実行」**を押します。GUIがAUTO要求と許可を確認してから、次の候補を1試行だけ開始します。試行中に一時停止した場合は`PAUSED`となるため、**「AUTOで再開」**で中断した候補を最初からやり直します。
5. 試験を終えるときは**「終了して設定を復元」**を押します。MANUALに切り替えてから終了処理を行います。

画面では調整状態、実測前後速度、推力`[FR, FL, RR, RL]`を確認します。オドメトリや推力の表示がSTALE、速度が上限超過、非常停止などで調整器が`PAUSED`になった場合は、状態を確認してから手動へ切り替え、終了操作をしてください。

## 計測時間の設定\n\n既定では、FF試行は開始前に3秒停止し、目標速度を5秒間出して安定を待った後、その次の5秒間の平均速度を評価します。1候補あたり計13秒です。P/D試行は開始前3秒停止し、目標速度への応答を5秒間計測します。1候補あたり計8秒です。\n\n時間を変更する場所は`src/control/velocity_autotuner/config/velocity_autotuner.yaml`です。launchがインストール済みYAMLを明示して起動するため、launch経由ではこの設定値が使われます。AGXで編集後にパッケージを再ビルドし、調整器を再起動してください。FFの評価区間は安定待ち終了後の評価窓を使います。\n\n## 探索範囲の設定

FF・P・Dそれぞれの候補範囲は、`src/control/velocity_autotuner/config/velocity_autotuner.yaml`で設定します。`feedforward_bounds`、`p_bounds`、`d_bounds`は`[下限, 上限]`の形式で、既定値はそれぞれ`[0.0, 80.0]`です。例としてFFの範囲を10〜50にする場合は、次のように変更します。

```yaml
feedforward_bounds: [10.0, 50.0]
```

探索回数は`search_rounds`で設定し、既定値は4回です。これは両端を試した後に中間候補で範囲を絞る回数です。FFは両端の速度誤差が目標を挟む場合に二分探索し、挟まない場合は中間候補を追加しません。P/Dは評価スコアに基づいて探索範囲を絞ります。値を変更したら、AGXでパッケージを再ビルドし、調整器を再起動してください。

## 結果と範囲

終了時は選定ゲインの反映が確認できてから`COMPLETE`になります。設定の復元に失敗した場合は`PAUSED`のまま理由を表示します。MANUALを維持し、原因を解消してから終了操作を再試行してください。パラメータ通信が処理中の場合も、終了操作は応答完了後に再試行してください。

結果はAGXの `~/njord_tuning` に保存されます。CSVは各候補の評価、YAMLは選ばれたFF/P/D値です。完了した調整は選定したFF/P/Dをコントローラに残し、推力上限とIゲインを元に戻します。途中で終了した場合は元のFF/P/Dも復元します。YAMLを標準設定へ自動反映する処理はありません。

この調整器が扱うのは前後速度だけです。横移動・旋回、スラスター個別の推力比率は調整しません。現在までに確認したのはビルド、画面操作テスト、実機を使わないROS模擬試験です。実船での性能確認はまだ行っていません。
