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
3. **「自動調整を開始」**を押します。調整器はFF、P、Dの順に候補を試し、実測速度と推力を表示します。
4. 手動で位置を直すときは、**「一時停止して手動へ」**を押します。調整指令がゼロになり、MANUALへ切り替わります。手動操作後は**「AUTOで再開」**を押すと、AUTO許可を確認して現在の候補の試行をやり直します。
5. 試験を終えるときは**「終了して設定を復元」**を押します。MANUALに切り替えてから終了処理を行います。

画面では調整状態、実測前後速度、推力`[FR, FL, RR, RL]`を確認します。オドメトリや推力の表示がSTALE、速度が上限超過、非常停止などで調整器が`PAUSED`になった場合は、状態を確認してから手動へ切り替え、終了操作をしてください。

## 結果と範囲

終了時は選定ゲインの反映が確認できてから`COMPLETE`になります。設定の復元に失敗した場合は`PAUSED`のまま理由を表示します。MANUALを維持し、原因を解消してから終了操作を再試行してください。パラメータ通信が処理中の場合も、終了操作は応答完了後に再試行してください。

結果はAGXの `~/njord_tuning` に保存されます。CSVは各候補の評価、YAMLは選ばれたFF/P/D値です。完了した調整は選定したFF/P/Dをコントローラに残し、推力上限とIゲインを元に戻します。途中で終了した場合は元のFF/P/Dも復元します。YAMLを標準設定へ自動反映する処理はありません。

この調整器が扱うのは前後速度だけです。横移動・旋回、スラスター個別の推力比率は調整しません。現在までに確認したのはビルド、画面操作テスト、実機を使わないROS模擬試験です。実船での性能確認はまだ行っていません。
