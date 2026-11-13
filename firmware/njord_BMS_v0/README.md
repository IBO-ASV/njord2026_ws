# 4S Li-Po セル電圧モニター

5台の Seeed Studio XIAO ESP32C3を使用して、4S Li-Poバッテリーの各セル電圧をPCへ出力するファームウェアです。

- セル側XIAO × 4台：各セルをローカルGND基準で測定し、ESP-NOWで送信
- マスターXIAO × 1台：4台分を受信し、USBシリアルへCSV出力

通常のWi-Fiアクセスポイント、Web画面、インターネット接続機能は実装していません。無線通信として使用するのは、セル側とマスター間のESP-NOWだけです。

```text
Cell 1 ─┐
Cell 2 ─┼─ ESP-NOW ─→ Master XIAO ── USB Serial ─→ PC
Cell 3 ─┼─    Ch.1
Cell 4 ─┘
```

## 重要な安全上の注意

> [!WARNING]
> 直列接続されたセルは、それぞれGND電位が異なります。セル側XIAO同士のGNDを接続しないでください。

- バッテリー接続中のセル側XIAOを、複数台同時にPCのUSBへ接続しないでください。
- 書き込みやデバッグを行うときは、対象基板をバッテリーから切り離すか、適切な絶縁機器を使用してください。
- セル電圧は必ず抵抗分圧し、ADC入力の許容電圧を超えないようにしてください。
- 本ファームウェアは電圧の監視専用です。過充電、過放電、過電流、短絡などの保護は、独立した保護回路で行ってください。
- 基板を各セルから常時給電すると、その消費電流がセルバランスに影響します。

## 現在の仕様

| 項目 | 設定 |
|---|---|
| 対象ボード | XIAO ESP32C3 |
| セル数 | 4セル |
| ADC入力 | `A0` / `D0` / `GPIO2`（すべて同じ端子） |
| 温湿度センサー | DHT20（Cell 1のみ） |
| DHT20接続 | SDA=`D4`/GPIO6、SCL=`D5`/GPIO7、I²Cアドレス=`0x38` |
| DHT20測定周期 | 2秒 |
| 分圧抵抗 | 上側100 kΩ、下側100 kΩ |
| ADC平均回数 | 32回 |
| 送信周期 | 250 ms |
| 通信方式 | ESP-NOWブロードキャスト |
| ESP-NOWチャネル | 1 |
| 暗号化 | なし |
| マスター出力周期 | 1秒 |
| USBシリアル | 115200 bps、CSV形式 |
| 通信途絶判定 | 最後の受信から2秒 |

100 kΩ＋100 kΩの分圧では、4.2 Vのセル電圧がADC端子で約2.1 Vになります。実際の基板で抵抗値が異なる場合は、ファームウェアの設定も必ず変更してください。

## プロジェクト構成

```text
BMS/
├─ cell_node_1/       Cell 1用ファームウェア
├─ cell_node_2/       Cell 2用ファームウェア
├─ cell_node_3/       Cell 3用ファームウェア
├─ cell_node_4/       Cell 4用ファームウェア
├─ master/            マスター用ファームウェア
├─ common/            共通の測定処理と通信パケット定義
└─ monitor/           Windows用Python監視アプリ
```

セル側4本の違いは、先頭で指定している`CELL_ID`です。

## セル側の設定

各`cell_node_N/cell_node_N.ino`の先頭に設定があります。

```cpp
#define CELL_ID 1
#define ADC_PIN D0
#define R_TOP_OHM 100000.0F
#define R_BOTTOM_OHM 100000.0F
#define CALIBRATION_GAIN 1.0F
#define CALIBRATION_OFFSET_V 0.0F
#define ESPNOW_CHANNEL 1
#define ENABLE_DHT20
#define DHT20_SDA_PIN D4
#define DHT20_SCL_PIN D5
```

| 設定 | 内容 |
|---|---|
| `CELL_ID` | セル番号。1～4で重複させない |
| `ADC_PIN` | 分圧回路を接続したADC端子 |
| `R_TOP_OHM` | セル＋側からADCまでの抵抗値 |
| `R_BOTTOM_OHM` | ADCから対象セルのローカルGNDまでの抵抗値 |
| `CALIBRATION_GAIN` | 測定値の倍率補正 |
| `CALIBRATION_OFFSET_V` | 測定値のオフセット補正（V） |
| `ESPNOW_CHANNEL` | ESP-NOWチャネル。5台すべて同じ値にする |
| `ENABLE_DHT20` | DHT20を使用する。Cell 1だけで定義 |
| `DHT20_SDA_PIN` | DHT20のSDA端子。現在は`D4`/GPIO6 |
| `DHT20_SCL_PIN` | DHT20のSCL端子。現在は`D5`/GPIO7 |

XIAO ESP32C3のボード定義では、`A0`、`D0`、`GPIO2`は同じ端子です。現在の`ADC_PIN D0`はA0端子を読み取ります。

### Cell 1のDHT20配線

| DHT20 | XIAO ESP32C3 |
|---|---|
| VCC | 3V3 |
| GND | Cell 1基板のローカルGND |
| SDA | D4 / GPIO6 |
| SCL | D5 / GPIO7 |

DHT20モジュール側にI²Cプルアップ抵抗がない場合は、SDAとSCLをそれぞれ3.3 Vへ適切な抵抗でプルアップしてください。

## マスターのシリアル出力

マスターを115200 bpsで開くと、起動時にヘッダーを表示し、その後1秒ごとに測定結果を出力します。

```csv
ms,cell1_V,cell2_V,cell3_V,cell4_V,total_V,cell1_temp_C,cell1_humidity_pct,age1_ms,age2_ms,age3_ms,age4_ms,status
1234,4.1010,4.0870,4.0930,4.0990,16.3800,24.5000,48.2500,41,38,44,40,OK
```

| 列 | 内容 |
|---|---|
| `ms` | マスター起動後の経過時間（ms） |
| `cell1_V`～`cell4_V` | 各セルの電圧（V） |
| `total_V` | 4セルの合計電圧（V） |
| `cell1_temp_C` | Cell 1に接続したDHT20の温度（°C） |
| `cell1_humidity_pct` | Cell 1に接続したDHT20の相対湿度（%RH） |
| `age1_ms`～`age4_ms` | 各セルを最後に受信してからの時間（ms） |
| `status` | 全体の受信・測定状態 |

ステータスの意味：

- `OK`：4セルすべて正常
- `STALE`：未受信、または2秒以上受信していないセルがある
- `ADC_FAULT`：ADC入力が上限付近まで飽和したセルがある
- `DHT20_FAULT`：DHT20が未接続、応答なし、またはCRC異常

異常なセル値は`nan`になります。`total_V`は4セルすべて正常な場合だけ出力され、それ以外は`nan`になります。

## Python監視アプリ

`monitor/bms_monitor.py`は、マスターのUSBシリアル出力をWindows上で表示する監視アプリです。

- 4セルの電圧、合計電圧、Cell 1の温度・湿度を表示
- 正常、低電圧、通信途絶、ADC異常を色分け
- セル電圧が設定値未満になると警告音を3秒間隔で再生
- 警告しきい値は画面から変更可能（初期値3.70 V）
- 警告音の消音に対応

Python 3をインストールした状態で、プロジェクトのルートから次を実行します。

```powershell
python -m pip install -r monitor\requirements.txt
python monitor\bms_monitor.py
```

アプリを起動したら、マスターXIAOのCOMポートを選択して「接続」を押します。Arduino IDEのシリアルモニターなど、同じCOMポートを使用するアプリは先に閉じてください。

警告音は有効な測定値がしきい値を下回った場合に鳴ります。`STALE`やシリアル通信途絶は画面で警告しますが、低電圧と区別するため音は鳴らしません。このアプリは監視補助であり、バッテリーを自動的に遮断する安全装置ではありません。

## ビルドと書き込み

### Arduino IDEを使用する場合

1. Arduino IDEへEspressifの`esp32`ボードパッケージを導入します。
2. ボードとして`XIAO_ESP32C3`を選択します。
3. 対象の`.ino`ファイルを開きます。
4. セル側は、基板のセル番号に対応したファームウェアを書き込みます。
5. マスターには`master/master.ino`を書き込みます。

### Arduino CLIを使用する場合

セル1の例：

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 cell_node_1
arduino-cli upload -p COM3 --fqbn esp32:esp32:XIAO_ESP32C3 cell_node_1
```

マスターの例：

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 master
arduino-cli upload -p COM3 --fqbn esp32:esp32:XIAO_ESP32C3 master
```

`COM3`は実際に表示されるポート名へ変更してください。

## 初回の動作確認

1. バッテリーから切り離した状態で、5台それぞれに最新版のファームウェアを書き込みます。通信パケットが更新されたため、旧版と混在させないでください。
2. マスターだけをPCへUSB接続し、115200 bpsでシリアルモニターを開きます。
3. セル側を1台ずつ安全に接続します。
4. 対応する`cellN_V`が表示されることを確認します。
5. テスターの実測値と比較し、必要ならセルごとに校正します。
6. Cell 1の温度と湿度が表示されることを確認します。
7. 4台すべてを接続し、`status`が`OK`になることを確認します。

セル側XIAOのUSBシリアルには、起動時のMACアドレス、ADC値、計算したセル電圧、送信状態が表示されます。セル側をUSBで確認する場合は、必ず安全上の注意に従ってください。

## 電圧校正

まず`CALIBRATION_OFFSET_V`を`0.0F`のままにして、倍率を調整します。

```text
CALIBRATION_GAIN = テスター実測電圧 ÷ ファームウェア表示電圧
```

例：テスターが4.100 V、表示が4.050 Vの場合：

```text
4.100 ÷ 4.050 = 1.01235
```

```cpp
#define CALIBRATION_GAIN 1.01235F
```

複数の電圧で確認し、一定量の差が残る場合にだけ`CALIBRATION_OFFSET_V`も調整してください。

## 通信について

- セル側はESP-NOWブロードキャストで送信するため、マスターのMACアドレス登録は不要です。
- マスターはパケット識別子、プロトコルバージョン、セル番号、データ長、電圧範囲を確認してから採用します。
- ESP-NOWと通常のWi-Fiアクセスポイントは別機能です。本仕様ではアクセスポイント機能を使用しません。
- ブロードキャストパケットは暗号化していません。充放電制御などの安全機能には使用しないでください。

## ビルド確認済み環境

- Arduino CLI 1.5.2-rc.1
- Espressif ESP32 Arduino Core 3.3.11
- ボード：`esp32:esp32:XIAO_ESP32C3`

セル側4本とマスターの合計5本について、XIAO ESP32C3向けのコンパイルを確認済みです。実機でのADC校正、DHT20の動作、無線到達距離の確認は別途必要です。
