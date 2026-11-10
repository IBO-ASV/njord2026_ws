# ブロッカー: ZED SDK が GPU/CUDA 未検出で起動不可（ホスト側/要sudo）

日時: 2026-07-12 / 記録者: メインClaude（実機起動確認中に発見）

## 事象
`ros2 launch zed2i_driver zed2i.launch.py`(既定=SDKモード)で ZED SDK ノードがクラッシュ。
```
libnvrm_gpu.so: NvRmGpuLibOpen failed, error=4
sl::Mat::alloc : Err [100]: no CUDA-capable device is detected.
[ZED][ERROR] No NVIDIA graphics card detected.
[ZED][ERROR] NO GPU DETECTED in sl::Camera::open()
[FATAL] Failed to open ZED camera: NO GPU DETECTED   → process exit -6
```

## 診断(計測結果)
- `nvidia-smi` / `nvidia-smi -L` も `NvRmGpuLibOpen failed, error=4` → `No devices found`。**userspace から iGPU が全く見えない**。
- `/proc/driver/nvidia/gpus/` が **空**（登録GPUゼロ）。NVIDIA Open Kernel Module r595 はロード済み。
- `/dev/nvgpu/igpu0/` に **`power` ノードのみ**存在。GPU計算ノード（channel/ctrl/as/dbg/tsg 等）が無い。
- カーネルモジュール: `nvidia`(open, refcount5) と 旧 `nvgpu`(refcount0) が **両方ロード**（混在/移行不整合の疑い）。
- 環境: bare-metal Jetson Orin Nano Super、L4T **R39 (JetPack 7 相当)**, CUDA **13.2**, ZED SDK **5.4.0**。コンテナではない(systemd PID1)。uptime 8.5h。
- 関連worktree存在: `zed-sdk-jetpack-installer`, `zed2i-cuda-auto` → 同種のGPU/CUDA/JetPack整備が進行中の可能性。

## 判定
- **ドライバ package のコード/launch は正常**（back_cam は同手順で正常配信、SDKノードのビルド・パラメータ・トピック生成も正常）。
- ボトルネックは **ホストの GPU ドライバが iGPU を登録できていない**こと。ROS/コード側では解決不能。**要sudo/ホスト作業**。
- ユーザ仮説「本体の設定が悪い」は **USB ではなく GPU ドライバ**に該当。

## CPUモードも即戦力にならない(補足)
`mode:=cpu`(zed2i_cpu_node)は `/dev/video0` は開けるが `/dev/video1` の open に失敗:
`CPU stereo mode needs both left_device and right_device to be readable`。
理由: ZED 2i はステレオ対を**単一UVCデバイス(左右横並び1フレーム)**として公開し、左右別デバイス前提の cpu_stereo_node と不一致。
→ CPUフォールバックをZEDで使うには単一デバイスを開いてフレーム分割する node 改修が必要(別タスク)。当面は SDK モード復旧が本線。

## ユーザ向け sudo リスト(実行して出力を共有 → メインClaudeが次手を判断)
### 手順1: まずGPU登録失敗の理由を採取(診断)
```
sudo dmesg | grep -iE "nvgpu|nvidia|nvrm|gsp|firmware|igpu|gpu" | tail -80
```
### 手順2: 一番安いリセットを試す(移行時のモジュール混在解消を期待)
```
sudo reboot
```
再起動後、sudo無しで以下を確認し結果共有:
```
nvidia-smi -L
ls -l /dev/nvgpu/igpu0/
cat /proc/driver/nvidia/gpus/*/information 2>/dev/null || echo "still no GPU registered"
```
### 手順3: 手順2で復旧しない場合(dmesgの出力を見て判断)
- iGPU が `nvidia`(open) と `nvgpu`(legacy) のどちらで駆動されるべきかを確定し、正しい L4T GPU ドライバ package 構成へ是正(JetPack 7 のドキュメント/`zed-sdk-jetpack-installer` worktree の手順に依存)。具体是正コマンドは手順1の dmesg 結果を見てから提示。

## 確定根本原因 (2026-07-12, dmesg採取 + Codex Sol 相談)

kernel ログ(journalctl -k, **毎ブート決定的に再現**)で原因が確定:
```
nvgpu: 17000000.gpu ga10b_acr_patch_wpr_info_to_ucode: invalid mem acr_falcon2_sysmem_desc
nvgpu: 17000000.gpu nvgpu_acr_bootstrap_hs_ucode_riscv: RISCV ucode patch wpr info failed
nvgpu: 17000000.gpu ga10b_bootstrap_hs_acr / nvgpu_acr_bootstrap_hs_acr: ACR bootstrap failed
nvgpu: 17000000.gpu nvgpu_acr_construct_execute: Bootstrap HS ACR failed
nvgpu: 17000000.gpu nvgpu_finalize_poweron: Failed initialization for: g->ops.acr.acr_construct_execute
```
= **iGPU(ga10b/Orin)のセキュア初期化 firmware(ACR HS/RISCV ucode)のブートストラップ失敗**。
GPUが poweron を finalize できず、計算ノード生成・CUDA登録に至らない → `NO GPU DETECTED`。

原因層 = **ZED/CUDA/ROS ユーザ空間ではなく、kernel driver ↔ nvgpu firmware ↔ OpenRM/legacy nvgpu ↔ DTB/boot の整合不良**(GPU初期化層)。
BSPは dpkg 上すべて 39.2.0 で揃うが、実行 kernel `6.8.12-1021-tegra`(oot variant)と module/firmware の実体整合が崩れている疑い。

## Codex Sol 判定 (gpt-5.5, read-only)
- **再起動だけでは直らない**(毎ブート決定的に失敗)。ただし是正後の反映には reboot 必須。
- **非sudo/コード側でできる対処は無い**。ZED/CUDA/ROS を触っても無意味。**打ち切り妥当**。
- 是正方向(リスク低→高, すべて要sudo):
  1. boot実体確認: `uname -a`, `cat /etc/nv_tegra_release`, `cat /boot/extlinux/extlinux.conf`, `ls -l /boot/Image /boot/initrd /boot/*.dtb`, `lsmod | grep -E 'nvidia|nvgpu'`
  2. module 提供元確認: `modinfo nvidia; modinfo nvgpu; dpkg -S $(modinfo -n nvidia); dpkg -S $(modinfo -n nvgpu)`
  3. L4T 再導入 + initramfs + reboot:
     `sudo apt install --reinstall nvidia-l4t-core nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-firmware nvidia-l4t-firmware-nvgpu nvidia-l4t-firmware-openrm && sudo update-initramfs -u -k all && sudo reboot`
  4. firmware path 確認: cmdline に `firmware_class.path=/etc/firmware` あり → `ls -l /etc/firmware`, `ls -l /lib/firmware/nvidia/ga10b`, `journalctl -k -b | grep -i firmware`
  5. 最終手段: JetPack 7 / L4T R39.2 **reflash**(破壊的, rootfsバックアップ後)。
- 是正後に `/proc/driver/nvidia/gpus/` と `nvidia-smi` が復活してから ZED 作業へ戻る。

## 追加診断 (2026-07-12, reinstall試行後)
- `update-initramfs` はL4Tに存在せず → 正しくは `sudo nv-update-initrd`(`nvidia-l4t-initrd`提供)。`/boot/initrd` は reinstall で自動再生成済(mtime 当日12:47)。
- kernel `6.8.12-1021-tegra` と `/lib/modules/6.8.12-1021-tegra` は**整合(矛盾なし)**。nvgpu module は running kernel に合致してロード済。
- `apt install --reinstall nvidia-l4t-firmware-nvgpu` は **同一 39.2.0 firmware を戻すのみ**(`/lib/firmware/nvidia/ga10b/acr-*` 内容不変, Jun2)。→ **再起動しても同じACRエラーの見込み**。
- cmdline `firmware_class.path=/etc/firmware` だが **`/etc/firmware` 不在** → `/lib/firmware` に正常フォールバック(firmwareは発見できている=「未検出」ではない)。
- 従い ACR `invalid mem acr_falcon2_sysmem_desc` は **ブートローダ(MB2/UEFI)側 GPU carveout/WPR 記述子の不正** = **ブート/フラッシュ層**の問題。rootfs(apt)では是正不可。
- **本命の是正 = JetPack 7 / L4T R39.2 の再フラッシュ**(破壊的, rootfsバックアップ必須)。apt reinstall/reboot は確認ステップに留まる。

## 解決 (2026-07-12 夕方) — GPU復旧 + calib配置でZED実配信成功
1. **GPU復旧**: ユーザが `sudo apt install --reinstall nvidia-l4t-{core,kernel,kernel-dtbs,firmware,firmware-nvgpu,firmware-openrm}` → `sudo reboot`。
   ※ `update-initramfs` はL4Tに無く `nv-update-initrd` が正だが、reinstallで /boot/initrd 自動再生成のため手動不要。
   再起動後 `nvidia-smi -L` = `GPU 0: Orin (nvgpu)`、`/dev/nvgpu/igpu0/` に計算ノードフル出現、ACRエラー消滅。
   → 当初「reinstallは同一firmwareで再起動しても直らない」の予想に反し、reinstall+クリーン再起動で解消(過去rebootが非クリーンだった可能性/推測)。
2. **第2ブロッカー=calibファイル欠落**: GPU復旧後 ZED は `Camera successfully opened`(HD720@15, SN34432991)まで到達も、
   `CALIBRATION FILE NOT AVAILABLE`(起動時ネット未接続でDL失敗、EEPROM fallbackも不可)。
   → 正しいDL URL は パラメータ `SN=`(not `serial=`): `https://calib.stereolabs.com/?SN=34432991`。
   `/usr/local/zed/settings/`(root:zed, 要sudo)に `SN34432991.conf` を配置(ユーザ実行)。
   (恒久策: `sudo usermod -aG zed <user>` でSDK自動DL/キャッシュがsudo不要に)。
3. **ZED実配信確認(メインClaude)**: 再起動後 `ros2 launch zed2i_driver zed2i.launch.py` で以下を実測:
   - `/zed2i/left/image_rect` ~8Hz, `/zed2i/right/image_rect` ~7Hz, `/zed2i/depth/image` ~7Hz, `/zed2i/points` ~6Hz, camera_info 左右。
   - ログ: `Camera successfully opened` クリーン。警告2(非致命): depth mode QUALITY 非推奨→NEURAL推奨 / self-calib skip(被写体テクスチャ不足)。
   - 実測 ~7-8Hz < 設定15fps: Orin Nanoの深度/点群演算スループット。性能面(任意 `nvpmodel -m 0`+`jetson_clocks`)で改善余地。ブロッカーではない。

## ステータス (最終)
- back_cam: **完了(実機配信 ~26-30Hz 確認済み)**。
- ZED: **完全復旧・完了**。GPU(ACR)ブロッカー=ユーザのL4T reinstall+reboot で解消。calib欠落=calibファイル配置で解消。全トピック実配信をメインClaudeが確認。
- 残(任意): depth mode を NEURAL 化、電源 MAXN 化で fps 改善。いずれも必須ではない。
