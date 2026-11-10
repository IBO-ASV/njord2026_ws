# 実装レビュー — カメラ実機bringup (zed2i / back_cam)

日時: 2026-07-12 / 対象: サブタスク2(カメラ device 割当正常化)
実装者: sonnet-coder / レビュアー: Codex Sol (gpt-5.5, read-only) ※実装者≠レビュアー維持

## 結論: **合格**(ブロッカー/必須修正なし)

指定4ファイルを実読し、「ZEDはZED / back_camはAdesso」への修正成立を確認。

## 確認済み
- `back_cam.launch.py`: video_device 既定 = Adesso by-id。Node parameter も同 LaunchConfiguration。取り違えなし。
- `real_bringup.launch.py`: zed2i へ mode=sdk、back_cam へ `{"video_device": device}` 伝播。引数名一致。
- `zed2i_cpu.yaml` / `zed2i_jetson_orin_nano.yaml`: ZED by-id (index0/index1)。Adesso混入なし。
- pixel_format=mjpeg2rgb / 640x480 / 30 / namespace=back_cam / frame_id=back_cam_link は Adesso 対応と整合、起動阻害なし。

## 指摘(いずれも Low、完了判定のブロッカーではない)
- L1: Adesso by-id に `SN0001` を含むため、**同型H7を2台接続すると一意性に不安**。単一back_cam構成なら完了ライン内。将来2台化するなら udev rule で物理ポート/属性ベースの専用symlink化を検討。
- L2: SDKモードでは ZED yaml の left/right_device は未使用(CPU fallback/単体launch向け修正として妥当)。SDK実機bringupの成否は ZED SDK 側 `camera_.open()` 依存。

## 司令塔(メインClaude)独立確認
- 差分を直接確認: 4ファイルとも by-id 値が正しい向き。
- sonnet-coder 報告の build 成功(robot / zed2i_driver)・`--show-args` 既定値一致を確認。
- CAC1〜CAC4 充足。CAC5(実映像通電)は艇上工程。

## 判定
テスト/レビュー失敗ゼロ → Sol/Fable への昇格なし。**サブタスク2 = 実装+レビュー完了**。
コミット可(4ファイル最小差分)。実HW映像確認は艇上工程。
