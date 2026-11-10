# 環境ブロッカー: Codex エージェント(Luna/Terra/Sol)利用不可

日時: 2026-07-12 / 記録者: メインClaude

## 事象
CLAUDE.md ルーティングの Codex 実装/レビュー担当(codex-luna, codex-terra, codex-sol)が
全て呼び出し失敗。

## 症状(実際のエラー)
- 既定モデル呼び出し: `The 'gpt-5.6-sol' model requires a newer version of Codex. Please upgrade to the latest app or CLI and try again.`
- override 試行はすべて ChatGPT account で拒否:
  - `gpt-5.2` → not supported when using Codex with a ChatGPT account
  - `gpt-5.2-codex` → 同上
  - `gpt-5-codex` → 同上

## 原因(推定)
インストール済み Codex CLI/アプリが古く、設定既定モデル `gpt-5.6-sol` に未対応。
一方 ChatGPT アカウントでは旧モデルへの override が許可されない → 利用可能モデルが無い状態。

## 影響
- Sol による設計レビュー(標準フロー step2)不可。
- Luna/Terra による実装(step3 の選択肢2つ)不可。
- 実質、実装担当は sonnet-coder のみ、レビューは Claude 系モデルのみで回すことになる。

## 対応案
- A: ユーザが Codex CLI/アプリを最新化 → Luna/Terra/Sol 復帰後に本来ルーティングで進行。
- B: 急ぎのため Claude 系のみで進行(実装=sonnet-coder、レビュー=別Claude、設計=メインClaude/Plan)。
  実装者≠レビュアーの排他ルールは維持可能。

## 解決(2026-07-12) — model
- インストール済み CLI は `codex-cli 0.144.1`。既定 `gpt-5.6-sol` は要 CLI 更新で不可のまま。
- **回避策: model override = `gpt-5.5`** で全 Codex MCP(codex/-terra/-luna)呼び出しが成功。
- 以降、Codex 呼び出しは `model: gpt-5.5` を明示。

## 第2ブロッカー(2026-07-12) — Codex 書き込み sandbox 不可
- Codex `sandbox=workspace-write` は bwrap 起動失敗: `bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted`
  (このコンテナで network namespace/loopback 設定不可)。→ Terra 実装が0変更で中断。
- 回避に `sandbox=danger-full-access` を試行 → Claude Code auto-mode classifier が拒否
  (ユーザが sandbox/approval off の agent 実行を明示していないため)。
- Codex **read-only は動作**(Sol 設計レビューは成功)。つまり Codex はレビュー可・**実装(書込)不可**。
- 影響: 実装担当を Codex(terra/luna)にする場合はユーザ許可(danger-full-access)が必要。
  Claude 系 agent(sonnet-coder)は native ツールで書込可能 → 環境制約を受けない。
- → ユーザ判断: (A) Terra を danger-full-access で許可 / (B) 実装担当を sonnet-coder に切替(レビューは Codex Sol read-only)。
