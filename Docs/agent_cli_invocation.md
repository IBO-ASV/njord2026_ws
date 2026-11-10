# エージェント呼び出しメモ (orchestrator用)

CLAUDE.md のルーティング規則を実際のCLIに落とし込んだもの。
調査日: 2026-08-02 / codex CLI 0.146.0

## Codex CLI

プロファイルは未設定 (`~/.codex/*.config.toml` は存在しない)。
`~/.codex/config.toml` の既定は `model = "gpt-5.6-luna"`, `model_reasoning_effort = "low"`。
そのため担当分けは `-m <slug>` と `-c model_reasoning_effort=<level>` で行う。

利用可能な slug:

| 担当 | slug | 用途 (CLAUDE.md) |
|---|---|---|
| Luna | `gpt-5.6-luna` | 1〜2ファイルの軽微な修正、明確なバグ修正、テスト追加、定型生成 |
| Terra | `gpt-5.6-terra` | 通常規模の機能実装、数ファイルにまたがる変更 |
| Sol | `gpt-5.6-sol` | レビュー、難アルゴリズム、並行処理、原因不明の不具合、設計判断 |

reasoning effort: `low` / `medium` / `high` / `xhigh` / `max` / `ultra`

### 非対話実行の基本形

```bash
codex exec \
  -m gpt-5.6-terra \
  -c model_reasoning_effort=medium \
  -C /home/gembu/njord2026_ws \
  -s workspace-write \
  "<プロンプト>"
```

主要オプション:

- `exec` … 非対話。`[PROMPT]` を省略すると stdin から読む (長文プロンプトはヒアドキュメントで渡すのが楽)
- `-C, --cd <DIR>` … 作業ルート
- `-s, --sandbox` … `read-only` / `workspace-write` / `danger-full-access`
  - レビュー担当 (Sol) は `read-only` で十分
  - 実装担当 (Luna/Terra) は `workspace-write`
- `--add-dir <DIR>` … 追加で書き込み可能にするディレクトリ
- `-c key=value` … 任意の config 上書き (値は TOML としてパースされる)
- `codex exec resume --last` … 直前セッションの継続
- `codex exec review` … 非対話コードレビュー

長文プロンプトを渡す例:

```bash
codex exec -m gpt-5.6-sol -c model_reasoning_effort=high -C /home/gembu/njord2026_ws -s read-only - <<'EOF'
<プロンプト本文>
EOF
```

## Claude CLI

Sonnet coder はメインClaudeの Agent tool (`subagent_type: sonnet-coder`) で呼ぶのが基本。
文脈を引き継げるのでこちらを優先する。

外部プロセスとして回したい場合:

```bash
claude -p "<プロンプト>" --model sonnet --permission-mode acceptEdits
```

- `-p, --print` … 非対話 (ワンショット)
- `--model` … `sonnet` / `opus` / `haiku` など
- `--permission-mode` … `acceptEdits` / `bypassPermissions` / `plan` / `default`
- `--output-format stream-json` … 機械可読出力
- `-c` / `--continue`, `-r <id>` / `--resume` … セッション継続

Fable escalation は Agent tool の `subagent_type: fable-escalation` (Write/Edit なし = 分析専用)。

## 排他ルール運用

同一 worktree で複数の実装エージェントを同時に走らせない。
並列で別案を作る場合は `git worktree add _worktrees/<name>` を切ってから
各エージェントに `-C` / `cwd` でそのパスを渡す。

実装者とレビュアーは別モデルにする (例: Terra実装 → Sol レビュー)。
