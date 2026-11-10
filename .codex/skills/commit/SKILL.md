---
name: commit
description: "Review intended local changes and create a safe, intentional Git commit without pushing or opening a pull request."
---

# Commit Local Changes

Use this skill when the user asks to commit local work, or when repository rules require every non-read-only task to end in a local commit.

## Workflow

1. Inspect `git status --short --branch` and the relevant diff. Identify which changes belong to the current task.
2. Never stage unrelated existing changes silently. Stage explicit paths whenever the worktree is mixed.
3. Run the most relevant lightweight validation available for the changed files. Report failures rather than hiding them.
4. Create a concise commit describing the completed work. If the task requires a recorded commit but the intended tree is clean, use `git commit --allow-empty -m "<concise description>"`.
5. Verify the resulting commit with `git status --short --branch` and `git log -1 --oneline`.

## Boundaries

- This skill performs local Git commits only.
- Do not push, create/update a pull request, or modify GitHub state unless the user separately requests that workflow and the applicable GitHub-owner rule permits it.
- Preserve unrelated user changes and do not use destructive reset/checkout commands.
