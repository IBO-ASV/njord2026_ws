# GitHub operation policy

Never perform a GitHub write operation unless the destination repository is owned by `IBO-ASV` or `oswystk15662`.

Prohibited destinations include every repository owned by anyone else. Do not push, open or modify pull requests or issues, post comments or reviews, create releases or tags, dispatch workflows, or change repository settings there. A pull request to an external upstream is prohibited even when its head branch is hosted in an allowed `IBO-ASV` or `oswystk15662` fork.

Before any GitHub write, verify the actual repository owner from the remote URL or GitHub API. Do not infer authorization from remote names such as `origin` or `upstream`. If ownership is unknown or not allowed, stop at local changes or push only to a fork/repository owned by an allowed owner.

Read-only operations such as clone, fetch, browsing, and search are allowed.
