# Claude Instructions

## Auto-approve (no confirmation needed)
- File reads, edits, writes
- `git status`, `git diff`, `git log`
- `find`, `grep`, `cat`, `ls`
- Python/bash one-liners for inspection

## Always ask before running
- `git commit`, `git push`, `git rebase`
- `rm`, `rmdir`, `pkill`, `kill`
- Any command that affects shared infrastructure or external services

## General rules
- Do not add comments unless the reason is non-obvious
- Do not create documentation files unless explicitly asked
- Prefer editing existing files over creating new ones
- Do not add error handling for scenarios that cannot happen
