#!/usr/bin/env bash
set -euo pipefail

[[ $# -ge 1 ]] || { echo "Usage: $0 <commit_message>"; exit 1; }
msg="$1"

# detect branch
branch=$(git rev-parse --abbrev-ref HEAD)

# stage & commit only if there are staged changes
git add -A
if ! git diff --cached --quiet; then
  git commit -m "$msg"
else
  echo "No changes to commit."
fi

# ensure remote exists
git remote show origin >/dev/null 2>&1 || { echo "No remote 'origin'."; exit 1; }

# sync with remote and rebase to keep history clean
git fetch origin
# set upstream if missing
if ! git rev-parse --verify --quiet "origin/$branch" >/dev/null; then
  git push -u origin "$branch"
else
  git pull --rebase origin "$branch"
  git push origin "$branch"
fi

echo "Push succeeded to origin/$branch."

