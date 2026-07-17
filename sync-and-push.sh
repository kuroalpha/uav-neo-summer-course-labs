#!/usr/bin/env bash
# Sync your lab work from the installer tree into this fork clone and push it.
# Edit files in ~/uav-neo-installer/drone-student/labs, then run this.
set -euo pipefail

SRC="/Users/nicholascheng/uav-neo-installer/drone-student/labs"
DST="$(cd "$(dirname "$0")" && pwd)/labs"

rsync -a --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' \
         --exclude='logs/' "$SRC"/ "$DST"/

cd "$(dirname "$0")"
if git diff --quiet && git diff --cached --quiet; then
  echo "Nothing changed — everything already on GitHub."
  exit 0
fi

git add -A
git status --short
git -c user.name="Nicholas Cheng" -c user.email="nicholascheng123456@gmail.com" \
    commit -m "${1:-sync lab work}"
git push origin main
echo "Pushed. https://github.com/kuroalpha/uav-neo-summer-course-labs"
