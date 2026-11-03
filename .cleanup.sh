#!/usr/bin/env bash
set -uo pipefail
cd "C:/stm32&c51/ZYSTM32-A0" || exit 1
git rm -q --cached .commit_plan.sh 2>/dev/null
rm -f .commit_plan.sh
git add -A
git commit -q -m "chore: 移除临时脚本" 2>/dev/null
git push -q origin main 2>&1 | tail -2
echo '--- status ---'; git status --short; echo '--- log ---'; git log --oneline -3; echo '--- tree ---'; git ls-files | wc -l
