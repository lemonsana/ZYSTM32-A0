#!/usr/bin/env bash
set -uo pipefail
cd "C:/stm32&c51/ZYSTM32-A0" || exit 1
rm -f .payload.json .create_repo.sh .push.sh .crop.py .crop2.py .ls_sensor.sh .ls_sensor2.sh
rm -rf .crops
git add -A
git status --short
git commit -q -m "docs: 新增 v2 改造方案（电机通道根因、引脚核对、灰度接线、避障改造）" && git log --oneline -2
git tag -a v2.0-plan -m "v2 改造方案（待确认，仅文档）"
GIT_TERMINAL_PROMPT=0 git push origin main --tags 2>&1 | tail -3
