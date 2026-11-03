#!/usr/bin/env bash
set -uo pipefail
cd "C:/stm32&c51/ZYSTM32-A0" || exit 1
rm -f _prep.sh
git add -A
git status --short
git commit -q -m "feat(v2.0): 修正电机PWM通道交叉+方向脚残留；电机层非阻塞化；避障改为不停车弧线绕行；启用串口调试" -m "- motor.c: 左轮改 CH3/PB8、右轮改 CH4/PB9；速度0也写方向脚；新增 Motor_SetSpeed/Motor_Stop/方向反转宏
- motor.c: 修正 ZYSTM32_back 的 100-speed 语义问题
- IRSEARCH.c: 去掉位与笔误，改 10ms 非阻塞采样
- IRAvoid.c: 停车+后退+左转 改为 差速弧线绕行（方向宏可配）
- main.c: 10ms 节拍调度，uart_init(115200) 输出调试信息
- README/docs: 更新版本记录、引脚表、串口说明"
git tag -a v2.0 -m "v2.0 电机修复 + 非阻塞改造 + 避障新策略（灰度留待 v2.1）"
GIT_TERMINAL_PROMPT=0 git push -q origin main --tags && echo "PUSH OK"
echo "--- log ---"; git log --oneline -3
echo "--- files ---"; git ls-files | wc -l
