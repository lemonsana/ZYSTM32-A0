#!/usr/bin/env bash
set -uo pipefail
cd "C:/stm32&c51/ZYSTM32-A0" || exit 1
git add -A
git commit -q -m "docs: 新增 K210 视觉避障可行性评估与集成方案" -m "
- 架构：K210 做感知节点，STM32 保持 100Hz 控制环，USART2(PA2/PA3) 通信
- 协议：6 字节定长帧（标志/距离/方向/CRC），200ms 超时自动降级
- 算法选型：色块 / 地面分割 / YOLO-tiny / AprilTag 对比，含单目测距方法
- 延迟预算：50~70ms，0.5m/s 下约 3cm，安全
- 集成：IRAvoid 增加 AVOID_SRC 宏，视觉方向动态、状态机复用
- 风险清单与三阶段实施计划"
GIT_TERMINAL_PROMPT=0 timeout 120 git -c http.proxy=http://127.0.0.1:7897 push origin main 2>&1 | tail -2
echo "=== log ==="; git log --oneline -2; git status --short
