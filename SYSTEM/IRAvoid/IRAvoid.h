#ifndef __IRAVOID_H_
#define __IRAVOID_H_

#include "stm32f10x.h"

/* ============================================================================
 *  避障模块接口  v3.4
 *  --------------------------------------------------------------------------
 *  硬件：避障探头 PB1（丝印 IO5），数字量输出，有障碍 = 低电平
 *        （模块上的电位器决定检测距离，程序不依赖具体距离）
 *
 *  v1 原厂：停车 300ms → 后退 400ms → 左转 500ms（全程阻塞 1.2s）
 *  v2     ：不停车直接弧线，但一触发就固定 400ms，障碍消失立刻回直行
 *  v3.4   ：状态机 + 去抖 + 两级转弯 + 回正补偿（详见 IRAvoid.c 注释）
 * ========================================================================== */

void IRAvoidInit(void);     /* 初始化避障探头（上拉输入，PB1） */
void AVoidRun(void);        /* 避障任务：主循环每 AVOID_TICK_MS 调一次 */

/* ---- 探头引脚（板载固定，不要改） ---- */
#define AVOID_PIN         GPIO_Pin_1
#define AVOID_PIN_GPIO    GPIOB
#define AVOID_IO          GPIO_ReadInputDataBit(AVOID_PIN_GPIO, AVOID_PIN)

#define BARRIER_Y 0         /* 有障碍物（探头输出低） */
#define BARRIER_N 1         /* 无障碍物（探头输出高） */

/* ---- 运行状态（串口调试打印用） ---- */
u8   AVoid_GetState(void);      /* 0=直行 1=转弯 2=回正 */

#endif
