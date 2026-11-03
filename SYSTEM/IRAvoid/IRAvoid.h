#ifndef __IRAVOID_H_
#define __IRAVOID_H_

#include "stm32f10x.h"

/* ==================== 避障模块接口（v2） ====================
 *  硬件（板载固定，不要改）：
 *      避障探头  PB1  丝印 IO5
 *  电平： BARRIER_Y = 0  有障碍
 *         BARRIER_N = 1  无障碍
 * ========================================================== */

void IRAvoidInit(void);     /* 初始化避障探头（上拉输入） */
void AVoidRun(void);        /* 避障任务：主循环每 10ms 调一次（不停车、不后退） */

#define AVOID_PIN         GPIO_Pin_1
#define AVOID_PIN_GPIO    GPIOB
#define AVOID_IO          GPIO_ReadInputDataBit(AVOID_PIN_GPIO, AVOID_PIN)

#define BARRIER_Y 0         /* 有障碍物 */
#define BARRIER_N 1         /* 无障碍物 */

#endif
