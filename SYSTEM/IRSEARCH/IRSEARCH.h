#ifndef __IRSEARCH_H_
#define __IRSEARCH_H_

#include "stm32f10x.h"

/* ==================== 循迹模块接口（v2） ====================
 *  硬件（板载固定，不要改）：
 *      右循迹探头  PA7  丝印 IO3
 *      左循迹探头  PB0  丝印 IO4
 *  电平： BLACK_AREA = 1（探头压到黑线）
 *         WHITE_AREA = 0（白底）
 * ========================================================== */

void IRSearchInit(void);    /* 初始化两路探头（上拉输入） */
void SearchRun(void);       /* 循迹任务：主循环每 10ms 调一次 */

#define SEARCH_R_PIN         GPIO_Pin_7
#define SEARCH_R_GPIO        GPIOA
#define SEARCH_R_IO          GPIO_ReadInputDataBit(SEARCH_R_GPIO, SEARCH_R_PIN)

#define SEARCH_L_PIN         GPIO_Pin_0
#define SEARCH_L_GPIO        GPIOB
#define SEARCH_L_IO          GPIO_ReadInputDataBit(SEARCH_L_GPIO, SEARCH_L_PIN)

#define BLACK_AREA 1        /* 压黑线 */
#define WHITE_AREA 0        /* 白底 */

#endif
