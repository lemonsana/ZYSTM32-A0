#ifndef __VISION_H_
#define __VISION_H_

#include "stm32f10x.h"

/* ============================================================================
 *  K210 视觉模块接口  v4.0
 *  ==========================================================================
 *  接线（K210 → STM32，3.3V TTL，必须共地）：
 *      K210 TX  →  PA3（丝印 IO0）USART2_RX
 *      K210 RX  →  PA2（丝印 IO1）USART2_TX  （可只接 TX 单向）
 *      GND      →  GND
 *      K210 供电：独立 5V（峰值 300~500mA，别和电机共用）
 *
 *  协议（6 字节定长帧）：
 *      AA 55 | FLAGS | DIR | DIST | XOR
 *        FLAGS bit0 = 检测到障碍
 *        DIR   0 = 障碍偏左（→ 往右绕）  1 = 居中  2 = 偏右（→ 往左绕）
 *        DIST  距离 cm，0 = 未知
 *        XOR   前 5 字节异或
 *
 *  安全：超过 VISION_TIMEOUT_MS 没收到有效帧 → 判失效，保持直行不乱转。
 * ========================================================================== */

#define VISION_UART_BAUD        115200
#define VISION_H1               0xAA
#define VISION_H2               0x55
#define VISION_FRAME_PAYLOAD    3       /* FLAGS + DIR + DIST */
#define VISION_TIMEOUT_MS       300     /* 超过这么久没帧 = 失效 */

/* ---- 避障动作参数 ---- */
#define VIS_BASE_SPEED          55      /* 直行速度 */
#define VIS_OUTER_SPEED         90      /* 绕行外侧轮 */
#define VIS_INNER_SOFT          20      /* 普通绕行内侧轮（仍向前） */
#define VIS_INNER_HARD         -30      /* 障碍居中且很近时，内侧轮倒转 */
#define VIS_NEAR_CM             30      /* 距离小于它 → 用 HARD 档 */
#define VIS_HOLD_MS            450      /* 一次绕行至少保持 */
#define VIS_RECOVER_MS         250      /* 绕完反向回正 */
#define VIS_RECOVER_DELTA       25      /* 回正差速量 */

void Vision_Init(void);             /* 初始化 USART2 + 中断 */
void Vision_Task(void);             /* 每 10ms 调一次：维护超时 */
void Vision_AVoidRun(void);         /* 避障任务：每 10ms 调一次 */
void Vision_Reset(void);            /* 清状态 */

u8  Vision_IsValid(void);           /* 1 = 最近收到过有效帧 */
u8  Vision_GetFlags(void);
u8  Vision_GetDir(void);
u8  Vision_GetDist(void);
u8  Vision_GetState(void);          /* 0=直行 1=绕行 2=回正 */
u16 Vision_GetFrameCount(void);     /* 收到的有效帧数 */

#endif
