#include "IRAvoid.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "stm32f10x.h"

/* ==================== 红外避障模块（v2 非阻塞弧线避让） ====================
 *
 *  硬件（板载固定）：
 *    避障探头  PB1 (丝印 IO5)，有障碍时输出低电平（BARRIER_Y = 0）
 *
 *  v1 的行为（已废弃）：
 *    有障碍 → 停车 300ms → 后退 400ms → 左转 500ms
 *    问题：全程阻塞 1.2 秒，动作期间看不见前方；每次都要停车后退，很笨。
 *
 *  v2 的行为（本文件）：
 *    检测到障碍 → 不停车、不后退，直接让"内侧轮慢、外侧轮快"，车沿弧线绕过去；
 *    避让动作至少保持 AVOID_HOLD_MS 毫秒（防止刚转一点又检测到而左右抖动）；
 *    保持时间内如果还能看到障碍，会自动续期，转弯更狠；
 *    障碍消失且保持时间结束 → 恢复直行。
 *
 *  方向：AVOID_TURN_DIR  = -1 往左绕（默认）  +1 往右绕
 *        改这一个数字就能换方向，不用改接线。
 * ==================================================================== */

#define AVOID_BASE_SPEED      60    /* 直行速度 */
#define AVOID_OUTER_SPEED     90    /* 绕行时外侧轮（快） */
#define AVOID_INNER_SPEED     15    /* 绕行时内侧轮（慢，只要 >0 就是"边走边转"，不停车） */
#define AVOID_HOLD_MS        400    /* 一次避让至少保持的时间（ms） */
#define AVOID_TURN_DIR        -1    /* -1 = 往左绕；+1 = 往右绕 */
#define AVOID_TICK_MS         10    /* 主循环节拍 */

static int avoid_hold = 0;          /* 避让剩余保持时间（ms） */

/* 初始化：避障探头配置为上拉输入 */
void IRAvoidInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = AVOID_PIN;      /* PB1 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(AVOID_PIN_GPIO, &GPIO_InitStructure);
}

/* 避障任务：主循环每 10ms 调用一次，立即返回 */
void AVoidRun(void)
{
    if (AVOID_IO == BARRIER_Y)          /* 前方有障碍 */
    {
        BEEP_SET;                       /* 声光提示 */
        LED_D3_SET;
        avoid_hold = AVOID_HOLD_MS;     /* 续期：只要还看得见，就一直绕 */
    }

    if (avoid_hold > 0)
    {
        avoid_hold -= AVOID_TICK_MS;

#if (AVOID_TURN_DIR > 0)
        /* 往右绕：左轮快、右轮慢 */
        Motor_SetSpeed(AVOID_OUTER_SPEED, AVOID_INNER_SPEED);
#else
        /* 往左绕：左轮慢、右轮快 */
        Motor_SetSpeed(AVOID_INNER_SPEED, AVOID_OUTER_SPEED);
#endif
    }
    else
    {
        BEEP_RESET;
        LED_D3_RESET;
        Motor_SetSpeed(AVOID_BASE_SPEED, AVOID_BASE_SPEED);   /* 无障碍：直行 */
    }
}
