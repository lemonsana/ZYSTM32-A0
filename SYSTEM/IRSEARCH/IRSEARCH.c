#include "IRSEARCH.h"
#include "delay.h"
#include "motor.h"
#include "stm32f10x.h"

/* ==================== 循迹模块（v2 非阻塞版） ====================
 *
 *  硬件（板载固定）：
 *    右循迹探头  PA7 (丝印 IO3)
 *    左循迹探头  PB0 (丝印 IO4)
 *    电平定义：  BLACK_AREA = 1（压到黑线）  WHITE_AREA = 0（白底）
 *
 *  控制策略（v2 改为 10ms 一拍、不阻塞）：
 *    两个探头中间夹着黑线 → 直行
 *    左探头压黑          → 左轮慢、右轮快（左转，把车头拉回线上）
 *    右探头压黑          → 左轮快、右轮慢（右转）
 *    两个都压黑          → 十字/横线，按 TRACK_CROSS_STRAIGHT 决定直行冲过还是停车
 *
 *  ★ 可调参数都在下面这个宏块里，调车时只改这里。
 * ============================================================== */

#define TRACK_SPEED           55    /* 直行基础速度 */
#define TRACK_TURN_OUTER      80    /* 转弯外侧轮（快） */
#define TRACK_TURN_INNER       0    /* 转弯内侧轮（0=原地转；调到 20~30 可边走边转） */

#define TRACK_CROSS_STRAIGHT   1    /* 双探头都压黑时：1=直行冲过去，0=停车 */

/* 初始化：两路循迹探头配置为上拉输入 */
void IRSearchInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = SEARCH_R_PIN;      /* PA7 右循迹 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SEARCH_R_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = SEARCH_L_PIN;      /* PB0 左循迹 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SEARCH_L_GPIO, &GPIO_InitStructure);
}

/* 循迹任务：主循环每 10ms 调用一次，读一次探头、给一次速度，立即返回 */
void SearchRun(void)
{
    u8 left  = SEARCH_L_IO;     /* 左探头电平 */
    u8 right = SEARCH_R_IO;     /* 右探头电平 */

    if (left == WHITE_AREA && right == WHITE_AREA)
    {
        /* 线在两探头中间：直行 */
        Motor_SetSpeed(TRACK_SPEED, TRACK_SPEED);
    }
    else if (left == BLACK_AREA && right == WHITE_AREA)
    {
        /* 线偏左：向左修正（左轮慢、右轮快） */
        Motor_SetSpeed(TRACK_TURN_INNER, TRACK_TURN_OUTER);
    }
    else if (left == WHITE_AREA && right == BLACK_AREA)
    {
        /* 线偏右：向右修正（左轮快、右轮慢） */
        Motor_SetSpeed(TRACK_TURN_OUTER, TRACK_TURN_INNER);
    }
    else
    {
        /* 两探头都压黑：十字路口 / 横线 / 掉出赛道 */
#if TRACK_CROSS_STRAIGHT
        Motor_SetSpeed(TRACK_SPEED, TRACK_SPEED);
#else
        Motor_Stop();
#endif
    }
}
