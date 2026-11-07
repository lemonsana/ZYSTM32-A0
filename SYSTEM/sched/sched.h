#ifndef __SCHED_H_
#define __SCHED_H_

#include "stm32f10x.h"

/* ============================================================================
 *  5ms 节拍调度内核
 *  ----------------------------------------------------------------------------
 *  原来的主循环是 while(1){ 任务(); delay_ms(10); }：
 *  只要某个任务耗时（避障测距最长 30ms、串口打印一行约 9ms），
 *  整个控制周期就被拉长。而 PID 的 D 项、丢线计时都假设"每拍正好 10ms"，
 *  节拍一抖，微分项和超时判定就跟着错，而且现场完全看不出来。
 *
 *  这里改成：TIM6 每 5ms 打一拍，节拍由硬件保证；
 *  任务各自按注册的周期被调度，控制周期不再被别的任务带偏。
 *
 *  两类任务：
 *      前台 Sched_Register()          中断里直接执行 —— 必须极短（读 IO、置标志）
 *      后台 Sched_RegisterDeferred()  中断只置 flag，主循环执行 —— 可以耗时
 *
 *  规则：
 *      1. 周期必须是 SYS_TICK_MS 的整数倍；
 *      2. 前台任务里禁止 delay / printf / 等硬件（会拖住整个节拍）；
 *      3. 后台任务单次耗时必须小于自己的周期。
 *         Sched_GetLoopMaxUs() 可以看到主循环最坏一次跑了多久。
 *
 *  时基：TIM6 —— 基本定时器，没有引脚、没有比较输出，只产生更新中断，
 *        和 TIM2(超声波) / TIM3(舵机) / TIM4(电机PWM) / TIM5(红外) 完全不冲突。
 *        SysTick 仍然归 delay.c 的 delay_us / delay_ms 使用，两者互不影响。
 * ========================================================================== */

#define SYS_TICK_MS     5       /* 系统节拍(ms) */
#define MAX_TASKS       12      /* 周期任务槽位数 */

typedef void (*task_func_t)(void);

typedef struct
{
    task_func_t func;
    u16         period_ms;
    u16         counter;
    u8          deferred;       /* 0 = 中断内执行   1 = 主循环执行 */
    volatile u8 flag;           /* 后台任务的执行请求 */
} task_t;

extern volatile u32 g_tick_ms;

void Sched_Init(void);          /* 配置 TIM6，此时还没开始计数 */
void Sched_Start(void);         /* 所有任务注册完成后再启动 */

u8   Sched_Register(task_func_t func, u16 period_ms);           /* 前台 */
u8   Sched_RegisterDeferred(task_func_t func, u16 period_ms);   /* 后台 */

void Sched_Run(void);           /* 主循环里反复调用（内部 __WFI 睡眠） */

u32  Sched_GetTick(void);
u32  Sched_GetUs(void);         /* 微秒时间戳（5ms 内 1us 分辨率） */
u32  Sched_GetLoopUs(void);     /* 上一轮后台处理耗时 */
u32  Sched_GetLoopMaxUs(void);  /* 上电以来最坏一次后台处理耗时 */
void Sched_ClearLoopMax(void);

#endif
