#include "sched.h"

/* ============================================================================
 *  5ms 节拍调度内核
 *  ----------------------------------------------------------------------------
 *  TIM6：72MHz / (71+1) = 1MHz 计数，5000 个计数 = 5ms。
 *        基本定时器没有引脚，不会和电机/舵机/超声波抢任何 IO。
 * ========================================================================== */

volatile u32 g_tick_ms = 0;

static task_t g_tasks[MAX_TASKS];
static u8     g_task_cnt = 0;

static u32 g_loop_us     = 0;
static u32 g_loop_max_us = 0;

/* ============================================================================
 *  TIM6 中断：每 5ms 一拍
 *  ----------------------------------------------------------------------------
 *  只做三件事：累加节拍、推进各任务计数器、到点执行（前台）或置标志（后台）。
 *  这里绝对不能放耗时操作，否则整个系统的时间基准都会漂。
 * ========================================================================== */
void TIM6_IRQHandler(void)
{
    u8 i;

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == RESET) return;

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    g_tick_ms += SYS_TICK_MS;

    for (i = 0; i < g_task_cnt; i++)
    {
        task_t *t = &g_tasks[i];

        t->counter = (u16)(t->counter + SYS_TICK_MS);
        if (t->counter >= t->period_ms)
        {
            t->counter = 0;

            if (t->deferred)
            {
                t->flag = 1;            /* 交给主循环，不在这里做 */
            }
            else if (t->func)
            {
                t->func();              /* 前台任务：必须极短 */
            }
        }
    }
}

void Sched_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    tb.TIM_Prescaler     = 71;
    tb.TIM_Period        = (u16)(SYS_TICK_MS * 1000 - 1);   /* 5000-1 */
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &tb);

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void Sched_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);
}

static u8 sched_add(task_func_t func, u16 period_ms, u8 deferred)
{
    task_t *t;

    if (func == 0 || period_ms == 0) return 0;
    if (g_task_cnt >= MAX_TASKS)     return 0;

    t = &g_tasks[g_task_cnt];
    t->func      = func;
    t->period_ms = period_ms;
    t->counter   = 0;
    t->deferred  = deferred;
    t->flag      = 0;

    g_task_cnt++;       /* 最后才加计数，避免中断看到半个任务 */
    return 1;
}

u8 Sched_Register(task_func_t func, u16 period_ms)
{
    return sched_add(func, period_ms, 0);
}

u8 Sched_RegisterDeferred(task_func_t func, u16 period_ms)
{
    return sched_add(func, period_ms, 1);
}

/* 微秒时间戳：节拍 * 1000 + TIM6 当前计数（1MHz）。
 * 注意：刚好跨过 5ms 重载点时，g_tick_ms 可能还没在中断里更新，
 *       会读出一次"倒退"。所以算差值的地方要把这种毛刺丢掉。 */
u32 Sched_GetUs(void)
{
    u32 ms;
    u16 cnt;
    u32 primask = __get_PRIMASK();

    __disable_irq();
    ms  = g_tick_ms;
    cnt = (u16)TIM6->CNT;
    __set_PRIMASK(primask);

    return ms * 1000U + (u32)cnt;
}

u32  Sched_GetTick(void)      { return g_tick_ms; }
u32  Sched_GetLoopUs(void)    { return g_loop_us; }
u32  Sched_GetLoopMaxUs(void) { return g_loop_max_us; }
void Sched_ClearLoopMax(void) { g_loop_max_us = 0; }

/* ============================================================================
 *  主循环：取走所有到期的后台任务，然后睡到下一拍
 *  ----------------------------------------------------------------------------
 *  flag 是"先清再执行"：任务执行期间又到期的一拍会把 flag 重新置 1，
 *  下一轮立刻补跑，不会丢拍。
 * ========================================================================== */
void Sched_Run(void)
{
    u32 t0, dt;
    u8  i;

    t0 = Sched_GetUs();

    for (i = 0; i < g_task_cnt; i++)
    {
        task_t *t = &g_tasks[i];

        if (t->deferred && t->flag)
        {
            t->flag = 0;
            if (t->func) t->func();
        }
    }

    dt = Sched_GetUs() - t0;
    if (dt > 1000000U) dt = 0;      /* 跨重载点的毛刺，丢弃 */

    g_loop_us = dt;
    if (dt > g_loop_max_us) g_loop_max_us = dt;

    __WFI();                        /* 没活就睡到下一拍，省电也省抖动 */
}
