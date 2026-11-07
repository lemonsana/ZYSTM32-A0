#include "stm32f10x.h"
#include "delay.h"
#include "keysacn.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "motor.h"
#include "remote.h"
#include "usart.h"
#include "sched.h"
#include "app_mode.h"

#include <stdio.h>

/* ============================================================================
 *  ZYSTM32-A0 智能小车  综合程序  V5
 *  ----------------------------------------------------------------------------
 *  V5 相对 R1.1 的改动（分两步落地：先架构、后高速转向）：
 *      1. 主循环从 while(1){ 任务(); delay_ms(10); } 换成 5ms 节拍调度：
 *         TIM6 每 5ms 打一拍，任务按注册周期执行，控制周期不再被任务耗时带偏；
 *      2. 三个模式从 switch(keyMode) 改成一张模式表（app_mode.c），
 *         加模式 = 加一行；
 *      3. 新增串口整行指令：发 '1'/'2'/'3' 直接切模式（原本只能按板载按键）；
 *      4. 新增上电 500ms 屏蔽，防止上电瞬间误触发模式切换；
 *      5. 调试行每 1s 增加 [sched] loop/max，可直接看出主循环最坏耗时；
 *      6. 循迹转向量加"归一化 + 速度补偿"（IRSEARCH.h 的 GRAY_STEER_*）：
 *         3~4 路宽的黑线压到阵列边缘误差只有 55~70，按 55 归一化后才算满舵，
 *         再按 base/60 跟着车速放大 —— 60~95 速下内侧轮能压到 0，不再冲出弯道。
 *
 *  避障 / 电机 / 灰度取样 的参数与算法保持 R1.1。
 * ========================================================================== */

#define FIRMWARE_VER    "V5"

int main(void)
{
    delay_init();               /* SysTick：delay_us / delay_ms 用 */
    KEY_Init();                 /* 按键 PC2 / 蜂鸣器 PC3 / LED D3 PB6 / D4 PA6 */
    IRSearchInit();             /* 灰度传感器（I2C：PB10/PB11） */
    IRAvoidInit();              /* 超声波 PC0/PC1 + 舵机 PB0(TIM3_CH3) */
    Motor_Init();               /* TIM4 PWM：左 CH4-PB9，右 CH3-PB8 */
    Remote_Init();              /* 红外遥控 PA1（当前未注册成任务） */

#if DEBUG_UART
    uart_init(115200);          /* USART1 PA9/PA10 → 板载 CH340G → USB */
    printf("\r\n[BOOT] ZYSTM32-A0 %s  build %s %s\r\n",
           FIRMWARE_VER, __DATE__, __TIME__);
    printf("[BOOT] gray ready = %d %s\r\n",
           Gray_IsReady(),
           Gray_IsReady() ? "(ok)" : "(FAIL: 查 PULL 跳线 / SCL-SDA / 5V)");
#endif

    Sched_Init();                                       /* TIM6 5ms，先不启动 */

    Sched_Register(app_mode_key_task, 5);               /* 前台：按键扫描 */
    Sched_RegisterDeferred(app_mode_task, MODE_TASK_MS);/* 后台：串口 + 模式推进 */
    Sched_RegisterDeferred(app_mode_debug_task, 200);   /* 后台：调试打印 */

    app_mode_init();
    app_mode_start(0);                                  /* 上电默认模式1（综合） */

    Sched_Start();                                      /* 节拍开跑 */

    while (1)
    {
        Sched_Run();                                    /* 内部 __WFI，睡到下一拍 */
    }
}
