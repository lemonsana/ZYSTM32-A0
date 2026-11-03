#include "stm32f10x.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "remote.h"
#include "usart.h"
#include <stdio.h>

/* ==========================================================================
 *  ZYSTM32-A0 智能小车  综合程序  v2.0
 *  --------------------------------------------------------------------
 *  三种模式，按 PC2 按键循环切换，LED 指示：
 *      模式1 循迹   D4 亮 / D3 灭      SearchRun()
 *      模式2 避障   D4 灭（D3 作障碍指示） AVoidRun()
 *      模式3 遥控   D4 亮 / D3 亮       IR_IN()
 *
 *  v2 相对 v1 的关键改动：
 *    1) 修正电机 PWM 通道交叉（左轮 CH3/PB8，右轮 CH4/PB9），
 *       并且速度为 0 时也会写方向脚 —— 修好"循迹只单向转左轮"。
 *    2) 电机层改为非阻塞 Motor_SetSpeed()，主循环 10ms 一拍；
 *       三个模式都不再长时间占用 CPU，按键响应更及时。
 *    3) 避障改为"不停车、不后退，直接弧线绕过"。
 *    4) 打开 USART1(PA9/PA10) 调试串口：板上 CH340G 转 USB，
 *       插 USB 线、串口助手 115200 即可看到运行数据。
 * ========================================================================== */

#define FIRMWARE_VER        "v2.0"
#define TICK_MS             10      /* 主循环节拍(ms)，即 100Hz */
#define DEBUG_UART          1       /* 1=打开串口调试打印，0=关闭 */

#define KEYMODE_1           1       /* 循迹 */
#define KEYMODE_2           2       /* 避障 */
#define KEYMODE_3           3       /* 遥控 */

#define REMOTE_ACTION_MS    120     /* 遥控每次按键动作保持时间(ms) */

uint8_t keyMode;                    /* 当前模式 */

static int remote_timer = 0;        /* 遥控动作剩余时间 */

/* ------------------------- 按键扫描：切换模式 ------------------------- */
void KeyScanTask(void)
{
    static u8 keypre = 0;

    if ((keypre == 0) && KEY)       /* 检测到按下（上拉输入，按下为高） */
    {
        keypre = 1;
        switch (keyMode)
        {
            case KEYMODE_1: keyMode = KEYMODE_2; break;
            case KEYMODE_2: keyMode = KEYMODE_3; break;
            case KEYMODE_3: keyMode = KEYMODE_1; break;
            default:        keyMode = KEYMODE_1; break;
        }
        Motor_Stop();               /* 换模式先停车，避免带着上一模式的速度跑 */
    }
    if (!KEY)                       /* 松开，允许下一次切换 */
    {
        keypre = 0;
    }
}

/* ------------------------- 红外遥控（非阻塞） -------------------------
 * 遥控器键值见光盘《STM32遥控器按键表》，NEC 协议，Remote_Scan() 返回命令字节:
 *     98  = CH     前进
 *     168 = FFA857 后退
 *     34  = VOL-   左转
 *     194 = CH+    右转
 *     2   = 暂停   停止
 *     224 = 上一曲 原地左旋
 *     144 = 下一曲 原地右旋
 * -------------------------------------------------------------------- */
void IR_IN(void)
{
    u8 key = Remote_Scan();

    if (key)
    {
        switch (key)
        {
            case 98:  Motor_SetSpeed( 80,  80); break;  /* 前进 */
            case 168: Motor_SetSpeed(-80, -80); break;  /* 后退 */
            case 34:  Motor_SetSpeed(  0,  80); break;  /* 左转 */
            case 194: Motor_SetSpeed( 80,   0); break;  /* 右转 */
            case 224: Motor_SetSpeed(-80,  80); break;  /* 原地左旋 */
            case 144: Motor_SetSpeed( 80, -80); break;  /* 原地右旋 */
            case 2:   Motor_Stop();             break;  /* 停止 */
            default:  break;
        }
        remote_timer = REMOTE_ACTION_MS;
    }

    if (remote_timer > 0)
    {
        remote_timer -= TICK_MS;
    }
    else
    {
        Motor_Stop();               /* 松手/超时自动停 */
    }
}

/* ------------------------- 模式调度 ------------------------- */
void ModeTask(void)
{
    switch (keyMode)
    {
        case KEYMODE_1:                     /* 循迹 */
            LED_D4_SET;
            LED_D3_RESET;
            SearchRun();
            break;

        case KEYMODE_2:                     /* 避障（D3 由 AVoidRun 控制） */
            LED_D4_RESET;
            AVoidRun();
            break;

        case KEYMODE_3:                     /* 遥控 */
            LED_D4_SET;
            LED_D3_SET;
            IR_IN();
            break;

        default:
            Motor_Stop();
            break;
    }
}

/* ------------------------- 串口调试打印（每 200ms 一次） ------------------------- */
void Debug_Task(void)
{
#if DEBUG_UART
    static u16 cnt = 0;

    if (++cnt >= 20)
    {
        cnt = 0;
        printf("mode=%d L=%4d R=%4d  |  trackL=%d trackR=%d avoid=%d\r\n",
               keyMode,
               Motor_GetLeft(),
               Motor_GetRight(),
               SEARCH_L_IO,
               SEARCH_R_IO,
               AVOID_IO);
    }
#endif
}

/* ------------------------- 主函数 ------------------------- */
int main(void)
{
    delay_init();
    KEY_Init();             /* 按键 PC2 / 蜂鸣器 PC3 / LED D3 PB6 / D4 PA6 */
    IRSearchInit();         /* 循迹探头 PA7 / PB0 */
    IRAvoidInit();          /* 避障探头 PB1 */
    Motor_Init();           /* TIM4 PWM：左 CH3-PB8，右 CH4-PB9 */
    Remote_Init();          /* 红外遥控 PA1 + TIM5 输入捕获 */

#if DEBUG_UART
    uart_init(115200);      /* USART1 PA9/PA10 → 板载 CH340G → USB */
    printf("\r\n[BOOT] ZYSTM32-A0 %s  build %s %s\r\n", FIRMWARE_VER, __DATE__, __TIME__);
#endif

    Motor_Stop();
    keyMode = KEYMODE_1;    /* 上电默认进入循迹模式 */

    while (1)
    {
        KeyScanTask();      /* 1. 读按键，必要时切模式 */
        ModeTask();         /* 2. 执行当前模式的任务（都是非阻塞） */
        Debug_Task();       /* 3. 串口打印调试信息 */
        delay_ms(TICK_MS);  /* 4. 10ms 节拍 */
    }
}
