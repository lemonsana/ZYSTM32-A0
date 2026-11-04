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
 *  ZYSTM32-A0 智能小车  综合程序  v3.0
 *  --------------------------------------------------------------------
 *  三种模式，按 PC2 按键循环切换，LED 指示：
 *      模式1 循迹   D4 亮 / D3 灭      SearchRun()   感为 8 路灰度（I2C）
 *      模式2 避障   D4 灭（D3 作障碍指示） AVoidRun()
 *      模式3 遥控   D4 亮 / D3 亮       IR_IN()
 *
 *  v3.0 关键修复：
 *    1) 电机通道映射改回实测正确值（左 CH4/PB9，右 CH3/PB8）——
 *       v2.0 的判断是错的，"可循迹版本"用原厂映射能跑，说明原厂是对的；
 *    2) 循迹算法整体换成"可循迹版本"的成熟实现（归一化模拟量 + 重心 + 滤波 + 死区）；
 *    3) 丢线超时按毫秒计，与循环节拍无关；
 *    4) 保留 v2.0 的非阻塞电机层与"不停车避障"。
 * ========================================================================== */

#define FIRMWARE_VER        "v3.3"
#define TICK_MS             10      /* 主循环节拍(ms)，必须与 GRAY_FRAME_MS 一致 */
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

    if ((keypre == 0) && KEY)       /* 上拉输入，按下为高 */
    {
        keypre = 1;
        switch (keyMode)
        {
            case KEYMODE_1: keyMode = KEYMODE_2; break;
            case KEYMODE_2: keyMode = KEYMODE_3; break;
            case KEYMODE_3: keyMode = KEYMODE_1; break;
            default:        keyMode = KEYMODE_1; break;
        }
        Motor_Stop();               /* 换模式先停车 */
        Gray_ResetIntegral();       /* 清掉上一模式残留的积分量 */
    }
    if (!KEY)                       /* 松开，允许下一次切换 */
    {
        keypre = 0;
    }
}

/* ------------------------- 红外遥控（非阻塞） -------------------------
 * 键值：98=前进 168=后退 34=左转 194=右转 2=停止 224=原地左旋 144=原地右旋
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

    if (remote_timer > 0) remote_timer -= TICK_MS;
    else                  Motor_Stop();             /* 松手/超时自动停 */
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

/* ------------------------- 串口调试打印（每 200ms 一次） -------------------------
 *  err    线位置误差：0 居中，负数偏左，正数偏右，999 丢线
 *  ready  1 = 灰度模块初始化成功（PING 通了、归一化已打开）
 *  avoid  避障探头电平（0=有障碍）
 * ------------------------------------------------------------------------------ */
void Debug_Task(void)
{
#if DEBUG_UART
    static u16 cnt = 0;

    if (++cnt >= 20)
    {
        cnt = 0;
        printf("mode=%d L=%4d R=%4d | err=%4d corr=%4d spd=%3d ready=%d avoid=%d\r\n",
               keyMode,
               Motor_GetLeft(),
               Motor_GetRight(),
               Gray_GetError(),
               Gray_GetCorrection(),
               Gray_GetBaseSpeed(),
               Gray_IsReady(),
               AVOID_IO);
    }
#endif
}

/* ------------------------- 主函数 ------------------------- */
int main(void)
{
    delay_init();
    KEY_Init();             /* 按键 PC2 / 蜂鸣器 PC3 / LED D3 PB6 / D4 PA6 */
    IRSearchInit();         /* 灰度传感器初始化（I2C：PB10/PB11） */
    IRAvoidInit();          /* 避障探头 PB1 */
    Motor_Init();           /* TIM4 PWM：左 CH4-PB9，右 CH3-PB8 */
    Remote_Init();          /* 红外遥控 PA1 + TIM5 输入捕获 */

#if DEBUG_UART
    uart_init(115200);      /* USART1 PA9/PA10 → 板载 CH340G → USB */
    printf("\r\n[BOOT] ZYSTM32-A0 %s  build %s %s\r\n", FIRMWARE_VER, __DATE__, __TIME__);
    printf("[BOOT] gray ready = %d %s\r\n",
           Gray_IsReady(),
           Gray_IsReady() ? "(ok)" : "(FAIL: 查 PULL 跳线 / SCL-SDA / 5V)");
#endif

    Motor_Stop();
    keyMode = KEYMODE_1;    /* 上电默认进入循迹模式 */

    while (1)
    {
        KeyScanTask();      /* 1. 读按键，必要时切模式 */
        ModeTask();         /* 2. 执行当前模式的任务（非阻塞） */
        Debug_Task();       /* 3. 串口打印调试信息 */
        delay_ms(TICK_MS);  /* 4. 10ms 节拍 */
    }
}
