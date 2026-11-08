#include "selftest.h"
#include "app_mode.h"
#include "IRSEARCH.h"
#include "cfg.h"
#include "motor.h"
#include "IRAvoid.h"
#include "keysacn.h"
#include "remote.h"
#include "stm32f10x.h"

#include <stdio.h>

/* ============================================================================
 *  自检模式（模式 4）
 *  ----------------------------------------------------------------------------
 *  设计要点：
 *      1. 不用 delay_ms 卡死 10ms 任务表 —— 每一步都是"进入跑一次 + 倒计时"，
 *         所以按下模式键能立刻中断自检（app_mode 会把 update 停掉）。
 *      2. 只有超声波是阻塞的（AVoid_Measure 最多 30ms），这是它的实现决定的，
 *         自检模式下可以接受。
 *      3. 结果按位记录，最后一次打印 PASS/FAIL，现场一眼看完。
 * ========================================================================== */

/* ---------------------------- 步骤编号 ---------------------------- */
enum
{
    ST_INTRO = 0,
    ST_GRAY,
    ST_GRAY_DONE,
    ST_MOTOR_L,
    ST_STOP_L,
    ST_MOTOR_R,
    ST_STOP_R,
    ST_MOTOR_B,
    ST_STOP_B,
    ST_SERVO_LEFT,
    ST_SERVO_RIGHT,
    ST_SERVO_CENTER,
    ST_SONIC,
    ST_SONIC_WAIT,
    ST_KEY,
    ST_REMOTE,
    ST_CFG,
    ST_SUMMARY,
    ST_DONE
};

/* ---------------------------- 结果位 ---------------------------- */
#define R_GRAY      (1 << 0)
#define R_MOTOR     (1 << 1)
#define R_SERVO     (1 << 2)
#define R_SONIC     (1 << 3)
#define R_KEY       (1 << 4)
#define R_REMOTE    (1 << 5)
#define R_CFG       (1 << 6)
#define R_ALL       (R_GRAY | R_MOTOR | R_SERVO | R_SONIC | R_KEY | R_REMOTE | R_CFG)

static u8  st_step = ST_INTRO;
static u8  st_init = 0;         /* 本步的"进入动作"是否已执行 */
static u16 st_timer = 0;        /* 本步还剩多少 ms */
static u16 st_result = 0;       /* 结果位图 */

static u8  st_gray_frames = 0;
static u8  st_gray_ok = 0;
static u8  st_sonic_n = 0;
static u8  st_sonic_d[3] = { 0, 0, 0 };
static u8  st_key_idle = 0;
static u16 st_wait_ms = 0;

/* 跳到下一步：清进入标志、设时长 */
static void st_goto(u8 step, u16 ms)
{
    st_step  = step;
    st_init  = 0;
    st_timer = ms;
}

static void st_pass(u16 bit) { st_result |= bit; }
static void st_fail(u16 bit) { st_result &= (u16)(~bit); }

static const char *st_result_text(u16 bit)
{
    return (st_result & bit) ? "PASS" : "FAIL";
}

/* ==========================================================================
 *  每一步的"进入动作"
 * ========================================================================== */
void SelfTest_Start(void)
{
    st_step   = ST_INTRO;
    st_init   = 0;
    st_timer  = 0;
    st_result = R_ALL;          /* 先当全过，哪一步失败就清掉对应位 */

    st_gray_frames = 0;
    st_gray_ok = 0;
    st_sonic_n = 0;
    st_wait_ms = 0;

    Motor_Stop();
    AVoid_Servo(90);

    LED_D4_SET;
    LED_D3_RESET;

#if DEBUG_UART
    printf("\r\n[TEST] ===== 自检开始（按 PC2 可随时退出）=====\r\n");
#endif
}

void SelfTest_Stop(void)
{
    Motor_Stop();
    AVoid_Servo(90);
    LED_D3_RESET;
}

/* 处理一步；返回 1 = 已推进到下一步 */
app_task_result_t SelfTest_Update(void)
{
    static u8 key_level = 0;
    u8 t;
    u8 med;

    switch (st_step)
    {
        /* ---------------------------------------------------------------- */
        case ST_INTRO:
            if (!st_init)
            {
                st_init = 1;
                st_timer = 300;
#if DEBUG_UART
                printf("[TEST] 步骤1/7 灰度...\r\n");
#endif
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_GRAY, 0);
            break;

        /* ---------------------------------------------------------------- */
        case ST_GRAY:
            if (!st_init)
            {
                u8 v[8];

                st_init = 1;
                st_gray_frames++;

                if (Gray_ReadRaw(v))
                {
                    st_gray_ok = 1;
#if DEBUG_UART
                    {
                        u8 act = 0;
                        u8 k;

                        /* 这里自己数一遍覆盖度：自检模式下 SearchRun() 没在跑，
                         * Gray_GetActive() 是上一轮留下的旧值，不能直接用 */
                        for (k = 0; k < 8; k++)
                        {
                            if ((int16_t)v[k] < Cfg_Get(CFG_CUTOFF)) act++;
                        }

                        printf("[TEST] gray %d: %3d %3d %3d %3d %3d %3d %3d %3d  act=%d\r\n",
                               st_gray_frames, v[0], v[1], v[2], v[3],
                               v[4], v[5], v[6], v[7], act);
                    }
#endif
                }
                else
                {
#if DEBUG_UART
                    printf("[TEST] gray %d: 读失败\r\n", st_gray_frames);
#endif
                }

                st_timer = 100;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }

            if (st_gray_frames < 5)
            {
                st_goto(ST_GRAY, 0);
            }
            else
            {
                st_goto(ST_GRAY_DONE, 0);
            }
            break;

        case ST_GRAY_DONE:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 灰度: %s (ready=%d, 读到=%d)\r\n",
                       (Gray_IsReady() && st_gray_ok) ? "PASS" : "FAIL",
                       Gray_IsReady(), st_gray_ok);
                printf("[TEST] 步骤2/7 电机（看轮子转不转、方向对不对）...\r\n");
#endif
                if (Gray_IsReady() && st_gray_ok) st_pass(R_GRAY);
                else                              st_fail(R_GRAY);
            }
            st_goto(ST_MOTOR_L, 0);
            break;

        /* ------------------------------ 电机 ------------------------------ */
        case ST_MOTOR_L:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 左轮 +50\r\n");
#endif
                Motor_SetSpeed(50, 0);
                st_timer = 600;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            Motor_Stop();
            st_goto(ST_STOP_L, 300);
            break;

        case ST_STOP_L:
            if (!st_init) { st_init = 1; }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_MOTOR_R, 0);
            break;

        case ST_MOTOR_R:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 右轮 +50\r\n");
#endif
                Motor_SetSpeed(0, 50);
                st_timer = 600;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            Motor_Stop();
            st_goto(ST_STOP_R, 300);
            break;

        case ST_STOP_R:
            if (!st_init) { st_init = 1; }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_MOTOR_B, 0);
            break;

        case ST_MOTOR_B:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 双轮 +40（车应当往前直走）\r\n");
#endif
                Motor_SetSpeed(40, 40);
                st_timer = 600;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            Motor_Stop();
            st_pass(R_MOTOR);           /* 只要两轮都动过就算 PASS（靠人看） */
            st_goto(ST_STOP_B, 300);
            break;

        case ST_STOP_B:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 步骤3/7 舵机（看是否扫到左/右/中）...\r\n");
#endif
                st_pass(R_SERVO);
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_SERVO_LEFT, 0);
            break;

        /* ------------------------------ 舵机 ------------------------------ */
        case ST_SERVO_LEFT:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 舵机 → 170°\r\n");
#endif
                AVoid_Servo(170);
                st_timer = 500;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_SERVO_RIGHT, 0);
            break;

        case ST_SERVO_RIGHT:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 舵机 → 10°\r\n");
#endif
                AVoid_Servo(10);
                st_timer = 500;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            st_goto(ST_SERVO_CENTER, 0);
            break;

        case ST_SERVO_CENTER:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 舵机 → 90°（回中）\r\n");
#endif
                AVoid_Servo(90);
                st_timer = 400;
                break;
            }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
#if DEBUG_UART
            printf("[TEST] 步骤4/7 超声波（手放前方 10~30cm，连测 3 次）...\r\n");
#endif
            st_goto(ST_SONIC, 0);
            break;

        /* ------------------------------ 超声波 ------------------------------ */
        case ST_SONIC:
            if (!st_init)
            {
                st_init = 1;
                if (st_sonic_n < 3) st_sonic_d[st_sonic_n] = AVoid_Measure();
#if DEBUG_UART
                printf("[TEST] 超声波 %d: %3d cm\r\n", st_sonic_n + 1, st_sonic_d[st_sonic_n]);
#endif
                st_sonic_n++;
                st_goto(ST_SONIC_WAIT, 80);
            }
            break;

        case ST_SONIC_WAIT:
            if (!st_init) { st_init = 1; }
            if (st_timer > MODE_TASK_MS) { st_timer -= MODE_TASK_MS; break; }
            if (st_sonic_n < 3)
            {
                st_goto(ST_SONIC, 0);
            }
            else
            {
                /* 三次取中位数：只要有一次有效就算通 */
                u8 a = st_sonic_d[0], b = st_sonic_d[1], c = st_sonic_d[2];
                if (a > b) { t = a; a = b; b = t; }
                if (b > c) { t = b; b = c; c = t; }
                if (a > b) { t = a; a = b; b = t; }
                med = b;

#if DEBUG_UART
                printf("[TEST] 超声波中位数: %d cm -> %s\r\n", med, (med > 0) ? "PASS" : "FAIL(无回波)");
                printf("[TEST] 步骤5/7 按键（4 秒内按一下 PC2）...\r\n");
#endif
                if (med > 0) st_pass(R_SONIC);
                else         st_fail(R_SONIC);

                st_key_idle = (u8)(KEY ? 1 : 0);   /* 记录空闲电平 */
                st_goto(ST_KEY, 0);
            }
            break;

        /* ------------------------------ 按键 ------------------------------ */
        case ST_KEY:
            if (!st_init)
            {
                st_init = 1;
                st_wait_ms = 0;
            }

            key_level = (u8)(KEY ? 1 : 0);
            if (key_level != st_key_idle)
            {
#if DEBUG_UART
                printf("[TEST] 按键: PASS\r\n");
#endif
                st_pass(R_KEY);
#if DEBUG_UART
                printf("[TEST] 步骤6/7 遥控（4 秒内按遥控器任意键）...\r\n");
#endif
                st_goto(ST_REMOTE, 0);
                break;
            }

            st_wait_ms += MODE_TASK_MS;
            if (st_wait_ms >= 4000)
            {
#if DEBUG_UART
                printf("[TEST] 按键: FAIL(超时)\r\n");
#endif
                st_fail(R_KEY);
                st_goto(ST_REMOTE, 0);
            }
            break;

        /* ------------------------------ 遥控 ------------------------------ */
        case ST_REMOTE:
            if (!st_init)
            {
                st_init = 1;
                st_wait_ms = 0;
            }

            if (Remote_Scan())
            {
#if DEBUG_UART
                printf("[TEST] 遥控: PASS\r\n");
#endif
                st_pass(R_REMOTE);
                st_goto(ST_CFG, 0);
                break;
            }

            st_wait_ms += MODE_TASK_MS;
            if (st_wait_ms >= 4000)
            {
#if DEBUG_UART
                printf("[TEST] 遥控: FAIL(超时，没按键或没插接收头)\r\n");
#endif
                st_fail(R_REMOTE);
                st_goto(ST_CFG, 0);
            }
            break;

        /* ------------------------------ 参数 ------------------------------ */
        case ST_CFG:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("[TEST] 步骤7/7 参数与 Flash...\r\n");
#endif
                Cfg_Print();
                if (Cfg_IsFromFlash()) st_pass(R_CFG);   /* 上电读到了存盘参数 */
                else                   st_fail(R_CFG);   /* 空白页/CRC 错 → 用默认值 */
            }
            st_goto(ST_SUMMARY, 0);
            break;

        /* ------------------------------ 汇总 ------------------------------ */
        case ST_SUMMARY:
            if (!st_init)
            {
                st_init = 1;
#if DEBUG_UART
                printf("\r\n[TEST] ===== 结果 =====\r\n");
                printf("[TEST]  灰度   %s\r\n", st_result_text(R_GRAY));
                printf("[TEST]  电机   %s (靠人看轮子)\r\n", st_result_text(R_MOTOR));
                printf("[TEST]  舵机   %s (靠人看角度)\r\n", st_result_text(R_SERVO));
                printf("[TEST]  超声波 %s\r\n", st_result_text(R_SONIC));
                printf("[TEST]  按键   %s\r\n", st_result_text(R_KEY));
                printf("[TEST]  遥控   %s\r\n", st_result_text(R_REMOTE));
                printf("[TEST]  参数   %s (src=%s)\r\n", st_result_text(R_CFG),
                       Cfg_IsFromFlash() ? "FLASH" : "DEFAULT");
                printf("[TEST] ==================\r\n");
                printf("[TEST] 按 PC2 或串口 '1'/'2'/'3' 退出自检\r\n");
#endif
                /* 全过 D3 亮，有失败 D3 灭——不接串口也能看结论 */
                if ((st_result & R_ALL) == R_ALL) LED_D3_SET;
                else                              LED_D3_RESET;
            }
            st_goto(ST_DONE, 0);
            break;

        case ST_DONE:
        default:
            Motor_Stop();
            break;
    }

    return APP_TASK_RUNNING;
}
