#include "app_mode.h"
#include "sched.h"
#include "motor.h"
#include "keysacn.h"
#include "usart.h"
#include "remote.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "cfg.h"
#include "selftest.h"

#include <stdio.h>
#include <string.h>

/* ============================================================================
 *  业务模式表 + 三个模式的实现
 *  ----------------------------------------------------------------------------
 *  模式1 融合：循迹 + 避障（原来的 CombinedTask）
 *  模式2 纯避障：只跑 IRAvoid 状态机
 *  模式3 纯循迹：只跑灰度 PID（调 PID / 测传感器用）
 *
 *  每个模式拆成 start / update / stop：
 *      start   进入时调一次，负责清状态、点灯
 *      update  每 MODE_TASK_MS(10ms) 调一次，返回 APP_TASK_DONE 表示要收尾
 *      stop    离开时调一次
 * ========================================================================== */

/* ------------------------- 融合状态机状态 ------------------------- */
#define FUS_LINE      0             /* 正常循迹 */
#define FUS_SCAN      1             /* 停车扫描左右 */
#define FUS_TURN      2             /* 转向绕行 */
#define FUS_SEEK      3             /* 原地反向找线 */
#define FUS_COOLDOWN  4             /* 冷静期 */

static u8  fus_state    = FUS_LINE;
static u8  fus_step     = 0;
static u16 fus_timer    = 0;
static u16 fus_tick     = 0;
static u8  fus_dist     = 0;
static u8  fus_dist_l   = 0;
static u8  fus_dist_r   = 0;
static u8  fus_dir_left = 1;        /* 往左绕?  0xFF = 原地掉头 */

static u8  g_active = APP_MODE_NONE;

static int remote_timer = 0;

/* 前置声明：下面的模式表要引用这些函数 */
static void mode1_start(void);
static app_task_result_t mode1_update(void);
static void mode1_stop(void);
static void mode2_start(void);
static app_task_result_t mode2_update(void);
static void mode2_stop(void);
static void mode3_start(void);
static app_task_result_t mode3_update(void);
static void mode3_stop(void);

/* ============================ 模式表 ============================
 * 加模式只改这里：菜单字符同时就是串口指令字符（大小写不敏感）。
 * 按键轮换、串口切换、调试打印都会自动跟着这张表走。
 * ============================================================== */
static const app_mode_desc_t g_modes[] =
{
    { '1', "T1 循迹+避障", mode1_start, mode1_update, mode1_stop },
    { '2', "T2 纯避障",    mode2_start, mode2_update, mode2_stop },
    { '3', "T3 纯循迹",    mode3_start, mode3_update, mode3_stop },
    { '4', "T4 自检",      SelfTest_Start, SelfTest_Update, SelfTest_Stop },
};

#define APP_MODE_COUNT  ((u8)(sizeof(g_modes) / sizeof(g_modes[0])))

/* ==========================================================================
 *  模式1：循迹 + 避障
 *  ========================================================================== */
static void mode1_start(void)
{
    fus_state    = FUS_LINE;
    fus_step     = 0;
    fus_timer    = 0;
    fus_tick     = 0;
    fus_dir_left = 1;

    LED_D4_SET;
    LED_D3_RESET;
    BEEP_RESET;
}

static void mode1_stop(void)
{
    fus_state = FUS_LINE;
    fus_timer = 0;
    fus_tick  = 0;
}

static app_task_result_t mode1_update(void)
{
    switch (fus_state)
    {
        /* ---------------- 循迹 + 定时测前方 ---------------- */
        case FUS_LINE:
            LED_D4_SET;
            LED_D3_RESET;
            SearchRun();

            if (++fus_tick >= (FUS_MEAS_PERIOD_MS / MODE_TASK_MS))
            {
                fus_tick = 0;
                fus_dist = AVoid_Measure();

                if (fus_dist && fus_dist < FUS_TRIGGER_CM)
                {
                    Motor_Stop();
                    Gray_ResetIntegral();
                    AVoid_Servo(90);
                    fus_state = FUS_SCAN;
                    fus_step  = 0;
                    fus_timer = FUS_SETTLE_MS;
                    BEEP_SET;
                    LED_D3_SET;
                }
            }
            break;

        /* ---------------- 停车扫描：中 → 左 → 右 ---------------- */
        case FUS_SCAN:
            Motor_Stop();

            if (fus_timer > 0) { fus_timer -= MODE_TASK_MS; break; }

            if (fus_step == 0)
            {
                fus_dist = AVoid_Measure();          /* 前方（已知有障碍） */
                AVoid_Servo(FUS_SERVO_LEFT);
                fus_step  = 1;
                fus_timer = FUS_SETTLE_MS;
            }
            else if (fus_step == 1)
            {
                fus_dist_l = AVoid_Measure();        /* 左侧 */
                AVoid_Servo(FUS_SERVO_RIGHT);
                fus_step  = 2;
                fus_timer = FUS_SETTLE_MS;
            }
            else
            {
                fus_dist_r = AVoid_Measure();        /* 右侧，然后决策 */
                AVoid_Servo(90);

                if (fus_dist_l < FUS_MIN_GAP_CM && fus_dist_r < FUS_MIN_GAP_CM)
                {
                    fus_dir_left = 0xFF;             /* 两侧都堵 → 原地掉头 */
                }
                else
                {
                    fus_dir_left = ((fus_dist_l ? fus_dist_l : 200) >=
                                    (fus_dist_r ? fus_dist_r : 200)) ? 1 : 0;
                }

                fus_state = FUS_TURN;
                fus_timer = FUS_TURN_MS;
            }
            break;

        /* ---------------- 转向绕行 ---------------- */
        case FUS_TURN:
            if (fus_dir_left == 0xFF)
                Motor_SetSpeed(-50, 50);             /* 原地左掉头 */
            else if (fus_dir_left)
                Motor_SetSpeed(0, 85);               /* 左转：左轮停 */
            else
                Motor_SetSpeed(85, 0);               /* 右转：右轮停 */

            if (fus_timer > MODE_TASK_MS) fus_timer -= MODE_TASK_MS;
            else
            {
                Motor_Stop();
                fus_state = FUS_SEEK;
                fus_timer = FUS_SEEK_TIMEOUT_MS;
            }
            break;

        /* ---------------- 原地反向找线 ---------------- */
        case FUS_SEEK:
            if (Gray_IsOnLine())                     /* 看到线了 */
            {
                Motor_Stop();
                Gray_ResetIntegral();
                fus_state = FUS_COOLDOWN;
                fus_timer = FUS_COOLDOWN_MS;
                BEEP_RESET;
                LED_D3_RESET;
                break;
            }

            if (fus_timer <= MODE_TASK_MS)           /* 超时：先回到循迹 */
            {
                Motor_Stop();
                Gray_ResetIntegral();
                fus_state = FUS_COOLDOWN;
                fus_timer = FUS_COOLDOWN_MS;
                BEEP_RESET;
                LED_D3_RESET;
                break;
            }
            fus_timer -= MODE_TASK_MS;

            /* 往反方向转：刚才是左绕，现在就右转找线（反之亦然） */
            if (fus_dir_left == 0xFF || fus_dir_left)
                Motor_SetSpeed(FUS_SEEK_SPEED, -FUS_SEEK_SPEED);   /* 右转找线 */
            else
                Motor_SetSpeed(-FUS_SEEK_SPEED, FUS_SEEK_SPEED);   /* 左转找线 */
            break;

        /* ---------------- 冷静期 ---------------- */
        case FUS_COOLDOWN:
            SearchRun();
            if (fus_timer > MODE_TASK_MS) fus_timer -= MODE_TASK_MS;
            else fus_state = FUS_LINE;
            break;

        default:
            fus_state = FUS_LINE;
            break;
    }

    return APP_TASK_RUNNING;
}

/* ==========================================================================
 *  模式2：纯避障
 *  ========================================================================== */
static void mode2_start(void)
{
    LED_D4_RESET;
}

static app_task_result_t mode2_update(void)
{
    AVoidRun();
    return APP_TASK_RUNNING;
}

static void mode2_stop(void)
{
}

/* ==========================================================================
 *  模式3：纯循迹
 *  ========================================================================== */
static void mode3_start(void)
{
    LED_D4_SET;
    LED_D3_SET;
}

static app_task_result_t mode3_update(void)
{
    SearchRun();
    return APP_TASK_RUNNING;
}

static void mode3_stop(void)
{
}

/* ==========================================================================
 *  红外遥控（R1.1 起未注册成任务，保留能力）
 *  ----------------------------------------------------------------------------
 *  键值：98=前进 168=后退 34=左转 194=右转 2=停止 224=原地左旋 144=原地右旋
 *  要启用：在 main.c 里 Sched_RegisterDeferred(IR_IN, 10);
 * ========================================================================== */
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
        remote_timer = 120;                             /* 动作保持时间(ms) */
    }

    if (remote_timer > 0) remote_timer -= MODE_TASK_MS;
    else                  Motor_Stop();
}

/* ==========================================================================
 *  模式管理
 *  ========================================================================== */
void app_mode_init(void)
{
    g_active = APP_MODE_NONE;
    fus_state = FUS_LINE;
    fus_step = 0;
    fus_timer = 0;
    fus_tick = 0;
    fus_dir_left = 1;
}

u8   app_mode_count(void)              { return APP_MODE_COUNT; }
u8   app_mode_get(void)                { return g_active; }
u8   app_mode_fusion_state(void)       { return fus_state; }

char app_mode_menu_char(u8 idx)
{
    if (idx >= APP_MODE_COUNT) return 0;
    return g_modes[idx].menu_char;
}

const char *app_mode_name(u8 idx)
{
    if (idx >= APP_MODE_COUNT) return "";
    return g_modes[idx].name;
}

int8_t app_mode_index_of_char(char c)
{
    u8 i;
    char key = c;

    if (key >= 'a' && key <= 'z') key = (char)(key - 'a' + 'A');

    for (i = 0; i < APP_MODE_COUNT; i++)
    {
        if (g_modes[i].menu_char == key) return (int8_t)i;
    }
    return -1;
}

void app_mode_stop_active(void)
{
    if (g_active >= APP_MODE_COUNT) return;

    if (g_modes[g_active].stop) g_modes[g_active].stop();

    g_active = APP_MODE_NONE;
    Motor_Stop();
    Gray_ResetIntegral();
}

void app_mode_start(u8 idx)
{
    if (idx >= APP_MODE_COUNT) return;

    app_mode_stop_active();         /* 旧模式先收尾 */

    Motor_Stop();
    Gray_ResetIntegral();

    g_active = idx;
    if (g_modes[idx].start) g_modes[idx].start();
}

u8 app_mode_start_by_char(char c)
{
    int8_t idx = app_mode_index_of_char(c);

    if (idx < 0) return 0;
    app_mode_start((u8)idx);
    return 1;
}

void app_mode_next(void)
{
    u8 next;

    if (g_active >= APP_MODE_COUNT) next = 0;
    else                            next = (u8)(g_active + 1);

    if (next >= APP_MODE_COUNT) next = 0;
    app_mode_start(next);
}

/* ==========================================================================
 *  周期任务入口
 *  ========================================================================== */

/* 5ms 前台（中断里执行）：只读一个 IO，别的都交给 10ms 后台 */
void app_mode_key_task(void)
{
    static u8 keypre = 0;

    if (g_tick_ms < STARTUP_MS) return;     /* 上电启动屏蔽，防误触发 */

    if ((keypre == 0) && KEY)
    {
        keypre = 1;
        app_mode_next();
    }
    if (!KEY) keypre = 0;
}

/* 串口整行指令（V6 重写）
 * ----------------------------------------------------------------------------
 * 老写法是"整行扫一遍，找到表里的模式字符就切"，这样有个坑：
 *      "SET KD 12" 里同时含 '1' 和 '2'，一条调参命令会被当成切模式。
 *
 * 现在分两步：
 *      1) 先交给 cfg 解析（? / SET / SAVE / LOAD / DEF / 直接 "BASE 55"）
 *      2) 剩下的一行必须"只有 1 个字符"才当模式切换
 */
static void app_mode_poll_uart(void)
{
    char line[40];
    char cmdline[40];
    u8   len;
    u8   i;
    u8   j;

    if (!uart_take_line(line, sizeof(line))) return;

    /* 去掉首尾空白和 \r\n */
    len = (u8)strlen(line);
    i = 0;
    while (i < len && (line[i] == ' ' || line[i] == '\t')) i++;
    while (len > i && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                       line[len - 1] == ' '  || line[len - 1] == '\t'))
    {
        len--;
    }
    if (i >= len) return;

    j = 0;
    while (i < len && j < (u8)(sizeof(cmdline) - 1)) cmdline[j++] = line[i++];
    cmdline[j] = '\0';

    if (Cfg_HandleLine(cmdline)) return;        /* 调参命令优先 */

    if (j == 1 && app_mode_index_of_char(cmdline[0]) >= 0)
    {
#if DEBUG_UART
        printf("T%c\r\n", cmdline[0]);
#endif
        app_mode_start_by_char(cmdline[0]);
    }
}

/* 10ms 后台：串口指令 + 当前模式推进 */
void app_mode_task(void)
{
    u8 idx;

    if (g_tick_ms < STARTUP_MS) return;

    app_mode_poll_uart();

    /* 先把下标拷出来再解引用：按键任务在 5ms 中断里可能把 g_active 改掉，
     * 期间它会短暂经过 APP_MODE_NONE(0xFF)，直接用 g_active 索引会越界。 */
    idx = g_active;
    if (idx < APP_MODE_COUNT && g_modes[idx].update)
    {
        if (g_modes[idx].update() != APP_TASK_RUNNING)
        {
            /* 只有模式没被中断切走，才由这里收尾 */
            if (g_active == idx)
            {
                Motor_Stop();
                app_mode_stop_active();
            }
        }
    }
}

/* 200ms 后台：一行调试信息；每 5 次(1s)补一行调度耗时 */
void app_mode_debug_task(void)
{
#if DEBUG_UART
    static u8 cnt = 0;

    printf("mode=%d L=%4d R=%4d | err=%4d corr=%4d spd=%3d ev=%-4s act=%d cc=%d | ready=%d st=%d dist=%3d ang=%3d fus=%d\r\n",
           (g_active == APP_MODE_NONE) ? 0 : (int)(g_active + 1),
           Motor_GetLeft(),
           Motor_GetRight(),
           Gray_GetError(),
           Gray_GetCorrection(),
           Gray_GetBaseSpeed(),
           Gray_EventName(Gray_GetEvent()),
           Gray_GetActive(),
           Gray_GetCrossCount(),
           Gray_IsReady(),
           AVoid_GetState(),
           AVoid_GetDistance(),
           AVoid_GetAngle(),
           fus_state);

    if (++cnt >= 5)
    {
        cnt = 0;
        printf("[sched] loop=%4dus max=%4dus\r\n",
               Sched_GetLoopUs(), Sched_GetLoopMaxUs());
        Sched_ClearLoopMax();
    }
#endif
}
