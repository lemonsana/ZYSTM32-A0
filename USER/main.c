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
 *  ZYSTM32-A0 智能小车  综合程序  v3.6
 *  --------------------------------------------------------------------
 *  三种模式，按 PC2 按键循环切换，LED 指示：
 *      模式1 综合   D4 亮（避让时 D3 亮）  CombinedTask()  循迹 + 避障
 *      模式2 纯避障 D4 灭（D3 作障碍指示） AVoidRun()      超声波 + 舵机扫描
 *      模式3 遥控   D4 亮 / D3 亮          IR_IN()
 *
 *  【版本演进要点】
 *    v3.0  电机通道改回实测正确值；循迹换成"可循迹版本"的成熟算法
 *    v3.1  加 PID（I/D 可调）+ 串口 corr 调试字段
 *    v3.2  急弯增强：速度调度 / 增益调度 / 内轮反转
 *    v3.3  按实测前视 12cm 重调参数，加"进弯预减速"
 *    v3.4  避障改为状态机（去抖 + 两级转弯 + 回正补偿）；全工程补充调参注释
 *    v3.5  避障升级为「超声波 HC-SR04 + 舵机 180° 扫描」：
 *            停车扫描版 / 边走边扫版，由 IRAvoid.h 的 AVOID_SCAN_WHILE_DRIVING 切换
 *    v3.6  把模式1（循迹）和模式2（避障）**合成一个综合模式**：
 *            循迹 → 遇到障碍 → 停车扫左右 → 绕行 → 原地找线 → 回到循迹
 *            模式2 保留为"纯避障"，方便单独调避障参数
 *
 *  【主循环结构】
 *      每 10ms 一拍：
 *        KeyScanTask()  读按键、切模式（切模式会停车并清 PID 积分）
 *        ModeTask()     按模式执行（三个任务都是非阻塞，立即返回）
 *        Debug_Task()   每 200ms 打一行串口
 *        delay_ms(10)   节拍
 * ========================================================================== */

#define FIRMWARE_VER        "v3.6"
#define TICK_MS             10      /* 主循环节拍(ms)，必须与 GRAY_FRAME_MS 一致 */
#define DEBUG_UART          1       /* 1=打开串口调试打印，0=关闭 */

#define KEYMODE_1           1       /* 综合：循迹 + 避障（默认） */
#define KEYMODE_2           2       /* 纯避障（单独调避障参数用） */
#define KEYMODE_3           3       /* 遥控 */

#define REMOTE_ACTION_MS    120     /* 遥控每次按键动作保持时间(ms) */

/* ============================================================================
 *  融合模式（模式1：循迹 + 避障）参数
 *  ----------------------------------------------------------------------------
 *  流程：
 *      循迹直行 ──前方 < FUS_TRIGGER_CM──→ 停车 → 舵机扫 左/中/右 → 选远侧
 *          → 转向绕行 → 原地反向找线 → 找到灰度线 → 回到循迹
 *
 *  调参提示：
 *      FUS_TRIGGER_CM   撞上了才停 → 调大（50）；太早停 → 调小（25）
 *      FUS_TURN_MS      绕完回不到线 → 调大（800）；转过头 → 调小
 *      FUS_SEEK_SPEED   找不到线 → 调大；冲过头 → 调小
 *      FUS_COOLDOWN_MS  刚上线又被触发 → 调大（700）
 *      FUS_SEEK_TIMEOUT_MS 找线最多转多久，超时就先回循迹
 * ========================================================================== */
#define FUS_TRIGGER_CM       35     /* 前方小于它触发避让 */
#define FUS_MIN_GAP_CM       20     /* 两侧都小于它 → 原地掉头 */
#define FUS_TURN_MS         600     /* 转向绕行时长 */
#define FUS_SETTLE_MS       250     /* 舵机到位等待 */
#define FUS_SEEK_SPEED       40     /* 找线转速 */
#define FUS_SEEK_TIMEOUT_MS 2500    /* 找线超时（超时就先回到循迹） */
#define FUS_COOLDOWN_MS     400     /* 绕完冷静期，防止立刻又触发 */
#define FUS_MEAS_PERIOD_MS  100     /* 循迹时多久测一次前方 */

#define FUS_SERVO_LEFT      170     /* 扫左的舵机角 */
#define FUS_SERVO_RIGHT      10     /* 扫右的舵机角 */

/* 融合状态机 */
#define FUS_LINE      0             /* 正常循迹 */
#define FUS_SCAN      1             /* 停车扫描左右 */
#define FUS_TURN      2             /* 转向绕行 */
#define FUS_SEEK      3             /* 原地反向找线 */
#define FUS_COOLDOWN  4             /* 冷静期 */

static u8  fus_state   = FUS_LINE;
static u8  fus_step    = 0;
static u16 fus_timer   = 0;
static u16 fus_tick    = 0;
static u8  fus_dist    = 0;
static u8  fus_dist_l  = 0;
static u8  fus_dist_r  = 0;
static u8  fus_dir_left = 1;        /* 往左绕?  0xFF = 原地掉头 */

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
        fus_state = FUS_LINE;       /* 融合状态复位 */
        fus_timer = 0;
        fus_tick  = 0;
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

/* ============================================================================
 *  融合任务：循迹 + 避障（模式 1）
 *  ----------------------------------------------------------------------------
 *  FUS_LINE     正常循迹；每 FUS_MEAS_PERIOD_MS 测一次前方
 *  FUS_SCAN     停车，舵机扫中/左/右，选距离大的一侧
 *  FUS_TURN     朝选定方向转向绕行 FUS_TURN_MS
 *  FUS_SEEK     原地朝反方向慢转，直到灰度看到线（或超时）
 *  FUS_COOLDOWN 冷静期，防止刚上线又被同一个障碍触发
 * ========================================================================== */
void CombinedTask(void)
{
    u8 d;

    switch (fus_state)
    {
        /* ---------------- 循迹 + 定时测前方 ---------------- */
        case FUS_LINE:
            LED_D4_SET;
            LED_D3_RESET;
            SearchRun();

            if (++fus_tick >= (FUS_MEAS_PERIOD_MS / TICK_MS))
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

            if (fus_timer > 0) { fus_timer -= TICK_MS; break; }

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

            if (fus_timer > TICK_MS) fus_timer -= TICK_MS;
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

            if (fus_timer <= TICK_MS)                /* 超时：先回到循迹 */
            {
                Motor_Stop();
                Gray_ResetIntegral();
                fus_state = FUS_COOLDOWN;
                fus_timer = FUS_COOLDOWN_MS;
                BEEP_RESET;
                LED_D3_RESET;
                break;
            }
            fus_timer -= TICK_MS;

            /* 往反方向转：刚才是左绕，现在就右转找线（反之亦然） */
            if (fus_dir_left == 0xFF || fus_dir_left)
                Motor_SetSpeed(FUS_SEEK_SPEED, -FUS_SEEK_SPEED);   /* 右转找线 */
            else
                Motor_SetSpeed(-FUS_SEEK_SPEED, FUS_SEEK_SPEED);   /* 左转找线 */
            break;

        /* ---------------- 冷静期 ---------------- */
        case FUS_COOLDOWN:
            SearchRun();
            if (fus_timer > TICK_MS) fus_timer -= TICK_MS;
            else fus_state = FUS_LINE;
            break;

        default:
            fus_state = FUS_LINE;
            break;
    }
}

/* ------------------------- 模式调度 ------------------------- */
void ModeTask(void)
{
    switch (keyMode)
    {
        case KEYMODE_1:                     /* 综合：循迹 + 避障 */
            CombinedTask();
            break;

        case KEYMODE_2:                     /* 纯避障（D3 由 AVoidRun 控制） */
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
        printf("mode=%d L=%4d R=%4d | err=%4d corr=%4d spd=%3d ready=%d st=%d dist=%3d ang=%3d fus=%d\r\n",
               keyMode,
               Motor_GetLeft(),
               Motor_GetRight(),
               Gray_GetError(),
               Gray_GetCorrection(),
               Gray_GetBaseSpeed(),
               Gray_IsReady(),
               AVoid_GetState(),
               AVoid_GetDistance(),
               AVoid_GetAngle(),
               fus_state);
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
