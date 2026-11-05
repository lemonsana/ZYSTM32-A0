#include "IRAvoid.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "stm32f10x.h"

/* ============================================================================
 *  避障模块  v3.5 实现
 *  ==========================================================================
 *  分层：
 *      1) 超声波驱动（HC-SR04）   TIM2 计数 + EXTI1 双沿，非阻塞，30ms 超时
 *      2) 舵机驱动                 TIM3_CH3 / PB0，50Hz
 *      3) 避障状态机              两版二选一（AVOID_SCAN_WHILE_DRIVING）
 *
 *  超声波测距原理：
 *      TRIG 给 15us 高电平 → 模块发 8 个 40kHz 脉冲
 *      → ECHO 拉高（上升沿）开始计时 → ECHO 拉低（下降沿）结束
 *      → 高电平时间 t(us) 与距离的关系：distance_cm = t / 58
 *      （等价于 t × 0.017，声速 340m/s，往返除以 2）
 *
 *  为什么用 EXTI 双沿而不是"中断里死等"：
 *      厂商例程在 EXTI 中断里 while(ECHO 高) 死等，没有回波时（前方无遮挡、
 *      超出量程、模块没接）会永远卡在中断里，整机死机。
 *      这里改成：上升沿记录起点、下降沿计算宽度，主循环最多等 30ms，
 *      超时就判"无效"，永远不会卡死。
 * ========================================================================== */

/* ---------------------------- 引脚 ---------------------------- */
#define ULTRA_TRIG_PORT        GPIOC
#define ULTRA_TRIG_PIN         GPIO_Pin_0      /* PC0  TRIG */
#define ULTRA_ECHO_PORT        GPIOC
#define ULTRA_ECHO_PIN         GPIO_Pin_1      /* PC1  ECHO */
#define SERVO_PORT             GPIOB
#define SERVO_PIN              GPIO_Pin_0      /* PB0  TIM3_CH3 */

/* ---------------------- 超声波测量状态 ---------------------- */
static volatile u8  ultra_stage = 0;    /* 0=空闲  1=等上升沿  2=等下降沿 */
static volatile u16 ultra_start = 0;    /* 上升沿时刻（TIM2 计数，1us） */
static volatile u16 ultra_width = 0;    /* 回波高电平宽度（us） */
static volatile u8  ultra_done  = 0;    /* 1=本次测量完成 */

/* ---------------------- 舵机 ---------------------- */
static u8 av_angle = AV_SERVO_CENTER;

/* ---------------------- 状态机 ---------------------- */
#define AV_IDLE      0
#define AV_SCAN      1
#define AV_TURN      2
#define AV_RECOVER   3

static u8  av_state      = AV_IDLE;
static u8  av_step       = 0;
static u16 av_timer      = 0;      /* 当前阶段倒计时(ms) */
static u16 av_tick_cnt   = 0;      /* 测量周期计数 */
static u8  av_dist       = 0;      /* 前方距离 */
static u8  av_dist_l     = 0;      /* 左侧距离 */
static u8  av_dist_r     = 0;      /* 右侧距离 */
static u8  av_turn_left  = 1;      /* 本次往左转? */
static u8  av_sweep_dir  = 1;      /* 边走边扫：舵机扫描方向 +1/-1 */
static u8  av_best_angle = AV_SERVO_CENTER;  /* 边走边扫：最空的角度 */
static u8  av_best_dist  = 0;
static u8  av_phase      = 0;      /* 边走边扫：0=转舵机 1=测距 */

/* ==========================================================================
 *  超声波驱动
 * ========================================================================== */

/* 初始化：PC0 推挽输出、PC1 下拉输入 + EXTI1 双沿、TIM2 1MHz 计数 */
static void Ultra_Init(void)
{
    GPIO_InitTypeDef        gpio;
    EXTI_InitTypeDef        exti;
    NVIC_InitTypeDef        nvic;
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TRIG：PC0 推挽输出，初始低 */
    gpio.GPIO_Pin   = ULTRA_TRIG_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRA_TRIG_PORT, &gpio);
    GPIO_ResetBits(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);

    /* ECHO：PC1 下拉输入 */
    gpio.GPIO_Pin  = ULTRA_ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(ULTRA_ECHO_PORT, &gpio);

    /* EXTI1：PC1 上升沿 + 下降沿都触发 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    exti.EXTI_Line    = EXTI_Line1;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = EXTI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* TIM2：72MHz / (71+1) = 1MHz，1 个计数 = 1us，16 位自由计数 */
    tb.TIM_Period        = 0xFFFF;
    tb.TIM_Prescaler     = 71;
    tb.TIM_ClockDivision = 0;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tb);
    TIM_Cmd(TIM2, ENABLE);
}

/* EXTI1：ECHO 双沿中断
 * 上升沿：记下起点
 * 下降沿：算出宽度，标记完成
 * 注意：这里只做最少的事，绝不死等（这是厂商例程最大的坑）
 */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if (GPIO_ReadInputDataBit(ULTRA_ECHO_PORT, ULTRA_ECHO_PIN))   /* 上升沿 */
        {
            if (ultra_stage == 1)
            {
                ultra_start = (u16)TIM2->CNT;
                ultra_stage = 2;
            }
        }
        else                                                          /* 下降沿 */
        {
            if (ultra_stage == 2)
            {
                ultra_width = (u16)((u16)TIM2->CNT - ultra_start);    /* 16 位自动回绕 */
                ultra_done  = 1;
                ultra_stage = 0;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/* 发起一次测距并等待结果
 * 返回：距离(cm)，0 = 超时/无回波/超量程（都视为"没有障碍"或"无效"）
 * 阻塞时间：正常 1~20ms，最坏 30ms（有超时保护）
 */
static u8 Ultra_Measure(void)
{
    u16 wait;

    /* 上一次没结束的残留先清掉 */
    ultra_done  = 0;
    ultra_width = 0;
    ultra_stage = 1;

    /* 15us 触发脉冲 */
    GPIO_SetBits(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);
    delay_us(15);
    GPIO_ResetBits(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);

    /* 最多等 30ms（300 × 100us） */
    for (wait = 0; wait < (AV_MEAS_TIMEOUT_US / 100); wait++)
    {
        if (ultra_done) break;
        delay_us(100);
    }

    if (!ultra_done)
    {
        ultra_stage = 0;
        return 0;
    }

    /* 宽度(us) → 厘米：t / 58 */
    {
        u32 cm = (u32)ultra_width / 58;
        if (cm == 0 || cm > 200) return 0;      /* 0 或超量程（>2m）都算无效 */
        return (u8)cm;
    }
}

/* ==========================================================================
 *  舵机驱动（TIM3_CH3 / PB0，50Hz）
 * ========================================================================== */

static void Servo_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB0 = TIM3_CH3，复用推挽 */
    gpio.GPIO_Pin   = SERVO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PORT, &gpio);

    /* 1MHz 计数，20ms 周期（ARR=19999） */
    tb.TIM_Period        = 19999;
    tb.TIM_Prescaler     = 71;
    tb.TIM_ClockDivision = 0;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 1500;              /* 中位 1.5ms */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC3Init(TIM3, &oc);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/* 角度 0~180 → 脉宽 500~2500us */
static void Servo_SetAngle(u8 deg)
{
    u16 ccr;
    if (deg > 180) deg = 180;
    ccr = (u16)(500 + (u16)deg * 2000 / 180);
    TIM_SetCompare3(TIM3, ccr);
    av_angle = deg;
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */
u8 AVoid_GetState(void)    { return av_state; }
u8 AVoid_GetDistance(void) { return av_dist; }
u8 AVoid_GetAngle(void)    { return av_angle; }

/* 供融合模式复用：测距 / 设舵机角 */
u8 AVoid_Measure(void)      { return Ultra_Measure(); }
void AVoid_Servo(u8 deg)    { Servo_SetAngle(deg); }

void IRAvoidInit(void)
{
    Ultra_Init();
    Servo_Init();

    av_state      = AV_IDLE;
    av_step       = 0;
    av_timer      = 0;
    av_tick_cnt   = 0;
    av_dist       = 0;
    av_turn_left  = 1;
    av_sweep_dir  = 1;
    av_best_angle = AV_SERVO_CENTER;
    av_best_dist  = 0;

    Servo_SetAngle(AV_SERVO_CENTER);
    delay_ms(300);              /* 等舵机归中 */
}

/* ==========================================================================
 *  版本一：停车扫描（AVOID_SCAN_WHILE_DRIVING = 0）
 * ==========================================================================
 *  IDLE 直行 ──前方<AV_TRIGGER_CM──→ 停车
 *      → SCAN：中位测前方 → 左极限测左 → 右极限测右
 *      → 选距离大的一侧（两侧都 <AV_MIN_GAP_CM 就原地掉头）
 *      → TURN 转向 → RECOVER 回正 → IDLE
 */
#if !AVOID_SCAN_WHILE_DRIVING

static void AVoid_RunStopScan(void)
{
    switch (av_state)
    {
        /* ---------------- 直行，定时量前方 ---------------- */
        case AV_IDLE:
            Motor_SetSpeed(AV_BASE_SPEED, AV_BASE_SPEED);

            if (++av_tick_cnt >= (AV_MEAS_PERIOD_MS / 10))
            {
                av_tick_cnt = 0;
                av_dist = Ultra_Measure();

                if (av_dist && av_dist < AV_TRIGGER_CM)      /* 有障碍 */
                {
                    Motor_Stop();                            /* 先停稳再扫，测距最准 */
                    Servo_SetAngle(AV_SERVO_CENTER);
                    av_state = AV_SCAN;
                    av_step  = 0;
                    av_timer = AV_SETTLE_MS;
                }
            }
            break;

        /* ---------------- 扫描三个方向 ---------------- */
        case AV_SCAN:
            if (av_timer > 0) { av_timer -= 10; break; }

            if (av_step == 0)                                /* 中位：已就绪，测前方 */
            {
                av_dist = Ultra_Measure();
                Servo_SetAngle(AV_SERVO_LEFT);
                av_step  = 1;
                av_timer = AV_SETTLE_MS;
            }
            else if (av_step == 1)                           /* 左侧 */
            {
                av_dist_l = Ultra_Measure();
                Servo_SetAngle(AV_SERVO_RIGHT);
                av_step  = 2;
                av_timer = AV_SETTLE_MS;
            }
            else                                             /* 右侧，然后决策 */
            {
                av_dist_r = Ultra_Measure();
                Servo_SetAngle(AV_SERVO_CENTER);

                if (av_dist_l < AV_MIN_GAP_CM && av_dist_r < AV_MIN_GAP_CM)
                {
                    av_turn_left = 0xFF;                     /* 特殊值：原地掉头 */
                }
                else
                {
                    /* 无效(0)按"很远"处理 */
                    av_turn_left = ((av_dist_l ? av_dist_l : 200) >=
                                    (av_dist_r ? av_dist_r : 200)) ? 1 : 0;
                }

                BEEP_SET;
                LED_D3_SET;
                av_state = AV_TURN;
                av_timer = AV_TURN_MS;
            }
            break;

        /* ---------------- 转向 ---------------- */
        case AV_TURN:
            if (av_turn_left == 0xFF)
                Motor_SetSpeed(-AV_SPIN_SPEED, AV_SPIN_SPEED);   /* 原地左掉头 */
            else if (av_turn_left)
                Motor_SetSpeed(AV_TURN_INNER, AV_TURN_OUTER);    /* 左转 */
            else
                Motor_SetSpeed(AV_TURN_OUTER, AV_TURN_INNER);    /* 右转 */

            if (av_timer > 0) av_timer -= 10;
            else { av_state = AV_RECOVER; av_timer = AV_RECOVER_MS; }
            break;

        /* ---------------- 回正 ---------------- */
        case AV_RECOVER:
            Motor_SetSpeed(AV_BASE_SPEED, AV_BASE_SPEED);
            if (av_timer > 0) av_timer -= 10;
            else
            {
                av_state    = AV_IDLE;
                av_tick_cnt = 0;
                BEEP_RESET;
                LED_D3_RESET;
            }
            break;

        default:
            av_state = AV_IDLE;
            break;
    }
}

#endif  /* !AVOID_SCAN_WHILE_DRIVING */

/* ==========================================================================
 *  版本二：边走边扫（AVOID_SCAN_WHILE_DRIVING = 1）
 * ==========================================================================
 *  车一直在走，舵机在 [中位 ± AV_SWEEP_HALF] 之间往返扫：
 *      phase0：把舵机转到当前扫描角
 *      phase1：等 AV_SETTLE_MS 后测距，记录"最空角度"
 *  前方 ±20° 内距离 < AV_TRIGGER_CM 时：
 *      朝最近一次扫到的"最空那一侧"直接弧线转过去（不停车）
 *  扫描范围比停车版窄（±40°）：因为行驶中舵机转太快会影响测距稳定性。
 */
#if AVOID_SCAN_WHILE_DRIVING

static void AVoid_RunDriveScan(void)
{
    static u16 turn_timer = 0;
    u8 d;

    switch (av_state)
    {
        /* ---------------- 直行 + 扫描 ---------------- */
        case AV_IDLE:
            Motor_SetSpeed(AV_BASE_SPEED, AV_BASE_SPEED);

            if (av_timer > 0)
            {
                av_timer -= 10;
                break;
            }

            if (av_phase == 0)                               /* 转舵机 */
            {
                Servo_SetAngle(av_angle);
                av_phase = 1;
                av_timer = AV_SETTLE_MS;
            }
            else                                             /* 测距 + 走下一步 */
            {
                d = Ultra_Measure();

                if (d)
                {
                    if (d > av_best_dist)                    /* 记录最空方向 */
                    {
                        av_best_dist  = d;
                        av_best_angle = av_angle;
                    }

                    /* 前方 ±20° 内太近 → 决定往哪边躲 */
                    if (av_angle >= (AV_SERVO_CENTER - 20) &&
                        av_angle <= (AV_SERVO_CENTER + 20) &&
                        d < AV_TRIGGER_CM)
                    {
                        av_turn_left = (av_best_angle <= AV_SERVO_CENTER) ? 1 : 0;
                        av_state  = AV_TURN;
                        turn_timer = AV_TURN_MS;
                        BEEP_SET;
                        LED_D3_SET;
                        break;
                    }
                }

                av_phase = 0;

                /* 扫描角往返：中位 ± AV_SWEEP_HALF，步进 10° */
                if (av_sweep_dir > 0)
                {
                    if (av_angle + 10 >= AV_SERVO_CENTER + AV_SWEEP_HALF)
                    {
                        av_sweep_dir = 0;
                    }
                    else av_angle += 10;
                }
                else
                {
                    if (av_angle <= AV_SERVO_CENTER - AV_SWEEP_HALF)
                    {
                        av_sweep_dir = 1;
                    }
                    else av_angle -= 10;
                }

                av_timer = 20;      /* 两次测量之间留一点空档，别把主循环占满 */
            }
            break;

        /* ---------------- 边走边转 ---------------- */
        case AV_TURN:
            if (av_turn_left)
                Motor_SetSpeed(AV_TURN_INNER, AV_TURN_OUTER);
            else
                Motor_SetSpeed(AV_TURN_OUTER, AV_TURN_INNER);

            if (turn_timer > 10) turn_timer -= 10;
            else
            {
                av_state     = AV_RECOVER;
                av_timer     = AV_RECOVER_MS;
                av_best_dist = 0;
                av_best_angle = AV_SERVO_CENTER;
                av_angle     = AV_SERVO_CENTER;
                Servo_SetAngle(AV_SERVO_CENTER);
            }
            break;

        /* ---------------- 回正后继续扫 ---------------- */
        case AV_RECOVER:
            Motor_SetSpeed(AV_BASE_SPEED, AV_BASE_SPEED);
            if (av_timer > 10) av_timer -= 10;
            else
            {
                av_state = AV_IDLE;
                av_phase = 0;
                av_timer = 0;
                av_angle = AV_SERVO_CENTER;
                Servo_SetAngle(AV_SERVO_CENTER);
                BEEP_RESET;
                LED_D3_RESET;
            }
            break;

        default:
            av_state = AV_IDLE;
            break;
    }
}

#endif  /* AVOID_SCAN_WHILE_DRIVING */

/* ==========================================================================
 *  统一入口
 * ========================================================================== */
void AVoidRun(void)
{
#if AVOID_SCAN_WHILE_DRIVING
    AVoid_RunDriveScan();
#else
    AVoid_RunStopScan();
#endif
}
