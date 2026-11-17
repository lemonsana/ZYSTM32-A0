#include "encoder.h"
#include "delay.h"

#if ENC_ENABLE

/* ============================================================================
 *  增量式正交编码器驱动
 *  ----------------------------------------------------------------------------
 *  编码器 1：TIM8 硬件正交解码（PC6/PC7）
 *      定时器自己按 A/B 的相位关系加减计数，CPU 完全不参与，4 倍频。
 *      读的时候取 CNT 和上次的差，用 16 位补码回绕 —— 10ms 内不可能转 32767 个
 *      计数，所以这么做是安全的。
 *
 *  编码器 2：EXTI 软件正交解码（PB14/PB15）
 *      A/B 每一根线都开双边沿中断，中断里读两相电平拼成 4 位索引查表得 ±1。
 *      小车编码器单相频率几百 Hz，一次中断不到 2us，占用可以忽略。
 *
 *  ！换引脚注意事项：
 *      · 第二路 A/B 必须落在同一个 EXTI 组，中断函数名才不用改：
 *        Pin_5~Pin_9 → EXTI9_5_IRQHandler
 *        Pin_10~Pin_15 → EXTI15_10_IRQHandler（本文件实现的就是这一个）
 *      · 两路不要用到同一个 EXTI 线（比如都用了 Pin_8），否则会互相抢中断。
 * ========================================================================== */

/* 软件正交解码查表：索引 = (上一拍AB << 2) | 这一拍AB，AB = (A<<1)|B
 * 值为 +1 / -1 / 0（0 = 非法跳变，比如两相同时变，直接丢弃） */
static const int8_t enc_quad[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* ---------------------------- 运行时状态 ---------------------------- */
static int16_t enc1_delta  = 0;
static int16_t enc1_speed  = 0;
static int32_t enc1_total  = 0;
static int16_t enc1_last   = 0;         /* 上一拍的定时器计数值 */

#if ENC2_ENABLE
static volatile int32_t enc2_total = 0; /* 中断里累加 */
static volatile u8      enc2_prev  = 0; /* 上一拍 AB */
static int32_t enc2_last  = 0;
static int16_t enc2_delta = 0;
static int16_t enc2_speed = 0;
static u8      enc2_ready = 0;
#endif

/* ============================ 编码器 1：TIM8 ============================ */

static void enc1_init(void)
{
    GPIO_InitTypeDef      gpio;
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB2PeriphClockCmd(ENC1_RCC | RCC_APB2Periph_TIM8, ENABLE);

    /* A/B 接上拉输入：松甲编码器是"带上拉输出"，两种接法都能读；
     * 内部 40k 上拉对新旧编码器都无害，且断线时不会浮空乱计数，故统一用 IPU */
    gpio.GPIO_Pin   = ENC1_A_PIN | ENC1_B_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC1_PORT, &gpio);

    tb.TIM_Period        = 0xFFFF;          /* 16 位满量程，让它自由回绕 */
    tb.TIM_Prescaler     = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &tb);

    /* TI12 = A/B 两相都参与计数（4 倍频）；
     * 两相都取上升沿不滤波 —— 分辨率最高，靠下面 CCMR1 的输入滤波抗干扰 */
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* IC1F/IC2F = 0xA：8 个采样点、约 6.7us 的毛刺滤除。电机 PWM 的尖峰
     * 很容易窜到编码器线上，这几位的收益比想象中大。 */
    TIM8->CCMR1 |= (uint16_t)((0x0Au << 4) | (0x0Au << 12));

    TIM_SetCounter(TIM8, 0);
    TIM_Cmd(TIM8, ENABLE);
}

/* ============================ 编码器 2：EXTI ============================ */

#if ENC2_ENABLE

static void enc2_init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(ENC2_RCC | RCC_APB2Periph_AFIO, ENABLE);

    /* 上拉输入：编码器没接线时读数稳定为高，不会因为悬空抖动刷出一堆假计数 */
    gpio.GPIO_Pin   = ENC2_A_PIN | ENC2_B_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC2_PORT, &gpio);

    GPIO_EXTILineConfig(ENC2_PORT_SRC, ENC2_A_PIN_SRC);
    GPIO_EXTILineConfig(ENC2_PORT_SRC, ENC2_B_PIN_SRC);

    exti.EXTI_Line    = ENC2_A_EXTI_LINE | ENC2_B_EXTI_LINE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    /* 双沿：4 倍频的前提 */
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = ENC2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;         /* 比电机控制低，比打印高 */
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* 记下初始相位，避免上电时把"当前状态"误判成一次跳变 */
    enc2_prev = (u8)(((GPIO_ReadInputDataBit(ENC2_PORT, ENC2_A_PIN) ? 1u : 0u) << 1)
                   |  (GPIO_ReadInputDataBit(ENC2_PORT, ENC2_B_PIN) ? 1u : 0u));
    enc2_total = 0;
    enc2_last  = 0;
    enc2_ready = 1;
}

/* A/B 两根线共用这一个中断入口（PB14/PB15 同属 EXTI15_10 组） */
void EXTI15_10_IRQHandler(void)
{
    u8 a, b, cur;

    if ((EXTI_GetITStatus(ENC2_A_EXTI_LINE) == RESET) &&
        (EXTI_GetITStatus(ENC2_B_EXTI_LINE) == RESET))
    {
        return;
    }

    a = (u8)(GPIO_ReadInputDataBit(ENC2_PORT, ENC2_A_PIN) ? 1u : 0u);
    b = (u8)(GPIO_ReadInputDataBit(ENC2_PORT, ENC2_B_PIN) ? 1u : 0u);
    cur = (u8)((a << 1) | b);

    enc2_total += (int32_t)enc_quad[((enc2_prev & 0x03u) << 2) | cur] * ENC2_SIGN;
    enc2_prev = cur;

    /* 两根线的标志都要清，否则会一直重进中断 */
    EXTI_ClearITPendingBit(ENC2_A_EXTI_LINE | ENC2_B_EXTI_LINE);
}

#endif /* ENC2_ENABLE */

/* ============================== 初始化 ============================== */

void Encoder_Init(void)
{
    enc1_delta = 0;
    enc1_speed = 0;
    enc1_total = 0;
    enc1_last  = 0;

#if ENC1_USE_TIM8
    enc1_init();
#endif

#if ENC2_ENABLE
    enc2_delta = 0;
    enc2_speed = 0;
    enc2_init();
#endif
}

/* ============================== 周期任务 ============================== */

void Encoder_Task(void)
{
#if ENC1_USE_TIM8
    {
        int16_t now = (int16_t)(u16)TIM8->CNT;      /* 只在 0~65535 里取值 */
        int16_t d   = (int16_t)(now - enc1_last);   /* 补码减法，回绕自动正确 */
        enc1_last  = now;
        enc1_delta = (int16_t)(d * ENC1_SIGN);
        enc1_total += enc1_delta;
        enc1_speed = (int16_t)(enc1_delta * (1000 / ENC_TICK_MS));
    }
#endif

#if ENC2_ENABLE
    {
        /* 32 位对齐读在 Cortex-M3 上是原子的，读一次即可；
         * 用"当前累计 - 上次累计"求增量，中断里就不用管什么 dt */
        int32_t t = enc2_total;
        int32_t d = t - enc2_last;

        enc2_last  = t;
        enc2_delta = (int16_t)(d * ENC2_SIGN);
        enc2_speed = (int16_t)(enc2_delta * (1000 / ENC_TICK_MS));
    }
#endif
}

/* ============================== 取值接口 ============================== */

int16_t Encoder_GetDeltaL(void) { return enc1_delta; }
int16_t Encoder_GetSpeedL(void) { return enc1_speed; }
int32_t Encoder_GetTotalL(void) { return enc1_total; }

#if ENC2_ENABLE
int16_t Encoder_GetDeltaR(void) { return enc2_delta; }
int16_t Encoder_GetSpeedR(void) { return enc2_speed; }
int32_t Encoder_GetTotalR(void) { return enc2_total; }
u8      Encoder_GetSecondReady(void) { return enc2_ready; }
#else
int16_t Encoder_GetDeltaR(void) { return 0; }
int16_t Encoder_GetSpeedR(void) { return 0; }
int32_t Encoder_GetTotalR(void) { return 0; }
u8      Encoder_GetSecondReady(void) { return 0; }
#endif

/* 平均线速度 mm/s。
 * 换算链：计数 → 转数（÷ ENC_CNT_PER_REV）→ 输出轴转数（÷ 减速比）
 *         → 周长（× π × 直径）→ 每秒
 * 换算宏没填（ENC_CNT_PER_REV == 0）时返回 -1，提示"还没标定"。 */
int32_t Encoder_GetSpeedMmS(void)
{
#if (ENC_CNT_PER_REV > 0)
    int32_t cnt_per_s = ((int32_t)enc1_speed
#if ENC2_ENABLE
                       + (int32_t)enc2_speed
#endif
                       ) / 2;
    int32_t mm_per_rev = (31416 * ENC_WHEEL_MM) / 10000;    /* 一圈走多少 mm（π×直径） */
    int32_t rev_x1000  = cnt_per_s * 1000 / ENC_CNT_PER_REV; /* 电机轴转速 ×1000 */
    int32_t out_x1000  = rev_x1000 * 10 / ENC_GEAR_X10;      /* 除减速比 → 轮子转速 ×1000 */

    return out_x1000 * mm_per_rev / 1000;
#else
    return -1;
#endif
}

void Encoder_Reset(void)
{
    enc1_delta = 0;
    enc1_speed = 0;
    enc1_total = 0;
#if ENC1_USE_TIM8
    TIM_SetCounter(TIM8, 0);
    enc1_last = 0;
#endif

#if ENC2_ENABLE
    enc2_delta = 0;
    enc2_speed = 0;
    enc2_total = 0;
    enc2_last  = 0;
#endif
}

#endif /* ENC_ENABLE */
