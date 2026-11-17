#include "encoder.h"
#include "delay.h"

#if ENC_ENABLE

/* ============================================================================
 *  增量式正交编码器驱动  V6.1
 *  ----------------------------------------------------------------------------
 *  两路都走 EXTI 软件正交解码（原因见 encoder.h：PC6/PC7/PB14/PB15 在实物板上
 *  接不出来，能确定用的是 P2 那排 —— 舵机 PB0 就在同一排）。
 *
 *      编码器 1：PA0 / PA2  → EXTI0 / EXTI2
 *      编码器 2：PA7 / PA8  → EXTI7 / EXTI8（同属 EXTI9_5 组，共用一个入口）
 *
 *  原理：A/B 每根线都开双边沿中断，中断里读两相电平拼成 4 位索引查表得 ±1，
 *  这样 4 个边沿全计数 = 4 倍频。小车编码器单相几百 Hz~14kHz，一次中断 2us 以内。
 *
 *  ！换引脚注意：A/B 落在哪个 EXTI 组决定中断函数名（见 encoder.h 注释），
 *    两路不要占用同一条 EXTI 线。
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

/* 读一次 A/B 电平，返回 (A<<1)|B */
static u8 enc_read_ab(GPIO_TypeDef *port, uint16_t a_pin, uint16_t b_pin)
{
    return (u8)(((GPIO_ReadInputDataBit(port, a_pin) ? 1u : 0u) << 1)
              |  (GPIO_ReadInputDataBit(port, b_pin) ? 1u : 0u));
}

/* ---------------------------- 编码器 1 状态 ---------------------------- */
static volatile int32_t enc1_total = 0;     /* 中断里累加 */
static volatile u8      enc1_prev  = 0;     /* 上一拍 AB */
static int32_t enc1_last   = 0;
static int16_t enc1_delta  = 0;
static int16_t enc1_speed  = 0;
#if ENC1_USE_TIM8
static int16_t enc1_tim_last = 0;           /* TIM8 模式下上一拍的 CNT */
#endif

/* ---------------------------- 编码器 2 状态 ---------------------------- */
#if ENC2_ENABLE
static volatile int32_t enc2_total = 0;
static volatile u8      enc2_prev  = 0;
static int32_t enc2_last  = 0;
static int16_t enc2_delta = 0;
static int16_t enc2_speed = 0;
static u8      enc2_ready = 0;
#endif

/* ======================= 编码器 1：TIM8 硬件正交 ======================= */

#if ENC1_USE_TIM8

static void enc1_init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB2PeriphClockCmd(ENC1_TIM_RCC | RCC_APB2Periph_TIM8, ENABLE);

    gpio.GPIO_Pin   = ENC1_A_TIM_CH | ENC1_B_TIM_CH;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC1_TIM_PORT, &gpio);

    tb.TIM_Period        = 0xFFFF;          /* 16 位满量程，让它自由回绕 */
    tb.TIM_Prescaler     = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &tb);

    /* TI12 = A/B 两相都参与计数（4 倍频） */
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* IC1F/IC2F = 0xA：8 个采样点、约 6.7us 的毛刺滤除（电机 PWM 尖峰很大） */
    TIM8->CCMR1 |= (uint16_t)((0x0Au << 4) | (0x0Au << 12));

    TIM_SetCounter(TIM8, 0);
    TIM_Cmd(TIM8, ENABLE);
}

#else   /* ---------------- 编码器 1：EXTI 软件正交 ---------------- */

static void enc1_init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(ENC1_RCC | RCC_APB2Periph_AFIO, ENABLE);

    /* 上拉输入：没接线时读数稳定为高，不会因悬空抖动刷出假计数 */
    gpio.GPIO_Pin   = ENC1_A_PIN | ENC1_B_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC1_PORT, &gpio);

    GPIO_EXTILineConfig(ENC1_PORT_SRC, ENC1_A_PIN_SRC);
    GPIO_EXTILineConfig(ENC1_PORT_SRC, ENC1_B_PIN_SRC);

    exti.EXTI_Line    = ENC1_A_EXTI_LINE | ENC1_B_EXTI_LINE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    /* 双沿：4 倍频的前提 */
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = ENC1_A_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    if (ENC1_B_IRQn != ENC1_A_IRQn)         /* A/B 不同组时还要开第二个通道 */
    {
        nvic.NVIC_IRQChannel = ENC1_B_IRQn;
        NVIC_Init(&nvic);
    }

    /* 记下初始相位，避免上电时把"当前状态"误判成一次跳变 */
    enc1_prev  = enc_read_ab(ENC1_PORT, ENC1_A_PIN, ENC1_B_PIN);
    enc1_total = 0;
    enc1_last  = 0;
}

/* A/B 中断的公共处理体 */
static void enc1_isr(void)
{
    u8 cur;

    if ((EXTI_GetITStatus(ENC1_A_EXTI_LINE) == RESET) &&
        (EXTI_GetITStatus(ENC1_B_EXTI_LINE) == RESET))
    {
        return;
    }

    cur = enc_read_ab(ENC1_PORT, ENC1_A_PIN, ENC1_B_PIN);
    enc1_total += (int32_t)enc_quad[((enc1_prev & 0x03u) << 2) | cur] * ENC1_SIGN;
    enc1_prev   = cur;

    /* 两根线的标志都要清，否则会一直重进中断 */
    EXTI_ClearITPendingBit(ENC1_A_EXTI_LINE | ENC1_B_EXTI_LINE);
}

void EXTI0_IRQHandler(void) { enc1_isr(); }
void EXTI2_IRQHandler(void) { enc1_isr(); }

#endif /* ENC1_USE_TIM8 */

/* ======================= 编码器 2：EXTI 软件正交 ======================= */

#if ENC2_ENABLE

static void enc2_init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(ENC2_RCC | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin   = ENC2_A_PIN | ENC2_B_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC2_PORT, &gpio);

    GPIO_EXTILineConfig(ENC2_PORT_SRC, ENC2_A_PIN_SRC);
    GPIO_EXTILineConfig(ENC2_PORT_SRC, ENC2_B_PIN_SRC);

    exti.EXTI_Line    = ENC2_A_EXTI_LINE | ENC2_B_EXTI_LINE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = ENC2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    enc2_prev  = enc_read_ab(ENC2_PORT, ENC2_A_PIN, ENC2_B_PIN);
    enc2_total = 0;
    enc2_last  = 0;
    enc2_ready = 1;
}

/* PA7/PA8 同属 EXTI9_5 组，两根线共用这一个入口 */
void EXTI9_5_IRQHandler(void)
{
    u8 cur;

    if ((EXTI_GetITStatus(ENC2_A_EXTI_LINE) == RESET) &&
        (EXTI_GetITStatus(ENC2_B_EXTI_LINE) == RESET))
    {
        return;
    }

    cur = enc_read_ab(ENC2_PORT, ENC2_A_PIN, ENC2_B_PIN);
    enc2_total += (int32_t)enc_quad[((enc2_prev & 0x03u) << 2) | cur] * ENC2_SIGN;
    enc2_prev   = cur;

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

    enc1_init();

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
        int16_t d   = (int16_t)(now - enc1_tim_last);   /* 补码减法，回绕自动正确 */
        enc1_tim_last = now;
        enc1_delta = (int16_t)(d * ENC1_SIGN);
        enc1_total += enc1_delta;
    }
#else
    {
        /* 32 位对齐读在 Cortex-M3 上是原子的，读一次即可；
         * 用"当前累计 - 上次累计"求增量，中断里就不用管什么 dt */
        int32_t t = enc1_total;
        int32_t d = t - enc1_last;

        enc1_last  = t;
        enc1_delta = (int16_t)d;
    }
#endif
    enc1_speed = (int16_t)(enc1_delta * (1000 / ENC_TICK_MS));

#if ENC2_ENABLE
    {
        int32_t t = enc2_total;
        int32_t d = t - enc2_last;

        enc2_last  = t;
        enc2_delta = (int16_t)d;
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
    int32_t rev_x1000  = cnt_per_s * 1000 / ENC_CNT_PER_REV; /* 轮子转速 ×1000 */
    int32_t out_x1000  = rev_x1000 * 10 / ENC_GEAR_X10;      /* 除额外减速比 ×1000 */

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
    enc1_last  = 0;
#if ENC1_USE_TIM8
    TIM_SetCounter(TIM8, 0);
    enc1_tim_last = 0;
#endif

#if ENC2_ENABLE
    enc2_delta = 0;
    enc2_speed = 0;
    enc2_total = 0;
    enc2_last  = 0;
#endif
}

#endif /* ENC_ENABLE */
