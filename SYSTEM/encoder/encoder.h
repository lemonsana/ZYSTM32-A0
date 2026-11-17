#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ============================================================================
 *  增量式正交编码器驱动  V6.0
 *  ----------------------------------------------------------------------------
 *  【接线方式】AB 两相正交（最常见的那种霍尔/光电编码器，VCC/GND/A/B 四根线）
 *
 *  ┌──────┬────────────┬──────────────────────────────────────────────┐
 *  │ 编码器│ 默认引脚   │ 说明                                          │
 *  ├──────┼────────────┼──────────────────────────────────────────────┤
 *  │ 1  A  │ PC6        │ TIM8_CH1，硬件正交解码（4 倍频，不占 CPU）     │
 *  │ 1  B  │ PC7        │ TIM8_CH2                                     │
 *  │ 2  A  │ PB14       │ EXTI14，软件正交解码（4 倍频，中断里只有几行）│
 *  │ 2  B  │ PB15       │ EXTI15                                       │
 *  └──────┴────────────┴──────────────────────────────────────────────┘
 *
 *  【为什么是这两个组合】TIM8 的 CH1/CH2 默认就在 PC6/PC7，本工程里这两只脚
 *  完全空闲，而且**硬件正交解码不需要 CPU 参与**，是编码器最该用的方式。
 *  但 STM32F103 上能做编码器接口的定时器只有 TIM1~TIM5、TIM8，而本工程：
 *      TIM4 = 电机 PWM        TIM5 = 红外遥控捕获
 *      TIM3 = 舵机 PWM        TIM2 = 原超声波时基（现已空出，但 PA0/PA1 被遥控占用）
 *  只剩下 TIM8 一个。所以第二路编码器走 EXTI 软件解码 —— 小车编码器单相
 *  频率只有几百 Hz，中断开销可以忽略，换来的是不用再抢定时器。
 *
 *  ！编码器供电：松甲编码器 4.5~5V 供电、A/B 是"带上拉输出"（约 5V 电平）。
 *    **本方案的 4 只脚全是 5V 容忍（FT）脚**，5V 直接进 MCU 即可，不用分压：
 *      依据 STM32F103xCDE_DS_CH_V5.pdf 表5：PC6/PC7/PC8/PC9 标 "I/O FT"，
 *      PB2~PB15、PD2、PA8~PA15 也是 FT。
 *    ！但 PC0~PC5 不是 FT（表5 只写 "I/O"）—— 陀螺仪的 PC0/PC1 只能 3.3V。
 *
 *  ！PC9 在这块板子上**没有引出**：ZYSTM32-A0 原理图里 PC9(pin40) 的走线悬空、
 *    没接到任何排针（PC6/PC7/PC8 都在 CN2 上，唯独 PC9 空）。所以第二路不用
 *    PC8/PC9，改成 **PB14/PB15（CN2 的 11/12 脚）**，两只脚同属 EXTI15_10 组。
 *
 *  【引脚待定怎么办】整套都在下面这几个宏里，改宏 + 重新编译即可，不用动 .c：
 *      · 编码器1 = PC6/PC7（TIM8_CH1/CH2，硬件正交，不要改，改了就没硬件解码）；
 *      · 编码器2 = PB14/PB15，两只脚都在 CN2 排针上、都是 FT、都属 EXTI15_10 组；
 *      · 换脚时 A/B 必须落在同一个 EXTI 组（Pin_5~Pin_9 → EXTI9_5_IRQHandler，
 *        Pin_10~Pin_15 → EXTI15_10_IRQHandler），否则要同时改中断函数名。
 *
 *  【计数与换算】
 *      Encoder_GetDeltaL/R()  每 ENC_TICK_MS 的增量（已做 4 倍频）
 *      Encoder_GetSpeedL/R()  折算成"每秒计数"
 *      Encoder_GetTotalL/R()  上电以来的累计计数
 *    要换算成厘米，把 ENC_CNT_PER_REV / ENC_WHEEL_MM / ENC_GEAR_X10 填上，
 *    然后用 Encoder_GetSpeedMmS()。不知道参数就先留 0，只用计数也能做闭环。
 * ========================================================================== */

#define ENC_ENABLE            1     /* 0 = 整个编码器模块不编译 */

/* ---------------- 采样节拍 ---------------- */
#define ENC_TICK_MS           10    /* 必须等于 sched 里的注册周期 */

/* ---------------- 编码器 1：TIM8 硬件正交解码 ---------------- */
#define ENC1_USE_TIM8         1
#define ENC1_PORT             GPIOC
#define ENC1_RCC              RCC_APB2Periph_GPIOC
#define ENC1_A_PIN            GPIO_Pin_6        /* PC6 = TIM8_CH1 */
#define ENC1_B_PIN            GPIO_Pin_7        /* PC7 = TIM8_CH2 */
#define ENC1_SIGN             1                 /* 装反了（前进时计数变小）改成 -1 */

/* ---------------- 编码器 2：EXTI 软件正交解码 ---------------- */
#define ENC2_ENABLE           1
#define ENC2_PORT             GPIOB
#define ENC2_RCC              RCC_APB2Periph_GPIOB
#define ENC2_A_PIN            GPIO_Pin_14       /* PB14 → EXTI14（CN2 第 11 脚） */
#define ENC2_B_PIN            GPIO_Pin_15       /* PB15 → EXTI15（CN2 第 12 脚） */
#define ENC2_A_PIN_SRC        GPIO_PinSource14
#define ENC2_B_PIN_SRC        GPIO_PinSource15
#define ENC2_PORT_SRC         GPIO_PortSourceGPIOB
#define ENC2_A_EXTI_LINE      EXTI_Line14
#define ENC2_B_EXTI_LINE      EXTI_Line15
#define ENC2_IRQn             EXTI15_10_IRQn
#define ENC2_SIGN             1                 /* 装反了改成 -1 */

/* ---------------- 单位换算（本车实测值，见 docs/陀螺仪与编码器.md） ----------------
 * 松甲「霍尔编码器 TT 马达」：13 线霍尔，AB 正交，输出带上拉，编码器供电 4.5~5V。
 *   1:48  → 13 × 48 × 4 = 2496 计数 / 轮子一圈   ← 本车用这个
 *   1:90  → 13 × 90 × 4 = 4680 计数 / 轮子一圈
 * 注意：2496 是"减速箱之后"的轴（也就是轮子）转一圈的计数，所以 GEAR_X10 保持 100。
 *   另一种填法等价的：ENC_CNT_PER_REV = 52（13×4，电机轴一圈）、ENC_GEAR_X10 = 480。
 * 填 0 时只输出计数，Encoder_GetSpeedMmS() 返回 -1。 */
#define ENC_CNT_PER_REV       2496  /* 轮子一圈的计数（已含 4 倍频 + 减速比） */
#define ENC_WHEEL_MM          65    /* 轮子直径 mm（TT 马达标配 65mm 轮） */
#define ENC_GEAR_X10          100   /* 额外减速 ×10：计数已含减速箱就写 100 */

/* ---------------- 对外接口 ---------------- */
void    Encoder_Init(void);
void    Encoder_Task(void);             /* 每 ENC_TICK_MS 调一次：算增量 / 速度 */

int16_t Encoder_GetDeltaL(void);        /* 本拍增量（4 倍频计数） */
int16_t Encoder_GetDeltaR(void);
int16_t Encoder_GetSpeedL(void);        /* 计数/秒 */
int16_t Encoder_GetSpeedR(void);
int32_t Encoder_GetTotalL(void);
int32_t Encoder_GetTotalR(void);
int32_t Encoder_GetSpeedMmS(void);      /* 平均线速度 mm/s；换算宏没填就返回 -1 */

void    Encoder_Reset(void);            /* 计数清零 */
u8      Encoder_GetSecondReady(void);   /* 1 = 第二路（软件解码）已使能 */

#endif
