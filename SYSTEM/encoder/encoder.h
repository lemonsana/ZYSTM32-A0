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
 *  │ 2  A  │ PC8        │ EXTI8，软件正交解码（4 倍频，中断里只有几行）  │
 *  │ 2  B  │ PC9        │ EXTI9                                        │
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
 *  ！编码器供电：常见模块 5V 供电、A/B 输出 5V 电平。PC6~PC9 **不是 5V 容忍脚**，
 *    所以要么选 3.3V 供电的编码器，要么串 1kΩ 分压 / 加电平转换。
 *    直接接 5V 会烧引脚（或至少读数异常）。
 *
 *  【引脚待定怎么办】整套都在下面这几个宏里，改宏 + 重新编译即可，不用动 .c：
 *      · PC6~PC9 是本工程当前唯一 4 只连续空闲的引脚，先按这套接；
 *      · 换到同一个 GPIO 口就行（比如 PA6/PA7、PB4/PB5）。
 *      · 第二路的 A/B 必须落在同一个 EXTI 组（Pin_5~Pin_9 一组，或 Pin_10~Pin_15
 *        一组），否则要改中断函数名，见 encoder.c 顶部的说明。
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
#define ENC2_PORT             GPIOC
#define ENC2_RCC              RCC_APB2Periph_GPIOC
#define ENC2_A_PIN            GPIO_Pin_8        /* PC8 → EXTI8 */
#define ENC2_B_PIN            GPIO_Pin_9        /* PC9 → EXTI9 */
#define ENC2_A_PIN_SRC        GPIO_PinSource8
#define ENC2_B_PIN_SRC        GPIO_PinSource9
#define ENC2_PORT_SRC         GPIO_PortSourceGPIOC
#define ENC2_A_EXTI_LINE      EXTI_Line8
#define ENC2_B_EXTI_LINE      EXTI_Line9
#define ENC2_IRQn             EXTI9_5_IRQn
#define ENC2_SIGN             1                 /* 装反了改成 -1 */

/* ---------------- 单位换算（不知道就保持 0，只输出计数） ---------------- */
#define ENC_CNT_PER_REV       0     /* 轮子转一圈的计数（已含 4 倍频后的总数） */
#define ENC_WHEEL_MM          65    /* 轮子直径 mm（约值） */
#define ENC_GEAR_X10          100   /* 减速比 ×10：直连 10，1:48 写 480 */

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
