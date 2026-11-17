#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ============================================================================
 *  增量式正交编码器驱动  V6.1
 *  ----------------------------------------------------------------------------
 *  【接线方式】AB 两相正交（最常见的那种霍尔/光电编码器，VCC/GND/A/B 四根线）
 *
 *  ┌──────┬────────────┬──────────────────────────────────────────────┐
 *  │ 编码器│ 引脚       │ 板上去哪接                                    │
 *  ├──────┼────────────┼──────────────────────────────────────────────┤
 *  │ 1  A  │ PA0        │ EXTI0，P2 排针第 3 脚                         │
 *  │ 1  B  │ PA2        │ EXTI2，P2 排针第 2 脚                         │
 *  │ 2  A  │ PA7        │ EXTI7，P2 排针第 4 脚                         │
 *  │ 2  B  │ PA8        │ EXTI8，P2 排针第 7 脚                         │
 *  └──────┴────────────┴──────────────────────────────────────────────┘
 *  编码器正极 → CN3 的 VCC5；编码器负极 → P3 的 GND（P2 那排没有电源脚）
 *
 *  【为什么全走 EXTI 软件解码】原本编码器 1 想用 TIM8_CH1/CH2（PC6/PC7）做硬件
 *  正交：不占 CPU、4 倍频最省事。但这块 ZYSTM32-A0 实际板子上 PC6/PC7/PB14/PB15
 *  都没有接到可用排针（原理图里 PC9 甚至整根悬空），只有 P2 那排是确定能用的
 *  （舵机 PB0 就在这一排）。所以两路统一改成 EXTI 软件正交解码，四个脚全落在
 *  P2 上、一簇接完。
 *
 *  代价：单相约 14.5kHz（350rpm × 52 线）时，一路 29k 次/秒中断，每次 ~2us，
 *  两路合计约占 10% CPU —— 5ms 调度下完全撑得住，但**别再往上加引脚负担**。
 *  若哪天确认 PC6/PC7 能接出来，把 ENC1_USE_TIM8 改回 1 即可切回硬件解码。
 *
 *  ！5V 容忍：PA0/PA2/PA7 不是 FT（数据手册表5 只写 "I/O"），只有 PA8 是 FT。
 *    松甲编码器 A/B 是"带上拉输出"、电平跟它自己的供电走：
 *      · 编码器用 5V 供电时，A/B 会有 5V 电平，进 PA0/PA2/PA7 会倒灌 →
 *        必须**改 3.3V 供电**（编码器标称 4.5~5V，实测 3.3V 也能输出，
 *        若你的型号不吃 3.3V，就在 A/B 各串 1kΩ + 3.3V 上拉分压）；
 *      · 或者干脆用 5V 供电 + 串电阻分压，但这是下策。
 *    推荐：编码器正极接 **3.3V**（CN3 上有 VCC3.3），A/B 直连，最省事最安全。
 *
 *  ！陀螺仪 PC0/PC1 同样不是 FT，只能 3.3V，见 gyro.h。
 *
 *  【引脚待定怎么办】整套都在下面这几个宏里，改宏 + 重新编译即可，不用动 .c：
 *      · 两路 A/B 各自落在哪个 EXTI 组决定中断函数名：
 *        Pin_0 → EXTI0_IRQHandler      Pin_1 → EXTI1_IRQHandler
 *        Pin_2 → EXTI2_IRQHandler      Pin_3 → EXTI3_IRQHandler
 *        Pin_4 → EXTI4_IRQHandler      Pin_5~9 → EXTI9_5_IRQHandler
 *        Pin_10~15 → EXTI15_10_IRQHandler
 *      · 换到别的 EXTI 组就要同时改 .c 里的中断函数名与 ENCx_x_IRQn。
 *      · 两路不要用到同一条 EXTI 线（比如都占 Pin_8），会互相抢中断。
 *
 *  【计数与换算】
 *      Encoder_GetDeltaL/R()  每 ENC_TICK_MS 的增量（已做 4 倍频）
 *      Encoder_GetSpeedL/R()  折算成"每秒计数"
 *      Encoder_GetTotalL/R()  上电以来的累计计数
 *    要换算成厘米，把 ENC_CNT_PER_REV / ENC_WHEEL_MM / ENC_GEAR_X10 填上，
 *    然后用 Encoder_GetSpeedMmS()。
 * ========================================================================== */

#define ENC_ENABLE            1     /* 0 = 整个编码器模块不编译 */

/* ---------------- 采样节拍 ---------------- */
#define ENC_TICK_MS           10    /* 必须等于 sched 里的注册周期 */

/* ---------------- 编码器 1 ---------------- */
#define ENC1_USE_TIM8         0     /* 1 = 用 TIM8 硬件正交（需 PC6/PC7 能接出来）*/
#define ENC1_PORT             GPIOA
#define ENC1_RCC              RCC_APB2Periph_GPIOA
#define ENC1_A_PIN            GPIO_Pin_0        /* PA0 → EXTI0（P2 第 3 脚）*/
#define ENC1_B_PIN            GPIO_Pin_2        /* PA2 → EXTI2（P2 第 2 脚）*/
#define ENC1_A_PIN_SRC        GPIO_PinSource0
#define ENC1_B_PIN_SRC        GPIO_PinSource2
#define ENC1_PORT_SRC         GPIO_PortSourceGPIOA
#define ENC1_A_EXTI_LINE      EXTI_Line0
#define ENC1_B_EXTI_LINE      EXTI_Line2
#define ENC1_A_IRQn           EXTI0_IRQn
#define ENC1_B_IRQn           EXTI2_IRQn
#define ENC1_A_TIM_CH         GPIO_Pin_6        /* 仅当 ENC1_USE_TIM8=1 时有效 */
#define ENC1_B_TIM_CH         GPIO_Pin_7
#define ENC1_TIM_PORT         GPIOC
#define ENC1_TIM_RCC          RCC_APB2Periph_GPIOC
#define ENC1_SIGN             1                 /* 装反了（前进时计数变小）改成 -1 */

/* ---------------- 编码器 2 ---------------- */
#define ENC2_ENABLE           1
#define ENC2_PORT             GPIOA
#define ENC2_RCC              RCC_APB2Periph_GPIOA
#define ENC2_A_PIN            GPIO_Pin_7        /* PA7 → EXTI7（P2 第 4 脚）*/
#define ENC2_B_PIN            GPIO_Pin_8        /* PA8 → EXTI8（P2 第 7 脚）*/
#define ENC2_A_PIN_SRC        GPIO_PinSource7
#define ENC2_B_PIN_SRC        GPIO_PinSource8
#define ENC2_PORT_SRC         GPIO_PortSourceGPIOA
#define ENC2_A_EXTI_LINE      EXTI_Line7
#define ENC2_B_EXTI_LINE      EXTI_Line8
#define ENC2_IRQn             EXTI9_5_IRQn       /* Pin_5~9 共用一个入口 */
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
u8      Encoder_GetSecondReady(void);   /* 1 = 第二路已使能 */

#endif
