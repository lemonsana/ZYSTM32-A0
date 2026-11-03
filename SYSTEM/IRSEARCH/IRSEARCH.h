#ifndef __IRSEARCH_H_
#define __IRSEARCH_H_

#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块接口  v2.2
 *  --------------------------------------------------------------------------
 *  两种循迹方案，用 LINE_USE_GRAY 切换：
 *      1 = 感为 8 路灰度（串行 CLK/DAT 接口）   ← 默认
 *      0 = 原来的两路红外探头                    ← 保留作对照/兜底
 * ========================================================================== */

#define LINE_USE_GRAY        1      /* 1=灰度循迹  0=两路红外探头循迹 */

/* ============================ 方案一：灰度 ============================
 *  感为 8 路灰度模块（5V 供电，串行 CLK/DAT 接口）
 *
 *  接线表（★ 就按这个接，其它接线一根都不用动）：
 *      模块 VCC   →  主板 5V 排针
 *      模块 GND   →  主板 GND
 *      模块 CLK   →  PB10（丝印 IO16）   ← 单片机输出时钟
 *      模块 DAT   →  PB11（丝印 IO15）   ← 模块输出数据，读进单片机
 *
 *  为什么这样接：
 *    - DAT 是模块的 5V 输出，PB11 是 STM32F103 的 5V 容忍脚（FT），不会打坏 MCU；
 *    - CLK 是单片机输出，模块输入 3.3V 高电平可以识别（厂家 MSPM0 3.3V 例程同款接法）；
 *    - 串行接口是推挽输出，不需要 PULL 上拉跳线、不需要 I2C 地址和应答。
 *
 *  协议（来自模块资料 examples/Serial/gpio_toggle_output.c）：
 *      for (i = 0; i < 8; i++) {
 *          CLK = 0;        // 下降沿
 *          delay 2us;
 *          bit[i] = DAT;   // 读数据，bit0 先出
 *          CLK = 1;        // 上升沿，模块切换到下一位
 *          delay 5us;
 *      }
 *      8 个时钟后得到 8 位数字量，bit0 = 探头1（最右），bit7 = 探头8（最左）
 * ==================================================================== */

/* ---- 灰度传感器引脚（串行 CLK/DAT） ---- */
#define GRAY_CLK_PIN          GPIO_Pin_10
#define GRAY_CLK_GPIO         GPIOB
#define GRAY_CLK_SET          GPIO_SetBits(GRAY_CLK_GPIO, GRAY_CLK_PIN)
#define GRAY_CLK_RESET        GPIO_ResetBits(GRAY_CLK_GPIO, GRAY_CLK_PIN)

#define GRAY_DAT_PIN          GPIO_Pin_11
#define GRAY_DAT_GPIO         GPIOB
#define GRAY_DAT_READ         GPIO_ReadInputDataBit(GRAY_DAT_GPIO, GRAY_DAT_PIN)

/* ---- 循迹控制参数（调车改这里） ---- */
#define LINE_BASE_SPEED       55    /* 直行基础速度 */
#define LINE_KP               25    /* 比例系数：误差 ±350 时转向量约 ±87 */
#define LINE_KD               30    /* 微分系数 */
#define LINE_LOST_STOP_MS    800    /* 丢线后最多找多久，超时就停车 */

void IRSearchInit(void);            /* 初始化（按 LINE_USE_GRAY 自动选择） */
void SearchRun(void);               /* 循迹任务：主循环每 10ms 调一次 */

/* ---- 灰度调试接口（串口打印用） ---- */
u8  Gray_ReadByte(void);            /* 直接读一次 8 位数字量 */
u8  Gray_GetBits(void);             /* 本周期读到的数字量 */
int Gray_GetError(void);            /* 本周期误差 -350~+350，999=丢线 */

/* ---- 方案二：原来的两路红外探头 ---- */
#define SEARCH_R_PIN         GPIO_Pin_7
#define SEARCH_R_GPIO        GPIOA
#define SEARCH_R_IO          GPIO_ReadInputDataBit(SEARCH_R_GPIO, SEARCH_R_PIN)

#define SEARCH_L_PIN         GPIO_Pin_0
#define SEARCH_L_GPIO        GPIOB
#define SEARCH_L_IO          GPIO_ReadInputDataBit(SEARCH_L_GPIO, SEARCH_L_PIN)

#define BLACK_AREA 1        /* 压黑线 */
#define WHITE_AREA 0        /* 白底 */

#endif
