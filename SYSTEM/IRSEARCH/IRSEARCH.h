#ifndef __IRSEARCH_H_
#define __IRSEARCH_H_

#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块接口  v2.1
 *  --------------------------------------------------------------------------
 *  两种循迹方案，用 LINE_USE_GRAY 切换：
 *      1 = 感为 8 路灰度（I2C）    ← v2.1 默认，分辨率高、更稳
 *      0 = 原来的两路红外探头      ← 保留作对照/兜底
 * ========================================================================== */

#define LINE_USE_GRAY        1      /* 1=灰度循迹  0=两路红外探头循迹 */

/* ============================ 方案一：灰度 ============================
 *  感为 8 路灰度模块（5V 供电，I2C 接口）
 *
 *  接线表（★ 就按这个接，其它接线一根都不用动）：
 *      模块 VCC    →  主板 5V 排针
 *      模块 GND    →  主板 GND
 *      模块 SCL    →  PB10（丝印 IO16）
 *      模块 SDA    →  PB11（丝印 IO15）
 *      模块 PULL 跳线 → 必须短接（打开模块上拉），否则 I2C 拉不高！
 *
 *  说明：PB10/PB11 是 STM32F103 的 5V 容忍引脚（FT），模块 5V 上拉不会
 *        打坏 MCU；这里用软件模拟 I2C（开漏输出），不占用硬件 I2C 外设。
 *
 *  感为寄存器（来自模块资料 gw_grayscale_sensor.h）：
 *      0xAA       读 → 返回 0x66 表示在线（PING）
 *      0xDD       读 → 8 位数字量，bit0 = 探头1（最右），bit7 = 探头8（最左）
 *      0xB0~0xB7  读 → 8 个探头的模拟量（各 8 位）
 *      0xC1       读 → 固件版本
 * ==================================================================== */

/* ---- 灰度传感器引脚（软件 I2C） ---- */
#define GRAY_SCL_PIN          GPIO_Pin_10
#define GRAY_SCL_GPIO         GPIOB
#define GRAY_SCL_SET          GPIO_SetBits(GRAY_SCL_GPIO, GRAY_SCL_PIN)
#define GRAY_SCL_RESET        GPIO_ResetBits(GRAY_SCL_GPIO, GRAY_SCL_PIN)

#define GRAY_SDA_PIN          GPIO_Pin_11
#define GRAY_SDA_GPIO         GPIOB
#define GRAY_SDA_SET          GPIO_SetBits(GRAY_SDA_GPIO, GRAY_SDA_PIN)
#define GRAY_SDA_RESET        GPIO_ResetBits(GRAY_SDA_GPIO, GRAY_SDA_PIN)
#define GRAY_SDA_READ         GPIO_ReadInputDataBit(GRAY_SDA_GPIO, GRAY_SDA_PIN)

/* ---- 感为模块 I2C 从机地址 ---- */
#define GRAY_I2C_ADDR         0x4C

/* ---- 感为模块寄存器 ---- */
#define GW_GRAY_PING          0xAA
#define GW_GRAY_PING_OK       0x66
#define GW_GRAY_DIGITAL_MODE  0xDD
#define GW_GRAY_ANALOG_BASE   0xB0
#define GW_GRAY_FIRMWARE      0xC1

/* ---- 循迹控制参数（调车改这里） ---- */
#define LINE_BASE_SPEED       55    /* 直行基础速度 */
#define LINE_KP               25    /* 比例系数：误差 ±350 时转向量约 ±87 */
#define LINE_KD               30    /* 微分系数 */
#define LINE_LOST_STOP_MS    800    /* 丢线后最多找多久，超时就停车 */

void IRSearchInit(void);            /* 初始化（按 LINE_USE_GRAY 自动选择） */
void SearchRun(void);               /* 循迹任务：主循环每 10ms 调一次 */

/* ---- 灰度调试接口（串口打印用） ---- */
u8  Gray_Ping(void);                /* 0 = 在线，1 = 不在线 */
u8  Gray_ReadDigital(void);         /* 读 8 位数字量 */
u8  Gray_ReadAnalog(u8 *buf8);      /* 读 8 路模拟量 */
u8  Gray_GetBits(void);             /* 本周期读到的数字量 */
u8  Gray_IsOnline(void);            /* 本周期是否在线 */
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
