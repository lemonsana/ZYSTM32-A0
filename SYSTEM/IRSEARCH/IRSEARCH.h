#ifndef __IRSEARCH_H_
#define __IRSEARCH_H_

#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块接口  v3.0
 *  --------------------------------------------------------------------------
 *  算法与参数取自"可循迹版本"（已验证能稳定循迹），并做了以下改良：
 *    1) 传感器接口可选：I2C（默认，已验证） / 串行 CLK-DAT（v2.2 方案，备选）
 *    2) 电机调用统一走非阻塞 Motor_SetSpeed()，不再依赖原厂符号约定
 *    3) 丢线超时按毫秒计算，与主循环节拍无关
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 *  传感器接口选择：
 *      1 = I2C（默认）—— 与"可循迹版本"一致，已验证可用
 *      0 = 串行 CLK/DAT —— v2.2 方案，模块接线见下
 * ------------------------------------------------------------------------- */
#define GRAY_IF_I2C          1

/* ============================ I2C 接线 ============================
 *      模块 VCC   →  5V
 *      模块 GND   →  GND
 *      模块 SCL   →  PB10（丝印 IO16）
 *      模块 SDA   →  PB11（丝印 IO15）
 *      模块 PULL 跳线 → 必须短接（5V 上拉）
 *      模块 AD0/AD1   → 不装（保持默认地址 0x4C）
 *  PB10/PB11 是 5V 容忍脚，5V 上拉安全。
 * ================================================================= */
#define GRAY_I2C_GPIO         GPIOB
#define GRAY_I2C_SCL_PIN      GPIO_Pin_10
#define GRAY_I2C_SDA_PIN      GPIO_Pin_11
#define GRAY_I2C_ADDR         0x4C

/* 感为寄存器 */
#define GW_GRAY_PING          0xAA
#define GW_GRAY_PING_OK       0x66
#define GW_GRAY_ANALOG_MODE   0xB0    /* 0xB0~0xB7：8 路模拟量 */
#define GW_GRAY_CHANNEL_EN    0xCE    /* 通道使能 */
#define GW_GRAY_NORMALIZE     0xCF    /* 归一化使能 */

/* ============================ 串行接线（备选） ============================
 *      模块 VCC   →  5V
 *      模块 GND   →  GND
 *      模块 CLK   →  PB10（单片机输出时钟）
 *      模块 DAT   →  PB11（模块输出数据，5V 容忍脚）
 *  时序：CLK 拉低 → 2us → 读DAT → CLK 拉高 → 5us，循环 8 次，bit0 先出
 * ======================================================================= */
#define GRAY_CLK_PIN          GPIO_Pin_10
#define GRAY_CLK_GPIO         GPIOB
#define GRAY_DAT_PIN          GPIO_Pin_11
#define GRAY_DAT_GPIO         GPIOB

/* ---------------------------- 循迹参数（调车改这里） ----------------------------
 *  GRAY_BASE_SPEED     直行速度 0~100
 *  GRAY_STEER_GAIN     满偏误差（±100）时的转向量，35 表示 ±35
 *  GRAY_BLACK_CUTOFF   归一化模拟量低于此值算"压在黑线上"（白≈255，黑≈0）
 *  GRAY_CENTER_DEADBAND 误差死区，避免中线附近来回抖
 *  GRAY_CHANNEL_MIRROR 1 = 镜像探头顺序（车往反方向修正时改这个）
 *  GRAY_LOST_STOP_MS   丢线超过多久就停车
 *  GRAY_FRAME_MS       必须与 main.c 的 TICK_MS 一致
 *
 *  PID 参数：
 *    GRAY_STEER_GAIN     = P：小误差时的比例增益
 *    GRAY_STEER_GAIN_FAR = P：大误差（急弯）时的比例增益，更"硬"
 *    GRAY_FAR_THRESHOLD  = |误差| 超过这个值就切成 FAR 增益
 *    GRAY_KI             = I：每累积 1000 (误差·毫秒) 贡献的转向量
 *    GRAY_KD             = D：微分系数
 *    GRAY_I_LIMIT        = 积分累加上限（误差·毫秒），防止饱和
 *
 *  v3.2 新增（过急弯用）：
 *    GRAY_SLOW_GAIN      = 误差越大越慢：base 减掉 |误差|×SLOW_GAIN/100
 *    GRAY_SLOW_D_GAIN    = 误差变化越快也越慢（进弯前先减速）
 *    GRAY_MIN_SPEED      = 减速后的最低基础速度
 *    GRAY_INNER_REVERSE  = 内轮允许反转的最大幅度（度数），急弯时能显著减小转弯半径
 *
 *  v3.3 按"前视 12cm"重调（几何分析见 README）：
 *    前视 12cm + 阵列半宽 3.5cm → 能连续跟住的最小弯半径约 20.6cm；
 *    比这更急的弯（视频里 8~10cm 的发夹弯）必然"丢线-找回"，
 *    所以参数往"少超调、快找回"方向调，丢线找回时间放宽到 3 秒。
 * ---------------------------------------------------------------------------- */
#define GRAY_LOOKAHEAD_CM      12   /* 灰度板到电机驱动轴的距离（用于说明/换算） */

#define GRAY_BASE_SPEED        32
#define GRAY_STEER_GAIN        32
#define GRAY_STEER_GAIN_FAR    50
#define GRAY_FAR_THRESHOLD     40
#define GRAY_KI                 0
#define GRAY_KD                12
#define GRAY_I_LIMIT         2500
#define GRAY_BLACK_CUTOFF     180
#define GRAY_CENTER_DEADBAND    4
#define GRAY_CHANNEL_MIRROR     0

#define GRAY_SLOW_GAIN         20
#define GRAY_SLOW_D_GAIN       40
#define GRAY_MIN_SPEED         18
#define GRAY_INNER_REVERSE     45

#define GRAY_LOST_SPIN_SPEED   50
#define GRAY_LOST_STOP_MS    3000
#define GRAY_FRAME_MS          10

/* 调试接口 */
void IRSearchInit(void);            /* 初始化传感器 */
void SearchRun(void);               /* 循迹任务：主循环每 TICK_MS 调一次 */

u8   Gray_IsReady(void);            /* 1 = 传感器已就绪 */
int  Gray_GetError(void);           /* 最近一次线位置误差 -100~+100，999=丢线 */
int  Gray_GetCorrection(void);      /* 最近一次 PID 输出（调参看这个最直观） */
int  Gray_GetBaseSpeed(void);       /* 最近一次调度后的基础速度 */
void Gray_ResetIntegral(void);      /* 清积分（切模式、重新上路时调用） */
u8   Gray_ReadRaw(u8 *values8);     /* 读原始 8 路数据，返回 1=成功 */

#endif
