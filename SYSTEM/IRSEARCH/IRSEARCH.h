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

/* ============================================================================
 *  循迹参数（调车就改这一段，改完重新编译烧录）
 *  ----------------------------------------------------------------------------
 *  控制链路（自下而上理解，调参时对号入座）：
 *
 *    8 路归一化模拟量 (白≈255 黑≈0)
 *        │
 *        │ ① 加权重心：wᵢ = CUTOFF − 值ᵢ（越黑权重越大），posᵢ = ±7,±5,±3,±1
 *        │    error = Σ(wᵢ·posᵢ) / (Σwᵢ·7) × 100      → 范围 −100 ~ +100
 *        │    0 = 线在正中；负数 = 线偏左；正数 = 线偏右
 *        ▼
 *    ┌──────────────────────────────────────────────────────────────┐
 *    │ ② 一阶低通：filt = (filt×3 + error)/4     （抑制单帧跳变）    │
 *    │ ③ 死区：|filt| < DEADBAND → 0             （中线附近不抖）    │
 *    │ ④ PID：                                                      │
 *    │      P = e × GAIN / 100                                      │
 *    │      I = (Σ e×帧时长) × KI / 1000                            │
 *    │      D = (e − e上一帧) × KD / 100                            │
 *    │      correction = P + I + D                                  │
 *    │ ⑤ 速度调度：base = BASE − |e|×SLOW_GAIN/100 − |Δe|×SLOW_D_GAIN/100 │
 *    │ ⑥ 输出：左 = base + correction，右 = base − correction         │
 *    │    限幅：左/右 ∈ [−INNER_REVERSE, 95]                        │
 *    └──────────────────────────────────────────────────────────────┘
 *
 *  ----------------------------------------------------------------------------
 *  【P 项】GRAY_STEER_GAIN / GRAY_STEER_GAIN_FAR / GRAY_FAR_THRESHOLD
 *      P 是"看到多大偏差就打多大方向"。
 *      · 直线画龙（左右摆）      → 调小 GAIN（32 → 25）
 *      · 弯道切不进去/转向不足    → 调大 GAIN（32 → 40）
 *      GAIN_FAR 是"大误差专用"的档位，|e| 超过 FAR_THRESHOLD 时启用，
 *      用来在不牺牲直线稳定性的前提下，让急弯打得更狠。
 *      只调一个的话：先动 GAIN，弯里不够再动 GAIN_FAR。
 *
 *  【D 项】GRAY_KD
 *      D 是"看到偏差变化多快就提前收手"，起阻尼作用。
 *      · 进弯冲过头、出弯甩尾    → 调大 KD（12 → 20）
 *      · 车发抖、电机嗡嗡响       → 调小 KD（12 → 6 或 0）
 *      本工程误差已过一阶低通，D 不会太吵，但别一次加太多。
 *
 *  【I 项】GRAY_KI / GRAY_I_LIMIT
 *      I 是"偏差长期存在就持续加力"，用来消静差。
 *      · 车总贴着线的一侧跑、回不到正中 → 加 KI（0 → 2 → 4）
 *      · 前视 12cm 时**建议保持 0**：探头看得远，积分容易攒成过冲
 *      I_LIMIT 是积分上限，防止长时间堵转后"甩不回来"。
 *
 *  【速度调度】GRAY_SLOW_GAIN / GRAY_SLOW_D_GAIN / GRAY_MIN_SPEED
 *      base 会随偏差和偏差变化率自动降速：
 *      · 弯里冲出去                → 调大 SLOW_GAIN / SLOW_D_GAIN
 *      · 整体太慢、过弯磨蹭        → 调小 SLOW_GAIN，或调大 MIN_SPEED
 *
 *  【转弯极限】GRAY_INNER_REVERSE
 *      内轮允许反转的最大幅度，急弯时"原地搓"能显著减小转弯半径。
 *      · 急弯拐不过来              → 45 → 60
 *      · 弯里车尾扫、轮胎打滑      → 45 → 30
 *
 *  【丢线找回】GRAY_LOST_SPIN_SPEED / GRAY_LOST_STOP_MS
 *      · 掉线后找不回来            → 速度调大、时间调长
 *      · 掉线后乱转圈              → 速度调小
 *
 *  【几何前提】GRAY_LOOKAHEAD_CM
 *      灰度板到驱动轴的距离。前视 L、阵列半宽 h 时，能连续跟住的最小弯半径
 *      R ≈ L²/(2h)。本车 L=12cm、h≈3.5cm → R ≈ 20.6cm，更急的弯必然丢线，
 *      此时靠"丢线 → 原地找回"兜底，属于正常现象。
 * ============================================================================ */
#define GRAY_LOOKAHEAD_CM      12   /* 灰度板到电机驱动轴的距离（仅说明/换算用） */

#define GRAY_BASE_SPEED        32   /* 直行基础速度 */
#define GRAY_STEER_GAIN        32   /* P：小误差增益 */
#define GRAY_STEER_GAIN_FAR    50   /* P：大误差（急弯）增益 */
#define GRAY_FAR_THRESHOLD     40   /* |误差| 超过它切换到 FAR 增益 */
#define GRAY_KI                 0   /* I：积分增益（前视 12cm 建议 0） */
#define GRAY_KD                12   /* D：微分增益 */
#define GRAY_I_LIMIT         2500   /* 积分上限（误差·毫秒） */
#define GRAY_BLACK_CUTOFF     180   /* 低于此值算黑线（白≈255 黑≈0） */
#define GRAY_CENTER_DEADBAND    4   /* 误差死区 */
#define GRAY_CHANNEL_MIRROR     0   /* 1 = 镜像探头顺序（方向反了改这个） */

#define GRAY_SLOW_GAIN         20   /* 速度调度：按 |误差| 降速 */
#define GRAY_SLOW_D_GAIN       40   /* 速度调度：按 |误差变化| 降速（进弯预减速） */
#define GRAY_MIN_SPEED         18   /* 降速下限 */
#define GRAY_INNER_REVERSE     45   /* 内轮允许反转的最大幅度 */

#define GRAY_LOST_SPIN_SPEED   50   /* 丢线找线速度 */
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
u8   Gray_IsOnLine(void);           /* 1 = 当前至少有一路探头压到黑线（融合模式找线用） */
u8   Gray_ReadRaw(u8 *values8);     /* 读原始 8 路数据，返回 1=成功 */

#endif
