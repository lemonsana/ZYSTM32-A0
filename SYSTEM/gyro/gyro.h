#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f10x.h"

/* ============================================================================
 *  MPU6050 六轴陀螺仪驱动  V6.0
 *  ----------------------------------------------------------------------------
 *  【接线】软件 I2C，两根信号线：
 *
 *      MPU6050 VCC → 3.3V 或 5V   （GY-521 这类模块板载 LDO，两种都行）
 *      MPU6050 GND → GND
 *      MPU6050 SCL → PC0           （丝印 A0）
 *      MPU6050 SDA → PC1           （丝印 A1）
 *      MPU6050 AD0 → 悬空 / GND    （地址 0x68；接 VCC 变 0x69，要同步改 GYRO_I2C_ADDR）
 *
 *  ！PC0/PC1 在 STM32F103 上不是 5V 容忍脚。
 *    GY-521 一类模块的 SDA/SCL 上拉电阻是接到模块自己 3.3V 上的，所以总线电平最高
 *    3.3V，直接接是安全的。若用裸芯片自己焊，上拉一定接 3.3V，不要接 5V。
 *
 *  【和超声波避障的关系 —— 重要】
 *    本工程 v3.5 起把 HC-SR04 接在 PC0(TRIG) / PC1(ECHO) 上，和陀螺仪**抢同一对脚**。
 *    现在这两只脚归陀螺仪，所以 IRAvoid.h 里 AVOID_HW_ENABLE 是 0：
 *    超声波不再初始化、EXTI1 不再注册，避障功能默认关闭（代码全部保留，
 *    哪天把超声波挪到别的脚，把那个宏改回 1 就能恢复）。
 *
 *  【输出什么】
 *      yaw     航向角，0.1° 单位，±1800 = ±180°。上电 Gyro_Init() 里会静止校准
 *              零偏；之后仍会慢慢漂移（MPU6050 零偏温漂，约 1~3°/分钟），
 *              这是器件特性，不是代码问题。要长时间定向往返，用陀螺仪当
 *              "短时基准"、灰度当"绝对基准"。
 *      pitch   俯仰（车头抬/低），0.1°
 *      roll    横滚（左右倾），0.1°
 *      rate_z  绕竖轴的角速度，0.1°/s —— 调试方向时看这个最直观
 *
 *  【方向约定】
 *    GYRO_YAW_CW_POSITIVE = 1 时：车头**向右**转（顺时针，从上往下看）→ yaw 变大。
 *    这和 IRSEARCH.c 里"正误差 = 线在右边 = 往右打方向"是同一个符号方向，
 *    接航向辅助时不用再取反。
 *    实测反了有两种改法：① 把 GYRO_YAW_CW_POSITIVE 改成 0；② 把模块翻个面装上。
 * ========================================================================== */

#define GYRO_ENABLE           1     /* 0 = 整个陀螺仪模块不编译（排故障 / 省 flash 用） */

/* ---------------- 软件 I2C 引脚（SCL/SDA 必须在同一个 GPIO 口） ---------------- */
#define GYRO_I2C_PORT         GPIOC
#define GYRO_I2C_RCC          RCC_APB2Periph_GPIOC
#define GYRO_SCL_PIN          GPIO_Pin_0        /* PC0 */
#define GYRO_SDA_PIN          GPIO_Pin_1        /* PC1 */
#define GYRO_CLK_DELAY_US     2                 /* 半周期延时(us)：2 ≈ 100~150kHz；排线长就加到 5 */

#define GYRO_I2C_ADDR         0x68              /* AD0 接 GND = 0x68；接 VCC = 0x69 */

/* ---------------- 采样与量程 ---------------- */
#define GYRO_SAMPLE_MS        10    /* 多久读一次，必须等于 sched 里的注册周期，否则 dt 算错 */
#define GYRO_GYRO_FS          500   /* 量程 ±500°/s（小车够用，撞一下也不会溢出） */
#define GYRO_GYRO_LSB         65.5f /* ±500 → 65.5 LSB/(°/s) */
#define GYRO_ACCEL_LSB        16384.0f  /* ±2g → 16384 LSB/g */

/* ---------------- 方向 ---------------- */
#define GYRO_YAW_CW_POSITIVE  1         /* 1 = 顺时针(右转)时 yaw 变大；实测反了就改 0 */

/* ---------------- 互补滤波 ---------------- */
#define GYRO_COMP_ALPHA       98        /* 陀螺权重 98%，加速度计 2%（越大越平滑、越慢对齐） */
#define GYRO_PITCH_RATE_SIGN  (-1)      /* 俯仰方向的陀螺积分符号；跟手转反了就改 1 */
#define GYRO_ROLL_RATE_SIGN   (1)       /* 横滚方向的陀螺积分符号；反了就改 -1 */

/* ---------------- 静止校准零偏 ---------------- */
#define GYRO_CAL_SAMPLES      200   /* 取多少点求平均 */
#define GYRO_CAL_DELAY_MS     2     /* 采样间隔 → 200 × 2ms ≈ 0.4s，上电时不能动车 */

/* ---------------- 角度闭环转向（Gyro_TurnBy） ---------------- */
#define GYRO_TURN_TOL10       30    /* 到位判据 ±3.0° */
#define GYRO_TURN_MIN         28    /* 最低占空比：1:48 减速比下再小就转不动 */

/* ---------------- 对外接口 ---------------- */
u8      Gyro_Init(void);                    /* 1 = 找到并配置好 MPU6050；0 = 没通 */
void    Gyro_Task(void);                    /* 每 GYRO_SAMPLE_MS 调一次（读 + 积分 + 滤波） */
u8      Gyro_IsReady(void);                 /* 1 = 初始化成功 */
void    Gyro_Calibrate(void);               /* 重新校准零偏（整车必须静止！） */
void    Gyro_ZeroYaw(void);                 /* 把当前航向清零（原地设基准点） */

int16_t Gyro_GetYaw10(void);                /* 航向角 0.1° */
int16_t Gyro_GetPitch10(void);              /* 俯仰 0.1° */
int16_t Gyro_GetRoll10(void);               /* 横滚 0.1° */
int16_t Gyro_GetRateZ10(void);              /* 角速度 0.1°/s，带符号，调方向用 */
int16_t Gyro_GetBiasZ10(void);              /* 校准出来的 Z 轴零偏 0.1°/s */
u8      Gyro_GetId(void);                   /* WHO_AM_I 读到的原始值（0x68 = MPU6050） */

/* 角度差：返回 a - b，自动回绕到 ±1800（跨越 ±180° 不会跳变） */
int16_t Gyro_AngleDiff10(int16_t a, int16_t b);

/* 转到相对角度 delta10（0.1°）：正 = 顺时针/右转，负 = 逆时针/左转。
 * 注意：**阻塞**函数，内部每 10ms 自己调一次 Gyro_Task()，
 *       只能用在车已经停下来、准备原地转向的阶段（避障绕行 / 掉头 / 找线）。 */
u8      Gyro_TurnBy(int16_t delta10, int max_speed, u16 timeout_ms);

#endif
