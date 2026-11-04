#ifndef __IRAVOID_H_
#define __IRAVOID_H_

#include "stm32f10x.h"

/* ============================================================================
 *  避障模块  v3.5 —— 超声波 HC-SR04 + 舵机 180° 扫描
 *  ==========================================================================
 *
 *  【硬件接线】
 *      超声波 TRIG  → PC0     （推挽输出，发 15us 触发脉冲）
 *      超声波 ECHO  → PC1     （下拉输入 + EXTI1 上升/下降沿）
 *      舵机 信号    → PB0     （TIM3_CH3，50Hz，0.5~2.5ms 对应 0~180°）
 *
 *  为什么不用原来的 PA0/TIM5 舵机接法：
 *      厂商例程把舵机接在 PA0/TIM5_CH1，而本工程的 TIM5 已被红外遥控解码占用
 *      （PA1 / TIM5_CH2，需要 10ms 溢出），一个定时器只能有一套时基，会打架。
 *      所以舵机改到 TIM3_CH3/PB0（PB0 换灰度循迹后已空闲），互不影响。
 *
 *  【两版二选一】改下面这个宏，重新编译即可：
 *      AVOID_SCAN_WHILE_DRIVING = 0  →  **停车扫描版**
 *          发现障碍 → 停车 → 舵机扫 左/中/右 三个方向 → 选距离大的一侧
 *          → 转向 → 回正 → 继续直行
 *          优点：扫描期间车是静止的，测距最准、逻辑最稳
 *          缺点：每遇一次障碍要停 1 秒左右
 *
 *      AVOID_SCAN_WHILE_DRIVING = 1  →  **边走边扫版**
 *          车不停，舵机在 ±40° 范围持续左右扫，边扫边直行
 *          前方(±20°)距离小于阈值时，朝"最近一次扫到的较空一侧"直接弧线绕过去
 *          优点：不停车，流畅
 *          缺点：扫描是"历史数据"，障碍突然出现时反应略慢；扫描期间转向角有限
 * ========================================================================== */

#define AVOID_SCAN_WHILE_DRIVING    0    /* 0=停车扫描版   1=边走边扫版 */

/* ============================ 传感器参数 ============================ */
#define AV_TRIGGER_CM        40     /* 前方小于这个距离 → 认为有障碍（触发避让） */
#define AV_CLEAR_CM          55     /* 侧向大于这个距离 → 认为这一侧可以走 */
#define AV_MIN_GAP_CM        25     /* 两侧都小于这个距离 → 无处可绕，原地掉头 */

#define AV_SERVO_CENTER      90     /* 舵机中位（正前方） */
#define AV_SERVO_LEFT       170     /* 左侧极限角 */
#define AV_SERVO_RIGHT       10     /* 右侧极限角 */
#define AV_SWEEP_HALF        40     /* 边走边扫版：以中位为中心左右各扫多少度 */

/* ============================ 运动参数 ============================ */
#define AV_BASE_SPEED        55     /* 直行速度 */
#define AV_TURN_OUTER        85     /* 转向外侧轮 */
#define AV_TURN_INNER         0     /* 转向内侧轮（0 = 内轮停，差速转弯） */
#define AV_SPIN_SPEED        55     /* 两侧都堵死时的原地掉头速度 */

#define AV_SETTLE_MS        250     /* 舵机转到位后等多久再测距（防抖） */
#define AV_TURN_MS          700     /* 一次转向持续多久 */
#define AV_RECOVER_MS       300     /* 转向后回正持续多久 */
#define AV_MEAS_PERIOD_MS   100     /* 直行时多久测一次前方 */

#define AV_MEAS_TIMEOUT_US  30000   /* 超声波等待回波上限（30ms ≈ 5m） */

/* ---- 兼容旧代码的探头宏（仅当 AVOID_SRC_ULTRASONIC = 0 时使用） ---- */
#define AVOID_PIN         GPIO_Pin_1
#define AVOID_PIN_GPIO    GPIOB
#define AVOID_IO          GPIO_ReadInputDataBit(AVOID_PIN_GPIO, AVOID_PIN)
#define BARRIER_Y 0
#define BARRIER_N 1

/* ---- 初始化 / 任务 ---- */
void IRAvoidInit(void);         /* 初始化超声波 + 舵机（主循环前调用一次） */
void AVoidRun(void);            /* 避障任务：主循环每 10ms 调一次 */

/* ---- 调试接口（串口打印用） ---- */
u8   AVoid_GetState(void);      /* 0=直行 1=扫描 2=转向 3=回正 */
u8   AVoid_GetDistance(void);   /* 最近一次前方距离(cm)，0=无效 */
u8   AVoid_GetAngle(void);      /* 当前舵机角度 */

#endif
