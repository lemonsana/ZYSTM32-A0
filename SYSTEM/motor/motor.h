#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f10x.h"

/* ================== 电机驱动（ZYSTM32-A0 主板）  v3.0 ==================
 *
 *  ★★★ 重要更正（v3 相比 v2.x 的关键修复） ★★★
 *
 *  v2.0 曾把"右轮"从 CH3 改到 CH4，这是**错的**，导致 v2.x 车子跑不直线。
 *  依据：把"可循迹版本"（只改了 IRSEARCH.c，motor.c 保持原厂）与 v1.0 对比，
 *        它用的就是原厂映射，且实车能稳定循迹 —— 说明原厂通道是对的。
 *
 *  实际接线（以车能跑通为准）：
 *      左电机： 方向 PB7 (丝印 IO8)      PWM PB9 = TIM4_CH4 (丝印 IO10)
 *      右电机： 方向 PA4 (丝印 IO11)     PWM PB8 = TIM4_CH3 (丝印 IO9)
 *
 *  方向脚极性（两轮相同）：
 *      低电平 = 正转（前进）      高电平 = 反转（后退）
 *
 *  注意：motor.h 里 RIGHT_MOTOR_PWM / LEFT_MOTOR_PWM 两个宏的名字与实测
 *        相反（历史遗留），本文件已按实测把注释写清楚，宏名保持不变以免
 *        影响其它代码。
 * ====================================================================== */

/* 方向反转开关：若实测某个轮子正反转相反，把对应宏改成 1（不用改接线） */
#define MOTOR_DIR_INVERT_L      0
#define MOTOR_DIR_INVERT_R      0

/* PWM 通道交叉开关：
 * 0 = 正常（左轮 CH4/PB9，右轮 CH3/PB8）—— 与本车实测一致
 * 1 = 如果你的板子反过来，改成 1 可整体翻转 */
#define MOTOR_PWM_CH_SWAP       0

/* PWM 定时器参数：72MHz / (0+1) / (7199+1) = 10kHz */
#define MOTOR_PWM_ARR           7200    /* 自动重装值 + 1 */

/* ==================== 速度闭环（R2 起默认开启） ====================
 * 开环时 Motor_SetSpeed(40) 只是给 40% 占空比，两个电机个体差异能差 10~20%，
 * 车直行必然画龙。开了闭环以后，Motor_SetSpeed() 里的数字变成"目标速度"，
 * 由 MOTOR_LOOP_MS 周期的 PI 调节器根据编码器实测增量去修正占空比。
 *
 * 换算基准 MOTOR_CNT_PER_UNIT_X100：本车实测，50% 占空比悬空约 40 计数/10ms，
 * 即 1% ≈ 0.8 计数/10ms，写成 x100 = 80。换底盘/换电机请重新量一次。
 *
 * 关闭：把 MOTOR_SPEED_LOOP 改 0，立刻回到纯开环（行为与 V6.2 完全一致）。
 * ================================================================== */
#define MOTOR_SPEED_LOOP        1       /* 1=速度闭环  0=纯开环 */
#define MOTOR_LOOP_MS           10      /* 闭环周期，必须等于 ENC_TICK_MS */
#define MOTOR_CNT_PER_UNIT_X100 80      /* 每 1% 占空比每 10ms 的计数 x100 */
#define MOTOR_LOOP_KP           60      /* 比例：每差 1 个计数，修正 0.6% 占空比 */
#define MOTOR_LOOP_KI           10      /* 积分：消灭稳态误差（电池掉压时靠它补） */
#define MOTOR_LOOP_I_LIMIT      4000    /* 积分限幅，防止绕死 */
#define MOTOR_LOOP_TRIM_MAX     40      /* 单次修正上限（占空比 %），防突变 */

/* ------------------------- 非阻塞接口（推荐） ------------------------- */
/* 设置左右轮速度，范围 -100 ~ +100（百分比），正数前进。
 * 只写寄存器，立即返回。 */
void Motor_SetSpeed(int left, int right);
void Motor_Stop(void);
void Motor_Init(void);

int  Motor_GetLeft(void);           /* 当前设定速度（调试打印用） */
int  Motor_GetRight(void);

/* 速度闭环：每 MOTOR_LOOP_MS 调一次（main.c 已注册进 sched 后台） */
void Motor_SpeedLoopTask(void);
int  Motor_GetPwmLeft(void);        /* 闭环实际输出的占空比（调试用） */
int  Motor_GetPwmRight(void);
int  Motor_GetTgtLeft(void);        /* 上层要求的目标速度 */
int  Motor_GetTgtRight(void);

/* --------------------- 兼容原厂接口（语义与 v1.0 完全一致） ---------------------
 * SetMotorSpeed(ch, speed)：
 *      ch = 0 右轮，speed > 0 前进，< 0 后退
 *      ch = 1 左轮，speed < 0 前进，> 0 后退   ← 与原厂一致，注意符号相反！
 * ZYSTM32_xxx()：阻塞式动作函数，内部 = 设置速度 + delay_ms
 * ------------------------------------------------------------------------- */
void TIM4_PWM_Init(unsigned short arr,unsigned short psc);  /* 等同 Motor_Init */
void SetMotorSpeed(unsigned char ucChannel, signed char cSpeed);

void ZYSTM32_run(signed char speed,int time);        /* 前进 */
void ZYSTM32_brake(int time);                        /* 刹车/停车 */
void ZYSTM32_Left(signed char speed,int time);       /* 左转（左轮停） */
void ZYSTM32_Spin_Left(signed char speed,int time);  /* 原地左旋 */
void ZYSTM32_Right(signed char speed,int time);      /* 右转（右轮停） */
void ZYSTM32_Spin_Right(signed char speed,int time); /* 原地右旋 */
void ZYSTM32_back(signed char speed,int time);       /* 后退 */

/* ---------------- 电机引脚定义（板载固定，不要改） ---------------- */
/*  左电机方向  PB7  */
#define LEFT_MOTOR_GO             GPIO_Pin_7
#define LEFT_MOTOR_GO_GPIO        GPIOB
#define LEFT_MOTOR_GO_SET         GPIO_SetBits(LEFT_MOTOR_GO_GPIO,LEFT_MOTOR_GO)
#define LEFT_MOTOR_GO_RESET       GPIO_ResetBits(LEFT_MOTOR_GO_GPIO,LEFT_MOTOR_GO)

/*  左电机 PWM   PB9 → TIM4_CH4（宏名沿用历史写法，实际是 CH4/PB9） */
#define LEFT_MOTOR_PWM            GPIO_Pin_9
#define LEFT_MOTOR_PWM_GPIO       GPIOB

/*  右电机方向  PA4  */
#define RIGHT_MOTOR_GO            GPIO_Pin_4
#define RIGHT_MOTOR_GPIO          GPIOA
#define RIGHT_MOTOR_GO_SET        GPIO_SetBits(RIGHT_MOTOR_GPIO,RIGHT_MOTOR_GO)
#define RIGHT_MOTOR_GO_RESET      GPIO_ResetBits(RIGHT_MOTOR_GPIO,RIGHT_MOTOR_GO)

/*  右电机 PWM   PB8 → TIM4_CH3（宏名沿用历史写法，实际是 CH3/PB8） */
#define RIGHT_MOTOR_PWM           GPIO_Pin_8
#define RIGHT_MOTOR_PWM_GPIO      GPIOB

#endif
