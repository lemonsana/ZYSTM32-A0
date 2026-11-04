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

/* ------------------------- 非阻塞接口（推荐） ------------------------- */
/* 设置左右轮速度，范围 -100 ~ +100（百分比），正数前进。
 * 只写寄存器，立即返回。 */
void Motor_SetSpeed(int left, int right);
void Motor_Stop(void);
void Motor_Init(void);

int  Motor_GetLeft(void);           /* 当前设定速度（调试打印用） */
int  Motor_GetRight(void);

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
