#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f10x.h"

/* ================== 电机驱动（ZYSTM32-A0 主板） ==================
 *
 *  左右两路电机，每路 = 1 个方向脚 + 1 路 TIM4 PWM：
 *
 *    左电机： 方向 PB7 (丝印 IO8)     PWM PB8 = TIM4_CH3 (丝印 IO9)
 *    右电机： 方向 PA4 (丝印 IO11)    PWM PB9 = TIM4_CH4 (丝印 IO10)
 *
 *  方向脚极性（两轮相同）：
 *    低电平 = 正转（前进）    高电平 = 反转（后退）
 *
 *  ★ v1 的 bug（已在本版修正）：
 *    v1 的 SetMotorSpeed() 把"右轮"写成 TIM_SetCompare3(PB8)、"左轮"写成
 *    TIM_SetCompare4(PB9)，与上面的引脚定义正好交叉 —— 每个轮子拿到的是
 *    另一个轮子的速度；而且速度为 0 时不写方向脚，方向脚会残留上一次状态。
 *    于是循迹时"左转"指令实际只让左轮转，且方向永远不变 → 表现为
 *    "开启循迹后只有左轮胎单向旋转"。
 *    v2 修正：左轮→CH3(PB8)、右轮→CH4(PB9)，并且每次都显式写方向脚。
 * ================================================================ */

/* 方向反转开关：若实测某个轮子正反转相反，把对应宏改成 1（不用改接线） */
#define MOTOR_DIR_INVERT_L      0
#define MOTOR_DIR_INVERT_R      0

/* PWM 通道交叉开关：
 * 0 = 正常（左轮 CH3/PB8，右轮 CH4/PB9，与原理图一致）
 * 1 = 如果你的板子实测仍是"通道与引脚交叉"，改成 1 可整体翻转 */
#define MOTOR_PWM_CH_SWAP       0

/* PWM 定时器参数：72MHz / (0+1) / (7199+1) = 10kHz */
#define MOTOR_PWM_ARR           7200    /* 自动重装值 + 1 */

/* ------------------------- v2 新接口（非阻塞） ------------------------- */
/* 设置左右轮速度，范围 -100 ~ +100（百分比），正数前进。
 * 只写寄存器，立即返回，不占用主循环时间。 */
void Motor_SetSpeed(int left, int right);
void Motor_Stop(void);
void Motor_Init(void);

int  Motor_GetLeft(void);           /* 当前设定速度（调试打印用） */
int  Motor_GetRight(void);

/* --------------------- 兼容 v1 的阻塞式接口（保留） --------------------- */
void TIM4_PWM_Init(unsigned short arr,unsigned short psc);  /* 等同 Motor_Init */
void SetMotorSpeed(unsigned char ucChannel, signed char cSpeed); /* 0=右轮 1=左轮 */

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

/*  左电机 PWM   PB8 → TIM4_CH3  */
#define LEFT_MOTOR_PWM            GPIO_Pin_8
#define LEFT_MOTOR_PWM_GPIO       GPIOB

/*  右电机方向  PA4  */
#define RIGHT_MOTOR_GO            GPIO_Pin_4
#define RIGHT_MOTOR_GPIO          GPIOA
#define RIGHT_MOTOR_GO_SET        GPIO_SetBits(RIGHT_MOTOR_GPIO,RIGHT_MOTOR_GO)
#define RIGHT_MOTOR_GO_RESET      GPIO_ResetBits(RIGHT_MOTOR_GPIO,RIGHT_MOTOR_GO)

/*  右电机 PWM   PB9 → TIM4_CH4  */
#define RIGHT_MOTOR_PWM           GPIO_Pin_9
#define RIGHT_MOTOR_PWM_GPIO      GPIOB

#endif
