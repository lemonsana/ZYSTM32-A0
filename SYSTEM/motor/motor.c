#include "motor.h"
#include "delay.h"
#include <stdlib.h>     /* abs() */

/* ------------------------------------------------------------------
 * v2 电机驱动
 *   非阻塞：Motor_SetSpeed() 只写 PWM 比较值和方向脚，立即返回
 *   阻塞版：ZYSTM32_xxx() 仍然保留，内部 = 设置速度 + delay_ms
 * ------------------------------------------------------------------ */

static int motor_l = 0;     /* 左轮当前设定速度 */
static int motor_r = 0;     /* 右轮当前设定速度 */

int Motor_GetLeft(void)  { return motor_l; }
int Motor_GetRight(void) { return motor_r; }

/* 限幅到 -100 ~ +100 */
static int Motor_Clamp(int v)
{
    if (v >  100) return  100;
    if (v < -100) return -100;
    return v;
}

/* 设置单个轮子
 * ch    : 0 = 左轮，1 = 右轮
 * speed : -100 ~ +100，正数前进
 */
static void Motor_SetOne(unsigned char ch, int speed)
{
    int pwm;

    speed = Motor_Clamp(speed);
    pwm   = MOTOR_PWM_ARR - (abs(speed) * MOTOR_PWM_ARR) / 100;

    /* PWM2 模式是"反相"的：CCR 越大占空比越小
     *   speed=0   → pwm=7200 > ARR(7199) → 输出恒低 → 占空比 0%
     *   speed=100 → pwm=0               → 输出恒高 → 占空比 100%
     *   speed=50  → pwm=3600            → 占空比 50%
     */
    if (ch == 0)        /* ---------------- 左轮 ---------------- */
    {
#if MOTOR_PWM_CH_SWAP
        TIM_SetCompare4(TIM4, pwm);         /* 交叉模式 */
#else
        TIM_SetCompare3(TIM4, pwm);         /* 正常：CH3 → PB8 */
#endif
        if (speed >= 0)                     /* 正转 */
        {
#if MOTOR_DIR_INVERT_L
            LEFT_MOTOR_GO_SET;
#else
            LEFT_MOTOR_GO_RESET;
#endif
        }
        else                                /* 反转 */
        {
#if MOTOR_DIR_INVERT_L
            LEFT_MOTOR_GO_RESET;
#else
            LEFT_MOTOR_GO_SET;
#endif
        }
        motor_l = speed;
    }
    else                /* ---------------- 右轮 ---------------- */
    {
#if MOTOR_PWM_CH_SWAP
        TIM_SetCompare3(TIM4, pwm);         /* 交叉模式 */
#else
        TIM_SetCompare4(TIM4, pwm);         /* 正常：CH4 → PB9 */
#endif
        if (speed >= 0)
        {
#if MOTOR_DIR_INVERT_R
            RIGHT_MOTOR_GO_SET;
#else
            RIGHT_MOTOR_GO_RESET;
#endif
        }
        else
        {
#if MOTOR_DIR_INVERT_R
            RIGHT_MOTOR_GO_RESET;
#else
            RIGHT_MOTOR_GO_SET;
#endif
        }
        motor_r = speed;
    }
}

/* 设置双轮速度（正数前进）。这是 v2 所有上层模块唯一的速度出口 */
void Motor_SetSpeed(int left, int right)
{
    Motor_SetOne(0, left);
    Motor_SetOne(1, right);
}

/* 停车：占空比清零，方向脚回到"正转"状态 */
void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

/* ==================== 初始化 ==================== */
void TIM4_PWM_Init(unsigned short arr,unsigned short psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    GPIO_InitTypeDef         GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB , ENABLE);

    /* 左电机方向 PB7 */
    GPIO_InitStructure.GPIO_Pin   = LEFT_MOTOR_GO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LEFT_MOTOR_GO_GPIO, &GPIO_InitStructure);

    /* 左电机 PWM  PB8 —— 复用推挽（TIM4_CH3） */
    GPIO_InitStructure.GPIO_Pin   = LEFT_MOTOR_PWM;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(LEFT_MOTOR_PWM_GPIO, &GPIO_InitStructure);

    /* 右电机方向 PA4 */
    GPIO_InitStructure.GPIO_Pin   = RIGHT_MOTOR_GO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RIGHT_MOTOR_GPIO, &GPIO_InitStructure);

    /* 右电机 PWM  PB9 —— 复用推挽（TIM4_CH4） */
    GPIO_InitStructure.GPIO_Pin   = RIGHT_MOTOR_PWM;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(RIGHT_MOTOR_PWM_GPIO, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period        = arr;
    TIM_TimeBaseStructure.TIM_Prescaler     = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;      /* 反相 PWM */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);    /* CH3 → PB8 左轮 */
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);    /* CH4 → PB9 右轮 */

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    /* 上电先停车，方向脚给到明确的"正转"状态 */
    Motor_Stop();
}

void Motor_Init(void)
{
    TIM4_PWM_Init(7199, 0);     /* 72MHz/7200 = 10kHz */
    Motor_Stop();
}

/* ==================== v1 兼容接口 ==================== */

/* ucChannel: 0=右轮, 1=左轮（保持 v1 的编号习惯） */
void SetMotorSpeed(unsigned char ucChannel,signed char cSpeed)
{
    if (ucChannel == 0) Motor_SetOne(1, cSpeed);   /* 右轮 */
    else                Motor_SetOne(0, cSpeed);   /* 左轮 */
}

/* 前进：双轮正转 */
void ZYSTM32_run(signed char speed,int time)
{
    Motor_SetSpeed(speed, speed);
    delay_ms(time);
}

/* 刹车：占空比清零 */
void ZYSTM32_brake(int time)
{
    Motor_Stop();
    delay_ms(time);
}

/* 左转：左轮不动，右轮正转 */
void ZYSTM32_Left(signed char speed,int time)
{
    Motor_SetSpeed(0, speed);
    delay_ms(time);
}

/* 原地左旋：左轮反转，右轮正转 */
void ZYSTM32_Spin_Left(signed char speed,int time)
{
    Motor_SetSpeed(-speed, speed);
    delay_ms(time);
}

/* 右转：左轮正转，右轮不动 */
void ZYSTM32_Right(signed char speed,int time)
{
    Motor_SetSpeed(speed, 0);
    delay_ms(time);
}

/* 原地右旋：左轮正转，右轮反转 */
void ZYSTM32_Spin_Right(signed char speed,int time)
{
    Motor_SetSpeed(speed, -speed);
    delay_ms(time);
}

/* 后退：双轮反转
 * 注意：v1 这里写成 speed = 100 - speed，导致 back(80) 实际只有 20 的速度，
 *       v2 已修正为直接用 speed。 */
void ZYSTM32_back(signed char speed,int time)
{
    Motor_SetSpeed(-speed, -speed);
    delay_ms(time);
}
