#include "motor.h"
#include "delay.h"
#include <stdlib.h>     /* abs() */
#include "encoder.h"    /* 速度闭环要读编码器增量 */

/* ------------------------------------------------------------------
 * v3.0 电机驱动
 *   通道映射按实车可用版本恢复：左轮 CH4/PB9，右轮 CH3/PB8
 *   非阻塞：Motor_SetSpeed() 只写 PWM 比较值和方向脚，立即返回
 *   阻塞版：ZYSTM32_xxx() 保留，内部 = 设置速度 + delay_ms
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

/* 速度 → CCR 值（PWM2 反相模式）
 *   speed=0   → 7200 > ARR(7199) → 恒低 → 0%
 *   speed=100 → 0                → 恒高 → 100%
 *   speed=50  → 3600             → 50%
 */
static int Motor_PwmOf(int speed)
{
    return MOTOR_PWM_ARR - (abs(Motor_Clamp(speed)) * MOTOR_PWM_ARR) / 100;
}

/* 设置单个轮子
 * ch    : 0 = 左轮，1 = 右轮
 * speed : -100 ~ +100，正数前进
 */
static void Motor_SetOne(unsigned char ch, int speed)
{
    int pwm;
    u8  forward;

    speed   = Motor_Clamp(speed);
    pwm     = Motor_PwmOf(speed);
    forward = (speed >= 0) ? 1 : 0;

    if (ch == 0)        /* ---------------- 左轮：CH4 → PB9 ---------------- */
    {
#if MOTOR_PWM_CH_SWAP
        TIM_SetCompare3(TIM4, pwm);
#else
        TIM_SetCompare4(TIM4, pwm);
#endif
        if (forward)
        {
#if MOTOR_DIR_INVERT_L
            LEFT_MOTOR_GO_SET;
#else
            LEFT_MOTOR_GO_RESET;
#endif
        }
        else
        {
#if MOTOR_DIR_INVERT_L
            LEFT_MOTOR_GO_RESET;
#else
            LEFT_MOTOR_GO_SET;
#endif
        }
        motor_l = speed;
    }
    else                /* ---------------- 右轮：CH3 → PB8 ---------------- */
    {
#if MOTOR_PWM_CH_SWAP
        TIM_SetCompare4(TIM4, pwm);
#else
        TIM_SetCompare3(TIM4, pwm);
#endif
        if (forward)
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

/* ==================== 速度闭环（PI） ==================== */
#if MOTOR_SPEED_LOOP
static int motor_tgt_l = 0;     /* 上层目标 */
static int motor_tgt_r = 0;
static int motor_pwm_l = 0;     /* 闭环实际输出 */
static int motor_pwm_r = 0;
static int motor_i_l    = 0;    /* 积分累加 */
static int motor_i_r    = 0;

int Motor_GetPwmLeft(void)  { return motor_pwm_l; }
int Motor_GetPwmRight(void) { return motor_pwm_r; }
int Motor_GetTgtLeft(void)  { return motor_tgt_l; }
int Motor_GetTgtRight(void) { return motor_tgt_r; }

/* 目标速度(%) → 每 10ms 期望计数 */
static int Motor_CntTarget(int speed)
{
    return (speed * MOTOR_CNT_PER_UNIT_X100) / 100;
}

/* 单轮 PI：返回应输出的占空比 */
static int Motor_PiOut(int target, int actual, int *integ)
{
    int expect;
    int err;
    int trim;
    int out;

    if (target == 0)                /* 目标 0 立刻停，积分清零 */
    {
        *integ = 0;
        return 0;
    }

    expect = Motor_CntTarget(target);
    err    = expect - actual;       /* 转太慢 → err>0 → 加大占空比 */

    *integ += err;
    if (*integ >  MOTOR_LOOP_I_LIMIT) *integ =  MOTOR_LOOP_I_LIMIT;
    if (*integ < -MOTOR_LOOP_I_LIMIT) *integ = -MOTOR_LOOP_I_LIMIT;

    trim = (MOTOR_LOOP_KP * err + MOTOR_LOOP_KI * (*integ)) / 100;
    if (trim >  MOTOR_LOOP_TRIM_MAX) trim =  MOTOR_LOOP_TRIM_MAX;
    if (trim < -MOTOR_LOOP_TRIM_MAX) trim = -MOTOR_LOOP_TRIM_MAX;

    out = target + trim;
    if (target > 0)                 /* 前进不反向：最低 0 */
    {
        if (out < 0)   out = 0;
        if (out > 100) out = 100;
    }
    else                            /* 后退同理 */
    {
        if (out > 0)    out = 0;
        if (out < -100) out = -100;
    }
    return out;
}

/* 每 MOTOR_LOOP_MS 调一次：读实测增量，算出占空比，写下去 */
void Motor_SpeedLoopTask(void)
{
    int dl = (int)Encoder_GetDeltaL();
    int dr = (int)Encoder_GetDeltaR();

    motor_pwm_l = Motor_PiOut(motor_tgt_l, dl, &motor_i_l);
    motor_pwm_r = Motor_PiOut(motor_tgt_r, dr, &motor_i_r);

    Motor_SetOne(0, motor_pwm_l);
    Motor_SetOne(1, motor_pwm_r);
}
#endif /* MOTOR_SPEED_LOOP */

/* 设置双轮速度（正数前进）。这是所有上层模块唯一的速度出口。
 * 闭环开启时：只记目标，真正的占空比由 Motor_SpeedLoopTask() 每 10ms 输出。
 * 目标为 0 的那一侧立即断电停车（不等下一拍）。 */
void Motor_SetSpeed(int left, int right)
{
#if MOTOR_SPEED_LOOP
    motor_tgt_l = Motor_Clamp(left);
    motor_tgt_r = Motor_Clamp(right);

    if (motor_tgt_l == 0)
    {
        motor_i_l   = 0;
        motor_pwm_l = 0;
        Motor_SetOne(0, 0);
    }
    if (motor_tgt_r == 0)
    {
        motor_i_r   = 0;
        motor_pwm_r = 0;
        Motor_SetOne(1, 0);
    }
#else
    Motor_SetOne(0, left);
    Motor_SetOne(1, right);
#endif
}

/* 停车 */
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

    /* 左电机 PWM  PB9 —— 复用推挽（TIM4_CH4） */
    GPIO_InitStructure.GPIO_Pin   = LEFT_MOTOR_PWM;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(LEFT_MOTOR_PWM_GPIO, &GPIO_InitStructure);

    /* 右电机方向 PA4 */
    GPIO_InitStructure.GPIO_Pin   = RIGHT_MOTOR_GO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RIGHT_MOTOR_GPIO, &GPIO_InitStructure);

    /* 右电机 PWM  PB8 —— 复用推挽（TIM4_CH3） */
    GPIO_InitStructure.GPIO_Pin   = RIGHT_MOTOR_PWM;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(RIGHT_MOTOR_PWM_GPIO, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period        = arr;
    TIM_TimeBaseStructure.TIM_Prescaler     = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);    /* CH3 → PB8 右轮 */
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);    /* CH4 → PB9 左轮 */

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    Motor_Stop();
}

void Motor_Init(void)
{
    TIM4_PWM_Init(7199, 0);     /* 72MHz/7200 = 10kHz */
    Motor_Stop();
}

/* ==================== 原厂兼容接口（语义与 v1.0 完全一致） ==================== */

void SetMotorSpeed(unsigned char ucChannel,signed char cSpeed)
{
    int pwm = Motor_PwmOf(cSpeed);

    if (ucChannel == 0)         /* 右轮：CH3/PB8，>0 前进 */
    {
        TIM_SetCompare3(TIM4, pwm);
        if (cSpeed > 0)      RIGHT_MOTOR_GO_RESET;
        else if (cSpeed < 0) RIGHT_MOTOR_GO_SET;
    }
    else                        /* 左轮：CH4/PB9，<0 前进（原厂符号约定） */
    {
        TIM_SetCompare4(TIM4, pwm);
        if (cSpeed > 0)      LEFT_MOTOR_GO_SET;
        else if (cSpeed < 0) LEFT_MOTOR_GO_RESET;
    }
}

void ZYSTM32_run(signed char speed,int time)
{
    Motor_SetSpeed(speed, speed);
    delay_ms(time);
}

void ZYSTM32_brake(int time)
{
    Motor_Stop();
    delay_ms(time);
}

void ZYSTM32_Left(signed char speed,int time)
{
    Motor_SetSpeed(0, speed);
    delay_ms(time);
}

void ZYSTM32_Spin_Left(signed char speed,int time)
{
    Motor_SetSpeed(-speed, speed);
    delay_ms(time);
}

void ZYSTM32_Right(signed char speed,int time)
{
    Motor_SetSpeed(speed, 0);
    delay_ms(time);
}

void ZYSTM32_Spin_Right(signed char speed,int time)
{
    Motor_SetSpeed(speed, -speed);
    delay_ms(time);
}

/* 后退：v1 里写成 100-speed，实际速度会被翻过来，这里修正为直接用 speed */
void ZYSTM32_back(signed char speed,int time)
{
    Motor_SetSpeed(-speed, -speed);
    delay_ms(time);
}
