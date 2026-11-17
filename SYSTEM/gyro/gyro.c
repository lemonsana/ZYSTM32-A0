#include "gyro.h"
#include "delay.h"
#include "motor.h"
#include <math.h>

#if GYRO_ENABLE

/* ============================================================================
 *  MPU6050 驱动（软件 I2C + 姿态解算）
 *  ----------------------------------------------------------------------------
 *  为什么用软件 I2C 而不是硬件 I2C：
 *    1) STM32F103 的硬件 I2C 有著名的死锁勘误，总线被拉低后很难恢复；
 *    2) PC0/PC1 不是 I2C1/I2C2 的复用脚（那是 PB6/PB7、PB10/PB11，都已被 LED
 *       和灰度占用），要用硬件 I2C 就得挪脚；
 *    3) MPU6050 读一次 14 字节只要 ~1.5ms（100kHz），10ms 的节拍完全够用。
 * ========================================================================== */

/* ------------------------------ MPU6050 寄存器 ------------------------------ */
#define MPU_SMPLRT_DIV        0x19
#define MPU_CONFIG            0x1A
#define MPU_GYRO_CONFIG       0x1B
#define MPU_ACCEL_CONFIG      0x1C
#define MPU_ACCEL_XOUT_H      0x3B
#define MPU_GYRO_XOUT_H       0x43
#define MPU_PWR_MGMT_1        0x6B
#define MPU_WHO_AM_I          0x75

/* ------------------------------ 引脚操作宏 ------------------------------ */
#define SCL_H()   GPIO_SetBits(GYRO_I2C_PORT,   GYRO_SCL_PIN)
#define SCL_L()   GPIO_ResetBits(GYRO_I2C_PORT, GYRO_SCL_PIN)
#define SDA_H()   GPIO_SetBits(GYRO_I2C_PORT,   GYRO_SDA_PIN)
#define SDA_L()   GPIO_ResetBits(GYRO_I2C_PORT, GYRO_SDA_PIN)
#define SDA_IN()  GPIO_ReadInputDataBit(GYRO_I2C_PORT, GYRO_SDA_PIN)

/* ============================== 软件 I2C ============================== */

/* 半周期延时。开漏输出 + 上拉的上升沿由 RC 决定，延时太小波形会塌；
 * 2us ≈ 100~150kHz，MPU6050 支持到 400kHz，余量很大。 */
static void i2c_delay(void)
{
    delay_us(GYRO_CLK_DELAY_US);
}

/* 总线恢复：从机把 SDA 拉死在低电平时，发 9 个时钟把它顶出来，再补一个 STOP。
 * 上电第一次通信前调一次，能救回"上次复位时正好停在传输中间"的模块。 */
static void i2c_bus_recover(void)
{
    u8 i;

    SDA_H();
    for (i = 0; i < 9; i++)
    {
        SCL_L(); i2c_delay();
        SCL_H(); i2c_delay();
    }
    /* 手动补一个 STOP：SCL 高电平期间 SDA 由低变高 */
    SDA_L(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();
}

static void i2c_start(void)
{
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_L(); i2c_delay();      /* SCL 高时 SDA 下降 = START */
    SCL_L(); i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();      /* SCL 高时 SDA 上升 = STOP */
}

/* 返回 1 = 从机应答（ACK），0 = 无应答 */
static u8 i2c_write_byte(u8 b)
{
    u8 i;
    u8 ack;

    for (i = 0; i < 8; i++)
    {
        if (b & 0x80u) SDA_H();
        else           SDA_L();
        b = (u8)(b << 1);
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
    }

    /* 第 9 个时钟：主机释放 SDA，从机若在就把它拉低 */
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    ack = (SDA_IN() == Bit_RESET) ? 1u : 0u;
    SCL_L(); i2c_delay();

    return ack;
}

/* ack = 1 时主机回 ACK（还要继续读），0 回 NACK（最后一字节） */
static u8 i2c_read_byte(u8 ack)
{
    u8 i;
    u8 b = 0;

    SDA_H();                    /* 释放 SDA，交给从机驱动 */
    for (i = 0; i < 8; i++)
    {
        i2c_delay();
        SCL_H(); i2c_delay();
        b = (u8)(b << 1);
        if (SDA_IN() == Bit_SET) b |= 1u;
        SCL_L(); i2c_delay();
    }

    if (ack) SDA_L();
    else     SDA_H();
    i2c_delay();
    SCL_H(); i2c_delay();
    SCL_L(); i2c_delay();

    return b;
}

/* ------------------------------ 寄存器读写 ------------------------------ */

static u8 mpu_write(u8 reg, u8 val)
{
    u8 ok = 0;

    i2c_start();
    if (i2c_write_byte((u8)(GYRO_I2C_ADDR << 1)))          /* 写地址 */
    {
        if (i2c_write_byte(reg) && i2c_write_byte(val))    /* 寄存器 + 数据 */
        {
            ok = 1;
        }
    }
    i2c_stop();

    return ok;
}

static u8 mpu_read(u8 reg, u8 *buf, u8 len)
{
    u8 i;

    i2c_start();
    if (!i2c_write_byte((u8)(GYRO_I2C_ADDR << 1)))         /* 写地址 */
    {
        i2c_stop();
        return 0;
    }
    if (!i2c_write_byte(reg))                              /* 要读的寄存器 */
    {
        i2c_stop();
        return 0;
    }

    i2c_start();                                           /* 重复 START 切换成读 */
    if (!i2c_write_byte((u8)((GYRO_I2C_ADDR << 1) | 1)))   /* 读地址 */
    {
        i2c_stop();
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        buf[i] = i2c_read_byte((u8)(i + 1u < len));        /* 最后一字节回 NACK */
    }
    i2c_stop();

    return 1;
}

/* ============================== 姿态状态 ============================== */

static u8      gyro_ready   = 0;        /* 1 = 初始化成功，Gyro_Task 才会干活 */
static u8      gyro_id      = 0;        /* WHO_AM_I 原始值 */
static float   gyro_bias_x  = 0.0f;     /* 零偏校准值，单位 °/s */
static float   gyro_bias_y  = 0.0f;
static float   gyro_bias_z  = 0.0f;
static float   gyro_yaw     = 0.0f;     /* ° */
static float   gyro_pitch   = 0.0f;
static float   gyro_roll    = 0.0f;
static int16_t gyro_yaw10   = 0;        /* 0.1° */
static int16_t gyro_pitch10 = 0;
static int16_t gyro_roll10  = 0;
static int16_t gyro_rate_z10= 0;        /* 0.1°/s */
static int16_t gyro_bias_z10= 0;
static u16     gyro_err_cnt = 0;        /* 连续读失败计数，调试打印用 */

/* 把 0.1° 的整数角回绕到 ±1800 */
static int16_t ang_wrap10(int32_t a)
{
    while (a >  1800) a -= 3600;
    while (a < -1800) a += 3600;
    return (int16_t)a;
}

/* ============================== 初始化 ============================== */

u8 Gyro_Init(void)
{
    GPIO_InitTypeDef gpio;
    u8 id = 0;
    u8 cfg = 0;
    u8 ok;

    gyro_ready    = 0;
    gyro_id       = 0;
    gyro_yaw      = 0.0f;
    gyro_pitch    = 0.0f;
    gyro_roll     = 0.0f;
    gyro_bias_x   = 0.0f;
    gyro_bias_y   = 0.0f;
    gyro_bias_z   = 0.0f;
    gyro_yaw10    = 0;
    gyro_pitch10  = 0;
    gyro_roll10   = 0;
    gyro_rate_z10 = 0;
    gyro_bias_z10 = 0;
    gyro_err_cnt  = 0;

    RCC_APB2PeriphClockCmd(GYRO_I2C_RCC, ENABLE);

    /* 先把两只脚置高，再切成开漏输出 —— 避免切换瞬间在 SCL 上造出一个假时钟 */
    GPIO_SetBits(GYRO_I2C_PORT, GYRO_SCL_PIN | GYRO_SDA_PIN);
    gpio.GPIO_Pin   = GYRO_SCL_PIN | GYRO_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GYRO_I2C_PORT, &gpio);
    delay_us(200);

    i2c_bus_recover();

    /* ① 先确认总线上真的有东西：WHO_AM_I 正常读回 0x68（MPU6050）/ 0x70~0x72（其它型号） */
    if (!mpu_read(MPU_WHO_AM_I, &id, 1))
    {
        return 0;                                   /* 从机没应答：没接线 / 没供电 / 引脚反了 */
    }
    gyro_id = id;
    if (id == 0x00u || id == 0xFFu)
    {
        return 0;                                   /* 有应答但数据是空的：SDA 被拉高拉低不透 */
    }

    /* ② 软复位 —— 清掉模块上一次运行残留的配置 */
    ok  = mpu_write(MPU_PWR_MGMT_1, 0x80);
    delay_ms(100);
    ok &= mpu_write(MPU_PWR_MGMT_1, 0x01);          /* 退出睡眠，时钟用 X 轴陀螺 PLL（比内部 RC 稳） */
    delay_ms(10);

    /* ③ 采样率与滤波 */
    ok &= mpu_write(MPU_SMPLRT_DIV, 9);             /* 1kHz / (1+9) = 100Hz 输出率 */
    ok &= mpu_write(MPU_CONFIG, 0x03);              /* DLPF：陀螺 44Hz / 加速度 42Hz，滤掉电机振动 */

    /* ④ 量程 */
#if   (GYRO_GYRO_FS == 250)
    ok &= mpu_write(MPU_GYRO_CONFIG, 0x00);         /* ±250  °/s */
#elif (GYRO_GYRO_FS == 500)
    ok &= mpu_write(MPU_GYRO_CONFIG, 0x08);         /* ±500  °/s */
#elif (GYRO_GYRO_FS == 1000)
    ok &= mpu_write(MPU_GYRO_CONFIG, 0x10);         /* ±1000 °/s */
#else
    ok &= mpu_write(MPU_GYRO_CONFIG, 0x18);         /* ±2000 °/s */
#endif
    ok &= mpu_write(MPU_ACCEL_CONFIG, 0x00);        /* ±2g */
    delay_ms(10);

    /* ⑤ 回读确认真的写进去了（写不进 = 供电不足或总线被干扰） */
    if (!mpu_read(MPU_GYRO_CONFIG, &cfg, 1))
    {
        return 0;
    }
    if (cfg != 0x00u && cfg != 0x08u && cfg != 0x10u && cfg != 0x18u)
    {
        return 0;                                   /* 回读值不在合法量程编码里，判为失败 */
    }

    if (!ok)
    {
        return 0;
    }

    /* ⑥ 静止校准零偏 —— 这一步跑的时候必须整车静止，否则零偏会带进积分里 */
    Gyro_Calibrate();

    gyro_ready = 1;
    return 1;
}

/* 静止校准：取 GYRO_CAL_SAMPLES 个点求平均当作零偏。
 * 期间整车必须静止（轮子不转、人别碰），否则标定值是错的。 */
void Gyro_Calibrate(void)
{
    u8  buf[6];
    u16 i;
    u16 n = 0;
    int32_t sx = 0, sy = 0, sz = 0;

    for (i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (mpu_read(MPU_GYRO_XOUT_H, buf, 6))
        {
            sx += (int16_t)(((u16)buf[0] << 8) | buf[1]);
            sy += (int16_t)(((u16)buf[2] << 8) | buf[3]);
            sz += (int16_t)(((u16)buf[4] << 8) | buf[5]);
            n++;
        }
        delay_ms(GYRO_CAL_DELAY_MS);
    }

    if (n == 0)
    {
        return;                                     /* 一个点都没读到，保留旧零偏 */
    }

    gyro_bias_x = (float)sx / (float)n / GYRO_GYRO_LSB;
    gyro_bias_y = (float)sy / (float)n / GYRO_GYRO_LSB;
    gyro_bias_z = (float)sz / (float)n / GYRO_GYRO_LSB;
    gyro_bias_z10 = (int16_t)(gyro_bias_z * 10.0f);

    /* 校准完把姿态角也归零，之后 yaw 就是从"现在这个朝向"开始算的 */
    gyro_yaw   = 0.0f;
    gyro_yaw10 = 0;
}

void Gyro_ZeroYaw(void)
{
    gyro_yaw   = 0.0f;
    gyro_yaw10 = 0;
}

/* ============================== 周期任务 ============================== */

/* 每 GYRO_SAMPLE_MS 调一次：
 *   一次连读 14 字节拿全 加速度(6) + 温度(2) + 陀螺(6)，比分 3 次读省时也用同一个时刻的数据。
 *   yaw 用陀螺积分（短期极准、长期会漂）；pitch/roll 用互补滤波（陀螺的快速响应 +
 *   加速度计的重力方向做长期基准）。 */
void Gyro_Task(void)
{
    u8      buf[14];
    int16_t ax, ay, az, gx, gy, gz;
    float   dt, a;
    float   rx, ry, rz;
    float   pitch_acc, roll_acc;

    if (!gyro_ready)
    {
        return;
    }

    if (!mpu_read(MPU_ACCEL_XOUT_H, buf, 14))
    {
        if (gyro_err_cnt < 0xFFFFu)
        {
            gyro_err_cnt++;
        }
        return;
    }
    gyro_err_cnt = 0;

    ax = (int16_t)(((u16)buf[0]  << 8) | buf[1]);
    ay = (int16_t)(((u16)buf[2]  << 8) | buf[3]);
    az = (int16_t)(((u16)buf[4]  << 8) | buf[5]);
    /* buf[6..7] 是片上温度，这里用不上，跳过 */
    gx = (int16_t)(((u16)buf[8]  << 8) | buf[9]);
    gy = (int16_t)(((u16)buf[10] << 8) | buf[11]);
    gz = (int16_t)(((u16)buf[12] << 8) | buf[13]);

    dt = (float)GYRO_SAMPLE_MS / 1000.0f;

    /* 去掉零偏，单位换成 °/s */
    rx = (float)gx / GYRO_GYRO_LSB - gyro_bias_x;
    ry = (float)gy / GYRO_GYRO_LSB - gyro_bias_y;
    rz = (float)gz / GYRO_GYRO_LSB - gyro_bias_z;

    gyro_rate_z10 = (int16_t)(rz * 10.0f);

    /* ---- 航向：纯陀螺积分 ----
     * 模块 Z 轴朝上时，右手定则的正方向是逆时针。我们约定"右转为正"，
     * 所以取负号；GYRO_YAW_CW_POSITIVE 置 0 就换回相反方向。 */
#if GYRO_YAW_CW_POSITIVE
    gyro_yaw -= rz * dt;
#else
    gyro_yaw += rz * dt;
#endif
    if (gyro_yaw >  180.0f) gyro_yaw -= 360.0f;
    if (gyro_yaw < -180.0f) gyro_yaw += 360.0f;
    gyro_yaw10 = (int16_t)(gyro_yaw * 10.0f);

    /* ---- 倾角：加速度计算重力方向（静止时无漂移，但电机加速时会抖） ---- */
    /* roll  = atan2(ay, az)                                   —— 绕 X 轴（车头方向） */
    /* pitch = atan2(-ax, sqrt(ay² + az²))                     —— 绕 Y 轴（左右方向） */
    roll_acc  = (float)(atan2((double)ay, (double)az) * 57.29577951308232);
    pitch_acc = (float)(atan2((double)(-ax),
                              sqrt((double)ay * (double)ay + (double)az * (double)az))
                        * 57.29577951308232);

    /* ---- 互补滤波：陀螺积分走"快"分量，加速度计每拍只拉 2% 回来 ---- */
    a = (float)GYRO_COMP_ALPHA / 100.0f;

    gyro_pitch = a * (gyro_pitch + (float)GYRO_PITCH_RATE_SIGN * ry * dt)
               + (1.0f - a) * pitch_acc;
    gyro_roll  = a * (gyro_roll  + (float)GYRO_ROLL_RATE_SIGN  * rx * dt)
               + (1.0f - a) * roll_acc;

    gyro_pitch10 = (int16_t)(gyro_pitch * 10.0f);
    gyro_roll10  = (int16_t)(gyro_roll  * 10.0f);
}

/* ============================== 取值接口 ============================== */

u8      Gyro_IsReady(void)   { return gyro_ready; }
u8      Gyro_GetId(void)     { return gyro_id; }
int16_t Gyro_GetYaw10(void)  { return gyro_yaw10; }
int16_t Gyro_GetPitch10(void){ return gyro_pitch10; }
int16_t Gyro_GetRoll10(void) { return gyro_roll10; }
int16_t Gyro_GetRateZ10(void){ return gyro_rate_z10; }
int16_t Gyro_GetBiasZ10(void){ return gyro_bias_z10; }

int16_t Gyro_AngleDiff10(int16_t a, int16_t b)
{
    return ang_wrap10((int32_t)a - (int32_t)b);
}

/* ============================== 角度闭环转向 ============================== */

/* 阻塞式原地转向：转到 yaw 变化 delta10 为止。
 * 正 delta10 = 顺时针（右转）→ 左轮前进、右轮后退。
 * 用于"车已经停下、需要精确摆一个角度"的场合（避障绕行、掉头、丢线找回）。
 * 返回 1 = 到位，0 = 超时/陀螺仪没就绪。 */
u8 Gyro_TurnBy(int16_t delta10, int max_speed, u16 timeout_ms)
{
    int16_t target;
    int16_t remain;
    int     pwm;

    if (!gyro_ready)
    {
        return 0;
    }

    if (max_speed > 95)            max_speed = 95;   /* 不超过 Track 的上限 */
    if (max_speed < GYRO_TURN_MIN) max_speed = GYRO_TURN_MIN;

    target = ang_wrap10((int32_t)gyro_yaw10 + (int32_t)delta10);

    while (timeout_ms >= GYRO_SAMPLE_MS)
    {
        Gyro_Task();                                  /* 自己推进采样，不依赖主循环 */
        remain = ang_wrap10((int32_t)target - (int32_t)gyro_yaw10);

        if (remain <= GYRO_TURN_TOL10 && remain >= -GYRO_TURN_TOL10)
        {
            Motor_Stop();
            return 1;
        }

        /* 比例输出：差 30° 以上满速，接近后自动减速，避免冲过头来回震荡 */
        pwm = (int)((int32_t)remain * max_speed / 300);
        if (pwm >  max_speed) pwm =  max_speed;
        if (pwm < -max_speed) pwm = -max_speed;
        if (pwm >= 0 && pwm <  GYRO_TURN_MIN) pwm =  GYRO_TURN_MIN;
        if (pwm <  0 && pwm > -GYRO_TURN_MIN) pwm = -GYRO_TURN_MIN;

        Motor_SetSpeed(pwm, -pwm);                    /* 正 = 右转：左轮正、右轮反 */

        delay_ms(GYRO_SAMPLE_MS);
        timeout_ms -= GYRO_SAMPLE_MS;
    }

    Motor_Stop();
    return 0;
}

#endif /* GYRO_ENABLE */
