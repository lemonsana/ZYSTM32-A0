#include "imu.h"
#include "delay.h"
#include "stm32f10x.h"

/* ============================================================================
 *  MPU6050 驱动（软件 I2C + 单轴积分）
 *  ----------------------------------------------------------------------------
 *  只读 GYRO_Z 两个字节（0x47/0x48），一次约 200us —— 10ms 周期下占 2% CPU。
 *  不读加速度：yaw 没有绝对参考，加速度帮不上忙；倾斜检测以后再加。
 * ========================================================================== */

/* ---------------- 寄存器 ---------------- */
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_GYRO_ZOUT_H     0x47
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

#define MPU_WHO_AM_I_VAL        0x68
#define MPU_I2C_ERR_LIMIT       20      /* 连续读错这么多次就降级（imu_ready=0） */

static u8    imu_ready   = 0;
static u8    imu_id      = 0;
static u8    imu_err     = 0;
static float imu_yaw     = 0.0f;
static float imu_gz      = 0.0f;
static float imu_gz_last = 0.0f;
static float imu_bias    = 0.0f;

/* ==========================================================================
 *  软件 I2C
 * ========================================================================== */
#define SCL_HIGH()   GPIO_SetBits(IMU_SCL_GPIO, IMU_SCL_PIN)
#define SCL_LOW()    GPIO_ResetBits(IMU_SCL_GPIO, IMU_SCL_PIN)
#define SDA_HIGH()   GPIO_SetBits(IMU_SDA_GPIO, IMU_SDA_PIN)
#define SDA_LOW()    GPIO_ResetBits(IMU_SDA_GPIO, IMU_SDA_PIN)
#define SDA_READ()   GPIO_ReadInputDataBit(IMU_SDA_GPIO, IMU_SDA_PIN)

static void imu_delay(void)
{
    delay_us(IMU_I2C_DELAY_US);
}

/* 从机把 SDA 拉死时，打 9 个时钟放出来 */
static void imu_bus_recover(void)
{
    u8 i;

    SDA_HIGH();
    for (i = 0; i < 9; i++)
    {
        SCL_LOW();
        imu_delay();
        SCL_HIGH();
        imu_delay();
    }
    SCL_LOW();
    imu_delay();
    SDA_HIGH();
    imu_delay();
}

static void imu_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    imu_delay();
    SDA_LOW();
    imu_delay();
    SCL_LOW();
    imu_delay();
}

static void imu_stop(void)
{
    SDA_LOW();
    imu_delay();
    SCL_HIGH();
    imu_delay();
    SDA_HIGH();
    imu_delay();
}

/* 返回 0 = 收到 ACK */
static u8 imu_write_byte(u8 value)
{
    u8 i;
    u8 nack;

    for (i = 0; i < 8; i++)
    {
        if (value & 0x80) SDA_HIGH();
        else              SDA_LOW();
        value = (u8)(value << 1);

        imu_delay();
        SCL_HIGH();
        imu_delay();
        SCL_LOW();
        imu_delay();
    }

    SDA_HIGH();
    imu_delay();
    SCL_HIGH();
    imu_delay();
    nack = SDA_READ();
    SCL_LOW();
    imu_delay();

    return nack;
}

/* ack=1 继续读，0 读最后一字节 */
static u8 imu_read_byte(u8 ack)
{
    u8 i;
    u8 value = 0;

    SDA_HIGH();
    imu_delay();

    for (i = 0; i < 8; i++)
    {
        value = (u8)(value << 1);
        SCL_HIGH();
        imu_delay();
        if (SDA_READ()) value |= 0x01;
        SCL_LOW();
        imu_delay();
    }

    if (ack) SDA_LOW();
    else     SDA_HIGH();
    imu_delay();
    SCL_HIGH();
    imu_delay();
    SCL_LOW();
    imu_delay();
    SDA_HIGH();
    imu_delay();

    return value;
}

static u8 imu_write_reg(u8 reg, u8 value)
{
    imu_start();
    if (imu_write_byte((u8)(IMU_I2C_ADDR << 1))) { imu_stop(); return 0; }
    if (imu_write_byte(reg))                     { imu_stop(); return 0; }
    if (imu_write_byte(value))                   { imu_stop(); return 0; }
    imu_stop();
    return 1;
}

static u8 imu_read_regs(u8 reg, u8 *buf, u8 len)
{
    u8 i;

    if (buf == 0 || len == 0) return 0;

    imu_start();
    if (imu_write_byte((u8)(IMU_I2C_ADDR << 1)))       { imu_stop(); return 0; }
    if (imu_write_byte(reg))                           { imu_stop(); return 0; }

    imu_start();
    if (imu_write_byte((u8)((IMU_I2C_ADDR << 1) | 1))) { imu_stop(); return 0; }

    for (i = 0; i < len; i++)
    {
        buf[i] = imu_read_byte((u8)(i < (u8)(len - 1)));
    }
    imu_stop();
    return 1;
}

static u8 imu_read_reg(u8 reg, u8 *value)
{
    return imu_read_regs(reg, value, 1);
}

/* ==========================================================================
 *  初始化
 * ========================================================================== */
static void imu_gpio_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(IMU_GPIO_RCC, ENABLE);

    gpio.GPIO_Pin   = IMU_SCL_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(IMU_SCL_GPIO, &gpio);

    gpio.GPIO_Pin   = IMU_SDA_PIN;
    GPIO_Init(IMU_SDA_GPIO, &gpio);

    SCL_HIGH();
    SDA_HIGH();
}

/* 读一次原始 GYRO_Z */
static u8 imu_read_gz_raw(s16 *raw)
{
    u8 buf[2];

    if (!imu_read_regs(MPU_REG_GYRO_ZOUT_H, buf, 2)) return 0;

    *raw = (s16)(((u16)buf[0] << 8) | (u16)buf[1]);
    return 1;
}

/* 静止标定：取平均当零偏，同时看波动判断"标定时车是否在动" */
static u8 imu_calibrate(void)
{
    u16 n = (u16)(IMU_CALIB_MS / IMU_UPDATE_MS);
    u16 i;
    u8  ok = 1;
    s16 raw;
    float sum = 0.0f;
    float v;
    float vmin = 1000.0f;
    float vmax = -1000.0f;

    if (n < 10) n = 10;

    for (i = 0; i < n; i++)
    {
        if (!imu_read_gz_raw(&raw))
        {
            ok = 0;
            break;
        }
        v = (float)raw / IMU_GYRO_LSB;
        sum += v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        delay_ms(IMU_UPDATE_MS);
    }

    if (!ok) return 0;

    imu_bias    = sum / (float)n;
    imu_gz      = 0.0f;
    imu_gz_last = 0.0f;
    imu_yaw     = 0.0f;

    /* 峰峰值 > 6dps 说明标定时车在动，零偏会不准（不致命，返回 2 提示） */
    if ((vmax - vmin) > 6.0f) return 2;

    return 1;
}

u8 Imu_Init(void)
{
    imu_ready   = 0;
    imu_err     = 0;
    imu_id      = 0;
    imu_yaw     = 0.0f;
    imu_gz      = 0.0f;
    imu_gz_last = 0.0f;
    imu_bias    = 0.0f;

    imu_gpio_init();
    imu_bus_recover();
    delay_ms(50);

    if (!imu_read_reg(MPU_REG_WHO_AM_I, &imu_id)) return 0;
    if (imu_id != MPU_WHO_AM_I_VAL)               return 0;

    imu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);      /* 软复位 */
    delay_ms(100);

    imu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);      /* 唤醒，时钟源 = X 轴陀螺 PLL */
    delay_ms(10);

    imu_write_reg(MPU_REG_SMPLRT_DIV, 9);         /* 1kHz / (1+9) = 100Hz */
    imu_write_reg(MPU_REG_CONFIG, 0x03);          /* DLPF：陀螺 42Hz */
    imu_write_reg(MPU_REG_GYRO_CONFIG, 0x08);     /* ±500 °/s */
    imu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);    /* ±2g（暂时不用） */
    delay_ms(50);

    if (imu_calibrate() == 0) return 0;

    imu_ready = 1;
    return 1;
}

/* ==========================================================================
 *  周期更新：读一次 → 去掉零偏 → 梯形积分
 * ========================================================================== */
void Imu_Update(void)
{
    s16 raw;
    float gz;
    float dt = (float)IMU_UPDATE_MS / 1000.0f;

    if (!imu_ready) return;

    if (!imu_read_gz_raw(&raw))
    {
        if (imu_err < 255) imu_err++;
        if (imu_err >= MPU_I2C_ERR_LIMIT)
        {
            imu_ready = 0;      /* 连续读不到：上层自动退回"没有陀螺"的行为 */
        }
        return;
    }

    imu_err = 0;
    gz = (float)raw / IMU_GYRO_LSB - imu_bias;
    imu_gz = gz;

    /* 梯形积分：这一拍和上一拍的平均角速度 × dt */
    imu_yaw += (float)IMU_YAW_SIGN * 0.5f * (gz + imu_gz_last) * dt;
    imu_gz_last = gz;
}

/* ==========================================================================
 *  查询接口
 * ========================================================================== */
u8    Imu_IsReady(void) { return imu_ready; }
u8    Imu_WhoAmI(void)  { return imu_id; }
float Imu_GetYaw(void)  { return imu_yaw; }
float Imu_GetGz(void)   { return imu_gz; }
float Imu_GetBias(void) { return imu_bias; }

void Imu_ResetYaw(void)
{
    imu_yaw     = 0.0f;
    imu_gz_last = imu_gz;       /* 免得复位后第一拍用旧值积出一大块 */
}
