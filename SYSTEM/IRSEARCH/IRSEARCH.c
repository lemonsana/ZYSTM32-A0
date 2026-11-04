#include "IRSEARCH.h"
#include "delay.h"
#include "motor.h"
#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块  v3.0
 *  --------------------------------------------------------------------------
 *  算法来源：可循迹版本（已验证）
 *    - I2C 读 8 路"归一化模拟量"，加权重心求线位置
 *    - 一阶低通滤波 + 中心死区 + 比例转向
 *    - 丢线时朝最后看到的方向原地找线，超时停车
 *  改良：
 *    - 电机统一走非阻塞 Motor_SetSpeed()（v3 已把通道映射改回实测正确值）
 *    - 丢线超时按毫秒计
 *    - 串行接口作为备选（GRAY_IF_I2C = 0）
 * ========================================================================== */

#define GRAY_I2C_DELAY_US        5
#define GRAY_PING_RETRIES        50
#define GRAY_PING_INTERVAL_MS    20
#define GRAY_MIN_ACTIVE_WEIGHT   100

/* 串行接口的极性：模块 DAT 的 bit 为 1 时是黑线(1) 还是白底(0) */
#define GRAY_SERIAL_BIT_IS_BLACK 0

static u8     gray_ready = 0;       /* 传感器就绪 */
static u8     gray_filter_ready = 0;
static int16_t gray_filtered_error = 0;
static int16_t gray_last_error = 0;
static u16    gray_lost_ms = 0;

static int16_t gray_last_raw_error = 0;   /* 最近一次原始误差（调试用） */

/* ---- v3.1 PID 状态 ---- */
static int32_t gray_i_sum      = 0;       /* 积分累加，单位：误差·毫秒 */
static int16_t gray_prev_error = 0;       /* 上一帧误差（D 项用） */
static int16_t gray_correction = 0;       /* 最近一次 PID 输出（调试用） */
static int16_t gray_base_speed = 0;       /* 最近一次调度后的基础速度（调试用） */

/* 探头位置权重：下标 0 = 通道1（最左）…… 下标 7 = 通道8（最右）
 * 与"可循迹版本"一致；若实车方向相反，把 GRAY_CHANNEL_MIRROR 改成 1 */
static const int8_t gray_position_weight[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };

/* ==========================================================================
 *  接口层：I2C
 * ========================================================================== */
#if GRAY_IF_I2C

#define GRAY_SCL_HIGH()   GPIO_SetBits(GRAY_I2C_GPIO, GRAY_I2C_SCL_PIN)
#define GRAY_SCL_LOW()    GPIO_ResetBits(GRAY_I2C_GPIO, GRAY_I2C_SCL_PIN)
#define GRAY_SDA_HIGH()   GPIO_SetBits(GRAY_I2C_GPIO, GRAY_I2C_SDA_PIN)
#define GRAY_SDA_LOW()    GPIO_ResetBits(GRAY_I2C_GPIO, GRAY_I2C_SDA_PIN)
#define GRAY_SDA_READ()   GPIO_ReadInputDataBit(GRAY_I2C_GPIO, GRAY_I2C_SDA_PIN)

static void gray_i2c_delay(void)
{
    delay_us(GRAY_I2C_DELAY_US);
}

/* 总线恢复：SDA 被从机拉死时，打 9 个时钟把它放出来 */
static void gray_i2c_bus_recover(void)
{
    u8 i;

    GRAY_SDA_HIGH();
    for (i = 0; i < 9; i++)
    {
        GRAY_SCL_LOW();
        gray_i2c_delay();
        GRAY_SCL_HIGH();
        gray_i2c_delay();
    }
    GRAY_SCL_LOW();
    gray_i2c_delay();
    GRAY_SDA_HIGH();
    gray_i2c_delay();
}

static void gray_i2c_start(void)
{
    GRAY_SDA_HIGH();
    GRAY_SCL_HIGH();
    gray_i2c_delay();
    GRAY_SDA_LOW();
    gray_i2c_delay();
    GRAY_SCL_LOW();
    gray_i2c_delay();
}

static void gray_i2c_stop(void)
{
    GRAY_SDA_LOW();
    gray_i2c_delay();
    GRAY_SCL_HIGH();
    gray_i2c_delay();
    GRAY_SDA_HIGH();
    gray_i2c_delay();
}

/* 写 1 字节，返回 0 = ACK */
static u8 gray_i2c_write_byte(u8 value)
{
    u8 i;
    u8 nack;

    for (i = 0; i < 8; i++)
    {
        if (value & 0x80) GRAY_SDA_HIGH();
        else              GRAY_SDA_LOW();
        value <<= 1;
        gray_i2c_delay();
        GRAY_SCL_HIGH();
        gray_i2c_delay();
        GRAY_SCL_LOW();
        gray_i2c_delay();
    }

    GRAY_SDA_HIGH();
    gray_i2c_delay();
    GRAY_SCL_HIGH();
    gray_i2c_delay();
    nack = GRAY_SDA_READ();
    GRAY_SCL_LOW();
    gray_i2c_delay();

    return nack;
}

/* 读 1 字节；ack=1 表示给应答(继续读)，0 表示不应答(最后一字节) */
static u8 gray_i2c_read_byte(u8 ack)
{
    u8 i;
    u8 value = 0;

    GRAY_SDA_HIGH();
    gray_i2c_delay();

    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        GRAY_SCL_HIGH();
        gray_i2c_delay();
        if (GRAY_SDA_READ()) value |= 0x01;
        GRAY_SCL_LOW();
        gray_i2c_delay();
    }

    if (ack) GRAY_SDA_LOW();
    else     GRAY_SDA_HIGH();
    gray_i2c_delay();
    GRAY_SCL_HIGH();
    gray_i2c_delay();
    GRAY_SCL_LOW();
    gray_i2c_delay();
    GRAY_SDA_HIGH();
    gray_i2c_delay();

    return value;
}

static u8 gray_i2c_write_reg(u8 reg, u8 value)
{
    gray_i2c_start();
    if (gray_i2c_write_byte((u8)(GRAY_I2C_ADDR << 1))) { gray_i2c_stop(); return 0; }
    if (gray_i2c_write_byte(reg))                      { gray_i2c_stop(); return 0; }
    if (gray_i2c_write_byte(value))                    { gray_i2c_stop(); return 0; }
    gray_i2c_stop();
    return 1;
}

static u8 gray_i2c_read_reg(u8 reg, u8 *data, u8 length)
{
    u8 i;

    if ((data == 0) || (length == 0)) return 0;

    gray_i2c_start();
    if (gray_i2c_write_byte((u8)(GRAY_I2C_ADDR << 1)))       { gray_i2c_stop(); return 0; }
    if (gray_i2c_write_byte(reg))                             { gray_i2c_stop(); return 0; }

    gray_i2c_start();
    if (gray_i2c_write_byte((u8)((GRAY_I2C_ADDR << 1) | 1)))  { gray_i2c_stop(); return 0; }

    for (i = 0; i < length; i++)
    {
        data[i] = gray_i2c_read_byte((u8)(i < (u8)(length - 1)));
    }
    gray_i2c_stop();

    return 1;
}

static void gray_iface_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GRAY_I2C_SCL_PIN | GRAY_I2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(GRAY_I2C_GPIO, &gpio);

    GRAY_SCL_HIGH();
    GRAY_SDA_HIGH();
    gray_i2c_bus_recover();
}

static u8 gray_iface_ping(void)
{
    u8 i;
    u8 value;

    for (i = 0; i < GRAY_PING_RETRIES; i++)
    {
        value = 0;
        if (gray_i2c_read_reg(GW_GRAY_PING, &value, 1) && (value == GW_GRAY_PING_OK))
        {
            return 1;
        }
        delay_ms(GRAY_PING_INTERVAL_MS);
    }
    return 0;
}

static u8 gray_iface_config(void)
{
    if (!gray_i2c_write_reg(GW_GRAY_CHANNEL_EN, 0xFF)) return 0;   /* 8 通道全开 */
    if (!gray_i2c_write_reg(GW_GRAY_NORMALIZE,  0xFF)) return 0;   /* 归一化打开 */
    delay_ms(10);                                                   /* 等模块刷新 */
    return 1;
}

static u8 gray_iface_read(u8 *values8)
{
    return gray_i2c_read_reg(GW_GRAY_ANALOG_MODE, values8, 8);
}

/* ==========================================================================
 *  接口层：串行 CLK/DAT（备选）
 * ========================================================================== */
#else

#define GRAY_CLK_HIGH()   GPIO_SetBits(GRAY_CLK_GPIO, GRAY_CLK_PIN)
#define GRAY_CLK_LOW()    GPIO_ResetBits(GRAY_CLK_GPIO, GRAY_CLK_PIN)
#define GRAY_DAT_READ()   GPIO_ReadInputDataBit(GRAY_DAT_GPIO, GRAY_DAT_PIN)

static void gray_iface_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GRAY_CLK_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GRAY_CLK_GPIO, &gpio);

    gpio.GPIO_Pin   = GRAY_DAT_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GRAY_DAT_GPIO, &gpio);

    GRAY_CLK_HIGH();
    delay_ms(5);
}

static u8 gray_iface_ping(void)
{
    return 1;       /* 串行接口没有应答机制，读一次就知道 */
}

static u8 gray_iface_config(void)
{
    return 1;
}

/* 串行读 8 位，再换算成 0/255 的"模拟量"，方便复用同一套重心算法 */
static u8 gray_iface_read(u8 *values8)
{
    u8 i, bits = 0;

    for (i = 0; i < 8; i++)
    {
        GRAY_CLK_LOW();
        delay_us(2);
        if (GRAY_DAT_READ()) bits |= (u8)(1 << i);
        GRAY_CLK_HIGH();
        delay_us(5);
    }

    for (i = 0; i < 8; i++)
    {
#if GRAY_SERIAL_BIT_IS_BLACK
        values8[i] = (bits & (1 << i)) ? 0 : 255;
#else
        values8[i] = (bits & (1 << i)) ? 255 : 0;
#endif
    }
    return 1;
}

#endif  /* GRAY_IF_I2C */

/* ==========================================================================
 *  公共层：初始化 + 循迹控制
 * ========================================================================== */

void IRSearchInit(void)
{
    gray_ready        = 0;
    gray_filter_ready = 0;
    gray_filtered_error = 0;
    gray_last_error   = 0;
    gray_lost_ms      = 0;

    gray_iface_init();
    delay_ms(50);                   /* 等模块上电稳定 */

    if (!gray_iface_ping())  return;
    if (!gray_iface_config()) return;

    gray_ready = 1;
}

/* 读一次并算出线位置误差
 * 返回：1 找到线（*error = -100~+100）；2 丢线；0 读失败
 */
static u8 gray_calc_error(int16_t *error)
{
    u8  values[8];
    u8  i;
    int16_t weight;
    int16_t pos;
    int32_t sum_weight = 0;
    int32_t sum_position = 0;

    if (!gray_iface_read(values)) return 0;

    for (i = 0; i < 8; i++)
    {
        /* 归一化后：白≈255，黑≈0。越黑权重越大 */
        weight = (int16_t)GRAY_BLACK_CUTOFF - (int16_t)values[i];
        if (weight < 0) weight = 0;

        pos = gray_position_weight[i];
#if GRAY_CHANNEL_MIRROR
        pos = -pos;
#endif
        sum_weight   += weight;
        sum_position += (int32_t)weight * pos;
    }

    if (sum_weight < GRAY_MIN_ACTIVE_WEIGHT) return 2;      /* 没有探头压线 */

    *error = (int16_t)((sum_position * 100L) / (sum_weight * 7L));
    return 1;
}

/* ============================================================================
 *  循迹主任务（主循环每 GRAY_FRAME_MS 调用一次）
 *  ----------------------------------------------------------------------------
 *  一帧的处理顺序（调试时按这个顺序排查）：
 *      1. gray_calc_error()   读 8 路模拟量 → 加权重心 → 原始误差 raw_error
 *                             返回 0=读失败 / 1=找到线 / 2=丢线
 *      2. 丢线分支           朝"最后看到的方向"原地转着找，超时停车
 *      3. 一阶低通            滤掉单帧噪声，en = (en-1×3 + raw)/4
 *      4. 死区                中线附近强制归零，避免原地抖
 *      5. 速度调度            base 随 |误差| 和 |误差变化| 下降
 *      6. 增益调度            小误差用 GAIN，大误差用 GAIN_FAR
 *      7. PID                 计算 correction
 *      8. 输出                左=base+corr，右=base−corr，限幅后写电机
 *
 *  调试建议：串口里 err / corr / spd 三个数一起看
 *      err  = 原始误差（±100，999=丢线）
 *      corr = PID 输出（±GAIN 附近，顶到 ±50 说明满舵）
 *      spd  = 调度后的基础速度（最低 GRAY_MIN_SPEED）
 * ========================================================================== */
void SearchRun(void)
{
    int16_t raw_error = 0;
    int16_t error;
    int16_t abs_e;
    int16_t d_err;
    int16_t abs_de;
    int16_t base_speed;
    int16_t gain;
    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;
    int16_t p_term;
    int16_t i_term;
    int16_t d_term;
    u8 result;

    if (!gray_ready)
    {
        gray_i_sum = 0;
        Motor_Stop();
        return;
    }

    result = gray_calc_error(&raw_error);

    if (result == 0)                    /* I2C 读失败：停车等下一帧 */
    {
        gray_i_sum = 0;
        Motor_Stop();
        return;
    }

    if (result == 2)                    /* 丢线：朝最后方向找线 */
    {
        gray_filter_ready = 0;
        gray_last_raw_error = 999;
        gray_i_sum = 0;                 /* 丢线时清积分，防止恢复后猛冲 */

        if (gray_lost_ms < GRAY_LOST_STOP_MS) gray_lost_ms += GRAY_FRAME_MS;

        if (gray_lost_ms >= GRAY_LOST_STOP_MS)
        {
            Motor_Stop();               /* 找太久 → 停车 */
        }
        else if (gray_last_error > GRAY_CENTER_DEADBAND)
        {
            Motor_SetSpeed( GRAY_LOST_SPIN_SPEED, -GRAY_LOST_SPIN_SPEED);   /* 向右找 */
        }
        else if (gray_last_error < -GRAY_CENTER_DEADBAND)
        {
            Motor_SetSpeed(-GRAY_LOST_SPIN_SPEED,  GRAY_LOST_SPIN_SPEED);   /* 向左找 */
        }
        else
        {
            Motor_Stop();
        }
        return;
    }

    gray_lost_ms = 0;
    gray_last_raw_error = raw_error;

    /* 一阶低通滤波，抑制单帧跳变 */
    if (!gray_filter_ready)
    {
        gray_filtered_error = raw_error;
        gray_filter_ready = 1;
    }
    else
    {
        gray_filtered_error = (int16_t)((gray_filtered_error * 3 + raw_error) / 4);
    }

    error = gray_filtered_error;
    if (error > -GRAY_CENTER_DEADBAND && error < GRAY_CENTER_DEADBAND)
    {
        error = 0;                      /* 中心死区 */
    }
    gray_last_error = error;

    /* ---------------- PID 控制（v3.3：按 12cm 前视距离重调） ----------------
     *   增益调度：|误差| ≤ FAR_THRESHOLD 用 GAIN，超过用 GAIN_FAR（急弯更硬）
     *   速度调度：base 随 |误差| 下降；另外"误差变化越快"也减速（进弯前先收油）
     *   内轮允许反转：|误差| 大时内侧轮可以给到 -INNER_REVERSE，显著减小转弯半径
     * -------------------------------------------------------------------- */
    abs_e = (error >= 0) ? error : -error;

    /* 误差变化率（判断是否正在进入弯道） */
    d_err = (int16_t)(error - gray_prev_error);
    abs_de = (d_err >= 0) ? d_err : -d_err;

    /* 速度调度：大误差 + 大变化率 → 减速 */
    base_speed = (int16_t)(GRAY_BASE_SPEED
                         - (abs_e  * GRAY_SLOW_GAIN)   / 100
                         - (abs_de * GRAY_SLOW_D_GAIN) / 100);
    if (base_speed < GRAY_MIN_SPEED) base_speed = GRAY_MIN_SPEED;
    gray_base_speed = base_speed;

    /* 增益调度 */
    gain = (abs_e > GRAY_FAR_THRESHOLD) ? GRAY_STEER_GAIN_FAR : GRAY_STEER_GAIN;

    p_term = (int16_t)((int32_t)error * gain / 100L);

#if GRAY_KI > 0
    if (error == 0)
    {
        /* 已经回中线：让积分慢慢泄放，避免"到点了还在往一边拧" */
        gray_i_sum = (gray_i_sum * 99L) / 100L;
    }
    else
    {
        gray_i_sum += (int32_t)error * GRAY_FRAME_MS;
        if (gray_i_sum >  GRAY_I_LIMIT) gray_i_sum =  GRAY_I_LIMIT;
        if (gray_i_sum < -GRAY_I_LIMIT) gray_i_sum = -GRAY_I_LIMIT;
    }
    i_term = (int16_t)(gray_i_sum * GRAY_KI / 1000L);
#else
    i_term = 0;
#endif

#if GRAY_KD > 0
    d_term = (int16_t)((int32_t)(error - gray_prev_error) * GRAY_KD / 100L);
#else
    d_term = 0;
#endif

    correction = p_term + i_term + d_term;
    gray_correction = correction;
    gray_prev_error = error;

    left_speed  = base_speed + correction;
    right_speed = base_speed - correction;

    /* 急弯时允许内轮倒转（"原地搓"能过 180°），但要限幅防止原地打转 */
    if (left_speed  < -GRAY_INNER_REVERSE) left_speed  = -GRAY_INNER_REVERSE;
    if (left_speed  >  95)                 left_speed  =  95;
    if (right_speed < -GRAY_INNER_REVERSE) right_speed = -GRAY_INNER_REVERSE;
    if (right_speed >  95)                 right_speed =  95;

    Motor_SetSpeed(left_speed, right_speed);
}

/* ---------------- 调试接口 ---------------- */
u8 Gray_IsReady(void)   { return gray_ready; }
int Gray_GetError(void) { return gray_last_raw_error; }
int Gray_GetCorrection(void) { return gray_correction; }
int Gray_GetBaseSpeed(void) { return gray_base_speed; }

/* 清积分：切模式、重新上路、手动复位时调用 */
void Gray_ResetIntegral(void)
{
    gray_i_sum      = 0;
    gray_prev_error = 0;
    gray_filter_ready = 0;
    gray_lost_ms    = 0;
}

u8 Gray_ReadRaw(u8 *values8) { return gray_iface_read(values8); }
