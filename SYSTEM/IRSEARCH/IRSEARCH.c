#include "IRSEARCH.h"
#include "delay.h"
#include "motor.h"
#include "cfg.h"
#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块  v6.0
 *  --------------------------------------------------------------------------
 *  算法来源：可循迹版本（已验证）
 *    - I2C 读 8 路"归一化模拟量"，加权重心求线位置
 *    - 一阶低通滤波 + 中心死区 + 比例转向
 *    - 丢线时朝最后看到的方向原地找线，超时停车
 *  改良：
 *    - 电机统一走非阻塞 Motor_SetSpeed()（v3 已把通道映射改回实测正确值）
 *    - 丢线超时按毫秒计
 *    - 串行接口作为备选（GRAY_IF_I2C = 0）
 *  V6 新增：
 *    - 覆盖度判别：几路压线 → 区分"正常压线 / 宽黑(十字·横带) / 丢线"
 *    - 事件分层：采样 → 事件 → 决策 → 控制，十字不再靠"误差恰好为 0"
 *    - 速度斜坡：只作用在基础速度上，起步不打滑、差速响应不打折
 *    - 参数运行期可调：全部走 cfg.c（串口 SET / SAVE 存 Flash）
 * ========================================================================== */

#define GRAY_I2C_DELAY_US        5
#define GRAY_PING_RETRIES        50
#define GRAY_PING_INTERVAL_MS    20
#define GRAY_MIN_ACTIVE_WEIGHT   100

/* 串行接口的极性：模块 DAT 的 bit 为 1 时是黑线(1) 还是白底(0) */
#define GRAY_SERIAL_BIT_IS_BLACK 0

/* ---- 采样结果（V6）：位置 + 覆盖度 ---- */
typedef struct
{
    int16_t error;      /* 加权重心误差 -100~+100 */
    u8      active;     /* 有几路压线 0~8 —— 这就是"线宽/覆盖度" */
    u8      on_line;    /* 1 = 有效压线；0 = 一路都没压实（丢线） */
} gray_frame_t;

static u8     gray_ready = 0;       /* 传感器就绪 */
static u8     gray_filter_ready = 0;
static int16_t gray_filtered_error = 0;
static int16_t gray_last_error = 0;
static u16    gray_lost_ms = 0;

static int16_t gray_last_raw_error = 0;   /* 最近一次原始误差（调试用） */

/* ---- V6 事件层状态 ---- */
static u8      gray_active       = 0;                  /* 最近一帧几路压线 */
static u8      gray_event        = GRAY_EV_NONE;       /* 当前事件 */
static u8      gray_wide_cnt     = 0;                  /* 连续宽黑帧数（确认用） */
static u8      gray_lost_cnt     = 0;                  /* 连续丢线帧数（确认用） */
static u8      gray_wide_started = 0;                  /* 本次宽黑是否已计过数 */
static u16     gray_wide_ms      = 0;                  /* 本次宽黑已持续多久 */
static u16     gray_cross_count  = 0;                  /* 上电以来宽黑次数 */

/* ---- V6 速度斜坡状态（只作用在基础速度上） ---- */
static int16_t gray_base_ramp = 0;

/* ---- PID 状态 ---- */
static int32_t gray_i_sum      = 0;       /* 积分累加，单位：误差·毫秒 */
static int16_t gray_prev_error = 0;       /* 上一帧误差（D 项用） */
static int16_t gray_correction = 0;       /* 最近一次 PID 输出（调试用） */
static int16_t gray_base_speed = 0;       /* 最近一次输出用的基础速度（调试用） */

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
    if (!gray_i2c_read_reg(GW_GRAY_ANALOG_MODE, values8, 8)) return 0;

#if GRAY_ANALOG_INVERT
    /* 本车模块：黑线输出大、白底输出小 → 反相成"白≈255 / 黑≈0"的统一约定 */
    {
        u8 i;
        for (i = 0; i < 8; i++) values8[i] = (u8)(255 - values8[i]);
    }
#endif

    return 1;
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

    gray_active       = 0;
    gray_event        = GRAY_EV_NONE;
    gray_wide_cnt     = 0;
    gray_lost_cnt     = 0;
    gray_wide_started = 0;
    gray_wide_ms      = 0;
    gray_cross_count  = 0;
    gray_base_ramp    = 0;
    gray_correction   = 0;
    gray_base_speed   = 0;
    gray_i_sum        = 0;
    gray_prev_error   = 0;

    gray_iface_init();
    delay_ms(50);                   /* 等模块上电稳定 */

    if (!gray_iface_ping())  return;
    if (!gray_iface_config()) return;

    gray_ready = 1;
}

/* 读一次 8 路，算出：位置误差 + 覆盖度（几路压线）
 * 返回：1 = 读到了（*f 已填）；0 = I2C 读失败
 * 注意：丢线时 *f 仍然有效（on_line = 0），由事件层决定怎么处理
 */
static u8 gray_read_frame(gray_frame_t *f)
{
    u8  values[8];
    u8  i;
    int16_t weight;
    int16_t pos;
    int32_t sum_weight = 0;
    int32_t sum_position = 0;
    u8      active = 0;
    int16_t cutoff = Cfg_Get(CFG_CUTOFF);

    if (!gray_iface_read(values)) return 0;

    for (i = 0; i < 8; i++)
    {
        /* 归一化后：白≈255，黑≈0。越黑权重越大 */
        weight = (int16_t)(cutoff - (int16_t)values[i]);
        if (weight <= 0)
        {
            weight = 0;
        }
        else
        {
            active++;               /* 这一路压到线了 —— 覆盖度 */
        }

        pos = gray_position_weight[i];
#if GRAY_CHANNEL_MIRROR
        pos = (int16_t)(-pos);
#endif
        sum_weight   += weight;
        sum_position += (int32_t)weight * pos;
    }

    f->active = active;

    if (sum_weight < GRAY_MIN_ACTIVE_WEIGHT)
    {
        f->error   = 0;
        f->on_line = 0;
        return 1;
    }

    f->error   = (int16_t)((sum_position * 100L) / (sum_weight * 7L));
    f->on_line = 1;
    return 1;
}

/* ---------------------------------------------------------------- 速度斜坡
 *  base_ramp += (target - base_ramp) × ALPHA / 1000
 *  只调基础速度：差速修正量不经过这里，所以急弯里内轮该反转还是立刻反转，
 *  但起步、找回线之后是缓加速，轮胎不打滑。
 */
static int16_t gray_ramp_step(int16_t cur, int16_t target)
{
    int16_t alpha = Cfg_Get(CFG_RAMP);
    int16_t d;
    int16_t step;

    if (alpha >= 1000) return target;                   /* 1000 = 关闭斜坡 */

    d = (int16_t)(target - cur);
    if (d == 0) return cur;

    step = (int16_t)(((int32_t)d * alpha) / 1000);
    if (step == 0) step = (int16_t)((d > 0) ? 1 : -1);  /* 保证最终贴得上 */

    return (int16_t)(cur + step);
}

/* ---------------------------------------------------------------- 事件层
 *  进事件要连续 N 帧确认（去抖）；退事件立刻生效（避免卡在 EV_WIDE）。
 */
static u8 gray_update_event(const gray_frame_t *f)
{
    u8 ev = gray_event;

    if (!f->on_line)
    {
        if (gray_lost_cnt < 200) gray_lost_cnt++;
        gray_wide_cnt = 0;
        if (gray_lost_cnt >= GRAY_LOST_CONFIRM) ev = GRAY_EV_LOST;
    }
    else if (f->active >= (u8)Cfg_Get(CFG_WIDE_MIN))
    {
        if (gray_wide_cnt < 200) gray_wide_cnt++;
        gray_lost_cnt = 0;
        if (gray_wide_cnt >= GRAY_WIDE_CONFIRM) ev = GRAY_EV_WIDE;
    }
    else
    {
        gray_lost_cnt = 0;
        gray_wide_cnt = 0;
        ev = GRAY_EV_LINE;
    }

    /* 宽黑持续计时 + 上升沿计数（"过了几个十字/横线"） */
    if (ev == GRAY_EV_WIDE)
    {
        if (!gray_wide_started)
        {
            gray_wide_started = 1;
            gray_wide_ms = 0;
            if (gray_cross_count < 60000) gray_cross_count++;
        }
        else if (gray_wide_ms < 60000)
        {
            gray_wide_ms += GRAY_FRAME_MS;
        }
    }
    else
    {
        gray_wide_started = 0;
        gray_wide_ms = 0;
    }

    gray_event = ev;
    return ev;
}

/* ============================================================================
 *  循迹主任务（每 GRAY_FRAME_MS 调用一次）
 *  ----------------------------------------------------------------------------
 *  V6 把一帧拆成四层，现场调试时按层排查：
 *
 *    ① 采样   gray_read_frame()   → error(位置) + active(几路压线)
 *    ② 事件   gray_update_event() → LINE / WIDE / LOST（连续 N 帧确认）
 *    ③ 决策   switch(event)：这一帧要不要走 PID？要不要停？
 *    ④ 控制   PID + 斜坡 + 限幅 → Motor_SetSpeed()
 *
 *  为什么要分这四层：
 *      加权重心是对称的（±7±5±3±1），8 路全黑时 error 恰好等于 0，看起来
 *      "很正"，其实只是"很对称"——和真的压在 2cm 黑线上完全无法区分。
 *      现在用覆盖度（几路压线）把十字/宽黑带单独识别成 EV_WIDE，走"锁 0
 *      直行"这一条独立分支，不再混在 PID 里靠运气。
 * ========================================================================== */
void SearchRun(void)
{
    gray_frame_t f;
    int16_t raw_error;
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
    int16_t deadband;
    u8      ev;

    if (!gray_ready)
    {
        gray_i_sum = 0;
        Motor_Stop();
        return;
    }

    if (!gray_read_frame(&f))               /* I2C 读失败：停车等下一帧 */
    {
        gray_i_sum     = 0;
        gray_base_ramp = 0;
        Motor_Stop();
        return;
    }

    gray_active = f.active;
    ev = gray_update_event(&f);

    /* ============================ 决策层 ============================ */
    switch (ev)
    {
        /* ---------------- 丢线：朝最后方向找，超时停车 ---------------- */
        case GRAY_EV_LOST:
            gray_filter_ready   = 0;
            gray_last_raw_error = 999;
            gray_i_sum          = 0;
            gray_base_ramp      = 0;
            gray_base_speed     = 0;
            gray_correction     = 0;

            if (gray_lost_ms < GRAY_LOST_STOP_MS) gray_lost_ms += GRAY_FRAME_MS;

            if (gray_lost_ms >= GRAY_LOST_STOP_MS)
            {
                Motor_Stop();
            }
            else if (gray_last_error > GRAY_CENTER_DEADBAND)
            {
                Motor_SetSpeed(GRAY_LOST_SPIN_SPEED, -GRAY_LOST_SPIN_SPEED);   /* 向右找 */
            }
            else if (gray_last_error < -GRAY_CENTER_DEADBAND)
            {
                Motor_SetSpeed(-GRAY_LOST_SPIN_SPEED, GRAY_LOST_SPIN_SPEED);   /* 向左找 */
            }
            else
            {
                Motor_Stop();
            }
            return;

        /* ---------------- 宽黑：十字 / 横黑带 / 停车线 → 锁 0 直行 ---------------- */
        case GRAY_EV_WIDE:
            gray_i_sum          = 0;
            gray_prev_error     = 0;
            gray_filter_ready   = 0;        /* 出宽黑后重新起滤，别把 0 当历史值 */
            gray_last_raw_error = 0;
            gray_correction     = 0;
            gray_lost_ms        = 0;

#if (GRAY_WIDE_STOP_MS > 0)
            if (gray_wide_ms >= GRAY_WIDE_STOP_MS)   /* 停车区 / 终点：停住 */
            {
                gray_base_ramp  = 0;
                gray_base_speed = 0;
                Motor_Stop();
                return;
            }
#endif

            base_speed = Cfg_Get(CFG_BASE);
            if (base_speed < Cfg_Get(CFG_MIN_SPEED)) base_speed = Cfg_Get(CFG_MIN_SPEED);

            gray_base_ramp  = gray_ramp_step(gray_base_ramp, base_speed);
            gray_base_speed = gray_base_ramp;
            Motor_SetSpeed(gray_base_ramp, gray_base_ramp);
            return;

        default:
            break;
    }

    /* ========================== 控制层：正常压线 ========================== */
    raw_error = f.error;
    gray_lost_ms        = 0;
    gray_last_raw_error = raw_error;

    /* 一阶低通，抑制单帧跳变 */
    if (!gray_filter_ready)
    {
        gray_filtered_error = raw_error;
        gray_filter_ready = 1;
    }
    else
    {
        gray_filtered_error = (int16_t)((gray_filtered_error * 3 + raw_error) / 4);
    }

    deadband = Cfg_Get(CFG_DEADBAND);
    error = gray_filtered_error;
    if (error > -deadband && error < deadband) error = 0;   /* 中心死区 */
    gray_last_error = error;

    abs_e  = (error >= 0) ? error : (int16_t)(-error);
    d_err  = (int16_t)(error - gray_prev_error);
    abs_de = (d_err >= 0) ? d_err : (int16_t)(-d_err);

    /* 速度调度：偏差大、偏差变化快 → 收油 */
    base_speed = (int16_t)(Cfg_Get(CFG_BASE)
                         - (abs_e  * Cfg_Get(CFG_SLOW))   / 100
                         - (abs_de * Cfg_Get(CFG_SLOW_D)) / 100);
    if (base_speed < Cfg_Get(CFG_MIN_SPEED)) base_speed = Cfg_Get(CFG_MIN_SPEED);

    /* 增益调度：小误差用 GAIN，大误差用 FAR，中间线性过渡 */
    if (abs_e >= Cfg_Get(CFG_FAR_THD))
    {
        gain = Cfg_Get(CFG_GAIN_FAR);
    }
    else
    {
        gain = (int16_t)(Cfg_Get(CFG_GAIN)
             + ((int32_t)(Cfg_Get(CFG_GAIN_FAR) - Cfg_Get(CFG_GAIN)) * abs_e)
               / Cfg_Get(CFG_FAR_THD));
    }

    p_term = (int16_t)((int32_t)error * gain / 100L);

    /* I 项：已经回中线时让积分慢慢泄放，避免"到点了还在往一边拧" */
    if (Cfg_Get(CFG_KI) > 0)
    {
        if (error == 0)
        {
            gray_i_sum = (gray_i_sum * 99L) / 100L;
        }
        else
        {
            gray_i_sum += (int32_t)error * GRAY_FRAME_MS;
            if (gray_i_sum >  GRAY_I_LIMIT) gray_i_sum =  GRAY_I_LIMIT;
            if (gray_i_sum < -GRAY_I_LIMIT) gray_i_sum = -GRAY_I_LIMIT;
        }
        i_term = (int16_t)(gray_i_sum * Cfg_Get(CFG_KI) / 1000L);
    }
    else
    {
        gray_i_sum = 0;
        i_term = 0;
    }

    d_term = (int16_t)((int32_t)d_err * Cfg_Get(CFG_KD) / 100L);

    correction = (int16_t)(p_term + i_term + d_term);

    /* 基础速度走斜坡；差速修正量不走斜坡，急弯响应不打折 */
    gray_base_ramp  = gray_ramp_step(gray_base_ramp, base_speed);
    gray_base_speed = gray_base_ramp;

    /* ---- 转向量归一化 + 速度补偿 ----
     *   k = (100/FULL) × (base_ramp/REF)
     *   100/FULL：线压到阵列边缘只有 55~70 的误差，按 FULL 归一化后"线到头"才算满舵
     *   base/REF：差速需求 ∝ 车速，60 速为 1.0 倍
     *   这里用**斜坡后**的速度：起步阶段实车才跑 10，若按目标 60 算修正量，
     *   会出现"两轮对搓"——外侧还没起来、内侧已经反向转。
     */
    {
        int32_t k = ((int32_t)gray_base_ramp * 100L / Cfg_Get(CFG_REF_SPEED))
                  * 100L / Cfg_Get(CFG_FULL_ERR);

        if (k > GRAY_STEER_GAIN_MAX) k = GRAY_STEER_GAIN_MAX;
        correction = (int16_t)((int32_t)correction * k / 100L);
    }

    gray_correction = correction;
    gray_prev_error = error;

    left_speed  = (int16_t)(gray_base_ramp + correction);
    right_speed = (int16_t)(gray_base_ramp - correction);

    /* 急弯允许内轮倒转（"原地搓"能过 180°），但要限幅防止原地打转 */
    if (left_speed  < -Cfg_Get(CFG_INNER_REV)) left_speed  = (int16_t)(-Cfg_Get(CFG_INNER_REV));
    if (left_speed  >  95)                     left_speed  =  95;
    if (right_speed < -Cfg_Get(CFG_INNER_REV)) right_speed = (int16_t)(-Cfg_Get(CFG_INNER_REV));
    if (right_speed >  95)                     right_speed =  95;

    Motor_SetSpeed(left_speed, right_speed);
}

/* ---------------- 调试接口 ---------------- */
u8  Gray_IsReady(void)       { return gray_ready; }
int Gray_GetError(void)      { return gray_last_raw_error; }
int Gray_GetCorrection(void) { return gray_correction; }
int Gray_GetBaseSpeed(void)  { return gray_base_speed; }
u8  Gray_GetActive(void)     { return gray_active; }
u8  Gray_GetEvent(void)      { return gray_event; }
u16 Gray_GetCrossCount(void) { return gray_cross_count; }

void Gray_ResetCrossCount(void) { gray_cross_count = 0; }
void Gray_ResetRamp(void)       { gray_base_ramp = 0; }

const char *Gray_EventName(u8 ev)
{
    switch (ev)
    {
        case GRAY_EV_LINE: return "LINE";
        case GRAY_EV_WIDE: return "WIDE";
        case GRAY_EV_LOST: return "LOST";
        default:           return "NONE";
    }
}

/* 清状态：切模式、重新上路、手动复位时调用 */
void Gray_ResetIntegral(void)
{
    gray_i_sum          = 0;
    gray_prev_error     = 0;
    gray_filter_ready   = 0;
    gray_filtered_error = 0;
    gray_lost_ms        = 0;
    gray_base_ramp      = 0;
    gray_wide_cnt       = 0;
    gray_lost_cnt       = 0;
    gray_wide_started   = 0;
    gray_wide_ms        = 0;
}

/* 是否压到黑线：任意一路归一化值 < 黑线阈值 即算压线
 * 融合模式（循迹+避障）绕完障碍后，用它判断有没有重新找到线 */
u8 Gray_IsOnLine(void)
{
    u8 values[8];
    u8 i;
    int16_t cutoff = Cfg_Get(CFG_CUTOFF);

    if (!gray_ready) return 0;
    if (!gray_iface_read(values)) return 0;

    for (i = 0; i < 8; i++)
    {
        if (values[i] < cutoff) return 1;
    }
    return 0;
}

u8 Gray_ReadRaw(u8 *values8) { return gray_iface_read(values8); }

