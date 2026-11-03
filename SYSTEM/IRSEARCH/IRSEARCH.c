#include "IRSEARCH.h"
#include "delay.h"
#include "motor.h"
#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块  v2.1
 *  --------------------------------------------------------------------------
 *  LINE_USE_GRAY = 1  →  感为 8 路灰度（软件 I2C，PB10=SCL / PB11=SDA）
 *  LINE_USE_GRAY = 0  →  原来的两路红外探头（PA7=右 / PB0=左）
 * ========================================================================== */

#if LINE_USE_GRAY
/* ======================= 方案一：感为 8 路灰度 ======================= */

/* 数字量极性：
 *   0 = bit 为 1 表示"白"（黑线时该位为 0）—— 感为资料里的约定
 *   1 = bit 为 1 表示"黑"
 * 如果实测方向反了（车往线的反方向修正），把这个宏改一下即可。 */
#define GRAY_BIT_BLACK_IS_1   0

/* ---------------- 软件 I2C 底层（开漏输出） ---------------- */
static void IIC_Delay(void)
{
    delay_us(2);        /* 约 100kHz；模块不稳时把 2 改大一点 */
}

static void IIC_Start(void)
{
    GRAY_SDA_SET;    IIC_Delay();
    GRAY_SCL_SET;    IIC_Delay();
    GRAY_SDA_RESET;  IIC_Delay();
    GRAY_SCL_RESET;  IIC_Delay();
}

static void IIC_Stop(void)
{
    GRAY_SDA_RESET;  IIC_Delay();
    GRAY_SCL_SET;    IIC_Delay();
    GRAY_SDA_SET;    IIC_Delay();
}

/* 发 1 字节，返回应答：0 = 从机 ACK，1 = 无应答 */
static u8 IIC_WriteByte(u8 dat)
{
    u8 i, ack;

    for (i = 0; i < 8; i++)
    {
        GRAY_SCL_RESET;  IIC_Delay();
        if (dat & 0x80)  GRAY_SDA_SET;
        else             GRAY_SDA_RESET;
        IIC_Delay();
        GRAY_SCL_SET;    IIC_Delay();
        dat <<= 1;
    }

    GRAY_SCL_RESET;  IIC_Delay();
    GRAY_SDA_SET;    IIC_Delay();      /* 释放 SDA，等从机应答 */
    GRAY_SCL_SET;    IIC_Delay();
    ack = GRAY_SDA_READ;               /* 0 = ACK */
    GRAY_SCL_RESET;  IIC_Delay();

    return ack;
}

/* 读 1 字节；ack：0 = 给应答(还要继续读)，1 = 不应答(最后一个字节) */
static u8 IIC_ReadByte(u8 ack)
{
    u8 i, dat = 0;

    GRAY_SDA_SET;    IIC_Delay();      /* 释放 SDA，作为输入 */
    for (i = 0; i < 8; i++)
    {
        GRAY_SCL_RESET;  IIC_Delay();
        GRAY_SCL_SET;    IIC_Delay();
        dat <<= 1;
        if (GRAY_SDA_READ) dat |= 0x01;
    }

    GRAY_SCL_RESET;  IIC_Delay();
    if (ack) GRAY_SDA_SET;  else GRAY_SDA_RESET;
    IIC_Delay();
    GRAY_SCL_SET;    IIC_Delay();
    GRAY_SCL_RESET;  IIC_Delay();
    GRAY_SDA_SET;

    return dat;
}

/* 读单个寄存器：写寄存器地址 → 重复起始 → 读 1 字节 */
static u8 IIC_ReadReg(u8 reg)
{
    u8 dat;

    IIC_Start();
    if (IIC_WriteByte((u8)(GRAY_I2C_ADDR << 1)))        { IIC_Stop(); return 0xFF; }
    if (IIC_WriteByte(reg))                              { IIC_Stop(); return 0xFF; }

    IIC_Start();
    if (IIC_WriteByte((u8)((GRAY_I2C_ADDR << 1) | 1)))   { IIC_Stop(); return 0xFF; }

    dat = IIC_ReadByte(1);      /* 最后 1 字节，回 NACK */
    IIC_Stop();

    return dat;
}

/* 连续读寄存器：从 reg 开始读 len 个字节 */
static u8 IIC_ReadRegs(u8 reg, u8 *buf, u8 len)
{
    u8 i;

    IIC_Start();
    if (IIC_WriteByte((u8)(GRAY_I2C_ADDR << 1)))         { IIC_Stop(); return 0; }
    if (IIC_WriteByte(reg))                               { IIC_Stop(); return 0; }

    IIC_Start();
    if (IIC_WriteByte((u8)((GRAY_I2C_ADDR << 1) | 1)))    { IIC_Stop(); return 0; }

    for (i = 0; i < len; i++)
    {
        buf[i] = IIC_ReadByte((i == (u8)(len - 1)) ? 1 : 0);
    }
    IIC_Stop();

    return 1;
}

/* ---------------- 模块状态 ---------------- */
static u8  gray_online = 0;      /* 1 = 模块在线 */
static u8  gray_bits   = 0;      /* 最近一次读到的 8 位数字量 */
static int gray_error  = 999;    /* 本周期线位置误差；999 = 丢线 */
static int last_error  = 0;      /* 上一次有效误差（丢线时用来决定往哪找） */
static u16 lost_ms     = 0;      /* 已丢线时间 */

/* 探头位置权重：bit0 = 探头1（最右）→ +350 …… bit7 = 探头8（最左）→ -350 */
static const int probe_pos[8] = { 350, 250, 150, 50, -50, -150, -250, -350 };

/* ---------------- 对外调试接口 ---------------- */
u8 Gray_Ping(void)
{
    return (IIC_ReadReg(GW_GRAY_PING) == GW_GRAY_PING_OK) ? 0 : 1;
}

u8 Gray_ReadDigital(void)
{
    return IIC_ReadReg(GW_GRAY_DIGITAL_MODE);
}

u8 Gray_ReadAnalog(u8 *buf8)
{
    return IIC_ReadRegs(GW_GRAY_ANALOG_BASE, buf8, 8);
}

u8  Gray_GetBits(void)    { return gray_bits; }
u8  Gray_IsOnline(void)   { return gray_online; }
int Gray_GetError(void)   { return gray_error; }

/* ---------------- 读一次并算出误差 ---------------- */
static void Gray_Update(void)
{
    u8  i, black_mask;
    u8  cnt = 0;
    int sum = 0;

    gray_bits = Gray_ReadDigital();

#if GRAY_BIT_BLACK_IS_1
    black_mask = gray_bits;                 /* bit=1 表示黑 */
#else
    black_mask = (u8)(~gray_bits);          /* bit=1 表示白 → 取反得黑 */
#endif

    for (i = 0; i < 8; i++)
    {
        if (black_mask & (1 << i))
        {
            sum += probe_pos[i];
            cnt++;
        }
    }

    if (cnt == 0)  gray_error = 999;        /* 一个探头都没压到黑线 → 丢线 */
    else           gray_error = sum / cnt;  /* 黑线重心位置 */
}

/* ---------------- 初始化 ---------------- */
void IRSearchInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GRAY_SCL_PIN | GRAY_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;     /* 开漏，靠模块上拉 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GRAY_SCL_SET;
    GRAY_SDA_SET;
    delay_ms(5);                        /* 等模块上电稳定 */

    gray_online = (Gray_Ping() == 0) ? 1 : 0;
}

/* ---------------- 循迹任务：每 10ms 调用一次 ---------------- */
void SearchRun(void)
{
    int steer, left, right;

    Gray_Update();

    /* ---- 丢线处理 ---- */
    if (gray_error == 999)
    {
        lost_ms += 10;
        if (lost_ms > LINE_LOST_STOP_MS)
        {
            Motor_Stop();                       /* 找太久还没找到 → 停车 */
            return;
        }
        if (last_error >= 0) Motor_SetSpeed( 70, -20);   /* 向右找 */
        else                 Motor_SetSpeed(-20,  70);   /* 向左找 */
        return;
    }

    lost_ms = 0;

    /* ---- PD 控制 ---- */
    steer = (LINE_KP * gray_error) / 100
          + (LINE_KD * (gray_error - last_error)) / 100;

    last_error = gray_error;

    left  = LINE_BASE_SPEED + steer;
    right = LINE_BASE_SPEED - steer;

    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    Motor_SetSpeed(left, right);
}

#else
/* ======================= 方案二：两路红外探头 ======================= */

#define TRACK_SPEED           55
#define TRACK_TURN_OUTER      80
#define TRACK_TURN_INNER       0
#define TRACK_CROSS_STRAIGHT   1

void IRSearchInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = SEARCH_R_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SEARCH_R_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = SEARCH_L_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SEARCH_L_GPIO, &GPIO_InitStructure);
}

void SearchRun(void)
{
    u8 left  = SEARCH_L_IO;
    u8 right = SEARCH_R_IO;

    if (left == WHITE_AREA && right == WHITE_AREA)
        Motor_SetSpeed(TRACK_SPEED, TRACK_SPEED);
    else if (left == BLACK_AREA && right == WHITE_AREA)
        Motor_SetSpeed(TRACK_TURN_INNER, TRACK_TURN_OUTER);
    else if (left == WHITE_AREA && right == BLACK_AREA)
        Motor_SetSpeed(TRACK_TURN_OUTER, TRACK_TURN_INNER);
    else
    {
#if TRACK_CROSS_STRAIGHT
        Motor_SetSpeed(TRACK_SPEED, TRACK_SPEED);
#else
        Motor_Stop();
#endif
    }
}

/* 方案二没有灰度传感器，调试接口给固定值 */
u8  Gray_Ping(void)          { return 1; }
u8  Gray_ReadDigital(void)   { return 0; }
u8  Gray_ReadAnalog(u8 *b)   { (void)b; return 0; }
u8  Gray_GetBits(void)       { return 0; }
u8  Gray_IsOnline(void)      { return 0; }
int Gray_GetError(void)      { return 999; }

#endif
