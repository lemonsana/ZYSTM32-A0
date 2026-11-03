#include "IRSEARCH.h"
#include "delay.h"
#include "motor.h"
#include "stm32f10x.h"

/* ============================================================================
 *  循迹模块  v2.2
 *  --------------------------------------------------------------------------
 *  LINE_USE_GRAY = 1  →  感为 8 路灰度（串行 CLK/DAT：PB10=CLK / PB11=DAT）
 *  LINE_USE_GRAY = 0  →  原来的两路红外探头（PA7=右 / PB0=左）
 * ========================================================================== */

#if LINE_USE_GRAY
/* ======================= 方案一：感为 8 路灰度（串行） =======================
 *
 * 读取时序（照抄厂家 Serial 例程）：
 *     CLK 拉低 → 延时 2us → 读 DAT（bit0 先出）→ CLK 拉高 → 延时 5us
 * 重复 8 次即得 8 位数字量。
 * ========================================================================== */

/* 数字量极性：
 *   0 = bit 为 1 表示"白"（黑线时该位为 0）—— 厂家资料里的约定
 *   1 = bit 为 1 表示"黑"
 * 如果实测车往线的反方向修正，把这个宏改成 1 即可。 */
#define GRAY_BIT_BLACK_IS_1   0

static u8 gray_bits  = 0;       /* 最近一次读到的 8 位数字量 */
static int gray_error = 999;    /* 本周期线位置误差；999 = 丢线 */
static int last_error = 0;      /* 上一次有效误差（丢线时决定往哪找） */
static u16 lost_ms    = 0;      /* 已丢线时间 */

/* 探头位置权重：bit0 = 探头1（最右）→ +350 …… bit7 = 探头8（最左）→ -350 */
static const int probe_pos[8] = { 350, 250, 150, 50, -50, -150, -250, -350 };

/* ---------------- 串行读取 8 位数字量 ---------------- */
u8 Gray_ReadByte(void)
{
    u8 i, dat = 0;

    for (i = 0; i < 8; i++)
    {
        GRAY_CLK_RESET;                 /* 时钟下降沿 */
        delay_us(2);                    /* 等数据稳定 */

        if (GRAY_DAT_READ) dat |= (u8)(1 << i);

        GRAY_CLK_SET;                   /* 上升沿：模块切换下一位 */
        delay_us(5);                    /* 厂家要求约 5us */
    }

    return dat;
}

u8  Gray_GetBits(void)  { return gray_bits; }
int Gray_GetError(void) { return gray_error; }

/* ---------------- 读一次并算出误差 ---------------- */
static void Gray_Update(void)
{
    u8  i, black_mask;
    u8  cnt = 0;
    int sum = 0;

    gray_bits = Gray_ReadByte();

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

    /* CLK：推挽输出（单片机 -> 模块） */
    GPIO_InitStructure.GPIO_Pin   = GRAY_CLK_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GRAY_CLK_GPIO, &GPIO_InitStructure);

    /* DAT：上拉输入（模块 -> 单片机）。模块是 5V 输出，PB11 是 5V 容忍脚 */
    GPIO_InitStructure.GPIO_Pin   = GRAY_DAT_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GRAY_DAT_GPIO, &GPIO_InitStructure);

    GRAY_CLK_SET;               /* 空闲时 CLK 保持高电平 */
    delay_ms(5);                /* 等模块上电稳定 */

    gray_bits = Gray_ReadByte();/* 先读一次，串口能看到 */
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
u8  Gray_ReadByte(void)  { return 0; }
u8  Gray_GetBits(void)   { return 0; }
int Gray_GetError(void)  { return 999; }

#endif
