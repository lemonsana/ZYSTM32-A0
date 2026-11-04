#include "vision.h"
#include "keysacn.h"
#include "motor.h"
#include "delay.h"
#include "stm32f10x.h"

/* ============================================================================
 *  K210 视觉避障  v4.0
 *  ==========================================================================
 *  数据流：
 *      K210（色块/YOLO）→ UART 6 字节帧 → USART2 中断 → 校验 → 本模块
 *      → 避障状态机 → Motor_SetSpeed()
 *
 *  为什么用中断而不是轮询：
 *      K210 可能随时发帧，主循环 10ms 一拍，轮询容易漏；中断里只做
 *      "收字节 + 组帧 + 校验"，很轻（6 字节/帧，20~30fps）。
 *
 *  帧格式：
 *      AA 55 | FLAGS | DIR | DIST | XOR
 *      XOR = AA ^ 55 ^ FLAGS ^ DIR ^ DIST
 * ========================================================================== */

/* ---------------------------- 接收状态 ---------------------------- */
static volatile u8  rx_state   = 0;     /* 0=等AA 1=等55 2=收数据 */
static volatile u8  rx_idx     = 0;
static volatile u8  rx_frame[VISION_FRAME_PAYLOAD + 1];  /* FLAGS DIR DIST XOR */

static volatile u8  vis_flags  = 0;
static volatile u8  vis_dir    = 0;
static volatile u8  vis_dist   = 0;
static volatile u16 vis_frames = 0;
static volatile u8  vis_valid  = 0;

static u16 vis_age_ms = 0xFFFF;         /* 距上一帧的毫秒数 */

/* ---------------------------- 避障状态 ---------------------------- */
#define VIS_ST_IDLE     0
#define VIS_ST_TURN     1
#define VIS_ST_RECOVER  2

static u8  vis_state      = VIS_ST_IDLE;
static u16 vis_timer      = 0;
static u8  vis_turn_right = 0;          /* 1=往右绕 0=往左绕 */

/* ==========================================================================
 *  USART2：PA2=TX(复用推挽) PA3=RX(浮空输入) 115200 8N1
 * ========================================================================== */
void Vision_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2;               /* PA2 TX */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_3;               /* PA3 RX */
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = VISION_UART_BAUD;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    rx_state   = 0;
    rx_idx     = 0;
    vis_valid  = 0;
    vis_frames = 0;
    vis_age_ms = 0xFFFF;
    vis_state  = VIS_ST_IDLE;
    vis_timer  = 0;
}

/* ==========================================================================
 *  USART2 中断：逐字节组帧（中断里不能 printf/delay）
 * ========================================================================== */
void USART2_IRQHandler(void)
{
    u8 b;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == RESET)
    {
        return;
    }

    b = (u8)USART_ReceiveData(USART2);

    switch (rx_state)
    {
        case 0:                                     /* 等帧头 1 */
            if (b == VISION_H1) rx_state = 1;
            break;

        case 1:                                     /* 等帧头 2 */
            if (b == VISION_H2)      { rx_state = 2; rx_idx = 0; }
            else if (b == VISION_H1) { rx_state = 1; }
            else                     { rx_state = 0; }
            break;

        case 2:                                     /* 收 payload + 校验 */
            rx_frame[rx_idx++] = b;
            if (rx_idx > VISION_FRAME_PAYLOAD)
            {
                u8 chk = (u8)(VISION_H1 ^ VISION_H2 ^ rx_frame[0] ^ rx_frame[1] ^ rx_frame[2]);
                if (chk == rx_frame[3])
                {
                    vis_flags = rx_frame[0];
                    vis_dir   = rx_frame[1];
                    vis_dist  = rx_frame[2];
                    vis_frames++;
                    vis_valid  = 1;
                    vis_age_ms = 0;
                }
                rx_state = 0;
                rx_idx   = 0;
            }
            break;

        default:
            rx_state = 0;
            break;
    }
}

/* ==========================================================================
 *  超时维护：主循环每 10ms 调一次
 * ========================================================================== */
void Vision_Task(void)
{
    if (vis_age_ms < 0xFFFF) vis_age_ms += 10;
    if (vis_age_ms > VISION_TIMEOUT_MS) vis_valid = 0;
}

void Vision_Reset(void)
{
    vis_state  = VIS_ST_IDLE;
    vis_timer  = 0;
    vis_age_ms = 0xFFFF;
    vis_valid  = 0;
    rx_state   = 0;
    rx_idx     = 0;
}

/* ==========================================================================
 *  避障任务
 *      IDLE    直行；检测到障碍 → 依方向进入 TURN
 *      TURN    差速绕行，至少保持 VIS_HOLD_MS
 *      RECOVER 反向小幅修正，把车头摆正
 *
 *  方向决策（视觉相对单点传感器的最大价值）：
 *      障碍偏左(dir=0) → 往右绕
 *      障碍偏右(dir=2) → 往左绕
 *      障碍居中(dir=1) → 看距离：近则内轮倒转急转，远则普通弧线
 * ========================================================================== */
void Vision_AVoidRun(void)
{
    u8 detected = 0;
    u8 dir      = 1;

    if (vis_valid && (vis_flags & 0x01))
    {
        detected = 1;
        dir = vis_dir;
    }

    switch (vis_state)
    {
        case VIS_ST_IDLE:
            if (detected)
            {
                if (dir == 0)       vis_turn_right = 1;     /* 偏左 → 往右 */
                else if (dir == 2)  vis_turn_right = 0;     /* 偏右 → 往左 */
                else                vis_turn_right = 1;     /* 居中 → 默认往右 */

                vis_state = VIS_ST_TURN;
                vis_timer = VIS_HOLD_MS;
                BEEP_SET;
                LED_D3_SET;
            }
            else
            {
                BEEP_RESET;
                LED_D3_RESET;
                Motor_SetSpeed(VIS_BASE_SPEED, VIS_BASE_SPEED);
            }
            break;

        case VIS_ST_TURN:
        {
            int inner;

            if ((dir == 1) && vis_valid && (vis_dist > 0) && (vis_dist < VIS_NEAR_CM))
                inner = VIS_INNER_HARD;             /* 居中且很近：内轮倒转 */
            else
                inner = VIS_INNER_SOFT;

            if (vis_turn_right) Motor_SetSpeed(VIS_OUTER_SPEED, inner);
            else                Motor_SetSpeed(inner, VIS_OUTER_SPEED);

            if (vis_timer > 0) vis_timer -= 10;

            if (!detected && (vis_timer == 0))
            {
                vis_state = VIS_ST_RECOVER;
                vis_timer = VIS_RECOVER_MS;
            }
            else if (detected)
            {
                vis_timer = VIS_HOLD_MS;            /* 还在挡路 → 续期 */
            }
            break;
        }

        case VIS_ST_RECOVER:
            if (detected)
            {
                vis_state = VIS_ST_TURN;
                vis_timer = VIS_HOLD_MS;
                BEEP_SET;
                LED_D3_SET;
            }
            else if (vis_timer > 10)
            {
                vis_timer -= 10;
                if (vis_turn_right)
                    Motor_SetSpeed(VIS_BASE_SPEED - VIS_RECOVER_DELTA,
                                   VIS_BASE_SPEED + VIS_RECOVER_DELTA);
                else
                    Motor_SetSpeed(VIS_BASE_SPEED + VIS_RECOVER_DELTA,
                                   VIS_BASE_SPEED - VIS_RECOVER_DELTA);
            }
            else
            {
                vis_state = VIS_ST_IDLE;
                vis_timer = 0;
                BEEP_RESET;
                LED_D3_RESET;
                Motor_SetSpeed(VIS_BASE_SPEED, VIS_BASE_SPEED);
            }
            break;

        default:
            vis_state = VIS_ST_IDLE;
            break;
    }
}

/* ==========================================================================
 *  调试接口
 * ========================================================================== */
u8  Vision_IsValid(void)       { return vis_valid; }
u8  Vision_GetFlags(void)      { return vis_flags; }
u8  Vision_GetDir(void)        { return vis_dir; }
u8  Vision_GetDist(void)       { return vis_dist; }
u8  Vision_GetState(void)      { return vis_state; }
u16 Vision_GetFrameCount(void) { return vis_frames; }
