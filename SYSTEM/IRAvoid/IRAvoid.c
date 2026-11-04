#include "IRAvoid.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "stm32f10x.h"

/* ============================================================================
 *  红外避障模块  v3.4
 *  ==========================================================================
 *
 *  【硬件】
 *      避障探头 PB1（丝印 IO5），输出低电平 = 前方有障碍。
 *      模块上的电位器决定检测距离（一般 10~30cm），本程序不关心具体数值。
 *      注意：只有一个正前方探头，**无法判断左右哪边更空**，所以绕行方向写死
 *            （由 AVOID_TURN_DIR 决定，默认往左）。想自动选边，需要再加一个
 *            右侧探头（可挂到空闲的 IO6 / PA8）。
 *
 *  【v1 原厂做法的问题】
 *      有障碍 → 停车 300ms → 后退 400ms → 左转 500ms
 *      三个动作都是 delay_ms 阻塞，合计 1.2 秒"闭眼"开环跑，途中看不见前方，
 *      而且每次都要停下来退一步，很笨。
 *
 *  【v2 做法的问题】
 *      检测到就进入"外侧 90 / 内侧 15"的弧线，固定保持 400ms。
 *      问题 1：单帧触发没有去抖，反光/噪声会误触发；
 *      问题 2：障碍很近时"内侧 15"转得不够，容易蹭上；
 *      问题 3：障碍刚消失就立刻直行，但车头还歪着，往往马上又触发 → 左右摆。
 *
 *  【v3.4 做法：状态机 + 去抖 + 两级转弯 + 回正补偿】
 *
 *      ┌──────────┐ 连续 N 帧有障碍   ┌──────────────┐
 *      │  IDLE    │ ───────────────→ │  TURN        │
 *      │ 直行 60  │                  │ 一级→二级    │
 *      └──────────┘ ←─────────────── └──────────────┘
 *            ↑      回正完成                │ 障碍消失
 *            │                              ↓
 *            │                      ┌──────────────┐
 *            └──────────────────────│  RECOVER     │
 *                                   │ 反向小幅修正 │
 *                                   └──────────────┘
 *
 *      ① 去抖：连续 AVOID_DEBOUNCE 帧（默认 3 帧 = 30ms）都检测到才算数
 *      ② 两级转弯：
 *           一级（前 250ms）：内侧 25，边走边转，先温和避让
 *           二级（之后）  ：内侧 -35（**反转**），转弯半径骤减，硬绕
 *      ③ 回正：障碍消失后不立刻直行，而是先**反方向**小幅修正 250ms，
 *              把车头摆回来，减少"刚绕开又撞上"的反复
 *      ④ 全程不堵车：每 10ms 只算一次、写一次 PWM，立即返回，不占用 CPU
 *
 *  【调参指南】（都在下面的宏里改，改完重新编译即可）
 *      AVOID_BASE_SPEED   直行速度：跑太快来不及反应就调小（60→45）
 *      AVOID_OUTER_SPEED  转弯外侧速度：想绕得更急就 90→100
 *      AVOID_INNER_SOFT   一级内侧速度：转不够就调小（25→10），转太猛就调大
 *      AVOID_INNER_HARD   二级内侧速度：允许负数（-35 = 内轮倒转，原地搓）
 *      AVOID_STAGE1_MS    一级持续多久升二级：障碍远就调大，障碍近就调小
 *      AVOID_HOLD_MS      二级保持时间：绕不过去就调大（600→900）
 *      AVOID_RECOVER_MS   回正时长：还在左右摆就调大（250→400）
 *      AVOID_RECOVER_DELTA 回正力度：摆回来太快/太慢调这个
 *      AVOID_TURN_DIR     -1 往左绕 / +1 往右绕（改这一个数字即可换边）
 * ========================================================================== */

/* ---------------------------- 可调参数 ---------------------------- */
#define AVOID_BASE_SPEED       60   /* 直行速度 */
#define AVOID_OUTER_SPEED      90   /* 绕行外侧轮速度（快） */
#define AVOID_INNER_SOFT       25   /* 一级绕行内侧轮速度（慢，但仍向前） */
#define AVOID_INNER_HARD      -35   /* 二级绕行内侧轮速度（负数 = 倒转，急转） */

#define AVOID_STAGE1_MS       250   /* 一级转多久后升级为二级 */
#define AVOID_HOLD_MS         600   /* 二级最少保持多久 */
#define AVOID_RECOVER_MS      250   /* 回正（反向修正）持续多久 */
#define AVOID_RECOVER_DELTA    25   /* 回正时内外轮的差速量 */

#define AVOID_DEBOUNCE          3   /* 连续多少帧检测到才算"有障碍"（3×10ms=30ms） */
#define AVOID_TURN_DIR         -1   /* -1 = 往左绕（默认）；+1 = 往右绕 */
#define AVOID_TICK_MS          10   /* 主循环节拍，必须与 main.c 的 TICK_MS 一致 */

/* ---------------------------- 状态机 ---------------------------- */
#define AV_STATE_IDLE     0
#define AV_STATE_TURN     1
#define AV_STATE_RECOVER  2

static u8  av_state   = AV_STATE_IDLE;   /* 当前状态 */
static u8  av_stage   = 1;               /* 转弯级别：1=一级 2=二级 */
static u16 av_timer   = 0;               /* 当前阶段的倒计时(ms) */
static u8  av_blocked = 0;               /* 去抖计数 */

u8 AVoid_GetState(void) { return av_state; }

/* 初始化：避障探头配置为上拉输入 */
void IRAvoidInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = AVOID_PIN;      /* PB1 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(AVOID_PIN_GPIO, &GPIO_InitStructure);

    av_state   = AV_STATE_IDLE;
    av_stage   = 1;
    av_timer   = 0;
    av_blocked = 0;
}

/* 按级别输出转弯动作
 * 一级：内侧轮慢但仍向前（走路带转）
 * 二级：内侧轮倒转（像坦克原地搓），转弯半径最小
 */
static void Avoid_ApplyTurn(u8 stage)
{
    int inner = (stage >= 2) ? AVOID_INNER_HARD : AVOID_INNER_SOFT;

#if (AVOID_TURN_DIR > 0)
    /* 往右绕：左轮（外侧）快、右轮（内侧）慢/倒 */
    Motor_SetSpeed(AVOID_OUTER_SPEED, inner);
#else
    /* 往左绕：左轮（内侧）慢/倒、右轮（外侧）快 */
    Motor_SetSpeed(inner, AVOID_OUTER_SPEED);
#endif
}

/* 回正：反方向小幅修正，把车头摆回来
 * 例如往左绕过障碍后，需要轻微往右带一下
 */
static void Avoid_ApplyRecover(void)
{
#if (AVOID_TURN_DIR > 0)
    /* 之前往右绕 → 现在轻微往左摆 */
    Motor_SetSpeed(AVOID_BASE_SPEED - AVOID_RECOVER_DELTA,
                   AVOID_BASE_SPEED + AVOID_RECOVER_DELTA);
#else
    /* 之前往左绕 → 现在轻微往右摆 */
    Motor_SetSpeed(AVOID_BASE_SPEED + AVOID_RECOVER_DELTA,
                   AVOID_BASE_SPEED - AVOID_RECOVER_DELTA);
#endif
}

/* 避障任务：主循环每 AVOID_TICK_MS 调用一次，立即返回 */
void AVoidRun(void)
{
    u8 detected;

    /* ---------- 1. 采样 + 去抖 ---------- */
    if (AVOID_IO == BARRIER_Y)
    {
        if (av_blocked < 255) av_blocked++;
    }
    else
    {
        av_blocked = 0;
    }
    detected = (av_blocked >= AVOID_DEBOUNCE) ? 1 : 0;

    /* ---------- 2. 状态机 ---------- */
    switch (av_state)
    {
        case AV_STATE_IDLE:
            if (detected)
            {
                av_state = AV_STATE_TURN;
                av_stage = 1;
                av_timer = AVOID_STAGE1_MS;
                BEEP_SET;               /* 声光提示 */
                LED_D3_SET;
            }
            else
            {
                BEEP_RESET;
                LED_D3_RESET;
                Motor_SetSpeed(AVOID_BASE_SPEED, AVOID_BASE_SPEED);   /* 直行 */
            }
            break;

        case AV_STATE_TURN:
            if (detected && av_timer == 0)
            {
                /* 一级转完还被挡 → 升级为二级（内轮倒转，硬绕） */
                av_stage = 2;
                av_timer = AVOID_HOLD_MS;
            }
            else if (!detected && av_timer == 0)
            {
                /* 障碍已消失且转弯时间到 → 进入回正 */
                av_state = AV_STATE_RECOVER;
                av_timer = AVOID_RECOVER_MS;
                break;
            }

            Avoid_ApplyTurn(av_stage);
            if (av_timer > 0) av_timer -= AVOID_TICK_MS;
            break;

        case AV_STATE_RECOVER:
            if (detected)
            {
                /* 回正途中又看到障碍 → 直接回到二级转弯 */
                av_state = AV_STATE_TURN;
                av_stage = 2;
                av_timer = AVOID_HOLD_MS;
                BEEP_SET;
                LED_D3_SET;
                Avoid_ApplyTurn(av_stage);
            }
            else if (av_timer > AVOID_TICK_MS)
            {
                av_timer -= AVOID_TICK_MS;
                Avoid_ApplyRecover();
            }
            else
            {
                av_state = AV_STATE_IDLE;
                av_timer = 0;
                BEEP_RESET;
                LED_D3_RESET;
                Motor_SetSpeed(AVOID_BASE_SPEED, AVOID_BASE_SPEED);
            }
            break;

        default:
            av_state = AV_STATE_IDLE;
            break;
    }
}
