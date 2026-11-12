#include "fusion_avoid.h"
#include "sched.h"
#include "motor.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "cfg.h"
#include "app_mode.h"
#include "imu.h"

#include <stdio.h>

/* ============================================================================
 *  融合避障状态机（循序渐进地看这一份代码）
 *  ----------------------------------------------------------------------------
 *  时间轴（10ms 一拍，全部是非阻塞的）：
 *
 *      LINE ──40cm──> SLOW ──22cm──> TURN_OUT ─> SHIFT ─> STRAIGHTEN ─> PASS
 *        ^                                                              │
 *        └──────── COOL <── ALIGN <── SEEK <────────────────────────────┘
 *
 *  每一段的"进入条件"和"退出条件"都写在 case 上面，没有隐式跳转。
 *  出问题只看两处：`[vision]` 每分钟那条（在 app_mode 里）和这里的 [fus] 切换行。
 * ========================================================================== */

/* ---------------------------- 内部状态 ---------------------------- */
static u8  fu_state;
static u8  fu_fault;

static u8  fu_hit_hist;         /* 3 帧滑窗：最近 3 次采样是否在减速距离内 */
static u8  fu_confirmed;        /* 窗内命中数 */
static u8  fu_near_cnt;         /* 连续几拍在触发距离内 */
static u8  fu_off_cnt;          /* 连续几拍看不到线 */
static u8  fu_clear_cnt;        /* 连续几次前方清空 */
static u8  fu_line_cnt;         /* 连续几拍压线 */
static u8  fu_lost_cnt;         /* 连续几拍丢线（ALIGN 用） */
static u8  fu_gray_fail;        /* 灰度连续失败次数 */
static u8  fu_armed;            /* 是否允许触发（防同一障碍反复触发） */
static u8  fu_side;             /* 0=向右绕，1=向左绕 */

static int16_t fu_out_l;        /* 上一拍实际输出的 PWM（斜率限制用） */
static int16_t fu_out_r;

static u32 fu_entered;          /* 进入当前状态的时刻 */
static u32 fu_maneuver_start;   /* 绕行动作起点（总超时用） */
static u32 fu_seek_start;       /* 找线起点（预算用） */
static u32 fu_clear_start;      /* 最近一次"前方有障碍"的时刻（重新武装用） */

/* ---------------------------- 超声波采样 ---------------------------- */
static u8  fu_in_flight;        /* 有一次测距还没取回 */
static u32 fu_last_start;       /* 上一次发起测距的时刻 */
static u32 fu_last_done;        /* 上一次测距完成的时刻 */
static u8  fu_meas_side;        /* 本次采样是"侧向"的 */
static u8  fu_side_meas_valid;  /* 侧向已经测过 */
static u8  fu_side_dist;        /* 侧向距离（0=空/无回波） */
static u8  fu_dist;             /* 前方距离（0=空/无回波） */
static u8  fu_new_sample;       /* 本拍刚拿到一个新样本（状态机据此推进） */
static u8  fu_probe_step;       /* SLOW 里的侧向探测序列 */

static float fu_yaw0;           /* 提交绕行那一刻的航向（FU_YAW_TURN） */
static u8    fu_yaw_on;         /* 本次绕行是否用航向闭环（陀螺没跟上会退回定时） */

/* ---------------------------- 灰度行信息 ---------------------------- */
typedef struct
{
    u8  any;                    /* 有没有任何一路压线 */
    u8  solid;                  /* 压线覆盖够不够宽（跟线用） */
    u8  center;                 /* 中间两路压线（对正判据） */
    int err;                    /* 加权重心误差 −100~+100 */
} fu_line_t;

/* ============================================================================
 *  基础工具
 * ========================================================================== */

static void enter(u8 next)
{
    fu_state      = next;
    fu_entered    = g_tick_ms;
    fu_near_cnt   = 0;
    fu_off_cnt    = 0;
    fu_clear_cnt  = 0;
    fu_line_cnt   = 0;
    fu_lost_cnt   = 0;
}

static void fail(u8 reason)
{
    fu_fault = reason;
    enter(FU_FAULT);
    Gray_SetSpeedScale(100);
    fu_out_l = 0;
    fu_out_r = 0;
    Motor_Stop();
    Gray_ResetIntegral();
#if DEBUG_UART
    printf("[fus] FAULT %u\r\n", fu_fault);
#endif
}

/* 每拍最多变 FU_SLEW_STEP，避免 0/85 这种阶跃把车甩出去 */
static int16_t slew_step(int16_t cur, int16_t tgt)
{
    int16_t d = (int16_t)(tgt - cur);

    if (d >  FU_SLEW_STEP) return (int16_t)(cur + FU_SLEW_STEP);
    if (d < -FU_SLEW_STEP) return (int16_t)(cur - FU_SLEW_STEP);
    return tgt;
}

/* 避障状态的唯一电机出口 */
static void drive(int l, int r)
{
    if (l >  100) l =  100;
    if (l < -100) l = -100;
    if (r >  100) r =  100;
    if (r < -100) r = -100;

    fu_out_l = slew_step(fu_out_l, (int16_t)l);
    fu_out_r = slew_step(fu_out_r, (int16_t)r);
    Motor_SetSpeed(fu_out_l, fu_out_r);
}

/* dir=+1 向绕行侧转，dir=−1 往回收 */
static void turn(int dir, int speed)
{
    int sign = fu_side ? dir : -dir;

    drive(-sign * speed, sign * speed);
}

#if FU_YAW_TURN
/* 相对提交点的转角，"朝绕行侧为正"。
 * 左绕时车头左转、yaw 增大；右绕时车头右转、yaw 减小，所以右绕要取反。
 * Imu_GetYaw() 是累计角（不 wrap），直接相减即可。 */
static float fu_yaw_since(void)
{
    float d = Imu_GetYaw() - fu_yaw0;

    return fu_side ? d : -d;
}
#endif

/* ============================================================================
 *  灰度：一次读 8 路，算覆盖度 + 加权重心
 *  和 IRSEARCH 里的算法保持一致，这样"对正"用的误差和循迹用的误差同尺度。
 * ========================================================================== */
static u8 fu_read_line(fu_line_t *l)
{
    static const int8_t pos_w[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };
    u8 v[8];
    u8 i;
    int16_t cutoff;
    int32_t sum_w;
    int32_t sum_pos;

    l->any    = 0;
    l->solid  = 0;
    l->center = 0;
    l->err    = 0;

    if (!Gray_ReadRaw(v)) return 0;

    cutoff  = Cfg_Get(CFG_CUTOFF);
    sum_w   = 0;
    sum_pos = 0;

    for (i = 0; i < 8u; i++)
    {
        int8_t  pos = pos_w[i];
        int16_t w   = (int16_t)(cutoff - (int16_t)v[i]);

#if GRAY_CHANNEL_MIRROR
        pos = (int8_t)(-pos);
#endif
        if (w > cutoff) w = cutoff;     /* 负值/异常值钳到 0 */
        if (w <= 0) continue;

        l->any = 1;
        sum_w   += w;
        sum_pos += (int32_t)w * pos;
    }

    l->center = (u8)((v[3] < (u8)cutoff) || (v[4] < (u8)cutoff));
    if (sum_w >= 100) l->solid = 1;
    if (sum_w > 0) l->err = (int)((sum_pos * 100) / (sum_w * 7));

    return 1;
}

/* ============================================================================
 *  超声波：非阻塞测距 + 舵机时序
 *  每拍推一次；一次测距 100ms 一次，最多 30ms 超时，超时按"没有障碍"算。
 * ========================================================================== */
static void fu_ultra_tick(void)
{
    u8 cm;
    u8 r;

    if (fu_in_flight)
    {
        r = AVoid_SamplePoll(&cm);
        if (r == 0u) return;

        fu_in_flight = 0;
        fu_last_done = g_tick_ms;

        if (fu_meas_side)
        {
            fu_side_dist       = (r == 1u) ? cm : 0;
            fu_side_meas_valid = 1;
            fu_meas_side       = 0;
        }
        else
        {
            fu_dist       = (r == 1u) ? cm : 0;
            fu_new_sample = 1;
        }
        return;
    }

    if ((g_tick_ms - fu_last_start) < FU_ULTRA_PERIOD_MS) return;

    if (AVoid_SampleStart())
    {
        fu_in_flight = 1;
        fu_last_start = g_tick_ms;
    }
}

/* ============================================================================
 *  对外接口
 * ========================================================================== */
void Fusion_Start(void)
{
    fu_state       = FU_LINE;
    fu_fault       = FF_NONE;
    fu_hit_hist    = 0;
    fu_confirmed   = 0;
    fu_near_cnt    = 0;
    fu_off_cnt     = 0;
    fu_clear_cnt   = 0;
    fu_line_cnt    = 0;
    fu_lost_cnt    = 0;
    fu_gray_fail   = 0;
    fu_armed       = 1;
    fu_side        = FU_LEFT_DEFAULT;
    fu_out_l       = 0;
    fu_out_r       = 0;
    fu_entered     = g_tick_ms;
    fu_maneuver_start = g_tick_ms;
    fu_seek_start  = g_tick_ms;
    fu_clear_start = g_tick_ms;
    fu_in_flight   = 0;
    fu_last_start  = g_tick_ms;
    fu_last_done   = g_tick_ms;
    fu_meas_side   = 0;
    fu_side_meas_valid = 0;
    fu_side_dist   = 0;
    fu_dist        = 0;
    fu_new_sample  = 0;
    fu_probe_step  = 0;
    fu_yaw0        = 0.0f;
    fu_yaw_on      = 0;

    Gray_SetSpeedScale(100);
    AVoid_Servo(FU_SERVO_CENTER);
    Motor_Stop();
    Gray_ResetIntegral();

#if DEBUG_UART
    printf("[fus] start\r\n");
#endif
}

void Fusion_Stop(void)
{
    Gray_SetSpeedScale(100);
    AVoid_Servo(FU_SERVO_CENTER);
    fu_out_l = 0;
    fu_out_r = 0;
    fu_state = FU_WAIT;
    Motor_Stop();
    Gray_ResetIntegral();
}

u8 Fusion_GetState(void)    { return fu_state; }
u8 Fusion_GetFault(void)    { return fu_fault; }
u8 Fusion_GetDistance(void) { return fu_dist; }
u8 Fusion_GetSide(void)     { return fu_side; }
u8 Fusion_IsArmed(void)     { return fu_armed; }

/* ============================================================================
 *  主状态机：每 10ms 调用一次
 * ========================================================================== */
void Fusion_Update(void)
{
    fu_line_t line;
    u8        line_ok;
    int       corr;

    fu_new_sample = 0;
    fu_ultra_tick();

    /* ---- 全局保护 ---- */
    if (fu_state == FU_FAULT)
    {
        Motor_Stop();
        return;
    }

    if (fu_state >= FU_TURN_OUT && fu_state <= FU_COOL)
    {
        if ((g_tick_ms - fu_maneuver_start) >= FU_MANEUVER_MAX_MS)
        {
            fail(FF_TIMEOUT);
            return;
        }
    }

    switch (fu_state)
    {
    /* =====================================================================
     *  FU_LINE：正常循迹，同时盯着前方
     *  进入：上电 / 绕行完成 / 减速段放弃
     *  退出：最近 3 次采样里有 2 次 ≤ FU_SLOW_CM，且已重新武装
     * ===================================================================== */
    case FU_LINE:
        Gray_SetSpeedScale(100);
        SearchRun();

        if (fu_new_sample)
        {
            u8 hit = (u8)((fu_dist && fu_dist <= FU_SLOW_CM) ? 1u : 0u);
            fu_hit_hist = (u8)(((fu_hit_hist << 1) | hit) & 0x07u);

            /* 数窗内命中数 */
            fu_confirmed = (u8)((fu_hit_hist & 1u) + ((fu_hit_hist >> 1) & 1u) +
                                ((fu_hit_hist >> 2) & 1u));
        }

        /* 重触发闩锁：must 连续 FU_REARM_MS 前方都是空的，才恢复武装 */
        if (fu_dist && fu_dist <= FU_TRIGGER_CM)
        {
            fu_clear_start = g_tick_ms;
        }
        else if (!fu_armed && (g_tick_ms - fu_clear_start) >= FU_REARM_MS)
        {
            fu_armed = 1;
        }

        if (fu_armed && fu_confirmed >= FU_CONFIRM_HITS)
        {
            fu_armed       = 0;
            fu_confirmed   = 0;
            fu_hit_hist    = 0;
            fu_near_cnt    = 0;
            fu_side        = FU_LEFT_DEFAULT;
            fu_side_dist   = 0;
            fu_side_meas_valid = 0;
            fu_probe_step  = 0;
#if DEBUG_UART
            printf("[fus] LINE->SLOW d=%u\r\n", fu_dist);
#endif
            enter(FU_SLOW);
        }
        break;

    /* =====================================================================
     *  FU_SLOW：已经看见障碍，但不踩停 —— 基础速度降到 60%，PID 照跑
     *  进入：LINE 确认看见障碍
     *  退出：连续 2 拍 ≤ FU_TRIGGER_CM → 提交绕行；1.5s 还没到 → 回 LINE
     *
     *  这一段同时做一件视觉版做不到的事：把舵机打到默认绕行侧，实测侧向净空。
     *  提交时"默认侧 < 25cm 就换另一边"，比闭着眼睛选边靠谱。
     * ===================================================================== */
    case FU_SLOW:
        Gray_SetSpeedScale(FU_SLOW_PCT);
        SearchRun();

        if (fu_probe_step == 0)
        {
            AVoid_Servo(fu_side ? FU_SERVO_LEFT : FU_SERVO_RIGHT);
            fu_probe_step = 1;
        }
        else if (fu_probe_step == 1)
        {
            if ((g_tick_ms - fu_entered) >= FU_SCAN_SETTLE_MS)
            {
                fu_meas_side  = 1;
                fu_probe_step = 2;
            }
        }
        else if (fu_probe_step == 2)
        {
            if (fu_side_meas_valid)
            {
                AVoid_Servo(FU_SERVO_CENTER);
                fu_probe_step = 3;
            }
        }

        if (fu_new_sample)
        {
            if (fu_dist && fu_dist <= FU_TRIGGER_CM) fu_near_cnt++;
            else                                     fu_near_cnt = 0;

            if (fu_near_cnt >= FU_NEAR_CONFIRM)
            {
                /* 默认那侧太窄就换一边；真两侧都窄，交给 PASS 超时兜底 */
                if (fu_side_meas_valid && fu_side_dist &&
                    fu_side_dist < FU_SIDE_NEAR_CM)
                {
                    fu_side = (u8)(!fu_side);
                }

                AVoid_Servo(FU_SERVO_CENTER);
                Gray_ResetIntegral();
                fu_maneuver_start = g_tick_ms;
#if FU_YAW_TURN
                fu_yaw0   = Imu_GetYaw();
                fu_yaw_on = Imu_IsReady() ? 1u : 0u;
#endif
#if DEBUG_UART
                printf("[fus] SLOW->TURN_OUT d=%u side=%u side_d=%u\r\n",
                       fu_dist, fu_side, fu_side_dist);
#endif
                enter(FU_TURN_OUT);
            }
        }

        if (fu_state == FU_SLOW && (g_tick_ms - fu_entered) >= FU_SLOW_MAX_MS)
        {
            AVoid_Servo(FU_SERVO_CENTER);
            Gray_SetSpeedScale(100);
            enter(FU_LINE);
        }
        break;

    /* =====================================================================
     *  FU_TURN_OUT：向绕行侧转出去
     *  进入：SLOW 连续确认
     *  退出：FU_YAW_TURN=0 → 转满 FU_TURN_OUT_MS
     *        FU_YAW_TURN=1 → 航向转出 FU_TURN_DEG（陀螺没跟上就退回定时）
     * ===================================================================== */
    case FU_TURN_OUT:
        turn(1, FU_TURN_SPEED);
#if FU_YAW_TURN
        if (fu_yaw_on)
        {
            u32   el = g_tick_ms - fu_entered;
            float d  = fu_yaw_since();

            if (el >= FU_YAW_STALL_MS && d < FU_YAW_STALL_DEG)
            {
                fu_yaw_on = 0;              /* 车在转但角度不涨：陀螺卡了 */
            }
            else if (d >= FU_TURN_DEG - FU_YAW_TOL_DEG)
            {
                enter(FU_SHIFT);
            }
            else if (el >= FU_TURN_OUT_MAX_MS)
            {
                enter(FU_SHIFT);
            }
            break;                          /* 有陀螺时不再看 FU_TURN_OUT_MS 定时 */
        }
#endif
        if ((g_tick_ms - fu_entered) >= FU_TURN_OUT_MS)
        {
            enter(FU_SHIFT);
        }
        break;

    /* =====================================================================
     *  FU_SHIFT：弧线侧移，直到"看不见线"
     *  进入：转出完成
     *  退出：线从视野里消失连续 3 拍（说明已经离开原线）
     *        或 1.0s 兜底（线太宽/灰度异常时不至于卡死）
     * ===================================================================== */
    case FU_SHIFT:
        if (fu_side) drive(FU_SHIFT_SLOW, FU_SHIFT_FAST);   /* 左绕：左轮慢 */
        else         drive(FU_SHIFT_FAST, FU_SHIFT_SLOW);

        line_ok = fu_read_line(&line);
        if (!line_ok) fu_gray_fail++;
        else
        {
            fu_gray_fail = 0;
            if (!line.any) fu_off_cnt++;
            else           fu_off_cnt = 0;
        }

        if ((g_tick_ms - fu_entered) >= FU_SHIFT_MIN_MS &&
            (fu_off_cnt >= FU_OFF_CONFIRM ||
             (g_tick_ms - fu_entered) >= FU_SHIFT_MAX_MS))
        {
            enter(FU_STRAIGHTEN);
        }
        break;

    /* =====================================================================
     *  FU_STRAIGHTEN：车头回正
     *  退出：FU_YAW_TURN=0 → 回满 FU_STRAIGHTEN_MS
     *        FU_YAW_TURN=1 → 航向回到提交点（兜底 FU_STRAIGHTEN_MAX_MS）
     * ===================================================================== */
    case FU_STRAIGHTEN:
        turn(-1, FU_TURN_SPEED);
#if FU_YAW_TURN
        if (fu_yaw_on)
        {
            u32   el = g_tick_ms - fu_entered;
            float d  = fu_yaw_since();

            if (d <= FU_YAW_TOL_DEG || el >= FU_STRAIGHTEN_MAX_MS)
            {
                enter(FU_PASS);
            }
            break;                          /* 有陀螺时不看 FU_STRAIGHTEN_MS 定时 */
        }
#endif
        if ((g_tick_ms - fu_entered) >= FU_STRAIGHTEN_MS)
        {
            enter(FU_PASS);
        }
        break;

    /* =====================================================================
     *  FU_PASS：贴着障碍侧面通过
     *  进入：回正完成
     *  退出：前方"清空"（>45cm 或无回波）连续 3 次，且已跑够 700ms
     *        超过 2.2s 还清不了 → 判卡住，停车
     * ===================================================================== */
    case FU_PASS:
        drive(FU_BASE_SPEED, FU_BASE_SPEED);

        if (fu_new_sample)
        {
            if (!fu_dist || fu_dist >= FU_CLEAR_CM) fu_clear_cnt++;
            else                                    fu_clear_cnt = 0;
        }

        if ((g_tick_ms - fu_entered) >= FU_PASS_MS &&
            fu_clear_cnt >= FU_CLEAR_CONFIRM)
        {
            fu_seek_start = g_tick_ms;
            enter(FU_SEEK);
        }
        else if ((g_tick_ms - fu_entered) >= FU_PASS_MAX_MS)
        {
            fail(FF_BLOCKED);
        }
        break;

    /* =====================================================================
     *  FU_SEEK：往线的方向画弧，同时盯灰度
     *  进入：PASS 判定已越过障碍
     *  退出：连续 3 拍压线 → ALIGN；预算 3s 用完 → 故障停车
     * ===================================================================== */
    case FU_SEEK:
        if (fu_side) drive(FU_SEEK_FAST, FU_SEEK_SLOW);
        else         drive(FU_SEEK_SLOW, FU_SEEK_FAST);

        line_ok = fu_read_line(&line);
        if (!line_ok)
        {
            fu_gray_fail++;
        }
        else
        {
            fu_gray_fail = 0;
            if (line.any) fu_line_cnt++;
            else          fu_line_cnt = 0;

            if (fu_line_cnt >= FU_LINE_CONFIRM)
            {
                fu_lost_cnt = 0;
                Gray_ResetIntegral();
                enter(FU_ALIGN);
                break;
            }
        }

        if ((g_tick_ms - fu_seek_start) >= FU_SEEK_BUDGET_MS)
        {
            fail(FF_TIMEOUT);
        }
        break;

    /* =====================================================================
     *  FU_ALIGN：找到线但对得不准 —— 用重心误差做一个小 P 慢慢把车头收正
     *  进入：SEEK 连续确认压线
     *  退出：|err| 在死区内连续 3 拍 → COOL；又丢线 3 拍 → 回 SEEK
     *        0.6s 收不正 → 回 SEEK（同一次预算内重试）
     * ===================================================================== */
    case FU_ALIGN:
        line_ok = fu_read_line(&line);
        if (!line_ok)
        {
            fu_gray_fail++;
        }
        else
        {
            fu_gray_fail = 0;

            if (line.any && line.solid && (line.err > -FU_ALIGN_DEADBAND) &&
                (line.err < FU_ALIGN_DEADBAND))
            {
                if (fu_line_cnt < 255u) fu_line_cnt++;
            }
            else
            {
                fu_line_cnt = 0;
            }

            if (!line.any)
            {
                if (fu_lost_cnt < 255u) fu_lost_cnt++;
            }
            else
            {
                fu_lost_cnt = 0;
            }

            corr = (int)((line.err * FU_ALIGN_GAIN) / 100);
            if (corr >  FU_ALIGN_MAX_CORR) corr =  FU_ALIGN_MAX_CORR;
            if (corr < -FU_ALIGN_MAX_CORR) corr = -FU_ALIGN_MAX_CORR;
            drive(FU_ALIGN_SPEED + corr, FU_ALIGN_SPEED - corr);

            if (fu_line_cnt >= FU_LINE_CONFIRM)
            {
                Gray_ResetIntegral();
                enter(FU_COOL);
                break;
            }

            if (fu_lost_cnt >= FU_OFF_CONFIRM)
            {
                enter(FU_SEEK);
                break;
            }
        }

        if (fu_state == FU_ALIGN && (g_tick_ms - fu_entered) >= FU_ALIGN_MS)
        {
            enter(FU_SEEK);
        }
        break;

    /* =====================================================================
     *  FU_COOL：把控制权交回 PID，速度从 55% 慢慢爬回 100%
     *  进入：对正完成
     *  退出：700ms 后回 LINE（并重置重触发计时，防止停在障碍旁又触发）
     * ===================================================================== */
    case FU_COOL:
    {
        u32 el = g_tick_ms - fu_entered;
        u32 pct;
        u8  p;

        pct = FU_HANDOVER_PCT +
              ((u32)(100u - FU_HANDOVER_PCT) * el) / FU_HANDOVER_RAMP_MS;
        if (pct > 100u) pct = 100u;
        p = (u8)pct;

        Gray_SetSpeedScale(p);
        SearchRun();

        if (el >= FU_COOLDOWN_MS)
        {
            Gray_SetSpeedScale(100);
            fu_clear_start = g_tick_ms;     /* 重新武装计时从这一刻开始 */
            enter(FU_LINE);
        }
        break;
    }

    case FU_WAIT:
    default:
        fu_state = FU_LINE;
        break;
    }

    /* ---- 灰度容错：只在绕行段统计，连续 15 拍（150ms）才认故障 ---- */
    if (fu_gray_fail >= FU_GRAY_FAIL_MAX)
    {
        fail(FF_GRAY);
    }
}
