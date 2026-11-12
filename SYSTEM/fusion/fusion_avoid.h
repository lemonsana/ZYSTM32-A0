#ifndef __FUSION_AVOID_H_
#define __FUSION_AVOID_H_

#include "stm32f10x.h"

/* ============================================================================
 *  融合避障状态机（循迹 + 超声波绕障）  V8
 *  ----------------------------------------------------------------------------
 *  它解决的问题（和视觉版 v1 同源）：
 *      老版本模式 1 是"看到 35cm 就 Motor_Stop() 停车扫左右 → 转向 600ms →
 *      原地反向找线"。四个硬伤：
 *        ① 停车扫描把所有时间预算都花在原地，车一动就要重新测；
 *        ② 35cm 才停，制动距离之外没有减速段，停得急、车头点头；
 *        ③ 转向/回正全是定时，没有"已经绕过障碍"的判据，快慢一变就撞；
 *        ④ 找线靠原地反向转，转不到就 2.5s 超时直接回循迹，线在背后也回。
 *
 *  这一版把运动控制权收进一个 10ms 状态机：
 *      LINE ─40cm─> SLOW ─22cm─> TURN_OUT ─> SHIFT ─> STRAIGHTEN ─> PASS
 *        ^                                                          │
 *        └── COOL <── ALIGN <── SEEK <──────────────────────────────┘
 *
 *  三条关键改动：
 *      1) 减速而不停车：SLOW 段仍跑 PID，只是基础速度降到 60%（Gray_SetSpeedScale），
 *         差速修正照常 —— 车是"慢慢贴过去"而不是"一脚踩停再起步"。
 *      2) 侧移用灰度闭环收尾：SHIFT 不是"转 600ms"，而是"往绕行侧打弧线，
 *         直到灰度连续 3 拍看不到线"——线走了多远，车就侧移了多远。
 *      3) 越障用超声波硬判据：PASS 段要求"前方 ≥45cm 连续 3 次"，而不是计时；
 *         侧向净空在提交前用舵机实测一次，用来选边和判"两侧都堵"。
 *
 *  所有输出都经过 slew 限幅（每 10ms 最多变 12），不存在 0/85 这种阶跃。
 *  任何一段超时都进 FF_* 故障并停车，不会无限绕。
 * ========================================================================== */

/* ---------------------------- 状态 / 故障 ---------------------------- */
#define FU_WAIT         0u      /* 等第一帧有效测距 */
#define FU_LINE         1u      /* 正常循迹 */
#define FU_SLOW         2u      /* 已发现障碍：减速循迹 */
#define FU_TURN_OUT     3u      /* 向绕行侧转出 */
#define FU_SHIFT        4u      /* 弧线侧移，直到离线 */
#define FU_STRAIGHTEN   5u      /* 车头回正 */
#define FU_PASS         6u      /* 贴着障碍侧面通过（侧向距离闭环） */
#define FU_SEEK         7u      /* 向内弧线找线 */
#define FU_ALIGN        8u      /* 找到线：对正 */
#define FU_COOL         9u      /* 冷靜 + 速度爬升，交回 PID */
#define FU_FAULT        10u     /* 故障停车 */

#define FF_NONE         0u
#define FF_BLOCKED      1u      /* 两侧都堵，无路可走 */
#define FF_TIMEOUT      2u      /* 绕行动作总时长超限 */
#define FF_GRAY         3u      /* 灰度连续读失败 */

/* ---------------------------- 距离门限 ---------------------------- */
#define FU_SLOW_CM          40u  /* 进入减速段 */
#define FU_TRIGGER_CM       22u  /* 提交绕行（必须 > 制动距离，见 .c 注释） */
#define FU_CLEAR_CM         45u  /* 前方超过它就算"已越过障碍" */
#define FU_SIDE_NEAR_CM     25u  /* 侧向小于它 = 这一侧太窄，换边 */
#define FU_PASS_KEEP_CM     20u  /* PASS 段与障碍侧面保持的最小距离 */

/* ---------------------------- 速度 / 斜率 ---------------------------- */
#define FU_SLOW_PCT         60u  /* 减速段基础速度百分比 */
#define FU_SLEW_STEP        12   /* 每 10ms 允许变化的最大 PWM */
#define FU_BASE_SPEED       60
#define FU_TURN_SPEED       60
#define FU_SHIFT_FAST       55   /* 侧移：外侧轮 */
#define FU_SHIFT_SLOW       25   /* 侧移：内侧轮 */
#define FU_SEEK_FAST        60
#define FU_SEEK_SLOW        20
#define FU_ALIGN_SPEED      30
#define FU_HANDOVER_PCT     55u  /* 交回 PID 时的起始速度百分比 */

/* ---------------------------- 时间（只做兜底，不做主判据） ---------------------------- */
#define FU_SLOW_MAX_MS      1500u
#define FU_TURN_OUT_MS      300u
#define FU_SHIFT_MIN_MS     550u
#define FU_SHIFT_MAX_MS     1000u
#define FU_STRAIGHTEN_MS    320u
#define FU_PASS_MS          700u
#define FU_PASS_MAX_MS      2200u
#define FU_ALIGN_MS         600u
#define FU_ALIGN_MAX_CORR   60
#define FU_ALIGN_GAIN       60
#define FU_ALIGN_DEADBAND   25
#define FU_HANDOVER_RAMP_MS 500u
#define FU_COOLDOWN_MS      700u
#define FU_SEEK_BUDGET_MS   3000u
#define FU_REARM_MS         1200u
#define FU_MANEUVER_MAX_MS  9000u

/* ---------------------------- 确认 / 容错 ---------------------------- */
#define FU_CONFIRM_WINDOW   3u   /* 减速判据的滑窗帧数 */
#define FU_CONFIRM_HITS     2u   /* 窗内命中几帧才算看见 */
#define FU_NEAR_CONFIRM     2u   /* 提交前连续几拍在触发距离内 */
#define FU_LINE_CONFIRM     3u   /* 连续几拍压线算"回到线上" */
#define FU_OFF_CONFIRM      3u   /* 连续几拍没线算"已经侧移出去" */
#define FU_CLEAR_CONFIRM    3u   /* 连续几次前方清空算"越过障碍" */
#define FU_GRAY_FAIL_MAX    15u  /* 灰度连续失败多少拍才报故障（150ms） */

/* ---------------------------- 舵机 / 扫描 ---------------------------- */
#define FU_SERVO_CENTER     90u
#define FU_SERVO_LEFT       170u
#define FU_SERVO_RIGHT      10u
#define FU_SCAN_SETTLE_MS   250u /* 舵机到位后等多久再测 */
#define FU_ULTRA_PERIOD_MS  100u /* 扫描周期 */

/* ---------------------------- 方向 ---------------------------- */
#define FU_LEFT_DEFAULT     1u   /* 左右都空/都没数据时默认往哪边绕（1=左） */

/* ---------------------- 转角闭环（可选，默认关） ---------------------- */
/*
 * FU_YAW_TURN = 1：TURN_OUT / STRAIGHTEN 改用 MPU6050 的航向角闭环，
 *   转出角度和回正角度由陀螺积分判定，不再依赖 FU_TURN_OUT_MS / FU_STRAIGHTEN_MS。
 *   陀螺没就绪、或者车在转但角度几乎不动（陀螺卡死 / 角度池没更新）时，
 *   本段自动退回原来的定时逻辑，绕行不会因此卡住。
 *
 * 默认 0：行为与 V8.0 完全一致。先把车架空、手推着转 90°，看串口里
 * `yaw` 的变化方向和 IMU_YAW_SIGN 是否一致，再考虑打开。
 *
 * 不想改这个文件时，也可以在 Keil 的 Options → C/C++ → Define 里加
 *   FU_YAW_TURN=1
 * 效果一样（下面用 #ifndef 保护过）。
 */
#ifndef FU_YAW_TURN
#define FU_YAW_TURN         0
#endif
#define FU_TURN_DEG         55      /* 绕行时车头要转出的角度（度） */
#define FU_YAW_TOL_DEG      6.0f    /* 角度到位死区（度） */
#define FU_YAW_STALL_MS     150u    /* 转这么久角度还不涨 → 判定陀螺没跟上 */
#define FU_YAW_STALL_DEG    8.0f    /* 上面那段时间至少要转过这么多 */
#define FU_TURN_OUT_MAX_MS  800u    /* 有陀螺时的兜底（定时版的 2.6 倍） */
#define FU_STRAIGHTEN_MAX_MS 800u   /* 回正兜底 */

/* ---------------------------- 对外接口 ---------------------------- */
void Fusion_Start(void);            /* 进入模式 1：清状态、复位限速 */
void Fusion_Update(void);           /* 每 10ms 调一次 */
void Fusion_Stop(void);             /* 离开模式 1：停车、限速复位、舵机回中 */

u8   Fusion_GetState(void);
u8   Fusion_GetFault(void);
u8   Fusion_GetDistance(void);      /* 最近一次前方测距（0=无效/远） */
u8   Fusion_GetSide(void);          /* 0=向右绕，1=向左绕 */
u8   Fusion_IsArmed(void);          /* 是否允许再次触发 */

#endif
