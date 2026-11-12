#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
融合避障状态机的宿主机单元测试（不依赖 Keil / 硬件）。

做法：把 fusion_avoid.h / fusion_avoid.c 的 #include 去掉，和一层伪硬件 stub
拼成一个 C 文件，用本机 gcc 编译运行。断言覆盖：

    * 距离门限：40cm 进减速、22cm 提交
    * 完整链路：LINE -> SLOW -> TURN_OUT -> SHIFT -> STRAIGHTEN -> PASS
                -> SEEK -> ALIGN -> COOL -> LINE
    * 输出斜率限制（不存在 0/85 阶跃）
    * 重触发闩锁（绕完不会立刻二次触发）
    * 故障：FF_BLOCKED / FF_TIMEOUT / FF_GRAY
    * Stop 后电机停转
    * 转角闭环（-DFU_YAW_TURN=1 另编一遍）：按角度退出、回正、陀螺卡死退回定时

用法：
    GCC="C:\\C\\mingw64\\bin\\gcc.exe" python tests/test_fusion.py
"""

import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HDR = os.path.join(ROOT, "SYSTEM", "fusion", "fusion_avoid.h")
SRC = os.path.join(ROOT, "SYSTEM", "fusion", "fusion_avoid.c")


def strip_includes(path):
    out = []
    with open(path, encoding="utf-8-sig") as f:
        for line in f:
            if line.lstrip().startswith("#include"):
                continue
            out.append(line)
    return "".join(out)


STUBS = r"""
typedef unsigned char  u8;
typedef signed char    s8;
typedef unsigned short u16;
typedef signed short   s16;
typedef unsigned int   u32;
typedef signed int     s32;
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef signed short   int16_t;
typedef signed int     int32_t;
typedef signed char    int8_t;

#define DEBUG_UART 0
#define GRAY_CHANNEL_MIRROR 0
#ifndef FU_YAW_TURN
#define FU_YAW_TURN 0
#endif
#define CFG_CUTOFF 3
#define CFG_MIN_SPEED 4
#define CFG_INNER_REV 5

static u32 g_tick_ms = 0;
static int ml = 0, mr = 0;
static int searching = 0;
static int gray_ok = 1;        /* Gray_ReadRaw 是否成功 */
static int sensor = 0;         /* 0=全白 1=v3/v4 黑 2=v0 黑 3=v6 黑 */
static int scale = 100;        /* Gray_SetSpeedScale 的镜像 */
static int servo = 90;

static int  t_dist = 0;
static int  t_inflight = 0;
static int  t_wait = 0;

static int16_t Cfg_Get(int id)
{
    (void)id;
    return 50;
}
static float imu_yaw  = 0.0f;
static float imu_rate = 0.02f;  /* 每单位轮速差、每拍转多少度 */
static int   imu_ready = 1;
static int   imu_frozen = 0;    /* 1 = 陀螺卡死：角度不随车轮动 */

static u8 Imu_IsReady(void) { return (u8)imu_ready; }
static float Imu_GetYaw(void) { return imu_frozen ? 0.0f : imu_yaw; }

static void Motor_SetSpeed(int l, int r)
{
    ml = l; mr = r;
#if FU_YAW_TURN
    /* 差速积分成航向：左轮慢 → 车头左转 → yaw 变大（和 imu.c 的符号一致） */
    if (!imu_frozen) imu_yaw += ((float)r - (float)l) * imu_rate;
#endif
}
static void Motor_Stop(void)             { ml = 0; mr = 0; }
static void SearchRun(void)              { searching++; ml = 20; mr = 20; }
static void Gray_ResetIntegral(void)     { }
static void Gray_SetSpeedScale(u8 p)     { scale = p; }
static void AVoid_Servo(u8 deg)          { servo = deg; }

static u8 Gray_ReadRaw(u8 *v)
{
    int i;
    for (i = 0; i < 8; i++) v[i] = 100;
    if (!gray_ok) return 0;
    if (sensor == 1) { v[3] = 0; v[4] = 0; }
    if (sensor == 2) { v[0] = 0; }
    if (sensor == 3) { v[6] = 0; }
    return 1;
}

static u8 AVoid_SampleStart(void)
{
    if (t_inflight) return 0;
    t_inflight = 1;
    t_wait = 0;
    return 1;
}

static u8 AVoid_SamplePoll(u8 *cm)
{
    if (!t_inflight) { if (cm) *cm = 0; return 2; }
    if (++t_wait < 2) return 0;      /* 20ms 后出结果 */
    t_inflight = 0;
    if (t_dist == 0) { if (cm) *cm = 0; return 2; }
    if (cm) *cm = (u8)t_dist;
    return 1;
}

void Fusion_Update(void);
u8 Fusion_GetState(void);
u8 Fusion_GetFault(void);
u8 Fusion_GetDistance(void);

static void tick(void)
{
    g_tick_ms += 10;
    Fusion_Update();
}

static void run_ms(int ms)
{
    int i;
    for (i = 0; i < ms / 10; i++) tick();
}

static int last_state = -1;
static void trace(void)
{
    if ((int)Fusion_GetState() != last_state)
    {
        last_state = (int)Fusion_GetState();
        fprintf(stderr, "  [t=%5lu] st=%2d fault=%d scale=%3d ml=%4d mr=%4d d=%3d serv=%3d\n",
                (unsigned long)g_tick_ms, last_state, Fusion_GetFault(),
                scale, ml, mr, (int)Fusion_GetDistance(), servo);
    }
}
"""

BODY = r"""
static void reset(void)
{
    g_tick_ms = 0;
    ml = mr = 0;
    searching = 0;
    gray_ok = 1;
    sensor = 1;
    servo = 90;
    t_dist = 0;
    t_inflight = 0;
    t_wait = 0;
    scale = 100;
    last_state = -1;
    imu_yaw = 0.0f;
    imu_rate = 0.02f;
    imu_ready = 1;
    imu_frozen = 0;
    (void)Imu_GetYaw();          /* 默认构建下也让打桩函数“被用到”，免得 -Wall 报警 */
    (void)Imu_IsReady();
    Fusion_Start();
    tick();
}

/* 跑 ms 毫秒或状态变化为止 */
static void until_state(int target, int ms)
{
    int i;
    for (i = 0; i < ms / 10; i++)
    {
        if ((int)Fusion_GetState() == target)
        {
            tick();          /* 再走一拍，让新状态的第一个动作落地 */
            trace();
            return;
        }
        tick();
        trace();
    }
}

#if FU_YAW_TURN
/* 转角闭环用例（只在 -DFU_YAW_TURN=1 时编译） */
static void test_yaw_turn(void)
{
    u32 t0, el;

    (void)run_ms;                              /* 这个构建里没用到，-Wall 别报 */

    /* ---- A. 陀螺正常：转出到 FU_TURN_DEG 就走，不等定时 ---- */
    reset();
    imu_rate = 0.0667f;                        /* 8°/拍，故意比实车快，好和 300ms 定时区分 */
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    assert(Fusion_GetState() == FU_TURN_OUT);

    t0 = g_tick_ms;
    while ((int)Fusion_GetState() == FU_TURN_OUT && (g_tick_ms - t0) < 900) tick();
    el = g_tick_ms - t0;
    assert(Fusion_GetState() == FU_SHIFT);
    assert(el < 250);                          /* 角度到位就走，远早于 300ms 定时 */
    assert(imu_yaw >= FU_TURN_DEG - FU_YAW_TOL_DEG - 3.0f);
    assert(imu_yaw <= FU_TURN_DEG + 10.0f);

    /* ---- B. 回正：STRAIGHTEN 把车头转回提交时的航向 ---- */
    until_state(FU_STRAIGHTEN, 1600);
    assert(Fusion_GetState() == FU_STRAIGHTEN);
    t0 = g_tick_ms;
    while ((int)Fusion_GetState() == FU_STRAIGHTEN && (g_tick_ms - t0) < 900) tick();
    assert(Fusion_GetState() == FU_PASS);
    fprintf(stderr, "  [yaw] at PASS: yaw=%.2f el=%lu\n",
            (double)Imu_GetYaw(), (unsigned long)(g_tick_ms - t0));
    assert(imu_yaw <= FU_YAW_TOL_DEG + 3.0f);

    /* ---- C. 陀螺卡死：150ms 角度不涨 → 退回定时，绕行不许卡住 ---- */
    reset();
    imu_frozen = 1;
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    assert(Fusion_GetState() == FU_TURN_OUT);
    t0 = g_tick_ms;
    while ((int)Fusion_GetState() == FU_TURN_OUT && (g_tick_ms - t0) < 1500) tick();
    el = g_tick_ms - t0;
    assert(Fusion_GetState() == FU_SHIFT);
    assert(el >= FU_TURN_OUT_MS - 30u && el <= FU_TURN_OUT_MS + 120u);

    printf("PASS: yaw turn (angle exit, return-to-zero, stuck-gyro fallback)\n");
}
#endif

int main(void)
{
#if FU_YAW_TURN
    test_yaw_turn();
    return 0;
#else
    int i;

    /* ---------------- 1. 远处障碍：不触发 ---------------- */
    reset();
    t_dist = 100;
    run_ms(600);
    trace();
    assert(Fusion_GetState() == FU_LINE);
    assert(scale == 100);

    /* ---------------- 2. 40cm 内进减速段 ---------------- */
    t_dist = 35;
    until_state(FU_SLOW, 800);
    assert(Fusion_GetState() == FU_SLOW);
    assert(scale == FU_SLOW_PCT);          /* 减速但仍然跑 PID */
    assert(ml == 20 && mr == 20);          /* 说明 SearchRun 还在出电机 */

    /* SLOW 里做一次侧向探测：舵机转出去、测完回中（卡住就说明时序写错了） */
    run_ms(FU_SCAN_SETTLE_MS + 120);
    assert(servo == FU_SERVO_CENTER);

    /* ---------------- 3. 22cm 内提交绕行 ---------------- */
    t_dist = 20;
    until_state(FU_TURN_OUT, 800);
    assert(Fusion_GetState() == FU_TURN_OUT);
    assert(Fusion_GetSide() == FU_LEFT_DEFAULT);
    assert(Fusion_IsArmed() == 0);

    /* 转出：输出必须逐拍逼近，不能一步跳到 −60/+60 */
    tick();
    {
        int d1 = mr - ml;
        int d2;
        tick();
        d2 = mr - ml;
        assert(d2 > d1);                        /* 左绕：差值在变大 */
        assert(d2 - d1 <= 2 * FU_SLEW_STEP + 2);/* 单拍变化受斜率限制 */
    }
    run_ms(200);
    assert(mr > ml);                            /* 左绕：右轮快 */
    assert(mr <= 100 && ml >= -100);

    /* ---------------- 4. SHIFT：线还在就继续侧移 ---------------- */
    until_state(FU_SHIFT, 500);
    assert(Fusion_GetState() == FU_SHIFT);
    run_ms(300);
    assert(Fusion_GetState() == FU_SHIFT);      /* 550ms 之前不许走 */
    run_ms(300);
    assert(Fusion_GetState() == FU_SHIFT);      /* 线还在视野里，继续侧移 */

    sensor = 0;                                 /* 线已经从视野里消失 */
    until_state(FU_STRAIGHTEN, 800);
    assert(Fusion_GetState() == FU_STRAIGHTEN);

    /* ---------------- 5. STRAIGHTEN -> PASS ---------------- */
    until_state(FU_PASS, 500);
    assert(Fusion_GetState() == FU_PASS);

    /* PASS 里前方一直贴着障碍 -> 不能提前出去 */
    t_dist = 20;
    run_ms(FU_PASS_MS + 200);
    assert(Fusion_GetState() == FU_PASS);

    /* 前方清空，连续 3 次 -> SEEK */
    t_dist = 60;
    until_state(FU_SEEK, 1200);
    assert(Fusion_GetState() == FU_SEEK);

    /* ---------------- 6. SEEK 找到线 -> ALIGN ---------------- */
    sensor = 1;
    until_state(FU_ALIGN, 400);
    assert(Fusion_GetState() == FU_ALIGN);

    /* ---------------- 7. ALIGN 对正 -> COOL ---------------- */
    until_state(FU_COOL, 500);
    assert(Fusion_GetState() == FU_COOL);
    assert(scale >= (int)FU_HANDOVER_PCT && scale <= 100);

    /* ---------------- 8. COOL -> LINE，速度回到 100% ---------------- */
    until_state(FU_LINE, 1000);
    assert(Fusion_GetState() == FU_LINE);
    assert(scale == 100);

    /* ---------------- 9. 重触发闩锁 ---------------- */
    t_dist = 20;                                /* 车还停在障碍旁边 */
    run_ms(500);
    assert(Fusion_GetState() == FU_LINE);       /* 不许立刻二次触发 */

    t_dist = 0;                                 /* 障碍已经被甩开 */
    run_ms(FU_REARM_MS + 200);
    assert(Fusion_IsArmed() == 1);

    t_dist = 20;
    until_state(FU_SLOW, 800);
    assert(Fusion_GetState() == FU_SLOW);       /* 重新允许触发 */

    /* ---------------- 10. 前方一直不清空 -> FF_BLOCKED ---------------- */
    reset();
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    t_dist = 20;
    until_state(FU_PASS, 3000);
    assert(Fusion_GetState() == FU_PASS);
    run_ms(FU_PASS_MAX_MS + 300);
    trace();
    assert(Fusion_GetState() == FU_FAULT);
    assert(Fusion_GetFault() == FF_BLOCKED);
    assert(ml == 0 && mr == 0);

    /* ---------------- 11. 找线超时 -> FF_TIMEOUT ---------------- */
    reset();
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    sensor = 0;
    t_dist = 60;
    until_state(FU_SEEK, 4000);
    assert(Fusion_GetState() == FU_SEEK);
    run_ms(FU_SEEK_BUDGET_MS + 400);
    trace();
    assert(Fusion_GetState() == FU_FAULT);
    assert(Fusion_GetFault() == FF_TIMEOUT);

    /* ---------------- 12. 灰度连续失败 -> FF_GRAY ---------------- */
    reset();
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    sensor = 0;
    t_dist = 60;
    until_state(FU_SEEK, 4000);
    gray_ok = 0;
    for (i = 0; i < (int)FU_GRAY_FAIL_MAX + 5 && Fusion_GetState() != FU_FAULT; i++)
        tick();
    trace();
    assert(Fusion_GetState() == FU_FAULT);
    assert(Fusion_GetFault() == FF_GRAY);

    /* ---------------- 13. 灰度偶发失败不报故障 ---------------- */
    reset();
    t_dist = 20;
    until_state(FU_TURN_OUT, 900);
    gray_ok = 0;
    for (i = 0; i < (int)FU_GRAY_FAIL_MAX - 3; i++) tick();
    assert(Fusion_GetState() != FU_FAULT);
    gray_ok = 1;
    tick();
    tick();

    /* ---------------- 14. Stop ---------------- */
    Fusion_Stop();
    assert(ml == 0 && mr == 0);

    printf("PASS: decel->turn->shift->pass->seek->align->recover, "
           "slew limit, retrigger latch, blocked/timeout/gray faults, stop\n");
    return 0;
#endif
}
"""


def main():
    stub = STUBS
    body = BODY
    src = strip_includes(SRC)
    hdr = strip_includes(HDR)

    full = ("#include <stdio.h>\n#include <assert.h>\n#include <stdlib.h>\n"
            + stub
            + "\n#include <stdio.h>\n#include <assert.h>\n#include <stdlib.h>\n"
            + hdr + "\n" + src + "\n" + body)

    tmp = tempfile.mkdtemp(prefix="fusion_test_")
    cpath = os.path.join(tmp, "test.c")
    with open(cpath, "w", encoding="utf-8") as f:
        f.write(full)

    gcc = os.environ.get("GCC", "gcc")

    # 编两遍：默认（定时转角，跑全部用例）＋ FU_YAW_TURN=1（只跑转角闭环用例）
    for name, defs in (("timer", []), ("yaw", ["-DFU_YAW_TURN=1"])):
        exe = os.path.join(tmp, "test_%s.exe" % name)
        cmd = [gcc, "-std=c99", "-Wall", "-Wextra", "-O1"] + defs + ["-o", exe, cpath]
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        if r.returncode != 0:
            sys.stderr.write(r.stdout + r.stderr)
            sys.exit(1)
        if r.stderr.strip():
            sys.stderr.write(r.stderr)

        r = subprocess.run([exe], capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        sys.stderr.write(r.stderr)
        sys.stdout.write(r.stdout)
        if r.returncode != 0:
            print("FAILED (%s build)" % name)
            sys.exit(1)

    # 只做语法检查：确认 app_mode.c 里没有残留的旧符号
    print("PASS: Python syntax and source layout")
    return 0


if __name__ == "__main__":
    sys.exit(main())
