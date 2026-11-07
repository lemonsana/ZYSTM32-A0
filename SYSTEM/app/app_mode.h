#ifndef __APP_MODE_H_
#define __APP_MODE_H_

#include "stm32f10x.h"

/* ============================================================================
 *  业务模式表
 *  ----------------------------------------------------------------------------
 *  把"当前跑哪个模式"从 switch 里挪到一张表：
 *      加模式 = 表里加一行（menu_char / name / start / update / stop）
 *  按键轮换、串口字符切换、调试打印都自动跟着表走，不用再改三处。
 * ========================================================================== */

typedef enum
{
    APP_TASK_RUNNING = 0,
    APP_TASK_DONE
} app_task_result_t;

typedef struct
{
    char        menu_char;              /* 串口字符 / 调试显示 */
    const char *name;                   /* 模式名 */
    void        (*start)(void);         /* 进入模式：清状态、点灯（必须极短） */
    app_task_result_t (*update)(void);  /* 每 MODE_TASK_MS 调一次 */
    void        (*stop)(void);          /* 离开模式 */
} app_mode_desc_t;

#define APP_MODE_NONE       0xFF        /* 当前没有活动模式 */
#define MODE_TASK_MS        10          /* 模式任务周期，必须与 main.c 的注册值一致 */

/* ---------------------------- 系统开关 ---------------------------- */
#define STARTUP_MS         500          /* 上电屏蔽按键/串口的时长，防误触发；0=关闭 */
#define DEBUG_UART            1         /* 1 = 每 200ms 打一行串口调试；0 = 关闭 */

/* ============================================================================
 *  模式1：循迹 + 避障（融合）
 *  ----------------------------------------------------------------------------
 *      循迹直行 ──前方 < FUS_TRIGGER_CM──→ 停车 → 舵机扫 中/左/右 → 选远侧
 *          → 转向绕行 → 原地反向找线 → 看到灰度线 → 冷静 → 回到循迹
 *
 *  调参提示（现象 → 改法）：
 *      FUS_TRIGGER_CM   撞上了才停 → 调大(50)；太早停 → 调小(25)
 *      FUS_TURN_MS      绕完回不到线 → 调大(800)；转过头 → 调小(450)
 *      FUS_SEEK_SPEED   找不到线 → 调大(55)；冲过头 → 调小(30)
 *      FUS_COOLDOWN_MS  刚上线又被触发 → 调大(700)
 *      FUS_SEEK_TIMEOUT_MS 找线最多转多久，超时先回循迹
 * ========================================================================== */
#define FUS_TRIGGER_CM       35     /* 前方小于它触发避让 */
#define FUS_MIN_GAP_CM       20     /* 两侧都小于它 → 原地掉头 */
#define FUS_TURN_MS         600     /* 转向绕行时长 */
#define FUS_SETTLE_MS       250     /* 舵机到位等待 */
#define FUS_SEEK_SPEED       40     /* 找线转速 */
#define FUS_SEEK_TIMEOUT_MS 2500    /* 找线超时（超时就先回到循迹） */
#define FUS_COOLDOWN_MS     400     /* 绕完冷静期，防止立刻又触发 */
#define FUS_MEAS_PERIOD_MS  100     /* 循迹时多久测一次前方 */
#define FUS_SERVO_LEFT      170     /* 扫左的舵机角 */
#define FUS_SERVO_RIGHT      10     /* 扫右的舵机角 */

/* ---------------------------- 接口 ---------------------------- */
void app_mode_init(void);
u8   app_mode_count(void);
u8   app_mode_get(void);
char app_mode_menu_char(u8 idx);
const char *app_mode_name(u8 idx);
int8_t app_mode_index_of_char(char c);

void app_mode_start(u8 idx);
u8   app_mode_start_by_char(char c);
void app_mode_next(void);
void app_mode_stop_active(void);

/* 周期任务入口（由 main.c 注册到 sched） */
void app_mode_key_task(void);       /* 5ms  前台：按键扫描 */
void app_mode_task(void);           /* 10ms 后台：串口指令 + 模式推进 */
void app_mode_debug_task(void);     /* 200ms 后台：调试打印 */

/* 模式1 融合状态机（调试字段 fus：0 循迹 1 扫描 2 转向 3 找线 4 冷静） */
u8   app_mode_fusion_state(void);

/* 红外遥控（R1.1 起未注册成任务；要用就 Sched_RegisterDeferred(IR_IN, 10)） */
void IR_IN(void);

#endif
