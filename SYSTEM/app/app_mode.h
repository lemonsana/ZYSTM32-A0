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
 *  状态机、距离门限、速度、超时等全部搬到 SYSTEM/fusion/fusion_avoid.h，
 *  这里只留一句调用：mode1_update() -> Fusion_Update()。
 *  调这部分参数请直接改 fusion_avoid.h 里的 FU_* 宏。
 * ========================================================================== */

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
