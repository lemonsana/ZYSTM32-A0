#ifndef __CFG_H_
#define __CFG_H_

#include "stm32f10x.h"

/* ============================================================================
 *  运行期参数 + 内部 Flash 保存  V6
 *  ----------------------------------------------------------------------------
 *  为什么要有这一层：
 *      V5 之前所有循迹参数都是 IRSEARCH.h 里的 #define，改一个数要重新编译、
 *      重新烧录，还要把车推回起跑线。野外调车的时间一大半耗在这上面。
 *      这里把最常动的 16 个参数搬进 RAM，串口一条命令就能改，SAVE 之后掉电
 *      不丢（存 STM32F103RC 内部 Flash 最后一页，2KB）。
 *
 *  Flash 里的存储格式（40 字节，全部 16 位对齐，无结构体填充问题）：
 *      magic(2) | ver(2) | count(2) | v[N](2N) | crc16(2)
 *      magic = 0x5A5A，ver = 1，crc16 覆盖除 crc 外的所有字节。
 *      读的时候 magic/ver/count/CRC 任一不对 → 用默认值（不会把坏数据当参数）。
 *
 *  串口命令（115200，一行一条，大小写不敏感，行尾 \r\n 或 \n）：
 *      ?                打印全部参数
 *      SET BASE 55      改一个参数（也可以直接写 "BASE 55"）
 *      SAVE             写入 Flash（回读校验后返回 OK / FAIL）
 *      LOAD             从 Flash 重新读
 *      DEF              恢复默认值（只改 RAM，要存请再 SAVE）
 *      HELP             命令帮助
 *
 *  参数名 → 含义对照见 cfg.c 的 g_cfg_name[] 与 docs/调参手册.md。
 * ========================================================================== */

/* 参数表：顺序就是 Flash 中的存储顺序。
 * 只能在末尾追加（追加后把 CFG_VERSION +1，旧的存档会因为 ver 不符自动作废）。 */
typedef enum
{
    CFG_BASE = 0,       /* BASE   直行基础速度            */
    CFG_GAIN,           /* GAIN   P：小误差增益            */
    CFG_GAIN_FAR,       /* FAR    P：大误差(急弯)增益      */
    CFG_FAR_THD,        /* THD    增益切换阈值             */
    CFG_KD,             /* KD     微分系数                 */
    CFG_KI,             /* KI     积分系数                 */
    CFG_SLOW,           /* SLOW   速度调度：按 |误差| 降速  */
    CFG_SLOW_D,         /* SLOWD  速度调度：按 |Δ误差| 降速 */
    CFG_MIN_SPEED,      /* MIN    速度调度下限             */
    CFG_INNER_REV,      /* REV    内轮允许反转幅度         */
    CFG_CUTOFF,         /* CUT    黑线判定阈值(归一化值)   */
    CFG_FULL_ERR,       /* FULL   满舵误差                 */
    CFG_REF_SPEED,      /* REF    速度补偿基准速度         */
    CFG_RAMP,           /* RAMP   加速斜坡系数/1000        */
    CFG_WIDE_MIN,       /* WIDE   几路压线算"宽黑"         */
    CFG_DEADBAND,       /* DEAD   中线死区                 */
    CFG_COUNT
} cfg_id_t;

#define CFG_VERSION     1

void        Cfg_Init(void);                    /* 上电调用一次：默认值 + 读 Flash */
int16_t     Cfg_Get(cfg_id_t id);              /* 取参数（越界返回 0） */
void        Cfg_Set(cfg_id_t id, int16_t v);   /* 设参数（内部按范围限幅） */
const char *Cfg_Name(u8 id);                   /* 参数名，打印用 */
u8          Cfg_IdOfName(const char *name);    /* 名字 → 编号，找不到返回 CFG_COUNT */

void        Cfg_Print(void);                   /* 打印全部参数 */
u8          Cfg_Save(void);                    /* 写 Flash：1=成功 */
u8          Cfg_Load(void);                    /* 读 Flash：1=有效 */
void        Cfg_Defaults(void);                /* 恢复默认值（只改 RAM） */
u8          Cfg_IsFromFlash(void);             /* 当前参数是否来自 Flash */

/* 串口一行命令；返回 1 = 本行已被本模块处理，0 = 不是我的命令 */
u8          Cfg_HandleLine(const char *line);

#endif
