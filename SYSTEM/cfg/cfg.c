#include "cfg.h"
#include "IRSEARCH.h"      /* 参数默认值直接取 GRAY_* 宏，避免两处维护 */
#include "app_mode.h"      /* DEBUG_UART */
#include "stm32f10x.h"
#include "delay.h"

#include <stdio.h>
#include <string.h>

/* 串口没开（DEBUG_UART=0）时，所有提示变成空操作。
 * 否则 printf 会卡在"没使能时钟的 USART1"上死等 TXE，整机起不来。 */
#if DEBUG_UART
#define CFG_PRINTF   printf
#else
static int cfg_printf_null(const char *fmt, ...)
{
    (void)fmt;
    return 0;
}
#define CFG_PRINTF   cfg_printf_null
#endif

/* ============================================================================
 *  运行期参数 + 内部 Flash 保存
 *  ----------------------------------------------------------------------------
 *  存储位置：STM32F103RC 有 256KB Flash，程序远小于 254KB，
 *  取最后 2KB（0x0803F800）单独存参数。若你的芯片是 128KB 版（RBT6/C8T6），
 *  把 CFG_FLASH_ADDR 改成 0x0801F800（128KB 的最后一页）。
 * ========================================================================== */

#define CFG_FLASH_ADDR   0x0803F800UL   /* 256KB 版最后一页（2KB） */
#define CFG_MAGIC        0x5A5A
#define CFG_VER          CFG_VERSION
#define CFG_BLOB_SIZE    (6 + 2 * CFG_COUNT + 2)

typedef struct
{
    u16     magic;
    u16     ver;
    u16     count;
    int16_t v[CFG_COUNT];
    u16     crc;
} cfg_blob_t;

/* 结构体必须偶数长度（按半字写入 Flash），长度不符时这里会编译报错 */
typedef char cfg_blob_size_check[(sizeof(cfg_blob_t) % 2) == 0 ? 1 : -1];

static int16_t g_cfg_v[CFG_COUNT];
static u8      g_cfg_from_flash = 0;

/* ---- 参数名（与 cfg_id_t 顺序一一对应） ---- */
static const char *const g_cfg_name[CFG_COUNT] =
{
    "BASE", "GAIN", "FAR", "THD", "KD", "KI", "SLOW", "SLOWD",
    "MIN", "REV", "CUT", "FULL", "REF", "RAMP", "WIDE", "DEAD"
};

/* ---- 取值范围（防止手滑把参数打成 5000） ---- */
static const int16_t g_cfg_min[CFG_COUNT] =
{
     0,   0,   0,   1,   0,   0,   0,   0,
     0,   0,  10,  10,  10,  10,   1,   0
};
static const int16_t g_cfg_max[CFG_COUNT] =
{
   100, 300, 300, 100, 200,  50, 100, 200,
    95, 100, 255, 100, 100,1000,   9,  20
};

/* ==========================================================================
 *  默认值：直接引 IRSEARCH.h 的宏，保证"改宏"和"恢复默认"是同一套数
 * ========================================================================== */
void Cfg_Defaults(void)
{
    g_cfg_v[CFG_BASE]      = GRAY_BASE_SPEED;
    g_cfg_v[CFG_GAIN]      = GRAY_STEER_GAIN;
    g_cfg_v[CFG_GAIN_FAR]  = GRAY_STEER_GAIN_FAR;
    g_cfg_v[CFG_FAR_THD]   = GRAY_FAR_THRESHOLD;
    g_cfg_v[CFG_KD]        = GRAY_KD;
    g_cfg_v[CFG_KI]        = GRAY_KI;
    g_cfg_v[CFG_SLOW]      = GRAY_SLOW_GAIN;
    g_cfg_v[CFG_SLOW_D]    = GRAY_SLOW_D_GAIN;
    g_cfg_v[CFG_MIN_SPEED] = GRAY_MIN_SPEED;
    g_cfg_v[CFG_INNER_REV] = GRAY_INNER_REVERSE;
    g_cfg_v[CFG_CUTOFF]    = GRAY_BLACK_CUTOFF;
    g_cfg_v[CFG_FULL_ERR]  = GRAY_STEER_FULL_ERROR;
    g_cfg_v[CFG_REF_SPEED] = GRAY_STEER_REF_SPEED;
    g_cfg_v[CFG_RAMP]      = GRAY_RAMP_ALPHA;
    g_cfg_v[CFG_WIDE_MIN]  = GRAY_WIDE_MIN_ACTIVE;
    g_cfg_v[CFG_DEADBAND]  = GRAY_CENTER_DEADBAND;

    g_cfg_from_flash = 0;
}

/* ==========================================================================
 *  CRC16-CCITT（多项式 0x1021，初值 0xFFFF）—— 存 Flash 的完整性校验
 * ========================================================================== */
static u16 cfg_crc16(const u8 *p, u16 n)
{
    u16 crc = 0xFFFF;
    u8  i;

    while (n--)
    {
        crc ^= (u16)((u16)(*p++) << 8);
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x8000) crc = (u16)((u16)(crc << 1) ^ 0x1021);
            else              crc = (u16)(crc << 1);
        }
    }
    return crc;
}

/* ==========================================================================
 *  读 / 写
 * ========================================================================== */
u8 Cfg_Load(void)
{
    const cfg_blob_t *b = (const cfg_blob_t *)CFG_FLASH_ADDR;
    u16 crc;
    u8  i;

    if (b->magic != CFG_MAGIC) return 0;            /* 空白页 = 0xFFFF */
    if (b->ver   != CFG_VER)   return 0;            /* 版本变了 → 旧存档作废 */
    if (b->count != CFG_COUNT) return 0;

    crc = cfg_crc16((const u8 *)b, (u16)(sizeof(cfg_blob_t) - 2));
    if (crc != b->crc) return 0;                    /* 写一半掉电 / 数据坏了 */

    for (i = 0; i < CFG_COUNT; i++) g_cfg_v[i] = b->v[i];

    g_cfg_from_flash = 1;
    return 1;
}

u8 Cfg_Save(void)
{
    cfg_blob_t blob;
    u16       *ph;
    u16        i;
    u8         ok = 1;

    memset(&blob, 0, sizeof(blob));
    blob.magic = CFG_MAGIC;
    blob.ver   = CFG_VER;
    blob.count = CFG_COUNT;
    for (i = 0; i < CFG_COUNT; i++) blob.v[i] = g_cfg_v[i];
    blob.crc   = cfg_crc16((const u8 *)&blob, (u16)(sizeof(blob) - 2));

    /* 擦写期间关中断：Flash 操作会停总线，5ms 节拍和串口都可能丢拍 */
    __disable_irq();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(CFG_FLASH_ADDR) != FLASH_COMPLETE)
    {
        ok = 0;
    }
    else
    {
        ph = (u16 *)&blob;
        for (i = 0; i < (u16)(sizeof(blob) / 2); i++)
        {
            if (FLASH_ProgramHalfWord(CFG_FLASH_ADDR + (u32)i * 2, ph[i]) != FLASH_COMPLETE)
            {
                ok = 0;
                break;
            }
        }
    }

    FLASH_Lock();
    __enable_irq();

    if (!ok) return 0;

    return Cfg_Load();              /* 回读校验：读不回来就算失败 */
}

/* ==========================================================================
 *  取值 / 设值
 * ========================================================================== */
int16_t Cfg_Get(cfg_id_t id)
{
    if ((u8)id >= CFG_COUNT) return 0;
    return g_cfg_v[id];
}

void Cfg_Set(cfg_id_t id, int16_t v)
{
    if ((u8)id >= CFG_COUNT) return;

    if (v < g_cfg_min[id]) v = g_cfg_min[id];
    if (v > g_cfg_max[id]) v = g_cfg_max[id];

    g_cfg_v[id] = v;
}

const char *Cfg_Name(u8 id)
{
    if (id >= CFG_COUNT) return "?";
    return g_cfg_name[id];
}

u8 Cfg_IsFromFlash(void) { return g_cfg_from_flash; }

/* 名字比较（不区分大小写） */
static u8 cfg_name_eq(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = *a;
        char cb = *b;

        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
        if (ca != cb) return 0;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0') ? 1 : 0;
}

u8 Cfg_IdOfName(const char *name)
{
    u8 i;

    for (i = 0; i < CFG_COUNT; i++)
    {
        if (cfg_name_eq(name, g_cfg_name[i])) return i;
    }
    return CFG_COUNT;
}

void Cfg_Print(void)
{
    CFG_PRINTF("[cfg] BASE=%d GAIN=%d FAR=%d THD=%d KD=%d KI=%d\r\n",
           g_cfg_v[CFG_BASE],   g_cfg_v[CFG_GAIN],     g_cfg_v[CFG_GAIN_FAR],
           g_cfg_v[CFG_FAR_THD], g_cfg_v[CFG_KD],      g_cfg_v[CFG_KI]);
    CFG_PRINTF("[cfg] SLOW=%d SLOWD=%d MIN=%d REV=%d CUT=%d FULL=%d REF=%d\r\n",
           g_cfg_v[CFG_SLOW],   g_cfg_v[CFG_SLOW_D],   g_cfg_v[CFG_MIN_SPEED],
           g_cfg_v[CFG_INNER_REV], g_cfg_v[CFG_CUTOFF], g_cfg_v[CFG_FULL_ERR],
           g_cfg_v[CFG_REF_SPEED]);
    CFG_PRINTF("[cfg] RAMP=%d WIDE=%d DEAD=%d  src=%s\r\n",
           g_cfg_v[CFG_RAMP],   g_cfg_v[CFG_WIDE_MIN], g_cfg_v[CFG_DEADBAND],
           g_cfg_from_flash ? "FLASH" : "DEFAULT");
}

/* ==========================================================================
 *  串口命令解析
 * --------------------------------------------------------------------------
 *  只认 ASCII 空格分隔，不用 strtok（省栈、避免全局状态）。
 * ========================================================================== */

static const char *cfg_skip_space(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/* 取一个 token 到 buf（自动大写），返回剩下的位置 */
static const char *cfg_take_token(const char *p, char *buf, u8 size)
{
    u8 n = 0;

    p = cfg_skip_space(p);

    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
    {
        if (n < (u8)(size - 1))
        {
            char c = *p;
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            buf[n++] = c;
        }
        p++;
    }
    buf[n] = '\0';

    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/* 十进制整数（可带负号）；ok=0 表示没有数字 */
static int cfg_atoi_simple(const char *p, u8 *ok)
{
    int v  = 0;
    u8  neg = 0;
    u8  any = 0;

    p = cfg_skip_space(p);

    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') { p++; }

    while (*p >= '0' && *p <= '9')
    {
        v = v * 10 + (*p - '0');
        if (v > 30000) return 0;
        any = 1;
        p++;
    }

    *ok = any;
    return neg ? -v : v;
}

static void cfg_help(void)
{
    CFG_PRINTF("[cfg] ?                  打印参数\r\n");
    CFG_PRINTF("[cfg] SET <NAME> <VAL>  改参数（也可直接 NAME VAL）\r\n");
    CFG_PRINTF("[cfg] SAVE / LOAD / DEF  存盘 / 重读 / 恢复默认\r\n");
    CFG_PRINTF("[cfg] 名字: ");
    {
        u8 i;
        for (i = 0; i < CFG_COUNT; i++) CFG_PRINTF("%s ", g_cfg_name[i]);
    }
    CFG_PRINTF("\r\n");
}

u8 Cfg_HandleLine(const char *line)
{
    char        t1[12];
    char        t2[12];
    const char *p;
    u8          id;
    u8          ok;
    int         val;

    if (line == 0) return 0;

    p = cfg_skip_space(line);
    if (*p == '\0') return 0;

    /* 单字符命令：? */
    if (*p == '?')
    {
        Cfg_Print();
        return 1;
    }

    p = cfg_take_token(p, t1, sizeof(t1));

    if (cfg_name_eq(t1, "HELP"))
    {
        cfg_help();
        return 1;
    }
    if (cfg_name_eq(t1, "SHOW"))
    {
        Cfg_Print();
        return 1;
    }
    if (cfg_name_eq(t1, "SAVE"))
    {
        CFG_PRINTF("[cfg] save ... %s\r\n", Cfg_Save() ? "OK" : "FAIL");
        return 1;
    }
    if (cfg_name_eq(t1, "LOAD"))
    {
        CFG_PRINTF("[cfg] load ... %s\r\n", Cfg_Load() ? "OK" : "EMPTY(用默认值)");
        Cfg_Print();
        return 1;
    }
    if (cfg_name_eq(t1, "DEF"))
    {
        Cfg_Defaults();
        CFG_PRINTF("[cfg] 已恢复默认（还在 RAM，要保存请发 SAVE）\r\n");
        Cfg_Print();
        return 1;
    }

    /* SET NAME VAL */
    if (cfg_name_eq(t1, "SET"))
    {
        p  = cfg_take_token(p, t2, sizeof(t2));
        id = Cfg_IdOfName(t2);
        if (id >= CFG_COUNT)
        {
            CFG_PRINTF("[cfg] 未知参数: %s（发 ? 看列表）\r\n", t2);
            return 1;
        }

        ok  = 0;
        val = cfg_atoi_simple(p, &ok);
        if (!ok)
        {
            CFG_PRINTF("[cfg] 用法: SET %s <整数>\r\n", t2);
            return 1;
        }

        Cfg_Set((cfg_id_t)id, (int16_t)val);
        CFG_PRINTF("[cfg] %s = %d%s\r\n", g_cfg_name[id], g_cfg_v[id],
               (Cfg_IsFromFlash() ? "" : "  (未存盘)"));
        return 1;
    }

    /* 省略 SET：直接 NAME VAL */
    id = Cfg_IdOfName(t1);
    if (id < CFG_COUNT)
    {
        ok  = 0;
        val = cfg_atoi_simple(p, &ok);
        if (!ok)
        {
            CFG_PRINTF("[cfg] 用法: %s <整数>\r\n", t1);
            return 1;
        }

        Cfg_Set((cfg_id_t)id, (int16_t)val);
        CFG_PRINTF("[cfg] %s = %d%s\r\n", g_cfg_name[id], g_cfg_v[id],
               (Cfg_IsFromFlash() ? "" : "  (未存盘)"));
        return 1;
    }

    return 0;       /* 不是调参命令，交给上层（模式字符）处理 */
}

void Cfg_Init(void)
{
    Cfg_Defaults();
    Cfg_Load();
    CFG_PRINTF("[cfg] %s, %d 项\r\n", g_cfg_from_flash ? "从 Flash 读取" : "用默认值", CFG_COUNT);
}
