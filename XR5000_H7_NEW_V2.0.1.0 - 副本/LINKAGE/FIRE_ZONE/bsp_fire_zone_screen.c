/*==============================================================
 * 文件名称   : bsp_fire_zone_screen.c
 * 模块功能   : 防火分区画面(画面84)交互处理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 功能说明   : 防火分区2×2四宫格界面(每页4个分区槽, 2页共8分区):
 *              - 每槽显示: 分区号/分区名/6类设备范围/启用状态
 *              - 翻页(264/265, 两页回绕) / 返回(266, 屏端自跳)
 *              - 分区名输入(id1~4): GBK合法性校验+超长截断
 *              - 设备范围输入: 仅11位"LLAAA.LLAAA"格式(全'0'清空归位),
 *                显示统一"LLAAA-LLAAA"(单台亦为起止相同格式),
 *                九步校验通过后写入RAM, 按返回键时统一落盘
 *              - 启用切换(105~108): 分区1禁止停用, 其余切换并回显
 *              - 回显策略: 设备范围栏只显示屏端输入的规范化文本
 *                ("LLAAA-LLAAA"), 无输入显示空, 不从分区表反推
 *              - 未识别产品码设备: 保存放行(先配置后接线), 提示条告知
 * 刷新策略   : 差分刷新(内容变化才写屏); 数据变更回调置脏后
 *              由周期刷新重刷(设备栏显示与分区表脱钩, 仅随输入变化)。
 * 依赖模块   : bsp_fire_zone(分区表: 名称/启用/划分落盘),
 *              bsp_mbus/bsp_mbus_control/bsp_rs485_detect(产品码识别,未识别统计),
 *              hmi_driver(SetTextValue写屏), bsp_debug(调试打印)
 * 控件依据   : 屏工程"防火分区.tft"实测控件表(2026-09-16):
 *              分区号101~104, 分区名输入1~4, 启用按钮105~108,
 *              状态回显111~114, 翻页264/265, 返回266, 提示条300(屏端待补)
 *==============================================================*/

#include "bsp_fire_zone_screen.h"   /* 自身头文件: 画面/控件定义 */
#include "bsp_fire_zone.h"          /* 分区表数据层API */
#include "bsp_mbus.h"               /* 回路1产品码(未识别统计用) */
#include "bsp_mbus_control.h"       /* 回路2设备在线状态/产品码 */
#include "bsp_rs485_detect.h"       /* 回路3探测器在线状态/产品码 */
#include "bsp_debug.h"              /* DebugPrintf调试打印 */
#include "hmi_driver.h"             /* SetTextValue屏幕写文本接口 */
#include "system.h"                /* SystemSaveInfo: 分区手/自动状态 */
#include "bsp_internal_board.h"    /* g_maintenance_state: 检修开关状态 */

#include <string.h>                 /* strcmp/strncpy/strlen */
#include <stdio.h>                  /* sprintf */

/*--------------------------------------------------------------
 * 1. 控件映射表与常量
 *--------------------------------------------------------------*/

/* 设备范围输入框ID表[类型][槽位](与屏工程防火分区.tft逐一核对) */
static const uint8_t s_fz_dev[FZ_DEV_TYPE_COUNT][FIRE_ZONE_PAGE_ZONES] =
{
    {  5U,   6U },   /* 温度        回路1 */
    {  9U,  10U },   /* 烟雾        回路1 */
    { 14U,  15U },   /* 可燃气体    回路3 */
    { 18U,  19U },   /* 复合探测器  回路3 */
    { 22U,  23U },   /* 手报        回路2 (XR2200) */
    { 26U,  27U },   /* 声光        回路2 (SGBJQ) */
    {  4U,   8U }    /* 火灾显示盘  回路2 (单地址) */
};

/* 各设备类型对应的回路号(与s_fz_dev行序一致) */
static const uint8_t s_fz_loop[FZ_DEV_TYPE_COUNT] =
{
    1U, 1U, 3U, 3U, 2U, 2U, 2U
};

/* 分区名输入框ID(槽0~1) */
static const uint8_t s_fz_name[FIRE_ZONE_PAGE_ZONES] = { 1U, 2U };

/* 系统级设备输入框ID(与FZ_SYS_*顺序一致: 手报/声光/火灾显示盘) */
static const uint8_t s_fz_sys[FZ_SYS_COUNT] =
{
    FIRE_ZONE_IN_SYS_MANUAL, FIRE_ZONE_IN_SYS_SIREN, FIRE_ZONE_IN_SYS_DISPLAY
};

/* 提示条自动清空时间(ms) */
#define FZ_TIP_TIMEOUT  3000U
/*--------------------------------------------------------------
 * 画面95(新防火分区)控件ID - 与屏工程"新防火分区.tft"核对
 *   分区1: 7=手/自动回显 8=进96 9=延时输入 10=进43
 *   分区2: 25=手/自动回显 26=进96 24=延时输入 27=进43
 *   系统级: 15=运行/检修 12=显示盘 16=手报 28=声光
 *--------------------------------------------------------------*/
#define FZ95_TXT_MODE_Z1    7U
#define FZ95_TXT_MODE_Z2    25U
#define FZ95_BTN_DEV_Z1     8U
#define FZ95_BTN_DEV_Z2     26U
#define FZ95_IN_DELAY_Z1    9U
#define FZ95_IN_DELAY_Z2    24U
#define FZ95_BTN_LOGIC_Z1   10U
#define FZ95_BTN_LOGIC_Z2   27U
#define FZ95_TXT_RUN        15U

/* 画面96(设备管理)每行控件与槽位映射: 视觉顺序为
 * 温度/烟雾/可燃气体/复合/声光/手报/显示盘/输入输出模块,
 * 注意"声光"排在"手报"之前, 与画面84的顺序相反 */
static const uint8_t s_dev_row_ctrl[FZ_DEV_TYPE_COUNT] = { 9U, 11U, 13U, 15U, 17U, 19U, 23U, 25U };
static const uint8_t s_dev_row_slot[FZ_DEV_TYPE_COUNT] =
{
    FZ_SLOT_TEMP, FZ_SLOT_SMOKE, FZ_SLOT_GAS, FZ_SLOT_COMPOUND,
    FZ_SLOT_SIREN, FZ_SLOT_MANUAL, FZ_SLOT_DISPLAY, FZ_SLOT_MODULE
};

/* 各槽位对应回路号(下标=FZ_SLOT_*) */
static const uint8_t s_dev_slot_loop[FZ_SLOT_COUNT] =
{
    1U, 1U, 3U, 3U, 2U, 2U, 2U, 2U
};

/* 画面95/96运行状态 */
static uint8_t s_dev_zone = 1U;    /* 画面96当前编辑分区(1/2), 由画面95的8/26设定 */
static uint8_t s_in95 = 0U;        /* 画面95进入标志(进入时全刷) */
static uint8_t s_in96 = 0U;        /* 画面96进入标志(进入时回显) */
static char s_last95_mode[FIRE_ZONE_MAX][8];   /* 手/自动回显缓存 */
static char s_last95_delay[FIRE_ZONE_MAX][4];  /* 延时回显缓存 */
static char s_last95_run[8];                   /* 运行/检修回显缓存 */
static char s_last96[FZ_DEV_TYPE_COUNT][FIRE_ZONE_DISP_LEN]; /* 96各行回显缓存 */

/*--------------------------------------------------------------
 * 2. 内部状态变量
 *--------------------------------------------------------------*/

static uint8_t s_page = 0;     /* 当前页码(0=分区1~4, 1=分区5~8) */
static uint8_t s_in_page = 0;  /* 当前是否处于防火分区画面(进入/离开边沿检测) */
static uint32_t s_tip_tick = 0;/* 提示条最近一次写入时刻(HAL_GetTick) */
static char s_tip[24];         /* 提示条当前文本 */

/* 差分刷新缓存: 每栏显示文本(设备栏=屏端输入的规范化文本, 名称最长15字节)。
 * 设备栏缓存按全部分区组织([分区0-7][类型0-5]): 翻页/重进画面时
 * 直接恢复对应分区内容, 不受当前页影响 */
static char s_last_dev[FIRE_ZONE_MAX][FZ_DEV_TYPE_COUNT][FIRE_ZONE_DISP_LEN];
static char s_last_name[FIRE_ZONE_PAGE_ZONES][FIRE_ZONE_NAME_LEN];
static char s_last_zone[FIRE_ZONE_PAGE_ZONES][4];  /* 分区号文本("1"~"8") */
static char s_last_en[FIRE_ZONE_PAGE_ZONES][8];    /* 启用状态("启用"/"停用") */
static char s_last_sys[FZ_SYS_COUNT][FIRE_ZONE_NAME_LEN]; /* 系统级设备栏文本 */

/* 输入会话标志: 1=本次进入画面以来有修改且batch挂起未落盘。
 * 交互约定: 输入/清空/启用切换只改RAM(batch挂起压住落盘),
 * 按返回键时(或离开画面兜底)才统一EndBatch落盘一次Flash;
 * 翻页不保存(纯UI切换) */
static uint8_t s_edit_dirty = 0;

/*--------------------------------------------------------------
 * 3. 内部工具函数 - 设备识别
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：GetDeviceProductCode
 * 功能说明：按回路号+设备号获取设备识别出的产品码(0x000E)。
 * 参数说明：loop_no - 回路号(1-3), dev_no - 设备地址。
 * 返回值：   产品码(0=未知/未识别)。
 *--------------------------------------------------------------*/
static uint16_t GetDeviceProductCode(uint8_t loop_no, uint16_t dev_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus点型混合探测器识别产品码 */
        return (dev_no <= MIXTURE_DEVICE_MAX_ADDR) ? MBus1_GetProductCode((uint8_t)dev_no) : 0;

    case 2:
        /* 回路2: MBus控制设备识别产品码 */
        return (dev_no < MBUS_CONTROL_MAX_DEVICES) ? MBusCtrl_GetProductCode((uint8_t)dev_no) : 0;

    case 3:
        /* 回路3: RS485探测器识别产品码 */
        return (dev_no < RS485_DETECT_MAX_DEVICES) ? RS485Detect_GetProductCode((uint8_t)dev_no) : 0;

    default:
        return 0;
    }
}

/*--------------------------------------------------------------
 * 4. 内部工具函数 - 提示条
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：BeginEditSession
 * 功能说明：开启输入会话(幂等): 首笔修改时BeginBatch压住落盘。
 *           之后所有修改只改RAM, 直至FlushPendingSave统一落盘。
 *--------------------------------------------------------------*/
static void BeginEditSession(void)
{
    if (s_edit_dirty == 0U)
    {
        FireZone_BeginBatch();  /* 会话首笔: 批量挂起(写函数内部只置脏不擦写) */
    }
    s_edit_dirty = 1U;
}

/*--------------------------------------------------------------
 * 函数名称：FlushPendingSave
 * 功能说明：收尾输入会话: 会话内有修改则EndBatch统一落盘一次Flash
 *           (A/B双副本擦写, 忙等45~400ms), 并通知屏幕层。
 *           幂等, 无会话直接返回。
 *           调用时机: 返回键屏端自跳/被切走时, 由UpdateUI离开分支
 *           统一执行(翻页不保存, 翻页纯UI切换)
 *--------------------------------------------------------------*/
static void FlushPendingSave(void)
{
    if (s_edit_dirty != 0U)
    {
        FireZone_EndBatch();    /* 最外层: s_batch_dirty=1才真正擦写 */
        s_edit_dirty = 0U;
    }
}

/*--------------------------------------------------------------
 * 函数名称：SetTip
 * 功能说明：写入提示条文本并记录时刻(3秒后由UpdateUI自动清空)。
 *           说明: 屏端tft暂缺id=300提示条控件, 写屏指令将被
 *           屏端忽略(无害), 屏端补控件后自动生效。
 * 参数说明：msg - GBK文本(源文件GB2312编码)。
 *--------------------------------------------------------------*/
static void SetTip(const char *msg)
{
    strncpy(s_tip, msg, sizeof(s_tip) - 1U);
    s_tip[sizeof(s_tip) - 1U] = '\0';
    SetTextValue(FIRE_ZONE_SCREEN_ID, FIRE_ZONE_TXT_TIP, (uint8_t *)s_tip);
    s_tip_tick = HAL_GetTick();
    /* 诊断打印: 提示条控件(300)屏端暂缺, 提示内容同步输出到UART4
     * 便于定位输入校验失败原因(待屏端补控件后可移除) */
    DebugPrintf("[FZS] tip: %s\r\n", msg);
}

/*--------------------------------------------------------------
 * 函数名称：TipTimeoutCheck
 * 功能说明：提示条超时检查(超3秒清空一次, 避免误导)。
 *--------------------------------------------------------------*/
static void TipTimeoutCheck(void)
{
    if (s_tip[0] != '\0')
    {
        if ((HAL_GetTick() - s_tip_tick) > FZ_TIP_TIMEOUT)
        {
            s_tip[0] = '\0';
            SetTextValue(FIRE_ZONE_SCREEN_ID, FIRE_ZONE_TXT_TIP, (uint8_t *)"");
        }
    }
}

/*--------------------------------------------------------------
 * 6. 内部工具函数 - 页面刷新
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：SendDiff
 * 功能说明：差分写屏: 内容变化(或强制)时写文本控件并更新缓存。
 * 参数说明：ctrl - 控件ID, buf - 新文本(<=15字节),
 *           last - 该控件的上次文本缓存, force - 1=强制写。
 *--------------------------------------------------------------*/
static void SendDiff(uint16_t ctrl, const char *buf, char *last, uint8_t force)
{
    if (force || (strcmp(buf, last) != 0))
    {
        strncpy(last, buf, FIRE_ZONE_DISP_LEN - 1U);
        last[FIRE_ZONE_DISP_LEN - 1U] = '\0';
        SetTextValue(FIRE_ZONE_SCREEN_ID, ctrl, (uint8_t *)buf);
    }
}

/*--------------------------------------------------------------
 * 函数名称：RestoreDispCache
 * 功能说明：从数据层恢复设备栏显示文本缓存。
 *           显示文本随分区表存Flash(V3记录), 上电后进入画面
 *           时恢复显示, 已配置内容无需重新输入。
 *--------------------------------------------------------------*/
static void RestoreDispCache(void)
{
    uint8_t z;   /* 分区下标(0-1对应分区1-2) */
    uint8_t t;   /* 设备类型 */
    uint8_t k;   /* 系统级设备类型(FZ_SYS_*) */

    for (z = 0U; z < FIRE_ZONE_MAX; z++)
    {
        for (t = 0U; t < FZ_DEV_TYPE_COUNT; t++)
        {
            FireZone_GetSlotDisp((uint8_t)(z + 1U), t, s_last_dev[z][t]);
            if (s_last_dev[z][t][0] != '\0')
            {
                /* 诊断打印: 输出从数据层恢复的非空显示文本 */
                DebugPrintf("[FZS] restore z%u t%u: %s\r\n",
                            (unsigned)(z + 1U), (unsigned)t, s_last_dev[z][t]);
            }
        }
    }

    /* 系统级设备三栏(手报16/声光28/火灾显示盘12) */
    for (k = 0U; k < FZ_SYS_COUNT; k++)
    {
        FireZone_GetSysDisp(k, s_last_sys[k]);
        if (s_last_sys[k][0] != '\0')
        {
            DebugPrintf("[FZS] restore sys%u: %s\r\n",
                        (unsigned)k, s_last_sys[k]);
        }
    }
}

/*--------------------------------------------------------------
 * 函数名称：RefreshPage
 * 功能说明：刷新当前页2个分区槽(分区号/名称/启用状态/7类设备栏)
 *           及画面底部系统级设备区3栏(手报/声光/火灾显示盘)。
 * 参数说明：force - 1=强制全量刷新(进入画面/翻页), 0=差分刷新。
 *--------------------------------------------------------------*/
static void RefreshPage(uint8_t force)
{
    uint8_t slot;         /* 槽位0~1 */
    uint8_t type;         /* 设备类型 */
    uint8_t zone;         /* 槽位对应分区号 */
    uint8_t k;            /* 系统级设备类型(FZ_SYS_*) */
    char buf[FIRE_ZONE_NAME_LEN + 32];  /* 临时构建缓冲(48字节) */

    for (slot = 0; slot < FIRE_ZONE_PAGE_ZONES; slot++)
    {
        zone = (uint8_t)(s_page * FIRE_ZONE_PAGE_ZONES + slot + 1U);

        /* 分区号(必刷项: 屏端默认"1", 进入必须重发) */
        sprintf(buf, "%u", (unsigned)zone);
        SendDiff(FIRE_ZONE_TXT_ZONE_0 + slot, buf, s_last_zone[slot], force);

        /* 分区名(GBK串直接显示) */
        strncpy(buf, FireZone_GetZoneName(zone), FIRE_ZONE_NAME_LEN - 1U);
        buf[FIRE_ZONE_NAME_LEN - 1U] = '\0';
        SendDiff(s_fz_name[slot], buf, s_last_name[slot], force);

        /* 启用状态回显(必刷项: 屏端默认"未启用") */
        strcpy(buf, (FireZone_GetZoneEnabled(zone) != 0U) ? "启用" : "停用");
        SendDiff(FIRE_ZONE_TXT_ENST_0 + slot, buf, s_last_en[slot], force);

        /* 6类设备范围栏: 只显示屏端输入内容(显示文本缓存, 无输入为空),
         * 不从分区表反推; force=1时把缓存重写回屏(重进画面/翻页后恢复) */
        for (type = 0; type < FZ_DEV_TYPE_COUNT; type++)
        {
            if (s_fz_dev[type][slot] == 0U)
            {
                continue;   /* 84画面无此设备栏(第8类输入输出模块) */
            }
            SendDiff(s_fz_dev[type][slot], s_last_dev[zone - 1U][type],
                     s_last_dev[zone - 1U][type], force);
        }
    }

    /* 系统级设备区(画面底部): 手报16 / 声光28 / 火灾显示盘12 */
    for (k = 0U; k < FZ_SYS_COUNT; k++)
    {
        SendDiff(s_fz_sys[k], s_last_sys[k], s_last_sys[k], force);
    }
}

/*--------------------------------------------------------------
 * 7. 输入处理(NotifyText分支)
 *--------------------------------------------------------------*/

#if 0 /* [2026-09-18 按需求注释] 分区名GBK合法性校验整体停用(接收限制约束取消),
       * 名称输入不再校验不再截断, 原样显示与保存 */
/*--------------------------------------------------------------
 * 函数名称：ValidateNameGBK
 * 功能说明：分区名GBK合法性校验+超长安全截断。
 *           - 拒绝控制字符(<0x20)/双引号(0x22)/反斜杠(0x5C)/0x7F
 *           - GBK双字节序列必须成对(拒绝孤立首字节)
 *           - 超过15字节截断, 且不拆散双字节字符
 * 参数说明：src - 输入串, dst - 输出缓冲(容量FIRE_ZONE_NAME_LEN),
 *           truncated - 输出: 1=发生了截断。
 * 返回值：   0=合法, 1=含非法字符。
 *--------------------------------------------------------------
static uint8_t ValidateNameGBK(const char *src, char *dst, uint8_t *truncated)
{
    uint8_t out = 0;      /* 输出长度 */
    uint8_t i = 0;        /* 读取游标 */
    uint8_t b, b2;        /* 当前/下一字节 */

    *truncated = 0U;
    while (src[i] != '\0')
    {
        b = (uint8_t)src[i];
        if ((b < 0x20U) || (b == 0x22U) || (b == 0x5CU) || (b == 0x7FU))
        {
            return 1;  /* 控制字符/引号/反斜杠: 拒绝 */
        }
        if ((b >= 0x81U) && (b <= 0xFEU))
        {
            /* GBK双字节首字节: 校验尾字节 */
            b2 = (uint8_t)src[i + 1U];
            if ((b2 < 0x40U) || (b2 == 0x7FU) || (b2 > 0xFEU))
            {
                return 1;  /* 孤立首字节/非法尾字节: 拒绝 */
            }
            if ((out + 2U) > (FIRE_ZONE_NAME_LEN - 1U))
            {
                *truncated = 1U;  /* 剩余空间不足2字节: 截断(不拆双字节) */
                break;
            }
            dst[out] = (char)b;
            dst[out + 1U] = (char)b2;
            out += 2U;
            i += 2U;
        }
        else
        {
            /* 单字节(ASCII可见字符) */
            if ((out + 1U) > (FIRE_ZONE_NAME_LEN - 1U))
            {
                *truncated = 1U;
                break;
            }
            dst[out] = (char)b;
            out++;
            i++;
        }
    }
    dst[out] = '\0';
    return 0;
}
--------------------------------------------------------------*/
#endif /* GBK校验停用结束 */

/*--------------------------------------------------------------
 * 函数名称：HandleNameInput
 * 功能说明：分区名输入框(id1~4)处理: 校验->保存->回显->提示。
 * 参数说明：slot - 槽位0~3, text - 屏端上送的输入文本。
 *--------------------------------------------------------------*/
static void HandleNameInput(uint8_t slot, const char *text)
{
    uint8_t zone = (uint8_t)(s_page * FIRE_ZONE_PAGE_ZONES + slot + 1U);

#if 0 /* [2026-09-18 按需求注释] 名称校验与截断提示(接收限制约束取消) */
    char clean[FIRE_ZONE_NAME_LEN];
    uint8_t truncated = 0;

    if (ValidateNameGBK(text, clean, &truncated) != 0U)
    {
        SetTip("名称含非法字符");
        return;
    }
#endif
    BeginEditSession();                       /* 压住落盘, 返回时统一保存 */
    (void)FireZone_SetZoneName(zone, text);   /* 原样保存用户输入(不校验不转换) */
    RefreshPage(0);                           /* 回显(名称变化差分写出) */
    SetTip("已输入,返回保存");
}

/*--------------------------------------------------------------
 * 函数名称：HandleRangeInput
 * 功能说明：设备范围输入框处理(九步校验, 校验失败不落盘)。
 *           仅支持一种格式(回路2位+地址3位):
 *           - "LLAAA.LLAAA"(如"01001.01003", 两侧回路须一致)
 *           - 清空键: 全'0'输入(如"00000.00000")清空本分区本栏
 *           校验链: 长度->字符->回路合法->回路与栏匹配->
 *           起止交换->地址上限->产品码识别统计(未识别放行)->
 *           写入RAM挂起(返回时统一落盘)->显示缓存更新(回显)+提示
 * 参数说明：type - 设备类型(栏), slot - 槽位0~3, text - 输入文本。
 *--------------------------------------------------------------*/
static void HandleRangeInput(uint8_t type, uint8_t slot, const char *text)
{
    uint8_t zone = (uint8_t)(s_page * FIRE_ZONE_PAGE_ZONES + slot + 1U);
    uint8_t loop = s_fz_loop[type];       /* 本栏对应回路 */
    /* char disp[16];  [2026-09-18已注释] 不再做格式转换, 显示文本=用户原文 */
    uint16_t dmax = (loop == 2U) ? 63U : 30U;  /* 回路地址上限 */
    size_t len = strlen(text);
    uint8_t ls = 0, le = 0;               /* 起/止回路号 */
    uint16_t as = 0, ae = 0;              /* 起/止地址 */
    uint8_t i;
    uint16_t d;
    uint8_t unknown = 0;                  /* 范围内未识别产品码设备数 */
    uint8_t range_ok = 0U;                /* 输入恰为11位合法格式(仅用于静默更新映射) */

    if (len == 11U)
    {
        /* 清空键: 全'0'输入(分隔位允许'.'或'0') -> 清空本分区本栏 */
        uint8_t allz = 1U;
        for (i = 0U; i < 11U; i++)
        {
            if (i == 5U)
            {
                if ((text[i] != '.') && (text[i] != '0'))
                {
                    allz = 0U; break;
                }
            }
            else if (text[i] != '0')
            {
                allz = 0U; break;
            }
        }
        if (allz != 0U)
        {
            /* 归属清空+显示文本清空: 只改RAM, 返回时统一落盘 */
            BeginEditSession();
            (void)FireZone_ClearZoneRange(zone, loop);
            (void)FireZone_SetSlotDisp(zone, type, "");
            s_last_dev[zone - 1U][type][0] = '\0';  /* 显示缓存同步清空(无输入显示空) */
            RefreshPage(0);
            SetTip("已清空");
            return;
        }
    }

#if 0 /* [2026-09-18 按需求整体注释] 原九步输入校验+格式转换+拒绝路径
       * (接收/保存限制约束取消): 不再拒绝任何输入, 不再把输入转换为
       * "LLAAA-LLAAA"规范化格式, 显示与保存均为用户原文 */
        /* 步骤2: 字符校验(数字, 第6位为'.'分隔符) */
        for (i = 0U; i < 11U; i++)
        {
            char c = text[i];
            if (i == 5U)
            {
                if (c != '.') { SetTip("格式错误"); return; }
            }
            else if ((c < '0') || (c > '9'))
            {
                SetTip("格式错误"); return;
            }
        }

        /* 步骤3/4: LLAAA.LLAAA — 回路2位+地址3位 */
        ls = (uint8_t)(((text[0] - '0') * 10U) + (uint8_t)(text[1] - '0'));
        le = (uint8_t)(((text[6] - '0') * 10U) + (uint8_t)(text[7] - '0'));
        as = (uint16_t)(((uint16_t)(text[2] - '0') * 100U) +
                        ((uint16_t)(text[3] - '0') * 10U) +
                        (uint16_t)(text[4] - '0'));
        ae = (uint16_t)(((uint16_t)(text[8] - '0') * 100U) +
                        ((uint16_t)(text[9] - '0') * 10U) +
                        (uint16_t)(text[10] - '0'));

        /* 起止回路一致性 */
        if (ls != le) { SetTip("起止回路不一致"); return; }

        /* 步骤5: 回路号合法性(1~3) */
        if ((ls < 1U) || (ls > 3U)) { SetTip("回路号无效"); return; }

        /* 回路必须与本栏对应回路一致(温度栏只能输入回路1等) */
        if (ls != loop) { SetTip("回路与本栏不符"); return; }

        /* 步骤6: 起止地址交换(输入反序时自动纠正) */
        if (as > ae) { uint16_t tmp = as; as = ae; ae = tmp; }

        /* 步骤7: 地址范围校验 */
        if ((as < 1U) || (ae > dmax)) { SetTip("地址超范围"); return; }
    }
    else
    {
        /* 步骤1: 长度不合法 */
        SetTip("格式错误");
        return;
    }

    /* 步骤8: 产品码识别统计(未识别放行, 仅记录并提示) */
    for (d = as; d <= ae; d++)
    {
        if (GetDeviceProductCode(loop, d) == 0U)
        {
            unknown++;
        }
    }
    if (unknown > 0U)
    {
        DebugPrintf("[FIREZONE] range L%u %u-%u has %u unidentified dev\r\n",
                    (unsigned)loop, (unsigned)as, (unsigned)ae, (unsigned)unknown);
    }

    /* 显示文本统一"LLAAA-LLAAA"格式(单台为"01001-01001") */
    sprintf(disp, "%02u%03u-%02u%03u", (unsigned)loop, (unsigned)as,
            (unsigned)loop, (unsigned)ae);

    /* 步骤9: 写入RAM(独占语义: 旧分区归属自动移出);
     * 落盘延迟到返回键时FlushPendingSave统一一次擦写 */
    BeginEditSession();
    if (FireZone_SetZoneRange(zone, loop, (uint8_t)as, (uint8_t)ae) != 0U)
    {
        SetTip("参数错误");
        return;
    }
    (void)FireZone_SetSlotDisp(zone, type, disp);
    /* 诊断打印: 输入已受理(挂起待保存)与规范化显示文本 */
    DebugPrintf("[FZS] pending z%u t%u L%u %u-%u disp=%s\r\n",
                (unsigned)zone, (unsigned)type, (unsigned)loop,
                (unsigned)as, (unsigned)ae, disp);

    strncpy(s_last_dev[zone - 1U][type], disp, FIRE_ZONE_NAME_LEN - 1U);
    s_last_dev[zone - 1U][type][FIRE_ZONE_NAME_LEN - 1U] = '\0';

    RefreshPage(0);  /* 回显(显示缓存已更新, 差分写出) */
    if (unknown > 0U)
    {
        SetTip("已输入含未识别,返回保存");  /* 23字节, 不超s_tip容量 */
    }
    else
    {
        SetTip("已输入,返回保存");
    }
#endif

    /* 静默解析: 仅当输入恰为11位合法"LLAAA.LLAAA"时更新设备分区映射
     * (供画面85联动分区受限编辑的FireZone_GetDeviceZone查询使用),
     * 解析失败不做任何提示、不影响显示与保存 */
    if (len == 11U)
    {
        range_ok = 1U;
        for (i = 0U; i < 11U; i++)
        {
            char c = text[i];
            if (i == 5U)
            {
                if (c != '.') { range_ok = 0U; break; }
            }
            else if ((c < '0') || (c > '9'))
            {
                range_ok = 0U; break;
            }
        }
        if (range_ok != 0U)
        {
            ls = (uint8_t)(((text[0] - '0') * 10U) + (uint8_t)(text[1] - '0'));
            le = (uint8_t)(((text[6] - '0') * 10U) + (uint8_t)(text[7] - '0'));
            as = (uint16_t)(((uint16_t)(text[2] - '0') * 100U) +
                            ((uint16_t)(text[3] - '0') * 10U) +
                            (uint16_t)(text[4] - '0'));
            ae = (uint16_t)(((uint16_t)(text[8] - '0') * 100U) +
                            ((uint16_t)(text[9] - '0') * 10U) +
                            (uint16_t)(text[10] - '0'));
            if ((ls != le) || (ls < 1U) || (ls > 3U) || (ls != loop) ||
                (as > ae) || (as < 1U) || (ae > dmax))
            {
                range_ok = 0U;  /* 静默放弃映射更新, 不拒绝不提示 */
            }
        }
    }
    if (range_ok != 0U)
    {
        /* 更新分区映射(独占语义: 旧分区归属自动移出);
         * 落盘延迟到返回键时FlushPendingSave统一一次擦写 */
        BeginEditSession();
        (void)FireZone_SetZoneRange(zone, loop, (uint8_t)as, (uint8_t)ae);
        for (d = as; d <= ae; d++)
        {
            if (GetDeviceProductCode(loop, d) == 0U)
            {
                unknown++;
            }
        }
        DebugPrintf("[FZS] map z%u t%u L%u %u-%u unknown=%u\r\n",
                    (unsigned)zone, (unsigned)type, (unsigned)loop,
                    (unsigned)as, (unsigned)ae, (unsigned)unknown);
    }

    /* 原样保存用户输入(不校验不转换): 显示文本=用户原文,
     * 落盘延迟到返回键时FlushPendingSave统一一次擦写 */
    BeginEditSession();
    (void)FireZone_SetSlotDisp(zone, type, text);
    strncpy(s_last_dev[zone - 1U][type], text, FIRE_ZONE_NAME_LEN - 1U);
    s_last_dev[zone - 1U][type][FIRE_ZONE_NAME_LEN - 1U] = '\0';

    RefreshPage(0);  /* 回显(显示缓存已更新, 差分写出) */
    SetTip("已输入,返回保存");
}

/*--------------------------------------------------------------
 * 函数名称：HandleSysInput
 * 功能说明：系统级设备栏输入处理(画面底部 手报16/声光28/显示盘12)。
 *           - 11位合法 LLAAA.LLAAA 时更新设备地址(回路2, 1-63)
 *           - 全0输入视为清空(地址置0)
 *           - 显示文本原样保存, 返回键时统一落盘
 * 参数说明：kind - 系统级设备类型(FZ_SYS_*), text - 输入文本。
 *--------------------------------------------------------------*/
static void HandleSysInput(uint8_t kind, const char *text)
{
    size_t len;
    uint8_t ls = 0, le = 0;               /* 起/止回路号 */
    uint16_t as = 0, ae = 0;              /* 起/止地址 */
    uint8_t ok = 0U;                      /* 1=11位合法格式 */
    uint8_t i;

    if ((kind >= FZ_SYS_COUNT) || (text == NULL))
    {
        return;
    }
    len = strlen(text);

    /* 清空键: 全0输入(分隔位允许 . 或 0) -> 系统级设备地址清0 */
    if (len == 11U)
    {
        uint8_t allz = 1U;
        for (i = 0U; i < 11U; i++)
        {
            if (i == 5U)
            {
                if ((text[i] != '.') && (text[i] != '0')) { allz = 0U; break; }
            }
            else if (text[i] != '0')
            {
                allz = 0U; break;
            }
        }
        if (allz != 0U)
        {
            BeginEditSession();
            (void)FireZone_SetSysDeviceAddr(kind, 0U);
            (void)FireZone_SetSysDisp(kind, text);
            strncpy(s_last_sys[kind], text, FIRE_ZONE_NAME_LEN - 1U);
            s_last_sys[kind][FIRE_ZONE_NAME_LEN - 1U] = '\0';
            RefreshPage(0);
            SetTip("已清空");
            return;
        }
    }

    /* 静默解析: 仅当输入恰为11位合法 LLAAA.LLAAA 时更新设备地址 */
    if (len == 11U)
    {
        ok = 1U;
        for (i = 0U; i < 11U; i++)
        {
            char c = text[i];
            if (i == 5U)
            {
                if (c != '.') { ok = 0U; break; }
            }
            else if ((c < '0') || (c > '9'))
            {
                ok = 0U; break;
            }
        }
        if (ok != 0U)
        {
            ls = (uint8_t)(((text[0] - '0') * 10U) + (uint8_t)(text[1] - '0'));
            le = (uint8_t)(((text[6] - '0') * 10U) + (uint8_t)(text[7] - '0'));
            as = (uint16_t)(((uint16_t)(text[2] - '0') * 100U) +
                            ((uint16_t)(text[3] - '0') * 10U) +
                            (uint16_t)(text[4] - '0'));
            ae = (uint16_t)(((uint16_t)(text[8] - '0') * 100U) +
                            ((uint16_t)(text[9] - '0') * 10U) +
                            (uint16_t)(text[10] - '0'));
            if ((ls != le) || (ls < 1U) || (ls > 3U) || (as > ae) ||
                (as < 1U) || (ae > 63U))
            {
                ok = 0U;  /* 静默放弃地址更新, 不拒绝不提示 */
            }
        }
    }

    /* 原样保存用户输入; 地址仅合法解析时更新(落盘延迟到返回键) */
    BeginEditSession();
    if (ok != 0U)
    {
        (void)FireZone_SetSysDeviceAddr(kind, (uint8_t)as);
        DebugPrintf("[FZS] sys k%u addr L%u %u\r\n",
                    (unsigned)kind, (unsigned)ls, (unsigned)as);
    }
    (void)FireZone_SetSysDisp(kind, text);
    strncpy(s_last_sys[kind], text, FIRE_ZONE_NAME_LEN - 1U);
    s_last_sys[kind][FIRE_ZONE_NAME_LEN - 1U] = '\0';

    RefreshPage(0);
    SetTip("已输入,返回保存");
}

/*--------------------------------------------------------------
 * 函数名称：SendDiffEx
 * 功能说明：通用差分写屏(带画面号与缓存长度), 供画面95/96使用。
 *--------------------------------------------------------------*/
static void SendDiffEx(uint16_t screen, uint16_t ctrl, const char *buf,
                       char *last, uint16_t lastlen, uint8_t force)
{
    if ((force != 0U) || (strcmp(buf, last) != 0))
    {
        strncpy(last, buf, (size_t)(lastlen - 1U));
        last[lastlen - 1U] = (char)0;
        SetTextValue(screen, ctrl, (uint8_t *)buf);
    }
}

/*--------------------------------------------------------------
 * 函数名称：Refresh95
 * 功能说明：刷新画面95(分区1/2手自动、分区延时、运行/检修)。
 *           进入时force=1全量刷新, 停留期差分刷新。
 *--------------------------------------------------------------*/
static void Refresh95(uint8_t force)
{
    char buf[16];

    /* 分区1/2 手自动: 1=手动 2=自动(数据源与联动引擎判定同源) */
    strcpy(buf, (SystemSaveInfo.part1_hand_or_auto_state == KEY_MANUAL) ? "手动" : "自动");
    SendDiffEx(FIRE_ZONE_SCREEN_MAIN, FZ95_TXT_MODE_Z1, buf, s_last95_mode[0], 8U, force);

    strcpy(buf, (SystemSaveInfo.part2_hand_or_auto_state == KEY_MANUAL) ? "手动" : "自动");
    SendDiffEx(FIRE_ZONE_SCREEN_MAIN, FZ95_TXT_MODE_Z2, buf, s_last95_mode[1], 8U, force);

    /* 分区联动延时(秒) */
    sprintf(buf, "%u", (unsigned)FireZone_GetZoneDelay(1U));
    SendDiffEx(FIRE_ZONE_SCREEN_MAIN, FZ95_IN_DELAY_Z1, buf, s_last95_delay[0], 4U, force);

    sprintf(buf, "%u", (unsigned)FireZone_GetZoneDelay(2U));
    SendDiffEx(FIRE_ZONE_SCREEN_MAIN, FZ95_IN_DELAY_Z2, buf, s_last95_delay[1], 4U, force);

    /* 系统级: 运行/检修(按键板KEY8检修开关) */
    strcpy(buf, (g_maintenance_state != 0U) ? "检修" : "运行");
    SendDiffEx(FIRE_ZONE_SCREEN_MAIN, FZ95_TXT_RUN, buf, s_last95_run, 8U, force);

    /* 系统级设备三栏(16=手报 28=声光 12=显示盘): 进入画面时回显一次 */
    if (force != 0U)
    {
        static const uint16_t sys95[FZ_SYS_COUNT] = { 16U, 28U, 12U };
        uint8_t k;
        char tmp[FIRE_ZONE_DISP_LEN];
        for (k = 0U; k < FZ_SYS_COUNT; k++)
        {
            FireZone_GetSysDisp(k, tmp);
            SetTextValue(FIRE_ZONE_SCREEN_MAIN, sys95[k], (uint8_t *)tmp);
        }
    }
}

#define FZ_DEV_ADDR_MAX   128U   /* 单行最多解析出的地址数(回路1上限100) */

/*--------------------------------------------------------------
 * 函数名称：DevRowMaxAddr
 * 功能说明：取该回路设备地址上限(取自各BSP模块宏, 与数据层一致)。
 *--------------------------------------------------------------*/
static uint16_t DevRowMaxAddr(uint8_t loop)
{
    switch (loop)
    {
    case 1: return (uint16_t)MIXTURE_DEVICE_MAX_ADDR;
    case 2: return (uint16_t)(MBUS_CONTROL_MAX_DEVICES - 1U);
    case 3: return (uint16_t)(RS485_DETECT_MAX_DEVICES - 1U);
    default: return 0U;
    }
}

/*--------------------------------------------------------------
 * 函数名称：AddrPush
 * 功能说明：地址去重追加, 返回1=已满。
 *--------------------------------------------------------------*/
static uint8_t AddrPush(uint16_t *addrs, uint16_t *n, uint16_t v)
{
    uint16_t k;
    for (k = 0U; k < *n; k++)
    {
        if (addrs[k] == v)
        {
            return 0U;   /* 重复地址, 忽略 */
        }
    }
    if (*n >= FZ_DEV_ADDR_MAX)
    {
        return 1U;
    }
    addrs[(*n)++] = v;
    return 0U;
}

/*--------------------------------------------------------------
 * 函数名称：ParseDevList
 * 功能说明：解析一行的设备列表文本, 语法:
 *             项 ("," 项)* , 项 = LLAAA | LLAAA.LLAAA | LLAAA-LLAAA
 *           分隔符用半角逗号, 兼容全角逗号(任何非数字字符均跳过);
 *           # 结束符可有可无, 自动忽略; 起止反序自动交换。
 * 返回值   ：0=成功 1=格式错误 2=回路不符 3=地址越界 4=数量超限
 *--------------------------------------------------------------*/
static uint8_t ParseDevList(const char *text, uint8_t loop_no, uint16_t *addrs, uint16_t *count)
{
    uint16_t i = 0U;
    uint16_t n = 0U;
    uint16_t dmax = DevRowMaxAddr(loop_no);

    *count = 0U;
    if (text == NULL || dmax == 0U)
    {
        return 1U;
    }

    while (text[i] != (char)0)
    {
        uint8_t digits = 0U;
        uint8_t val[5];
        uint16_t l;
        uint16_t a;

        /* 跳过非数字字符: 逗号/空格/全角逗号/# 等 */
        if ((text[i] < (char)48) || (text[i] > (char)57))
        {
            i++;
            continue;
        }

        /* 第一组5位: 回路2位 + 地址3位 */
        while ((digits < 5U) && (text[i] >= (char)48) && (text[i] <= (char)57))
        {
            val[digits++] = (uint8_t)(text[i] - (char)48);
            i++;
        }
        if (digits != 5U)
        {
            return 1U;
        }
        l = (uint16_t)((uint16_t)val[0] * 10U + (uint16_t)val[1]);
        a = (uint16_t)((uint16_t)val[2] * 100U + (uint16_t)val[3] * 10U + (uint16_t)val[4]);
        if (l != (uint16_t)loop_no)
        {
            return 2U;
        }
        if ((a < 1U) || (a > dmax))
        {
            return 3U;
        }

        if ((text[i] == (char)46) || (text[i] == (char)45))
        {
            uint16_t b;
            i++;
            digits = 0U;
            while ((digits < 5U) && (text[i] >= (char)48) && (text[i] <= (char)57))
            {
                val[digits++] = (uint8_t)(text[i] - (char)48);
                i++;
            }
            if (digits != 5U)
            {
                return 1U;
            }
            if ((uint16_t)((uint16_t)val[0] * 10U + (uint16_t)val[1]) != (uint16_t)loop_no)
            {
                return 2U;
            }
            b = (uint16_t)((uint16_t)val[2] * 100U + (uint16_t)val[3] * 10U + (uint16_t)val[4]);
            if ((b < 1U) || (b > dmax))
            {
                return 3U;
            }
            if (a > b)
            {
                uint16_t tmp = a;
                a = b;
                b = tmp;   /* 起止反序自动交换 */
            }
            while (a <= b)
            {
                if (AddrPush(addrs, &n, a) != 0U)
                {
                    return 4U;
                }
                a++;
            }
        }
        else
        {
            if (AddrPush(addrs, &n, a) != 0U)
            {
                return 4U;
            }
        }
    }

    *count = n;
    return 0U;
}

/*--------------------------------------------------------------
 * 函数名称：SortAddrs
 * 功能说明：地址升序插入排序(地址数<=128, 开销可忽略)。
 *--------------------------------------------------------------*/
static void SortAddrs(uint16_t *a, uint16_t n)
{
    uint16_t i;
    uint16_t j;
    for (i = 1U; i < n; i++)
    {
        uint16_t v = a[i];
        j = i;
        while ((j > 0U) && (a[j - 1U] > v))
        {
            a[j] = a[j - 1U];
            j--;
        }
        a[j] = v;
    }
}

/*--------------------------------------------------------------
 * 函数名称：BuildDevListText
 * 功能说明：把地址集合压缩成显示文本: 连续段合并为 LLAAA.LLBBB,
 *           单点保留 LLAAA, 段间用半角逗号分隔。
 *           压缩后仍超缓冲容量时改写为"已录入N台"。
 *--------------------------------------------------------------*/
static void BuildDevListText(uint16_t *addrs, uint16_t count, uint8_t loop, char *out, uint16_t outsize)
{
    uint16_t i = 0U;
    uint16_t len = 0U;

    out[0] = (char)0;
    if (count == 0U)
    {
        return;
    }

    SortAddrs(addrs, count);

    while (i < count)
    {
        uint16_t j = i;
        char seg[24];
        int sl;
        while (((j + 1U) < count) && (addrs[j + 1U] == (uint16_t)(addrs[j] + 1U)))
        {
            j++;
        }
        if (j == i)
        {
            sl = sprintf(seg, "%02u%03u", (unsigned)loop, (unsigned)addrs[i]);
        }
        else
        {
            sl = sprintf(seg, "%02u%03u.%02u%03u", (unsigned)loop, (unsigned)addrs[i], (unsigned)loop, (unsigned)addrs[j]);
        }
        if (((uint16_t)sl + len + 1U) >= outsize)
        {
            sprintf(out, "已录入%u台", (unsigned)count);
            return;
        }
        if (len > 0U)
        {
            out[len++] = (char)44;
        }
        memcpy(&out[len], seg, (size_t)sl);
        len += (uint16_t)sl;
        out[len] = (char)0;
        i = j + 1U;
    }
}

/*--------------------------------------------------------------
 * 函数名称：HandleDevListInput
 * 功能说明：画面96单行设备列表录入(覆盖式)。
 *           1) 解析新输入; 2) 按该行旧文本清除本分区旧归属;
 *           3) 写入新集合; 4) 压缩回显。
 *           落盘延迟到离开画面时由FlushPendingSave统一执行。
 *           解析失败不修改任何分区数据, 把原因回写到该行输入框。
 *--------------------------------------------------------------*/
static void HandleDevListInput(uint8_t row, const char *text)
{
    static uint16_t old_addrs[FZ_DEV_ADDR_MAX];
    static uint16_t new_addrs[FZ_DEV_ADDR_MAX];
    uint16_t old_n = 0U;
    uint16_t new_n = 0U;
    uint16_t k;
    uint8_t slot;
    uint8_t loop;
    uint8_t err;
    char oldtext[FIRE_ZONE_DISP_LEN];
    char disp[FIRE_ZONE_DISP_LEN];
    const char *msg;

    if ((row >= FZ_DEV_TYPE_COUNT) || (text == NULL))
    {
        return;
    }
    if ((s_dev_zone < 1U) || (s_dev_zone > (uint8_t)FIRE_ZONE_SLOT_MAX))
    {
        return;
    }

    slot = s_dev_row_slot[row];
    loop = s_dev_slot_loop[slot];

    /* 清空约定: 空串或单个 0 */
    if ((text[0] == (char)0) || ((text[0] == (char)48) && (text[1] == (char)0)))
    {
        err = 0U;
        new_n = 0U;
    }
    else
    {
        err = ParseDevList(text, loop, new_addrs, &new_n);
    }

    if (err != 0U)
    {
        switch (err)
        {
        case 1U: msg = "格式错误"; break;
        case 2U: msg = "回路不符"; break;
        case 3U: msg = "地址越界"; break;
        default: msg = "数量超限"; break;
        }
        SetTextValue(FIRE_ZONE_SCREEN_DEV, s_dev_row_ctrl[row], (uint8_t *)msg);
        DebugPrintf("[FZS96] row%u err=%u\r\n", (unsigned)row, (unsigned)err);
        return;
    }

    BeginEditSession();   /* 只改RAM, 离开画面时统一落盘一次 */

    /* 1. 清除该行旧文本列出的、属于本分区的地址 */
    FireZone_GetSlotDisp(s_dev_zone, slot, oldtext);
    if (ParseDevList(oldtext, loop, old_addrs, &old_n) == 0U)
    {
        for (k = 0U; k < old_n; k++)
        {
            (void)FireZone_ClearDevice(s_dev_zone, loop, old_addrs[k]);
        }
    }

    /* 2. 写入新集合(独占语义: 若该地址原属另一分区, 会自动移出) */
    for (k = 0U; k < new_n; k++)
    {
        (void)FireZone_SetZoneRange(s_dev_zone, loop, (uint8_t)new_addrs[k], (uint8_t)new_addrs[k]);
    }

    /* 3. 压缩回显并保存显示文本 */
    BuildDevListText(new_addrs, new_n, loop, disp, (uint16_t)sizeof(disp));
    (void)FireZone_SetSlotDisp(s_dev_zone, slot, disp);
    strncpy(s_last96[row], disp, FIRE_ZONE_DISP_LEN - 1U);
    s_last96[row][FIRE_ZONE_DISP_LEN - 1U] = (char)0;
    SetTextValue(FIRE_ZONE_SCREEN_DEV, s_dev_row_ctrl[row], (uint8_t *)disp);

    DebugPrintf("[FZS96] z%u slot%u n=%u %s\r\n",
                (unsigned)s_dev_zone, (unsigned)slot, (unsigned)new_n, disp);
}

/*--------------------------------------------------------------
 * 8. 对外接口
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_UpdateUI
 * 功能说明：防火分区画面UI刷新(cmd_process.c的UpdateUI循环调用)。
 *           离开画面复位标志; 进入画面回第1页并全量刷新;
 *           停留期间差分刷新+提示条超时清空。
 * 参数说明：screen_id - 当前画面ID。
 *--------------------------------------------------------------*/
void FireZoneScreen_UpdateUI(uint16_t screen_id)
{
    /* 画面95: 气灭分区总览 */
    if (screen_id == FIRE_ZONE_SCREEN_MAIN)
    {
        s_in96 = 0U;
        if (s_in95 == 0U)
        {
            FlushPendingSave();   /* 首次进入(含从96返回): 统一落盘一次 */
            s_in95 = 1U;
            Refresh95(1U);       /* 进入: 全量刷新 */
        }
        else
        {
            Refresh95(0U);       /* 停留: 差分刷新 */
        }
        return;
    }

    /* 画面96: 分区设备管理 */
    if (screen_id == FIRE_ZONE_SCREEN_DEV)
    {
        s_in95 = 0U;
        if (s_in96 == 0U)
        {
            uint8_t i;
            FlushPendingSave();   /* 首次进入: 先清掉84遗留的挂起修改 */
            s_in96 = 1U;
            /* 进入: 按当前编辑分区回显8行设备列表 */
            for (i = 0U; i < FZ_DEV_TYPE_COUNT; i++)
            {
                FireZone_GetSlotDisp(s_dev_zone, s_dev_row_slot[i], s_last96[i]);
                SetTextValue(FIRE_ZONE_SCREEN_DEV, s_dev_row_ctrl[i], (uint8_t *)s_last96[i]);
            }
            DebugPrintf("[FZS96] enter zone=%u\r\n", (unsigned)s_dev_zone);
        }
        return;
    }

    if (screen_id != FIRE_ZONE_SCREEN_ID)
    {
        FlushPendingSave();
        s_in_page = 0U;
        s_in95 = 0U;
        s_in96 = 0U;
        return;
    }

    s_in95 = 0U;
    s_in96 = 0U;

    if (s_in_page == 0U)
    {
        s_in_page = 1U;
        s_page = 0U;
        s_tip[0] = (char)0;
        RestoreDispCache();
        RefreshPage(1U);
        return;
    }

    TipTimeoutCheck();
    RefreshPage(0U);
}

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_OnButton
 * 功能说明：防火分区画面按钮处理(cmd_process.c的NotifyButton调用)。
 *           105/106: 启用/停用切换(分区1禁止停用);
 *           264/265: 两页回绕翻页(全量刷新);
 *           266: 返回键屏端自跳新菜单界面, MCU仅复位标志防误处理。
 * 参数说明：screen_id - 画面ID, control_id - 按钮ID, state - 1=按下。
 *--------------------------------------------------------------*/
void FireZoneScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    uint8_t slot;         /* 槽位 */
    uint8_t zone;         /* 槽位对应分区号 */

    /* 只处理本画面的按下事件 */
    if ((screen_id != FIRE_ZONE_SCREEN_ID) || (state != 1U))
    {
        return;
    }

    if (s_in_page == 0U)
    {
        /* 自愈: 屏端已在本画面才会发按键, 补做进入初始化,
         * 防止UI任务轮询滞后导致进入画面后的首批按键被丢弃(操作无反应) */
        s_in_page = 1U;
        s_page = 0U;
        RestoreDispCache();
    }

    /* 启用/停用切换按钮(105/106) */
    if ((control_id >= FIRE_ZONE_BTN_ENABLE_0) && (control_id <= FIRE_ZONE_BTN_ENABLE_1))
    {
        slot = (uint8_t)(control_id - FIRE_ZONE_BTN_ENABLE_0);
        zone = (uint8_t)(s_page * FIRE_ZONE_PAGE_ZONES + slot + 1U);

        /* 硬约束: 分区1禁止停用 */
        if ((zone == 1U) && (FireZone_GetZoneEnabled(1U) != 0U))
        {
            SetTip("分区1不可停用");
            return;
        }

        BeginEditSession();                      /* 压住落盘, 返回时统一保存 */
        (void)FireZone_ToggleZoneEnabled(zone);  /* 翻转启用状态(只改RAM) */
        RefreshPage(0);                          /* 状态回显差分刷新 */
        return;
    }

    switch (control_id)
    {
    case FIRE_ZONE_BTN_PREV:
    case FIRE_ZONE_BTN_NEXT:
        /* 2分区单页(分区1~2): 屏端翻页键仍在, MCU直接忽略,
         * 不改页码/不刷屏/不产生保存动作 */
        return;

    case FIRE_ZONE_BTN_RETURN:
        s_in_page = 0U;    /* 屏端自跳画面68, MCU不发切页指令(防双跳);
                            * 落盘由UpdateUI离开分支FlushPendingSave兜底 */
        return;

    default:
        return;            /* 其他控件不处理 */
    }
}

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_NotifyText
 * 功能说明：防火分区画面文本输入处理(cmd_process.c的NotifyText调用)。
 *           分发: 分区名输入框(1~4)/设备范围输入框(5~29中映射表内)。
 *           注意: 本函数在cmd_process.c权限检查之后调用(画面84
 *           为三级权限), 此处仅做画面/控件归属分发。
 * 参数说明：screen_id - 画面ID, control_id - 控件ID, text - 输入文本。
 * 返回值：   1=本画面控件已处理, 0=未处理(交由其他模块)。
 *--------------------------------------------------------------*/
uint8_t FireZoneScreen_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text)
{
    uint8_t type;         /* 设备类型游标 */
    uint8_t slot;         /* 槽位游标 */

    /* 画面95: 分区联动延时输入(9=分区1, 24=分区2), 范围0-30秒 */
    if (screen_id == FIRE_ZONE_SCREEN_MAIN)
    {
        unsigned int uv = 0U;
        uint8_t zone;
        if (control_id == FZ95_IN_DELAY_Z1)
        {
            zone = 1U;
        }
        else if (control_id == FZ95_IN_DELAY_Z2)
        {
            zone = 2U;
        }
        else
        {
            static const uint16_t sys95[FZ_SYS_COUNT] = { 16U, 28U, 12U };
            uint8_t k;
            char tmp[FIRE_ZONE_DISP_LEN];
            if (text == NULL)
            {
                return 1;
            }
            /* 系统级设备输入框: 16=手报 28=声光 12=显示盘 */
            for (k = 0U; k < FZ_SYS_COUNT; k++)
            {
                if (sys95[k] == control_id)
                {
                    HandleSysInput(k, (const char *)text);
                    FireZone_GetSysDisp(k, tmp);
                    SetTextValue(FIRE_ZONE_SCREEN_MAIN, sys95[k], (uint8_t *)tmp);
                    return 1;
                }
            }
            return 0;
        }
        if (text == NULL)
        {
            return 1;
        }
        if (sscanf((const char *)text, "%u", &uv) != 1)
        {
            uv = 0U;
        }
        if (uv > (unsigned int)FIRE_ZONE_DELAY_MAX)
        {
            uv = (unsigned int)FIRE_ZONE_DELAY_MAX;   /* 超上限截断, 回显实际生效值 */
        }
        (void)FireZone_SetZoneDelay(zone, (uint8_t)uv);
        Refresh95(1U);
        DebugPrintf("[FZS95] delay zone%u = %u\r\n", (unsigned)zone, uv);
        return 1;
    }

    /* 画面96: 8行设备列表录入 */
    if (screen_id == FIRE_ZONE_SCREEN_DEV)
    {
        uint8_t row;
        if (text == NULL)
        {
            return 0;
        }
        for (row = 0U; row < FZ_DEV_TYPE_COUNT; row++)
        {
            if (s_dev_row_ctrl[row] == (uint16_t)control_id)
            {
                HandleDevListInput(row, (const char *)text);
                return 1;
            }
        }
        return 0;
    }

    if (screen_id != FIRE_ZONE_SCREEN_ID)
    {
        return 0;         /* 非本画面: 不处理 */
    }
    if (text == NULL)
    {
        return 0;
    }

#if 0 /* [2026-09-18 按需求注释] 接收侧净化过滤取消: 屏端回传文本原样分发,
       * 不截断尾部字符不去空格, 显示与保存均为用户原文(原防尾部垃圾机制) */
    char clean[FIRE_ZONE_NAME_LEN + 16]; /* 净化后的输入文本缓冲 */
    uint8_t n = 0U;       /* 净化后文本长度 */

    /* 屏端键盘回传文本可能携带尾部控制字符(回车0x0D/换行0x0A, 不同屏
     * 固件版本行为不一), 而设备范围校验严格要求11位、分区名校验拒绝
     * 0x20以下字符, 尾部垃圾会导致全部输入被拒(84页所有内容保存不了)。
     * 此处统一截断尾部回车/换行并去除尾部空格后再分发。 */
    while ((text[n] != '\0') && (text[n] != '\r') && (text[n] != '\n') &&
           (n < (sizeof(clean) - 1U)))
    {
        clean[n] = (char)text[n];
        n++;
    }
    clean[n] = '\0';
    while ((n > 0U) && (clean[n - 1U] == ' '))
    {
        n--;
        clean[n] = '\0';  /* 去除尾部空格 */
    }
    text = (const uint8_t *)clean;  /* 后续统一使用净化后的文本 */
#endif

    if (s_in_page == 0U)
    {
        /* 自愈: 屏端已在本画面(否则不会上送文本), 补做进入初始化,
         * 防止UI任务轮询滞后导致进入画面后的首笔输入被丢弃(操作无反应)。
         * 注意: 此处不能再调 RefreshPage(1) - 屏端输入框此刻已显示用户刚输入的
         * 内容, 全量刷新会用Flash旧值把它覆盖掉, 表现为"输入了但没保存",
         * 且只出现在进入画面后的第一笔输入(控件24系统级手报最易触发)。 */
        s_in_page = 1U;
        s_page = 0U;
        RestoreDispCache();
    }

    /* 诊断打印: 确认文本通知到达本模块(控件/原文长度/文本) */
    DebugPrintf("[FZS] notify ctl=%u len=%u text=%s\r\n",
                (unsigned)control_id, (unsigned)strlen((const char *)text),
                (const char *)text);

    /* 分区名输入框(1/2) */
    if ((control_id >= FIRE_ZONE_IN_NAME_0) && (control_id <= FIRE_ZONE_IN_NAME_1))
    {
        HandleNameInput((uint8_t)(control_id - FIRE_ZONE_IN_NAME_0), (const char *)text);
        return 1;
    }

    /* 设备范围输入框(按映射表反查类型与槽位) */
    for (type = 0U; type < FZ_DEV_TYPE_COUNT; type++)
    {
        for (slot = 0U; slot < FIRE_ZONE_PAGE_ZONES; slot++)
        {
            if (s_fz_dev[type][slot] == (uint8_t)control_id)
            {
                HandleRangeInput(type, slot, (const char *)text);
                return 1;
            }
        }
    }

    /* 系统级设备输入框(16/28/12) */
    for (type = 0U; type < FZ_SYS_COUNT; type++)
    {
        if (s_fz_sys[type] == (uint8_t)control_id)
        {
            HandleSysInput(type, (const char *)text);
            return 1;
        }
    }

    /* 兜底: 本画面其余控件若上送 11 位 LLAAA.LLAAA 文本,
     * 视为系统级手报输入。用于屏工程 24 号控件的事件通知可能绑定了
     * 其它 control_id 的场合(属性面板看不出差异, 只能运行期兜住)。
     * 设备栏(5/6/9/10/14/15/18/19/22/23/26/27/4/8)与分区名(1/2)
     * 已在上面匹配, 不会走到这里; 只读控件(101/102/111/112)不上送文本。
     * 屏端修正 ID 后, 此分支自然不再触发。 */
    if (strlen((const char *)text) == 11U)
    {
        HandleSysInput((uint8_t)FZ_SYS_MANUAL, (const char *)text);
        return 1;
    }

    return 0;             /* 分区号/状态回显等只读控件: 不处理 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_OnZoneChanged
 * 功能说明：分区数据变更回调(经FireZone_SetChangedCallback注册)。
 *           当前显示策略: 设备栏只随屏端输入变化(与分区表脱钩),
 *           输入路径已即时回显, 无需在此刷屏; 保留空实现供
 *           联动模块扩展(如分区停用后联动屏蔽)。
 *--------------------------------------------------------------*/
void FireZoneScreen_OnZoneChanged(void)
{
    /* 预留: 联动扩展点 */
}
/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_SetDevZone
 * 功能说明：设定画面96当前编辑的分区(由画面95的8/26调用)。
 *--------------------------------------------------------------*/
void FireZoneScreen_SetDevZone(uint8_t zone)
{
    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_SLOT_MAX))
    {
        return;
    }
    s_dev_zone = zone;
    s_in96 = 0U;   /* 换分区后重新回显 */
    DebugPrintf("[FZS96] dev zone=%u\r\n", (unsigned)zone);
}

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_GetDevZone
 * 功能说明：查询画面96当前编辑的分区。
 *--------------------------------------------------------------*/
uint8_t FireZoneScreen_GetDevZone(void)
{
    return s_dev_zone;
}
