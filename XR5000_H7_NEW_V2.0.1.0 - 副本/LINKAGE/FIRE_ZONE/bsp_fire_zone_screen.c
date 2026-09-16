/*==============================================================
 * 文件名称   : bsp_fire_zone_screen.c
 * 模块功能   : 防火分区画面显示与翻页（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 功能说明   : 在防火分区画面(9行×6列表格)显示各分区设备分布:
 *              列0=分区号, 列1=分区名称, 列2=温度, 列3=烟雾,
 *              列4=声光, 列5=输出。
 *              - 温度/烟雾列: 回路1温/烟探测器 + 回路3烟温复合探测器
 *              - 声光列:     回路2声光报警器(XR-SGBJQ)
 *              - 输出列:     回路2其他可输出设备(如FCM-1011)
 *              - 每类设备按地址升序显示"1,3,5", 超过4台显示"n台"
 *              - 仅统计实际接入的设备(在线/已识别), 未接入不显示
 *              - 翻页按钮22/23: 每页9个分区, 分区数<=9时仅1页
 * 刷新策略   : 差分刷新(内容变化才写屏), 避免周期刷新导致画面闪烁
 * 依赖模块   : bsp_fire_zone(分区表), bsp_device_registry(产品能力),
 *              bsp_mbus/bsp_mbus_control/bsp_rs485_detect(设备识别结果),
 *              hmi_driver(SetTextValue写屏)
 *==============================================================*/

#include "bsp_fire_zone_screen.h"   /* 自身头文件: 画面/控件定义 */
#include "bsp_fire_zone.h"          /* 分区表数据层API */
#include "bsp_device_registry.h"    /* 产品码枚举/设备能力查询 */
#include "bsp_mbus.h"               /* 回路1探测器在线状态/产品码 */
#include "bsp_mbus_control.h"       /* 回路2设备在线状态/产品码 */
#include "bsp_rs485_detect.h"       /* 回路3探测器在线状态/产品码 */
#include "hmi_driver.h"             /* SetTextValue屏幕写文本接口 */

#include <string.h>                 /* strcmp/strcpy/strncpy */
#include <stdio.h>                  /* sprintf */

/*--------------------------------------------------------------
 * 1. 常量与类型定义区
 *--------------------------------------------------------------*/

/* 设备类别列掩码(用于将设备归并到表格的类别列) */
#define ZCOL_TEMP            0x01U  /* 温度列 */
#define ZCOL_SMOKE           0x02U  /* 烟雾列 */
#define ZCOL_SOUND           0x04U  /* 声光列 */
#define ZCOL_OUTPUT          0x08U  /* 输出列 */

/* 每类设备最多逐个显示的地址个数, 超过则显示"n台" */
#define FIRE_ZONE_LIST_MAX   4

/* 表格控件ID映射表[行][列](与屏工程防火分区.tft逐一核对):
 * 行0: 分区号=1, 行内5列=10~14
 * 行1: 分区号=2, 行内5列=15,16,17,18,28 (屏工程此行末列不连续)
 * 行2~8: 分区号=3~9, 行内5列=31~65连续 */
static const uint16_t s_grid_id[FIRE_ZONE_ROWS][FIRE_ZONE_COLS] =
{
    {1, 10, 11, 12, 13, 14},
    {2, 15, 16, 17, 18, 28},
    {3, 31, 32, 33, 34, 35},
    {4, 36, 37, 38, 39, 40},
    {5, 41, 42, 43, 44, 45},
    {6, 46, 47, 48, 49, 50},
    {7, 51, 52, 53, 54, 55},
    {8, 56, 57, 58, 59, 60},
    {9, 61, 62, 63, 64, 65},
};

/* 各列对应的设备类别掩码(列0分区号/列1名称不走类别) */
static const uint8_t s_col_mask[FIRE_ZONE_COLS] =
{
    0,           /* 列0: 分区号(数字) */
    0,           /* 列1: 分区名称(文本) */
    ZCOL_TEMP,   /* 列2: 温度 */
    ZCOL_SMOKE,  /* 列3: 烟雾 */
    ZCOL_SOUND,  /* 列4: 声光 */
    ZCOL_OUTPUT, /* 列5: 输出 */
};

/*--------------------------------------------------------------
 * 2. 内部状态变量
 *--------------------------------------------------------------*/

static uint8_t s_page = 0;     /* 当前页码(0基), 分区数>9时翻页有效 */
static uint8_t s_in_page = 0;  /* 当前是否处于防火分区画面(进入/离开边沿检测) */

/* 差分刷新缓存: 每格上次写入的文本(名称最长15字节, 地址列表最长11字节) */
static char s_last_text[FIRE_ZONE_ROWS][FIRE_ZONE_COLS][FIRE_ZONE_NAME_LEN];

/*--------------------------------------------------------------
 * 3. 内部工具函数 - 设备信息查询
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
 * 函数名称：IsDevicePresent
 * 功能说明：判断设备当前是否实际接入(与联动可用性判断一致)。
 * 参数说明：loop_no - 回路号(1-3), dev_no - 设备地址。
 * 返回值：   1=接入, 0=未接入/参数非法。
 *--------------------------------------------------------------*/
static uint8_t IsDevicePresent(uint8_t loop_no, uint16_t dev_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: 探测器在线状态(1=在线, 0=离线, 255=参数错误) */
        return (getPointTypeMixtureDetectOnlineState((uint8_t)dev_no) == 1) ? 1 : 0;

    case 2:
        /* 回路2: 设备在线标志(与联动控制可用性一致) */
        return (MBusCtrl_GetOnline((uint8_t)dev_no) != 0) ? 1 : 0;

    case 3:
        /* 回路3: 探测器真正在线(上线且未掉线) */
        return (RS485Detect_IsOnline((uint8_t)dev_no) != 0) ? 1 : 0;

    default:
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：ClassifyDeviceColumns
 * 功能说明：按产品码将设备归并到表格类别列。
 *           分类依据(产品白名单, 不依赖各回路私有掩码位定义):
 *           回路1: XR8002温度传感器 / XR8001烟雾感烟探测器
 *           回路3: XR805系列/XR8303/XR8305均为烟温复合探测器
 *           回路2: XR-SGBJQ声光报警器 / 其他带输出能力设备(FCM-1011)
 *           回路2手动报警按钮/火灾显示盘等不属于四类, 不显示。
 * 参数说明：loop_no - 回路号, pc - 设备产品码。
 * 返回值：   类别列掩码(ZCOL_xxx按位或), 0=不显示。
 *--------------------------------------------------------------*/
static uint8_t ClassifyDeviceColumns(uint8_t loop_no, uint16_t pc)
{
    /* 先按产品码分派, 再校验回路号是否匹配该产品 */
    switch (pc)
    {
    case DEVICE_PRODUCT_XR8002_TEMP:
        return (loop_no == 1U) ? ZCOL_TEMP : 0;         /* 回路1温度探测器 */

    case DEVICE_PRODUCT_XR8001_SMOKE:
        return (loop_no == 1U) ? ZCOL_SMOKE : 0;        /* 回路1感烟探测器 */

    case DEVICE_PRODUCT_XR805_V20:
    case DEVICE_PRODUCT_XR805_EXD:
    case DEVICE_PRODUCT_XR805_EXI:
    case DEVICE_PRODUCT_XR8303:
    case DEVICE_PRODUCT_XR8305:
        return (loop_no == 3U) ? (ZCOL_TEMP | ZCOL_SMOKE) : 0; /* 回路3烟温复合 */

    case DEVICE_PRODUCT_SGBJQ:
        return (loop_no == 2U) ? ZCOL_SOUND : 0;        /* 回路2声光报警器 */

    default:
        /* 其他回路2输出设备(如FCM-1011): 按能力位判定输出列 */
        if (loop_no == 2U)
        {
            if ((DeviceRegistry_GetCapabilities(pc) & DEVICE_CAP_OUTPUT_CONTROL) != 0U)
            {
                return ZCOL_OUTPUT;
            }
        }
        return 0;
    }
}

/*--------------------------------------------------------------
 * 4. 内部工具函数 - 单元格内容构建与页面刷新
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：BuildCellList
 * 功能说明：构建某分区某类别列的设备地址列表文本。
 *           格式: 地址升序"1,3,5"(最多4个), 超过显示"n台"。
 * 参数说明：zone_no - 分区号(1-8), col_mask - 类别列掩码,
 *           buf - 输出缓冲区(容量FIRE_ZONE_NAME_LEN)。
 *--------------------------------------------------------------*/
static void BuildCellList(uint8_t zone_no, uint8_t col_mask, char *buf)
{
    uint8_t n;            /* 分区内地址总数 */
    uint8_t idx;          /* 枚举索引 */
    uint8_t count = 0;    /* 已匹配设备计数 */
    uint8_t loop_no;      /* 枚举出的回路号 */
    uint16_t dev_no;      /* 枚举出的设备地址 */
    uint16_t pc;          /* 设备产品码 */

    n = FireZone_GetZoneDeviceCount(zone_no);
    for (idx = 0; idx < n; idx++)
    {
        if (FireZone_GetZoneDeviceAt(zone_no, idx, &loop_no, &dev_no) == 0)
        {
            break;  /* 越界保护 */
        }

        /* 只统计实际接入的设备 */
        if (IsDevicePresent(loop_no, dev_no) == 0)
        {
            continue;
        }

        /* 产品码未识别时不显示 */
        pc = GetDeviceProductCode(loop_no, dev_no);
        if (pc == 0U)
        {
            continue;
        }

        /* 类别不匹配本列则跳过 */
        if ((ClassifyDeviceColumns(loop_no, pc) & col_mask) == 0U)
        {
            continue;
        }

        count++;
        if (count <= FIRE_ZONE_LIST_MAX)
        {
            /* 逐个拼接地址: "1,3,5" */
            sprintf(buf + strlen(buf), "%s%u", (count > 1U) ? "," : "", (unsigned)dev_no);
        }
    }

    if (count > FIRE_ZONE_LIST_MAX)
    {
        /* 超过逐个显示上限: 显示台数 */
        sprintf(buf, "%u台", (unsigned)count);
    }
}

/*--------------------------------------------------------------
 * 函数名称：GetMaxPage
 * 功能说明：按当前分区总数计算最大页码数(至少1页)。
 * 返回值：   最大页数。
 *--------------------------------------------------------------*/
static uint8_t GetMaxPage(void)
{
    uint8_t zone_total = FireZone_GetZoneCount();
    return (uint8_t)((zone_total + FIRE_ZONE_ROWS - 1U) / FIRE_ZONE_ROWS);
}

/*--------------------------------------------------------------
 * 函数名称：RefreshPage
 * 功能说明：刷新整页表格(9行×6列), 支持差分刷新。
 * 参数说明：force - 1=强制全量刷新, 0=差分刷新(内容变化才写屏)。
 *--------------------------------------------------------------*/
static void RefreshPage(uint8_t force)
{
    uint8_t row;          /* 行索引 */
    uint8_t col;          /* 列索引 */
    uint8_t zone;         /* 当前行对应的分区号 */
    uint8_t zone_total;   /* 当前分区总数 */
    char buf[FIRE_ZONE_NAME_LEN];

    zone_total = FireZone_GetZoneCount();

    /* 页码越界保护 */
    if (s_page >= GetMaxPage())
    {
        s_page = (uint8_t)(GetMaxPage() - 1U);
    }

    for (row = 0; row < FIRE_ZONE_ROWS; row++)
    {
        zone = (uint8_t)(s_page * FIRE_ZONE_ROWS + row + 1U);

        for (col = 0; col < FIRE_ZONE_COLS; col++)
        {
            /* 默认空文本(分区号超过总数时整行留空) */
            buf[0] = '\0';

            if (zone <= zone_total)
            {
                if (col == 0U)
                {
                    /* 列0: 分区号 */
                    sprintf(buf, "%u", (unsigned)zone);
                }
                else if (col == 1U)
                {
                    /* 列1: 分区名称(GBK串直接显示) */
                    strncpy(buf, FireZone_GetZoneName(zone), FIRE_ZONE_NAME_LEN - 1U);
                }
                else
                {
                    /* 列2-5: 温度/烟雾/声光/输出设备地址列表 */
                    BuildCellList(zone, s_col_mask[col], buf);
                }
            }

            buf[FIRE_ZONE_NAME_LEN - 1U] = '\0';  /* 确保结束符 */

            /* 差分刷新: 内容变化才写屏, 避免画面闪烁 */
            if (force || (strcmp(buf, s_last_text[row][col]) != 0))
            {
                strcpy(s_last_text[row][col], buf);
                SetTextValue(FIRE_ZONE_SCREEN_ID, s_grid_id[row][col], (uint8_t *)buf);
            }
        }
    }
}

/*--------------------------------------------------------------
 * 5. 对外接口
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_UpdateUI
 * 功能说明：防火分区画面UI刷新(在cmd_process.c的UpdateUI循环中调用)。
 *           非本画面时复位进入标志; 进入画面首帧全量刷新,
 *           之后差分刷新(在线状态/识别结果变化时自动更新)。
 * 参数说明：screen_id - 当前画面ID。
 *--------------------------------------------------------------*/
void FireZoneScreen_UpdateUI(uint16_t screen_id)
{
    if (screen_id != FIRE_ZONE_SCREEN_ID)
    {
        s_in_page = 0;  /* 记录离开画面, 下次进入强制全刷 */
        return;
    }

    if (s_in_page == 0U)
    {
        s_in_page = 1;      /* 进入画面 */
        RefreshPage(1);     /* 首帧全量刷新 */
        return;
    }

    RefreshPage(0);         /* 停留画面: 差分刷新 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZoneScreen_OnButton
 * 功能说明：防火分区画面按钮处理(在cmd_process.c的NotifyButton中调用)。
 *           仅处理本画面的上一页(22)/下一页(23)按下事件;
 *           按钮19=返回由屏工程自跳转, 不在此处理。
 * 参数说明：screen_id - 画面ID, control_id - 按钮ID, state - 1=按下。
 *--------------------------------------------------------------*/
void FireZoneScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    uint8_t max_page;

    /* 只处理本画面的按下事件 */
    if (screen_id != FIRE_ZONE_SCREEN_ID || state != 1U)
    {
        return;
    }

    if (control_id != FIRE_ZONE_BTN_PREV && control_id != FIRE_ZONE_BTN_NEXT)
    {
        return;
    }

    max_page = GetMaxPage();
    if (max_page == 0U)
    {
        max_page = 1U;
    }

    if (control_id == FIRE_ZONE_BTN_PREV)
    {
        /* 上一页 */
        if (s_page > 0U)
        {
            s_page--;
            if (s_in_page != 0U)
            {
                RefreshPage(1);  /* 翻页后全量刷新 */
            }
        }
    }
    else
    {
        /* 下一页 */
        if (s_page < (uint8_t)(max_page - 1U))
        {
            s_page++;
            if (s_in_page != 0U)
            {
                RefreshPage(1);  /* 翻页后全量刷新 */
            }
        }
    }
}
