/*==============================================================
 * 文件名称   : bsp_fire_zone.c
 * 模块功能   : 防火分区管理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 提供设备防火分区归属的管理功能。
 *              - 设备分区映射: (回路号,设备地址) -> 分区号(1-2)
 *              - 分区名称管理(GBK)
 *              - 分区启用状态(V2新增): 每分区启用/停用存储与显示
 *              - Flash持久化(基于W25Qxx SPI Flash, A/B双副本)
 *              - 分区变更回调通知(解耦)
 *
 * 架构定位   : MODEL层，数据自包含
 *              不依赖设备注册表/联动模块，分区变更通过回调
 *              通知外部，便于后续扩展与修改。
 *
 * 依赖模块   :
 *   - w25qxx.h        : Flash读/写/擦除
 *   - system.h        : CalcCrc16()
 *   - bsp_debug.h     : DebugPrintf: test trace on UART4
 *==============================================================*/

#include "bsp_fire_zone.h"      /* 本模块头文件 */
#include "w25qxx.h"             /* W25QXX_Read/Write/Erase_Sector */
#include "system.h"             /* CalcCrc16() */
#include "bsp_debug.h"          /* DebugPrintf: test trace on UART4 */
#include "bsp_mbus.h"           /* MIXTURE_DEVICE_MAX_ADDR: 回路1地址上限100 */
#include "bsp_mbus_control.h"   /* MBUS_CONTROL_MAX_DEVICES: 回路2设备数64(地址1~63) */
#include "bsp_rs485_detect.h"   /* RS485_DETECT_MAX_DEVICES: 回路3设备数65(地址1~64) */
#include "bsp_ig3302.h"         /* IG3302_MAX_ADDRESS: 回路4地址上限10 */
#include <string.h>             /* memset/memcpy/strncpy */
#include <stdio.h>              /* sprintf生成默认分区名 */

/*--------------------------------------------------------------
 * 第一部分：内部数据结构与静态变量
 *--------------------------------------------------------------*/

/* Flash存储记录结构 - packed紧凑布局, 共725字节(V5)
 * 布局 = 头部(魔数+版本+保留) + 分区映射(404B, 地址0~100) + 分区名称(32B)
 *        + 分区启用(2B) + 槽位显示文本(224B, 2分区x7槽位) + 分区显示盘地址(2B)
 *        + 系统级设备显示文本(48B) + 系统级设备地址(3B) + CRC16
 * V4变更: 分区数8->2, 映射第二维64->101, 新增分区显示盘地址与系统级设备;
 *         旧版本(V1/V2/V3)记录布局与新结构不兼容, 加载一律作废回出厂默认
 * 存储时同一份记录写入扇区前半(副本A)与后半(副本B)两个位置(各2KB, 725B可容纳) */
typedef struct
{
    uint32_t magic;                                         /* 魔数 FIRE_ZONE_MAGIC "FRZ2" */
    uint8_t  version;                                       /* 存储格式版本号 */
    uint8_t  reserved[3];                                   /* 保留对齐填充 */
    uint8_t  zone_map[4][101];                              /* 设备分区映射, [回路号-1][设备地址0-100], 值0-1对应分区1-2 */
    char     zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];  /* 分区名称(GBK), 下标0-1对应分区1-2 */
    uint8_t  zone_enabled[FIRE_ZONE_MAX];                   /* 分区启用状态, 1=启用 0=停用, 下标0-1对应分区1-2 */
    char     zone_disp[FIRE_ZONE_MAX][FZ_SLOT_COUNT][FIRE_ZONE_NAME_LEN]; /* 槽位显示文本, [分区][7类设备栏][GBK串] */
    uint8_t  zone_display_addr[FIRE_ZONE_MAX];              /* V4新增: 各分区关联的火灾显示盘地址(回路2, 0=未配置) */
    char     sys_disp[FZ_SYS_COUNT][FIRE_ZONE_NAME_LEN];    /* V4新增: 系统级设备显示文本 */
    uint8_t  sys_addr[FZ_SYS_COUNT];                        /* V4新增: 系统级设备地址(回路2, 0=未配置) */
    uint16_t crc;                                           /* CRC16校验和, 覆盖除crc字段外全部字节 */
} __attribute__((packed)) FireZoneRecord_t;

/* 编译期长度校验: 记录布局一旦与Flash存储格式不一致(增删成员/改分区数/改槽位数),
 * 此处立即编译失败, 避免运行期CRC覆盖长度与记录偏移错位(C90兼容写法) */
typedef char FireZoneRecordSizeCheck_t[(sizeof(FireZoneRecord_t) == 725U) ? 1 : -1];

/* 设备分区映射表: s_zone_map[回路号-1][设备地址]
 * 第二维为设备地址0~100, 按最大回路地址(回路1上限100)取101;
 * 值0-1对应分区1-2, 默认全0 = 全部设备属于分区1 */
static uint8_t s_zone_map[4][101];

/* 分区名称表(GBK编码): 对外分区号1-2对应下标0-1 */
static char s_zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];

/* 分区启用状态表: 下标0-1对应分区1-2, 1=启用 0=停用 */
static uint8_t s_zone_enabled[FIRE_ZONE_MAX];

/* 槽位显示文本表: [分区0-1][FZ_SLOT_COUNT类设备栏], 屏端输入的规范化显示文本 */
static char s_zone_disp[FIRE_ZONE_MAX][FZ_SLOT_COUNT][FIRE_ZONE_NAME_LEN];

/* 分区关联火灾显示盘地址(V4新增): 下标0-1对应分区1-2, 0=未配置 */
static uint8_t s_zone_display_addr[FIRE_ZONE_MAX];

/* 系统级设备显示文本/地址(V4新增): 下标对应 FireZoneSysDev */
static char    s_sys_disp[FZ_SYS_COUNT][FIRE_ZONE_NAME_LEN];
static uint8_t s_sys_addr[FZ_SYS_COUNT];

/* 批量写挂起计数(支持嵌套): >0时修改只置脏不落盘 */
static uint8_t s_batch_depth = 0;
/* 批量挂起期间有改动的脏标志: EndBatch最外层据此统一落盘一次 */
static uint8_t s_batch_dirty = 0;

/* 模块初始化标志：0=未初始化, 1=已初始化 */
static uint8_t s_initialized = 0;

/* 分区变更通知回调函数，由外部模块(屏幕层)注册
 * 若为NULL则不通知 */
static FireZoneChangedCb_t s_changed_cb = NULL;

/*--------------------------------------------------------------
 * 第二部分：内部辅助函数
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_LoopMaxAddr
 * 功能描述：取指定回路的设备地址上限(与联动规则校验保持一致)
 * 说明     ：上限直接取自各BSP模块宏, 避免本模块与BSP口径不一致
 * 输入参数：loop_no - 回路号(1-4)
 * 返回值   ：该回路最大设备地址; 回路号非法返回0
 *--------------------------------------------------------------*/
static uint16_t FireZone_LoopMaxAddr(uint8_t loop_no)
{
    switch (loop_no)
    {
    case 1: return (uint16_t)MIXTURE_DEVICE_MAX_ADDR;                /* 100 */
    case 2: return (uint16_t)(MBUS_CONTROL_MAX_DEVICES - 1U);        /* 63  */
    case 3: return (uint16_t)(RS485_DETECT_MAX_DEVICES - 1U);        /* 64  */
    case 4: return (uint16_t)IG3302_MAX_ADDRESS;                     /* 10  */
    default: return 0U;
    }
}

/*--------------------------------------------------------------
 * 函数名称：IsValidDevice
 * 功能描述：校验设备标识(回路号+设备地址)是否在有效范围内
 * 地址范围与联动规则校验一致, 各回路上限见 FireZone_LoopMaxAddr:
 *   回路1(MBus探测器):   设备地址1-100
 *   回路2(MBus控制回路): 设备地址1-63
 *   回路3(RS485探测器):  设备地址1-64
 *   回路4(IG3302回路):   设备地址1-10
 * 输入参数：loop_no - 回路号(1-4), dev_no - 设备地址
 * 返回值   ：1=有效, 0=无效
 *--------------------------------------------------------------*/
static uint8_t IsValidDevice(uint8_t loop_no, uint16_t dev_no)
{
    uint16_t dmax;  /* 该回路设备地址上限 */

    dmax = FireZone_LoopMaxAddr(loop_no);
    if (dmax == 0U)
    {
        return 0;  /* 回路号非法 */
    }
    if (dev_no < 1U || dev_no > dmax)
    {
        return 0;  /* 设备地址非法 */
    }

    return 1;  /* 设备标识有效 */
}

/*--------------------------------------------------------------
 * 函数名称：CalcRecordCRC
 * 功能描述：计算Flash存储记录的CRC16校验和
 * CRC计算时排除最后2字节（crc字段自身），避免自引用
 * 输入参数：rec - 待计算的存储记录
 * 返回值   ：CRC16值
 *--------------------------------------------------------------*/
static uint16_t CalcRecordCRC(const FireZoneRecord_t *rec)
{
    /* sizeof(FireZoneRecord_t)=725字节(V5), crc字段占最后2字节, 实际计算723字节 */
    return CalcCrc16((uint8_t *)rec, sizeof(FireZoneRecord_t) - 2U);
}

/*--------------------------------------------------------------
 * 函数名称：NotifyChanged
 * 功能描述：触发分区变更通知回调（解耦：本模块不直接调用
 *           任何外部业务函数，仅通过回调通知）
 *--------------------------------------------------------------*/
static void NotifyChanged(void)
{
    if (s_changed_cb != NULL)
    {
        s_changed_cb();  /* 通知外部模块分区数据已变化 */
    }
}

/* 前向声明：PersistChanges()在其定义之前调用本函数,
 * 若无此声明, C90下未声明调用会被当作外部符号, 链接时报L6218E未定义 */
static void SaveRecordToFlash(void);

/*--------------------------------------------------------------
 * 函数名称：PersistChanges
 * 功能描述：统一持久化入口(V2新增)
 *           批量挂起期间(s_batch_depth>0)只置脏标志不落盘,
 *           由FireZone_EndBatch在最外层统一落盘一次;
 *           非挂起状态立即落盘并通知(保持原有单设备写行为)
 *--------------------------------------------------------------*/
static void PersistChanges(void)
{
    if (s_batch_depth > 0U)
    {
        s_batch_dirty = 1;  /* 批量挂起中, 仅置脏标志 */
    }
    else
    {
        SaveRecordToFlash();  /* 立即持久化(A/B双副本) */
        NotifyChanged();      /* 通知外部(屏幕刷新等) */
    }
}

/*--------------------------------------------------------------
 * 函数名称：SetDefaultAll
 * 功能描述：恢复出厂默认分区数据
 *           - 全部设备属于分区1（映射表全0）
 *           - 分区名称默认为"分区1"~"分区2"
 *--------------------------------------------------------------*/
static void SetDefaultAll(void)
{
    uint8_t i;              /* 分区序号循环 */
    char name[FIRE_ZONE_NAME_LEN];  /* 默认名称临时缓冲区 */

    /* 映射表清零: 全部设备为"未做分区"(0), 必须在画面84配置后才参与分区功能 */
    memset(s_zone_map, 0, sizeof(s_zone_map));

    /* 逐个生成默认分区名称"分区1"~"分区2" */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        sprintf(name, "分区%u", (unsigned int)(i + 1U));
        strncpy(s_zone_name[i], name, FIRE_ZONE_NAME_LEN - 1U);
        s_zone_name[i][FIRE_ZONE_NAME_LEN - 1U] = '\0';  /* 保证以结束符结尾 */
    }

    /* 分区启用状态默认全部启用(V5: 两个分区出厂均启用) */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        s_zone_enabled[i] = 1U;
    }

    /* 槽位显示文本/分区显示盘地址/系统级设备默认全部为空(V3/V4新增字段) */
    memset(s_zone_disp, 0, sizeof(s_zone_disp));
    memset(s_zone_display_addr, 0, sizeof(s_zone_display_addr));
    memset(s_sys_disp, 0, sizeof(s_sys_disp));
    memset(s_sys_addr, 0, sizeof(s_sys_addr));
}

/*--------------------------------------------------------------
 * 函数名称：ApplyRecord
 * 功能描述：将Flash读取的记录应用到RAM数据区
 * 输入参数：rec - 已通过校验的存储记录
 *--------------------------------------------------------------*/
static void ApplyRecord(const FireZoneRecord_t *rec)
{
    uint8_t i;  /* 分区序号循环 */

    /* 拷贝设备分区映射与分区名称 */
    memcpy(s_zone_map, rec->zone_map, sizeof(s_zone_map));
    memcpy(s_zone_name, rec->zone_name, sizeof(s_zone_name));

    /* 拷贝分区启用状态, 归一化为0/1防御Flash异常数据 */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        s_zone_enabled[i] = (rec->zone_enabled[i] != 0U) ? 1U : 0U;
    }

    /* 槽位显示文本(7类设备栏)、分区显示盘地址、系统级设备显示文本与地址:
     * V4记录与结构体一一对应, 逐项拷贝 */
    memcpy(s_zone_disp, rec->zone_disp, sizeof(s_zone_disp));
    memcpy(s_zone_display_addr, rec->zone_display_addr, sizeof(s_zone_display_addr));
    memcpy(s_sys_disp, rec->sys_disp, sizeof(s_sys_disp));
    memcpy(s_sys_addr, rec->sys_addr, sizeof(s_sys_addr));

    /* 防御处理: 强制每个名称以结束符结尾, 防止Flash数据异常导致越界 */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        s_zone_name[i][FIRE_ZONE_NAME_LEN - 1U] = '\0';
    }
}

/*--------------------------------------------------------------
 * 函数名称：LoadRecord
 * 功能描述：从Flash指定地址读取存储记录并校验
 * 校验项：魔数、版本号、CRC16
 * V4说明：分区数由8降到2、记录布局变更, 旧版本(V1/V2/V3)记录
 *         与新结构不兼容, 一律作废并回出厂默认(不再做旧版兼容读取)
 * 输入参数：addr - Flash读取地址(副本A或B)
 *           rec  - 输出读取到的记录
 * 返回值   ：1=校验通过, 0=记录无效
 *--------------------------------------------------------------*/
static uint8_t LoadRecord(uint32_t addr, FireZoneRecord_t *rec)
{
    /* 读取整条记录(sizeof(FireZoneRecord_t)=725字节) */
    W25QXX_Read((uint8_t *)rec, addr, sizeof(FireZoneRecord_t));

    /* 魔数校验 */
    if (rec->magic != FIRE_ZONE_MAGIC)
    {
        return 0;  /* 魔数不符(空白扇区或数据损坏) */
    }

    /* 版本校验: 仅当前版本(V5: 0=未分区/1=分区1/2=分区2)有效, 旧版本记录作废 */
    if (rec->version != FIRE_ZONE_VERSION)
    {
        return 0;  /* 旧版本/未知版本, 走默认流程 */
    }

    /* CRC覆盖除crc字段外全部字节(723字节) */
    if (rec->crc != CalcRecordCRC(rec))
    {
        return 0;  /* 校验失败, 数据损坏 */
    }

    return 1;  /* 记录有效 */
}

/*--------------------------------------------------------------
 * 函数名称：SaveRecordToFlash
 * 功能描述：将当前RAM分区数据持久化到Flash
 *           擦除整个扇区后, 同一记录写入副本A(前半)与副本B(后半)
 * 注意：Flash擦写为低频操作(仅用户修改分区时触发)
 *--------------------------------------------------------------*/
static void SaveRecordToFlash(void)
{
    /* [2026-09-18 修复] rec改为静态: 本函数在InterScreenUpdataTask任务
     * (栈仅1536字节)内经FlushPendingSave/EndBatch调用, 725字节的任务栈
     * 局部变量必然溢出任务栈导致内存破坏/复位, 实测表现为"画面84按返回
     * 保存后重新进入分区数据全部丢失" */
    static FireZoneRecord_t rec;         /* 待写入的存储记录(移出任务栈) */
    static FireZoneRecord_t verify_buf;  /* 回读校验缓冲(同样避免占任务栈) */

    /* 组装存储记录 */
    rec.magic   = FIRE_ZONE_MAGIC;
    rec.version = FIRE_ZONE_VERSION;
    memset(rec.reserved, 0, sizeof(rec.reserved));
    memcpy(rec.zone_map, s_zone_map, sizeof(s_zone_map));
    memcpy(rec.zone_name, s_zone_name, sizeof(s_zone_name));
    memcpy(rec.zone_enabled, s_zone_enabled, sizeof(s_zone_enabled));  /* V2新增: 分区启用状态 */
    memcpy(rec.zone_disp, s_zone_disp, sizeof(s_zone_disp));                     /* 槽位显示文本(7类设备栏) */
    memcpy(rec.zone_display_addr, s_zone_display_addr, sizeof(s_zone_display_addr)); /* V4新增: 分区显示盘地址 */
    memcpy(rec.sys_disp, s_sys_disp, sizeof(s_sys_disp));                       /* V4新增: 系统级设备显示文本 */
    memcpy(rec.sys_addr, s_sys_addr, sizeof(s_sys_addr));                       /* V4新增: 系统级设备地址 */
    rec.crc = CalcRecordCRC(&rec);

    /* 擦除整个4KB扇区(副本A/B同扇区, 只需擦一次)
     * 注意: 驱动W25QXX_Erase_Sector入参为扇区号(内部x4096), 须传字节地址/4096 */
    W25QXX_Erase_Sector(FIRE_ZONE_FLASH_ADDR / 4096U);

    /* 写入副本A(扇区前半)与副本B(扇区后半): 扇区已整擦为0xFF,
     * 用BspFlashWrite免检直写(带事务锁, 与bsp_save_ctrl用法一致) */
    BspFlashWrite((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_A, sizeof(FireZoneRecord_t));
    BspFlashWrite((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_B, sizeof(FireZoneRecord_t));

    /* 回读校验副本A(魔数+CRC), 结果打UART4便于实机确认保存是否成功 */
    W25QXX_Read((uint8_t *)&verify_buf, FIRE_ZONE_FLASH_COPY_A, sizeof(verify_buf));
    if ((verify_buf.magic == FIRE_ZONE_MAGIC) &&
        (verify_buf.crc == CalcRecordCRC(&verify_buf)))
    {
        DebugPrintf("[FIREZONE] save verify OK\r\n");
    }
    else
    {
        DebugPrintf("[FIREZONE] save verify FAIL!\r\n");
    }
}

/*--------------------------------------------------------------
 * 第三部分：API实现 - 初始化与回调注册
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_Init
 * 功能描述：模块初始化，从Flash加载分区表
 * 加载策略：优先副本A，A损坏则尝试副本B并回写修复A；
 *           两个副本均无效时保持出厂默认(全部设备在分区1)，
 *           不立即写Flash，待下次用户修改时自然覆盖。
 *--------------------------------------------------------------*/
void FireZone_Init(void)
{
    FireZoneRecord_t rec;  /* Flash读取缓冲 */
    uint8_t loaded = 0;    /* 是否成功从Flash加载 */
    uint8_t need_save = 0; /* 是否需要在初始化末尾回写Flash */

    /* 防止重复初始化 */
    if (s_initialized)
    {
        return;
    }

    /* 先恢复出厂默认(映射全0=全在分区1 + 默认分区名) */
    SetDefaultAll();

    /* 优先尝试副本A */
    if (LoadRecord(FIRE_ZONE_FLASH_COPY_A, &rec))
    {
        ApplyRecord(&rec);
        loaded = 1;  /* 副本A有效 */
    }
    else if (LoadRecord(FIRE_ZONE_FLASH_COPY_B, &rec))
    {
        /* 副本A损坏但副本B有效: 应用B并回写修复副本A */
        ApplyRecord(&rec);
        need_save = 1;  /* 延迟到末尾统一落盘(修复副本A) */
        loaded = 1;
        DebugPrintf("[FIREZONE] copy A bad, restored from copy B\r\n");
    }
    /* 两个副本均无效: 保持出厂默认, 不立即写Flash */

    /* 旧版本记录不再兼容: 分区数与记录布局均变更, LoadRecord已拒绝,
     * 此处保持出厂默认, 待首次用户修改时自然落盘 */

    /* 统一回写: 副本A损坏时用副本B数据修复 */
    if (need_save)
    {
        SaveRecordToFlash();
    }

    s_initialized = 1;

    DebugPrintf("[FIREZONE] init %s, zone count=%u\r\n",
                loaded ? "loaded" : "default",
                (unsigned int)FireZone_GetZoneCount());
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetSlotDisp
 * 功能描述：设置指定分区指定设备栏的显示文本(V3新增)。
 *           显示文本与分区表一并持久化, 上电自动恢复(免重设)。
 *           与分区归属写入配对使用时置于BeginBatch/EndBatch之间,
 *           合并为单次Flash擦写。
 * 输入参数：zone_no - 分区号(1-2), slot - 设备栏(FZ_SLOT_*, 对应屏幕
 *           温度/烟雾/可燃气体/复合/手报/声光/火灾显示盘),
 *           text - GBK显示串(超长截断到15字节)
 * 返回值   ：0=成功, 1=参数非法
 *--------------------------------------------------------------*/
uint8_t FireZone_SetSlotDisp(uint8_t zone_no, uint8_t slot, const char *text)
{
    char *dst;  /* 目标槽位 */

    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX) ||
        (slot >= (uint8_t)FZ_SLOT_COUNT) || (text == NULL))
    {
        return 1;  /* 参数非法 */
    }
    dst = s_zone_disp[zone_no - 1U][slot];
    if (strncmp(dst, text, FIRE_ZONE_NAME_LEN) == 0)
    {
        return 0;  /* 内容未变化, 不落盘 */
    }
    strncpy(dst, text, FIRE_ZONE_NAME_LEN - 1U);
    dst[FIRE_ZONE_NAME_LEN - 1U] = '\0';
    PersistChanges();  /* 落盘+通知 */
    return 0;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetSlotDisp
 * 功能描述：读取指定分区指定设备栏的显示文本(V3新增)。
 * 输入参数：zone_no - 分区号(1-2), slot - 设备栏(FZ_SLOT_*),
 *           out - 输出缓冲(容量>=FIRE_ZONE_NAME_LEN)
 *--------------------------------------------------------------*/
void FireZone_GetSlotDisp(uint8_t zone_no, uint8_t slot, char *out)
{
    if (out == NULL)
    {
        return;
    }
    out[0] = '\0';
    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX) || (slot >= (uint8_t)FZ_SLOT_COUNT))
    {
        return;
    }
    memcpy(out, s_zone_disp[zone_no - 1U][slot], FIRE_ZONE_NAME_LEN);
    out[FIRE_ZONE_NAME_LEN - 1U] = '\0';
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetZoneDisplayAddr
 * 功能描述：设置分区关联的火灾显示盘地址(V4新增)
 *           地址位于回路2(1-63), 0表示未配置; 内容无变化不擦写Flash
 * 输入参数：zone_no - 分区号(1-2), addr - 显示盘地址(0-63, 0=未配置)
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_SetZoneDisplayAddr(uint8_t zone_no, uint8_t addr)
{
    /* 分区号与地址范围校验(回路2地址1-63, 0表示未配置) */
    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX) || (addr > 63U))
    {
        return 2;  /* 参数非法 */
    }

    /* 内容未变化则不擦写Flash */
    if (s_zone_display_addr[zone_no - 1U] == addr)
    {
        return 0;
    }

    s_zone_display_addr[zone_no - 1U] = addr;
    PersistChanges();  /* 持久化并通知(批量挂起期间只置脏) */
    return 0;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneDisplayAddr
 * 功能描述：读取分区关联的火灾显示盘地址(V4新增)
 * 输入参数：zone_no - 分区号(1-2)
 * 返回值   ：显示盘地址(0=未配置), 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneDisplayAddr(uint8_t zone_no)
{
    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX))
    {
        return 0;  /* 参数非法 */
    }

    return s_zone_display_addr[zone_no - 1U];
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetSysDeviceAddr
 * 功能描述：设置系统级设备地址(V4新增)
 *           地址位于回路2(1-63), 0表示未配置; 内容无变化不擦写Flash
 * 输入参数：kind - 设备类别(FireZoneSysDev 0-2), addr - 设备地址(0-63)
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_SetSysDeviceAddr(uint8_t kind, uint8_t addr)
{
    /* 类别与地址范围校验(回路2地址1-63, 0表示未配置) */
    if ((kind >= (uint8_t)FZ_SYS_COUNT) || (addr > 63U))
    {
        return 2;  /* 参数非法 */
    }

    /* 内容未变化则不擦写Flash */
    if (s_sys_addr[kind] == addr)
    {
        return 0;
    }

    s_sys_addr[kind] = addr;
    PersistChanges();  /* 持久化并通知(批量挂起期间只置脏) */
    return 0;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetSysDeviceAddr
 * 功能描述：读取系统级设备地址(V4新增)
 * 输入参数：kind - 设备类别(FireZoneSysDev 0-2)
 * 返回值   ：设备地址(0=未配置), 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t FireZone_GetSysDeviceAddr(uint8_t kind)
{
    if (kind >= (uint8_t)FZ_SYS_COUNT)
    {
        return 0;  /* 参数非法 */
    }

    return s_sys_addr[kind];
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetSysDisp
 * 功能描述：设置系统级设备显示文本(V4新增), 原样保存用户输入
 *           与分区表一并持久化, 上电自动恢复(免重设)
 * 输入参数：kind - 设备类别(FireZoneSysDev 0-2)
 *           text - GBK显示串(超长截断到15字节)
 * 返回值   ：0=成功, 1=参数非法
 *--------------------------------------------------------------*/
uint8_t FireZone_SetSysDisp(uint8_t kind, const char *text)
{
    if ((kind >= (uint8_t)FZ_SYS_COUNT) || (text == NULL))
    {
        return 1;  /* 参数非法 */
    }

    /* 内容未变化则不擦写Flash */
    if (strncmp(s_sys_disp[kind], text, FIRE_ZONE_NAME_LEN) == 0)
    {
        return 0;
    }

    strncpy(s_sys_disp[kind], text, FIRE_ZONE_NAME_LEN - 1U);
    s_sys_disp[kind][FIRE_ZONE_NAME_LEN - 1U] = '\0';  /* 强置结束符 */
    PersistChanges();  /* 持久化并通知(批量挂起期间只置脏) */
    return 0;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetSysDisp
 * 功能描述：读取系统级设备显示文本(V4新增)
 * 输入参数：kind - 设备类别(FireZoneSysDev 0-2)
 *           out  - 输出缓冲(容量>=FIRE_ZONE_NAME_LEN)
 *--------------------------------------------------------------*/
void FireZone_GetSysDisp(uint8_t kind, char *out)
{
    if (out == NULL)
    {
        return;
    }
    out[0] = '\0';

    if (kind >= (uint8_t)FZ_SYS_COUNT)
    {
        return;  /* 参数非法, 返回空串 */
    }

    memcpy(out, s_sys_disp[kind], FIRE_ZONE_NAME_LEN);
    out[FIRE_ZONE_NAME_LEN - 1U] = '\0';
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_ResetAll
 * 功能描述：清空全部分区信息并恢复出厂默认, 立即写Flash(V3新增)。
 *           - 设备分区映射: 全部设备改为"未做分区"(0)
 *           - 分区名称: 恢复默认"分区1"~"分区2"
 *           - 分区启用状态: 全部启用(V4出厂默认)
 *           - 槽位显示文本/分区显示盘地址/系统级设备: 全部清空
 *           用于调试排查或需要强制清空全部配置的场合。
 *           调用后屏幕层经变更回调自动刷新显示。
 *--------------------------------------------------------------*/
void FireZone_ResetAll(void)
{
    SetDefaultAll();     /* 恢复出厂默认(映射/名称/启用/显示文本) */
    SaveRecordToFlash(); /* 立即落盘(A/B双副本) */
    NotifyChanged();     /* 通知屏幕层刷新 */
    DebugPrintf("[FIREZONE] reset all to factory default\r\n");
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetChangedCallback
 * 功能描述：注册分区变更通知回调函数
 * 输入参数：cb - 回调函数指针, 传NULL取消注册
 *--------------------------------------------------------------*/
void FireZone_SetChangedCallback(FireZoneChangedCb_t cb)
{
    s_changed_cb = cb;  /* 保存回调指针, 变更时由NotifyChanged调用 */
}

/*--------------------------------------------------------------
 * 第四部分：API实现 - 设备分区归属查询与设置
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetDeviceZone
 * 功能描述：查询设备所属防火分区号
 * 输入参数：loop_no - 回路号, dev_no - 设备地址
 * 返回值   ：分区号(1-2), 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t FireZone_GetDeviceZone(uint8_t loop_no, uint16_t dev_no)
{
    /* 参数校验 */
    if (!IsValidDevice(loop_no, dev_no))
    {
        return 0;  /* 参数非法 */
    }

    /* 直接返回存储值: 0=未做分区, 1=分区1, 2=分区2 */
    return s_zone_map[loop_no - 1U][dev_no];
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetDeviceZone
 * 功能描述：设置设备所属防火分区（设置后立即持久化并通知）
 * 输入参数：loop_no - 回路号, dev_no - 设备地址
 *           zone_no - 目标分区号(1-2)
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_SetDeviceZone(uint8_t loop_no, uint16_t dev_no, uint8_t zone_no)
{
    /* 设备标识校验 */
    if (!IsValidDevice(loop_no, dev_no))
    {
        return 2;  /* 设备参数非法 */
    }

    /* 分区号校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 2;  /* 分区号非法 */
    }

    /* 归属未变化则不擦写Flash(延长Flash寿命) */
    if (s_zone_map[loop_no - 1U][dev_no] == zone_no)
    {
        return 0;  /* 无变化视为成功 */
    }

    /* 更新RAM映射: 直接存分区号(1/2) */
    s_zone_map[loop_no - 1U][dev_no] = zone_no;

    /* 持久化并通知(批量挂起期间只置脏) */
    PersistChanges();

    DebugPrintf("[FIREZONE] device L%u-%u -> zone %u\r\n",
                (unsigned int)loop_no, (unsigned int)dev_no, (unsigned int)zone_no);

    return 0;  /* 设置成功 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_IsSameZone
 * 功能描述：判断两个设备是否属于同一防火分区
 *           供联动规则设定时做同分区校验使用
 * 输入参数：两设备的回路号与设备地址
 * 返回值   ：1=同区(含任一参数非法, 交由原有规则校验处理)
 *            0=跨区(联动校验应拒绝)
 *--------------------------------------------------------------*/
uint8_t FireZone_IsSameZone(uint8_t loop1, uint16_t dev1, uint8_t loop2, uint16_t dev2)
{
    /* 任一设备标识非法时不做拦截, 交由原有联动规则合法性校验处理 */
    if (!IsValidDevice(loop1, dev1) || !IsValidDevice(loop2, dev2))
    {
        return 1;
    }

    /* 比较两设备的内部分区值 */
    if (s_zone_map[loop1 - 1U][dev1] == s_zone_map[loop2 - 1U][dev2])
    {
        return 1;  /* 同一分区 */
    }

    return 0;  /* 跨分区 */
}

/*--------------------------------------------------------------
 * 第五部分：API实现 - 分区设备枚举
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneDeviceCount
 * 功能描述：统计分区内设备(地址)总数
 *           说明: 本模块不依赖设备注册表, 统计的是映射表内
 *           属于该分区的全部地址, 是否为实际接入设备由调用方过滤
 * 输入参数：zone_no - 分区号(1-2)
 * 返回值   ：设备地址总数, 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneDeviceCount(uint8_t zone_no)
{
    uint8_t l;             /* 回路号循环 */
    uint16_t d;            /* 设备地址循环 */
    uint8_t count = 0;     /* 计数器 */

    /* 分区号校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 0;
    }

    /* 按回路1→2→3顺序统计(回路4为IG3302回路, 暂不纳入分区枚举) */
    for (l = 1U; l <= 3U; l++)
    {
        /* 各回路地址上限: 回路1=100, 回路2=63, 回路3=64(取自BSP宏) */
        uint16_t dmax = FireZone_LoopMaxAddr(l);
        for (d = 1U; d <= dmax; d++)
        {
            if (s_zone_map[l - 1U][d] == zone_no)
            {
                count++;  /* 该地址属于目标分区 */
            }
        }
    }

    return count;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneDeviceAt
 * 功能描述：取分区内第index个设备(枚举顺序: 回路1→2→3、地址升序)
 * 输入参数：zone_no - 分区号(1-2), index - 序号(0基)
 *           loop_no/dev_no - 输出设备标识
 * 返回值   ：1=成功, 0=越界或参数非法
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneDeviceAt(uint8_t zone_no, uint8_t index, uint8_t *loop_no, uint16_t *dev_no)
{
    uint8_t l;             /* 回路号循环 */
    uint16_t d;            /* 设备地址循环 */
    uint8_t cur = 0;       /* 当前枚举序号 */

    /* 参数校验: 输出指针与分区号 */
    if (loop_no == NULL || dev_no == NULL)
    {
        return 0;
    }
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 0;
    }

    /* 按回路1→2→3顺序遍历, 找第index个属于目标分区的地址 */
    for (l = 1U; l <= 3U; l++)
    {
        uint16_t dmax = FireZone_LoopMaxAddr(l);  /* 各回路地址上限 */
        for (d = 1U; d <= dmax; d++)
        {
            if (s_zone_map[l - 1U][d] == zone_no)
            {
                if (cur == index)
                {
                    *loop_no = l;        /* 输出回路号 */
                    *dev_no = d;         /* 输出设备地址 */
                    return 1;            /* 找到目标序号设备 */
                }
                cur++;  /* 序号递增 */
            }
        }
    }

    return 0;  /* index超出分区内设备数 */
}

/*--------------------------------------------------------------
 * 第六部分：API实现 - 分区数量与名称
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneCount
 * 功能描述：获取当前已被占用的最高分区号
 *           出厂默认全部设备"未做分区"(0), 此时返回0
 * 返回值   ：0=没有任何设备划区, 1=仅分区1有设备, 2=两区都有设备
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneCount(void)
{
    uint8_t l;             /* 回路号循环 */
    uint16_t d;            /* 设备地址循环 */
    uint8_t max_zone = 0U;  /* 已被占用的最高分区号(0=未分区) */

    /* 遍历全部映射, 找最高被占用的分区 */
    for (l = 0; l < 4U; l++)
    {
        for (d = 0; d < 101U; d++)  /* 映射第二维为设备地址0~100 */
        {
            if (s_zone_map[l][d] > max_zone)
            {
                max_zone = s_zone_map[l][d];  /* 更新最高占用分区 */
            }
        }
    }

    return max_zone;  /* 0=没有设备划区 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneName
 * 功能描述：获取分区名称(GBK字符串)
 * 输入参数：zone_no - 分区号(1-2)
 * 返回值   ：名称字符串指针, 参数非法返回空串
 *--------------------------------------------------------------*/
const char* FireZone_GetZoneName(uint8_t zone_no)
{
    /* 参数非法返回空串, 避免调用方判空 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return "";
    }

    return s_zone_name[zone_no - 1U];
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetZoneName
 * 功能描述：设置分区名称(GBK字符串, 超15字节截断)
 *           设置后立即持久化并通知
 * 输入参数：zone_no - 分区号(1-2), name - 名称字符串
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_SetZoneName(uint8_t zone_no, const char *name)
{
    /* 参数校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX || name == NULL)
    {
        return 2;
    }

    /* 拷贝名称并保证以结束符结尾 */
    strncpy(s_zone_name[zone_no - 1U], name, FIRE_ZONE_NAME_LEN - 1U);
    s_zone_name[zone_no - 1U][FIRE_ZONE_NAME_LEN - 1U] = '\0';

    /* 持久化并通知(批量挂起期间只置脏) */
    PersistChanges();

    return 0;  /* 设置成功 */
}

/*--------------------------------------------------------------
 * 第七部分：API实现 - 分区启用状态(V2新增)
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneEnabled
 * 功能描述：查询分区启用状态
 *           V2.2脉络: 启用状态仅存储与显示, 不参与联动求值
 * 输入参数：zone_no - 分区号(1-2)
 * 返回值   ：1=启用, 0=停用; 参数非法返回1(默认启用)
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneEnabled(uint8_t zone_no)
{
    /* 参数非法按启用处理, 避免误停 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 1;
    }

    return (s_zone_enabled[zone_no - 1U] != 0U) ? 1U : 0U;
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetZoneEnabled
 * 功能描述：设置分区启用状态(非0视为启用)
 *           状态无变化时不擦写Flash(延长Flash寿命)
 * 输入参数：zone_no - 分区号(1-2), on - 1=启用 0=停用
 *--------------------------------------------------------------*/
void FireZone_SetZoneEnabled(uint8_t zone_no, uint8_t on)
{
    /* 参数校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return;
    }

    on = (on != 0U) ? 1U : 0U;  /* 归一化为0/1 */

    /* 无变化不落盘 */
    if (s_zone_enabled[zone_no - 1U] == on)
    {
        return;
    }

    s_zone_enabled[zone_no - 1U] = on;

    /* 持久化并通知(批量挂起期间只置脏) */
    PersistChanges();

    DebugPrintf("[FIREZONE] zone %u enabled=%u\r\n",
                (unsigned int)zone_no, (unsigned int)on);
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_ToggleZoneEnabled
 * 功能描述：翻转分区启用状态(供画面启用/停用按钮调用)
 * 输入参数：zone_no - 分区号(1-2)
 * 返回值   ：翻转后的状态(1=启用 0=停用; 参数非法返回1且不翻转)
 *--------------------------------------------------------------*/
uint8_t FireZone_ToggleZoneEnabled(uint8_t zone_no)
{
    /* 参数校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 1;
    }

    s_zone_enabled[zone_no - 1U] ^= 1U;  /* 0/1翻转 */

    /* 持久化并通知(批量挂起期间只置脏) */
    PersistChanges();

    DebugPrintf("[FIREZONE] zone %u toggle -> %u\r\n",
                (unsigned int)zone_no, (unsigned int)s_zone_enabled[zone_no - 1U]);

    return s_zone_enabled[zone_no - 1U];
}

/*--------------------------------------------------------------
 * 第八部分：API实现 - 批量设备范围写(V2新增)
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FireZone_BeginBatch
 * 功能描述：挂起自动持久化(支持嵌套调用)
 *           挂起期间所有修改只更新内存并置脏标志, 不擦写Flash
 *--------------------------------------------------------------*/
void FireZone_BeginBatch(void)
{
    s_batch_depth++;  /* 嵌套计数递增 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_EndBatch
 * 功能描述：恢复自动持久化; 最外层结束时若挂起期间有改动,
 *           统一落盘一次并通知(保证一次交互只擦写一次Flash)
 *--------------------------------------------------------------*/
void FireZone_EndBatch(void)
{
    /* 未配对调用, 防御处理 */
    if (s_batch_depth == 0U)
    {
        return;
    }

    s_batch_depth--;  /* 嵌套计数递减 */

    /* 最外层结束且挂起期间有改动: 统一落盘一次 */
    if (s_batch_depth == 0U && s_batch_dirty)
    {
        s_batch_dirty = 0;
        SaveRecordToFlash();
        NotifyChanged();
        DebugPrintf("[FIREZONE] batch end, saved\r\n");
    }
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetZoneRange
 * 功能描述：将指定回路的连续地址段划入目标分区(批量写)
 *           独占语义: 地址映射单值, 设备写入新分区即自动移出旧分区
 *           内部仅落盘一次(有实际变化时), 无需Begin/EndBatch配对
 * 输入参数：zone_no - 目标分区号(1-2), loop_no - 回路号(1-4),
 *           start/end - 设备地址段(含端点, 升序)
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_SetZoneRange(uint8_t zone_no, uint8_t loop_no, uint8_t start, uint8_t end)
{
    uint16_t d;           /* 设备地址循环 */
    uint16_t dmax;        /* 回路地址上限 */
    uint8_t changed = 0;  /* 是否有实际变化 */

    /* 分区号校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX)
    {
        return 2;
    }

    /* 回路号校验 */
    if (loop_no < 1U || loop_no > 4U)
    {
        return 2;
    }

    /* 地址段校验: 各回路地址上限见 FireZone_LoopMaxAddr */
    dmax = FireZone_LoopMaxAddr(loop_no);
    if (start < 1U || start > end || (uint16_t)end > dmax)
    {
        return 2;
    }

    /* 逐地址写入目标分区(映射单值, 旧分区归属自动被覆盖) */
    for (d = start; d <= (uint16_t)end; d++)
    {
        if (s_zone_map[loop_no - 1U][d] != zone_no)
        {
            s_zone_map[loop_no - 1U][d] = zone_no;
            changed = 1;
        }
    }

    /* 有实际变化才落盘(延长Flash寿命) */
    if (changed)
    {
        PersistChanges();
        DebugPrintf("[FIREZONE] zone %u <= L%u %u-%u\r\n",
                    (unsigned int)zone_no, (unsigned int)loop_no,
                    (unsigned int)start, (unsigned int)end);
    }

    return 0;  /* 设置成功 */
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_ClearZoneRange
 * 功能描述：清空目标分区在指定回路的全部设备(改为"未做分区")
 *           对应画面输入"00000.00000"的清空操作
 * 输入参数：zone_no - 分区号(1-2), loop_no - 回路号(1-4)
 * 返回值   ：0=成功, 2=参数错误
 *--------------------------------------------------------------*/
uint8_t FireZone_ClearZoneRange(uint8_t zone_no, uint8_t loop_no)
{
    uint16_t d;           /* 设备地址循环 */
    uint16_t dmax;        /* 回路地址上限 */
    uint8_t changed = 0;  /* 是否有实际变化 */

    /* 参数校验 */
    if (zone_no < 1U || zone_no > FIRE_ZONE_MAX || loop_no < 1U || loop_no > 4U)
    {
        return 2;
    }

    dmax = FireZone_LoopMaxAddr(loop_no);  /* 各回路地址上限(取自BSP宏) */

    /* 遍历该回路, 把属于目标分区的地址改为"未做分区"(映射值0) */
    for (d = 1U; d <= dmax; d++)
    {
        if (s_zone_map[loop_no - 1U][d] == zone_no)
        {
            s_zone_map[loop_no - 1U][d] = FIRE_ZONE_NONE;
            changed = 1;
        }
    }

    /* 有实际变化才落盘 */
    if (changed)
    {
        PersistChanges();
        DebugPrintf("[FIREZONE] zone %u clear L%u\r\n",
                    (unsigned int)zone_no, (unsigned int)loop_no);
    }

    return 0;  /* 清空成功 */
}
