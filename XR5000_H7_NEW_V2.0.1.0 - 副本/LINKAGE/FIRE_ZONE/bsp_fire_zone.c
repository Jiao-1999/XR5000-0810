/*==============================================================
 * 文件名称   : bsp_fire_zone.c
 * 模块功能   : 防火分区管理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 提供设备防火分区归属的管理功能。
 *              - 设备分区映射: (回路号,设备地址) -> 分区号(1-8)
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
#include <string.h>             /* memset/memcpy/strncpy */
#include <stdio.h>              /* sprintf生成默认分区名 */

/*--------------------------------------------------------------
 * 第一部分：内部数据结构与静态变量
 *--------------------------------------------------------------*/

/* Flash存储记录结构 - packed紧凑布局, 共1170字节(V3)
 * 布局 = 头部(魔数+版本) + 分区映射(256B) + 分区名称(128B) + 分区启用(8B)
 *        + 槽位显示文本(768B, V3新增) + CRC16
 * V2旧布局(402B)无槽位显示文本, V1旧布局(394B)无启用状态, 加载时兼容读取并升级为V3
 * 存储时同一份记录写入扇区前半(副本A)与后半(副本B)两个位置(各2KB, 1170B可容纳) */
typedef struct
{
    uint32_t magic;                                         /* 魔数 FIRE_ZONE_MAGIC "FRZ1" */
    uint8_t  version;                                       /* 存储格式版本号 */
    uint8_t  reserved[3];                                   /* 保留对齐填充 */
    uint8_t  zone_map[4][64];                               /* 设备分区映射, [回路号-1][设备地址], 值0-7对应分区1-8 */
    uint8_t  zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];  /* 分区名称(GBK), 下标0-7对应分区1-8 */
    uint8_t  zone_enabled[FIRE_ZONE_MAX];                   /* V2新增: 分区启用状态, 1=启用 0=停用, 下标0-7对应分区1-8 */
    char     zone_disp[FIRE_ZONE_MAX][6][FIRE_ZONE_NAME_LEN]; /* V3新增: 槽位显示文本, [分区][6类设备栏][GBK串] */
    uint16_t crc;                                           /* CRC16校验和, 覆盖除crc字段外全部字节 */
} __attribute__((packed)) FireZoneRecord_t;

/* 设备分区映射表: s_zone_map[回路号-1][设备地址]
 * 值0-7对应分区1-8, 默认全0 = 全部设备属于分区1 */
static uint8_t s_zone_map[4][64];

/* 分区名称表(GBK编码): 对外分区号1-8对应下标0-7 */
static char s_zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];

/* 分区启用状态表(V2新增): 下标0-7对应分区1-8, 1=启用 0=停用 */
static uint8_t s_zone_enabled[FIRE_ZONE_MAX];

/* 槽位显示文本表(V3新增): [分区0-7][6类设备栏], 屏端输入的规范化显示文本 */
static char s_zone_disp[FIRE_ZONE_MAX][6][FIRE_ZONE_NAME_LEN];

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
 * 函数名称：IsValidDevice
 * 功能描述：校验设备标识(回路号+设备地址)是否在有效范围内
 * 地址范围与联动规则校验保持一致:
 *   回路2(MBus控制回路): 设备地址1-63
 *   回路1/回路3/回路4:   设备地址1-30 (回路4为预留回路)
 * 输入参数：loop_no - 回路号(1-4), dev_no - 设备地址
 * 返回值   ：1=有效, 0=无效
 *--------------------------------------------------------------*/
static uint8_t IsValidDevice(uint8_t loop_no, uint16_t dev_no)
{
    /* 回路号有效范围1-4 */
    if (loop_no < 1U || loop_no > 4U)
    {
        return 0;  /* 回路号非法 */
    }

    if (loop_no == 2U)
    {
        /* 回路2(MBus控制回路)设备地址范围1-63 */
        if (dev_no < 1U || dev_no > 63U)
        {
            return 0;  /* 设备地址非法 */
        }
    }
    else
    {
        /* 其他回路设备地址范围1-30 */
        if (dev_no < 1U || dev_no > 30U)
        {
            return 0;  /* 设备地址非法 */
        }
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
    /* sizeof(FireZoneRecord_t)=402字节(V2), crc字段占最后2字节, 实际计算400字节 */
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
 *           - 分区名称默认为"分区1"~"分区8"
 *--------------------------------------------------------------*/
static void SetDefaultAll(void)
{
    uint8_t i;              /* 分区序号循环 */
    char name[FIRE_ZONE_NAME_LEN];  /* 默认名称临时缓冲区 */

    /* 映射表清零: 全部设备属于分区1 */
    memset(s_zone_map, 0, sizeof(s_zone_map));

    /* 逐个生成默认分区名称"分区1"~"分区8" */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        sprintf(name, "分区%u", (unsigned int)(i + 1U));
        strncpy(s_zone_name[i], name, FIRE_ZONE_NAME_LEN - 1U);
        s_zone_name[i][FIRE_ZONE_NAME_LEN - 1U] = '\0';  /* 保证以结束符结尾 */
    }

    /* 分区启用状态默认全部不启用(V2新增字段, 含分区1) */
    memset(s_zone_enabled, 0, sizeof(s_zone_enabled));

    /* 槽位显示文本默认全部为空(V3新增字段) */
    memset(s_zone_disp, 0, sizeof(s_zone_disp));
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

    /* 拷贝分区启用状态(V2字段), 归一化为0/1防御Flash异常数据
     * (V1旧记录无此字段, 此处读到的值无意义, 由Init的V1升级流程覆盖) */
    for (i = 0; i < FIRE_ZONE_MAX; i++)
    {
        s_zone_enabled[i] = (rec->zone_enabled[i] != 0U) ? 1U : 0U;
    }

    /* 槽位显示文本(V3字段): 仅V3记录有效; V1/V2旧记录该区域读到的是
     * 扇区杂散数据, 必须清空(显示文本按"无输入"处理) */
    if (rec->version == FIRE_ZONE_VERSION)
    {
        memcpy(s_zone_disp, rec->zone_disp, sizeof(s_zone_disp));
    }
    else
    {
        memset(s_zone_disp, 0, sizeof(s_zone_disp));
    }

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
 * 输入参数：addr - Flash读取地址(副本A或B)
 *           rec  - 输出读取到的记录
 * 返回值   ：1=校验通过, 0=记录无效
 *--------------------------------------------------------------*/
static uint8_t LoadRecord(uint32_t addr, FireZoneRecord_t *rec)
{
    /* 读取整条记录(394字节) */
    W25QXX_Read((uint8_t *)rec, addr, sizeof(FireZoneRecord_t));

    /* 魔数校验 */
    if (rec->magic != FIRE_ZONE_MAGIC)
    {
        return 0;  /* 魔数不符(空白扇区或数据损坏) */
    }

    /* 版本校验: V3为当前格式; V1/V2为旧格式(兼容读取, 由调用方补缺省字段)
     * 注意: 旧版本记录长度短于当前结构, 其CRC字段在当前结构视图中的
     * 偏移与覆盖长度均为固定值, 不能用sizeof(FireZoneRecord_t)推导 */
    if (rec->version == FIRE_ZONE_VERSION)
    {
        /* V3记录: CRC覆盖除crc字段外全部字节(1168字节) */
        if (rec->crc != CalcRecordCRC(rec))
        {
            return 0;  /* 校验失败, 数据损坏 */
        }
        return 1;  /* V3记录有效 */
    }

    if (rec->version == 0x02U)
    {
        /* V2旧记录(402字节): 其CRC在当前结构视图的zone_disp[0][0][0..1]
         * (偏移400), CRC覆盖前400字节(头部+映射+名称+启用)。V2无显示文本 */
        uint16_t crc_v2 = (uint16_t)(rec->zone_disp[0][0][0]) |
                          ((uint16_t)(rec->zone_disp[0][0][1]) << 8);
        if (crc_v2 == CalcCrc16((uint8_t *)rec, 400U))
        {
            return 1;  /* V2记录有效, 由调用方做兼容升级 */
        }
        return 0;  /* V2数据损坏 */
    }

    if (rec->version == 0x01U)
    {
        /* V1旧记录(394字节): 其CRC在当前结构视图的zone_enabled[0..1]
         * (偏移392), CRC覆盖前392字节(头部+映射+名称)。V1无启用/显示文本 */
        uint16_t crc_v1 = (uint16_t)(rec->zone_enabled[0]) |
                          ((uint16_t)(rec->zone_enabled[1]) << 8);
        if (crc_v1 == CalcCrc16((uint8_t *)rec, 392U))
        {
            return 1;  /* V1记录有效, 由调用方做兼容升级 */
        }
        return 0;  /* V1数据损坏 */
    }

    return 0;  /* 未知版本, 走默认流程 */

    /* CRC16完整性校验 */
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
    FireZoneRecord_t rec;  /* 待写入的存储记录 */

    /* 组装存储记录 */
    rec.magic   = FIRE_ZONE_MAGIC;
    rec.version = FIRE_ZONE_VERSION;
    memset(rec.reserved, 0, sizeof(rec.reserved));
    memcpy(rec.zone_map, s_zone_map, sizeof(s_zone_map));
    memcpy(rec.zone_name, s_zone_name, sizeof(s_zone_name));
    memcpy(rec.zone_enabled, s_zone_enabled, sizeof(s_zone_enabled));  /* V2新增: 分区启用状态 */
    memcpy(rec.zone_disp, s_zone_disp, sizeof(s_zone_disp));           /* V3新增: 槽位显示文本 */
    rec.crc = CalcRecordCRC(&rec);

    /* 擦除整个4KB扇区(副本A/B同扇区, 只需擦一次)
     * 注意: 驱动W25QXX_Erase_Sector入参为扇区号(内部x4096), 须传字节地址/4096 */
    W25QXX_Erase_Sector(FIRE_ZONE_FLASH_ADDR / 4096U);

    /* 写入副本A(扇区前半)与副本B(扇区后半): 扇区已整擦为0xFF,
     * 用BspFlashWrite免检直写(带事务锁, 与bsp_save_ctrl用法一致) */
    BspFlashWrite((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_A, sizeof(FireZoneRecord_t));
    BspFlashWrite((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_B, sizeof(FireZoneRecord_t));
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
        need_save = 1;  /* 延迟到末尾统一落盘(与V1升级合并为一次擦写) */
        loaded = 1;
        DebugPrintf("[FIREZONE] copy A bad, restored from copy B\r\n");
    }
    /* 两个副本均无效: 保持出厂默认, 不立即写Flash */

    /* 旧记录兼容升级: V1/V2统一升级为V3格式
     * V1无启用状态字段, 按新默认补0(全部不启用);
     * V1/V2无槽位显示文本字段, ApplyRecord已清空(按"无输入"处理) */
    if (loaded && (rec.version != FIRE_ZONE_VERSION))
    {
        if (rec.version == 0x01U)
        {
            uint8_t i;
            for (i = 0; i < FIRE_ZONE_MAX; i++)
            {
                s_zone_enabled[i] = 0U;  /* V1无启用字段, 按新默认填0 */
            }
        }
        need_save = 1;
        DebugPrintf("[FIREZONE] old record upgraded to v3\r\n");
    }

    /* 统一回写: 副本B修复与V1升级共用一次Flash擦写 */
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
 * 输入参数：zone_no - 分区号(1-8), slot - 设备栏(0-5, 对应屏幕
 *           温度/烟雾/可燃气体/复合/手报/声光),
 *           text - GBK显示串(超长截断到15字节)
 * 返回值   ：0=成功, 1=参数非法
 *--------------------------------------------------------------*/
uint8_t FireZone_SetSlotDisp(uint8_t zone_no, uint8_t slot, const char *text)
{
    char *dst;  /* 目标槽位 */

    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX) ||
        (slot >= 6U) || (text == NULL))
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
 * 输入参数：zone_no - 分区号(1-8), slot - 设备栏(0-5),
 *           out - 输出缓冲(容量>=FIRE_ZONE_NAME_LEN)
 *--------------------------------------------------------------*/
void FireZone_GetSlotDisp(uint8_t zone_no, uint8_t slot, char *out)
{
    if (out == NULL)
    {
        return;
    }
    out[0] = '\0';
    if ((zone_no < 1U) || (zone_no > FIRE_ZONE_MAX) || (slot >= 6U))
    {
        return;
    }
    memcpy(out, s_zone_disp[zone_no - 1U][slot], FIRE_ZONE_NAME_LEN);
    out[FIRE_ZONE_NAME_LEN - 1U] = '\0';
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_ResetAll
 * 功能描述：清空全部分区信息并恢复出厂默认, 立即写Flash(V3新增)。
 *           - 设备分区映射: 全部设备归位分区1
 *           - 分区名称: 恢复默认"分区1"~"分区8"
 *           - 分区启用状态: 全部不启用
 *           - 槽位显示文本: 全部清空
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
 * 返回值   ：分区号(1-8), 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t FireZone_GetDeviceZone(uint8_t loop_no, uint16_t dev_no)
{
    /* 参数校验 */
    if (!IsValidDevice(loop_no, dev_no))
    {
        return 0;  /* 参数非法 */
    }

    /* 内部0-7转对外1-8 */
    return (uint8_t)(s_zone_map[loop_no - 1U][dev_no] + 1U);
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_SetDeviceZone
 * 功能描述：设置设备所属防火分区（设置后立即持久化并通知）
 * 输入参数：loop_no - 回路号, dev_no - 设备地址
 *           zone_no - 目标分区号(1-8)
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
    if (s_zone_map[loop_no - 1U][dev_no] == (zone_no - 1U))
    {
        return 0;  /* 无变化视为成功 */
    }

    /* 更新RAM映射 */
    s_zone_map[loop_no - 1U][dev_no] = (uint8_t)(zone_no - 1U);

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
 * 输入参数：zone_no - 分区号(1-8)
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

    /* 按回路1→2→3顺序统计(回路4为预留回路, 当前无设备) */
    for (l = 1U; l <= 3U; l++)
    {
        /* 回路2地址1-63, 回路1/3地址1-30 */
        uint16_t dmax = (l == 2U) ? 63U : 30U;
        for (d = 1U; d <= dmax; d++)
        {
            if (s_zone_map[l - 1U][d] == (zone_no - 1U))
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
 * 输入参数：zone_no - 分区号(1-8), index - 序号(0基)
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
        uint16_t dmax = (l == 2U) ? 63U : 30U;
        for (d = 1U; d <= dmax; d++)
        {
            if (s_zone_map[l - 1U][d] == (zone_no - 1U))
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
 * 功能描述：获取当前分区总数 = 非空最高分区号
 *           出厂默认全部设备在分区1, 至少返回1
 * 返回值   ：分区总数(1-8)
 *--------------------------------------------------------------*/
uint8_t FireZone_GetZoneCount(void)
{
    uint8_t l;             /* 回路号循环 */
    uint16_t d;            /* 设备地址循环 */
    uint8_t max_idx = 0;   /* 已占用的最高内部分区值(0基) */

    /* 遍历全部映射, 找最高被占用的分区 */
    for (l = 0; l < 4U; l++)
    {
        for (d = 0; d < 64U; d++)
        {
            if (s_zone_map[l][d] > max_idx)
            {
                max_idx = s_zone_map[l][d];  /* 更新最高占用分区 */
            }
        }
    }

    /* 内部0基转对外1基: 全0时返回1(至少1个分区) */
    return (uint8_t)(max_idx + 1U);
}

/*--------------------------------------------------------------
 * 函数名称：FireZone_GetZoneName
 * 功能描述：获取分区名称(GBK字符串)
 * 输入参数：zone_no - 分区号(1-8)
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
 * 输入参数：zone_no - 分区号(1-8), name - 名称字符串
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
 * 输入参数：zone_no - 分区号(1-8)
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
 * 输入参数：zone_no - 分区号(1-8), on - 1=启用 0=停用
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
 * 输入参数：zone_no - 分区号(1-8)
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
 * 输入参数：zone_no - 目标分区号(1-8), loop_no - 回路号(1-4),
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

    /* 地址段校验: 回路2地址1-63, 回路1/3/4地址1-30 */
    dmax = (loop_no == 2U) ? 63U : 30U;
    if (start < 1U || start > end || (uint16_t)end > dmax)
    {
        return 2;
    }

    /* 逐地址写入目标分区(映射单值, 旧分区归属自动被覆盖) */
    for (d = start; d <= (uint16_t)end; d++)
    {
        if (s_zone_map[loop_no - 1U][d] != (uint8_t)(zone_no - 1U))
        {
            s_zone_map[loop_no - 1U][d] = (uint8_t)(zone_no - 1U);
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
 * 功能描述：清空目标分区在指定回路的全部设备(归位分区1)
 *           对应画面输入"00000.00000"的清空归位操作
 * 输入参数：zone_no - 分区号(1-8), loop_no - 回路号(1-4)
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

    dmax = (loop_no == 2U) ? 63U : 30U;

    /* 遍历该回路, 把属于目标分区的地址归位分区1(映射值0) */
    for (d = 1U; d <= dmax; d++)
    {
        if (s_zone_map[loop_no - 1U][d] == (uint8_t)(zone_no - 1U))
        {
            s_zone_map[loop_no - 1U][d] = 0U;
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
