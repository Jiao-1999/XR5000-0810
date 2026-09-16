/*==============================================================
 * 文件名称   : bsp_fire_zone.c
 * 模块功能   : 防火分区管理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 提供设备防火分区归属的管理功能。
 *              - 设备分区映射: (回路号,设备地址) -> 分区号(1-8)
 *              - 分区名称管理(GBK)
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

/* Flash存储记录结构 - packed紧凑布局, 共394字节
 * 布局 = 头部(魔数+版本) + 分区映射(256B) + 分区名称(128B) + CRC16
 * 存储时同一份记录写入扇区前半(副本A)与后半(副本B)两个位置 */
typedef struct
{
    uint32_t magic;                                         /* 魔数 FIRE_ZONE_MAGIC "FRZ1" */
    uint8_t  version;                                       /* 存储格式版本号 */
    uint8_t  reserved[3];                                   /* 保留对齐填充 */
    uint8_t  zone_map[4][64];                               /* 设备分区映射, [回路号-1][设备地址], 值0-7对应分区1-8 */
    uint8_t  zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];  /* 分区名称(GBK), 下标0-7对应分区1-8 */
    uint16_t crc;                                           /* CRC16校验和, 覆盖除crc字段外全部字节 */
} __attribute__((packed)) FireZoneRecord_t;

/* 设备分区映射表: s_zone_map[回路号-1][设备地址]
 * 值0-7对应分区1-8, 默认全0 = 全部设备属于分区1 */
static uint8_t s_zone_map[4][64];

/* 分区名称表(GBK编码): 对外分区号1-8对应下标0-7 */
static char s_zone_name[FIRE_ZONE_MAX][FIRE_ZONE_NAME_LEN];

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
    /* sizeof(FireZoneRecord_t)=394字节, crc字段占最后2字节, 实际计算392字节 */
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

    /* 版本校验(格式变更时旧版本数据不识别, 走默认流程) */
    if (rec->version != FIRE_ZONE_VERSION)
    {
        return 0;  /* 版本不符 */
    }

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
    rec.crc = CalcRecordCRC(&rec);

    /* 擦除整个4KB扇区(副本A/B同扇区, 只需擦一次) */
    W25QXX_Erase_Sector(FIRE_ZONE_FLASH_ADDR);

    /* 写入副本A(扇区前半) */
    W25QXX_Write((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_A, sizeof(FireZoneRecord_t));

    /* 写入副本B(扇区后半) */
    W25QXX_Write((uint8_t *)&rec, FIRE_ZONE_FLASH_COPY_B, sizeof(FireZoneRecord_t));
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
        SaveRecordToFlash();
        loaded = 1;
        DebugPrintf("[FIREZONE] copy A bad, restored from copy B\r\n");
    }
    /* 两个副本均无效: 保持出厂默认, 不立即写Flash */

    s_initialized = 1;

    DebugPrintf("[FIREZONE] init %s, zone count=%u\r\n",
                loaded ? "loaded" : "default",
                (unsigned int)FireZone_GetZoneCount());
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

    /* 持久化到Flash(A/B双副本) */
    SaveRecordToFlash();

    /* 通知外部(屏幕刷新等) */
    NotifyChanged();

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

    /* 持久化到Flash(A/B双副本) */
    SaveRecordToFlash();

    /* 通知外部(屏幕刷新等) */
    NotifyChanged();

    return 0;  /* 设置成功 */
}
