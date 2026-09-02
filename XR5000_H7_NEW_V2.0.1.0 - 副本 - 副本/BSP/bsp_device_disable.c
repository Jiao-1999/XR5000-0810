/* ============================================================================
 * 模块名称: 设备屏蔽管理模块 (Device Disable Management)
 * 功能描述: 实现回路1/回路3探测器的屏蔽/解除屏蔽,
 *          屏蔽状态持久化存储(Flash 0x112000/0x113000),
 *          屏蔽历史记录(Flash 0x0A6000~0x0AAFFF),
 *          画面70(屏蔽操作界面)和画面57(屏蔽历史查询)的HMI交互。
 * 关键约束: 屏蔽后保留总线轮询和原始数据,隔离报警/故障/掉线事件;
 *          解除后由下一业务周期立即重评当前状态。
 * ============================================================================ */

#include "bsp_device_disable.h"

#include "bsp_mbus.h"            
#include "bsp_rs485_detect.h"    
#include "bsp_rtc.h"             
#include "bsp_screen.h"         
#include "hmi_driver.h"          
#include "system.h"              
#include "w25qxx.h"
#include "bsp_storage_event.h"  /* 锟斤拷匣锟接存储锟铰硷拷锟斤拷锟斤拷锟?*/
#include "bsp_fecbus_report.h" /* FECbus RS485 锟较憋拷锟斤拷锟斤拷锟?(GB4717 锟斤拷录C) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Flash地址定义 ---- */
#define DEVICE_DISABLE_STATE_MAIN_ADDR   0x112000UL  /* 屏蔽状态主存储区 */
#define DEVICE_DISABLE_STATE_BACKUP_ADDR 0x113000UL  /* 屏蔽状态备份区 */
#define DEVICE_DISABLE_HISTORY_META_ADDR 0x0A6000UL  /* 屏蔽历史管理区 */
#define DEVICE_DISABLE_HISTORY_DATA_ADDR 0x0A7000UL  /* 屏蔽历史数据区 */
#define DEVICE_DISABLE_HISTORY_SECTORS   4U           /* 历史数据占用扇区数 */
#define DEVICE_DISABLE_STATE_MAGIC       0x4453424CUL /* 状态魔数 "DSBL" */
#define DEVICE_DISABLE_HISTORY_MAGIC     0x4453484CUL /* 历史魔数 "DSHL" */
#define DEVICE_DISABLE_STATE_VERSION     1U           /* 状态格式版本 */
#define DEVICE_DISABLE_EVENT_SET         72U          /* 设置屏蔽事件码 */
#define DEVICE_DISABLE_EVENT_CLEAR       73U          /* 解除屏蔽事件码 */

/* 屏蔽状态持久化结构(存储于Flash主/备区) */
typedef struct
{
    uint32_t magic;                                    /* 魔数校验 */
    uint16_t version;                                  /* 格式版本 */
    uint16_t sequence;                                 /* 写序列号 */
    uint8_t loop1_bitmap[13];                          /* 回路1屏蔽位图(100bit) */
    uint8_t loop3_bitmap[13];                          /* 回路3屏蔽位图(100bit) */
    DeviceDisableRecent recent[DEVICE_DISABLE_RECENT_MAX]; /* 最近屏蔽列表 */
    uint16_t disabled_count;                           /* 总屏蔽数量 */
    uint16_t crc;                                      /* CRC16校验 */
} DeviceDisablePersistentState;

/* 屏蔽历史管理区结构 */
typedef struct
{
    uint32_t magic;                                    /* 魔数校验 */
    uint16_t count;                                    /* 已写入记录数 */
    uint16_t crc;                                      /* CRC16校验 */
} DeviceDisableHistoryMeta;

/* 单条屏蔽历史记录 */
typedef struct
{
    uint8_t event_type;                                /* 事件类型: 72=设置 / 73=解除 */
    uint8_t loop_id;                                   /* 回路编号 */
    uint16_t address;                                  /* 设备地址 */
    uint8_t device_type;                               /* 设备类型 */
    uint8_t time[5];                                   /* 时间戳(压缩格式) */
    uint16_t crc;                                      /* CRC16校验 */
} DeviceDisableHistoryRecord;

/* ---- 全局状态变量 ---- */
static DeviceDisablePersistentState g_disable_state;          /* 当前屏蔽状态 */
static DeviceDisablePersistentState g_disable_state_verify;   /* Flash读取校验缓冲 */
static DeviceDisablePersistentState g_disable_state_rollback; /* 失败回滚缓冲 */
extern uint8_t shielding_state;                               /* 全局屏蔽指示(通知主机) */

/* ---- 画面70 HMI交互状态 ---- */
static uint8_t g_hmi_loop_id = DEVICE_DISABLE_LOOP1;          /* 当前选择的回路 */
static uint16_t g_hmi_address = 0U;                            /* 当前输入的设备地址 */
static uint8_t g_hmi_page_active = 0U;                         /* 画面70是否激活 */
static uint8_t g_hmi_lines[DEVICE_DISABLE_RECENT_MAX][96];     /* 左侧20行文本缓冲 */
static volatile uint8_t g_hmi_operation_pending = 0U;          /* 操作请求待处理标志 */
static uint8_t g_hmi_operation = 0U;                            /* 操作类型: 1=设置 / 2=解除 */
static DeviceIdentity g_hmi_operation_identity;                 /* 待操作设备标识 */
static volatile uint8_t g_hmi_menu_refresh_pending = 0U;       /* 菜单刷新请求 */
static volatile uint8_t g_hmi_address_refresh_pending = 0U;    /* 地址输入刷新请求 */
static volatile uint8_t g_hmi_query_pending = 0U;              /* 查询请求标志 */
static uint8_t g_hmi_address_text[4];                          /* 地址输入快照 */
static uint8_t g_hmi_query_text[6];                            /* 查询编号快照 */

/* ---- 画面57 屏蔽历史HMI状态 ---- */
static uint8_t g_history_hmi_active = 0U;                      /* 历史页面是否激活 */
static volatile uint8_t g_history_hmi_refresh_pending = 0U;    /* 历史刷新请求 */
static uint16_t g_history_hmi_page = 1U;                       /* 当前页码 */

/* ---- 中文字符串(用于HMI显示, GBK编码) ---- */
#define GBK_LOOP             "回路"
#define GBK_ADDR_SUFFIX      "号"
#define GBK_SMOKE            "烟雾探测器"
#define GBK_TEMPERATURE      "温度探测器"
#define GBK_MULTI            "复合探测器"
#define GBK_UNKNOWN          "未知设备"
#define GBK_SET_OK           "已设置为屏蔽状态"
#define GBK_CLEAR_OK         "已解除屏蔽状态"
#define GBK_WARNING          "正在预警,禁止屏蔽"
#define GBK_FIRE             "正在火警,禁止屏蔽"
#define GBK_FAULT            "正在故障,禁止屏蔽"
#define GBK_OFFLINE          "设备掉线,禁止屏蔽"
#define GBK_NOT_CONFIGURED   "设备未配置"
#define GBK_UNSUPPORTED_TYPE "设备类型不支持"
#define GBK_ALREADY_SET      "已经处于屏蔽状态"
#define GBK_NOT_SET          "当前未屏蔽"
#define GBK_INVALID_ADDR     "地址无效"
#define GBK_INVALID_LOOP     "回路不支持"
#define GBK_SAVE_FAILED      "保存失败"
#define GBK_DEVICE_CODE      "设备编号："
#define GBK_DEVICE_TYPE      "设备类型："
#define GBK_DISABLE_STATE    "屏蔽状态："
#define GBK_DISABLED         "已屏蔽"
#define GBK_NOT_DISABLED     "未屏蔽"
#define GBK_DISABLE_UNSUP    "不支持屏蔽"
#define GBK_NOT_FOUND        "未找到该设备"
#define GBK_UNAVAILABLE      "不可用"
#define GBK_LOOP1_TEXT       "第" "1" "回路"
#define GBK_LOOP3_TEXT       "第" "3" "回路"
#define GBK_DEVICE_LOOP      "第"
#define GBK_DEVICE_LOOP_END  "回路"
#define GBK_DEVICE_NUMBER    "号"
#define GBK_HISTORY_SET      "设置屏蔽"
#define GBK_HISTORY_CLEAR    "解除屏蔽"
#define GBK_YEAR_SUFFIX      "年"
#define GBK_MONTH_SUFFIX     "月"
#define GBK_DAY_SUFFIX       "日"
#define GBK_INVALID_TIME     "时间无效"
#define GBK_DISABLED_COUNT   "屏蔽设备:"

/* ========================================================================
 * 内部工具函数: Flash存储与位图操作
 * ======================================================================== */

/* 获取指定回路的屏蔽位图指针 */
static uint8_t *DeviceDisableBitmap(uint8_t loop_id)
{
    if(loop_id == DEVICE_DISABLE_LOOP1) return g_disable_state.loop1_bitmap;
    if(loop_id == DEVICE_DISABLE_LOOP3) return g_disable_state.loop3_bitmap;
    return NULL;
}

/* 计算屏蔽状态的CRC16校验值 */
static uint16_t DeviceDisableStateCrc(const DeviceDisablePersistentState *state)
{
    return CalcCrc16((uint8_t *)state, (uint16_t)(sizeof(*state) - sizeof(state->crc)));
}

/* 校验屏蔽状态数据完整性(魔数+版本+CRC) */
static uint8_t DeviceDisableStateValid(const DeviceDisablePersistentState *state)
{
    return state->magic == DEVICE_DISABLE_STATE_MAGIC &&
           state->version == DEVICE_DISABLE_STATE_VERSION &&
           state->crc == DeviceDisableStateCrc(state);
}

/* 保存屏蔽状态到Flash(先写备份区,校验通过后写主区,再校验) */
static uint8_t DeviceDisableSaveState(void)
{
    g_disable_state.magic = DEVICE_DISABLE_STATE_MAGIC;
    g_disable_state.version = DEVICE_DISABLE_STATE_VERSION;
    g_disable_state.sequence++;
    g_disable_state.crc = DeviceDisableStateCrc(&g_disable_state);
    W25QXX_Write((uint8_t *)&g_disable_state, DEVICE_DISABLE_STATE_BACKUP_ADDR, sizeof(g_disable_state));
    W25QXX_Read((uint8_t *)&g_disable_state_verify, DEVICE_DISABLE_STATE_BACKUP_ADDR, sizeof(g_disable_state_verify));
    if(!DeviceDisableStateValid(&g_disable_state_verify)) return 0U;
    W25QXX_Write((uint8_t *)&g_disable_state, DEVICE_DISABLE_STATE_MAIN_ADDR, sizeof(g_disable_state));
    W25QXX_Read((uint8_t *)&g_disable_state_verify, DEVICE_DISABLE_STATE_MAIN_ADDR, sizeof(g_disable_state_verify));
    return DeviceDisableStateValid(&g_disable_state_verify);
}

/* 从RTC读取当前时间并压缩为32位格式 */
static uint8_t DeviceDisablePackedTime(uint32_t *packed)
{
    BM8563_TimeTypeDef rtc_time;
    if(packed == NULL) return 0U;
    BM8563_Soft_I2C_GetTime(&rtc_time);
    if(rtc_time.year > 99U || rtc_time.month < 1U || rtc_time.month > 12U ||
       rtc_time.day < 1U || rtc_time.day > 31U || rtc_time.hours > 23U ||
       rtc_time.minutes > 59U || rtc_time.seconds > 59U) return 0U;
    SystemTime = rtc_time;
    *packed = ((uint32_t)rtc_time.year << 26) |
              ((uint32_t)rtc_time.month << 22) |
              ((uint32_t)rtc_time.day << 17) |
              ((uint32_t)rtc_time.hours << 12) |
              ((uint32_t)rtc_time.minutes << 6) |
              (uint32_t)rtc_time.seconds;
    return 1U;
}

/* 将压缩时间打包为5字节历史记录格式 */
static uint8_t DeviceDisablePackHistoryTime(uint8_t time[5])
{
    uint32_t packed;
    if(!DeviceDisablePackedTime(&packed)) return 0U;
    time[0] = (uint8_t)(packed >> 24);
    time[1] = (uint8_t)(packed >> 16);
    time[2] = (uint8_t)(packed >> 8);
    time[3] = (uint8_t)packed;
    time[4] = 0U;
    return 1U;
}

/* 计算历史管理区的CRC16校验 */
static uint16_t DeviceDisableHistoryMetaCrc(const DeviceDisableHistoryMeta *meta)
{
    return CalcCrc16((uint8_t *)meta, (uint16_t)(sizeof(*meta) - sizeof(meta->crc)));
}

/* 追加一条屏蔽历史记录到Flash */
static uint8_t DeviceDisableAppendHistory(uint8_t event_type, const DeviceInformation *info)
{
    DeviceDisableHistoryMeta meta;
    DeviceDisableHistoryRecord record;
    const uint16_t max_records = (DEVICE_DISABLE_HISTORY_SECTORS * 4096U) / sizeof(record);
    W25QXX_Read((uint8_t *)&meta, DEVICE_DISABLE_HISTORY_META_ADDR, sizeof(meta));
    if(meta.magic != DEVICE_DISABLE_HISTORY_MAGIC || meta.crc != DeviceDisableHistoryMetaCrc(&meta))
    {
        memset(&meta, 0, sizeof(meta));
        meta.magic = DEVICE_DISABLE_HISTORY_MAGIC;
    }
    if(meta.count >= max_records) return 0U;
    memset(&record, 0, sizeof(record));
    record.event_type = event_type;
    record.loop_id = info->identity.loop_id;
    record.address = info->identity.address;
    record.device_type = (uint8_t)info->type;
    if(!DeviceDisablePackHistoryTime(record.time)) return 0U;
    record.crc = CalcCrc16((uint8_t *)&record, (uint16_t)(sizeof(record) - sizeof(record.crc)));
    W25QXX_Write((uint8_t *)&record,
                 DEVICE_DISABLE_HISTORY_DATA_ADDR + (uint32_t)meta.count * sizeof(record),
                 sizeof(record));
    meta.count++;
    meta.crc = DeviceDisableHistoryMetaCrc(&meta);
    W25QXX_Write((uint8_t *)&meta, DEVICE_DISABLE_HISTORY_META_ADDR, sizeof(meta));
    return 1U;
}

/* ========================================================================
 * 内部工具函数: 历史记录查询与最近列表管理
 * ======================================================================== */

/* 检查历史记录是否还有存储空间 */
static uint8_t DeviceDisableHistoryHasSpace(void)
{
    DeviceDisableHistoryMeta meta;
    const uint16_t max_records = (DEVICE_DISABLE_HISTORY_SECTORS * 4096U) / sizeof(DeviceDisableHistoryRecord);
    W25QXX_Read((uint8_t *)&meta, DEVICE_DISABLE_HISTORY_META_ADDR, sizeof(meta));
    if(meta.magic != DEVICE_DISABLE_HISTORY_MAGIC || meta.crc != DeviceDisableHistoryMetaCrc(&meta)) return 1U;
    return meta.count < max_records;
}

/* 获取已写入的历史记录总数 */
static uint16_t DeviceDisableHistoryCount(void)
{
    DeviceDisableHistoryMeta meta;
    const uint16_t max_records = (DEVICE_DISABLE_HISTORY_SECTORS * 4096U) / sizeof(DeviceDisableHistoryRecord);
    W25QXX_Read((uint8_t *)&meta, DEVICE_DISABLE_HISTORY_META_ADDR, sizeof(meta));
    if(meta.magic != DEVICE_DISABLE_HISTORY_MAGIC || meta.crc != DeviceDisableHistoryMetaCrc(&meta)) return 0U;
    return meta.count <= max_records ? meta.count : 0U;
}

/* 按索引读取单条历史记录(含CRC校验) */
static uint8_t DeviceDisableHistoryRead(uint16_t index, DeviceDisableHistoryRecord *record)
{
    const uint16_t max_records = (DEVICE_DISABLE_HISTORY_SECTORS * 4096U) / sizeof(DeviceDisableHistoryRecord);
    if(record == NULL) return 0U;
    if(index >= max_records) return 0U;
    W25QXX_Read((uint8_t *)record,
                DEVICE_DISABLE_HISTORY_DATA_ADDR + (uint32_t)index * sizeof(*record),
                sizeof(*record));
    return record->crc == CalcCrc16((uint8_t *)record,
                                    (uint16_t)(sizeof(*record) - sizeof(record->crc)));
}

/* 将5字节压缩时间解包为32位格式 */
static uint32_t DeviceDisableHistoryUnpackTime(const uint8_t time[5])
{
    return ((uint32_t)time[0] << 24) | ((uint32_t)time[1] << 16) |
           ((uint32_t)time[2] << 8) | (uint32_t)time[3];
}

/* 从最近屏蔽列表中移除指定设备 */
static void DeviceDisableRemoveRecent(const DeviceIdentity *identity)
{
    uint8_t i;
    for(i = 0U; i < DEVICE_DISABLE_RECENT_MAX; i++)
    {
        if(g_disable_state.recent[i].identity.loop_id == identity->loop_id &&
           g_disable_state.recent[i].identity.address == identity->address)
        {
            memmove(&g_disable_state.recent[i], &g_disable_state.recent[i + 1U],
                    (DEVICE_DISABLE_RECENT_MAX - i - 1U) * sizeof(g_disable_state.recent[0]));
            memset(&g_disable_state.recent[DEVICE_DISABLE_RECENT_MAX - 1U], 0,
                   sizeof(g_disable_state.recent[0]));
            break;
        }
    }
}

/* 将设备插入最近屏蔽列表头部(自动去重) */
static void DeviceDisableInsertRecent(const DeviceInformation *info)
{
    uint32_t packed_time = 0U;
    DeviceDisableRemoveRecent(&info->identity);
    memmove(&g_disable_state.recent[1], &g_disable_state.recent[0],
            (DEVICE_DISABLE_RECENT_MAX - 1U) * sizeof(g_disable_state.recent[0]));
    g_disable_state.recent[0].identity = info->identity;
    g_disable_state.recent[0].device_type = (uint8_t)info->type;
    DeviceDisablePackedTime(&packed_time);
    g_disable_state.recent[0].disabled_time = packed_time;
}

/* ========================================================================
 * 公共接口: 初始化与设备编号解析
 * ======================================================================== */

/* 模块初始化: 从Flash加载屏蔽状态(主区→备区→新建) */
void DeviceDisable_Init(void)
{
    DeviceDisablePersistentState main_state;
    DeviceDisablePersistentState backup_state;
    W25QXX_Read((uint8_t *)&main_state, DEVICE_DISABLE_STATE_MAIN_ADDR, sizeof(main_state));
    W25QXX_Read((uint8_t *)&backup_state, DEVICE_DISABLE_STATE_BACKUP_ADDR, sizeof(backup_state));
    if(DeviceDisableStateValid(&main_state)) g_disable_state = main_state;
    else if(DeviceDisableStateValid(&backup_state))
    {
        g_disable_state = backup_state;
        DeviceDisableSaveState();
    }
    else
    {
        memset(&g_disable_state, 0, sizeof(g_disable_state));
        DeviceDisableSaveState();
    }
    shielding_state = g_disable_state.disabled_count != 0U;
}

/* 解析5位数字设备编号(格式: LLAAA, 前2位回路号, 后3位地址) */
uint8_t DeviceCodeParse(const uint8_t *text, DeviceIdentity *identity)
{
    uint8_t i;
    if(text == NULL || identity == NULL) return 0U;
    for(i = 0U; i < 5U; i++) if(text[i] < '0' || text[i] > '9') return 0U;
    if(text[5] != '\0') return 0U;
    identity->loop_id = (uint8_t)((text[0] - '0') * 10U + (text[1] - '0'));
    identity->address = (uint16_t)((text[2] - '0') * 100U + (text[3] - '0') * 10U + (text[4] - '0'));
    return identity->loop_id != 0U && identity->address != 0U;
}

/* 将设备标识格式化为5位数字编号 */
void DeviceCodeFormat(const DeviceIdentity *identity, uint8_t output[6])
{
    output[0] = (uint8_t)('0' + (identity->loop_id / 10U) % 10U);
    output[1] = (uint8_t)('0' + identity->loop_id % 10U);
    output[2] = (uint8_t)('0' + (identity->address / 100U) % 10U);
    output[3] = (uint8_t)('0' + (identity->address / 10U) % 10U);
    output[4] = (uint8_t)('0' + identity->address % 10U);
    output[5] = '\0';
}

/* ========================================================================
 * 公共接口: 屏蔽状态查询
 * ======================================================================== */

/* 查询指定设备是否处于屏蔽状态(位图判读) */
uint8_t DeviceDisableIsSet(const DeviceIdentity *identity)
{
    uint8_t *bitmap;
    uint16_t bit;
    if(identity == NULL || identity->address == 0U || identity->address > DEVICE_DISABLE_MAX_ADDRESS) return 0U;
    bitmap = DeviceDisableBitmap(identity->loop_id);
    if(bitmap == NULL) return 0U;
    bit = identity->address - 1U;
    return (bitmap[bit / 8U] >> (bit % 8U)) & 0x01U;
}

/* 按回路和地址查询屏蔽状态 */
uint8_t DeviceDisableIsLoopAddressSet(uint8_t loop_id, uint16_t address)
{
    DeviceIdentity identity;
    identity.loop_id = loop_id;
    identity.address = address;
    return DeviceDisableIsSet(&identity);
}

/* 查询设备完整信息: 从回路1/回路3的实时状态中获取配置、在线、类型、运行状态 */
uint8_t DeviceRegistryQuery(const DeviceIdentity *identity, DeviceInformation *information)
{
    uint8_t state_class;
    uint8_t sensor;
    if(identity == NULL || information == NULL) return 0U;
    memset(information, 0, sizeof(*information));
    information->identity = *identity;
    information->disabled = DeviceDisableIsSet(identity);
    if(identity->loop_id == DEVICE_DISABLE_LOOP1 && identity->address <= MIXTURE_DEVICE_MAX_ADDR)
    {
        information->configured = getPointTypeMixtureSettingOnlieState((uint8_t)identity->address);
        information->exists = information->configured;
        information->online = information->configured &&
                              getPointTypeMixtureDisconnectCount((uint8_t)identity->address) < MIXTURE_DEVICE_DISCONNECT_SUM;
        if(getPointTypeMixtureDetectName((uint8_t)identity->address) == 5U) information->type = DEVICE_TYPE_SMOKE;
        else if(getPointTypeMixtureDetectName((uint8_t)identity->address) == 6U) information->type = DEVICE_TYPE_TEMPERATURE;
        state_class = getPointTypeMixtureStateClass((uint8_t)identity->address);
        information->state = state_class == 1U ? DEVICE_RUNTIME_WARNING :
                             state_class == 2U ? DEVICE_RUNTIME_FIRE :
                             state_class == 3U ? DEVICE_RUNTIME_FAULT : DEVICE_RUNTIME_NORMAL;
        if(information->configured && !information->online) information->state = DEVICE_RUNTIME_OFFLINE;
        return information->exists;
    }
    if(identity->loop_id == DEVICE_DISABLE_LOOP3 && identity->address < RS485_DETECT_MAX_DEVICES)
    {
        information->configured = RS485Detect_GetOnline((uint8_t)identity->address);
        information->exists = information->configured;
        information->online = information->configured && !RS485Detect_IsDisconnected((uint8_t)identity->address);
        information->type = RS485Detect_GetType((uint8_t)identity->address) == RS485_DETECT_TYPE_UNKNOWN ?
                            DEVICE_TYPE_UNKNOWN : DEVICE_TYPE_MULTI_SENSOR;
        information->state = DEVICE_RUNTIME_NORMAL;
        if(information->configured && !information->online) information->state = DEVICE_RUNTIME_OFFLINE;
        else
        {
            for(sensor = 0U; sensor < RS485_SENSOR_COUNT; sensor++)
            {
                uint8_t raw = RS485Detect_GetSensorState((uint8_t)identity->address, sensor);
                if((sensor == RS485_SENSOR_TEMPERATURE && raw == 3U) ||
                   (sensor == RS485_SENSOR_SMOKE && raw == 8U)) information->state = DEVICE_RUNTIME_FAULT;
                else if(raw == 2U && information->state < DEVICE_RUNTIME_FIRE) information->state = DEVICE_RUNTIME_FIRE;
                else if(raw == 1U && information->state < DEVICE_RUNTIME_WARNING) information->state = DEVICE_RUNTIME_WARNING;
            }
        }
        return information->exists;
    }
    return 0U;
}

/* 校验屏蔽前置条件: 回路支持→地址有效→设备已配置→类型已知→未屏蔽→非预警/火警/故障/掉线 */
static DeviceDisableResult DeviceDisableValidateSet(const DeviceIdentity *identity, DeviceInformation *info)
{
    if(identity->loop_id != DEVICE_DISABLE_LOOP1 && identity->loop_id != DEVICE_DISABLE_LOOP3) return DEVICE_DISABLE_INVALID_LOOP;
    if(identity->address == 0U || identity->address > DEVICE_DISABLE_MAX_ADDRESS) return DEVICE_DISABLE_INVALID_ADDRESS;
    if(!DeviceRegistryQuery(identity, info) || !info->configured) return DEVICE_DISABLE_NOT_CONFIGURED;
    if(info->type == DEVICE_TYPE_UNKNOWN) return DEVICE_DISABLE_UNSUPPORTED_TYPE;
    if(info->disabled) return DEVICE_DISABLE_ALREADY_SET;
    if(info->state == DEVICE_RUNTIME_WARNING) return DEVICE_DISABLE_WARNING;
    if(info->state == DEVICE_RUNTIME_FIRE) return DEVICE_DISABLE_FIRE;
    if(info->state == DEVICE_RUNTIME_FAULT) return DEVICE_DISABLE_FAULT;
    if(info->state == DEVICE_RUNTIME_OFFLINE) return DEVICE_DISABLE_OFFLINE;
    return DEVICE_DISABLE_OK;
}

/* 锟借备注锟斤拷锟斤拷锟酵★拷锟斤拷匣锟斤拷锟借备锟斤拷锟酵达拷锟斤拷映锟斤拷(锟斤拷LogShield锟斤拷锟斤拷) */
static uint16_t DeviceDisableMapStorageType(DeviceRegistryType type)
{
    switch(type)
    {
        case DEVICE_TYPE_SMOKE:        return DEV_TYPE_SMOKE;       /* 锟斤拷锟斤拷探锟斤拷锟斤拷 */
        case DEVICE_TYPE_TEMPERATURE:  return DEV_TYPE_TEMPERATURE; /* 锟斤拷锟斤拷探锟斤拷锟斤拷 */
        case DEVICE_TYPE_MULTI_SENSOR: return DEV_TYPE_FIRE_ALARM;  /* 锟斤拷锟斤拷探锟斤拷锟斤拷锟斤拷锟矫伙拷锟街憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 */
        default:                       return DEV_TYPE_CONTROL_DEV; /* 未知锟斤拷锟酵帮拷锟斤拷锟斤拷锟借备 */
    }
}

/* 锟斤拷锟斤拷锟斤拷锟斤拷: 校锟斤拷锟叫次煌硷拷锟斤拷锟斤拷锟紽lash锟斤拷写锟斤拷史(失锟斤拷锟斤拷毓锟斤拷锟斤拷指锟斤拷锟斤拷锟? */
DeviceDisableResult DeviceDisableSet(const DeviceIdentity *identity)
{
    DeviceInformation info;
    uint8_t *bitmap;
    uint16_t bit;
    DeviceDisableResult result;
    if(identity == NULL) return DEVICE_DISABLE_INVALID_ADDRESS;
    result = DeviceDisableValidateSet(identity, &info);
    if(result != DEVICE_DISABLE_OK) return result;
    if(!DeviceDisableHistoryHasSpace()) return DEVICE_DISABLE_STORAGE_ERROR;
    g_disable_state_rollback = g_disable_state;
    bitmap = DeviceDisableBitmap(identity->loop_id);
    bit = identity->address - 1U;
    bitmap[bit / 8U] |= (uint8_t)(1U << (bit % 8U));
    g_disable_state.disabled_count++;
    DeviceDisableInsertRecent(&info);
    if(!DeviceDisableSaveState())
    {
        g_disable_state = g_disable_state_rollback;
        DeviceDisableSaveState();
        return DEVICE_DISABLE_STORAGE_ERROR;
    }
    info.disabled = 1U;
    if(!DeviceDisableAppendHistory(DEVICE_DISABLE_EVENT_SET, &info))
    {
        g_disable_state = g_disable_state_rollback;
        DeviceDisableSaveState();
        return DEVICE_DISABLE_STORAGE_ERROR;
    }
    shielding_state = 1U;
    StorageEvent_LogShield((uint8_t)identity->address,
                           DeviceDisableMapStorageType(info.type), 0U); /* 锟斤拷匣锟斤拷:锟斤拷录锟斤拷锟斤拷 */
    FecbusReport_Shield((uint8_t)identity->address,
                        DeviceDisableMapStorageType(info.type), 0U); /* FECbus:锟较憋拷锟斤拷锟斤拷 */
    return DEVICE_DISABLE_OK;
}

/* 解除屏蔽: 校验→清除位图→从最近列表移除→保存Flash→写历史(失败则回滚并恢复备份) */
DeviceDisableResult DeviceDisableClear(const DeviceIdentity *identity)
{
    DeviceInformation info;
    uint8_t *bitmap;
    uint16_t bit;
    if(identity == NULL) return DEVICE_DISABLE_INVALID_ADDRESS;
    if(identity->loop_id != DEVICE_DISABLE_LOOP1 && identity->loop_id != DEVICE_DISABLE_LOOP3) return DEVICE_DISABLE_INVALID_LOOP;
    if(identity->address == 0U || identity->address > DEVICE_DISABLE_MAX_ADDRESS) return DEVICE_DISABLE_INVALID_ADDRESS;
    DeviceRegistryQuery(identity, &info);
    if(!DeviceDisableIsSet(identity)) return DEVICE_DISABLE_NOT_SET;
    info.identity = *identity;
    info.disabled = 1U;
    if(info.type == DEVICE_TYPE_UNKNOWN)
    {
        uint8_t i;
        for(i = 0U; i < DEVICE_DISABLE_RECENT_MAX; i++)
            if(g_disable_state.recent[i].identity.loop_id == identity->loop_id &&
               g_disable_state.recent[i].identity.address == identity->address)
                info.type = (DeviceRegistryType)g_disable_state.recent[i].device_type;
    }
    if(!DeviceDisableHistoryHasSpace()) return DEVICE_DISABLE_STORAGE_ERROR;
    g_disable_state_rollback = g_disable_state;
    bitmap = DeviceDisableBitmap(identity->loop_id);
    bit = identity->address - 1U;
    bitmap[bit / 8U] &= (uint8_t)~(1U << (bit % 8U));
    if(g_disable_state.disabled_count > 0U) g_disable_state.disabled_count--;
    DeviceDisableRemoveRecent(identity);
    if(!DeviceDisableSaveState())
    {
        g_disable_state = g_disable_state_rollback;
        DeviceDisableSaveState();
        return DEVICE_DISABLE_STORAGE_ERROR;
    }
    info.disabled = 0U;
    if(!DeviceDisableAppendHistory(DEVICE_DISABLE_EVENT_CLEAR, &info))
    {
        g_disable_state = g_disable_state_rollback;
        DeviceDisableSaveState();
        return DEVICE_DISABLE_STORAGE_ERROR;
    }
    shielding_state = g_disable_state.disabled_count != 0U;
    StorageEvent_LogShield((uint8_t)identity->address,
                           DeviceDisableMapStorageType(info.type), 1U); /* 锟斤拷匣锟斤拷:锟斤拷录锟斤拷锟斤拷锟斤拷锟?*/
    FecbusReport_Shield((uint8_t)identity->address,
                        DeviceDisableMapStorageType(info.type), 1U); /* FECbus:锟较憋拷锟斤拷锟斤拷锟斤拷锟?*/
    return DEVICE_DISABLE_OK;
}

/* 获取当前总屏蔽设备数量 */
uint16_t DeviceDisableGetCount(void)
{
    return g_disable_state.disabled_count;
}

/* 获取指定回路的屏蔽设备数量(遍历位图统计) */
uint16_t DeviceDisableGetLoopCount(uint8_t loop_id)
{
    uint16_t count = 0U;
    uint16_t address;
    if(loop_id != DEVICE_DISABLE_LOOP1 && loop_id != DEVICE_DISABLE_LOOP3) return 0U;
    for(address = 1U; address <= DEVICE_DISABLE_MAX_ADDRESS; address++)
    {
        if(DeviceDisableIsLoopAddressSet(loop_id, address)) count++;
    }
    return count;
}

/* 获取最近屏蔽设备列表(按时间倒序,最多max_count条) */
uint8_t DeviceDisableGetRecent(DeviceDisableRecent *output, uint8_t max_count)
{
    uint8_t count = 0U;
    uint8_t i;
    if(output == NULL) return 0U;
    for(i = 0U; i < DEVICE_DISABLE_RECENT_MAX && count < max_count; i++)
    {
        if(g_disable_state.recent[i].identity.loop_id == 0U) continue;
        if(!DeviceDisableIsSet(&g_disable_state.recent[i].identity)) continue;
        output[count++] = g_disable_state.recent[i];
    }
    return count;
}

/* 设备类型→英文名称映射 */
const char *DeviceDisableTypeText(DeviceRegistryType type)
{
    if(type == DEVICE_TYPE_SMOKE) return "smoke detector";
    if(type == DEVICE_TYPE_TEMPERATURE) return "temperature detector";
    if(type == DEVICE_TYPE_MULTI_SENSOR) return "multi-sensor detector";
    return "unknown device";
}

/* 操作结果码→英文消息映射 */
const char *DeviceDisableResultText(DeviceDisableResult result)
{
    switch(result)
    {
        case DEVICE_DISABLE_OK: return "success";
        case DEVICE_DISABLE_INVALID_LOOP: return "loop not supported";
        case DEVICE_DISABLE_INVALID_ADDRESS: return "invalid address";
        case DEVICE_DISABLE_NOT_CONFIGURED: return "device not configured";
        case DEVICE_DISABLE_UNSUPPORTED_TYPE: return "device type not supported";
        case DEVICE_DISABLE_ALREADY_SET: return "already disabled";
        case DEVICE_DISABLE_NOT_SET: return "not disabled";
        case DEVICE_DISABLE_WARNING: return "device is in warning";
        case DEVICE_DISABLE_FIRE: return "device is in fire alarm";
        case DEVICE_DISABLE_FAULT: return "device is in fault";
        case DEVICE_DISABLE_OFFLINE: return "device is offline";
        default: return "flash save failed";
    }
}

/* 设备类型→GBK中文名称映射(用于HMI显示) */
static const char *DeviceDisableTypeTextGbk(DeviceRegistryType type)
{
    if(type == DEVICE_TYPE_SMOKE) return GBK_SMOKE;
    if(type == DEVICE_TYPE_TEMPERATURE) return GBK_TEMPERATURE;
    if(type == DEVICE_TYPE_MULTI_SENSOR) return GBK_MULTI;
    return GBK_UNKNOWN;
}

/* 操作结果码→GBK中文消息映射(用于HMI显示) */
static const char *DeviceDisableResultTextGbk(DeviceDisableResult result)
{
    switch(result)
    {
        case DEVICE_DISABLE_WARNING: return GBK_WARNING;
        case DEVICE_DISABLE_FIRE: return GBK_FIRE;
        case DEVICE_DISABLE_FAULT: return GBK_FAULT;
        case DEVICE_DISABLE_OFFLINE: return GBK_OFFLINE;
        case DEVICE_DISABLE_NOT_CONFIGURED: return GBK_NOT_CONFIGURED;
        case DEVICE_DISABLE_UNSUPPORTED_TYPE: return GBK_UNSUPPORTED_TYPE;
        case DEVICE_DISABLE_ALREADY_SET: return GBK_ALREADY_SET;
        case DEVICE_DISABLE_NOT_SET: return GBK_NOT_SET;
        case DEVICE_DISABLE_INVALID_ADDRESS: return GBK_INVALID_ADDR;
        case DEVICE_DISABLE_INVALID_LOOP: return GBK_INVALID_LOOP;
        default: return GBK_SAVE_FAILED;
    }
}

/* ========================================================================
 * 内部HMI辅助函数: 画面70(屏蔽操作)与画面57(屏蔽历史)界面交互
 * ======================================================================== */

/* 刷新画面70左侧20行最近屏蔽列表文本 */
static void DeviceDisableHmiRefreshLines(void)
{
    uint8_t i;
    for(i = 0U; i < DEVICE_DISABLE_RECENT_MAX; i++)
        SetTextValue(70U, (uint16_t)(i + 1U), g_hmi_lines[i]);
}

/* 将操作结果消息推送到画面70左侧列表顶部(旧条目下移) */
static void DeviceDisableHmiPush(const DeviceIdentity *identity, DeviceRegistryType type, const char *message)
{
    memmove(&g_hmi_lines[1][0], &g_hmi_lines[0][0],
            (DEVICE_DISABLE_RECENT_MAX - 1U) * sizeof(g_hmi_lines[0]));
    memset(g_hmi_lines[0], 0, sizeof(g_hmi_lines[0]));
    snprintf((char *)g_hmi_lines[0], sizeof(g_hmi_lines[0]), "%s%u %u%s%s %s",
             GBK_LOOP, identity->loop_id, identity->address, GBK_ADDR_SUFFIX,
             DeviceDisableTypeTextGbk(type), message);
    DeviceDisableHmiRefreshLines();
}

/* 从Flash加载当前屏蔽设备列表到画面70左侧显示区 */
static void DeviceDisableHmiLoadCurrent(void)
{
    DeviceDisableRecent recent[DEVICE_DISABLE_RECENT_MAX];
    uint8_t count;
    uint8_t i;
    memset(g_hmi_lines, 0, sizeof(g_hmi_lines));
    count = DeviceDisableGetRecent(recent, DEVICE_DISABLE_RECENT_MAX);
    for(i = 0U; i < count; i++)
    {
        snprintf((char *)g_hmi_lines[i], sizeof(g_hmi_lines[i]), "%s%u %u%s%s %s",
                 GBK_LOOP, recent[i].identity.loop_id, recent[i].identity.address,
                 GBK_ADDR_SUFFIX, DeviceDisableTypeTextGbk((DeviceRegistryType)recent[i].device_type),
                 GBK_SET_OK);
    }
    DeviceDisableHmiRefreshLines();
}

/* 处理画面70的设备编号查询: 解析编号→查询设备信息→显示类型和屏蔽状态 */
static void DeviceDisableHmiProcessQuery(const uint8_t *query_text)
{
    DeviceIdentity identity;
    DeviceInformation info;
    uint8_t code[6];
    uint8_t line[96];
    SetTextValue(70U, 207U, (uint8_t *)query_text);
    if(!DeviceCodeParse(query_text, &identity))
    {
        SetTextValue(70U, 22U, (uint8_t *)(GBK_DEVICE_CODE GBK_INVALID_ADDR));
        SetTextValue(70U, 23U, (uint8_t *)(GBK_DEVICE_TYPE GBK_NOT_FOUND));
        SetTextValue(70U, 24U, (uint8_t *)(GBK_DISABLE_STATE GBK_UNAVAILABLE));
        return;
    }
    DeviceCodeFormat(&identity, code);
    snprintf((char *)line, sizeof(line), "%s%s", GBK_DEVICE_CODE, code);
    SetTextValue(70U, 22U, line);
    if(!DeviceRegistryQuery(&identity, &info))
    {
        SetTextValue(70U, 23U, (uint8_t *)(GBK_DEVICE_TYPE GBK_NOT_FOUND));
        SetTextValue(70U, 24U, (uint8_t *)(GBK_DISABLE_STATE GBK_UNAVAILABLE));
        return;
    }
    snprintf((char *)line, sizeof(line), "%s%s", GBK_DEVICE_TYPE, DeviceDisableTypeTextGbk(info.type));
    SetTextValue(70U, 23U, line);
    snprintf((char *)line, sizeof(line), "%s%s", GBK_DISABLE_STATE,
             (identity.loop_id == DEVICE_DISABLE_LOOP1 || identity.loop_id == DEVICE_DISABLE_LOOP3) ?
             (info.disabled ? GBK_DISABLED : GBK_NOT_DISABLED) : GBK_DISABLE_UNSUP);
    SetTextValue(70U, 24U, line);
}

/* 执行屏蔽/解除操作并在画面70显示结果(操作=1设置屏蔽, =2解除屏蔽) */
static void DeviceDisableHmiProcessOperation(const DeviceIdentity *identity, uint8_t operation)
{
    DeviceInformation info;
    DeviceDisableResult result;
    memset(&info, 0, sizeof(info));
    DeviceRegistryQuery(identity, &info);
    result = operation == 1U ? DeviceDisableSet(identity) : DeviceDisableClear(identity);
    DeviceDisableHmiPush(identity, info.type,
                         result == DEVICE_DISABLE_OK ? (operation == 1U ? GBK_SET_OK : GBK_CLEAR_OK) :
                         DeviceDisableResultTextGbk(result));
}

/* 清除画面57指定行的4列文本(序号/设备/时间/类型) */
static void DeviceDisableHistoryHmiClearRow(uint8_t row)
{
    clearTextValue(57U, (uint16_t)(6U + row));
    clearTextValue(57U, (uint16_t)(16U + row));
    clearTextValue(57U, (uint16_t)(26U + row));
    clearTextValue(57U, (uint16_t)(36U + row));
}

/* 刷新画面57屏蔽历史: 按页码从Flash读取记录并显示(序号/设备/时间/类型) */
static void DeviceDisableHistoryHmiRefresh(void)
{
    uint16_t total = DeviceDisableHistoryCount();
    uint16_t total_pages = total == 0U ? 1U : (uint16_t)((total + 9U) / 10U);
    uint16_t start;
    uint8_t row;
    if(g_history_hmi_page < 1U) g_history_hmi_page = 1U;
    if(g_history_hmi_page > total_pages) g_history_hmi_page = total_pages;
    start = (uint16_t)((g_history_hmi_page - 1U) * 10U);
    SetTextInt32(57U, 97U, g_history_hmi_page, 0U, 1U);
    SetTextInt32(57U, 99U, total, 0U, 1U);
    for(row = 0U; row < 10U; row++)
    {
        uint16_t display_index = (uint16_t)(start + row);
        DeviceDisableHistoryRecord record;
        uint8_t text[48];
        uint32_t packed;
        if(display_index >= total ||
           !DeviceDisableHistoryRead((uint16_t)(total - 1U - display_index), &record))
        {
            DeviceDisableHistoryHmiClearRow(row);
            continue;
        }
        snprintf((char *)text, sizeof(text), "%03u", (unsigned int)(display_index + 1U));
        SetTextValue(57U, (uint16_t)(6U + row), text);
        snprintf((char *)text, sizeof(text), "%s%u%s%u%s", GBK_DEVICE_LOOP,
                 record.loop_id, GBK_DEVICE_LOOP_END, record.address, GBK_DEVICE_NUMBER);
        SetTextValue(57U, (uint16_t)(16U + row), text);
        packed = DeviceDisableHistoryUnpackTime(record.time);
        if(((packed >> 22) & 0x0FU) < 1U || ((packed >> 22) & 0x0FU) > 12U ||
           ((packed >> 17) & 0x1FU) < 1U || ((packed >> 17) & 0x1FU) > 31U ||
           ((packed >> 12) & 0x1FU) > 23U || ((packed >> 6) & 0x3FU) > 59U ||
           (packed & 0x3FU) > 59U)
        {
            snprintf((char *)text, sizeof(text), "%s", GBK_INVALID_TIME);
        }
        else
        {
            snprintf((char *)text, sizeof(text), "20%02lu%s%02lu%s%02lu%s %02lu:%02lu:%02lu",
                     (unsigned long)((packed >> 26) & 0x3FU), GBK_YEAR_SUFFIX,
                     (unsigned long)((packed >> 22) & 0x0FU), GBK_MONTH_SUFFIX,
                     (unsigned long)((packed >> 17) & 0x1FU), GBK_DAY_SUFFIX,
                     (unsigned long)((packed >> 12) & 0x1FU),
                     (unsigned long)((packed >> 6) & 0x3FU),
                     (unsigned long)(packed & 0x3FU));
        }
        SetTextValue(57U, (uint16_t)(26U + row), text);
        SetTextValue(57U, (uint16_t)(36U + row),
                     (uint8_t *)(record.event_type == DEVICE_DISABLE_EVENT_SET ?
                                 GBK_HISTORY_SET : GBK_HISTORY_CLEAR));
    }
}

/* ========================================================================
 * 公共HMI接口: 由cmd_process.c的UpdateUI/NotifyButton/NotifyText调用
 * ======================================================================== */

/* 画面切换时更新: 画面6更新屏蔽计数, 画面57/70执行初始化或刷新 */
void DeviceDisableHmiScreenUpdate(uint16_t screen_id)
{
    if(screen_id == 6U)
    {
        uint8_t count_text[32];
        snprintf((char *)count_text, sizeof(count_text), "%s%u", GBK_DISABLED_COUNT,
                 DeviceDisableGetLoopCount(DEVICE_DISABLE_LOOP1));
        SetTextValue(6U, 11U, count_text);
        snprintf((char *)count_text, sizeof(count_text), "%s0", GBK_DISABLED_COUNT);
        SetTextValue(6U, 17U, count_text);
        snprintf((char *)count_text, sizeof(count_text), "%s%u", GBK_DISABLED_COUNT,
                 DeviceDisableGetLoopCount(DEVICE_DISABLE_LOOP3));
        SetTextValue(6U, 24U, count_text);
    }
    shielding_state = g_disable_state.disabled_count != 0U;
    if(screen_id == 57U && g_history_hmi_active)
    {
        if(g_history_hmi_refresh_pending)
        {
            g_history_hmi_refresh_pending = 0U;
            DeviceDisableHistoryHmiRefresh();
        }
    }
    else if(screen_id == 70U)
    {
        if(!g_hmi_page_active)
        {
            g_hmi_page_active = 1U;
            g_hmi_loop_id = DEVICE_DISABLE_LOOP1;
            g_hmi_address = 0U;
            memset(g_hmi_address_text, 0, sizeof(g_hmi_address_text));
            memset(g_hmi_query_text, 0, sizeof(g_hmi_query_text));
            SetTextValue(70U, 202U, (uint8_t *)GBK_LOOP1_TEXT);
            SetTextValue(70U, 204U, g_hmi_address_text);
            SetTextValue(70U, 207U, g_hmi_query_text);
            DeviceDisableHmiLoadCurrent();
        }
        if(g_hmi_menu_refresh_pending)
        {
            g_hmi_menu_refresh_pending = 0U;
            SetTextValue(70U, 202U, (uint8_t *)(g_hmi_loop_id == DEVICE_DISABLE_LOOP3 ? GBK_LOOP3_TEXT : GBK_LOOP1_TEXT));
        }
        if(g_hmi_address_refresh_pending)
        {
            g_hmi_address_refresh_pending = 0U;
            SetTextValue(70U, 204U, g_hmi_address_text);
        }
        if(g_hmi_query_pending)
        {
            uint8_t query_text[sizeof(g_hmi_query_text)];
            memcpy(query_text, g_hmi_query_text, sizeof(query_text));
            g_hmi_query_pending = 0U;
            DeviceDisableHmiProcessQuery(query_text);
        }
        if(g_hmi_operation_pending)
        {
            DeviceIdentity identity = g_hmi_operation_identity;
            uint8_t operation = g_hmi_operation;
            g_hmi_operation_pending = 0U;
            DeviceDisableHmiProcessOperation(&identity, operation);
        }
    }
    else
    {
        g_hmi_page_active = 0U;
        if(screen_id != 57U) g_history_hmi_active = 0U;
    }
}

/* 按钮响应: 画面57翻页(控件1=上页/2=下页), 画面70设置/解除(控件211/212) */
void DeviceDisableHmiButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    if(screen_id == 57U && g_history_hmi_active && state == 1U)
    {
        uint16_t total;
        uint16_t total_pages;
        if(control_id != 1U && control_id != 2U) return;
        total = DeviceDisableHistoryCount();
        total_pages = total == 0U ? 1U : (uint16_t)((total + 9U) / 10U);
        if(control_id == 1U && g_history_hmi_page > 1U) g_history_hmi_page--;
        if(control_id == 2U && g_history_hmi_page < total_pages) g_history_hmi_page++;
        g_history_hmi_refresh_pending = 1U;
        return;
    }
    if(screen_id != 70U || state != 1U || (control_id != 211U && control_id != 212U)) return;
    if(g_hmi_operation_pending) return;
    g_hmi_operation_identity.loop_id = g_hmi_loop_id;
    g_hmi_operation_identity.address = g_hmi_address;
    g_hmi_operation = control_id == 211U ? 1U : 2U;
    g_hmi_operation_pending = 1U;
}

/* 菜单选择响应: 画面68控件20索引2→跳转画面57, 画面70控件300→切换回路 */
void DeviceDisableHmiMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state)
{
    (void)state;
    if(screen_id == 68U && control_id == 20U && item == 2U && state == 1U)
    {
        g_history_hmi_active = 1U;
        g_history_hmi_page = 1U;
        g_history_hmi_refresh_pending = 1U;
        SwitchCurrentScreenId(57U);
        return;
    }
    if(screen_id != 70U || control_id != 300U) return;
    if(item == 0U) g_hmi_loop_id = DEVICE_DISABLE_LOOP1;
    else if(item == 1U) g_hmi_loop_id = DEVICE_DISABLE_LOOP3;
    else return;
    g_hmi_menu_refresh_pending = 1U;
}

/* 文本输入响应: 画面70控件204(地址输入)→更新地址, 控件207(编号查询)→触发查询 */
void DeviceDisableHmiText(uint16_t screen_id, uint16_t control_id, const uint8_t *text)
{
    if(screen_id != 70U || text == NULL) return;
    if(control_id == 204U)
    {
        unsigned long value = strtoul((const char *)text, NULL, 10);
        g_hmi_address = value <= DEVICE_DISABLE_MAX_ADDRESS ? (uint16_t)value : 0U;
        snprintf((char *)g_hmi_address_text, sizeof(g_hmi_address_text), "%u", g_hmi_address);
        g_hmi_address_refresh_pending = 1U;
        return;
    }
    if(control_id != 207U) return;
    memset(g_hmi_query_text, 0, sizeof(g_hmi_query_text));
    strncpy((char *)g_hmi_query_text, (const char *)text, sizeof(g_hmi_query_text) - 1U);
    g_hmi_query_pending = 1U;
}

/* 文本设置过滤器: 画面6控件11/17/24(屏蔽计数)由本模块管理, 拦截后自行设置 */
void DeviceDisableHmiSetTextFilter(uint16_t screen_id, uint16_t control_id, uint8_t *text)
{
    if(screen_id == 6U && (control_id == 11U || control_id == 17U || control_id == 24U)) return;
    SetTextValue(screen_id, control_id, text);
}

