#ifndef __BSP_DEVICE_DISABLE_H
#define __BSP_DEVICE_DISABLE_H

#include "main.h"

/* ============================================================================
 * 模块名称: 设备屏蔽管理模块 (Device Disable Management)
 * 功能描述: 提供回路1/回路3探测器的屏蔽/解除屏蔽功能,
 *          包括屏蔽状态持久化、屏蔽历史记录、HMI界面交互。
 * 屏蔽规则: 仅允许对正常在线且类型已识别的设备设置屏蔽,
 *          预警/火警/故障/掉线/未配置/未知类型均拒绝屏蔽。
 * 屏蔽后保留总线轮询和原始数据,隔离报警/故障/掉线业务事件。
 * 解除后由下一业务周期立即重评当前状态。
 * ============================================================================ */

/* 支持的回路编号 */
#define DEVICE_DISABLE_LOOP1             1U   /* 回路1: 二总线点型探测器 */
#define DEVICE_DISABLE_LOOP3             3U   /* 回路3: RS485复合探测器 */
#define DEVICE_DISABLE_MAX_ADDRESS       100U /* 单回路最大地址 */
#define DEVICE_DISABLE_RECENT_MAX        20U  /* 最近屏蔽列表容量 */

/* 设备注册类型 */
typedef enum
{
    DEVICE_TYPE_UNKNOWN = 0,       /* 未知类型 */
    DEVICE_TYPE_SMOKE,             /* 烟雾探测器 */
    DEVICE_TYPE_TEMPERATURE,       /* 温度探测器 */
    DEVICE_TYPE_MULTI_SENSOR       /* 复合探测器(多传感器) */
} DeviceRegistryType;

/* 设备运行时状态 */
typedef enum
{
    DEVICE_RUNTIME_NORMAL = 0,     /* 正常 */
    DEVICE_RUNTIME_WARNING,        /* 预警 */
    DEVICE_RUNTIME_FIRE,           /* 火警 */
    DEVICE_RUNTIME_FAULT,          /* 故障 */
    DEVICE_RUNTIME_OFFLINE         /* 掉线 */
} DeviceRuntimeState;

/* 设备身份标识(回路号+地址) */
typedef struct
{
    uint8_t loop_id;               /* 回路编号 */
    uint16_t address;              /* 设备地址 */
} DeviceIdentity;

/* 设备完整信息 */
typedef struct
{
    uint8_t exists;                /* 是否存在 */
    uint8_t configured;            /* 是否已配置上线 */
    uint8_t online;                /* 是否在线 */
    uint8_t disabled;              /* 是否已屏蔽 */
    DeviceIdentity identity;       /* 设备标识 */
    DeviceRegistryType type;       /* 设备类型 */
    DeviceRuntimeState state;      /* 运行时状态 */
} DeviceInformation;

/* 屏蔽操作结果码 */
typedef enum
{
    DEVICE_DISABLE_OK = 0,             /* 操作成功 */
    DEVICE_DISABLE_INVALID_LOOP,       /* 回路不支持 */
    DEVICE_DISABLE_INVALID_ADDRESS,    /* 地址无效 */
    DEVICE_DISABLE_NOT_CONFIGURED,     /* 设备未配置 */
    DEVICE_DISABLE_UNSUPPORTED_TYPE,   /* 设备类型不支持 */
    DEVICE_DISABLE_ALREADY_SET,        /* 已处于屏蔽状态 */
    DEVICE_DISABLE_NOT_SET,            /* 当前未屏蔽 */
    DEVICE_DISABLE_WARNING,            /* 设备正在预警,禁止屏蔽 */
    DEVICE_DISABLE_FIRE,               /* 设备正在火警,禁止屏蔽 */
    DEVICE_DISABLE_FAULT,              /* 设备正在故障,禁止屏蔽 */
    DEVICE_DISABLE_OFFLINE,            /* 设备掉线,禁止屏蔽 */
    DEVICE_DISABLE_STORAGE_ERROR       /* Flash存储失败 */
} DeviceDisableResult;

/* 最近屏蔽记录条目 */
typedef struct
{
    DeviceIdentity identity;       /* 设备标识 */
    uint8_t device_type;           /* 设备类型 */
    uint32_t disabled_time;        /* 屏蔽时间(压缩格式) */
} DeviceDisableRecent;

/* ---- 初始化 ---- */
void DeviceDisable_Init(void);

/* ---- 设备编号解析/格式化 ---- */
uint8_t DeviceCodeParse(const uint8_t *text, DeviceIdentity *identity);
void DeviceCodeFormat(const DeviceIdentity *identity, uint8_t output[6]);

/* ---- 设备注册表查询 ---- */
uint8_t DeviceRegistryQuery(const DeviceIdentity *identity, DeviceInformation *information);

/* ---- 屏蔽操作 ---- */
DeviceDisableResult DeviceDisableSet(const DeviceIdentity *identity);
DeviceDisableResult DeviceDisableClear(const DeviceIdentity *identity);

/* ---- 屏蔽状态查询 ---- */
uint8_t DeviceDisableIsSet(const DeviceIdentity *identity);
uint8_t DeviceDisableIsLoopAddressSet(uint8_t loop_id, uint16_t address);
uint16_t DeviceDisableGetCount(void);
uint16_t DeviceDisableGetLoopCount(uint8_t loop_id);
uint8_t DeviceDisableGetRecent(DeviceDisableRecent *output, uint8_t max_count);

/* ---- 文本转换(英文) ---- */
const char *DeviceDisableTypeText(DeviceRegistryType type);
const char *DeviceDisableResultText(DeviceDisableResult result);

/* ---- HMI界面交互(由cmd_process.c调用) ---- */
void DeviceDisableHmiScreenUpdate(uint16_t screen_id);
void DeviceDisableHmiButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
void DeviceDisableHmiMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state);
void DeviceDisableHmiText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);
void DeviceDisableHmiSetTextFilter(uint16_t screen_id, uint16_t control_id, uint8_t *text);

#endif

