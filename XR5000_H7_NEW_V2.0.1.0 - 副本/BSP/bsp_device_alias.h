/*
 * 设备中文名称模块公共接口。
 *
 * 名称以“回路 + 地址 + 产品码”为索引，最多13个中文、英文字母、数字或空格字符。
 * 本接口同时向业务层提供名称存取结果、设备有效性查询以及画面82的交互入口。
 */

#ifndef __BSP_DEVICE_ALIAS_H
#define __BSP_DEVICE_ALIAS_H

#include "main.h"

#define DEVICE_ALIAS_MAX_ADDRESS 100U
#define DEVICE_ALIAS_MAX_CHARS   13U
#define DEVICE_ALIAS_NAME_BYTES  28U

typedef enum
{
    DEVICE_ALIAS_NONE = 0,
    DEVICE_ALIAS_MATCH,
    DEVICE_ALIAS_PRODUCT_MISMATCH
} DeviceAliasLookupResult;

typedef enum
{
    DEVICE_ALIAS_OK = 0,
    DEVICE_ALIAS_INVALID_DEVICE,
    DEVICE_ALIAS_EMPTY_NAME,
    DEVICE_ALIAS_NAME_TOO_LONG,
    DEVICE_ALIAS_INVALID_CHARACTER,
    DEVICE_ALIAS_STORAGE_ERROR
} DeviceAliasResult;

/* 查询名称；返回无名称、匹配或产品码不匹配，并在匹配时复制到name。 */
DeviceAliasLookupResult DeviceAlias_Get(uint8_t loop_id, uint8_t address,
                                        uint16_t product_code, uint8_t *name,
                                        uint8_t name_size);
/* 设置单台设备的中文名称并持久化。 */
DeviceAliasResult DeviceAlias_Set(uint8_t loop_id, uint8_t address,
                                  uint16_t product_code, const uint8_t *name);
/* 清除单台设备的中文名称。 */
DeviceAliasResult DeviceAlias_Clear(uint8_t loop_id, uint8_t address);
/* 清除全部回路保存的中文名称。 */
DeviceAliasResult DeviceAlias_ClearAll(void);
/* 校验名称长度和字符范围。 */
DeviceAliasResult DeviceAlias_ValidateName(const uint8_t *name);
/* 获取名称数据修订号，供界面刷新判断使用。 */
uint32_t DeviceAlias_GetRevision(void);

/* 判断指定回路和地址的设备当前是否可用于查询或命名。 */
uint8_t DeviceAliasDevice_IsAvailable(uint8_t loop_id, uint8_t address);
/* 获取指定设备当前识别出的产品码，无法获取时返回0。 */
uint16_t DeviceAliasDevice_GetProductCode(uint8_t loop_id, uint8_t address);
/* 获取指定设备用于界面显示的中文类型名称。 */
const char *DeviceAliasDevice_GetTypeText(uint8_t loop_id, uint8_t address);
/* 处理画面82的进入、退出、异步保存及清除任务。 */
void DeviceAliasHmiScreenUpdate(uint16_t screen_id);
/* 处理画面82的查询、上一台、下一台、保存和清除按钮。 */
void DeviceAliasHmiButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
/* 接收画面82返回的设备编号或中文名称文本。 */
void DeviceAliasHmiText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);

#endif
