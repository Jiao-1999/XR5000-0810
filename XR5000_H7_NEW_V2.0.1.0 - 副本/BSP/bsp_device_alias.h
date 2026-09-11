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

DeviceAliasLookupResult DeviceAlias_Get(uint8_t loop_id, uint8_t address,
                                        uint16_t product_code, uint8_t *name,
                                        uint8_t name_size);
DeviceAliasResult DeviceAlias_Set(uint8_t loop_id, uint8_t address,
                                  uint16_t product_code, const uint8_t *name);
DeviceAliasResult DeviceAlias_Clear(uint8_t loop_id, uint8_t address);
DeviceAliasResult DeviceAlias_ClearAll(void);
DeviceAliasResult DeviceAlias_ValidateName(const uint8_t *name);
uint32_t DeviceAlias_GetRevision(void);

uint8_t DeviceAliasDevice_IsAvailable(uint8_t loop_id, uint8_t address);
uint16_t DeviceAliasDevice_GetProductCode(uint8_t loop_id, uint8_t address);
const char *DeviceAliasDevice_GetTypeText(uint8_t loop_id, uint8_t address);
void DeviceAliasHmiScreenUpdate(uint16_t screen_id);
void DeviceAliasHmiButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
void DeviceAliasHmiText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);

#endif
