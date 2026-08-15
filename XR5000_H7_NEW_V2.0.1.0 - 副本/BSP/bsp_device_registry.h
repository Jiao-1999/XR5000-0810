#ifndef __BSP_DEVICE_REGISTRY_H
#define __BSP_DEVICE_REGISTRY_H

#include <stdint.h>

#define DEVICE_REGISTRY_LOOP1  1U
#define DEVICE_REGISTRY_LOOP2  2U
#define DEVICE_REGISTRY_LOOP3  3U

#define DEVICE_REGISTRY_LOOP_MASK(loop) ((uint8_t)(1U << ((loop) - 1U)))

typedef enum
{
    DEVICE_PRODUCT_UNKNOWN       = 0,
    DEVICE_PRODUCT_XR805_V20     = 1,
    DEVICE_PRODUCT_XR805_EXD     = 2,
    DEVICE_PRODUCT_XR805_EXI     = 3,
    DEVICE_PRODUCT_DLYGWG        = 4,
    DEVICE_PRODUCT_XR8001_SMOKE  = 5,
    DEVICE_PRODUCT_XR8002_TEMP   = 6,
    DEVICE_PRODUCT_XR8303        = 7,
    DEVICE_PRODUCT_XR8305        = 8,
    DEVICE_PRODUCT_XR2200        = 9,
    DEVICE_PRODUCT_SGBJQ         = 10,
    DEVICE_PRODUCT_XR1503        = 11,
    DEVICE_PRODUCT_GCM1002       = 12,
    DEVICE_PRODUCT_FIM1017       = 13
} DeviceProductCode;

typedef enum
{
    DEVICE_PARSER_NONE = 0,
    DEVICE_PARSER_XR805,
    DEVICE_PARSER_XR8001,
    DEVICE_PARSER_XR8002,
    DEVICE_PARSER_XR8303,
    DEVICE_PARSER_XR8305,
    DEVICE_PARSER_XR2200,
    DEVICE_PARSER_SGBJQ,
    DEVICE_PARSER_XR1503,
    DEVICE_PARSER_GCM1002,
    DEVICE_PARSER_FIM1017,
    DEVICE_PARSER_DLYGWG
} DeviceParserType;

typedef struct
{
    uint16_t product_code;
    const char *name;
    uint8_t loop_mask;
    uint8_t parser_type;
    uint8_t register_count;
} DeviceProductDescriptor;

const DeviceProductDescriptor *DeviceRegistry_Find(uint16_t product_code);
uint8_t DeviceRegistry_IsSupportedOnLoop(uint16_t product_code, uint8_t loop);
uint8_t DeviceRegistry_GetParserType(uint16_t product_code);
uint8_t DeviceRegistry_GetRegisterCount(uint16_t product_code);
const char *DeviceRegistry_GetName(uint16_t product_code);

void DeviceRegistry_SetProductUnknown(uint8_t loop, uint8_t address, uint8_t state);
uint8_t DeviceRegistry_IsProductUnknown(uint8_t loop, uint8_t address);
uint16_t DeviceRegistry_GetProductUnknownCount(void);
uint8_t DeviceRegistry_GetProductUnknownCountByLoop(uint8_t loop);
uint8_t DeviceRegistry_GetProductUnknownAt(uint16_t index, uint8_t *loop, uint8_t *address);

#endif
