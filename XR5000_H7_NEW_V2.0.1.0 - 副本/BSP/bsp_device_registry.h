#ifndef __BSP_DEVICE_REGISTRY_H
#define __BSP_DEVICE_REGISTRY_H

#include <stdint.h>

/* 三条设备回路编号。产品表通过位掩码声明某种产品允许接入哪些回路。 */
#define DEVICE_REGISTRY_LOOP1  1U
#define DEVICE_REGISTRY_LOOP2  2U
#define DEVICE_REGISTRY_LOOP3  3U

#define DEVICE_REGISTRY_LOOP_MASK(loop) ((uint8_t)(1U << ((loop) - 1U)))

typedef enum
{
    /* 0x000E输入寄存器返回的内部产品码。地址不再参与设备类型判断。 */
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
    /* 各回路识别产品后使用的业务解析器类型。 */
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

typedef enum
{
    /* 设备识别阶段产生的运行时故障，只用于实时显示和计数，不写入Flash。 */
    DEVICE_IDENTIFY_OK = 0,
    DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE,
    DEVICE_IDENTIFY_NATIONAL_UNKNOWN,
    DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE,
    DEVICE_IDENTIFY_PRODUCT_UNKNOWN,
    DEVICE_IDENTIFY_CODE_MISMATCH,
    DEVICE_IDENTIFY_SENSOR_READ_FAILED,
    DEVICE_IDENTIFY_SENSOR_TYPE_UNKNOWN
} DeviceIdentifyError;

typedef struct
{
    uint16_t product_code;         /* 0x000E内部产品码，也是本表的查找键 */
    const char *name;              /* 产品名称，主要用于调试和详情显示 */
    uint8_t loop_mask;             /* 允许接入的回路位掩码 */
    uint8_t parser_type;           /* 识别成功后采用的状态数据解析器 */
    uint8_t register_count;        /* 正常轮询从0x0000开始读取的寄存器数量 */
    uint16_t national_type_code;   /* 0x000D期望值；0表示该产品暂不校验国标码 */
    uint16_t sensor_mask_allowed;  /* 0x000F允许出现的位；0表示无需读取0x000F */
} DeviceProductDescriptor;

/* 产品表查询及产品属性读取接口。找不到产品时返回空指针或安全默认值。 */
const DeviceProductDescriptor *DeviceRegistry_Find(uint16_t product_code); /* 按0x000E内部产品码查表，失败返回NULL */
uint8_t DeviceRegistry_IsSupportedOnLoop(uint16_t product_code, uint8_t loop); /* 判断产品是否允许接入指定回路 */
uint8_t DeviceRegistry_GetParserType(uint16_t product_code); /* 获取产品对应的业务解析器类型 */
uint8_t DeviceRegistry_GetRegisterCount(uint16_t product_code); /* 获取正常轮询读取的寄存器数量 */
const char *DeviceRegistry_GetName(uint16_t product_code); /* 获取产品名称，未知时返回"Unknown" */
uint16_t DeviceRegistry_GetNationalTypeCode(uint16_t product_code); /* 获取0x000D期望值，0表示暂不校验 */
uint8_t DeviceRegistry_RequiresSensorMask(uint16_t product_code); /* 判断是否需要读取0x000F */
uint8_t DeviceRegistry_IsSensorMaskValid(uint16_t product_code, uint16_t sensor_mask); /* 校验0x000F传感器位是否合法 */
uint8_t DeviceRegistry_IsNationalTypeKnown(uint16_t national_type_code); /* 判断国标设备类型码是否已登记 */
uint8_t DeviceRegistry_IsNationalProductMatch(uint16_t national_type_code, uint16_t product_code); /* 检查国标码与内部产品码是否匹配 */
const char *DeviceRegistry_GetNameByNationalCode(uint16_t national_type_code); /* 按0x000D国标设备类型码返回中文显示名，未知返回"未知设备" */

/* 按“回路+设备地址”保存、查询和枚举当前识别故障。 */
void DeviceRegistry_SetIdentifyError(uint8_t loop, uint8_t address, DeviceIdentifyError error); /* 设置识别故障，传入OK清除 */
DeviceIdentifyError DeviceRegistry_GetIdentifyError(uint8_t loop, uint8_t address); /* 获取指定设备的识别故障 */
uint8_t DeviceRegistry_GetIdentifyErrorAt(uint16_t index, uint8_t *loop, uint8_t *address,
                                          DeviceIdentifyError *error); /* 按显示顺序取得故障位置和原因 */
const char *DeviceRegistry_GetIdentifyErrorText(DeviceIdentifyError error); /* 获取供HMI显示的GBK故障文字 */

/* 兼容旧调用名称：这里的ProductUnknown现表示任意一种设备识别故障。 */
void DeviceRegistry_SetProductUnknown(uint8_t loop, uint8_t address, uint8_t state); /* 兼容设置接口，state为0时清除 */
uint8_t DeviceRegistry_IsProductUnknown(uint8_t loop, uint8_t address); /* 兼容查询接口：是否存在识别故障 */
uint16_t DeviceRegistry_GetProductUnknownCount(void); /* 统计三个回路的识别故障总数 */
uint8_t DeviceRegistry_GetProductUnknownCountByLoop(uint8_t loop); /* 统计指定回路的识别故障数 */
uint8_t DeviceRegistry_GetProductUnknownAt(uint16_t index, uint8_t *loop, uint8_t *address); /* 按顺序取得故障回路和地址 */

#endif
