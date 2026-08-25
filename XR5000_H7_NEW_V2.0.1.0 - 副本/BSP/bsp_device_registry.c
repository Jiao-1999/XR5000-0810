#include "bsp_device_registry.h"

#define DEVICE_REGISTRY_MAX_ADDRESS  100U
#define LOOP1_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP1)
#define LOOP2_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP2)
#define LOOP3_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP3)

/*
 * 统一产品登记表，以下标无关的product_code作为查找键。
 * 字段顺序：内部产品码、名称、允许回路、解析器、正常读取数量、国标码、传感器掩码、
 *           通用能力、输出通道、控制驱动。
 * DLYGWG和GCM1002的national_type_code暂为0：仍读取0x000D，但识别出产品后跳过匹配。
 */
static const DeviceProductDescriptor g_product_table[] =
{
    {DEVICE_PRODUCT_XR805_V20,    "XR805-V2.0", LOOP3_MASK, DEVICE_PARSER_XR805,  16U, 50U, 0x003FU, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR805_EXD,    "XR805-EXD",  LOOP3_MASK, DEVICE_PARSER_XR805,  16U, 50U, 0x003FU, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR805_EXI,    "XR805-EXi",  LOOP3_MASK, DEVICE_PARSER_XR805,  16U, 50U, 0x003FU, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_DLYGWG,       "XR-DLYGWG",  LOOP3_MASK, DEVICE_PARSER_DLYGWG, 1U,  0U, 0U, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR8001_SMOKE, "JTY-XR8001", LOOP1_MASK, DEVICE_PARSER_XR8001, 4U, 23U, 0U, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR8002_TEMP,  "JTY-XR8002", LOOP1_MASK, DEVICE_PARSER_XR8002, 4U, 31U, 0U, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR8303,       "XR8303",     LOOP3_MASK, DEVICE_PARSER_XR8303, 12U, 50U, 0x007FU, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR8305,       "XR8305",     LOOP3_MASK, DEVICE_PARSER_XR8305, 12U, 50U, 0x007FU, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_XR2200,       "XR-2200",    LOOP2_MASK, DEVICE_PARSER_XR2200,  1U, 61U, 0U,
        DEVICE_CAP_STATUS_READ, 0U, DEVICE_CONTROL_DRIVER_NONE},
    /* 当前协议只支持声光整体启停，因此只登记一个组合输出通道；后续协议支持拆分时再扩展位。 */
    {DEVICE_PRODUCT_SGBJQ,        "XR-SGBJQ",   LOOP2_MASK, DEVICE_PARSER_SGBJQ,   1U, 82U, 0U,
        DEVICE_CAP_STATUS_READ | DEVICE_CAP_OUTPUT_CONTROL, DEVICE_OUTPUT_1, DEVICE_CONTROL_DRIVER_SGBJQ},
    {DEVICE_PRODUCT_XR1503,       "XR1503",     LOOP2_MASK, DEVICE_PARSER_XR1503,  1U, 10U, 0U,
        DEVICE_CAP_STATUS_READ | DEVICE_CAP_EVENT_RECEIVE, 0U, DEVICE_CONTROL_DRIVER_FIRE_DISPLAY},
    {DEVICE_PRODUCT_GCM1002,      "GCM-1002",   LOOP2_MASK, DEVICE_PARSER_GCM1002, 1U,  0U, 0U, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE},
    {DEVICE_PRODUCT_FIM1017,      "FIM-1017",   LOOP2_MASK, DEVICE_PARSER_FIM1017, 1U, 76U, 0U, 0U, 0U, DEVICE_CONTROL_DRIVER_NONE}
};

/* 运行时识别故障表：[回路][真实设备地址]，掉电清除且不写入外部Flash。 */
static uint8_t g_identify_error[4U][DEVICE_REGISTRY_MAX_ADDRESS + 1U];

/* 根据0x000E返回值顺序遍历产品表；找到后返回对应表项，只读使用。 */
const DeviceProductDescriptor *DeviceRegistry_Find(uint16_t product_code)
{
    uint32_t index;
    for(index = 0U; index < (sizeof(g_product_table) / sizeof(g_product_table[0])); index++)
    {
        if(g_product_table[index].product_code == product_code)
        {
            return &g_product_table[index];
        }
    }
    return 0;
}

uint8_t DeviceRegistry_IsSupportedOnLoop(uint16_t product_code, uint8_t loop)
{
    /* 同一内部产品码必须同时存在于表中，并允许挂接在指定回路。 */
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    if(descriptor == 0 || loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3)
    {
        return 0U;
    }
    return (descriptor->loop_mask & DEVICE_REGISTRY_LOOP_MASK(loop)) != 0U ? 1U : 0U;
}

uint8_t DeviceRegistry_GetParserType(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? DEVICE_PARSER_NONE : descriptor->parser_type;
}

uint8_t DeviceRegistry_GetRegisterCount(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? 0U : descriptor->register_count;
}

const char *DeviceRegistry_GetName(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? "Unknown" : descriptor->name;
}

uint16_t DeviceRegistry_GetNationalTypeCode(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? 0U : descriptor->national_type_code;
}

uint8_t DeviceRegistry_RequiresSensorMask(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor != 0 && descriptor->sensor_mask_allowed != 0U ? 1U : 0U;
}

uint8_t DeviceRegistry_IsSensorMaskValid(uint16_t product_code, uint16_t sensor_mask)
{
    /* 0合法；只要没有设置产品未定义的位，就认为0x000F有效。 */
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor != 0 && descriptor->sensor_mask_allowed != 0U &&
           (sensor_mask & (uint16_t)~descriptor->sensor_mask_allowed) == 0U ? 1U : 0U;
}

uint8_t DeviceRegistry_IsNationalTypeKnown(uint16_t national_type_code)
{
    /* 国标码可能被多个产品共用，例如805、8303和8305均为50。 */
    uint32_t index;
    for(index = 0U; index < (sizeof(g_product_table) / sizeof(g_product_table[0])); index++)
        if(g_product_table[index].national_type_code == national_type_code && national_type_code != 0U) return 1U;
    return 0U;
}

uint8_t DeviceRegistry_IsNationalProductMatch(uint16_t national_type_code, uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    if(descriptor == 0) return 0U;
    /* 表内国标码为0的产品按当前约定暂不做一致性校验。 */
    return descriptor->national_type_code == 0U || descriptor->national_type_code == national_type_code ? 1U : 0U;
}

uint32_t DeviceRegistry_GetCapabilities(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? 0U : descriptor->capabilities;
}

uint32_t DeviceRegistry_GetSupportedOutputs(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? 0U : descriptor->supported_outputs;
}

uint8_t DeviceRegistry_GetControlDriver(uint16_t product_code)
{
    const DeviceProductDescriptor *descriptor = DeviceRegistry_Find(product_code);
    return descriptor == 0 ? DEVICE_CONTROL_DRIVER_NONE : descriptor->control_driver;
}

/* 国标设备类型码 → 屏幕显示中文名映射表（供联动逻辑规则显示使用）。
 * 命名按GB4717标准；10(消防泵控制)与76(防排烟)待确认暂不登记，未知码统一返回"未知设备"。 */
static const struct
{
    uint16_t code;      /* 0x000D国标设备类型码 */
    const char *name;   /* GB标准中文名 */
} g_national_name_table[] =
{
    {23U, "感烟火灾探测器"},
    {31U, "感温火灾探测器"},
    {50U, "复合式火灾探测器"},
    {61U, "消防电气控制装置"},
    {82U, "声光报警器"}
};

const char *DeviceRegistry_GetNameByNationalCode(uint16_t national_type_code)
{
    uint32_t index;
    for(index = 0U; index < (sizeof(g_national_name_table) / sizeof(g_national_name_table[0])); index++)
    {
        if(g_national_name_table[index].code == national_type_code)
        {
            return g_national_name_table[index].name;
        }
    }
    return "未知设备";
}

void DeviceRegistry_SetIdentifyError(uint8_t loop, uint8_t address, DeviceIdentifyError error)
{
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 || address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS) return;
    g_identify_error[loop][address] = (uint8_t)error;
}

DeviceIdentifyError DeviceRegistry_GetIdentifyError(uint8_t loop, uint8_t address)
{
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 || address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS) return DEVICE_IDENTIFY_OK;
    return (DeviceIdentifyError)g_identify_error[loop][address];
}

void DeviceRegistry_SetProductUnknown(uint8_t loop, uint8_t address, uint8_t state)
{
    /* 保留该接口，避免三个回路的既有对外接口发生不必要变化。 */
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 ||
       address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS)
    {
        return;
    }
    g_identify_error[loop][address] = state != 0U ? (uint8_t)DEVICE_IDENTIFY_PRODUCT_UNKNOWN : (uint8_t)DEVICE_IDENTIFY_OK;
}

uint8_t DeviceRegistry_IsProductUnknown(uint8_t loop, uint8_t address)
{
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 ||
       address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS)
    {
        return 0U;
    }
    return g_identify_error[loop][address] != DEVICE_IDENTIFY_OK ? 1U : 0U;
}

uint16_t DeviceRegistry_GetProductUnknownCount(void)
{
    uint16_t count = 0U;
    uint8_t loop;
    uint8_t address;
    for(loop = DEVICE_REGISTRY_LOOP1; loop <= DEVICE_REGISTRY_LOOP3; loop++)
    {
        for(address = 1U; address <= DEVICE_REGISTRY_MAX_ADDRESS; address++)
        {
            count += g_identify_error[loop][address] != DEVICE_IDENTIFY_OK ? 1U : 0U;
        }
    }
    return count;
}

uint8_t DeviceRegistry_GetProductUnknownCountByLoop(uint8_t loop)
{
    uint8_t count = 0U;
    uint8_t address;
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3)
    {
        return 0U;
    }
    for(address = 1U; address <= DEVICE_REGISTRY_MAX_ADDRESS; address++)
    {
        if(g_identify_error[loop][address] != DEVICE_IDENTIFY_OK)
        {
            count++;
        }
    }
    return count;
}

uint8_t DeviceRegistry_GetProductUnknownAt(uint16_t index, uint8_t *loop, uint8_t *address)
{
    /* 画面59按回路1→2→3、地址小→大的顺序枚举实时识别故障。 */
    uint16_t current = 0U;
    uint8_t loop_index;
    uint8_t address_index;
    if(loop == 0 || address == 0)
    {
        return 0U;
    }
    for(loop_index = DEVICE_REGISTRY_LOOP1; loop_index <= DEVICE_REGISTRY_LOOP3; loop_index++)
    {
        for(address_index = 1U; address_index <= DEVICE_REGISTRY_MAX_ADDRESS; address_index++)
        {
            if(g_identify_error[loop_index][address_index] != DEVICE_IDENTIFY_OK)
            {
                if(current == index)
                {
                    *loop = loop_index;
                    *address = address_index;
                    return 1U;
                }
                current++;
            }
        }
    }
    return 0U;
}

uint8_t DeviceRegistry_GetIdentifyErrorAt(uint16_t index, uint8_t *loop, uint8_t *address, DeviceIdentifyError *error)
{
    if(DeviceRegistry_GetProductUnknownAt(index, loop, address) == 0U || error == 0) return 0U;
    *error = DeviceRegistry_GetIdentifyError(*loop, *address);
    return 1U;
}

const char *DeviceRegistry_GetIdentifyErrorText(DeviceIdentifyError error)
{
    /* HMI旧工程使用GBK，采用字节转义避免UTF-8源码字符串显示乱码。 */
    static const char national_no_response[] = "\xB9\xFA\xB1\xEA\xC9\xE8\xB1\xB8\xC2\xEB\xCE\xDE\xBB\xD8\xB8\xB4";
    static const char national_unknown[] = "\xB9\xFA\xB1\xEA\xC9\xE8\xB1\xB8\xC0\xE0\xD0\xCD\xCE\xB4\xD6\xAA";
    static const char product_no_response[] = "\xB2\xFA\xC6\xB7\xD0\xCD\xBA\xC5\xC2\xEB\xCE\xDE\xBB\xD8\xB8\xB4";
    static const char product_unknown[] = "\xB2\xFA\xC6\xB7\xD0\xCD\xBA\xC5\xCE\xB4\xD6\xAA";
    static const char mismatch[] = "\xC9\xE8\xB1\xB8\xCA\xB6\xB1\xF0\xC2\xEB\xB2\xBB\xC6\xA5\xC5\xE4";
    static const char sensor_read_failed[] = "\xB4\xAB\xB8\xD0\xC6\xF7\xD7\xB4\xCC\xAC\xB6\xC1\xC8\xA1\xCA\xA7\xB0\xDC";
    static const char sensor_unknown[] = "\xB4\xAB\xB8\xD0\xC6\xF7\xC6\xF4\xD3\xC3\xC0\xE0\xD0\xCD\xCE\xB4\xD6\xAA";
    switch(error)
    {
        case DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE: return national_no_response;
        case DEVICE_IDENTIFY_NATIONAL_UNKNOWN: return national_unknown;
        case DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE: return product_no_response;
        case DEVICE_IDENTIFY_PRODUCT_UNKNOWN: return product_unknown;
        case DEVICE_IDENTIFY_CODE_MISMATCH: return mismatch;
        case DEVICE_IDENTIFY_SENSOR_READ_FAILED: return sensor_read_failed;
        case DEVICE_IDENTIFY_SENSOR_TYPE_UNKNOWN: return sensor_unknown;
        default: return "";
    }
}
