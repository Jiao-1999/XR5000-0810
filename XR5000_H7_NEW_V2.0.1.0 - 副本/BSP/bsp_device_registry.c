#include "bsp_device_registry.h"

#define DEVICE_REGISTRY_MAX_ADDRESS  100U
#define LOOP1_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP1)
#define LOOP2_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP2)
#define LOOP3_MASK DEVICE_REGISTRY_LOOP_MASK(DEVICE_REGISTRY_LOOP3)

static const DeviceProductDescriptor g_product_table[] =
{
    {DEVICE_PRODUCT_XR805_V20,    "XR805-V2.0",  LOOP3_MASK, DEVICE_PARSER_XR805,   16U},
    {DEVICE_PRODUCT_XR805_EXD,    "XR805-EXD",   LOOP3_MASK, DEVICE_PARSER_XR805,   16U},
    {DEVICE_PRODUCT_XR805_EXI,    "XR805-EXi",   LOOP3_MASK, DEVICE_PARSER_XR805,   16U},
    {DEVICE_PRODUCT_DLYGWG,       "XR-DLYGWG",   LOOP3_MASK, DEVICE_PARSER_DLYGWG,   1U},
    {DEVICE_PRODUCT_XR8001_SMOKE, "JTY-XR8001",  LOOP1_MASK, DEVICE_PARSER_XR8001,   4U},
    {DEVICE_PRODUCT_XR8002_TEMP,  "JTY-XR8002",  LOOP1_MASK, DEVICE_PARSER_XR8002,   4U},
    {DEVICE_PRODUCT_XR8303,       "XR8303",      LOOP3_MASK, DEVICE_PARSER_XR8303,  12U},
    {DEVICE_PRODUCT_XR8305,       "XR8305",      LOOP3_MASK, DEVICE_PARSER_XR8305,  12U},
    {DEVICE_PRODUCT_XR2200,       "XR-2200",     LOOP2_MASK, DEVICE_PARSER_XR2200,   1U},
    {DEVICE_PRODUCT_SGBJQ,        "XR-SGBJQ",    LOOP2_MASK, DEVICE_PARSER_SGBJQ,    1U},
    {DEVICE_PRODUCT_XR1503,       "XR1503",      LOOP2_MASK, DEVICE_PARSER_XR1503,   1U},
    {DEVICE_PRODUCT_GCM1002,      "GCM-1002",    LOOP2_MASK, DEVICE_PARSER_GCM1002,  1U},
    {DEVICE_PRODUCT_FIM1017,      "FIM-1017",    LOOP2_MASK, DEVICE_PARSER_FIM1017,  1U}
};

/* Runtime-only faults. They are intentionally not stored in external Flash. */
static uint8_t g_product_unknown[4U][DEVICE_REGISTRY_MAX_ADDRESS + 1U];

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

void DeviceRegistry_SetProductUnknown(uint8_t loop, uint8_t address, uint8_t state)
{
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 ||
       address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS)
    {
        return;
    }
    g_product_unknown[loop][address] = state != 0U ? 1U : 0U;
}

uint8_t DeviceRegistry_IsProductUnknown(uint8_t loop, uint8_t address)
{
    if(loop < DEVICE_REGISTRY_LOOP1 || loop > DEVICE_REGISTRY_LOOP3 ||
       address == 0U || address > DEVICE_REGISTRY_MAX_ADDRESS)
    {
        return 0U;
    }
    return g_product_unknown[loop][address];
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
            count += g_product_unknown[loop][address] != 0U ? 1U : 0U;
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
        if(g_product_unknown[loop][address] != 0U)
        {
            count++;
        }
    }
    return count;
}

uint8_t DeviceRegistry_GetProductUnknownAt(uint16_t index, uint8_t *loop, uint8_t *address)
{
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
            if(g_product_unknown[loop_index][address_index] != 0U)
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
