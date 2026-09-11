#include "bsp_device_alias.h"

#include <string.h>
#include "w25qxx.h"

#define DEVICE_ALIAS_MAGIC          0x53414C44UL
#define DEVICE_ALIAS_VERSION        1U
#define DEVICE_ALIAS_LOOP_COUNT     3U

typedef struct
{
    uint16_t product_code;
    uint8_t name[DEVICE_ALIAS_NAME_BYTES];
} DeviceAliasRecord;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint8_t loop_id;
    uint8_t reserved;
    uint32_t sequence;
    DeviceAliasRecord record[DEVICE_ALIAS_MAX_ADDRESS];
    uint32_t crc;
} DeviceAliasLoopStorage;

typedef char DeviceAliasStorageMustFitOneSector[
    (sizeof(DeviceAliasLoopStorage) <= 4096U) ? 1 : -1];

static const uint32_t g_alias_main_address[DEVICE_ALIAS_LOOP_COUNT] = {
    0x115000UL, 0x117000UL, 0x119000UL
};
static const uint32_t g_alias_backup_address[DEVICE_ALIAS_LOOP_COUNT] = {
    0x116000UL, 0x118000UL, 0x11A000UL
};

static DeviceAliasLoopStorage g_alias_storage[DEVICE_ALIAS_LOOP_COUNT];
static DeviceAliasLoopStorage g_alias_verify;
static uint8_t g_alias_loaded;
static uint32_t g_alias_revision;

static uint32_t DeviceAliasCrc(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;
    for(index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for(bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
    }
    return ~crc;
}

static uint8_t DeviceAliasStorageValid(const DeviceAliasLoopStorage *storage,
                                       uint8_t loop_id)
{
    uint32_t stored_crc;
    DeviceAliasLoopStorage *mutable_storage;
    if(storage->magic != DEVICE_ALIAS_MAGIC ||
       storage->version != DEVICE_ALIAS_VERSION ||
       storage->loop_id != loop_id) return 0U;
    mutable_storage = (DeviceAliasLoopStorage *)storage;
    stored_crc = mutable_storage->crc;
    mutable_storage->crc = 0U;
    if(stored_crc != DeviceAliasCrc((const uint8_t *)storage,
                                    (uint32_t)sizeof(*storage)))
    {
        mutable_storage->crc = stored_crc;
        return 0U;
    }
    mutable_storage->crc = stored_crc;
    return 1U;
}

static void DeviceAliasStorageReset(DeviceAliasLoopStorage *storage,
                                    uint8_t loop_id)
{
    memset(storage, 0, sizeof(*storage));
    storage->magic = DEVICE_ALIAS_MAGIC;
    storage->version = DEVICE_ALIAS_VERSION;
    storage->loop_id = loop_id;
}

static void DeviceAliasLoad(void)
{
    uint8_t index;
    if(g_alias_loaded != 0U) return;
    for(index = 0U; index < DEVICE_ALIAS_LOOP_COUNT; index++)
    {
        uint8_t loop_id = (uint8_t)(index + 1U);
        uint8_t main_valid;
        uint8_t backup_valid;
        W25QXX_Read((uint8_t *)&g_alias_storage[index], g_alias_main_address[index],
                    (uint16_t)sizeof(g_alias_storage[index]));
        W25QXX_Read((uint8_t *)&g_alias_verify, g_alias_backup_address[index],
                    (uint16_t)sizeof(g_alias_verify));
        main_valid = DeviceAliasStorageValid(&g_alias_storage[index], loop_id);
        backup_valid = DeviceAliasStorageValid(&g_alias_verify, loop_id);
        if(main_valid != 0U &&
           (backup_valid == 0U ||
            g_alias_storage[index].sequence >= g_alias_verify.sequence))
        {
        }
        else if(backup_valid != 0U)
            memcpy(&g_alias_storage[index], &g_alias_verify, sizeof(g_alias_verify));
        else
            DeviceAliasStorageReset(&g_alias_storage[index], loop_id);
    }
    g_alias_loaded = 1U;
}

static uint8_t DeviceAliasSaveLoop(uint8_t loop_id)
{
    DeviceAliasLoopStorage *storage;
    uint8_t index;
    if(loop_id == 0U || loop_id > DEVICE_ALIAS_LOOP_COUNT) return 0U;
    index = (uint8_t)(loop_id - 1U);
    storage = &g_alias_storage[index];
    storage->magic = DEVICE_ALIAS_MAGIC;
    storage->version = DEVICE_ALIAS_VERSION;
    storage->loop_id = loop_id;
    storage->sequence++;
    storage->crc = 0U;
    storage->crc = DeviceAliasCrc((const uint8_t *)storage,
                                  (uint32_t)sizeof(*storage));

    W25QXX_Write((uint8_t *)storage, g_alias_backup_address[index],
                 (uint16_t)sizeof(*storage));
    W25QXX_Read((uint8_t *)&g_alias_verify, g_alias_backup_address[index],
                (uint16_t)sizeof(g_alias_verify));
    if(DeviceAliasStorageValid(&g_alias_verify, loop_id) == 0U ||
       g_alias_verify.sequence != storage->sequence) return 0U;

    W25QXX_Write((uint8_t *)storage, g_alias_main_address[index],
                 (uint16_t)sizeof(*storage));
    W25QXX_Read((uint8_t *)&g_alias_verify, g_alias_main_address[index],
                (uint16_t)sizeof(g_alias_verify));
    /* The verified backup is authoritative if refreshing the main copy fails. */
    if(DeviceAliasStorageValid(&g_alias_verify, loop_id) == 0U ||
       g_alias_verify.sequence != storage->sequence) return 1U;
    return 1U;
}

DeviceAliasResult DeviceAlias_ValidateName(const uint8_t *name)
{
    uint8_t index = 0U;
    uint8_t characters = 0U;
    if(name == NULL || name[0] == 0U) return DEVICE_ALIAS_EMPTY_NAME;
    while(name[index] != 0U)
    {
        uint8_t first = name[index];
        if(characters >= DEVICE_ALIAS_MAX_CHARS) return DEVICE_ALIAS_NAME_TOO_LONG;
        if(first < 0x80U)
        {
            if(!((first >= '0' && first <= '9') ||
                 (first >= 'A' && first <= 'Z') ||
                 (first >= 'a' && first <= 'z') || first == ' '))
                return DEVICE_ALIAS_INVALID_CHARACTER;
            index++;
        }
        else
        {
            uint8_t second = name[index + 1U];
            if(first < 0x81U || first > 0xFEU ||
               (first >= 0xA1U && first <= 0xA9U) || second < 0x40U ||
               second > 0xFEU || second == 0x7FU)
                return DEVICE_ALIAS_INVALID_CHARACTER;
            index += 2U;
        }
        characters++;
        if(index >= DEVICE_ALIAS_NAME_BYTES) return DEVICE_ALIAS_NAME_TOO_LONG;
    }
    return DEVICE_ALIAS_OK;
}

DeviceAliasLookupResult DeviceAlias_Get(uint8_t loop_id, uint8_t address,
                                        uint16_t product_code, uint8_t *name,
                                        uint8_t name_size)
{
    const DeviceAliasRecord *record;
    if(name != NULL && name_size != 0U) name[0] = 0U;
    if(loop_id == 0U || loop_id > DEVICE_ALIAS_LOOP_COUNT || address == 0U ||
       address > DEVICE_ALIAS_MAX_ADDRESS) return DEVICE_ALIAS_NONE;
    DeviceAliasLoad();
    record = &g_alias_storage[loop_id - 1U].record[address - 1U];
    if(record->name[0] == 0U) return DEVICE_ALIAS_NONE;
    if(product_code == 0U || record->product_code != product_code)
        return DEVICE_ALIAS_PRODUCT_MISMATCH;
    if(name != NULL && name_size != 0U)
    {
        strncpy((char *)name, (const char *)record->name, name_size - 1U);
        name[name_size - 1U] = 0U;
    }
    return DEVICE_ALIAS_MATCH;
}

DeviceAliasResult DeviceAlias_Set(uint8_t loop_id, uint8_t address,
                                  uint16_t product_code, const uint8_t *name)
{
    DeviceAliasRecord rollback_record;
    DeviceAliasRecord *record;
    uint32_t rollback_sequence;
    uint32_t rollback_crc;
    DeviceAliasResult result = DeviceAlias_ValidateName(name);
    if(loop_id == 0U || loop_id > DEVICE_ALIAS_LOOP_COUNT || address == 0U ||
       address > DEVICE_ALIAS_MAX_ADDRESS || product_code == 0U)
        return DEVICE_ALIAS_INVALID_DEVICE;
    if(result != DEVICE_ALIAS_OK) return result;
    DeviceAliasLoad();
    record = &g_alias_storage[loop_id - 1U].record[address - 1U];
    if(record->product_code == product_code &&
       strcmp((const char *)record->name, (const char *)name) == 0)
        return DEVICE_ALIAS_OK;
    memcpy(&rollback_record, record, sizeof(rollback_record));
    rollback_sequence = g_alias_storage[loop_id - 1U].sequence;
    rollback_crc = g_alias_storage[loop_id - 1U].crc;
    memset(record, 0, sizeof(*record));
    record->product_code = product_code;
    strncpy((char *)record->name, (const char *)name, sizeof(record->name) - 1U);
    if(DeviceAliasSaveLoop(loop_id) == 0U)
    {
        memcpy(record, &rollback_record, sizeof(rollback_record));
        g_alias_storage[loop_id - 1U].sequence = rollback_sequence;
        g_alias_storage[loop_id - 1U].crc = rollback_crc;
        return DEVICE_ALIAS_STORAGE_ERROR;
    }
    g_alias_revision++;
    return DEVICE_ALIAS_OK;
}

DeviceAliasResult DeviceAlias_Clear(uint8_t loop_id, uint8_t address)
{
    DeviceAliasRecord rollback_record;
    DeviceAliasRecord *record;
    uint32_t rollback_sequence;
    uint32_t rollback_crc;
    if(loop_id == 0U || loop_id > DEVICE_ALIAS_LOOP_COUNT || address == 0U ||
       address > DEVICE_ALIAS_MAX_ADDRESS) return DEVICE_ALIAS_INVALID_DEVICE;
    DeviceAliasLoad();
    record = &g_alias_storage[loop_id - 1U].record[address - 1U];
    if(record->name[0] == 0U) return DEVICE_ALIAS_OK;
    memcpy(&rollback_record, record, sizeof(rollback_record));
    rollback_sequence = g_alias_storage[loop_id - 1U].sequence;
    rollback_crc = g_alias_storage[loop_id - 1U].crc;
    memset(record, 0, sizeof(*record));
    if(DeviceAliasSaveLoop(loop_id) == 0U)
    {
        memcpy(record, &rollback_record, sizeof(rollback_record));
        g_alias_storage[loop_id - 1U].sequence = rollback_sequence;
        g_alias_storage[loop_id - 1U].crc = rollback_crc;
        return DEVICE_ALIAS_STORAGE_ERROR;
    }
    g_alias_revision++;
    return DEVICE_ALIAS_OK;
}

DeviceAliasResult DeviceAlias_ClearAll(void)
{
    uint8_t loop_id;
    DeviceAliasLoad();
    for(loop_id = 1U; loop_id <= DEVICE_ALIAS_LOOP_COUNT; loop_id++)
    {
        DeviceAliasStorageReset(&g_alias_storage[loop_id - 1U], loop_id);
        if(DeviceAliasSaveLoop(loop_id) == 0U)
        {
            W25QXX_Read((uint8_t *)&g_alias_storage[loop_id - 1U],
                        g_alias_main_address[loop_id - 1U],
                        (uint16_t)sizeof(g_alias_storage[0]));
            if(DeviceAliasStorageValid(&g_alias_storage[loop_id - 1U], loop_id) == 0U)
                DeviceAliasStorageReset(&g_alias_storage[loop_id - 1U], loop_id);
            return DEVICE_ALIAS_STORAGE_ERROR;
        }
    }
    g_alias_revision++;
    return DEVICE_ALIAS_OK;
}

uint32_t DeviceAlias_GetRevision(void)
{
    return g_alias_revision;
}
