#include "bsp_device_registration.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_device_registry.h"
#include "w25qxx.h"

/* 登记使用独立扇区，不复用上线、屏蔽、中文名称及事件历史区。 */
#define DEVICE_REG_MAIN_ADDR    0x11B000UL
#define DEVICE_REG_BACKUP_ADDR  0x11C000UL
#define DEVICE_REG_MAGIC        0x47524544UL
#define DEVICE_REG_VERSION      1U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t sequence;
    /* 0表示未登记；下标即实际设备地址，回路1额外覆盖101～110。 */
    uint16_t product_code[4U][DEVICE_REG_MAX_ADDRESS + 1U];
    uint32_t crc;
} DeviceRegStorage;

typedef char DeviceRegStorageFitsSector[
    (sizeof(DeviceRegStorage) <= 4096U) ? 1 : -1];

static DeviceRegStorage g_saved;
static DeviceRegStorage g_candidate;
static DeviceRegStorage g_verify;
static uint8_t g_observation[4U][DEVICE_REG_MAX_ADDRESS + 1U];
static uint8_t g_next_address[4U];
static uint8_t g_inflight_address[4U];
static uint16_t g_scanned_count[4U];
static uint16_t g_pending_new_count[4U];
static uint16_t g_saved_new_count[4U];
static uint8_t g_active_mask;
static uint8_t g_loaded;
static DeviceRegScanState g_state = DEVICE_REG_SCAN_IDLE;
static uint32_t g_revision;

static uint8_t DeviceRegMaxAddress(uint8_t loop)
{
    if(loop == DEVICE_REG_LOOP1) return 110U;
    if(loop == DEVICE_REG_LOOP2) return 63U;
    if(loop == DEVICE_REG_LOOP3) return 64U;
    return 0U;
}

static uint32_t DeviceRegCrc(const uint8_t *bytes, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;
    for(index = 0U; index < length; index++)
    {
        crc ^= bytes[index];
        for(bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
    }
    return ~crc;
}

static uint8_t DeviceRegStorageValid(DeviceRegStorage *storage)
{
    uint32_t stored_crc;
    if(storage->magic != DEVICE_REG_MAGIC ||
       storage->version != DEVICE_REG_VERSION) return 0U;
    stored_crc = storage->crc;
    storage->crc = 0U;
    if(stored_crc != DeviceRegCrc((const uint8_t *)storage,
                                 (uint32_t)sizeof(*storage)))
    {
        storage->crc = stored_crc;
        return 0U;
    }
    storage->crc = stored_crc;
    return 1U;
}

void DeviceReg_Init(void)
{
    uint8_t main_valid;
    uint8_t backup_valid;
    if(g_loaded != 0U) return;
    W25QXX_Read((uint8_t *)&g_saved, DEVICE_REG_MAIN_ADDR,
                 (uint16_t)sizeof(g_saved));
    W25QXX_Read((uint8_t *)&g_verify, DEVICE_REG_BACKUP_ADDR,
                 (uint16_t)sizeof(g_verify));
    main_valid = DeviceRegStorageValid(&g_saved);
    backup_valid = DeviceRegStorageValid(&g_verify);
    if(backup_valid != 0U &&
       (main_valid == 0U || g_verify.sequence > g_saved.sequence))
        memcpy(&g_saved, &g_verify, sizeof(g_saved));
    else if(main_valid == 0U)
    {
        memset(&g_saved, 0, sizeof(g_saved));
        g_saved.magic = DEVICE_REG_MAGIC;
        g_saved.version = DEVICE_REG_VERSION;
    }
    memcpy(&g_candidate, &g_saved, sizeof(g_candidate));
    g_loaded = 1U;
}

uint8_t DeviceReg_StartScan(uint8_t loop_mask)
{
    uint8_t loop;
    DeviceReg_Init();
    loop_mask &= DEVICE_REG_ALL_MASK;
    if(loop_mask == 0U) return 0U;
    taskENTER_CRITICAL();
    if(g_state == DEVICE_REG_SCAN_RUNNING ||
       g_state == DEVICE_REG_SCAN_SAVING)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    memcpy(&g_candidate, &g_saved, sizeof(g_candidate));
    g_active_mask = loop_mask;
    memset(g_pending_new_count, 0, sizeof(g_pending_new_count));
    memset(g_saved_new_count, 0, sizeof(g_saved_new_count));
    for(loop = DEVICE_REG_LOOP1; loop <= DEVICE_REG_LOOP3; loop++)
    {
        if((loop_mask & (uint8_t)(1U << (loop - 1U))) == 0U) continue;
        memset(g_observation[loop], 0, sizeof(g_observation[loop]));
        g_next_address[loop] = 1U;
        g_inflight_address[loop] = 0U;
        g_scanned_count[loop] = 0U;
    }
    g_state = DEVICE_REG_SCAN_RUNNING;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t DeviceReg_Clear(uint8_t loop_mask)
{
    uint8_t loop;
    uint8_t changed = 0U;
    DeviceReg_Init();
    loop_mask &= DEVICE_REG_ALL_MASK;
    if(loop_mask == 0U) return 0U;
    taskENTER_CRITICAL();
    if(g_state == DEVICE_REG_SCAN_SAVING)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    g_active_mask = 0U;
    memset(g_inflight_address, 0, sizeof(g_inflight_address));
    memset(g_pending_new_count, 0, sizeof(g_pending_new_count));
    memset(g_saved_new_count, 0, sizeof(g_saved_new_count));
    memcpy(&g_candidate, &g_saved, sizeof(g_candidate));
    for(loop = DEVICE_REG_LOOP1; loop <= DEVICE_REG_LOOP3; loop++)
    {
        uint8_t address;
        if((loop_mask & (uint8_t)(1U << (loop - 1U))) == 0U) continue;
        for(address = 1U; address <= DeviceRegMaxAddress(loop); address++)
        {
            if(g_candidate.product_code[loop][address] != 0U) changed = 1U;
            g_candidate.product_code[loop][address] = 0U;
        }
        memset(g_observation[loop], 0, sizeof(g_observation[loop]));
        g_scanned_count[loop] = 0U;
    }
    g_state = changed != 0U ? DEVICE_REG_SCAN_SAVING : DEVICE_REG_SCAN_DONE;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t DeviceReg_TakeProbe(uint8_t loop, uint8_t *address)
{
    uint8_t max_address = DeviceRegMaxAddress(loop);
    uint8_t mask;
    if(address == 0 || max_address == 0U) return 0U;
    mask = (uint8_t)(1U << (loop - 1U));
    taskENTER_CRITICAL();
    if(g_state != DEVICE_REG_SCAN_RUNNING ||
       (g_active_mask & mask) == 0U ||
       g_inflight_address[loop] != 0U ||
       g_next_address[loop] == 0U ||
       g_next_address[loop] > max_address)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    *address = g_next_address[loop];
    g_inflight_address[loop] = *address;
    taskEXIT_CRITICAL();
    return 1U;
}

void DeviceReg_CancelProbe(uint8_t loop, uint8_t address)
{
    if(DeviceRegMaxAddress(loop) == 0U) return;
    taskENTER_CRITICAL();
    if(g_inflight_address[loop] == address)
        g_inflight_address[loop] = 0U;
    taskEXIT_CRITICAL();
}

void DeviceReg_CompleteProbe(uint8_t loop, uint8_t address,
                            DeviceRegProbeResult result, uint16_t product_code)
{
    uint8_t mask;
    uint8_t max_address = DeviceRegMaxAddress(loop);
    if(address == 0U || address > max_address || max_address == 0U) return;
    mask = (uint8_t)(1U << (loop - 1U));
    taskENTER_CRITICAL();
    if(g_state != DEVICE_REG_SCAN_RUNNING ||
       (g_active_mask & mask) == 0U ||
       g_inflight_address[loop] != address)
    {
        taskEXIT_CRITICAL();
        return;
    }
    if(result == DEVICE_REG_PROBE_IDENTIFIED &&
       DeviceRegistry_IsSupportedOnLoop(product_code, loop) != 0U)
    {
        g_observation[loop][address] = DEVICE_REG_OBSERVATION_IDENTIFIED;
        if(g_saved.product_code[loop][address] == 0U)
            g_pending_new_count[loop]++;
        g_candidate.product_code[loop][address] = product_code;
    }
    else if(result == DEVICE_REG_PROBE_UNIDENTIFIED ||
            result == DEVICE_REG_PROBE_IDENTIFIED)
        g_observation[loop][address] = DEVICE_REG_OBSERVATION_UNIDENTIFIED;
    else
        g_observation[loop][address] = DEVICE_REG_OBSERVATION_NO_RESPONSE;
    g_scanned_count[loop]++;
    g_inflight_address[loop] = 0U;
    g_next_address[loop] = (uint8_t)(address + 1U);
    if(g_next_address[loop] > max_address)
    {
        g_active_mask &= (uint8_t)~mask;
        if(g_active_mask == 0U)
        {
            g_state = memcmp(g_candidate.product_code, g_saved.product_code,
                             sizeof(g_saved.product_code)) != 0 ?
                      DEVICE_REG_SCAN_SAVING : DEVICE_REG_SCAN_DONE;
        }
    }
    taskEXIT_CRITICAL();
}

void DeviceReg_ServiceSave(void)
{
    uint8_t backup_valid;
    uint8_t loop;
    if(g_loaded == 0U || g_state != DEVICE_REG_SCAN_SAVING) return;
    /* 保存由低频界面刷新任务负责，不在UART事务或接收回调内擦写Flash。 */
    taskENTER_CRITICAL();
    memcpy(&g_verify, &g_candidate, sizeof(g_verify));
    g_verify.sequence = g_saved.sequence + 1U;
    g_verify.crc = 0U;
    taskEXIT_CRITICAL();
    g_verify.crc = DeviceRegCrc((const uint8_t *)&g_verify,
                                (uint32_t)sizeof(g_verify));
    W25QXX_Write((uint8_t *)&g_verify, DEVICE_REG_BACKUP_ADDR,
                  (uint16_t)sizeof(g_verify));
    W25QXX_Read((uint8_t *)&g_candidate, DEVICE_REG_BACKUP_ADDR,
                 (uint16_t)sizeof(g_candidate));
    backup_valid = DeviceRegStorageValid(&g_candidate) != 0U &&
                   g_candidate.sequence == g_verify.sequence;
    if(backup_valid != 0U)
    {
        W25QXX_Write((uint8_t *)&g_verify, DEVICE_REG_MAIN_ADDR,
                      (uint16_t)sizeof(g_verify));
        W25QXX_Read((uint8_t *)&g_candidate, DEVICE_REG_MAIN_ADDR,
                     (uint16_t)sizeof(g_candidate));
        /* 已校验的备份副本即使主副本刷新失败也足以恢复。 */
        taskENTER_CRITICAL();
        memcpy(&g_saved, &g_verify, sizeof(g_saved));
        for(loop = DEVICE_REG_LOOP1; loop <= DEVICE_REG_LOOP3; loop++)
            g_saved_new_count[loop] = g_pending_new_count[loop];
        g_revision++;
        g_state = DEVICE_REG_SCAN_DONE;
        taskEXIT_CRITICAL();
    }
    else
    {
        taskENTER_CRITICAL();
        memcpy(&g_candidate, &g_saved, sizeof(g_candidate));
        g_state = DEVICE_REG_SCAN_SAVE_ERROR;
        taskEXIT_CRITICAL();
    }
}

DeviceRegScanState DeviceReg_GetState(void)
{
    return g_state;
}

void DeviceReg_GetStats(uint8_t loop, DeviceRegStats *stats)
{
    uint8_t address;
    uint8_t max_address = DeviceRegMaxAddress(loop);
    if(stats == 0) return;
    memset(stats, 0, sizeof(*stats));
    if(max_address == 0U) return;
    taskENTER_CRITICAL();
    stats->total_addresses = max_address;
    stats->scanned_addresses = g_scanned_count[loop];
    stats->newly_registered = g_saved_new_count[loop];
    for(address = 1U; address <= max_address; address++)
    {
        if(g_saved.product_code[loop][address] != 0U) stats->registered++;
        if(g_observation[loop][address] == DEVICE_REG_OBSERVATION_IDENTIFIED)
            stats->scan_identified++;
        else if(g_observation[loop][address] == DEVICE_REG_OBSERVATION_UNIDENTIFIED)
            stats->unidentified++;
        else if(g_observation[loop][address] == DEVICE_REG_OBSERVATION_NO_RESPONSE &&
                g_saved.product_code[loop][address] != 0U)
            stats->registered_no_response++;
    }
    taskEXIT_CRITICAL();
}

uint8_t DeviceReg_GetEntry(uint8_t loop, uint8_t address, DeviceRegEntry *entry)
{
    if(entry == 0 || address == 0U || address > DeviceRegMaxAddress(loop)) return 0U;
    taskENTER_CRITICAL();
    if(g_saved.product_code[loop][address] == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    entry->loop = loop;
    entry->address = address;
    entry->product_code = g_saved.product_code[loop][address];
    entry->observation = (DeviceRegObservation)g_observation[loop][address];
    taskEXIT_CRITICAL();
    return 1U;
}

uint16_t DeviceReg_List(uint8_t loop_mask, DeviceRegEntry *entries, uint16_t capacity)
{
    uint16_t count = 0U;
    uint8_t loop;
    if(entries == 0 || capacity == 0U) return 0U;
    loop_mask &= DEVICE_REG_ALL_MASK;
    taskENTER_CRITICAL();
    for(loop = DEVICE_REG_LOOP1; loop <= DEVICE_REG_LOOP3; loop++)
    {
        uint8_t address;
        if((loop_mask & (uint8_t)(1U << (loop - 1U))) == 0U) continue;
        for(address = 1U; address <= DeviceRegMaxAddress(loop); address++)
        {
            uint16_t code = g_saved.product_code[loop][address];
            if(code == 0U) continue;
            if(count >= capacity) break;
            entries[count].loop = loop;
            entries[count].address = address;
            entries[count].product_code = code;
            entries[count].observation =
                (DeviceRegObservation)g_observation[loop][address];
            count++;
        }
    }
    taskEXIT_CRITICAL();
    return count;
}

uint32_t DeviceReg_GetRevision(void)
{
    return g_revision;
}
