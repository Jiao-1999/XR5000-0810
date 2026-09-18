#include "bsp_ig3302.h"

#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_itcallback.h"
#include "system.h"
#include "usart.h"
#include "w25qxx.h"

/* Loop 4 is dedicated to IG3302-DC on UART9, Modbus RTU 9600/8N1. */
#define IG3302_FLASH_MAIN_ADDR       0x11D000UL
#define IG3302_FLASH_BACKUP_ADDR     0x11E000UL
#define IG3302_FLASH_MAGIC           0x30334749UL
#define IG3302_FLASH_VERSION         1U
#define IG3302_POLL_INTERVAL_MS      50U
#define IG3302_RESPONSE_TIMEOUT_MS   200U
#define IG3302_INTER_FRAME_GUARD_MS  20U
#define IG3302_TASK_PERIOD_MS        5U
#define IG3302_CONTROL_QUEUE_SIZE    8U
#define IG3302_MAX_CONTROL_BURST     2U
#define IG3302_RX_STREAM_SIZE        64U

typedef struct
{
    uint8_t configured;
    uint8_t data_valid;
    uint8_t disconnected;
    uint8_t miss_count;
    uint8_t recovery_count;
    uint8_t fan_state[2];
} IG3302Device;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t sequence;
    uint8_t configured[IG3302_MAX_ADDRESS + 1U];
    uint8_t padding;
    uint32_t crc;
} IG3302Storage;

typedef enum
{
    IG3302_TRANSACTION_NONE = 0,
    IG3302_TRANSACTION_POLL,
    IG3302_TRANSACTION_CONTROL
} IG3302TransactionType;

static IG3302Device g_devices[IG3302_MAX_ADDRESS + 1U];
static IG3302CtrlRequest g_control_queue[IG3302_CONTROL_QUEUE_SIZE];
static volatile uint8_t g_control_head;
static volatile uint8_t g_control_tail;
static uint8_t g_rx_stream[IG3302_RX_STREAM_SIZE];
static uint8_t g_rx_chunk[32];
static uint8_t g_tx_frame[8];
static uint16_t g_rx_stream_length;
static uint8_t g_next_poll_address = 1U;
static uint8_t g_control_burst;
static uint8_t g_transaction_address;
static uint8_t g_initialized;
static IG3302TransactionType g_transaction_type;
static uint32_t g_transaction_start_tick;
static uint32_t g_last_transaction_end_tick;
static uint32_t g_revision;
static uint32_t g_storage_sequence;

static uint32_t IG3302_StorageCrc(const uint8_t *data, uint32_t length)
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

static uint8_t IG3302_StorageValid(IG3302Storage *storage)
{
    uint32_t stored_crc;
    if(storage->magic != IG3302_FLASH_MAGIC ||
       storage->version != IG3302_FLASH_VERSION)
        return 0U;
    stored_crc = storage->crc;
    storage->crc = 0U;
    if(stored_crc != IG3302_StorageCrc((const uint8_t *)storage,
                                      (uint32_t)sizeof(*storage)))
    {
        storage->crc = stored_crc;
        return 0U;
    }
    storage->crc = stored_crc;
    return 1U;
}

static void IG3302_ResetRuntime(uint8_t address)
{
    g_devices[address].data_valid = 0U;
    g_devices[address].disconnected = 0U;
    g_devices[address].miss_count = 0U;
    g_devices[address].recovery_count = 0U;
    g_devices[address].fan_state[0] = IG3302_FAN_STATE_INVALID;
    g_devices[address].fan_state[1] = IG3302_FAN_STATE_INVALID;
}

void IG3302_LoadOnlineState(void)
{
    IG3302Storage main_storage;
    IG3302Storage backup_storage;
    IG3302Storage *selected = NULL;
    uint8_t address;

    W25QXX_Read((uint8_t *)&main_storage, IG3302_FLASH_MAIN_ADDR,
                (uint16_t)sizeof(main_storage));
    W25QXX_Read((uint8_t *)&backup_storage, IG3302_FLASH_BACKUP_ADDR,
                (uint16_t)sizeof(backup_storage));
    if(IG3302_StorageValid(&main_storage) != 0U) selected = &main_storage;
    if(IG3302_StorageValid(&backup_storage) != 0U &&
       (selected == NULL || backup_storage.sequence > selected->sequence))
        selected = &backup_storage;

    g_storage_sequence = (selected != NULL) ? selected->sequence : 0U;
    for(address = 1U; address <= IG3302_MAX_ADDRESS; address++)
    {
        g_devices[address].configured =
            (selected != NULL && selected->configured[address] == 1U) ? 1U : 0U;
        IG3302_ResetRuntime(address);
    }
}

void IG3302_SaveOnlineState(void)
{
    IG3302Storage storage;
    IG3302Storage verify;
    uint8_t address;

    memset(&storage, 0, sizeof(storage));
    storage.magic = IG3302_FLASH_MAGIC;
    storage.version = IG3302_FLASH_VERSION;
    storage.sequence = ++g_storage_sequence;
    for(address = 1U; address <= IG3302_MAX_ADDRESS; address++)
        storage.configured[address] = g_devices[address].configured;
    storage.crc = 0U;
    storage.crc = IG3302_StorageCrc((const uint8_t *)&storage,
                                   (uint32_t)sizeof(storage));

    W25QXX_Write((uint8_t *)&storage, IG3302_FLASH_BACKUP_ADDR,
                 (uint16_t)sizeof(storage));
    W25QXX_Read((uint8_t *)&verify, IG3302_FLASH_BACKUP_ADDR,
                (uint16_t)sizeof(verify));
    if(IG3302_StorageValid(&verify) == 0U || verify.sequence != storage.sequence)
        return;
    W25QXX_Write((uint8_t *)&storage, IG3302_FLASH_MAIN_ADDR,
                 (uint16_t)sizeof(storage));
}

void IG3302_Init(void)
{
    if(g_initialized != 0U) return;
    memset(g_devices, 0, sizeof(g_devices));
    g_control_head = 0U;
    g_control_tail = 0U;
    g_rx_stream_length = 0U;
    g_transaction_type = IG3302_TRANSACTION_NONE;
    g_last_transaction_end_tick = osKernelGetTickCount();
    IG3302_LoadOnlineState();
    IG3302UartInitRx();
    g_initialized = 1U;
}

uint8_t IG3302_SetOnline(uint8_t address, uint8_t online)
{
    uint8_t new_state;
    if(address == 0U || address > IG3302_MAX_ADDRESS) return 0U;
    new_state = (online != 0U) ? 1U : 0U;
    if(g_devices[address].configured == new_state) return 0U;
    g_devices[address].configured = new_state;
    IG3302_ResetRuntime(address);
    if(new_state == 0U && g_transaction_address == address)
        g_transaction_type = IG3302_TRANSACTION_NONE;
    g_revision++;
    return 1U;
}

uint8_t IG3302_IsConfigured(uint8_t address)
{
    if(address == 0U || address > IG3302_MAX_ADDRESS) return 0U;
    return g_devices[address].configured;
}

uint8_t IG3302_IsActive(uint8_t address)
{
    if(address == 0U || address > IG3302_MAX_ADDRESS) return 0U;
    return (uint8_t)(g_devices[address].configured != 0U &&
                     g_devices[address].data_valid != 0U &&
                     g_devices[address].disconnected == 0U);
}

uint8_t IG3302_IsDisconnected(uint8_t address)
{
    if(address == 0U || address > IG3302_MAX_ADDRESS) return 0U;
    return (uint8_t)(g_devices[address].configured != 0U &&
                     g_devices[address].disconnected != 0U);
}

uint8_t IG3302_GetFanState(uint8_t address, uint8_t fan)
{
    if(address == 0U || address > IG3302_MAX_ADDRESS || fan == 0U || fan > 2U)
        return IG3302_FAN_STATE_INVALID;
    return g_devices[address].fan_state[fan - 1U];
}

uint8_t IG3302_GetConfiguredCount(void)
{
    uint8_t address;
    uint8_t count = 0U;
    for(address = 1U; address <= IG3302_MAX_ADDRESS; address++)
        if(g_devices[address].configured != 0U) count++;
    return count;
}

uint8_t IG3302_GetActiveCount(void)
{
    uint8_t address;
    uint8_t count = 0U;
    for(address = 1U; address <= IG3302_MAX_ADDRESS; address++)
        if(IG3302_IsActive(address) != 0U) count++;
    return count;
}

uint8_t IG3302_GetFaultDeviceCount(void)
{
    uint8_t address;
    uint8_t count = 0U;
    for(address = 1U; address <= IG3302_MAX_ADDRESS; address++)
    {
        if(g_devices[address].configured == 0U) continue;
        if(g_devices[address].disconnected != 0U ||
           (g_devices[address].data_valid != 0U &&
            (g_devices[address].fan_state[0] >= IG3302_FAN_PUSHROD_FAULT ||
             g_devices[address].fan_state[1] >= IG3302_FAN_PUSHROD_FAULT)))
            count++;
    }
    return count;
}

uint32_t IG3302_GetRevision(void)
{
    return g_revision;
}

IG3302CtrlResult IG3302_Request(const IG3302CtrlRequest *request)
{
    uint8_t next;
    if(request == NULL || request->address == 0U ||
       request->address > IG3302_MAX_ADDRESS ||
       request->fan == 0U || request->fan > 2U)
        return IG3302_CTRL_INVALID_PARAM;
    if(g_devices[request->address].configured == 0U)
        return IG3302_CTRL_NOT_CONFIGURED;

    taskENTER_CRITICAL();
    next = (uint8_t)((g_control_head + 1U) % IG3302_CONTROL_QUEUE_SIZE);
    if(next == g_control_tail)
    {
        taskEXIT_CRITICAL();
        return IG3302_CTRL_QUEUE_FULL;
    }
    g_control_queue[g_control_head] = *request;
    g_control_queue[g_control_head].start = (request->start != 0U) ? 1U : 0U;
    g_control_head = next;
    taskEXIT_CRITICAL();
    return IG3302_CTRL_OK;
}

static uint8_t IG3302_PopControl(IG3302CtrlRequest *request)
{
    if(g_control_tail == g_control_head) return 0U;
    taskENTER_CRITICAL();
    if(g_control_tail == g_control_head)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    *request = g_control_queue[g_control_tail];
    g_control_tail = (uint8_t)((g_control_tail + 1U) % IG3302_CONTROL_QUEUE_SIZE);
    taskEXIT_CRITICAL();
    return 1U;
}

static void IG3302_RecordPollFailure(uint8_t address)
{
    IG3302Device *device;
    if(address == 0U || address > IG3302_MAX_ADDRESS ||
       g_devices[address].configured == 0U) return;
    device = &g_devices[address];
    device->recovery_count = 0U;
    if(device->miss_count < 0xFFU) device->miss_count++;
    if(device->miss_count >= IG3302_DISCONNECT_THRESHOLD &&
       device->disconnected == 0U)
    {
        device->disconnected = 1U;
        g_revision++;
    }
}

static void IG3302_RecordPollSuccess(uint8_t address, uint8_t fan1, uint8_t fan2)
{
    IG3302Device *device = &g_devices[address];
    uint8_t changed = 0U;
    if(device->fan_state[0] != fan1 || device->fan_state[1] != fan2 ||
       device->data_valid == 0U) changed = 1U;
    device->fan_state[0] = fan1;
    device->fan_state[1] = fan2;
    device->data_valid = 1U;
    device->miss_count = 0U;

    if(device->disconnected != 0U)
    {
        if(device->recovery_count < 0xFFU) device->recovery_count++;
        if(device->recovery_count >= IG3302_RECOVERY_SUCCESS_THRESHOLD)
        {
            device->disconnected = 0U;
            device->recovery_count = 0U;
            changed = 1U;
        }
    }
    else
    {
        device->recovery_count = 0U;
    }
    if(changed != 0U) g_revision++;
}

static void IG3302_EndTransaction(void)
{
    g_transaction_type = IG3302_TRANSACTION_NONE;
    g_transaction_address = 0U;
    g_transaction_start_tick = 0U;
    g_last_transaction_end_tick = osKernelGetTickCount();
}

static uint8_t IG3302_StartTransaction(const uint8_t frame[8], uint8_t address,
                                       IG3302TransactionType type)
{
    HAL_StatusTypeDef status;
    memcpy(g_tx_frame, frame, sizeof(g_tx_frame));
    IG3302UartClearRx();
    g_rx_stream_length = 0U;
    g_transaction_address = address;
    g_transaction_type = type;
    g_transaction_start_tick = osKernelGetTickCount();
    status = HAL_UART_Transmit_IT(&huart9, g_tx_frame, sizeof(g_tx_frame));
    if(status != HAL_OK)
    {
        if(type == IG3302_TRANSACTION_POLL) IG3302_RecordPollFailure(address);
        IG3302_EndTransaction();
        return 0U;
    }
    return 1U;
}

static uint8_t IG3302_StartPoll(uint8_t address)
{
    uint16_t crc;
    uint8_t frame[8] = {address, 0x04U, 0x00U, 0x00U, 0x00U, 0x02U, 0U, 0U};
    crc = CalcCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)(crc >> 8U);
    return IG3302_StartTransaction(frame, address, IG3302_TRANSACTION_POLL);
}

static uint8_t IG3302_StartControl(const IG3302CtrlRequest *request)
{
    uint16_t crc;
    uint16_t coil = (uint16_t)(request->fan - 1U);
    uint8_t frame[8] = {request->address, 0x05U,
                        (uint8_t)(coil >> 8U), (uint8_t)coil,
                        request->start != 0U ? 0xFFU : 0x00U, 0x00U, 0U, 0U};
    crc = CalcCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)(crc >> 8U);
    return IG3302_StartTransaction(frame, request->address,
                                   IG3302_TRANSACTION_CONTROL);
}

static uint8_t IG3302_NextConfiguredAddress(void)
{
    uint8_t checked;
    uint8_t address;
    for(checked = 0U; checked < IG3302_MAX_ADDRESS; checked++)
    {
        address = g_next_poll_address;
        g_next_poll_address++;
        if(g_next_poll_address > IG3302_MAX_ADDRESS) g_next_poll_address = 1U;
        if(g_devices[address].configured != 0U) return address;
    }
    return 0U;
}

static void IG3302_RemoveStreamBytes(uint16_t count)
{
    if(count >= g_rx_stream_length)
    {
        g_rx_stream_length = 0U;
        return;
    }
    memmove(g_rx_stream, &g_rx_stream[count], g_rx_stream_length - count);
    g_rx_stream_length = (uint16_t)(g_rx_stream_length - count);
}

static void IG3302_AppendRx(void)
{
    uint16_t count;
    uint16_t free_space;
    do
    {
        count = IG3302UartRead(g_rx_chunk, (uint16_t)sizeof(g_rx_chunk));
        if(count == 0U) break;
        free_space = (uint16_t)(IG3302_RX_STREAM_SIZE - g_rx_stream_length);
        if(count > free_space)
        {
            uint16_t discard = (uint16_t)(count - free_space);
            if(discard >= g_rx_stream_length) g_rx_stream_length = 0U;
            else IG3302_RemoveStreamBytes(discard);
        }
        memcpy(&g_rx_stream[g_rx_stream_length], g_rx_chunk, count);
        g_rx_stream_length = (uint16_t)(g_rx_stream_length + count);
    } while(count == sizeof(g_rx_chunk));
}

static uint8_t IG3302_ProcessRxFrame(void)
{
    uint16_t expected_length;
    uint16_t crc;
    uint16_t received_crc;

    while(g_rx_stream_length >= 5U)
    {
        if(g_rx_stream[0] == 0U || g_rx_stream[0] > IG3302_MAX_ADDRESS)
        {
            IG3302_RemoveStreamBytes(1U);
            continue;
        }
        if((g_rx_stream[1] & 0x80U) != 0U) expected_length = 5U;
        else if(g_rx_stream[1] == 0x04U)
        {
            if(g_rx_stream[2] > 32U)
            {
                IG3302_RemoveStreamBytes(1U);
                continue;
            }
            expected_length = (uint16_t)(5U + g_rx_stream[2]);
        }
        else if(g_rx_stream[1] == 0x05U) expected_length = 8U;
        else
        {
            IG3302_RemoveStreamBytes(1U);
            continue;
        }
        if(g_rx_stream_length < expected_length) return 0U;
        received_crc = (uint16_t)g_rx_stream[expected_length - 2U] |
                       ((uint16_t)g_rx_stream[expected_length - 1U] << 8U);
        crc = CalcCrc16(g_rx_stream, (uint16_t)(expected_length - 2U));
        if(crc != received_crc)
        {
            IG3302_RemoveStreamBytes(1U);
            continue;
        }
        if(g_transaction_type == IG3302_TRANSACTION_NONE ||
           g_rx_stream[0] != g_transaction_address)
        {
            IG3302_RemoveStreamBytes(expected_length);
            continue;
        }

        if(g_transaction_type == IG3302_TRANSACTION_POLL &&
           g_rx_stream[1] == 0x04U && g_rx_stream[2] == 4U)
        {
            uint16_t fan1 = ((uint16_t)g_rx_stream[3] << 8U) | g_rx_stream[4];
            uint16_t fan2 = ((uint16_t)g_rx_stream[5] << 8U) | g_rx_stream[6];
            if(fan1 <= IG3302_FAN_AND_PUSHROD_FAULT &&
               fan2 <= IG3302_FAN_AND_PUSHROD_FAULT)
            {
                uint8_t address = g_transaction_address;
                IG3302_RemoveStreamBytes(expected_length);
                IG3302_RecordPollSuccess(address, (uint8_t)fan1, (uint8_t)fan2);
                IG3302_EndTransaction();
                return 1U;
            }
        }
        else if(g_transaction_type == IG3302_TRANSACTION_CONTROL &&
                g_rx_stream[1] == 0x05U && expected_length == 8U &&
                memcmp(g_rx_stream, g_tx_frame, 6U) == 0)
        {
            uint8_t address = g_transaction_address;
            IG3302_RemoveStreamBytes(expected_length);
            g_next_poll_address = address;
            IG3302_EndTransaction();
            return 1U;
        }

        IG3302_RemoveStreamBytes(expected_length);
        if(g_transaction_type == IG3302_TRANSACTION_POLL)
            IG3302_RecordPollFailure(g_transaction_address);
        IG3302_EndTransaction();
        return 1U;
    }
    return 0U;
}

void IG3302_PollAndReceiveTask(void *argument)
{
    IG3302CtrlRequest request;
    uint8_t address;
    uint32_t now;
    (void)argument;
    IG3302_Init();

    for(;;)
    {
        IG3302_AppendRx();
        (void)IG3302_ProcessRxFrame();
        now = osKernelGetTickCount();

        if(g_transaction_type != IG3302_TRANSACTION_NONE)
        {
            if((now - g_transaction_start_tick) >= IG3302_RESPONSE_TIMEOUT_MS)
            {
                if(g_transaction_type == IG3302_TRANSACTION_POLL)
                    IG3302_RecordPollFailure(g_transaction_address);
                IG3302_EndTransaction();
            }
        }
        else if((now - g_last_transaction_end_tick) >= IG3302_INTER_FRAME_GUARD_MS)
        {
            if(g_control_burst < IG3302_MAX_CONTROL_BURST &&
               IG3302_PopControl(&request) != 0U)
            {
                if(IG3302_StartControl(&request) != 0U) g_control_burst++;
            }
            else if((now - g_last_transaction_end_tick) >= IG3302_POLL_INTERVAL_MS)
            {
                address = IG3302_NextConfiguredAddress();
                if(address != 0U)
                {
                    if(IG3302_StartPoll(address) != 0U) g_control_burst = 0U;
                }
                else
                {
                    g_control_burst = 0U;
                    g_last_transaction_end_tick = now;
                }
            }
        }
        osDelay(IG3302_TASK_PERIOD_MS);
    }
}
