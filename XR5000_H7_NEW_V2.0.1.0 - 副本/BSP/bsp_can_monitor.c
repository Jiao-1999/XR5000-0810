#include "bsp_can_monitor.h"

#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_fdcan1.h"
#include "hmi_driver.h"
#include "w25qxx.h"

#define CAN_MONITOR_IDENTIFIER       0x10000001U
#define CAN_MONITOR_COMMAND          0x11U
#define CAN_MONITOR_DEVICE_ID        0x01U
#define CAN_MONITOR_CHANNEL_COUNT    6U
#define CAN_MONITOR_QUERY_PERIOD_MS  200U
#define CAN_MONITOR_TIMEOUT_MS       150U
#define CAN_MONITOR_FAILURE_LIMIT    3U
#define CAN_MONITOR_DISPLAY_ID       71U
#define CAN_MONITOR_INDICATOR_RADIUS 15U
#define CAN_MONITOR_DEFAULT_COLOR    0xC618U
#define CAN_MONITOR_GREEN_COLOR      0x07E0U
#define CAN_MONITOR_RED_COLOR        0xF800U
#define CAN_MONITOR_YELLOW_COLOR     0xFFE0U
#define CAN_MONITOR_NAME_PAGE_ID     75U
#define CAN_MONITOR_NAME_FLASH_ADDR  0x114000UL
#define CAN_MONITOR_NAME_MAGIC       0x434E414DU
#define CAN_MONITOR_NAME_VERSION     1U
#define CAN_MONITOR_NAME_MAX_CHARS   13U
#define CAN_MONITOR_NAME_BYTES       32U
#define CAN_MONITOR_NAME_ALL_ITEM    6U

/* 新增功能：FCP-1011六路控制板；时间：2026-08-06 */
typedef struct
{
    uint8_t channel_state[CAN_MONITOR_CHANNEL_COUNT];
    uint8_t online;
    uint8_t awaiting_response;
    uint8_t failure_count;
    uint8_t query_started;
    uint32_t query_tick;
    uint32_t last_query_tick;
} CanMonitorContext_t;

static CanMonitorContext_t g_can_monitor;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t name[CAN_MONITOR_CHANNEL_COUNT][CAN_MONITOR_NAME_BYTES];
    uint32_t crc;
} CanMonitorNameStorage_t;

static CanMonitorNameStorage_t g_can_monitor_names;
static uint8_t g_can_monitor_pending_name[CAN_MONITOR_CHANNEL_COUNT][CAN_MONITOR_NAME_BYTES];
static uint8_t g_can_monitor_pending_mask;
static uint8_t g_can_monitor_names_loaded;
static uint32_t g_can_monitor_name_revision;

static const uint8_t g_can_monitor_default_name[CAN_MONITOR_CHANNEL_COUNT][CAN_MONITOR_NAME_BYTES] = {
    {0xB5U,0xDAU,0x20U,0x31U,0x20U,0xC2U,0xB7U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U},
    {0xB5U,0xDAU,0x20U,0x32U,0x20U,0xC2U,0xB7U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U},
    {0xB5U,0xDAU,0x20U,0x33U,0x20U,0xC2U,0xB7U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U},
    {0xB5U,0xDAU,0x20U,0x34U,0x20U,0xC2U,0xB7U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U},
    {0xB5U,0xDAU,0x20U,0x35U,0x20U,0xC2U,0xB7U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U},
    {0xB1U,0xB8U,0xD3U,0xC3U,0xC3U,0xF0U,0xBBU,0xF0U,0xC9U,0xE8U,0xB1U,0xB8U,0U}
};
static const uint8_t g_can_monitor_query[8] = {
    CAN_MONITOR_COMMAND, CAN_MONITOR_DEVICE_ID, 0U, 0U, 0U, 0U, 0U, 0U
};

typedef struct
{
    uint16_t x;
    uint16_t y;
} CanMonitorIndicatorPosition_t;

static const CanMonitorIndicatorPosition_t g_can_monitor_active_position[CAN_MONITOR_CHANNEL_COUNT] = {
    {267U, 202U}, {685U, 200U}, {265U, 348U},
    {685U, 344U}, {265U, 502U}, {685U, 498U}
};

static const CanMonitorIndicatorPosition_t g_can_monitor_fault_position[CAN_MONITOR_CHANNEL_COUNT] = {
    {414U, 202U}, {832U, 200U}, {414U, 348U},
    {832U, 344U}, {414U, 502U}, {832U, 498U}
};

static uint32_t CanMonitorNameCrc(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;
    for(index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for(bit = 0U; bit < 8U; bit++)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

static void CanMonitorCopyName(uint8_t *destination, const uint8_t *source)
{
    uint8_t source_index = 0U;
    uint8_t destination_index = 0U;
    uint8_t characters = 0U;
    memset(destination, 0, CAN_MONITOR_NAME_BYTES);
    if(source == NULL) return;
    while(source[source_index] != 0U && characters < CAN_MONITOR_NAME_MAX_CHARS)
    {
        uint8_t first = source[source_index];
        if(first < 0x80U)
        {
            destination[destination_index++] = first;
            source_index++;
        }
        else if(first >= 0x81U && first <= 0xFEU &&
                source[source_index + 1U] >= 0x40U && source[source_index + 1U] <= 0xFEU &&
                source[source_index + 1U] != 0x7FU)
        {
            destination[destination_index++] = first;
            destination[destination_index++] = source[source_index + 1U];
            source_index += 2U;
        }
        else break;
        characters++;
    }
}

static void CanMonitorLoadNames(void)
{
    uint32_t stored_crc;
    if(g_can_monitor_names_loaded != 0U) return;
    W25QXX_Read((uint8_t *)&g_can_monitor_names, CAN_MONITOR_NAME_FLASH_ADDR,
                (uint16_t)sizeof(g_can_monitor_names));
    stored_crc = g_can_monitor_names.crc;
    g_can_monitor_names.crc = 0U;
    if(g_can_monitor_names.magic != CAN_MONITOR_NAME_MAGIC ||
       g_can_monitor_names.version != CAN_MONITOR_NAME_VERSION ||
       g_can_monitor_names.size != sizeof(g_can_monitor_names) ||
       stored_crc != CanMonitorNameCrc((const uint8_t *)&g_can_monitor_names,
                                      (uint32_t)sizeof(g_can_monitor_names)))
    {
        memset(&g_can_monitor_names, 0, sizeof(g_can_monitor_names));
        g_can_monitor_names.magic = CAN_MONITOR_NAME_MAGIC;
        g_can_monitor_names.version = CAN_MONITOR_NAME_VERSION;
        g_can_monitor_names.size = (uint16_t)sizeof(g_can_monitor_names);
    }
    else g_can_monitor_names.crc = stored_crc;
    g_can_monitor_names_loaded = 1U;
}

void CanMonitorSetChannelName(uint8_t channel, const uint8_t *name)
{
    if(channel == 0U || channel > CAN_MONITOR_CHANNEL_COUNT) return;
    taskENTER_CRITICAL();
    CanMonitorCopyName(g_can_monitor_pending_name[channel - 1U], name);
    g_can_monitor_pending_mask |= (uint8_t)(1U << (channel - 1U));
    taskEXIT_CRITICAL();
}

void CanMonitorResetChannelName(uint8_t item)
{
    uint8_t mask;
    uint8_t index;
    if(item > CAN_MONITOR_NAME_ALL_ITEM) return;
    mask = (item == CAN_MONITOR_NAME_ALL_ITEM) ? 0x3FU : (uint8_t)(1U << item);
    taskENTER_CRITICAL();
    g_can_monitor_pending_mask |= mask;
    for(index = 0U; index < CAN_MONITOR_CHANNEL_COUNT; index++)
    {
        if((mask & (uint8_t)(1U << index)) != 0U)
            memset(g_can_monitor_pending_name[index], 0, CAN_MONITOR_NAME_BYTES);
    }
    taskEXIT_CRITICAL();
}

static void CanMonitorNameStorageProcess(void)
{
    uint8_t mask;
    uint8_t i;
    CanMonitorLoadNames();
    taskENTER_CRITICAL();
    mask = g_can_monitor_pending_mask;
    g_can_monitor_pending_mask = 0U;
    if(mask != 0U)
    {
        for(i = 0U; i < CAN_MONITOR_CHANNEL_COUNT; i++)
        {
            if((mask & (uint8_t)(1U << i)) != 0U)
                memcpy(g_can_monitor_names.name[i], g_can_monitor_pending_name[i], CAN_MONITOR_NAME_BYTES);
        }
    }
    taskEXIT_CRITICAL();
    if(mask == 0U) return;
    g_can_monitor_names.magic = CAN_MONITOR_NAME_MAGIC;
    g_can_monitor_names.version = CAN_MONITOR_NAME_VERSION;
    g_can_monitor_names.size = (uint16_t)sizeof(g_can_monitor_names);
    g_can_monitor_names.crc = 0U;
    g_can_monitor_names.crc = CanMonitorNameCrc((const uint8_t *)&g_can_monitor_names,
                                                (uint32_t)sizeof(g_can_monitor_names));
    W25QXX_Write((uint8_t *)&g_can_monitor_names, CAN_MONITOR_NAME_FLASH_ADDR,
                 (uint16_t)sizeof(g_can_monitor_names));
    g_can_monitor_name_revision++;
}

static void CanMonitorDrawIndicator(const CanMonitorIndicatorPosition_t *position, uint16_t color)
{
    SetFcolor(color);
    GUI_CircleFill(position->x, position->y, CAN_MONITOR_INDICATOR_RADIUS);
}

static uint8_t CanMonitorFrameIsValid(const FdcanFrame_t *frame)
{
    uint8_t i;
    if(frame->identifier != CAN_MONITOR_IDENTIFIER ||
       frame->id_type != FDCAN_EXTENDED_ID ||
       frame->frame_type != FDCAN_DATA_FRAME ||
       frame->length != 8U ||
       frame->data[0] != CAN_MONITOR_COMMAND ||
       frame->data[1] != CAN_MONITOR_DEVICE_ID)
    {
        return 0U;
    }
    for(i = 0U; i < CAN_MONITOR_CHANNEL_COUNT; i++)
    {
        if(frame->data[i + 2U] < CAN_MONITOR_STATE_NORMAL ||
           frame->data[i + 2U] > CAN_MONITOR_STATE_FAULT) return 0U;
    }
    return 1U;
}

static void CanMonitorHandleFrame(const FdcanFrame_t *frame)
{
    if(g_can_monitor.awaiting_response == 0U || CanMonitorFrameIsValid(frame) == 0U) return;
    taskENTER_CRITICAL();
    memcpy(g_can_monitor.channel_state, &frame->data[2], CAN_MONITOR_CHANNEL_COUNT);
    g_can_monitor.awaiting_response = 0U;
    g_can_monitor.failure_count = 0U;
    g_can_monitor.online = 1U;
    taskEXIT_CRITICAL();
}

void CanMonitorProcess(void)
{
    FdcanFrame_t frame;
    uint32_t now = osKernelGetTickCount();

    if(Fdcan1Receive(&frame) != 0U) CanMonitorHandleFrame(&frame);

    if(g_can_monitor.awaiting_response != 0U &&
       (uint32_t)(now - g_can_monitor.query_tick) >= CAN_MONITOR_TIMEOUT_MS)
    {
        g_can_monitor.awaiting_response = 0U;
        if(g_can_monitor.failure_count < CAN_MONITOR_FAILURE_LIMIT) g_can_monitor.failure_count++;
        if(g_can_monitor.failure_count >= CAN_MONITOR_FAILURE_LIMIT) g_can_monitor.online = 0U;
    }

    if(g_can_monitor.awaiting_response == 0U &&
       (g_can_monitor.query_started == 0U ||
        (uint32_t)(now - g_can_monitor.last_query_tick) >= CAN_MONITOR_QUERY_PERIOD_MS))
    {
        g_can_monitor.query_started = 1U;
        g_can_monitor.last_query_tick = now;
        if(FDCAN1_SendExtDataFrame(CAN_MONITOR_IDENTIFIER, (uint8_t *)g_can_monitor_query, 8U) == HAL_OK)
        {
            g_can_monitor.query_tick = now;
            g_can_monitor.awaiting_response = 1U;
        }
    }
}

void CanMonitorTask(void *parameter)
{
    (void)parameter;
    memset(&g_can_monitor, 0, sizeof(g_can_monitor));
    for(;;)
    {
        CanMonitorProcess();
        osDelay(20U);
    }
}

void CanMonitorRefreshDisplay(uint16_t screen_id)
{
    static const CanMonitorIndicatorPosition_t online_position = {907U, 84U};
    static uint8_t rendered = 0U;
    static uint8_t previous_online = 0xFFU;
    static uint8_t previous_state[CAN_MONITOR_CHANNEL_COUNT] = {0xFFU,0xFFU,0xFFU,0xFFU,0xFFU,0xFFU};
    static uint8_t name_page_rendered = 0U;
    static uint32_t rendered_name_revision = 0xFFFFFFFFUL;
    uint8_t i;
    uint8_t online_snapshot;
    uint8_t state_snapshot[CAN_MONITOR_CHANNEL_COUNT];

    CanMonitorNameStorageProcess();
    if(screen_id == CAN_MONITOR_NAME_PAGE_ID)
    {
        if(name_page_rendered == 0U)
        {
            for(i = 0U; i < CAN_MONITOR_CHANNEL_COUNT; i++)
                SetTextValue(CAN_MONITOR_NAME_PAGE_ID, (uint16_t)(i + 1U), (uint8_t *)"");
            name_page_rendered = 1U;
        }
    }
    else name_page_rendered = 0U;

    if(screen_id != CAN_MONITOR_DISPLAY_ID)
    {
        rendered = 0U;
        return;
    }

    taskENTER_CRITICAL();
    online_snapshot = g_can_monitor.online;
    memcpy(state_snapshot, g_can_monitor.channel_state, sizeof(state_snapshot));
    taskEXIT_CRITICAL();

    if(rendered == 0U || rendered_name_revision != g_can_monitor_name_revision)
    {
        for(i = 0U; i < CAN_MONITOR_CHANNEL_COUNT; i++)
        {
            uint8_t *name = g_can_monitor_names.name[i][0] != 0U ?
                            g_can_monitor_names.name[i] : (uint8_t *)g_can_monitor_default_name[i];
            SetTextValue(CAN_MONITOR_DISPLAY_ID, (uint16_t)(210U + i), name);
        }
        rendered_name_revision = g_can_monitor_name_revision;
    }

    if(rendered == 0U || previous_online != online_snapshot)
    {
        CanMonitorDrawIndicator(&online_position,
                                online_snapshot ? CAN_MONITOR_GREEN_COLOR : CAN_MONITOR_RED_COLOR);
        previous_online = online_snapshot;
    }

    for(i = 0U; i < CAN_MONITOR_CHANNEL_COUNT; i++)
    {
        uint8_t state = online_snapshot ? state_snapshot[i] : 0U;
        if(rendered == 0U || previous_state[i] != state)
        {
            uint16_t active_color = (state == CAN_MONITOR_STATE_STARTED || state == CAN_MONITOR_STATE_FEEDBACK) ?
                                     CAN_MONITOR_GREEN_COLOR : CAN_MONITOR_DEFAULT_COLOR;
            uint16_t fault_color = (state == CAN_MONITOR_STATE_FAULT) ?
                                    CAN_MONITOR_YELLOW_COLOR : CAN_MONITOR_DEFAULT_COLOR;
            CanMonitorDrawIndicator(&g_can_monitor_active_position[i], active_color);
            CanMonitorDrawIndicator(&g_can_monitor_fault_position[i], fault_color);
            previous_state[i] = state;
        }
    }
    rendered = 1U;
}

uint8_t CanMonitorIsOnline(void)
{
    return g_can_monitor.online;
}

uint8_t CanMonitorGetChannelState(uint8_t channel)
{
    if(channel == 0U || channel > CAN_MONITOR_CHANNEL_COUNT) return 0U;
    return g_can_monitor.channel_state[channel - 1U];
}
