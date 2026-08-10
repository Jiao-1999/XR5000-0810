#include "bsp_can_monitor.h"

#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_fdcan1.h"
#include "hmi_driver.h"

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

/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
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
    uint8_t i;
    uint8_t online_snapshot;
    uint8_t state_snapshot[CAN_MONITOR_CHANNEL_COUNT];

    if(screen_id != CAN_MONITOR_DISPLAY_ID)
    {
        rendered = 0U;
        return;
    }

    taskENTER_CRITICAL();
    online_snapshot = g_can_monitor.online;
    memcpy(state_snapshot, g_can_monitor.channel_state, sizeof(state_snapshot));
    taskEXIT_CRITICAL();

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
