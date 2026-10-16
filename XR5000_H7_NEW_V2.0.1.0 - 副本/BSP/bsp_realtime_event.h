#ifndef __BSP_REALTIME_EVENT_H
#define __BSP_REALTIME_EVENT_H

#include <stdint.h>

#define REALTIME_EVENT_SCREEN_ALARM       86U
#define REALTIME_EVENT_SCREEN_FAULT       87U
#define REALTIME_EVENT_CAPACITY           512U
#define REALTIME_EVENT_ROWS_PER_CATEGORY  7U

typedef enum
{
    REALTIME_EVENT_ALARM = 0,
    REALTIME_EVENT_FAULT,
    REALTIME_EVENT_KIND_COUNT
}RealtimeEventKind;

typedef enum
{
    REALTIME_ALARM_FIRE = 0,
    REALTIME_ALARM_GAS,
    REALTIME_ALARM_LINKAGE,
    REALTIME_ALARM_EXTINGUISH,
    REALTIME_ALARM_CATEGORY_COUNT
}RealtimeAlarmCategory;

typedef enum
{
    REALTIME_FAULT_CONTROLLER = 0,
    REALTIME_FAULT_DETECTOR,
    REALTIME_FAULT_LINKAGE,
    REALTIME_FAULT_EXTINGUISH,
    REALTIME_FAULT_CATEGORY_COUNT
}RealtimeFaultCategory;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
}RealtimeEventTime;

/*
 * Runtime-only event manager for screens 86 and 87.
 * It does not replace flash history and does not participate in control logic.
 */
void BspRealtimeEvent_Init(void);
void BspRealtimeEvent_Reset(void);
void BspRealtimeEvent_BeginScan(RealtimeEventKind kind);
uint8_t BspRealtimeEvent_Observe(RealtimeEventKind kind, uint8_t category,
                                 uint32_t key, const RealtimeEventTime *time,
                                 uint8_t loop, uint8_t address,
                                 const uint8_t *event_name);
void BspRealtimeEvent_EndScan(RealtimeEventKind kind);

void BspRealtimeEvent_OnScreenChanged(uint16_t previous_screen,
                                      uint16_t current_screen,
                                      uint32_t now_tick);
uint16_t BspRealtimeEvent_GetNavigationTarget(uint16_t current_screen,
                                              uint32_t now_tick);
void BspRealtimeEvent_Refresh(uint16_t current_screen, uint32_t now_tick);

uint16_t BspRealtimeEvent_GetCount(RealtimeEventKind kind, uint8_t category);
uint8_t BspRealtimeEvent_HasUnconfirmed(RealtimeEventKind kind);

/* Reserved for the later arrow-based confirmation operation. */
uint8_t BspRealtimeEvent_ConfirmVisible(RealtimeEventKind kind,
                                       uint8_t category, uint8_t row);

#endif
