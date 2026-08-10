#ifndef __BSP_CAN_MONITOR_H
#define __BSP_CAN_MONITOR_H

#include "main.h"

typedef enum
{
    CAN_MONITOR_STATE_NORMAL = 1,
    CAN_MONITOR_STATE_STARTED = 2,
    CAN_MONITOR_STATE_FEEDBACK = 3,
    CAN_MONITOR_STATE_FAULT = 4
} CanMonitorState_t;

/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
void CanMonitorTask(void *parameter);
void CanMonitorProcess(void);
void CanMonitorRefreshDisplay(uint16_t screen_id);
uint8_t CanMonitorIsOnline(void);
uint8_t CanMonitorGetChannelState(uint8_t channel);

#endif
