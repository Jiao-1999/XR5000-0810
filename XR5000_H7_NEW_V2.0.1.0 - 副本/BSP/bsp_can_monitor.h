#ifndef __BSP_CAN_MONITOR_H
#define __BSP_CAN_MONITOR_H

#include "main.h"

/*
 * 六路多线盘接入画面86/87的编译开关：
 * 1U：显示启停和故障事件，并允许新事件触发实时页面跳转；
 * 0U：仅关闭画面86/87事件显示，不影响CAN轮询、画面71和总复位。
 */
#ifndef CAN_MONITOR_REALTIME_EVENT_ENABLE
#define CAN_MONITOR_REALTIME_EVENT_ENABLE  0U
#endif

typedef enum
{
    CAN_MONITOR_STATE_NORMAL = 1,
    CAN_MONITOR_STATE_STARTED = 2,
    CAN_MONITOR_STATE_FEEDBACK = 3,
    CAN_MONITOR_STATE_FAULT = 4
} CanMonitorState_t;

/* 六路多线盘人工操作事件；由CAN任务产生，画面任务异步消费。 */
typedef enum
{
    CAN_MONITOR_ACTION_NONE = 0,
    CAN_MONITOR_ACTION_START,
    CAN_MONITOR_ACTION_STOP
} CanMonitorActionType_t;

typedef struct
{
    uint8_t channel;                      /* 通道号，范围1～6 */
    uint8_t action;                       /* CanMonitorActionType_t */
    uint32_t sequence;                    /* 本次运行期内单调递增的事件序号 */
} CanMonitorActionEvent_t;

/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
void CanMonitorTask(void *parameter);
void CanMonitorProcess(void);
void CanMonitorRefreshDisplay(uint16_t screen_id);
void CanMonitorSetChannelName(uint8_t channel, const uint8_t *name);
void CanMonitorResetChannelName(uint8_t item);
uint8_t CanMonitorIsOnline(void);
uint8_t CanMonitorGetChannelState(uint8_t channel);
uint8_t CanMonitorHasEverOnline(void);
uint8_t CanMonitorPopActionEvent(CanMonitorActionEvent_t *event);
void CanMonitorRequestReset(void);
uint8_t CanMonitorGetResetStatus(void);

#endif
