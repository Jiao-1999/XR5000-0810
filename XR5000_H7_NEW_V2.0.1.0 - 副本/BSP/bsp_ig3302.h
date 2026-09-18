#ifndef __BSP_IG3302_H
#define __BSP_IG3302_H

#include "main.h"

#define IG3302_MAX_ADDRESS                 10U
#define IG3302_DISCONNECT_THRESHOLD        3U
#define IG3302_RECOVERY_SUCCESS_THRESHOLD  2U
#define IG3302_FLASH_ID                    0x54U

typedef enum
{
    IG3302_FAN_STOPPED = 0,
    IG3302_FAN_RUNNING = 1,
    IG3302_FAN_PUSHROD_FAULT = 2,
    IG3302_FAN_FAULT = 3,
    IG3302_FAN_AND_PUSHROD_FAULT = 4,
    IG3302_FAN_STATE_INVALID = 0xFF
} IG3302FanState;

typedef enum
{
    IG3302_CTRL_OK = 0,
    IG3302_CTRL_INVALID_PARAM,
    IG3302_CTRL_NOT_CONFIGURED,
    IG3302_CTRL_QUEUE_FULL
} IG3302CtrlResult;

typedef struct
{
    uint8_t address;
    uint8_t fan;    /* 1 or 2 */
    uint8_t start;  /* 0: stop, non-zero: start */
} IG3302CtrlRequest;

void IG3302_Init(void);
void IG3302_PollAndReceiveTask(void *argument);

uint8_t IG3302_SetOnline(uint8_t address, uint8_t online);
uint8_t IG3302_IsConfigured(uint8_t address);
uint8_t IG3302_IsActive(uint8_t address);
uint8_t IG3302_IsDisconnected(uint8_t address);
uint8_t IG3302_GetFanState(uint8_t address, uint8_t fan);
uint8_t IG3302_GetConfiguredCount(void);
uint8_t IG3302_GetActiveCount(void);
uint8_t IG3302_GetFaultDeviceCount(void);
uint32_t IG3302_GetRevision(void);

void IG3302_SaveOnlineState(void);
void IG3302_LoadOnlineState(void);
IG3302CtrlResult IG3302_Request(const IG3302CtrlRequest *request);

#endif
