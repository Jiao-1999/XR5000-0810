#ifndef __BSP_DEVICE_THRESHOLD_H
#define __BSP_DEVICE_THRESHOLD_H

#include "main.h"

/* Device threshold configuration/query service.
 * The service owns only static storage.  UART5 remains exclusively owned by
 * bsp_rs485_detect.c; HMI callbacks merely submit or cancel requests. */

typedef struct
{
    uint8_t online;
    uint8_t identified;
    uint8_t device_type;
    uint16_t national_code;
    uint16_t product_code;
    uint16_t sensor_enable;
} DeviceThresholdIdentity;

void DeviceThreshold_Init(void);
void DeviceThreshold_NotifyScreen(uint16_t screen_id);
void DeviceThreshold_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
void DeviceThreshold_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);
void DeviceThreshold_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state);
void DeviceThreshold_UpdateUI(uint16_t screen_id, uint8_t fire_active);

/* UART5 owner integration.  Frames are always eight bytes. */
uint8_t DeviceThreshold_BuildNextFrame(uint8_t frame[8], uint8_t *address);
uint8_t DeviceThreshold_HandleResponse(const uint8_t *frame, uint16_t length);
void DeviceThreshold_HandleTimeout(void);
void DeviceThreshold_NotifyNormalPoll(void);

/* Implemented by bsp_rs485_detect.c without changing its existing public API. */
uint8_t DeviceThreshold_GetLoop3Identity(uint8_t address, DeviceThresholdIdentity *identity);

#endif
