#ifndef __XR5000_HMI_PASSWORD_WRAPPER_H
#define __XR5000_HMI_PASSWORD_WRAPPER_H

#include "../BSP/bsp_password.h"
#include "bsp_device_disable.h"

/* XR5000_DEVICE_DISABLE_20260803: add screen-70 handling to extension calls
 * that cmd_process.c already makes, while preserving the legacy handlers. */
#define InternalScreenLinkageMonitorUpdataUI(screen_id) \
    (DeviceDisableHmiScreenUpdate((screen_id)), InternalScreenLinkageMonitorUpdataUI((screen_id)))
#define InternalLinkageMonitorButtonDeal(screen_id, control_id, state) \
    (DeviceDisableHmiButton((screen_id), (control_id), (state)), \
     InternalLinkageMonitorButtonDeal((screen_id), (control_id), (state)))
#define OutFireDeviceInternalScreenButtonSet(screen_id, control_id, item, state, ctrl) \
    (DeviceDisableHmiMenu((screen_id), (control_id), (item), (state)), \
     OutFireDeviceInternalScreenButtonSet((screen_id), (control_id), (item), (state), (ctrl)))
#define OutFireDeviceInternalScreenTexttSet(screen_id, control_id, text, ctrl) \
    (DeviceDisableHmiText((screen_id), (control_id), (text)), \
     OutFireDeviceInternalScreenTexttSet((screen_id), (control_id), (text), (ctrl)))

/* The legacy page-6 code writes zero to these controls every refresh. Suppress
 * those three writes so the persistent bitmap is their only data source. */
#define SetTextValue(screen_id, control_id, text) \
    DeviceDisableHmiSetTextFilter((screen_id), (control_id), (uint8_t *)(text))

/* Keep bus polling and raw data updates active, but suppress loop 1/3 business
 * events in cmd_process.c for devices that are currently disabled. */
#define getPointTypeMixtureDisconnectCount(address) \
    (DeviceDisableIsLoopAddressSet(1U, (address)) ? 0U : getPointTypeMixtureDisconnectCount((address)))
#define getPointTypeMixtureReceiveState(kind, address) \
    (DeviceDisableIsLoopAddressSet(1U, (address)) ? 0U : getPointTypeMixtureReceiveState((kind), (address)))
#define RS485Detect_IsDisconnected(address) \
    (DeviceDisableIsLoopAddressSet(3U, (address)) ? 0U : RS485Detect_IsDisconnected((address)))
#define RS485Detect_GetSensorState(address, sensor) \
    (DeviceDisableIsLoopAddressSet(3U, (address)) ? 0U : RS485Detect_GetSensorState((address), (sensor)))

#endif
