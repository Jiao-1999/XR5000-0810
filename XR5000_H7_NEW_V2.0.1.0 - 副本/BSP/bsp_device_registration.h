#ifndef BSP_DEVICE_REGISTRATION_H
#define BSP_DEVICE_REGISTRATION_H

#include <stdint.h>

/* 独立的物理登记层；登记标记不等于业务层的设置上线。 */
#define DEVICE_REG_LOOP1 1U
#define DEVICE_REG_LOOP2 2U
#define DEVICE_REG_LOOP3 3U
#define DEVICE_REG_ALL_MASK 0x07U
#define DEVICE_REG_MAX_ADDRESS 110U
#define DEVICE_REG_MAX_ENTRIES 237U

typedef enum
{
    DEVICE_REG_SCAN_IDLE = 0,
    DEVICE_REG_SCAN_RUNNING,
    DEVICE_REG_SCAN_SAVING,
    DEVICE_REG_SCAN_DONE,
    DEVICE_REG_SCAN_SAVE_ERROR
} DeviceRegScanState;

typedef enum
{
    DEVICE_REG_PROBE_NO_RESPONSE = 0,
    DEVICE_REG_PROBE_IDENTIFIED,
    DEVICE_REG_PROBE_UNIDENTIFIED
} DeviceRegProbeResult;

typedef enum
{
    DEVICE_REG_OBSERVATION_NOT_SCANNED = 0,
    DEVICE_REG_OBSERVATION_IDENTIFIED,
    DEVICE_REG_OBSERVATION_NO_RESPONSE,
    DEVICE_REG_OBSERVATION_UNIDENTIFIED
} DeviceRegObservation;

typedef struct
{
    uint16_t registered;
    uint16_t scan_identified;
    uint16_t registered_no_response;
    uint16_t unidentified;
    uint16_t newly_registered;
    uint16_t scanned_addresses;
    uint16_t total_addresses;
} DeviceRegStats;

typedef struct
{
    uint8_t loop;
    uint8_t address;
    uint16_t product_code;
    DeviceRegObservation observation;
} DeviceRegEntry;

void DeviceReg_Init(void);
uint8_t DeviceReg_StartScan(uint8_t loop_mask);
uint8_t DeviceReg_Clear(uint8_t loop_mask);
uint8_t DeviceReg_TakeProbe(uint8_t loop, uint8_t *address);
void DeviceReg_CancelProbe(uint8_t loop, uint8_t address);
void DeviceReg_CompleteProbe(uint8_t loop, uint8_t address,
                            DeviceRegProbeResult result, uint16_t product_code);
void DeviceReg_ServiceSave(void);
DeviceRegScanState DeviceReg_GetState(void);
void DeviceReg_GetStats(uint8_t loop, DeviceRegStats *stats);
uint8_t DeviceReg_GetEntry(uint8_t loop, uint8_t address, DeviceRegEntry *entry);
uint16_t DeviceReg_List(uint8_t loop_mask, DeviceRegEntry *entries, uint16_t capacity);
uint32_t DeviceReg_GetRevision(void);

#endif
