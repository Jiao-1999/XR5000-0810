#ifndef __BSP_FDCAN1_H
#define __BSP_FDCAN1_H

#include "main.h"
#include "fdcan.h"

typedef struct
{
    uint32_t identifier;
    uint32_t id_type;
    uint32_t frame_type;
    uint8_t length;
    uint8_t data[8];
} FdcanFrame_t;

/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
void FDCAN1_Start(void);
HAL_StatusTypeDef FDCAN1_SendStdDataFrame(uint32_t id, uint8_t *data, uint8_t length);
HAL_StatusTypeDef FDCAN1_SendExtDataFrame(uint32_t id, uint8_t *data, uint8_t length);
HAL_StatusTypeDef FDCAN1_SendRemoteFrame(uint32_t id, uint8_t is_extended);
uint8_t Fdcan1Receive(FdcanFrame_t *frame);

#endif
