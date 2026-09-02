#include "bsp_fdcan1.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_itcallback.h"

/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
static uint32_t FdcanLengthToDlc(uint8_t length)
{
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    return dlc_table[length];
}

void FDCAN1_Start(void)
{
    FDCAN_FilterTypeDef filter;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) Error_Handler();

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) Error_Handler();

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) Error_Handler();
}

HAL_StatusTypeDef FDCAN1_SendStdDataFrame(uint32_t id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header;
    if(length > 8U || (length > 0U && data == NULL)) return HAL_ERROR;
    header.Identifier = id & 0x7FFU;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FdcanLengthToDlc(length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

HAL_StatusTypeDef FDCAN1_SendExtDataFrame(uint32_t id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header;
    if(length > 8U || (length > 0U && data == NULL)) return HAL_ERROR;
    header.Identifier = id & 0x1FFFFFFFU;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FdcanLengthToDlc(length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

HAL_StatusTypeDef FDCAN1_SendRemoteFrame(uint32_t id, uint8_t is_extended)
{
    FDCAN_TxHeaderTypeDef header;
    header.Identifier = is_extended ? (id & 0x1FFFFFFFU) : (id & 0x7FFU);
    header.IdType = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_REMOTE_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_0;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, NULL);
}

uint8_t Fdcan1Receive(FdcanFrame_t *frame)
{
    if(frame == NULL) return 0U;
    taskENTER_CRITICAL();
    if(fdcanbuff[FDCAN1SITE].recepetion_flag == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    frame->identifier = fdcanbuff[FDCAN1SITE].identifier;
    frame->id_type = fdcanbuff[FDCAN1SITE].id_type;
    frame->frame_type = fdcanbuff[FDCAN1SITE].frame_type;
    frame->length = fdcanbuff[FDCAN1SITE].recepetion_len;
    memcpy(frame->data, fdcanbuff[FDCAN1SITE].recepetion_buff, sizeof(frame->data));
    fdcanbuff[FDCAN1SITE].recepetion_flag = 0U;
    taskEXIT_CRITICAL();
    return 1U;
}
