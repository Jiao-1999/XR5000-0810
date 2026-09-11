#include "bsp_itcallback.h"
#include "usart.h"

#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "bsp_debug.h"
#include "bsp_fecbus_rx.h"   /* FECbus RX: USART3(???)/USART1(????) ????? IT ???? */

//UartBuffer_t uartbuff[10];

uint8_t screendata;

FdcanBuffer_t fdcanbuff[2];

__attribute__((section(".sram2"))) UartBuffer_t uartbuff[10];
volatile MBus2UartDiag_t g_mbus2_uart_diag;
volatile RS485DetectUartDiag_t g_rs4853_uart_diag;

#define RS4853_RX_RING_SIZE 512U
#define RS4853_RX_RING_MASK (RS4853_RX_RING_SIZE - 1U)
#define RS4853_DMA_RX_SIZE 128U
__attribute__((section(".sram2"), aligned(32))) static uint8_t g_rs4853_dma_rx_buffer[RS4853_DMA_RX_SIZE];
static uint8_t g_rs4853_rx_ring[RS4853_RX_RING_SIZE];
static volatile uint16_t g_rs4853_rx_head;
static volatile uint16_t g_rs4853_rx_tail;
static volatile uint8_t g_rs4853_tx_complete;

static void RS485DetectUartPushFromIsr(const uint8_t *data, uint16_t length)
{
	uint16_t head = g_rs4853_rx_head;
	uint16_t tail = g_rs4853_rx_tail;
	uint16_t i;

	for(i = 0U; i < length; i++)
	{
		uint16_t next = (uint16_t)((head + 1U) & RS4853_RX_RING_MASK);
		if(next == tail)
		{
			g_rs4853_uart_diag.rx_ring_overflow_count++;
			break;
		}
		g_rs4853_rx_ring[head] = data[i];
		head = next;
	}
	__DMB();
	g_rs4853_rx_head = head;
}

uint16_t RS485DetectUartRead(uint8_t *buffer, uint16_t capacity)
{
	uint16_t head;
	uint16_t tail;
	uint16_t count = 0U;

	if(buffer == NULL || capacity == 0U) return 0U;
	tail = g_rs4853_rx_tail;
	head = g_rs4853_rx_head;
	__DMB();
	while(tail != head && count < capacity)
	{
		buffer[count++] = g_rs4853_rx_ring[tail];
		tail = (uint16_t)((tail + 1U) & RS4853_RX_RING_MASK);
	}
	__DMB();
	g_rs4853_rx_tail = tail;
	return count;
}

void RS485DetectUartClearRx(void)
{
	uint16_t head = g_rs4853_rx_head;
	__DMB();
	g_rs4853_rx_tail = head;
}

void RS485DetectUartPrepareTx(void)
{
	g_rs4853_tx_complete = 0U;
	__DMB();
}

uint8_t RS485DetectUartTakeTxComplete(void)
{
	uint8_t completed = g_rs4853_tx_complete;
	if(completed != 0U)
	{
		g_rs4853_tx_complete = 0U;
		__DMB();
	}
	return completed;
}

static HAL_StatusTypeDef UartDmaEnsureRx(UART_HandleTypeDef *huart,
									 eUartOrder site,
									 volatile uint32_t *restart_fail_count,
									 volatile uint8_t *last_restart_status)
{
	HAL_StatusTypeDef status;

	if(huart->RxState == HAL_UART_STATE_BUSY_RX)
	{
		return HAL_OK;
	}

	status = HAL_UARTEx_ReceiveToIdle_DMA(huart, uartbuff[site].recepetion_buff, BUFF_MAX);
	*last_restart_status = (uint8_t)status;
	if(status != HAL_OK)
	{
		(*restart_fail_count)++;
	}
	return status;
}

HAL_StatusTypeDef MBus2UartEnsureRx(void)
{
	return UartDmaEnsureRx(&huart2, MBUS2SITE,
							 &g_mbus2_uart_diag.rx_restart_fail_count,
							 &g_mbus2_uart_diag.last_rx_restart_status);
}

HAL_StatusTypeDef RS485DetectUartEnsureRx(void)
{
	HAL_StatusTypeDef status;

	if(huart5.RxState == HAL_UART_STATE_BUSY_RX)
	{
		return HAL_OK;
	}

	status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_rs4853_dma_rx_buffer, RS4853_DMA_RX_SIZE);
	g_rs4853_uart_diag.last_rx_restart_status = (uint8_t)status;
	if(status != HAL_OK)
	{
		g_rs4853_uart_diag.rx_restart_fail_count++;
	}
	return status;
}

void UartBufferInit(void)
{
	memset(uartbuff, 0, sizeof(UartBuffer_t) * 10);
	g_rs4853_rx_head = 0U;
	g_rs4853_rx_tail = 0U;
	g_rs4853_tx_complete = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART5)
	{
		g_rs4853_tx_complete = 1U;
		g_rs4853_uart_diag.tx_complete_count++;
		__DMB();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART8) {
		// ????????????§¿????????????§Õ???
		queue_push(screendata);
//		HAL_UART_Receive_IT(&huart8, &screendata, 1); // ????????¦Í???
		while(HAL_UART_Receive_IT(&huart8,&screendata,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart8);
		}
  }
  else if (huart->Instance == USART3) {
		/* FECbus RX (??????): ????? + ????????????? */
		FecbusRx_OnByte(g_fecbus_rx_byte);
		while(HAL_UART_Receive_IT(&huart3,(uint8_t *)&g_fecbus_rx_byte,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart3);
		}
  }
  else if (huart->Instance == USART1) {
		/* FECbus RX (???????): ???????, ?? USART3 ?????????????? */
		FecbusRx_OnByte(g_fecbus_rx_byte_u1);
		while(HAL_UART_Receive_IT(&huart1,(uint8_t *)&g_fecbus_rx_byte_u1,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart1);
		}
  }
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if (huart->Instance == USART1)
//  {
//		uartbuff[0].recepetion_flag = 1;
//		uartbuff[0].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartbuff[0].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART2)
//  {
//		uartbuff[1].recepetion_flag = 1;
//		uartbuff[1].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart2, uartbuff[1].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART3)
//  {
//		uartbuff[2].recepetion_flag = 1;
//		uartbuff[2].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart3, uartbuff[2].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART4)
//  {
//		uartbuff[3].recepetion_flag = 1;
//		uartbuff[3].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART5)
//  {
//		uartbuff[4].recepetion_flag = 1;
//		uartbuff[4].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart5, uartbuff[4].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART6)
//	{
//		uartbuff[5].recepetion_flag = 1;
//		uartbuff[5].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART7)
//	{
//		uartbuff[6].recepetion_flag = 1;
//		uartbuff[6].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart7, uartbuff[6].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART8)
//	{
////		uartbuff[7].recepetion_flag = 1;
////		uartbuff[7].recepetion_len = Size;
////		HAL_UARTEx_ReceiveToIdle_IT(&huart8, uartbuff[7].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART9)
//	{
//		uartbuff[8].recepetion_flag = 1;
//		uartbuff[8].recepetion_len = Size;
//		
////		// ???Cache?????
////    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[8].recepetion_buff, BUFF_MAX);
////		HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
//		
//		HAL_UARTEx_ReceiveToIdle_IT(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
//		
//	}
//	else if (huart->Instance == USART10)
//	{
//		uartbuff[9].recepetion_flag = 1;
//		uartbuff[9].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart10, uartbuff[9].recepetion_buff, BUFF_MAX);
//	}
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
  {
		/* FECbus ????? 20260818: USART1 ??? DMA ???, ???? FecbusRx_OnByte ????? IT ????(???????) */
	}
	else if (huart->Instance == USART2)
  {
		// ???Cache?????
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[1].recepetion_buff, BUFF_MAX);
		g_mbus2_uart_diag.rx_event_count++;
		g_mbus2_uart_diag.last_rx_size = Size;
		if(Size < 5U) g_mbus2_uart_diag.rx_short_event_count++;
		uartbuff[1].recepetion_len = Size;
		(void)MBus2UartEnsureRx();
		__DMB();
		uartbuff[1].recepetion_flag = 1;
	}
	else if (huart->Instance == USART3)
  {
		/* FECbus ??? USART3 ???? (bsp_fecbus_rx.c ?????IT), ???DMA/uartbuff[2] */
	}
	else if (huart->Instance == UART4)
  {
		uartbuff[3].recepetion_flag = 1;
		uartbuff[3].recepetion_len = Size;

		// ???Cache?????
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[3].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART5)
  {
		SCB_InvalidateDCache_by_Addr((uint32_t*)g_rs4853_dma_rx_buffer, RS4853_DMA_RX_SIZE);
		g_rs4853_uart_diag.rx_event_count++;
		g_rs4853_uart_diag.last_rx_size = Size;
		if(Size < 5U) g_rs4853_uart_diag.rx_short_event_count++;
		RS485DetectUartPushFromIsr(g_rs4853_dma_rx_buffer, Size);
		(void)RS485DetectUartEnsureRx();
	}
	else if (huart->Instance == USART6)
	{
		uartbuff[5].recepetion_flag = 1;
		uartbuff[5].recepetion_len = Size;

		// ???Cache?????
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[5].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART7)
	{
		// ???Cache?????
		SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[6].recepetion_buff, BUFF_MAX);
		uartbuff[6].recepetion_len = Size;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uartbuff[6].recepetion_buff, BUFF_MAX);
		__DMB();
		uartbuff[6].recepetion_flag = 1;
	}
	else if (huart->Instance == UART8)
	{
//		uartbuff[7].recepetion_flag = 1;
//		uartbuff[7].recepetion_len = Size;
//		HAL_UARTEx_ReceiveToIdle_IT(&huart8, uartbuff[7].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART9)
	{
		uartbuff[8].recepetion_flag = 1;
		uartbuff[8].recepetion_len = Size;
		
		// ???Cache?????
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[8].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
		
	}
	else if (huart->Instance == USART10)
	{
		uartbuff[9].recepetion_flag = 1;
		uartbuff[9].recepetion_len = Size;
		
		// ???Cache?????
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[9].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uartbuff[9].recepetion_buff, BUFF_MAX);
	}
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if (huart->Instance == USART1)
//  {
//		uartbuff[0].recepetion_flag = 1;
//		uartbuff[0].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartbuff[0].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART2)
//  {
//		uartbuff[1].recepetion_flag = 1;
//		uartbuff[1].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartbuff[1].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART3)
//  {
//		uartbuff[2].recepetion_flag = 1;
//		uartbuff[2].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uartbuff[2].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART4)
//  {
//		uartbuff[3].recepetion_flag = 1;
//		uartbuff[3].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART5)
//  {
//		uartbuff[4].recepetion_flag = 1;
//		uartbuff[4].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uartbuff[4].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == USART6)
//	{
//		uartbuff[5].recepetion_flag = 1;
//		uartbuff[5].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART7)
//	{
//		uartbuff[6].recepetion_flag = 1;
//		uartbuff[6].recepetion_len = Size;

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uartbuff[6].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART8)
//	{
////		uartbuff[7].recepetion_flag = 1;
////		uartbuff[7].recepetion_len = Size;
////		HAL_UARTEx_ReceiveToIdle_IT(&huart8, uartbuff[7].recepetion_buff, BUFF_MAX);
//	}
//	else if (huart->Instance == UART9)
//	{
//		uartbuff[8].recepetion_flag = 1;
//		uartbuff[8].recepetion_len = Size;
//		
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
//		
//	}
//	else if (huart->Instance == USART10)
//	{
//		uartbuff[9].recepetion_flag = 1;
//		uartbuff[9].recepetion_len = Size;
//		
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uartbuff[9].recepetion_buff, BUFF_MAX);
//	}
//}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U) g_mbus2_uart_diag.frame_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U) g_mbus2_uart_diag.noise_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) g_mbus2_uart_diag.overrun_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U) g_mbus2_uart_diag.parity_error_count++;
	}
	else if(huart->Instance == UART5)
	{
		if((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U) g_rs4853_uart_diag.frame_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U) g_rs4853_uart_diag.noise_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) g_rs4853_uart_diag.overrun_error_count++;
		if((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U) g_rs4853_uart_diag.parity_error_count++;
	}

    /* 1. ??????? (FE) */
    if(huart->ErrorCode & HAL_UART_ERROR_FE) {
        __HAL_UART_CLEAR_FEFLAG(huart);  // ?????????? ?????
//        volatile uint8_t temp = huart->Instance->DR; // ???DR???????????????
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 2. ????????? (ORE) */
    if(huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart); // ???????
        // ????DMA??????????? (USART3=FECbus IT????, ????????DMA)
        if((huart->hdmarx != NULL) && (huart->Instance != USART3) && (huart->Instance != USART2) && (huart->Instance != UART5)) {
            HAL_UART_DMAStop(huart);
						//huart->hdmarx->Instance->CNDTR = BUFF_MAX;
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    }

    /* 3. ?????????? (NE) */
    if(huart->ErrorCode & HAL_UART_ERROR_NE) {
        __HAL_UART_CLEAR_NEFLAG(huart); // ????????
      //  volatile uint8_t temp = huart->Instance->DR; // ??????????
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 4. ???§µ??????? (PE) */
    if(huart->ErrorCode & HAL_UART_ERROR_PE) {
        __HAL_UART_CLEAR_PEFLAG(huart); // ??????????
      //  volatile uint8_t temp = huart->Instance->DR; // ??????????
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }
		
		if(huart->Instance == UART8)
		{
			HAL_UART_Receive_IT(&huart8, &screendata, 1);
		}
		else if (huart->Instance == USART1)
		{
			/* FECbus ?????: USART1 ????????? IT ????(??????????????) */
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_fecbus_rx_byte_u1, 1);
		}
		else if (huart->Instance == USART2)
		{
			(void)MBus2UartEnsureRx();
		}
		else if (huart->Instance == USART3)
		{
			/* FECbus RX (??????): ????????? IT ???? */
			HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_fecbus_rx_byte, 1);
		}
		else if (huart->Instance == UART4)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == UART5)
		{
			(void)RS485DetectUartEnsureRx();
		}
		else if (huart->Instance == USART6)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == UART7)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uartbuff[6].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == UART9)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == USART10)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uartbuff[9].recepetion_buff, BUFF_MAX);
		}
}


// CAN??????: FDCAN1 RX FIFO0 ?????
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance == FDCAN1) 
	{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) 
		{
      FDCAN_RxHeaderTypeDef RxHeader;
			
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, fdcanbuff[FDCAN1SITE].recepetion_buff) == HAL_OK) 
			{
				/* ????????: FCP-1011??¡¤?????; ??? 2026-08-06 */
			fdcanbuff[FDCAN1SITE].identifier = RxHeader.Identifier;
				fdcanbuff[FDCAN1SITE].id_type = RxHeader.IdType;
				fdcanbuff[FDCAN1SITE].frame_type = RxHeader.RxFrameType;
				fdcanbuff[FDCAN1SITE].recepetion_len = (uint8_t)RxHeader.DataLength; /* HAL?????DLC????????? */
				fdcanbuff[FDCAN1SITE].recepetion_flag = 1U;
//				if(RxHeader.Identifier == ID5306_EXTEND_FRAME_ID)
//				{
//					fdcanbuff[FDCAN1SITE].recepetion_flag = 1;
//					fdcanbuff[FDCAN1SITE].recepetion_len = RxHeader.DataLength;
//				}
			}
    }
  }
}

