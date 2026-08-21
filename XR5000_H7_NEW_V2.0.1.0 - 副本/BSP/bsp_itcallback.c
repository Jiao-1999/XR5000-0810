#include "bsp_itcallback.h"
#include "usart.h"

#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "bsp_debug.h"
#include "bsp_fecbus_rx.h"   /* FECbus RX: USART3 ï¿½ï¿½ï¿½Ö½ï¿½ITï¿½ï¿½ï¿½ï¿½ï¿½ë»· */

//UartBuffer_t uartbuff[10];

uint8_t screendata;

FdcanBuffer_t fdcanbuff[2];

__attribute__((section(".sram2"))) UartBuffer_t uartbuff[10];

void UartBufferInit(void)
{
	memset(uartbuff, 0, sizeof(UartBuffer_t) * 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART8) {
		// ²ÉÓÃ´ó²Ê¿Æ¼¼Ìá¹©µÄÐ­ÒéÀ´¶ÔÊý¾Ý½øÐÐ´¦Àí
		queue_push(screendata);
//		HAL_UART_Receive_IT(&huart8, &screendata, 1); // ¿ªÆôÏÂÒ»´Î½ÓÊÕ
		while(HAL_UART_Receive_IT(&huart8,&screendata,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart8);
		}
  }
  else if (huart->Instance == USART3) {
		/* FECbus RX: ï¿½Ö½ï¿½ï¿½ë»· + ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Ö½Ú½ï¿½ï¿½ï¿½ */
		FecbusRx_OnByte(g_fecbus_rx_byte);
		while(HAL_UART_Receive_IT(&huart3,(uint8_t *)&g_fecbus_rx_byte,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart3);
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
////		// Î¬»¤CacheÒ»ÖÂÐÔ
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
		uartbuff[0].recepetion_flag = 1;
		uartbuff[0].recepetion_len = Size;
		
		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[0].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartbuff[0].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == USART2)
  {
		uartbuff[1].recepetion_flag = 1;
		uartbuff[1].recepetion_len = Size;
		
		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[1].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartbuff[1].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == USART3)
  {
		/* FECbus ï¿½ï¿½Õ¼ USART3 ï¿½ï¿½ï¿½ï¿½ (bsp_fecbus_rx.c ï¿½ï¿½ï¿½Ö½ï¿½IT), ï¿½ï¿½ï¿½ï¿½DMA/uartbuff[2] */
	}
	else if (huart->Instance == UART4)
  {
		uartbuff[3].recepetion_flag = 1;
		uartbuff[3].recepetion_len = Size;

		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[3].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART5)
  {
		uartbuff[4].recepetion_flag = 1;
		uartbuff[4].recepetion_len = Size;

		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[4].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uartbuff[4].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == USART6)
	{
		uartbuff[5].recepetion_flag = 1;
		uartbuff[5].recepetion_len = Size;

		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[5].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART7)
	{
		// Î¬»¤CacheÒ»ÖÂÐÔ
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
		
		// Î¬»¤CacheÒ»ÖÂÐÔ
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[8].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
		
	}
	else if (huart->Instance == USART10)
	{
		uartbuff[9].recepetion_flag = 1;
		uartbuff[9].recepetion_len = Size;
		
		// Î¬»¤CacheÒ»ÖÂÐÔ
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
    /* 1. Ö¡´íÎó´¦Àí (FE) */
    if(huart->ErrorCode & HAL_UART_ERROR_FE) {
        __HAL_UART_CLEAR_FEFLAG(huart);  // ±ØÐëÇå³ý±êÖ¾ Ö¡´íÎó
//        volatile uint8_t temp = huart->Instance->DR; // ¶ÁÈ¡DR¼Ä´æÆ÷Çå¿Õ´íÎóÊý¾Ý
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 2. Òç³ö´íÎó´¦Àí (ORE) */
    if(huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
        // ï¿½ï¿½ï¿½ï¿½DMAÄ£Ê½ï¿½ï¿½Òªï¿½ï¿½ï¿½â´¦ï¿½ï¿½ (USART3=FECbus ITï¿½ï¿½ï¿½ï¿½, ï¿½Å³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½DMA)
        if((huart->hdmarx != NULL) && (huart->Instance != USART3)) {
            HAL_UART_DMAStop(huart);
						//huart->hdmarx->Instance->CNDTR = BUFF_MAX;
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    }

    /* 3. ÔëÉù´íÎó´¦Àí (NE) */
    if(huart->ErrorCode & HAL_UART_ERROR_NE) {
        __HAL_UART_CLEAR_NEFLAG(huart); // ÔëÉù´íÎó
      //  volatile uint8_t temp = huart->Instance->DR; // Çå¿Õ´íÎóÊý¾Ý
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 4. ÆæÅ¼Ð£Ñé´íÎó´¦Àí (PE) */
    if(huart->ErrorCode & HAL_UART_ERROR_PE) {
        __HAL_UART_CLEAR_PEFLAG(huart); // ÆæÅ¼¼ìÑé´íÎó
      //  volatile uint8_t temp = huart->Instance->DR; // Çå¿Õ´íÎóÊý¾Ý
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }
		
		if(huart->Instance == UART8)
		{
			HAL_UART_Receive_IT(&huart8, &screendata, 1);
		}
		else if (huart->Instance == USART1)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartbuff[0].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == USART2)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartbuff[1].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == USART3)
		{
			/* FECbus RX: ï¿½ï¿½ï¿½ï¿½Ö¸ï¿? ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½ï¿½ITï¿½ï¿½ï¿½ï¿½ (bsp_fecbus_rx.c) */
			HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_fecbus_rx_byte, 1);
		}
		else if (huart->Instance == UART4)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == UART5)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uartbuff[4].recepetion_buff, BUFF_MAX);
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


// CANÖÐ¶Ï»Øµ÷º¯Êý
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance == FDCAN1) 
	{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) 
		{
      FDCAN_RxHeaderTypeDef RxHeader;
			
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, fdcanbuff[FDCAN1SITE].recepetion_buff) == HAL_OK) 
			{
				/* ÐÂ¼Ó¹¦ÄÜ£ºFCP-1011ÁùÂ·¿ØÖÆ°å£»Ê±¼ä£º2026-08-06 */
				fdcanbuff[FDCAN1SITE].identifier = RxHeader.Identifier;
				fdcanbuff[FDCAN1SITE].id_type = RxHeader.IdType;
				fdcanbuff[FDCAN1SITE].frame_type = RxHeader.RxFrameType;
				fdcanbuff[FDCAN1SITE].recepetion_len = (uint8_t)RxHeader.DataLength; /* HALÒÑ·µ»Ø½âÎöºóµÄDLCÖµ */
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

