#include "bsp_itcallback.h"
#include "usart.h"

#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "bsp_debug.h"
#include "bsp_fecbus_rx.h"  /* FECbus RX: USART3 逐字节IT接收入环 */

//UartBuffer_t uartbuff[10];

uint8_t screendata;

FdcanBuffer_t fdcanbuff[2];

__attribute__((section(".sram2"))) UartBuffer_t uartbuff[10];
volatile MBus2UartDiag_t g_mbus2_uart_diag;
volatile RS485DetectUartDiag_t g_rs4853_uart_diag;
volatile IG3302UartDiag_t g_ig3302_uart_diag;

#define MBUS2_RX_RING_SIZE 512U
#define MBUS2_RX_RING_MASK (MBUS2_RX_RING_SIZE - 1U)
#define MBUS2_DMA_RX_SIZE 128U
__attribute__((section(".sram2"), aligned(32))) static uint8_t g_mbus2_dma_rx_buffer[MBUS2_DMA_RX_SIZE];
static uint8_t g_mbus2_rx_ring[MBUS2_RX_RING_SIZE];
static volatile uint16_t g_mbus2_rx_head;
static volatile uint16_t g_mbus2_rx_tail;

#define RS4853_RX_RING_SIZE 512U
#define RS4853_RX_RING_MASK (RS4853_RX_RING_SIZE - 1U)
#define RS4853_DMA_RX_SIZE 128U
__attribute__((section(".sram2"), aligned(32))) static uint8_t g_rs4853_dma_rx_buffer[RS4853_DMA_RX_SIZE];
static uint8_t g_rs4853_rx_ring[RS4853_RX_RING_SIZE];
static volatile uint16_t g_rs4853_rx_head;
static volatile uint16_t g_rs4853_rx_tail;
static volatile uint8_t g_rs4853_tx_complete;

#define IG3302_RX_RING_SIZE 256U
#define IG3302_RX_RING_MASK (IG3302_RX_RING_SIZE - 1U)
#define IG3302_DMA_RX_SIZE 64U
__attribute__((section(".sram2"), aligned(32))) static uint8_t g_ig3302_dma_rx_buffer[IG3302_DMA_RX_SIZE];
static uint8_t g_ig3302_rx_ring[IG3302_RX_RING_SIZE];
static volatile uint16_t g_ig3302_rx_head;
static volatile uint16_t g_ig3302_rx_tail;
static volatile uint8_t g_ig3302_dma_active;

static void MBus2UartPushFromIsr(const uint8_t *data, uint16_t length)
{
	uint16_t head = g_mbus2_rx_head;
	uint16_t tail = g_mbus2_rx_tail;
	uint16_t i;

	for(i = 0U; i < length; i++)
	{
		uint16_t next = (uint16_t)((head + 1U) & MBUS2_RX_RING_MASK);
		if(next == tail)
		{
			g_mbus2_uart_diag.rx_ring_overflow_count++;
			break;
		}
		g_mbus2_rx_ring[head] = data[i];
		head = next;
	}
	__DMB();
	g_mbus2_rx_head = head;
}

uint16_t MBus2UartRead(uint8_t *buffer, uint16_t capacity)
{
	uint16_t head;
	uint16_t tail;
	uint16_t count = 0U;

	if(buffer == NULL || capacity == 0U) return 0U;
	tail = g_mbus2_rx_tail;
	head = g_mbus2_rx_head;
	__DMB();
	while(tail != head && count < capacity)
	{
		buffer[count++] = g_mbus2_rx_ring[tail];
		tail = (uint16_t)((tail + 1U) & MBUS2_RX_RING_MASK);
	}
	__DMB();
	g_mbus2_rx_tail = tail;
	return count;
}

void MBus2UartClearRx(void)
{
	uint16_t head = g_mbus2_rx_head;
	__DMB();
	g_mbus2_rx_tail = head;
}

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

static void IG3302UartPushFromIsr(const uint8_t *data, uint16_t length)
{
	uint16_t head = g_ig3302_rx_head;
	uint16_t tail = g_ig3302_rx_tail;
	uint16_t i;
	for(i = 0U; i < length; i++)
	{
		uint16_t next = (uint16_t)((head + 1U) & IG3302_RX_RING_MASK);
		if(next == tail)
		{
			g_ig3302_uart_diag.rx_ring_overflow_count++;
			break;
		}
		g_ig3302_rx_ring[head] = data[i];
		head = next;
	}
	__DMB();
	g_ig3302_rx_head = head;
}

uint16_t IG3302UartRead(uint8_t *buffer, uint16_t capacity)
{
	uint16_t head;
	uint16_t tail;
	uint16_t count = 0U;
	if(buffer == NULL || capacity == 0U) return 0U;
	tail = g_ig3302_rx_tail;
	head = g_ig3302_rx_head;
	__DMB();
	while(tail != head && count < capacity)
	{
		buffer[count++] = g_ig3302_rx_ring[tail];
		tail = (uint16_t)((tail + 1U) & IG3302_RX_RING_MASK);
	}
	__DMB();
	g_ig3302_rx_tail = tail;
	return count;
}

void IG3302UartClearRx(void)
{
	uint16_t head = g_ig3302_rx_head;
	__DMB();
	g_ig3302_rx_tail = head;
}

HAL_StatusTypeDef IG3302UartEnsureRx(void)
{
	HAL_StatusTypeDef status;
	if(g_ig3302_dma_active == 0U) return HAL_ERROR;
	if(huart9.RxState == HAL_UART_STATE_BUSY_RX) return HAL_OK;
	status = HAL_UARTEx_ReceiveToIdle_DMA(&huart9, g_ig3302_dma_rx_buffer, IG3302_DMA_RX_SIZE);
	g_ig3302_uart_diag.last_rx_restart_status = (uint8_t)status;
	if(status != HAL_OK) g_ig3302_uart_diag.rx_restart_fail_count++;
	return status;
}

void IG3302UartInitRx(void)
{
	(void)HAL_UART_AbortReceive(&huart9);
	g_ig3302_rx_head = 0U;
	g_ig3302_rx_tail = 0U;
	g_ig3302_dma_active = 1U;
	(void)IG3302UartEnsureRx();
}
HAL_StatusTypeDef MBus2UartEnsureRx(void)
{
	HAL_StatusTypeDef status;
	if(huart2.RxState == HAL_UART_STATE_BUSY_RX) return HAL_OK;
	status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_mbus2_dma_rx_buffer, MBUS2_DMA_RX_SIZE);
	g_mbus2_uart_diag.last_rx_restart_status = (uint8_t)status;
	if(status != HAL_OK) g_mbus2_uart_diag.rx_restart_fail_count++;
	return status;
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
	g_mbus2_rx_head = 0U;
	g_mbus2_rx_tail = 0U;
	g_rs4853_rx_head = 0U;
	g_rs4853_rx_tail = 0U;
	g_rs4853_tx_complete = 0U;
	 g_ig3302_rx_head = 0U;
	 g_ig3302_rx_tail = 0U;
	 g_ig3302_dma_active = 0U;
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
		// 采用大彩科技提供的协议来对数据进行处理
		queue_push(screendata);
//		HAL_UART_Receive_IT(&huart8, &screendata, 1); // 开启下一次接收
		while(HAL_UART_Receive_IT(&huart8,&screendata,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart8);
		}
  }
  else if (huart->Instance == USART3) {
		/* FECbus RX: 字节入环 + 重启下一字节接收 */
		FecbusRx_OnByte(g_fecbus_rx_byte);
		while(HAL_UART_Receive_IT(&huart3,(uint8_t *)&g_fecbus_rx_byte,1)!= HAL_OK)
		{
				__HAL_UNLOCK(&huart3);
		}
  }
  else if (huart->Instance == USART1) {
		/* FECbus RX (第二路径): 字节入环 + 重启下一字节接收 (与USART3各自独立) */
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
////		// 维护Cache一致性
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
		//		uartbuff[7].recepetion_flag = 1; //		uartbuff[7].recepetion_len = Size; //		HAL_UARTEx_ReceiveToIdle_IT(&huart8, uartbuff[7].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == USART2)
  {
		/* 独立对齐DMA缓冲写入环形队列，业务任务不再与DMA共享同一帧缓存。 */
    SCB_InvalidateDCache_by_Addr((uint32_t*)g_mbus2_dma_rx_buffer, MBUS2_DMA_RX_SIZE);
		g_mbus2_uart_diag.rx_event_count++;
		g_mbus2_uart_diag.last_rx_size = Size;
		if(Size < 5U) g_mbus2_uart_diag.rx_short_event_count++;
		if(Size > MBUS2_DMA_RX_SIZE) Size = MBUS2_DMA_RX_SIZE;
		MBus2UartPushFromIsr(g_mbus2_dma_rx_buffer, Size);
		(void)MBus2UartEnsureRx();
	}
	else if (huart->Instance == USART3)
  {
		//		uartbuff[7].recepetion_flag = 1; //		uartbuff[7].recepetion_len = Size; //		HAL_UARTEx_ReceiveToIdle_IT(&huart8, uartbuff[7].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART4)
  {
		uartbuff[3].recepetion_flag = 1;
		uartbuff[3].recepetion_len = Size;

		// 维护Cache一致性
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

		// 维护Cache一致性
    SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[5].recepetion_buff, BUFF_MAX);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartbuff[5].recepetion_buff, BUFF_MAX);
	}
	else if (huart->Instance == UART7)
	{
		// 维护Cache一致性
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
		if(g_ig3302_dma_active != 0U)
		{
			SCB_InvalidateDCache_by_Addr((uint32_t*)g_ig3302_dma_rx_buffer, IG3302_DMA_RX_SIZE);
			g_ig3302_uart_diag.rx_event_count++;
			g_ig3302_uart_diag.last_rx_size = Size;
			if(Size < 5U) g_ig3302_uart_diag.rx_short_event_count++;
			if(Size > IG3302_DMA_RX_SIZE) Size = IG3302_DMA_RX_SIZE;
			IG3302UartPushFromIsr(g_ig3302_dma_rx_buffer, Size);
			(void)IG3302UartEnsureRx();
		}
		else
		{
			uartbuff[8].recepetion_flag = 1;
			uartbuff[8].recepetion_len = Size;
			SCB_InvalidateDCache_by_Addr((uint32_t*)uartbuff[8].recepetion_buff, BUFF_MAX);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
		}
	}
	else if (huart->Instance == USART10)
	{
		uartbuff[9].recepetion_flag = 1;
		uartbuff[9].recepetion_len = Size;
		
		// 维护Cache一致性
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

    /* 1. 帧错误处理 (FE) */
    if(huart->ErrorCode & HAL_UART_ERROR_FE) {
        __HAL_UART_CLEAR_FEFLAG(huart);  // 必须清除标志 帧错误
//        volatile uint8_t temp = huart->Instance->DR; // 读取DR寄存器清空错误数据
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 2. 溢出错误处理 (ORE) */
    if(huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart);  // 溢出错误
        // 对于DMA模式需要特殊处理 (USART3=FECbus IT接收, 排除避免误启DMA)
        if((huart->hdmarx != NULL) && (huart->Instance != USART3) && (huart->Instance != USART2) && (huart->Instance != UART5)) {
            HAL_UART_DMAStop(huart);
						//huart->hdmarx->Instance->CNDTR = BUFF_MAX;
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    }

    /* 3. 噪声错误处理 (NE) */
    if(huart->ErrorCode & HAL_UART_ERROR_NE) {
        __HAL_UART_CLEAR_NEFLAG(huart);  // 噪声错误
      //        volatile uint8_t temp = huart->Instance->DR; // 读取DR寄存器清空错误数据
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }

    /* 4. 奇偶校验错误处理 (PE) */
    if(huart->ErrorCode & HAL_UART_ERROR_PE) {
        __HAL_UART_CLEAR_PEFLAG(huart);  // 奇偶检验错误
      //        volatile uint8_t temp = huart->Instance->DR; // 读取DR寄存器清空错误数据
			volatile uint8_t temp_rda = huart->Instance->RDR;
			volatile uint8_t temp_tda = huart->Instance->TDR;
    }
		
		if(huart->Instance == UART8)
		{
			HAL_UART_Receive_IT(&huart8, &screendata, 1);
		}
		else if (huart->Instance == USART1)
		{
			/* FECbus 第二路径: USART1 重新挂起单字节 IT 接收 */
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_fecbus_rx_byte_u1, 1);
		}
		else if (huart->Instance == USART2)
		{
			(void)MBus2UartEnsureRx();
		}
		else if (huart->Instance == USART3)
		{
			/* FECbus RX: 错误恢复, 重启逐字节IT接收 (bsp_fecbus_rx.c) */
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
			if(g_ig3302_dma_active != 0U) (void)IG3302UartEnsureRx();
			else HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uartbuff[8].recepetion_buff, BUFF_MAX);
		}
		else if (huart->Instance == USART10)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uartbuff[9].recepetion_buff, BUFF_MAX);
		}
}


// CAN中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance == FDCAN1) 
	{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) 
		{
      FDCAN_RxHeaderTypeDef RxHeader;
			
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, fdcanbuff[FDCAN1SITE].recepetion_buff) == HAL_OK) 
			{
				/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
			fdcanbuff[FDCAN1SITE].identifier = RxHeader.Identifier;
				fdcanbuff[FDCAN1SITE].id_type = RxHeader.IdType;
				fdcanbuff[FDCAN1SITE].frame_type = RxHeader.RxFrameType;
				fdcanbuff[FDCAN1SITE].recepetion_len = (uint8_t)RxHeader.DataLength;  /* HAL已返回解析后的DLC值 */
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

