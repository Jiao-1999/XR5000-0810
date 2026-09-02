/************************************版权申明********************************************
**                             广州大彩光电科技有限公司
**                             http://www.gz-dc.com
**-----------------------------------文件信息--------------------------------------------
** 文件名称:   hmi_user_uart.c
** 修改时间:   2011-05-18
** 文件说明:   用户MCU串口驱动函数库
** 技术支持：  Tel: 020-82186683  Email: hmi@gz-dc.com Web:www.gz-dc.com
--------------------------------------------------------------------------------------

--------------------------------------------------------------------------------------
                                  使用必读
   hmi_user_uart.c中的串口发送接收函数共3个函数：串口初始化Uartinti()、发送1个字节SendChar()、
   发送字符串SendStrings().若移植到其他平台，需要修改底层寄
   存器设置,但禁止修改函数名称，否则无法与HMI驱动库(hmi_driver.c)匹配。
--------------------------------------------------------------------------------------



----------------------------------------------------------------------------------------
                          1. 基于STM32平台串口驱动
----------------------------------------------------------------------------------------*/
#include "hmi_user_uart.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t g_hmi_tx_mutex = NULL;

void HmiTxInit(void)
{
	if(g_hmi_tx_mutex == NULL)
	{
		/* XR5000_HMI_UART_LOCK_FIX_20260729: serialize complete UART8 HMI frames across tasks. */
		 g_hmi_tx_mutex = xSemaphoreCreateRecursiveMutex();
	}
}

void HmiTxFrameBegin(void)
{
	if(g_hmi_tx_mutex != NULL)
	{
		(void)xSemaphoreTakeRecursive(g_hmi_tx_mutex, portMAX_DELAY);
	}
}

void HmiTxFrameEnd(void)
{
	if(g_hmi_tx_mutex != NULL)
	{
		(void)xSemaphoreGiveRecursive(g_hmi_tx_mutex);
	}
}

void HmiTxBatchBegin(void)
{
	HmiTxFrameBegin();
}

void HmiTxBatchEnd(void)
{
	HmiTxFrameEnd();
}
/****************************************************************************
* 名    称： UartInit()
* 功    能： 串口初始化
* 入口参数： 无
* 出口参数： 无
****************************************************************************/


void UartInit(uint32 BaudRate)
{
    
}


/*****************************************************************
* 名    称： SendChar()
* 功    能： 发送1个字节
* 入口参数： t  发送的字节
* 出口参数： 无
 *****************************************************************/
void  SendChar(uchar t)
{
	SendChar_2(t);
}

void  SendChar_2(uchar t)
{
	uint8_t tx4dbuf[1];
	tx4dbuf[0]=t;
	
	HAL_UART_Transmit(&huart8, (uint8_t *)tx4dbuf, 1, HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart5, (uint8_t *)tx4dbuf, 1, HAL_MAX_DELAY);
//    USART_SendData(USART1,t);
//    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//    while((USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET));//等待串口发送完毕
}


