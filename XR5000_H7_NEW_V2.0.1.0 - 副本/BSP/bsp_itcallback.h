#ifndef __BSP_ITCALLBACK_H
#define __BSP_ITCALLBACK_H

#include "main.h"

#define BUFF_MAX 400

#define ID5306_EXTEND_FRAME_ID 0x10000001

typedef struct
{
	uint8_t recepetion_flag;
	uint16_t recepetion_len;
	uint8_t recepetion_buff[BUFF_MAX];
}UartBuffer_t;

typedef struct
{
	uint32_t rx_event_count;
	uint32_t rx_short_event_count;
	uint32_t rx_restart_fail_count;
	uint32_t rx_crc_error_count;
	uint32_t tx_fail_count;
	uint32_t frame_error_count;
	uint32_t noise_error_count;
	uint32_t overrun_error_count;
	uint32_t parity_error_count;
	uint16_t last_rx_size;
	uint8_t last_rx_restart_status;
	uint8_t last_tx_status;
} MBus2UartDiag_t;


typedef enum
{
	STATION_OPTICALFIBER = 0,
	MBUS2SITE = 1,
	EMSSITE   = 2,
	DEBUGSITE = 4,
	MBUS1SITE = 6,
	SCREENSITE = 7,
	PACKSITE = 8,
	INSCREENSITE = 9,
	
	ERRORSITE = 255,
}eUartOrder;

typedef struct
{
	uint8_t recepetion_flag;
	uint8_t recepetion_len;
	uint32_t identifier;
	uint32_t id_type;
	uint32_t frame_type;
	uint8_t recepetion_buff[8]; // 对于标准CAN来说 一次传输最多八字节数据
}FdcanBuffer_t;

typedef enum
{
	FDCAN1SITE = 0,
	FDCAN2SITE = 1,
	
}eFdcanOrder;

extern UartBuffer_t uartbuff[10];
extern volatile MBus2UartDiag_t g_mbus2_uart_diag;
extern uint8_t screendata;

HAL_StatusTypeDef MBus2UartEnsureRx(void);

extern FdcanBuffer_t fdcanbuff[2];

#endif
