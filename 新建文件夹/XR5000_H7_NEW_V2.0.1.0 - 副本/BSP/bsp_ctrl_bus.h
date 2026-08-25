#ifndef __BSP_CTRL_BUS_H
#define __BSP_CTRL_BUS_H

#include "main.h"

typedef enum
{
	fan_stop       = 0xA0,
	fan_run        = 0xA1,
	fan_break      = 0xA2,
	fan_disconnect = 0xA3,
	
	FAN_INIT
}FAN_STATE;

typedef enum
{
	fan_auto = 0xA4,
	fan_hand = 0xA5,
	
}FAN_MODE;

#define FAN_ADDR 52

/* XR5000_UART5_EXCLUSIVE_FIX_20260730: fan communication is disabled until another port is assigned. */
#define FAN_COMMUNICATION_ENABLED 0
#define FAN_RX_BUFFER_SIZE 64U

void FanBusTransportSend(uint8_t *buf, uint8_t len);
uint8_t FanBusTransportReceive(uint8_t *buf, uint16_t *len);

extern uint8_t fan_state1;
extern uint8_t fan_state2;
extern uint8_t fan_mode  ; // 0 自动 1 手动
extern uint8_t fan_disconnect_count;

void Fan1CtrlOpen(void);
void Fan1CtrlClose(void);
void Fan2CtrlOpen(void);
void Fan2CtrlClose(void);
void FanCtrlReset(void);

void PollFanManager(void);

extern int8_t SendDataToFanBusQueue(uint8_t *buf, uint8_t buf_len);
extern int8_t ReceiveDataFromFanBusQueue(uint8_t *buf);

void CtrlBusPollAndReceiveTask(void *parameter);

#endif
