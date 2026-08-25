#include "bsp_ctrl_bus.h"

#include "system.h"

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

// 新增变量 风机变量
uint8_t fan_addr = 52;

uint8_t fan1_open[]  = {FAN_ADDR, 5, 0, 0, 0xFF, 0, 0x89, 0x9F};
uint8_t fan1_close[] = {FAN_ADDR, 5, 0, 0, 0x00, 0, 0xC8, 0x6F};

uint8_t fan2_open[]  = {FAN_ADDR, 5, 0, 1, 0xFF, 0, 0xD8, 0x5F};
uint8_t fan2_close[] = {FAN_ADDR, 5, 0, 1, 0x00, 0, 0x99, 0xAF};

uint8_t fan_reset[]  = {FAN_ADDR, 5, 0, 0x14, 0xFF, 0, 0xC9, 0x9B};


uint8_t fan_state1 = fan_stop;
uint8_t fan_state2 = fan_stop;
uint8_t fan_mode   = fan_auto; // 0 自动 1 手动

uint8_t fan_disconnect_count = 0;

/* XR5000_UART5_EXCLUSIVE_FIX_20260730: fan transport is isolated and disabled until reassigned. */
static void FanQueueCommand(uint8_t *buf, uint8_t len)
{
#if FAN_COMMUNICATION_ENABLED
	(void)SendDataToFanBusQueue(buf, len);
#else
	(void)buf;
	(void)len;
#endif
}

__weak void FanBusTransportSend(uint8_t *buf, uint8_t len)
{
	/* Override this adapter after assigning the fan to a non-UART5 transport. */
	(void)buf;
	(void)len;
}

__weak uint8_t FanBusTransportReceive(uint8_t *buf, uint16_t *len)
{
	/* Override this adapter together with FanBusTransportSend(). */
	(void)buf;
	if (len != NULL)
	{
		*len = 0;
	}
	return 0;
}
// end



uint8_t getCurrentFanMode(void)
{
	uint8_t temp_fan_state = 0;
	// 如果有任意风机掉线 损坏都返回2
	if(fan_state1 == fan_break || fan_state2 == fan_break || fan_state1 == fan_disconnect || fan_state2 == fan_disconnect)
	{
		temp_fan_state = 2;
	}
	if(fan_mode == fan_auto)
	{
		temp_fan_state = 0;
	}
	else
	{
		temp_fan_state = 1;
	}

	return temp_fan_state;
}

uint8_t getCurrentFanRunState(void)
{
	uint8_t temp_fan_state = 0;
	
	if(fan_state1 == fan_run || fan_state2 == fan_run)
	{
		temp_fan_state = 1;
	}
	else if(fan_state1 == fan_stop || fan_state2 == fan_stop)
	{
		temp_fan_state = 0;
	}
	return temp_fan_state;
}

// 风机1 开启 进风
void Fan1CtrlOpen(void)
{
//	PackBehaviorManageSendString(fan1_open, sizeof(fan1_open));
	FanQueueCommand(fan1_open, sizeof(fan1_open));
	
}

// 风机1 关闭
void Fan1CtrlClose(void)
{
//	PackBehaviorManageSendString(fan1_close, sizeof(fan1_close));
	FanQueueCommand(fan1_close, sizeof(fan1_close));
}

// 风机1 开启 排风
void Fan2CtrlOpen(void)
{
//	PackBehaviorManageSendString(fan2_open, sizeof(fan2_open));

	FanQueueCommand(fan2_open, sizeof(fan2_open));
}

// 风机1 关闭
void Fan2CtrlClose(void)
{
//	PackBehaviorManageSendString(fan2_close, sizeof(fan2_close));
	FanQueueCommand(fan2_close, sizeof(fan2_close));
}

// 风机 复位
void FanCtrlReset(void)
{
//	PackBehaviorManageSendString(fan_reset, sizeof(fan_reset));
	FanQueueCommand(fan_reset, sizeof(fan_reset));
}

void CtrlBusSendString(uint8_t *buf, uint8_t len)
{
	/* XR5000_UART5_EXCLUSIVE_FIX_20260730: compatibility wrapper, never uses UART5. */
	FanBusTransportSend(buf, len);
}

void PollFanManager(void)
{
#if FAN_COMMUNICATION_ENABLED
	uint8_t modbusbuf[8];
	uint16_t crc16 = 0;
	
	modbusbuf[0] = FAN_ADDR;
	modbusbuf[1] = 0x04;
	modbusbuf[2] = 0x00;
	modbusbuf[3] = 0x00;
	modbusbuf[4] = 0x00;
	modbusbuf[5] = 2; // 读取2个寄存器
	
	crc16 = CalcCrc16(modbusbuf, 6);
	modbusbuf[6] = crc16 & 0xFF;
	modbusbuf[7] = crc16 >> 8;
	
//	PackBehaviorManageSendString(modbusbuf, sizeof(modbusbuf));
	FanQueueCommand(modbusbuf, 8); // 发送到队列
	if(fan_disconnect_count < 5)
	{
		fan_disconnect_count++;
	}
	else
	{
		fan_state1 = fan_disconnect;
		fan_state2 = fan_disconnect;
	}
#else
	/* Keep the legacy fan state machine dormant until a new transport is assigned. */
#endif
}

void FansReceiveDataDeal(void)
{
#if FAN_COMMUNICATION_ENABLED
	uint8_t receive_buff[FAN_RX_BUFFER_SIZE] = {0};
	uint16_t receive_len = 0;
	uint16_t crc16;

	if (FanBusTransportReceive(receive_buff, &receive_len) != 1U || receive_len < 4U || receive_len > FAN_RX_BUFFER_SIZE)
	{
		return;
	}

	crc16 = (receive_buff[receive_len - 1U] << 8) | receive_buff[receive_len - 2U];
	if (CalcCrc16(receive_buff, receive_len - 2U) != crc16 || receive_buff[0] != FAN_ADDR)
	{
		return;
	}

	fan_disconnect_count = 0;
	if (receive_buff[1] == 0x04U && receive_len >= 7U)
	{
		fan_state1 = (receive_buff[4] == 0U) ? fan_stop : ((receive_buff[4] == 1U) ? fan_run : fan_break);
		fan_state2 = (receive_buff[6] == 0U) ? fan_stop : ((receive_buff[6] == 1U) ? fan_run : fan_break);
	}
#else
	/* Fan transport remains dormant until a non-UART5 adapter is supplied. */
#endif
}
void CtrlBusPollAndReceiveTask(void *parameter)
{
	uint8_t modbusbuf[8] = {0};      // 发送缓冲区

	uint8_t ctrl_bus_poll_delay_count = 0;    // 延时计数 每此执行该任务+1 
	
	for(;;)
	{
		ctrl_bus_poll_delay_count++;
		if(ctrl_bus_poll_delay_count == 10)
		{
			ctrl_bus_poll_delay_count = 0;
			PollFanManager();
		}
		
		if(ReceiveDataFromFanBusQueue(modbusbuf) == 1)
		{
			FanBusTransportSend(modbusbuf, sizeof(modbusbuf));
		}
		
		// 掉线判断处理
		FansReceiveDataDeal();
		
		osDelay(50); // 
	}
}



