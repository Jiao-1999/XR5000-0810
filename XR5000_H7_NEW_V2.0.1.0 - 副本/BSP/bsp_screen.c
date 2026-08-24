#include "bsp_screen.h"
#include "usart.h"
#include "bsp_itcallback.h"

#include "bsp_debug.h"
#include "cmd_process.h"
#include "cmd_queue.h"
#include "cmsis_os.h"

#include "system.h"

#include "bsp_key.h"


#include "bsp_rs485_01.h"
#include "bsp_adc.h"

#include "bsp_super.h"

#include "bsp_ctrl_bus.h"
#include "bsp_storage_event.h" /* 黑匣子事件记录API: 时钟调整事件(GB4717-2024 B.1.1.1d) */

uint8_t size;
uint8_t LinkageTextCtrlID[40] = {26,27,28,29,30,31,32,33,34,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,0};
uint8_t LinkageButtCtrlID[40] = {13,14,15,16,17,18,19,20,21,47,48,49,50,51,52,53,54,55,74,75,76,77,78,79,80,81,82,0};


extern float capacity_percentage; // 电池电量显示
extern uint16_t BattrayVoltage;

// 内屏发送函数 RX8 TX8
void InternalScreenSendString(uint8_t* buf, uint8_t len)
{
	HAL_UART_Transmit(&huart8,buf,len,0xff); // 
}

void InternalScreenRecvDeal(void* parameter)
{
	
	size = queue_find_cmd(cmd_buffer,CMD_MAX_SIZE);                              //从缓冲区中获取一条指令         

	if(size>0&&cmd_buffer[1]!=0x07)                                              //接收到指令 ，及判断是否为开机提示
	{
		//DebugSendString(cmd_buffer, size);
		ProcessMessage((PCTRL_MSG)cmd_buffer, size);                             //指令处理  
	}                                                                           
	else if(size>0&&cmd_buffer[1]==0x07)                                         //如果为指令0x07就软重置STM32  
	{                                                                                                                    
//			NVIC_SystemReset();                                                                                                                                          
	} 
	
//	if(uartbuff[SCREENSITE].recepetion_flag == 1)
//	{
//		uartbuff[SCREENSITE].recepetion_flag = 0;
//		if(uartbuff[SCREENSITE].recepetion_buff[1]!= 0x07)
//		{
//			DebugSendString(uartbuff[SCREENSITE].recepetion_buff, uartbuff[SCREENSITE].recepetion_len);
//			ProcessMessage((PCTRL_MSG)uartbuff[SCREENSITE].recepetion_buff, uartbuff[SCREENSITE].recepetion_len);                             //指令处理  
//		}
//		else
//		{
//			NVIC_SystemReset(); 
//		}
//	}
}

void InternalScreenRecvDealTask(void* paremeter)
{
	for(;;)
	{
		InternalScreenRecvDeal(paremeter);
		osDelay(20);
	}
}

void SwitchCurrentScreenId(uint16_t target_screen)
{
	SetScreen(target_screen);
	osDelay(5);
	GetScreen();
//	osDelay(5);
//	if(*fact_screen != target_screen)
//	{
//		SwitchCurrentScreenId(target_screen, fact_screen);
//	}
}



// 入口参数: _screen_id  当前屏幕ID号
// 					_control_id 控制id号
//					_state      按键状态
// 函数功能: 联动监控开关处理
void InternalLinkageMonitorButtonDeal(uint16_t _screen_id, uint8_t _control_id, uint8_t _state)
{
	if(_screen_id == 39) // 串口屏页面ID号
	{
		// 使用for循环遍历所有按钮ID
		for(uint8_t i = 0; i < LinkageUseNum; i++)
		{
			if(_control_id == LinkageButtCtrlID[i])
			{
				if(_state == 0)
				{
					// 将对应联动设备设置为禁用（屏蔽）状态
					linkage_shield_state[i] = LinkageShield;
				}
				else
				{
					// 将对应联动设备设置为启用（打开）状态
					linkage_shield_state[i] = LinkageOpen;
				}
				SaveLinkageSheildState(); // 保存外联设备的屏蔽状态
				break; // 找到匹配的按钮后退出循环
			}
		}
	}
}

void InternalScreenLinkageMonitorUpdataUI(uint16_t _screen_id)
{
	uint8_t i;
	uint8_t state;
	uint8_t tempbuff[32];
	if(_screen_id == 39)
	{
		for(i = 0; i < LinkageUseNum; i++)
		{
			if(linkage_shield_state[i] == LinkageOpen) // 如果设置为启用
			{
				state = linkage_work_state[i];
				if(state == LinkageOnline)
				{
					sprintf((char*)tempbuff,"在线");
					SetTextValue(_screen_id, LinkageTextCtrlID[i], tempbuff);
					setkey_Value(_screen_id, LinkageButtCtrlID[i], 1);
				}
				else if(state == LinkageFault)
				{
					sprintf((char*)tempbuff,"故障");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == LinkageDisconnect)
				{
					sprintf((char*)tempbuff,"掉线");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == LinkageOpen)
				{
					sprintf((char*)tempbuff,"启用");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == LinkageShortCircuit)
				{
					sprintf((char*)tempbuff,"短路");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				
				if(state == FeedbackKeyOnline)
				{
					sprintf((char*)tempbuff,"在线");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == FeedbackKeyPress)
				{
					sprintf((char*)tempbuff,"按下");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == FeedbackKeyShort)
				{
					sprintf((char*)tempbuff,"短路");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}
				else if(state == FeedbackKeyDisconnect)
				{
					sprintf((char*)tempbuff,"掉线");
					SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
					setkey_Value(_screen_id,LinkageButtCtrlID[i],1);
				}	
			}
			else
			{
				sprintf((char*)tempbuff,"禁用");
				SetTextValue(_screen_id,LinkageTextCtrlID[i],tempbuff);
				setkey_Value(_screen_id,LinkageButtCtrlID[i],0);
			}
		}
	}
}

void InternalScreenRTCSetting(uint16_t screen_id, uint16_t control_id, uint8_t *str)
{
	if(screen_id == 41) // 主界面
	{
		int32_t temp;
		BM8563_Soft_I2C_GetTime(&SystemTime); // 读一次时间 存放到全局结构体中
		if(control_id == 16)      // year
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.year = (uint16_t)(temp%100);
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", temp);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
		else if(control_id == 17)      // 月
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.month  = (uint8_t)temp;
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", SystemTime.month);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
		else if(control_id == 18)      // 日
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.day    = (uint8_t)temp;		
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", SystemTime.day);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
		else if(control_id == 19)      // 时
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.hours  = (uint8_t)temp;
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", SystemTime.hours);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
		else if(control_id == 20)      // 分
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.minutes= (uint8_t)temp;
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", SystemTime.minutes);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
		else if(control_id == 21)      // 秒
		{
			uint8_t len = 0;
			len = sscanf((const char*)str,"%d", &temp); 
			SystemTime.seconds= (uint8_t)temp;
			if(len == 1)
			{
				uint8_t temp_buff[8];
				sprintf((char *)temp_buff, "%d", SystemTime.seconds);
				SetTextValue(screen_id, control_id, temp_buff);

				set_RTC(SystemTime.year,SystemTime.month,SystemTime.day,SystemTime.hours,SystemTime.minutes,SystemTime.seconds);
				BM8563_Soft_I2C_SetTime(&SystemTime);
				StorageEvent_LogClockAdjust(); /* 黑匣子:时钟调整事件(EVT_CLOCK_ADJUST=131), 时间戳=调整后新值 */
			}
		}
	}
}

void SetInternalScreenRTCTime(void)
{
	BM8563_TimeTypeDef temp_time;
	BM8563_Soft_I2C_GetTime(&temp_time);
	
	years   = temp_time.year;
	months  = temp_time.month;
	days    = temp_time.day;
	hours   = temp_time.hours;
	minutes = temp_time.minutes;
	secs    = temp_time.seconds;
	
	set_RTC(years, temp_time.month, temp_time.day, temp_time.hours, temp_time.minutes, temp_time.seconds);
}

uint8_t last_fan_state1 = FAN_INIT;
uint8_t last_fan_state2 = FAN_INIT;
void FansStateUpdataUI(uint16_t current_screen_id, uint8_t fan_disconnect_count, uint8_t fan_state1, uint8_t fan_state2, uint8_t fan_mode)
{
	if(current_screen_id == 1)
	{
		// 如果入口参数不等于上一次状态
		if(fan_state1 != last_fan_state1)
		{
			last_fan_state1 = fan_state1;
			
			switch(fan_state1)
			{
				case fan_disconnect:
					SetTextValue(current_screen_id,18,"掉线");
					break;
				case fan_stop:
					SetTextValue(current_screen_id,18,"停止");
					break;
				case fan_break:
					SetTextValue(current_screen_id,18,"损坏");
					break;
				case fan_run:
					SetTextValue(current_screen_id,18,"运行");
					break;
				case fan_auto:
					SetTextValue(current_screen_id,18,"自动");
					break;
				case fan_hand:
					SetTextValue(current_screen_id,18,"手动");
					break;
			}
		}

		if(fan_state2 != last_fan_state2)
		{
			last_fan_state2 = fan_state2;
			switch(fan_state2)
			{
				case fan_disconnect:
					SetTextValue(current_screen_id,19,"掉线");
					break;
				case fan_stop:
					SetTextValue(current_screen_id,19,"停止");
					break;
				case fan_break:
					SetTextValue(current_screen_id,19,"损坏");
					break;
				case fan_run:
					SetTextValue(current_screen_id,19,"运行");
					break;
				case fan_auto:
					SetTextValue(current_screen_id,19,"自动");
					break;
				case fan_hand:
					SetTextValue(current_screen_id,19,"手动");
					break;
			}
		}
		
	}
}

typedef enum
{
	NORMAL = 0,
	FAULTS = 1,
	DEINIT = 0xFF,
}TEMP_FRESH_STATE;

uint8_t last_main_power_state = DEINIT;

uint8_t last_bkup_power_state = DEINIT;

void PowerStateUpdataUI(uint16_t curr_screen_id,uint8_t zhu_state_, uint8_t bei_state_)
{
	if(curr_screen_id == 1)
	{
		// 主备电状态显示修改 新增了电池状态判断 包括 充电 放电 短路 断路
		if(last_main_power_state != zhu_state_)
		{
			last_main_power_state = zhu_state_;
			if(zhu_state_ == 1)
			{
				SetTextValue(curr_screen_id,9,(uint8_t *)"主电供电");
			}
			else
			{
				SetTextValue(curr_screen_id,9,(uint8_t *)"主电异常");
			}
		}
		
		if(last_bkup_power_state != bei_state_)
		{
			last_bkup_power_state = bei_state_;
			if(bei_state_ == discharge)
			{
				SetTextValue(curr_screen_id, 33, (uint8_t *)"备电放电");
			}
			else if(bei_state_ == normal_charge)
			{
				SetTextValue(curr_screen_id, 33, (uint8_t *)"备电充电");
			}
			else if(bei_state_ == open_circuit)
			{
				SetTextValue(curr_screen_id, 33, (uint8_t *)"备电断路");
				clearTextValue(curr_screen_id, 16);
				clearTextValue(curr_screen_id, 34);
			}
			else if(bei_state_ == short_circuit)
			{
				SetTextValue(curr_screen_id, 33, (uint8_t *)"备电短路");
				clearTextValue(curr_screen_id, 16);
				clearTextValue(curr_screen_id, 34);
			}
			else if(bei_state_ == undervoltage)
			{
				SetTextValue(curr_screen_id, 33, (uint8_t *)"备电欠压");
			}
		}

		if(bei_state_ != short_circuit && bei_state_ != open_circuit)
		{
			uint8_t test_buff[20];
			sprintf((char *)test_buff,"%.0f%%", capacity_percentage);
			SetTextValue(curr_screen_id, 34, test_buff);
			
			sprintf((char *)test_buff,"%.1fV", BattrayVoltage/100.0);
			SetTextValue(curr_screen_id, 16, test_buff);
		}
	}
	
}

uint8_t curr_press_device_state1 = NORMAL;
uint8_t curr_press_device_state2 = NORMAL;
uint8_t last_press_device_state1 = DEINIT;
uint8_t last_press_device_state2 = DEINIT;
void OutfirePressureUpdataUI(uint16_t curr_id, float pressure1, float pressure2, ThresholdSeting_t ts) // 更新灭火装置状态
{
	if(curr_id == 1)
	{
		float pressure = 0;
		
		if(pressure1 > 4)
		{
			pressure = (pressure1-4.0f)/3.2f; // 计算压力值
			if(pressure > ts.device1_pressure_lowlimit && pressure < ts.device1_pressure_uplimit)
			{
				curr_press_device_state1 = NORMAL;
				
			}
			else 
			{
				curr_press_device_state1 = FAULTS;
			}
		}
		else
		{
			curr_press_device_state1 = FAULTS;
		}
		
		if(pressure2 > 4)
		{
			pressure = (pressure2-4.0f)/3.2f; // 计算压力值.
			if(pressure > ts.device1_pressure_lowlimit && pressure < ts.device1_pressure_uplimit)
			{
				curr_press_device_state2 = NORMAL;
			}
			else 
			{
				curr_press_device_state2 = FAULTS;
			}

		}
		else
		{
			curr_press_device_state2 = FAULTS;
		}
		
		if(last_press_device_state1 != curr_press_device_state1)
		{
			last_press_device_state1 = curr_press_device_state1;
			if(curr_press_device_state1 == NORMAL)
			{
				SetTextValue(curr_id, 21, "正常");//刷新报警内容
			}
			else
			{
				SetTextValue(curr_id, 21, "故障");//刷新报警内容
			}
		}
		
		if(last_press_device_state2 != curr_press_device_state2)
		{
			last_press_device_state2 = curr_press_device_state2;
			if(curr_press_device_state2 == NORMAL)
			{
				SetTextValue(curr_id, 23, "正常");//刷新报警内容
			}
			else
			{
				SetTextValue(curr_id, 23, "故障");//刷新报警内容
			}
		}
		
		
	}
	
}





