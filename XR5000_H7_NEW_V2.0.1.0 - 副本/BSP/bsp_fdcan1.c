#include "bsp_fdcan1.h"

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

#include "bsp_itcallback.h"

typedef enum
{
	Isolate_Output_1 = 0,
	Isolate_Output_2 = 1,
	Isolate_Output_3 = 2,
	Isolate_Output_4 = 3,
}eIsolateOutputStateIndex;

typedef enum
{
	Solenoid_Valve_1 = 0,
	Solenoid_Valve_2 = 1,
	Solenoid_Valve_3 = 2,
	Solenoid_Valve_4 = 3,
}eSolenoidValveStateIndex;

typedef enum
{
	Sounds_Lights_1 = 0,
	Sounds_Lights_2 = 1,
	Sounds_Lights_3 = 2,
	Sounds_Lights_4 = 3,
}eSoundsLightsStateIndex;

typedef enum
{
	FeedbackTrigger_1 = 0,
	FeedbackTrigger_2 = 1,
	FeedbackTrigger_3 = 2,
	FeedbackTrigger_4 = 3,
	FeedbackTrigger_5 = 4,
	FeedbackTrigger_6 = 5,

}eFeedbackTriggerStateIndex;

typedef enum
{
	Dry_Contact_1 = 0,
	Dry_Contact_2 = 1,
	Dry_Contact_3 = 2,
	Dry_Contact_4 = 3,

}eDryContactStateIndex;

typedef struct
{
	uint8_t isolate_output_state[4]; // 四路自带隔离的24V输出
	
	uint8_t solenoid_valve_state[4]; // 电磁阀状态
	
	uint8_t sounds_lights_state[4];  // 声光启动状态
	
	uint8_t feedbacktriggerstate[8]; // 反馈触发状态
	
	uint8_t dry_contact_state[4];    // 干接点状态
	
}IG3305AllStateRegister_t;

IG3305AllStateRegister_t ig3305_recevie_data = {0};

// 配置FDCAN1过滤器和启动
void FDCAN1_Start(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  
  // 配置标准ID过滤器：接收所有标准帧
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;      // ID = 0
  sFilterConfig.FilterID2 = 0x0000;      // 掩码 = 0，接收所有ID
  
	// 配置接收标准帧
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  
	// 配置扩展ID过滤器：接收所有扩展帧
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;  // ID = 0
  sFilterConfig.FilterID2 = 0x00000000;  // 掩码 = 0，接收所有扩展帧ID
	
	// 配置接收扩展帧
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
	
  // 启动FDCAN1
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  
  // 使能接收中断（可选）
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }

}

/**
  * @brief  发送标准CAN数据帧
  * @param  id: 标准CAN ID (11位)
  * @param  data: 数据指针
  * @param  length: 数据长度 (0-8)
  * @retval HAL status
  */
HAL_StatusTypeDef FDCAN1_SendStdDataFrame(uint32_t id, uint8_t* data, uint8_t length)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  
  // 检查数据长度
  if (length > 8) {
    return HAL_ERROR;
  }
	
	// 确保ID是11位标准ID
  id = id & 0x7FF;
  
  // 配置发送头
  TxHeader.Identifier = id;                    // CAN ID
  TxHeader.IdType = FDCAN_STANDARD_ID;         // 标准帧
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;     // 数据帧
  TxHeader.DataLength = length;          // 数据长度 (DLC编码)
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      // 禁用比特率切换
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       // 经典CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;                  // 消息标记
  
  // 将消息添加到TX FIFO并发送
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}

/**
  * @brief  发送扩展CAN数据帧
  * @param  id: 扩展CAN ID (29位)
  * @param  data: 数据指针
  * @param  length: 数据长度 (0-8)
  * @retval HAL status
  */
HAL_StatusTypeDef FDCAN1_SendExtDataFrame(uint32_t id, uint8_t* data, uint8_t length)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  
  if (length > 8) {
    return HAL_ERROR;
  }
  
	// 确保ID是29位扩展ID
  id = id & 0x1FFFFFFF;
	
  TxHeader.Identifier = id;                    // CAN ID
  TxHeader.IdType = FDCAN_EXTENDED_ID;         // 扩展帧
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;     // 数据帧
  TxHeader.DataLength = length;          // 数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      // 禁用比特率切换
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       // 经典CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}

/**
  * @brief  发送远程帧
  * @param  id: CAN ID
  * @param  isExtended: 是否为扩展帧
  * @retval HAL status
  */
HAL_StatusTypeDef FDCAN1_SendRemoteFrame(uint32_t id, uint8_t isExtended)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  
  TxHeader.Identifier = id;
  TxHeader.IdType = isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;   // 远程帧
  TxHeader.DataLength = 0;               // 远程帧数据长度为0
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, NULL);
}

// 查询所有寄存器 除反馈 状态
uint8_t FdcanQuerySlaveAllRegister[8] = {IG3306_QUERY_ALL_STATE_CMD, IG3306_SLAVE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 查询反馈寄存器状态
uint8_t FdcanQueryFeedbakRegister[8] ={IG3306_QUERY_FEEDBCAK_CMD, IG3306_SLAVE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t *poll_buff[] = {
	FdcanQuerySlaveAllRegister,
	FdcanQueryFeedbakRegister,
};

void setIG3306AllRegisterState(uint8_t *receive_buff, IG3305AllStateRegister_t *ig3305_register_tab)
{
	uint8_t *temp_p = receive_buff;
	
	// 整理所有隔离输出的状态
	for(uint8_t i = 0, j = 0; i < 8; i+=2 ,j++)
	{
		if ( ((temp_p[2] >> i) & 0x03) == 0x02)
		{
			ig3305_register_tab->isolate_output_state[j] = IsolateOutputShort;
		}
		else
		{
			ig3305_register_tab->isolate_output_state[j] = IsolateOutputNomal;
		}
	}
	
	// 整理所有电磁阀的状态
	for(uint8_t i = 0, j = 0; i < 8; i+=2 ,j++)
	{
		switch(((temp_p[3] >> i) & 0x03))
		{
			case 0x00: {
				ig3305_register_tab->solenoid_valve_state[j] = SolenoidValveNomal;
				break;
			}
			case 0x02: {
				ig3305_register_tab->solenoid_valve_state[j] = SolenoidValveShort;
				break;
			}
			case 0x01: {
				ig3305_register_tab->solenoid_valve_state[j] = SolenoidValveOpens;
				break;
			}
			case 0x03: {
				ig3305_register_tab->solenoid_valve_state[j] = SolenoidValveRunDc;
				break;
			}
		}
	}
	
	// 整理所有声光的状态
	for(uint8_t i = 0, j = 0; i < 8; i+=2 ,j++)
	{
		switch(((temp_p[4] >> i) & 0x03))
		{
			case 0x00: {
				ig3305_register_tab->sounds_lights_state[j] = SoundsLightsNomal;
				break;
			}
			case 0x02: {
				ig3305_register_tab->sounds_lights_state[j] = SoundsLightsShort;
				break;
			}
			case 0x01: {
				ig3305_register_tab->sounds_lights_state[j] = SoundsLightsOpens;
				break;
			}
			case 0x03: {
				ig3305_register_tab->sounds_lights_state[j] = SoundsLightsRunDc;
				break;
			}
		}
	}
	
	// 整理所有干接点的状态
	for(uint8_t i = 0; i < 8; i++)
	{
		switch(((temp_p[5] >> i) & 0x01))
		{
			case 0x00: {
				ig3305_register_tab->dry_contact_state[i] = DryContactNomal;
				break;
			}
			case 0x02: {
				ig3305_register_tab->dry_contact_state[i] = DryContactRunDc;
				break;
			}
		}
	}
}

void setIG3306FeedbackRegisterState(uint8_t *receive_buff, IG3305AllStateRegister_t *ig3305_register_tab)
{
	uint8_t *temp_p = receive_buff;
	
	// 整理所有反馈的状态
	for(uint8_t i = 2, j = 0; i < 8; i++, j++)
	{
		switch(temp_p[i])
		{
			case 0x00: {
				ig3305_register_tab->feedbacktriggerstate[j] = FeedbackTriggerNomal;
				break;
			}
			case 0x01: {
				ig3305_register_tab->feedbacktriggerstate[j] = FeedbackTriggerShort;
				break;
			}
			case 0x02: {
				ig3305_register_tab->feedbacktriggerstate[j] = FeedbackTriggerOpens;
				break;
			}
			case 0x03: {
				ig3305_register_tab->feedbacktriggerstate[j] = FeedbackTriggerRunDc;
				break;
			}
		}
	}

}

#define FDCAN_MASTER_ADDR 0x10000100

uint8_t fdcan1_poll_index = 0;
void Fdcan1PollManager(void)
{
	FDCAN1_SendExtDataFrame(FDCAN_MASTER_ADDR, poll_buff[fdcan1_poll_index], 8);
	
	fdcan1_poll_index++;
	fdcan1_poll_index &= 0x01;
}

extern void DebugSendString(uint8_t *buf, uint8_t len);

void Fdcan1ReceiveDataDeal(void)
{
	if(fdcanbuff[FDCAN1SITE].recepetion_flag == 1)
	{
		uint8_t temp_cmd = fdcanbuff[FDCAN1SITE].recepetion_buff[0];
		if(temp_cmd == IG3306_QUERY_FEEDBCAK_CMD)
		{
			setIG3306FeedbackRegisterState(fdcanbuff[FDCAN1SITE].recepetion_buff, &ig3305_recevie_data);
		}
		else
		{
			setIG3306AllRegisterState(fdcanbuff[FDCAN1SITE].recepetion_buff, &ig3305_recevie_data);
		}

//		DebugSendString(fdcanbuff[FDCAN1SITE].recepetion_buff, fdcanbuff[FDCAN1SITE].recepetion_len);
		
		fdcanbuff[FDCAN1SITE].recepetion_flag = 0;
	}
}

// *********錹錷？******
void Fdcan1SendAndReceiveTask(void *parameter)
{
	uint8_t fdcan_poll_delay_count = 0;    // 延时计数 每此执行该任务+1 
	for(;;)
	{
		fdcan_poll_delay_count++;
		if(fdcan_poll_delay_count == 10)
		{
			fdcan_poll_delay_count = 0;
			Fdcan1PollManager();
		}
		
		Fdcan1ReceiveDataDeal();
		
		osDelay(50); // 
	}
}

uint8_t getIsolateOutputState(uint8_t query_register_id)
{
	return ig3305_recevie_data.isolate_output_state[query_register_id];
}

uint8_t getSolenoidValveState(uint8_t query_register_id)
{
	return ig3305_recevie_data.solenoid_valve_state[query_register_id];
}

uint8_t getSoundsLightsState(uint8_t query_register_id)
{
	return ig3305_recevie_data.sounds_lights_state[query_register_id];
}

uint8_t getfeedbackState(uint8_t query_register_id)
{
	return ig3305_recevie_data.feedbacktriggerstate[query_register_id];
}

uint8_t getDryContactState(uint8_t query_register_id)
{
	return ig3305_recevie_data.dry_contact_state[query_register_id];
}


