#include "bsp_adc.h"
#include "bsp_debug.h"
#include "system.h"
#include "bsp_screen.h"

#include "bsp_key.h"

#include "bsp_relay.h"

#include "cmsis_os.h"

#include "cmd_process.h"

#include "bsp_save_ctrl.h"
#include "bsp_internal_board.h"
#include "bsp_storage_event.h" /* 黑匣子事件记录API: 关机事件(GB4717-2024 B.1.1.1d) */

#define TIMEOUT_NUM 60000

typedef enum
{
	OutFire1_Bit   = 0, // 第一位
	OutFire2_Bit   = 1,
	Deflate_Bit    = 2,
	SoundLight_Bit = 3,
	Siren_Bit      = 4,
	CabinSpray_Bit = 5,
	
}LinkageBeepCtrlBit;

extern uint8_t silencers_state;

uint16_t linkage_disconnect_state = 0;

LinkageStorage linkage_storage_buff[14] = {0};

__attribute__((section(".sram1"))) uint32_t ADC_DMA_BUFF[ADC_CARRY_NUM]; 
//uint32_t ADC_DMA_BUFF[ADC_CARRY_NUM] = {0}; 

uint16_t linkage_beep_ctrl = 0; // 联动设备蜂鸣器开关 默认关

uint8_t main_power_under_voltage_time_out = 0;
uint16_t open_circuit_judge_timeout = 0;
uint16_t exit_circuit_judge_timeout = 0;
uint16_t curr_back_up_power_timeout = 0;

// 如果完成容量计算 则置1 否则为0
uint8_t battary_capacity_read_flag = 0;

// 该变量用来计时 如果电池进入浮充状态 则会关闭继电器开始计时 计时到一定数后会重新打开继电器
uint32_t battary_charge_ctrl_counter = 0;

float adc2voltage = 0.8057; // ADC对应的电压值转换系数3.3/4096*1000 转换单位为mV

float voltage2pressure = 0.006667; // 电压转换压力系数 1/20/7.5 （7.5为采样电阻）

float outfire1_pressure = 0.0;
float outfire2_pressure = 0.0;

//float total_capacity = 7000000.0;           // 整体容量 7000mAh
float total_capacity = 24000000.0;           // 整体容量 24000000uAh

float remain_battery_capacity = 0;
//float voltage_capacity_coefficient = 21.2134; // 
float voltage_capacity_coefficient = 79.99f; // 

float capacity_percentage = 0;

float mAh_capacity_coefficient = 0.278; // 

uint16_t BattrayVoltage = 0; 

uint8_t beep_tweet_num = 4;

extern uint8_t creatNewFaultRecordToCache(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
extern uint8_t findRecoveryDevice(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
extern void deletRecoveryRecord(uint8_t recovery_index);

void LinkageDeviceStateInit(void)
{
	memset(linkage_storage_buff, FeedbackKeyOnline, sizeof(LinkageStorage)*14);
	
	memset(linkage_work_state, LinkageOnline, sizeof(linkage_work_state));
}

// 冒泡排序算法
void bubbleSort(uint32_t arr[], uint16_t n) {
	uint16_t swapped = 0;
	// 外层循环控制排序轮数
	for (uint16_t i = 0; i < n - 1; i++) {
		// 标志位，用于优化（当某轮无交换时提前终止）
		// 内层循环进行相邻元素比较
		// 每轮将最大的元素"冒泡"到最后
		for (int j = 0; j < n - 1 - i; j++) {
				if (arr[j] > arr[j + 1]) {
						// 交换相邻元素
						int temp = arr[j];
						arr[j] = arr[j + 1];
						arr[j + 1] = temp;
						swapped = 1;
				}
		}
		// 如果没有发生交换，说明数组已经有序
		if (!swapped) {
				break;
		}
	}
}

// 函数功能 从DMA缓冲区中取出对应通道的ADC值，并计算平滑滤波后的值
uint16_t CaculateAdcSmoothValue(uint32_t arr[], AdcChannelSite adc_channel)
{
	uint16_t i;
	uint32_t temp_buff[10] = {0};
	uint32_t adc_value = 0;
	
	SCB_InvalidateDCache_by_Addr((uint32_t *)ADC_DMA_BUFF, sizeof(ADC_DMA_BUFF));

	for(i = 0;i < 10; i++)
	{
		temp_buff[i] = arr[adc_channel + i*14];
	}

	bubbleSort(temp_buff, 10);
	
//	for(i = 0;i < 10; i++)
//	{
//		DebugPrintf("temp_buff2[%d] = %d\r\n", i, temp_buff[i]);
//	}
	
	for(i = 3;i < 7;i++)
	{
		adc_value = adc_value + temp_buff[i];
	}
	
	return adc_value/4;
}

// 灭火装置1电磁阀在线状态判断
void OutFire1SolenoidValveOnlineMonitor()
{
	uint16_t adc_value = 0;
	
	if(linkage_shield_state[OutFire1LinkageScreenID] == LinkageOpen)  // 如果设置为上线
	{	
		if(mhqdbiaozhi == 0 && FetchOutFire1RelayState() == JDQ_OFF) // 如果灭火未启动
		{
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, OutFire1AdcSite);
//			DebugPrintf("adc_value = %d\r\n", adc_value);
			if(adc_value > SolenoidValveDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[OutFire1LinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (SolenoidValveOnlineThreshold - 250) && adc_value < (SolenoidValveOnlineThreshold + 250))
			{
				linkage_work_state[OutFire1LinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < SolenoidValveShortCircuitThreshold)
			{
				linkage_work_state[OutFire1LinkageScreenID] = LinkageShortCircuit; // 短路
			}
			linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type = linkage_work_state[OutFire1LinkageScreenID];
		}
		else
		{
			linkage_work_state[OutFire1LinkageScreenID] = linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type; // 如果现在灭火装置启动了 那么显示固定为灭火装置启动前的状态
		}
		
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[OutFire1LinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type != linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type
		) 
		{
			// 存储到FLASH故障存储区中 灭火装置1掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, OutFir1_Package_ID);
			// 存储到缓冲区
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, OutFir1_Package_ID, DISCONNECT);
			
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type = linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type;
			// 掉线标志位置一
			my_set_x_bit(linkage_disconnect_state, OutFire1_Bit); // 把第1位置1 
			my_set_x_bit(linkage_beep_ctrl, OutFire1_Bit); // 把第0位置1
			silencers_state = 0; // 关闭消音灯
			// 打开分区1故障LED
			Part1FaultLedCtrl(LED_ON);
		}else if(linkage_work_state[OutFire1LinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type != linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type) 
		{	
			// 存储到FLASH故障存储区中 灭火装置1 短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, OutFir1_Package_ID);	
			// 存RAM			
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, OutFir1_Package_ID, SHORTCIRCUIT);
			
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type = linkage_storage_buff[OutFire1LinkageScreenID].curr_fault_type;
			// 掉线标志位置一
			my_set_x_bit(linkage_disconnect_state, OutFire1_Bit); // 把第1位置1 
			my_set_x_bit(linkage_beep_ctrl, OutFire1_Bit); // 把第0位置1
			silencers_state = 0; // 关闭消音灯
			// 打开分区1故障LED
			Part1FaultLedCtrl(LED_ON);
		}
		else	if(linkage_work_state[OutFire1LinkageScreenID] == LinkageOnline && // 如果设备恢复在线
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type != linkage_work_state[OutFire1LinkageScreenID])  
		{	
			if(linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 灭火装置1 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, OutFir1_Package_ID);
				
			}
			else if(linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 灭火装置1 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, OutFir1_Package_ID);
			}
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, OutFir1_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			// 关闭分区1故障LED
			Part1FaultLedCtrl(LED_OFF);
			// 清除上一次储存的故障信息 确保下一次启动后正常存储
			linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type = linkage_work_state[OutFire1LinkageScreenID]; 
			my_clear_x_bit(linkage_disconnect_state, OutFire1_Bit); // 清除掉线标志
			my_clear_x_bit(linkage_beep_ctrl, OutFire1_Bit); // 把第0位置0
		}
	}else { // 如果设置为禁用
		linkage_storage_buff[OutFire1LinkageScreenID].last_fault_type = 0; // 清除上一次储存的故障信息 确保下一次启动后正常存储	
		my_clear_x_bit(linkage_beep_ctrl, OutFire1_Bit); // 把第0位置0 设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, OutFire1_Bit); // 清除掉线标志
	}
}

// 灭火装置2电磁阀在线状态判断
void OutFire2SolenoidValveOnlineMonitor()
{
	uint16_t adc_value = 0;
	
	if(linkage_shield_state[OutFire2LinkageScreenID] == LinkageOpen) {
		if(mhqdbiaozhi == 0 && FetchOutFire2RelayState() == JDQ_OFF) {// 如果灭火未启动
		
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, OutFire2AdcSite);
	//	DebugPrintf("adc_value = %d\r\n", adc_value);
			if(adc_value > SolenoidValveDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[OutFire2LinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (SolenoidValveOnlineThreshold - 250) && adc_value < (SolenoidValveOnlineThreshold + 250))
			{
				linkage_work_state[OutFire2LinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < SolenoidValveShortCircuitThreshold)
			{
				linkage_work_state[OutFire2LinkageScreenID] = LinkageShortCircuit; // 短路
			}
			linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type = linkage_work_state[OutFire2LinkageScreenID];
		}else {
			linkage_work_state[OutFire2LinkageScreenID] = linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type; // 如果现在灭火装置启动了 那么显示固定为灭火装置启动前的状态
		}
		
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[OutFire2LinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type != linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type) 
		{		 
			// 存储到FLASH故障存储区中 灭火装置2 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, OutFir2_Package_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, OutFir2_Package_ID, DISCONNECT);

			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type = linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type;

			my_set_x_bit(linkage_disconnect_state, OutFire2_Bit); // 把第1位置1 
			my_set_x_bit(linkage_beep_ctrl, OutFire2_Bit); // 把第1位置1 
			silencers_state = 0;
		}
		else if(linkage_work_state[OutFire2LinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type != linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type) 
		{		
			// 存储到FLASH故障存储区中 灭火装置1 短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, OutFir2_Package_ID);
			// 存储到RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, OutFir2_Package_ID, SHORTCIRCUIT);

			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type = linkage_storage_buff[OutFire2LinkageScreenID].curr_fault_type;
			// 掉线标志位置一
			my_set_x_bit(linkage_disconnect_state, OutFire2_Bit); // 把第1位置1 
			// 蜂鸣器启动
			my_set_x_bit(linkage_beep_ctrl, OutFire2_Bit); // 把第0位置1 
			silencers_state = 0;	
		}
		else if(linkage_work_state[OutFire2LinkageScreenID] == LinkageOnline && // 如果设备恢复在线
			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type != LinkageOnline)  
		{	
			if(linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 灭火装置1 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, OutFir2_Package_ID);
			}
			else if(linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 灭火装置1 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, OutFir2_Package_ID);
			}
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, OutFir2_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			
			linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type = LinkageOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
			my_clear_x_bit(linkage_beep_ctrl, OutFire2_Bit); // 把第0位置0 设为禁用不在判断	
			my_clear_x_bit(linkage_disconnect_state, OutFire2_Bit); // 清除掉线标志
			
		}
	}else { // 如果设置为禁用
		linkage_storage_buff[OutFire2LinkageScreenID].last_fault_type = 0; // 清除上一次储存的故障信息 确保下一次启动后正常存储
		my_clear_x_bit(linkage_beep_ctrl, OutFire2_Bit); // 把第0位置0 设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, OutFire2_Bit); // 清除掉线标志
	}
}

// 放气在线状态判断
void DefauleOnlineMonitor()
{
	uint16_t adc_value = 0;
	
	if(linkage_shield_state[DefauleLinkageScreenID] == LinkageOpen) {
		if(mhqdbiaozhi == 0 && FetchDefauleRelayState() == JDQ_OFF) // 如果灭火未启动 且继电器未吸合
		{
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, DefauleAdcSite);
	//	DebugPrintf("adc_value = %d\r\n", adc_value);
			if(adc_value > DefauleDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[DefauleLinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (DefauleOnlineThreshold - 200) && adc_value < (DefauleOnlineThreshold + 300))
			{
				linkage_work_state[DefauleLinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < DefauleShortCircuitThreshold)
			{
				linkage_work_state[DefauleLinkageScreenID] = LinkageShortCircuit; // 短路
			}
			linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type = linkage_work_state[DefauleLinkageScreenID];
		}
		else
		{
			linkage_work_state[DefauleLinkageScreenID] = linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type;
		}

		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[DefauleLinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type != linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type) 
		{		 
			// 存储到FLASH故障存储区中 灭火装置1 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, Deflate_Package_ID);
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, Deflate_Package_ID, DISCONNECT);

			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type = linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, Deflate_Bit); // 把第0位置0 设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, Deflate_Bit); // 清除掉线标志
			silencers_state = 0;	
		}else if(linkage_work_state[DefauleLinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type != linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type) 
		{	
			// 存储到FLASH故障存储区中 灭火装置1 短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, Deflate_Package_ID);			
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, Deflate_Package_ID, SHORTCIRCUIT);
			
			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type = linkage_storage_buff[DefauleLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, Deflate_Bit); // 把第0位置0 设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, Deflate_Bit); // 清除掉线标志
			silencers_state = 0;	
		}
		else	if(linkage_work_state[DefauleLinkageScreenID] == LinkageOnline && // 如果设备恢复在线
			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type != LinkageOnline) 
		{ 
			
			if(linkage_storage_buff[DefauleLinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 放气勿入 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, Deflate_Package_ID);
			}
			else if(linkage_storage_buff[DefauleLinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 放气 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, Deflate_Package_ID);
			}
			
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, Deflate_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			
			linkage_storage_buff[DefauleLinkageScreenID].last_fault_type = LinkageOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
			
			my_clear_x_bit(linkage_beep_ctrl, Deflate_Bit); // 把第0位置0 设为禁用不在判断
			my_clear_x_bit(linkage_disconnect_state, Deflate_Bit); // 清除掉线标志
		}
		
	}else { // 如果设置为禁用
		linkage_storage_buff[DefauleLinkageScreenID].last_fault_type = 0; // 清除上一次储存的故障信息 确保下一次启动后正常存储
		my_clear_x_bit(linkage_beep_ctrl, Deflate_Bit); // 把第0位置0 设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, Deflate_Bit); // 清除掉线标志
	}
}

// 声光在线状态判断
void SoundLightOnlineMonitor()
{
	uint16_t adc_value = 0;
	
	if(linkage_shield_state[SoundLightLinkageScreenID] == LinkageOpen) {
		if(mhqdbiaozhi == 0 && FetchSoundLightRelayState() == JDQ_OFF) // 如果灭火未启动
		{
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, SoundLightAdcSite);
//			DebugPrintf("adc_value = %d\r\n", adc_value);
			if(adc_value > DefauleDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[SoundLightLinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (DefauleOnlineThreshold - 200) && adc_value < (DefauleOnlineThreshold + 300))
			{
				linkage_work_state[SoundLightLinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < DefauleShortCircuitThreshold)
			{
				linkage_work_state[SoundLightLinkageScreenID] = LinkageShortCircuit; // 短路
			}
			linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type = linkage_work_state[SoundLightLinkageScreenID];
		}
		else {
			linkage_work_state[SoundLightLinkageScreenID] = linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type;
			
//			DebugPrintf("work_state = %d\r\n", linkage_work_state[SoundLightLinkageScreenID]);
		}
		
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[SoundLightLinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type != linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type) 
		{		 
			// 存储到FLASH故障存储区中 声光 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SoundLt_Package_ID);
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SoundLt_Package_ID, DISCONNECT);

			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type = linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, SoundLight_Bit); //  设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, SoundLight_Bit); // 清除掉线标志
			silencers_state = 0;	
			// 启动报警器故障LED
			SysSirenFaultLedCtrl(LED_ON);
		}else if(linkage_work_state[SoundLightLinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type != linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type) 
		{		
			// 存储到FLASH故障存储区中 声光 短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, SoundLt_Package_ID);
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SoundLt_Package_ID, SHORTCIRCUIT);

			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type = linkage_storage_buff[SoundLightLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, SoundLight_Bit); //  设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, SoundLight_Bit); // 清除掉线标志
			silencers_state = 0;	
			// 启动报警器故障LED
			SysSirenFaultLedCtrl(LED_ON);
		}
		else	if(linkage_work_state[SoundLightLinkageScreenID] == LinkageOnline && // 如果设备恢复在线
			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type != LinkageOnline) 
		{ 
			
			if(linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 声光 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SoundLt_Package_ID);
			}
			else if(linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 声光 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, SoundLt_Package_ID);
			}
			// 关
			SysSirenFaultLedCtrl(LED_OFF);
			
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SoundLt_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			// 清除上一次储存的故障信息 确保下一次启动后正常存储
			linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type = LinkageOnline; 
			
			my_clear_x_bit(linkage_beep_ctrl, SoundLight_Bit); //  设为禁用不在判断
			my_clear_x_bit(linkage_disconnect_state, SoundLight_Bit); // 清除掉线标志
		}
		
	}else { // 如果设置为禁用
		linkage_storage_buff[SoundLightLinkageScreenID].last_fault_type = LinkageOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
		my_clear_x_bit(linkage_beep_ctrl, SoundLight_Bit); //  设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, SoundLight_Bit); // 清除掉线标志
	}
}

#define SirenToSolenoid 0 /* 用来控制警笛连接判断的宏 1 表示警笛接电磁阀 0 表示警笛接其他 */

// 警笛在线状态判断 // 现接电气间电磁阀
void SirenOnlineMonitor()
{
	uint16_t adc_value = 0;

#if(SirenToSolenoid == 0)	
	
	if(linkage_shield_state[SirenLinkageScreenID] == LinkageOpen) { // 如果设备启用
		
		if(mhqdbiaozhi == 0 && FetchSirenRelayState() == JDQ_OFF) // 如果灭火未启动
		{
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, SirenAdcSite);
			if(adc_value > DefauleDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[SirenLinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (DefauleOnlineThreshold - 200) && adc_value < (DefauleOnlineThreshold + 300))
			{
				linkage_work_state[SirenLinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < DefauleShortCircuitThreshold)
			{
				linkage_work_state[SirenLinkageScreenID] = LinkageShortCircuit; // 短路
			}
			linkage_storage_buff[SirenLinkageScreenID].curr_fault_type = linkage_work_state[SirenLinkageScreenID];
		}
		else
		{
			linkage_work_state[SirenLinkageScreenID] = linkage_storage_buff[SirenLinkageScreenID].curr_fault_type;
		}
		
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[SirenLinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type != linkage_storage_buff[SirenLinkageScreenID].curr_fault_type) 
		{		 
			// 存储到FLASH故障存储区中 警笛 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SirenBk_Package_ID);
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SirenBk_Package_ID, DISCONNECT);
			
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type = linkage_storage_buff[SirenLinkageScreenID].curr_fault_type;

			my_set_x_bit(linkage_beep_ctrl, Siren_Bit); //  设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, Siren_Bit); // 清除掉线标志
			silencers_state = 0;		
		}else if(linkage_work_state[SirenLinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type != linkage_storage_buff[SirenLinkageScreenID].curr_fault_type) 
		{		
			// 存储到FLASH故障存储区中 警笛 短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, SirenBk_Package_ID);
			// 存储到RAM中显示到屏幕
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SirenBk_Package_ID, SHORTCIRCUIT);
			
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type = linkage_storage_buff[SirenLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, Siren_Bit); //  设为禁用不在判断
			my_set_x_bit(linkage_disconnect_state, Siren_Bit); // 清除掉线标志
			silencers_state = 0;	
		}
		else	if(linkage_work_state[SirenLinkageScreenID] == LinkageOnline && 
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type != LinkageOnline) 
		{ // 如果设备恢复在线
			
			if(linkage_storage_buff[SirenLinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 警笛 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SirenBk_Package_ID);
			}
			else if(linkage_storage_buff[SirenLinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 警笛 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, SirenBk_Package_ID);
			}
			
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SirenBk_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			
			linkage_storage_buff[SirenLinkageScreenID].last_fault_type = LinkageOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
			
			my_clear_x_bit(linkage_beep_ctrl, Siren_Bit); //  设为禁用不在判断
			my_clear_x_bit(linkage_disconnect_state, Siren_Bit); // 清除掉线标志
		}
		
	}else { // 如果设置为禁用
		linkage_storage_buff[SirenLinkageScreenID].last_fault_type = 0; // 清除上一次储存的故障信息 确保下一次启动后正常存储

		my_clear_x_bit(linkage_beep_ctrl, Siren_Bit); //  设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, Siren_Bit); // 清除掉线标志
	}
	
#elif(SirenToSolenoid == 1)
	if(mhqdbiaozhi == 0) // 如果灭火未启动
	{
		adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, SirenAdcSite);
		if(adc_value > SolenoidValveDisconnectThreshold) // 如果大于掉线阈值
		{
			linkage_work_state[SirenLinkageScreenID] = LinkageDisconnect;
		}
		else if(adc_value > (SolenoidValveOnlineThreshold - 50) && adc_value < (SolenoidValveOnlineThreshold + 50))
		{
			linkage_work_state[SirenLinkageScreenID] = LinkageOnline; // 正常工作 在线状态
		}
		else if(adc_value < SolenoidValveShortCircuitThreshold)
		{
			linkage_work_state[SirenLinkageScreenID] = LinkageShortCircuit; // 短路
		}
		pre_state = linkage_work_state[CabinSprayLinkageScreenID];
	}
	else if(mhqdbiaozhi != 0)
	{
		linkage_work_state[CabinSprayLinkageScreenID] = pre_state;
	}
#endif	
}

// 仓喷在线状态判断 // 接电池间电磁阀
void  CabinSprayOnlineMonitor()
{
	uint16_t adc_value = 0;

	if(linkage_shield_state[CabinSprayLinkageScreenID] == LinkageOpen) { // 如果设备启用
		if(mhqdbiaozhi == 0 && FetchCabinSprayRelayState() == JDQ_OFF) // 如果灭火未启动
		{
			adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, CabinSprayAdcSite);
	//	DebugPrintf("adc_value = %d\r\n", adc_value);
			if(adc_value > SolenoidValveDisconnectThreshold) // 如果大于掉线阈值
			{
				linkage_work_state[CabinSprayLinkageScreenID] = LinkageDisconnect;
			}
			else if(adc_value > (SolenoidValveOnlineThreshold - 200) && adc_value < (SolenoidValveOnlineThreshold + 300))
			{
				linkage_work_state[CabinSprayLinkageScreenID] = LinkageOnline; // 正常工作 在线状态
			}
			else if(adc_value < SolenoidValveShortCircuitThreshold)
			{
				linkage_work_state[CabinSprayLinkageScreenID] = LinkageShortCircuit; // 短路
			}
			
			linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type = linkage_work_state[CabinSprayLinkageScreenID];
		}
		else
		{
			linkage_work_state[CabinSprayLinkageScreenID] = linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type;
		}
		
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageDisconnect && 
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type != linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type) 
		{		 
			// 存储到FLASH故障存储区中 仓喷 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, CabinBK_Package_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, CabinBK_Package_ID, DISCONNECT);
			
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type = linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, CabinSpray_Bit); //  
			my_set_x_bit(linkage_disconnect_state, CabinSpray_Bit); // 
			silencers_state = 0;		
		}
		else if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageShortCircuit && 
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type != linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type) 
		{		
			// 存储到FLASH故障存储区中 仓喷 短路恢复
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, CabinBK_Package_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, CabinBK_Package_ID, SHORTCIRCUIT);
			
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type = linkage_storage_buff[CabinSprayLinkageScreenID].curr_fault_type;
			
			my_set_x_bit(linkage_beep_ctrl, CabinSpray_Bit); //  
			my_set_x_bit(linkage_disconnect_state, CabinSpray_Bit); // 
			silencers_state = 0;	
		}
		else	if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageOnline &&  // 如果设备恢复在线
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type != LinkageOnline) 
		{
			
			if(linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type == LinkageDisconnect)
			{
				// 存储到FLASH故障存储区中 仓喷 掉线恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, CabinBK_Package_ID);
			}
			else if(linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type == LinkageShortCircuit)
			{
				// 存储到FLASH故障存储区中 仓喷 短路恢复
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, CabinBK_Package_ID);
			}
			
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, CabinBK_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			
			linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type = LinkageOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
			my_clear_x_bit(linkage_beep_ctrl, CabinSpray_Bit); //  设为禁用不在判断
			my_clear_x_bit(linkage_disconnect_state, CabinSpray_Bit); // 清除掉线标志
		}
		
	}else { // 如果设置为禁用
		linkage_storage_buff[CabinSprayLinkageScreenID].last_fault_type = 0; // 清除上一次储存的故障信息 确保下一次启动后正常存储
		my_clear_x_bit(linkage_beep_ctrl, CabinSpray_Bit); //  设为禁用不在判断
		my_clear_x_bit(linkage_disconnect_state, CabinSpray_Bit); // 清除掉线标志
	}
	
}

void PressureSensorMonitor()
{
	uint16_t adc_value = 0;

	adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, Pressure1AdcSite); // 先获取压传一的ADC值
//	DebugPrintf("adc_value1 = %d\r\n", adc_value);
	
	outfire1_pressure = adc_value * adc2voltage * voltage2pressure; // 压力传感器电流值（单位：毫安）
	
	adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, Pressure2AdcSite); // 先获取压传一的ADC值
//	DebugPrintf("adc_value2 = %d\r\n", adc_value);
	
	outfire2_pressure = adc_value * adc2voltage * voltage2pressure; // 压力传感器电流值（单位：毫安）

}

typedef enum
{
	BACKUP_ONLINE_STATE = 1, // 进入60秒计时
	BACKUP_DOUBT_STATE  = 2, // 关闭充电继电器 等待几秒
	BACKUP_JUDGE_STATE  = 3, // 判断电池电压还在不在
	BACKUP_EXIST_STATE  = 4, // 
	BACKUP_RECOVER_STATE = 5, // 开启继电器
	BACKUP_OFFLINE_STATE = 6, // 电池不存在的状态
	
	BACKUP_FLOAT_CLOSE_STATE = 7, // 备电进入浮充 关闭充电状态
	BACKUP_ENERGY_LOW_STATE = 8, // 备电电量低于设定值 将开启再充电状态
	
	BACKUP_INIT_STATE = 0xFF,
}BackupStateJudgeState;

BackupStateJudgeState backup_judge_state = BACKUP_INIT_STATE;
uint8_t over_flow_flag = 0;
uint16_t time_over_flow = 0;

uint8_t turn_off_flag = 0;

void MainAndStandbyPowerJudge(void)
{
	uint16_t voltage_adc_value = 0;
	uint16_t curr_adc_value = 0;

//	uint8_t test_buff[32];
//	
//	voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, PowerAdcSite); // 获取主电电压ADC值
//	sprintf((char *)test_buff,"vol_val = %d\r\n", voltage_adc_value); // 打印电池ADC原始值
//	DebugSendString(test_buff, strlen((char *)test_buff));
//	
//	voltage_adc_value = (voltage_adc_value * adc2voltage); // 将主电电压ADC值转为电压值
//	sprintf((char *)test_buff,"vol_val = %d\r\n", voltage_adc_value); // 打印电池ADC原始值
//	DebugSendString(test_buff, strlen((char *)test_buff));
	
	voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, PowerAdcSite); // 获取主电电压ADC值
	voltage_adc_value = (voltage_adc_value * adc2voltage); // 将主电电压ADC值转为电压值

	if(voltage_adc_value < 2000 && zhu_state == 1 && over_flow_flag == 0) // 如果主电小于18V 且之前主电是正常状态
	{
		if(voltage_adc_value < 1800)
		{
			main_power_under_voltage_time_out = 2;
		}
		else if(main_power_under_voltage_time_out < 2)
		{
			main_power_under_voltage_time_out++; // 超时判断，如果超过2次就准备切换备电
		}
		
		if(main_power_under_voltage_time_out == 2)
		{
			float current_value = 0;
			curr_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, PwrOutCurrAdcSite) * 1.0f; // 获取输出电流
			// a = curr_adc_value * 1.0f * adc2voltage (mV)
			// b = a/20/5 (A)20是放大倍数 5是采样电阻毫欧
			// c = b * 1000 (mA)
			current_value = curr_adc_value * 1.0f * adc2voltage*10.0f;  // 采样电阻是5毫欧 放大倍数是20倍 计算出来是毫安
			
			if(current_value > 5000.0f) // 如果电流很大 过流保护
			{
				zhu_state = 1; // 过流认为主电还在只是过流保护了
				over_flow_flag = 1; // 过流标志位
				time_over_flow = baojingjishi; // 记录时间
			}
			else
			{
				zhu_state = 0; // 主电掉电
				BatteryBoostRelayCtrl(JDQ_ON); // 打开升压电路继电器
				if(battary_capacity_read_flag == 0) // 如果没有计算过电量
				{
					voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BattryAdcSite); // 获取备电电池电压ADC值
					voltage_adc_value = (voltage_adc_value * adc2voltage); // 将备电电池电压ADC值转为电压值
					// 计算电量
					remain_battery_capacity = (voltage_adc_value - 1050 )*voltage_capacity_coefficient*1000.0f;
					capacity_percentage = remain_battery_capacity*100.0f/total_capacity;

					battary_capacity_read_flag = 1;
				}
			}
		}
	}
	else if(voltage_adc_value > 2000 && zhu_state == 0) // 如果主电恢复 且现在是电池供电 
	{
		zhu_state = 1; // 主电恢复
//		BatteryChargeRelayCtrl(JDQ_ON); // 打开充电继电器
		BatteryBoostRelayCtrl(JDQ_OFF); // 关掉升压电路继电器
		main_power_under_voltage_time_out = 0;
	}
	else
	{
		main_power_under_voltage_time_out = 0;
	}
	
	voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BattryAdcSite); // 获取备电电池电压ADC值
	voltage_adc_value = (voltage_adc_value * adc2voltage); // 将备电电池电压ADC值转为电压值

	if(zhu_state == 0) // 如果是备电工作 如果备电不存在则不工作 若备电存在则不用判断
	{
		bei_state = discharge; // 放电
		BattrayVoltage = voltage_adc_value; // 给电池电压赋值
		if(voltage_adc_value < 1050) // 如果电池电压小于 10.5V 欠压
		{
			if(turn_off_flag == 0) // 避免存储一堆关机记录
			{
				BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_TURN_OFF, LINKAGE_CLUSTER_ID, SYS_TURN_OFF_Package_ID);
				StorageEvent_LogPowerOff(); /* 黑匣子:关机事件(EVT_POWER_OFF=121), 主电已掉且备电耗尽 */
				turn_off_flag = 1;
			}
			BatteryBoostRelayCtrl(JDQ_OFF); // 断开升压电路继电器 避免电池过放
		}
	}
	else // 如果主电还在线 才判断电池状态
	{
		if(over_flow_flag == 1) // 因为过流状态认为主电还在 所以会进入此
		{
			// 十秒内连续判断
			if(baojingjishi - time_over_flow < 5)
			{
				uint16_t temp_main_power_val;
				temp_main_power_val = CaculateAdcSmoothValue(ADC_DMA_BUFF, PowerAdcSite); // 获取主电电压ADC值
				temp_main_power_val = (temp_main_power_val * adc2voltage); // 将主电电压ADC值转为电压值
				if(temp_main_power_val < 2000)
				{
					time_over_flow = baojingjishi;
				}
			}
			else
			{
				over_flow_flag = 0;
			}
		}
		
		if(voltage_adc_value > 1030 && voltage_adc_value < 2000) // 如果电池电压值读到了 就开启充电继电器
		{
			if(exit_circuit_judge_timeout < 10)
			{
				exit_circuit_judge_timeout++;
			}
			else
			{
				BattrayVoltage = voltage_adc_value;
				if(FetchBatteryChargeRelayState() == JDQ_OFF && battary_capacity_read_flag == 0) // 如果继电器还没有吸合 并且没有计算过剩余电量
				{
					battary_capacity_read_flag = 1;
					osDelay(1000); // 延迟一秒后 重新获取一次电压，以获得较准确的数据
					voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BattryAdcSite); // 获取备电电池电压ADC值
					voltage_adc_value = (voltage_adc_value * adc2voltage); // 将备电电池电压ADC值转为电压值
					remain_battery_capacity = (voltage_adc_value - 1050 ) * voltage_capacity_coefficient * 1000.0f;
					capacity_percentage = remain_battery_capacity * 100.0f/total_capacity;

					if(capacity_percentage > total_capacity)
					{
						capacity_percentage = 23750000;
					}
					BatteryChargeRelayCtrl(JDQ_ON); // 打开充电继电器
				}

				float voltage_current = 0.0;
				curr_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BatCurrentAdcSite); // 获取备电电池电流ADC值
				// a = curr_adc_value * adc2voltage (mV)
				// b = a / 20 (mV)
				// c = b / 6.6667 （1000/150 = 6.66667）
				voltage_current = curr_adc_value * adc2voltage / 3.0f; // 计算出电流
				
				switch(backup_judge_state)
				{
					case BACKUP_ONLINE_STATE : {
						break;
					}
					case BACKUP_DOUBT_STATE : {
						if(voltage_current > 8.1f) // 如果电流恢复正常状态了
						{
							backup_judge_state = BACKUP_INIT_STATE;
						}
						else
						{
							// 25秒 都是在电流有问题的区间内
							if(baojingjishi - open_circuit_judge_timeout > 25)
							{
								BatteryChargeRelayCtrl(JDQ_OFF); // 充电继电器 关
								open_circuit_judge_timeout = baojingjishi; // 记录一下跳转时间戳
								backup_judge_state = BACKUP_JUDGE_STATE; // 跳转至审判状态
							}
						}
						
						break;
					}
					case BACKUP_JUDGE_STATE : {
						// 等待10秒 等待数据更新后在读取
						if(baojingjishi - open_circuit_judge_timeout >= 10)
						{
							// 读取一次备电电压
							voltage_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BattryAdcSite); // 获取备电电池电压ADC值
							voltage_adc_value = (voltage_adc_value * adc2voltage); // 将备电电池电压ADC值转为电压值
							if(voltage_adc_value < 800) // 如果确实不存在
							{
								bei_state = open_circuit; // 断路
								backup_judge_state = BACKUP_OFFLINE_STATE; // 跳转到电池不在的状态
							}
							else if(voltage_adc_value > 1050) // 电池还在
							{
								bei_state = float_charge; // 浮充 //
								// 不再次打开充电继电器
//								BatteryChargeRelayCtrl(JDQ_ON); // 充电继电器 开
								
								// 2025/11/28 10:56 跳转至新状态 此状态不再充电 而是等到电池电压低到一定程度后才开始充电
//								backup_judge_state = BACKUP_INIT_STATE; // 打开继电器后 开启下一轮判断
								
								backup_judge_state = BACKUP_FLOAT_CLOSE_STATE; // 打开继电器后 开启下一轮判断
							}
						}
						break;
					}
					case BACKUP_EXIST_STATE : {
						break;
					}
					case BACKUP_RECOVER_STATE : {
						break;
					}
					case BACKUP_OFFLINE_STATE : { // 电池断路状态
						backup_judge_state = BACKUP_INIT_STATE;
						break;
					}
					case BACKUP_INIT_STATE : {
						if(voltage_current < 5.1f) // 如果电流小于5.1
						{
							// 先认为是在线
							bei_state = normal_charge; // 正常充电状态
							
							open_circuit_judge_timeout = baojingjishi; // 记录一下跳转时间戳
							backup_judge_state = BACKUP_DOUBT_STATE; // 跳转至怀疑状态
						}
						break;
					}
					case BACKUP_FLOAT_CLOSE_STATE : {
						if(voltage_adc_value < 1240 && voltage_adc_value > 1050) // 加上下限判断 避免电池掉线还判断为电池电压低
						{
							open_circuit_judge_timeout = baojingjishi; // 获取一下时间戳
							backup_judge_state = BACKUP_ENERGY_LOW_STATE;
						}
						break;
					}
					case BACKUP_ENERGY_LOW_STATE : {
						
						if(baojingjishi - open_circuit_judge_timeout > 3) // 等待ADC搬运三次数据后再次 判断
						{
							if(voltage_adc_value < 1240 && voltage_adc_value > 800) // 加上下限判断 避免电池掉线还判断为电池电压低
							{
								BatteryChargeRelayCtrl(JDQ_ON); // 充电继电器 开
								backup_judge_state = BACKUP_INIT_STATE;
							}
							else
							{
								backup_judge_state = BACKUP_INIT_STATE; // 认为 备电断路了
							}
						}
						
						break;
					}
					default:{
						break;
					}
				}
	
			} // 判断十次外部电压确实存在
		}
		else if(voltage_adc_value < 100)
		{
			curr_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BatCurrentAdcSite); // 获取备电电池电压ADC值
			curr_adc_value = curr_adc_value * adc2voltage/20*6.6667f;
			if(curr_adc_value > 150)
			{
				bei_state = short_circuit; // 短路
			}
			else
			{
				bei_state = open_circuit; // 断路
			}
		}
	}
}

void PowerCapacityCaculate(void)
{
	uint16_t curr_adc_value = 0;
	float current_value = 0;

//	uint8_t test_buff[16];
//	sprintf((char *)test_buff,"rbc = %.2f\r\n", remain_battery_capacity);
//	DebugSendString(test_buff, strlen((char *)test_buff));
	
	if(zhu_state == 1) // 如果现在主电在
	{
		curr_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, BatCurrentAdcSite); // 获取备电电池电流ADC值
		// a = curr_adc_value * 1.0f * adc2voltage (mV)
		// b = a / 20 / 150 (A) 20是放大倍数 150是充电采样电阻 毫欧
		// c = b * 1000 (mA)
		current_value = curr_adc_value * 1.0f * adc2voltage / 3.0f; // 此结果计算出来单位是毫安
		if(bei_state == normal_charge) // 正常充电状态
		{
			if(current_value < 88.8f && remain_battery_capacity < 19000000) { // 如果电流小于88.8毫安 
				remain_battery_capacity += 222 + current_value * mAh_capacity_coefficient; // 剩余容量加上充入的库伦量 + 800mA的补充值
			}
			else {
				remain_battery_capacity += current_value * mAh_capacity_coefficient; // 剩余容量加上充入的库伦量
			}
			if(remain_battery_capacity > total_capacity)
			{
				remain_battery_capacity = total_capacity;
			}
		}
		
//		sprintf((char *)test_buff,"c_d_i = %.4f\r\n", current_value);
//		DebugSendString(test_buff, strlen((char *)test_buff));
		
		capacity_percentage = remain_battery_capacity * 100 / total_capacity;
		
	}
	else // 主电掉电
	{
		curr_adc_value = CaculateAdcSmoothValue(ADC_DMA_BUFF, PwrOutCurrAdcSite); // 获取备电电池电流ADC值
		// a = curr_adc_value * 1.0f * adc2voltage (mV)
		// b = a/20/5 (A)20是放大倍数 5是采样电阻毫欧
		// c = b * 1000 (mA)
		current_value = curr_adc_value * 1.0f * adc2voltage*10.0f;  // 采样电阻是5毫欧 放大倍数是20倍 计算出来是毫安
		if(bei_state == discharge)
		{
			remain_battery_capacity = remain_battery_capacity - current_value * mAh_capacity_coefficient; // 剩余容量加上充入的库伦量
			
			if(remain_battery_capacity < 0)
			{
				remain_battery_capacity = 0;
			}
			else if(remain_battery_capacity > total_capacity)
			{
				remain_battery_capacity = total_capacity;
			}
		}
//		sprintf((char *)test_buff,"f_d_i = %.4f\r\n", current_value);
//		DebugSendString(test_buff, strlen((char *)test_buff));
		
		capacity_percentage = remain_battery_capacity * 100 / total_capacity;
	}
	
	
}

void LinkageOnlineJudgeTask(void* parameter)
{
	
	for(;;)
	{
		OutFire1SolenoidValveOnlineMonitor();
		OutFire2SolenoidValveOnlineMonitor();
		DefauleOnlineMonitor();
		SoundLightOnlineMonitor();
		SirenOnlineMonitor();
		CabinSprayOnlineMonitor();
		PressureSensorMonitor();

		osDelay(1000);
	}
}

void PowerOnlineJudgeTask(void* parameter)
{
	uint16_t timeout = 0;
	for(;;)
	{
		timeout++;
		MainAndStandbyPowerJudge();
		if(timeout == 1000)
		{
			PowerCapacityCaculate();
			timeout = 0;
		}
		
		
		osDelay(1);
	}
}





