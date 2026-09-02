#include "bsp_bms.h"
#include "bsp_itcallback.h"
#include "system.h"

#include "cmd_process.h"

#include "bsp_mbus.h"

#include "bsp_screen.h"

#include "bsp_key.h"

#include "bsp_adc.h"

#include "bsp_rs485_01.h"

#include "cmsis_os.h"

#include "bsp_ctrl_bus.h"

// 此文件为上行协议管理

uint8_t extract_buff[512]  = {0};   // 用来提取接受数组中的数据
uint8_t transmit_buff[512] = {0};   // 实际发送数据的数组

void EMS_SendString(uint8_t *buf, uint8_t len)
{
	HAL_UART_Transmit(&huart3, buf, len, 0xff); // 
}

void EMSDataRequestManage(void)
{
	if(uartbuff[EMSSITE].recepetion_flag == 1)
	{
		uartbuff[EMSSITE].recepetion_flag = 0;
		if(uartbuff[EMSSITE].recepetion_len < 2)
		{
			return;
		}
		if(uartbuff[EMSSITE].recepetion_buff[0] == SystemSaveInfo.slave_addr485_EMS) // 本机地址
		{	
			uint16_t crc16 = (uartbuff[EMSSITE].recepetion_buff[uartbuff[EMSSITE].recepetion_len-1]<<8)|uartbuff[EMSSITE].recepetion_buff[uartbuff[EMSSITE].recepetion_len-2];
			if(CalcCrc16(uartbuff[EMSSITE].recepetion_buff, uartbuff[EMSSITE].recepetion_len - 2) == crc16)
			{
				if(uartbuff[EMSSITE].recepetion_buff[1] == 4)
				{
					uint8_t  num;
					uint16_t i; // 遍历变量
					
					num = 37 - 21; // 37 - 21 = 16 40001
					i = 0;
					// 烟雾 仓级感烟探测器
					if(Cang_YWZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_YWZT_buf[num] == 1 || Cang_YWZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_YWZT_buf[num] == 8 || Cang_YWZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 40 - 21;
					i+=2;
					// 烟雾 仓级感烟探测器 40002
					if(Cang_YWZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_YWZT_buf[num] == 1 || Cang_YWZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_YWZT_buf[num] == 8 || Cang_YWZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 38 - 21;
					i+=2;
					// 温度 仓级感温探测器 40003
					if(Cang_WDZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_WDZT_buf[num] == 1 || Cang_WDZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_WDZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 39 - 21;
					i+=2;
					// 温度 仓级感温探测器  40004
					if(Cang_WDZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_WDZT_buf[num] == 1 || Cang_WDZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_WDZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 41 - 21; // 41 - 21 = 20 40005
					i+=2;
					// 烟雾 电器间 仓级感烟探测器
					if(Cang_YWZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_YWZT_buf[num] == 1 || Cang_YWZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_YWZT_buf[num] == 8 || Cang_YWZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 42 - 21;
					i+=2;
					// 温度 电器间 仓级感温探测器  40006
					if(Cang_WDZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_WDZT_buf[num] == 1 || Cang_WDZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 1; // 没有故障 有报警
					}
					else if(Cang_WDZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 33 - 21;
					i+=2;
					// CO  仓级CO探测器  40007
					if(Cang_zx_buf[num] == CabinDisconnectCount)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线
					}
					else if(Cang_COZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_COZT_buf[num] == 1)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 4; // 没有故障 有报警
					}
					else if(Cang_COZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 8; // 没有故障 有报警
					}
					else if(Cang_COZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 34 - 21;
					i+=2;
					// HH  仓级HH探测器  40008
					if(Cang_zx_buf[num] == CabinDisconnectCount)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线
					}
					else if(Cang_H2ZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_H2ZT_buf[num] == 1)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 4; // 没有故障 有报警
					}
					else if(Cang_H2ZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 8; // 没有故障 有报警
					}
					else if(Cang_H2ZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 35 - 21;
					i+=2;
					// HH  仓级HH探测器  40009
					if(Cang_zx_buf[num] == CabinDisconnectCount)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线
					}
					else if(Cang_H2ZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_H2ZT_buf[num] == 1)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 4; // 没有故障 有报警
					}
					else if(Cang_H2ZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 8; // 没有故障 有报警
					}
					else if(Cang_H2ZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					num = 36 - 21;
					i+=2;
					// CO  仓级CO探测器  40010
					if(Cang_zx_buf[num] == CabinDisconnectCount)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线
					}
					else if(Cang_COZT_buf[num] == 0)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 没有故障 没有报警
					}
					else if(Cang_COZT_buf[num] == 1)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 4; // 没有故障 有报警
					}
					else if(Cang_COZT_buf[num] == 2)
					{
						extract_buff[i] = 0; // 高位 
						extract_buff[i + 1] = 8; // 没有故障 有报警
					}
					else if(Cang_COZT_buf[num] == 9)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 有故障 没有报警
					}
					
					i+=2;
					// 外联设备在线状态 手报 40011
					if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyPress)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 按下
					}
					else if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyDisconnect || linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyShort)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线 故障
					}
					else if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}						
					
					i+=2;
					// 外联设备在线状态 急停 40012
					if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyPress)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 按下
					}
					else if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyDisconnect || linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyShort)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线 故障
					}
					else if(linkage_work_state[HandPaperLinkageScreenID] == FeedbackKeyOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}			
					
					i+=2;
					// 外联设备在线状态 声光 40013
					if(linkage_work_state[SoundLightLinkageScreenID] == LinkageWorking)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(linkage_work_state[SoundLightLinkageScreenID] == LinkageDisconnect || linkage_work_state[SoundLightLinkageScreenID] == LinkageShortCircuit)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else if(linkage_work_state[SoundLightLinkageScreenID] == LinkageOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}		
					
					i+=2;
					// 外联设备在线状态 放气勿入 40014
					if(linkage_work_state[DefauleLinkageScreenID] == LinkageWorking)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(linkage_work_state[DefauleLinkageScreenID] == LinkageDisconnect || linkage_work_state[DefauleLinkageScreenID] == LinkageShortCircuit)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else if(linkage_work_state[DefauleLinkageScreenID] == LinkageOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}		
					
					i+=2;
					// 外联设备在线状态 电气间电磁阀 40015 （目前电气间接到警笛上了）
					if(linkage_work_state[SirenLinkageScreenID] == LinkageWorking)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(linkage_work_state[SirenLinkageScreenID] == LinkageDisconnect || linkage_work_state[SirenLinkageScreenID] == LinkageShortCircuit)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else if(linkage_work_state[SirenLinkageScreenID] == LinkageOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}		
					
					i+=2;
					// 外联设备在线状态 电气间电磁阀 40016 （目前电池间接到仓上了）
					if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageWorking)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageDisconnect || linkage_work_state[CabinSprayLinkageScreenID] == LinkageShortCircuit)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else if(linkage_work_state[CabinSprayLinkageScreenID] == LinkageOnline)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}
					
					i+=2;
					// 消防主机状态 40017 
					if(zhu_state == 1 && bei_state == normal_charge)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(zhu_state != 1 || bei_state == open_circuit || bei_state == short_circuit)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}	
					
					i+=2;
					// 风机1状态 40018 进风
					if(fan_state1 == fan_run)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(fan_state1 == fan_disconnect || fan_state1 == fan_break)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}	
					
					i+=2;
					// 风机2状态 40019 排风
					if(fan_state2 == fan_run)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 2; // 继电器吸合 工作状态
					}
					else if(fan_state2 == fan_disconnect || fan_state2 == fan_break)
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 1; // 掉线或断路 故障
					}
					else
					{
						extract_buff[i] = 0;
						extract_buff[i + 1] = 0; // 在线 正常
					}	
					
					i+=(2 * 12); // 跳过保留的地址 40031  CO探测器浓度值
					num = 33 - 21;
					extract_buff[i] = Cang_COzhi_buf[num] >> 8;
					extract_buff[i + 1] = Cang_COzhi_buf[num]; // 在线 正常
					
					i+=2;        // 防爆H2探测器浓度值(5簇) 40032
					num = 34 - 21;
					extract_buff[i] = Cang_H2zhi_buf[num] >> 8;
					extract_buff[i + 1] = Cang_H2zhi_buf[num]; // 在线 正常
					
					i+=2;        // 防爆H2探测器浓度值(8簇) 40033
					num = 35 - 21;
					extract_buff[i] = Cang_H2zhi_buf[num] >> 8;
					extract_buff[i + 1] = Cang_H2zhi_buf[num]; // 在线 正常
					
					i+=2;        // 防爆CO探测器浓度值(11簇) 40034
					num = 36 - 21;
					extract_buff[i] = Cang_COzhi_buf[num] >> 8;
					extract_buff[i + 1] = Cang_COzhi_buf[num]; // 在线 正常
					
					i+=(2 * 106); // 跳过保留 40140 1簇复合探测器状态
					
					for(num = 1;num < 13;num++)
					{
						if(CU_zx_buf[num] == PackDisconnectCount)
						{
							extract_buff[i] = 0;
							extract_buff[i + 1] = 1; // 通信故障
						}
						else
						{
							for(uint8_t k = 1;k < cu_tcq_sxzt[num]; k++)
							{
								if(PACK_zx_buf[num][k] == 0)
								{
									extract_buff[i] = 0;
									extract_buff[i + 1] = 2; // 设备故障
								}
								else if(PACK_WDZT_buf[num][k] == 2)
								{
									extract_buff[i] = 0;
									extract_buff[i + 1] = 8; // 高限报警
								}
								else if(PACK_WDZT_buf[num][k] == 1 && (PACK_YWZT_buf[num][k] !=0 || PACK_COZT_buf[num][k] !=0 || PACK_CH4ZT_buf[num][k] !=0))
								{
									extract_buff[i] = 0;
									extract_buff[i + 1] = 8; // 高限报警
								}	
								else if(PACK_WDZT_buf[num][k] == 0 && (PACK_YWZT_buf[num][k] !=0 || PACK_COZT_buf[num][k] !=0 || PACK_CH4ZT_buf[num][k] !=0))
								{
									extract_buff[i] = 0;
									extract_buff[i + 1] = 4; // 低限报警
								}
								else
								{
									extract_buff[i] = 0;
									extract_buff[i + 1] = 0; // 正常
								}
								
							}
							
						}
						i+=2;
						
					}

				}
				else if(uartbuff[EMSSITE].recepetion_buff[1] == 6) // 06 功能码
				{
					int16_t index = uartbuff[EMSSITE].recepetion_buff[3] -160;
					
					if(index >= 0 && index <= 12)
					{
						if(uartbuff[EMSSITE].recepetion_buff[5] == 1)
						{
							BMS_Temp[index] = BMS_Temp[index] + 1;
						}
						else
						{
							BMS_Temp[index] = 0;
						}
					}
				}
				
				
			}
			
		}

	}
}

void BMSRecvDealTask(void *parameter)
{
	for(;;)
	{
		EMSDataRequestManage();
		
		osDelay(250);
	}
}




