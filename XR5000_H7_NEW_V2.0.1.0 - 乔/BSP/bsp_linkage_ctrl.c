#include "bsp_linkage_ctrl.h"

#include "cmd_process.h"
#include "cmsis_os.h"
#include "bsp_relay.h"

#include "bsp_rs485_01.h"

#include "bsp_logic_set.h"

#include "queue.h"

#include "bsp_ctrl_bus.h"

extern uint8_t UART_zhongduan, zhongduanz_YS, cang_reset;

extern uint8_t IG5305_DX,IG3302_DX,IG1102_DX,XR803_3301ID,XR803_RST,IG3301ID,IG3301_PCAK;

extern QueueHandle_t xMyRs485QueueHandle;

uint32_t fire_device_ctrl_timeout = 0;
uint8_t  fire_device_time_start_flag = 0;
uint8_t  spray_ctrl_state = StartDelay; // 默认喷放状态是启动延时等待

// 莫名其妙的变量
uint8_t IG3301_QD=1,IG3301_TZ=1,XR805_QD=1,XR805_TZ=1;

void LinkageRelayCtrl(void)
{
	zhongduanz_YS = zhongduanz_YS + 1;
	zhongduanz_YS = zhongduanz_YS & 0x01;
	if(zhongduanz_YS == 1)
	{
		if(UART_zhongduan == 1) // 有传感器报预警
		{
			// 打开风机
			Fan1CtrlOpen();
			Fan2CtrlOpen();
			UART_zhongduan = 68;
		}
		else if(UART_zhongduan == 2)
		{
			// 关闭风机
			Fan1CtrlClose();
			Fan2CtrlClose();
			UART_zhongduan=69;
		}
		else if(UART_zhongduan == 3)
		{
			// 开启灭火装置
			OutFire1RelayCtrl(JDQ_ON);
			OutFire2RelayCtrl(JDQ_ON);
			UART_zhongduan = 52;
		}
		else if(UART_zhongduan == 4)
		{
			SoundLightRelayCtrl(JDQ_ON); // 打开声光继电器
			UART_zhongduan = 20;
		}
		else if(UART_zhongduan == 5)
		{
			SoundLightRelayCtrl(JDQ_OFF); // 关闭声光继电器
			UART_zhongduan = 21;
		}
		else if(UART_zhongduan == 6)
		{
			DefauleRelayCtrl(JDQ_ON); // 打开放气继电器
			UART_zhongduan = 7;
		}
		else if(UART_zhongduan == 7)
		{
//			DefauleRelayCtrl(JDQ_ON); // 打开放气继电器
			
			UART_zhongduan = 16;
		}
		else if(UART_zhongduan == 8) 
		{
			// 先关闭风机 
			Fan1CtrlClose();
			Fan2CtrlClose();
			UART_zhongduan = 70;
		}
		else if(UART_zhongduan == 10) 
		{
			// XR805远程复位
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 11) 
		{
			// 兼容老系统（切换手动LED）
			UART_zhongduan = 13;
		}
		else if(UART_zhongduan == 12) 
		{
			// 兼容老系统（切换自动LED）
			UART_zhongduan = 14;
		}
		else if(UART_zhongduan == 13) 
		{
			// 兼容老系统（切换手动LED）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 14) 
		{
			// 兼容老系统（切换自动LED）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 15) // 有火警报警
		{
			// NULL 为了兼容老系统 此处什么都不干 跳转到7
			FireAlarmRelayCtrl(JDQ_ON);
			UART_zhongduan = 8;
		}
		else if(UART_zhongduan == 16) 
		{
			// 兼容老系统（关闭延时LED）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 17) 
		{
			// 关闭灭火装置
			OutFire1RelayCtrl(JDQ_OFF);
			OutFire2RelayCtrl(JDQ_OFF);
			UART_zhongduan = 23;
		}
		else if(UART_zhongduan == 18) 
		{
			// 关闭放气指示灯
			DefauleRelayCtrl(JDQ_OFF); // 打开放气继电器
			UART_zhongduan = 19;
		}
		else if(UART_zhongduan == 19) 
		{
			// 兼容老系统（关闭喷洒LED）
			UART_zhongduan = 25;
		}
		else if(UART_zhongduan == 20)
		{
			ForeWarmRelayCtrl(JDQ_ON); // 打开预警继电器
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 21)
		{
//			ForeWarmRelayCtrl(JDQ_OFF); // 关闭预警继电器
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 22)
		{
			FireAlarmRelayCtrl(JDQ_ON); // 打开火警继电器
			UART_zhongduan = 66;
		}
		else if(UART_zhongduan == 23)
		{
			FireAlarmRelayCtrl(JDQ_ON); // 关闭火警继电器
			UART_zhongduan = 18;
		}
		else if(UART_zhongduan == 24)
		{
			OutFire1RelayCtrl(JDQ_ON); // 打开灭火装置1 （对应老主机OUT1口）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 25)
		{
			OutFire1RelayCtrl(JDQ_OFF); // 关闭灭火装置1 （对应老主机OUT1口）
			UART_zhongduan = 27;
		}
		else if(UART_zhongduan == 26)
		{
			OutFire2RelayCtrl(JDQ_ON); // 打开灭火装置2 （对应老主机OUT2口）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 27)
		{
			OutFire2RelayCtrl(JDQ_OFF); // 关闭灭火装置2 （对应老主机OUT2口）
			UART_zhongduan = 29;
		}
		else if(UART_zhongduan == 28)
		{
			CabinSprayRelayCtrl(JDQ_ON); // 打开舱级阀门 （对应老主机OUT3口）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 29)
		{
			CabinSprayRelayCtrl(JDQ_OFF); // 关闭舱级阀门 （对应老主机OUT3口）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 30)
		{
			OutFire1RelayCtrl(JDQ_OFF); // 关闭灭火装置1 （对应老主机OUT1口）
			UART_zhongduan = 31;
		}
		else if(UART_zhongduan == 31)
		{
			OutFire2RelayCtrl(JDQ_OFF); // 关闭灭火装置2 （对应老主机OUT2口）
			UART_zhongduan = 32;
		}
		else if(UART_zhongduan == 32)
		{
			CabinSprayRelayCtrl(JDQ_OFF); // 关闭舱级阀门 （对应老主机OUT3口）
			UART_zhongduan = 33;
		}
		else if(UART_zhongduan == 33)
		{
			SoundLightRelayCtrl(JDQ_OFF); // 关闭舱级阀门 （对应老主机OUT6口）
			UART_zhongduan = 34;
		}
		else if(UART_zhongduan == 34)
		{
			DefauleRelayCtrl(JDQ_OFF); // 关闭舱级阀门 （对应老主机OUT7口）
			UART_zhongduan = 35;
		}
		else if(UART_zhongduan == 35)
		{
			ForeWarmRelayCtrl(JDQ_OFF); // 关闭预警 （对应老主机干节点1）
			UART_zhongduan = 36;
		}
		else if(UART_zhongduan == 36)
		{
			FireAlarmRelayCtrl(JDQ_OFF); // 关闭火警 （对应老主机干节点2）
			UART_zhongduan = 37;
		}
		else if(UART_zhongduan == 37)
		{
			// NULL 为了兼容老系统 此处什么都不干 跳转到
			UART_zhongduan = 38;
		}
		else if(UART_zhongduan == 38)
		{
			// NULL 为了兼容老系统 此处什么都不干 跳转到
			UART_zhongduan = 41;
		}
		else if(UART_zhongduan == 39)
		{
			// 远程复位XR803
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 40)
		{
			if(IG3301_PCAK==1) {
				//控制IG3301 继电器开启
				uint8_t modbusbuf[8]; // 发送缓冲区
				uint16_t crc16 = 0x0000; // CRC校验码
				
				modbusbuf[0] = IG3301ID; // 从机地址取值
				modbusbuf[1] = 5; // 05功能码
				modbusbuf[2] = 0;
				modbusbuf[3] = 0x28;
				modbusbuf[4] = 0xFF;
				modbusbuf[5] = 0; 
				crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
				modbusbuf[6] = crc16 >> 0;
				modbusbuf[7] = crc16 >> 8;
				xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
				
				UART_zhongduan=65;//转去打开对应XR805继电器
			}else if(IG3301_PCAK==2) {
				UART_zhongduan=42;
			}
		}
		else if(UART_zhongduan == 41)
		{
			//控制IG3301 继电器关闭
			uint8_t modbusbuf[8]; // 发送缓冲区
			uint16_t crc16 = 0x0000; // CRC校验码
			
			modbusbuf[0] = IG3301ID; // 从机地址取值
			modbusbuf[1] = 5; // 05功能码
			modbusbuf[2] = 0;
			modbusbuf[3] = 0x28;
			modbusbuf[4] = 0;
			modbusbuf[5] = 0;
			crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列

			UART_zhongduan = 43; 
		}
		else if(UART_zhongduan == 42)
		{
			//控制IG3301 继电器2 打开
			uint8_t modbusbuf[8]; // 发送缓冲区
			uint16_t crc16 = 0x0000; // CRC校验码
			
			modbusbuf[0] = IG3301ID; // 从机地址取值
			modbusbuf[1] = 5; // 05功能码
			modbusbuf[2] = 0;
			modbusbuf[3] = 0x29;
			modbusbuf[4] = 0xFF;
			modbusbuf[5] = 0;
			crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
			
			UART_zhongduan = 3; 
		}
		else if(UART_zhongduan == 43)
		{
			//控制IG3301 继电器2 关闭
			uint8_t modbusbuf[8]; // 发送缓冲区
			uint16_t crc16 = 0x0000; // CRC校验码
			
			modbusbuf[0] = IG3301ID; // 从机地址取值
			modbusbuf[1] = 5; // 05功能码
			modbusbuf[2] = 0;
			modbusbuf[3] = 0x29;
			modbusbuf[4] = 0;
			modbusbuf[5] = 0;
			crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
			
			UART_zhongduan = 54; 
		}
		else if(UART_zhongduan == 44) // 有设备掉线
		{
			FaultRelayCtrl(JDQ_ON); // 打开故障继电器（对应老主机的干节点三）
			UART_zhongduan = 37;
		}
		else if(UART_zhongduan == 45)
		{
			FaultRelayCtrl(JDQ_OFF); // 关闭故障继电器
			UART_zhongduan = 37;
		}
		else if(UART_zhongduan == 46)
		{
			SoundLightRelayCtrl(JDQ_ON); // 打开声光
			UART_zhongduan = 47;
		}
		else if(UART_zhongduan == 47)
		{
			DefauleRelayCtrl(JDQ_ON); // 打开放气
			UART_zhongduan = 48;
		}
		else if(UART_zhongduan == 48)
		{
			// 打开欠压干节点（）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 49)
		{
			SoundLightRelayCtrl(JDQ_OFF); // 关闭声光
			UART_zhongduan = 50;
		}
		else if(UART_zhongduan == 50)
		{
			DefauleRelayCtrl(JDQ_OFF); // 关闭放气指示灯
			UART_zhongduan = 51;
		}
		else if(UART_zhongduan == 51)
		{
			// 关闭欠压干节点（）
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 52)
		{
			OutFire1RelayCtrl(JDQ_ON); // 对应老主机OUT2
			UART_zhongduan = 22;
		}
		else if(UART_zhongduan == 53)
		{
			CabinSprayRelayCtrl(JDQ_ON); // 对应老主机OUT4
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 54)
		{
			CabinSprayRelayCtrl(JDQ_OFF); // 对应老主机OUT4
			UART_zhongduan = 67;
		}
		else if(UART_zhongduan == 55)
		{
			for(uint8_t i=IG3301_QD;i<=20;i++)
			{
				IG3301_QD = i + 1;
				if(cu_sxzt[i] == 1)
				{
					uint8_t modbusbuf[8]; // 发送缓冲区
					uint16_t crc16 = 0x0000; // CRC校验码
					
					modbusbuf[0] = i; // 从机地址取值
					modbusbuf[1] = 5; // 05功能码
					modbusbuf[2] = 0;
					modbusbuf[3] = 0x28;
					modbusbuf[4] = 0xFF;
					modbusbuf[5] = 0;
					crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
					modbusbuf[6] = crc16 >> 0;
					modbusbuf[7] = crc16 >> 8;
					xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
					
					break;
				}
			}
			if(IG3301_QD>20)
			{
				IG3301_QD=1;
				UART_zhongduan=63;
			}
		}
		else if(UART_zhongduan == 56)
		{
			for(uint8_t i = IG3301_TZ;i <= 20;i++)
			{
				IG3301_TZ = i + 1;
				if(cu_sxzt[i] == 1)
				{
					uint8_t modbusbuf[8]; // 发送缓冲区
					uint16_t crc16 = 0x0000; // CRC校验码
					
					modbusbuf[0] = i; // 从机地址取值
					modbusbuf[1] = 5; // 05功能码
					modbusbuf[2] = 0;
					modbusbuf[3] = 0x28;
					modbusbuf[4] = 0;
					modbusbuf[5] = 0;
					crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
					modbusbuf[6] = crc16 >> 0;
					modbusbuf[7] = crc16 >> 8;
					xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
					
					break;
				}
			}
			if(IG3301_TZ > 20)
			{
				IG3301_TZ = 1;
				UART_zhongduan = 64;
			}
		}
		else if(UART_zhongduan == 57)
		{
			OutFire1RelayCtrl(JDQ_OFF); // 关闭OUT1
			UART_zhongduan = 58;
		}
		else if(UART_zhongduan == 58)
		{
			OutFire2RelayCtrl(JDQ_OFF); // 关闭OUT2
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 59)
		{
			OutFire1RelayCtrl(JDQ_ON); // 打开OUT1
			UART_zhongduan = 60;
		}
		else if(UART_zhongduan == 60)
		{
			OutFire2RelayCtrl(JDQ_ON); // 打开OUT2
   
			UART_zhongduan = 61;
		}
		else if(UART_zhongduan == 61)
		{
//			CabinSprayRelayCtrl(JDQ_ON); // 打开OUT3 对应簇喷
			UART_zhongduan = 62;
		}
		else if(UART_zhongduan == 62)
		{
			CabinSprayRelayCtrl(JDQ_ON); // 打开OUT4 对应仓喷
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 63)
		{
			for(uint8_t i = XR805_QD;i <= 20;i++)
			{
				XR805_QD = i + 1;
				if(cang_sxzt[i]==1)
				{
//					ReqWriteCoil(i+20,1,1,100);//控制XR805 继电器开启
					
					uint8_t modbusbuf[8]; // 发送缓冲区
					uint16_t crc16 = 0x0000; // CRC校验码
					
					modbusbuf[0] = i + 20; // 从机地址取值
					modbusbuf[1] = 5; // 05功能码
					modbusbuf[2] = 0;
					modbusbuf[3] = 0x01;
					modbusbuf[4] = 0xFF;
					modbusbuf[5] = 0;
					crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
					modbusbuf[6] = crc16 >> 0;
					modbusbuf[7] = crc16 >> 8;
					xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列

					break;
				}
			}
			if(XR805_QD > 20)
			{
				XR805_QD = 1;
				UART_zhongduan = 3;
			}
		}
		else if(UART_zhongduan == 64)
		{
			for(uint8_t i = XR805_TZ;i <= 20;i++)
			{
				XR805_TZ = i + 1;
				if(cang_sxzt[i]==1)
				{
					//ReqWriteCoil(i+20,1,0,100);//控制XR805  继电器关闭
					uint8_t modbusbuf[8]; // 发送缓冲区
					uint16_t crc16 = 0x0000; // CRC校验码
					
					modbusbuf[0] = i + 20; // 从机地址取值
					modbusbuf[1] = 5; // 05功能码
					modbusbuf[2] = 0;
					modbusbuf[3] = 0x01;
					modbusbuf[4] = 0;
					modbusbuf[5] = 0;
					crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
					modbusbuf[6] = crc16 >> 0;
					modbusbuf[7] = crc16 >> 8;
					xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
					break;
				}
			}
			if(XR805_TZ > 20)
			{
				XR805_TZ = 1;
				UART_zhongduan = 30;
			}
		}
		else if(UART_zhongduan == 65)
		{
			uint8_t modbusbuf[8]; // 发送缓冲区
			uint16_t crc16 = 0x0000; // CRC校验码
			
			modbusbuf[0] = IG3301ID + 20; // 从机地址取值
			modbusbuf[1] = 5; // 05功能码
			modbusbuf[2] = 0;
			modbusbuf[3] = 0x01;
			modbusbuf[4] = 0xFF;
			modbusbuf[5] = 0;
			crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			xQueueSend(xMyRs485QueueHandle, modbusbuf, portMAX_DELAY); // 发送到队列
			
			UART_zhongduan = 3;
		}
		else if(UART_zhongduan == 66)
		{
			// NULL 为了兼容老系统 此处什么都不干 跳转到开启声光
			UART_zhongduan = 6;
		}
		else if(UART_zhongduan == 67)
		{
			// NULL 为了兼容老系统 关闭喷洒干节点
			UART_zhongduan = 0;
		}
		else if(UART_zhongduan == 68)
		{
			// NULL 为了兼容老系统 风机干节点
			UART_zhongduan = 4;
		}
		else if(UART_zhongduan == 69)
		{
			// NULL 为了兼容老系统 风机干节点
			ForeWarmRelayCtrl(JDQ_OFF); // 关闭预警继电器
			UART_zhongduan = 5;
		}
		else if(UART_zhongduan == 70) 
		{
			// NULL 为了兼容老系统 此处什么都不干 跳转到4
			// 老系统此处关闭了风机干接点 新主机上没有
			UART_zhongduan = 4;
		}
		else if(UART_zhongduan == 71) 
		{
			// 5305OUT3对应新主机 簇级管路阀门
			
			UART_zhongduan = 72;
		}
		else if(UART_zhongduan == 72) 
		{
			CabinSprayRelayCtrl(JDQ_ON);
			UART_zhongduan = 3;
		}
		
	}
	
}

void EarlyWarningCtrl(void *parameter)
{
	if(0) // 如果预警触发了 执行对应的逻辑 例如温度预警 烟雾预警
	{
		
	}
}

void FireAlarmSprayStateCtrl(void *parameter) // 伪状态机
{
	uint8_t* fire_alarm_state = (uint8_t*)parameter; // 对指针强转
	if(*fire_alarm_state == fire_alarm) // 如果逻辑设定火警触发了 执行对应的逻辑
	{
		if(out_fire_start_ctrl.cabin_spray_number == 1) {       // 如果喷放次数为1
			
			if(spray_ctrl_state != SprayEnded) {
				fire_device_time_start_flag = 1;                      // 启动间隔倒计时 开始计时
			}else {
				fire_device_time_start_flag = 0;                      // 喷放结束
				fire_device_ctrl_timeout = 0;                         // 清空计时
			}
			if(out_fire_start_ctrl.cabin_start_delay == fire_device_ctrl_timeout && spray_ctrl_state == StartDelay)  { // 
				spray_ctrl_state = FirstSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[0] == fire_device_ctrl_timeout && spray_ctrl_state == FirstSpray)  { // 
				spray_ctrl_state = SprayEnded;
				fire_device_ctrl_timeout = 0;
			}

		}else if(out_fire_start_ctrl.cabin_spray_number == 2) { // 如果喷放次数为2
		
			if(spray_ctrl_state != SprayEnded) {
				fire_device_time_start_flag = 1;                      // 启动间隔倒计时 开始计时
			}else {
				fire_device_time_start_flag = 0;                      // 喷放结束
				fire_device_ctrl_timeout = 0;                         // 清空计时
			}
			if(out_fire_start_ctrl.cabin_start_delay == fire_device_ctrl_timeout && spray_ctrl_state == StartDelay)  { // 启动延时状态
				spray_ctrl_state = FirstSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[0] == fire_device_ctrl_timeout && spray_ctrl_state == FirstSpray)  { // 第一次喷放持续时长
				spray_ctrl_state = Interval_1;
				fire_device_ctrl_timeout = 0;
			}else if( (out_fire_start_ctrl.cabin_spray_buff[1] * 60) == fire_device_ctrl_timeout && spray_ctrl_state == Interval_1)  { // 间隔时长
				spray_ctrl_state = SecndSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[2] == fire_device_ctrl_timeout && spray_ctrl_state == SecndSpray)  { // 
				spray_ctrl_state = SprayEnded;
				fire_device_ctrl_timeout = 0;
			}

		}else if(out_fire_start_ctrl.cabin_spray_number == 3) { // 如果喷放次数为3
		
			if(spray_ctrl_state != SprayEnded) {
				fire_device_time_start_flag = 1;                      // 启动间隔倒计时 开始计时
			}else {
				fire_device_time_start_flag = 0;                      // 喷放结束
				fire_device_ctrl_timeout = 0;                         // 清空计时
			}
			if(out_fire_start_ctrl.cabin_start_delay == fire_device_ctrl_timeout && spray_ctrl_state == StartDelay)  { // 启动延时状态
				spray_ctrl_state = FirstSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[0] == fire_device_ctrl_timeout && spray_ctrl_state == FirstSpray)  { // 第一次喷放持续时长
				spray_ctrl_state = Interval_1;
				fire_device_ctrl_timeout = 0;
			}else if( (out_fire_start_ctrl.cabin_spray_buff[1] * 60) == fire_device_ctrl_timeout && spray_ctrl_state == Interval_1)  { // 间隔时长
				spray_ctrl_state = SecndSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[2] == fire_device_ctrl_timeout && spray_ctrl_state == SecndSpray)  { // 第二次喷放
				spray_ctrl_state = Interval_2;
				fire_device_ctrl_timeout = 0;
			}else if( (out_fire_start_ctrl.cabin_spray_buff[3] * 60) == fire_device_ctrl_timeout && spray_ctrl_state == Interval_2)  { // 间隔时长
				spray_ctrl_state = ThirdSpray;
				fire_device_ctrl_timeout = 0;
			}else if(out_fire_start_ctrl.cabin_spray_buff[4] == fire_device_ctrl_timeout && spray_ctrl_state == ThirdSpray)  { // 第三次喷放
				spray_ctrl_state = SprayEnded; // 标记为喷放完成
				fire_device_ctrl_timeout = 0;
			}
		}
	}
}

void LinkageRelayCtrlTask(void *parameter)
{
	for(;;)
	{
		LinkageRelayCtrl();
		osDelay(50);
	}
}

