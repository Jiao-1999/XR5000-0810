#include "bsp_station.h"

#include "bsp_itcallback.h"
#include "system.h"
#include "cmd_process.h"

#include "usart.h"

#include "cmsis_os.h"

#include "bsp_debug.h"

#define FRAME_HAND 3
#define STATION_DATA (0x45*2)

#define BUFF_LEN (FRAME_HAND + STATION_DATA)

#define SLEF_ADDR 1

uint8_t station_buff[BUFF_LEN] = {0};

extern uint8_t getCurrentSystemRunState(void);
extern uint8_t getCurrentSystemFaultState(void);
extern uint8_t getCurrentStartStopKeyState(void);
extern uint8_t getOutFireSprayState(void);
extern uint8_t getCurrentMainBackupPowerState(void);
extern int8_t getCurrentFanMode(void);
extern uint8_t getCurrentFanRunState(void);

void StationSendString(uint8_t *buf, uint8_t len)
{
	HAL_UART_Transmit(&huart1,buf,len,0xff); // 
}

// 根据寄存器地址获取值的函数 - 清晰版本
uint16_t GetRegisterValue(uint16_t reg_addr)
{
    // 基本状态寄存器 0-7
    if(reg_addr <= 7)
    {
        switch(reg_addr)
        {
            case 0: return getCurrentSystemRunState();
            case 1: return getCurrentSystemFaultState();
            case 2: return SystemSaveInfo.system_hand_or_auto_state;
            case 3: return getCurrentStartStopKeyState(); // 手启 停止状态
            case 4: return getOutFireSprayState(); // 喷放状态
            case 5: return getCurrentMainBackupPowerState();
            case 6: return getCurrentFanMode();
            case 7: return getCurrentFanRunState();
            default: return 0;
        }
    }
    // 簇状态寄存器 8-27
    else if(reg_addr >= 8 && reg_addr <= 27)
    {
//        return GetClusterState(reg_addr); // 需要实现簇状态获取函数
			return 0;
    }
    // 保留寄存器 28
    else if(reg_addr == 28)
    {
        return 0;
    }
    // CO浓度寄存器 29-48 (对应Cang_COzhi_buf[1]~[20])
    else if(reg_addr >= 29 && reg_addr <= 48)
    {
        uint8_t index = reg_addr - 29 + 1; // 29->1, 30->2, ..., 48->20
        if(index >= 1 && index <= 24)
            return Cang_COzhi_buf[index];
        else
            return 0;
    }
    // 氢气浓度寄存器 49-68 (对应Cang_H2zhi_buf[1]~[20])
    else if(reg_addr >= 49 && reg_addr <= 68)
    {
        uint8_t index = reg_addr - 49 + 1; // 49->1, 50->2, ..., 68->20
        if(index >= 1 && index <= 24)
            return Cang_H2zhi_buf[index];
        else
            return 0;
    }
    // 其他未定义寄存器
    else
    {
        return 0;
    }
}

void BspStationUploadDataApp(void)
{
    if(uartbuff[STATION_OPTICALFIBER].recepetion_flag == 1) // 
    {
        uartbuff[STATION_OPTICALFIBER].recepetion_flag = 0;
        if(uartbuff[STATION_OPTICALFIBER].recepetion_len < 2)
				{
					return;
				}
        if(uartbuff[STATION_OPTICALFIBER].recepetion_buff[0] == SystemSaveInfo.slave_addr485_Station) // 如果是本机地址
        {
            uint16_t crc16 = (uartbuff[STATION_OPTICALFIBER].recepetion_buff[uartbuff[STATION_OPTICALFIBER].recepetion_len-1]<<8) |
                            uartbuff[STATION_OPTICALFIBER].recepetion_buff[uartbuff[STATION_OPTICALFIBER].recepetion_len-2];
            
            if(CalcCrc16(uartbuff[STATION_OPTICALFIBER].recepetion_buff, uartbuff[STATION_OPTICALFIBER].recepetion_len - 2) == crc16) // 校验码正确
            {
                if(uartbuff[STATION_OPTICALFIBER].recepetion_buff[1] == 0x04) // 04功能码
                {
                    uint8_t buff_count = 0;
                    
                    // 解析请求参数
                    uint16_t start_addr = (uartbuff[STATION_OPTICALFIBER].recepetion_buff[2] << 8) | 
                                         uartbuff[STATION_OPTICALFIBER].recepetion_buff[3];
                    uint16_t reg_count = (uartbuff[STATION_OPTICALFIBER].recepetion_buff[4] << 8) | 
                                        uartbuff[STATION_OPTICALFIBER].recepetion_buff[5];
                    
                    // 构建响应数据
                    station_buff[buff_count++] = SystemSaveInfo.slave_addr485_Station;
                    station_buff[buff_count++] = 0x04;
                    station_buff[buff_count++] = reg_count * 2; // 字节数 = 寄存器数 × 2
                    
                    // 根据请求的起始地址和数量填充数据
                    for(uint16_t i = 0; i < reg_count; i++)
                    {
                        uint16_t reg_addr = start_addr + i;
                        uint16_t reg_value = GetRegisterValue(reg_addr); // 需要实现这个函数
                        
                        station_buff[buff_count++] = (reg_value >> 8) & 0xFF;
                        station_buff[buff_count++] = reg_value & 0xFF;
                    }
                    
                    // 计算CRC
                    crc16 = CalcCrc16(station_buff, buff_count);
                    station_buff[buff_count++] = crc16 & 0xFF;
                    station_buff[buff_count++] = (crc16 >> 8) & 0xFF;
                    
                    // 发送响应
                    StationSendString(station_buff, buff_count);
                }
            }
        }
    }
}

void StationResponseTesk(void *parameter)
{
	for(;;)
	{
		BspStationUploadDataApp();
		osDelay(100);
	}
}




