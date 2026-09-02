#include "system.h"
#include "cmd_process.h"
#include "24c04.h"
#include "w25qxx.h"

#include "string.h"

#include "bsp_debug.h"
#include "bsp_mbus.h"
#include "bsp_rs485_detect.h"

SystemSaveInfo_t  SystemSaveInfo;

uint8_t tanceqiming[20][20] = {0};//XR805备命名

uint8_t cang_sxzt[30] = {0};           		 // 舱上线状态
uint8_t cang_pbzt[30] = {0};           		 // 舱屏蔽状态
uint8_t cu_sxzt[30] = {0};           		 // 簇上线状态(是否设置为上线)
uint8_t cu_tcq_sxzt[30] = {0};           		 // 簇下挂探测器上线个数
uint8_t pack_pbzt[30][33] = {0};           		 // pack屏蔽状态

uint8_t pack_circuit = 1;
// x回路y探测器
uint8_t pack_online_buff[4][33] = {0};

uint8_t linkage_shield_state[40] = {0};  // 外联设备屏蔽状态（默认屏蔽）
uint8_t linkage_work_state[40] = {0};    // 外联设备工作状态（默认屏蔽）

void SystemInfoSave(void)
{
    ee_WriteBytes(0, (uint8_t *)&SystemSaveInfo, sizeof(SystemSaveInfo_t));//写入24C04配置信息
}

void SystemInfoLoad(void)
{
    ee_ReadBytes(0, (uint8_t *)&SystemSaveInfo, sizeof(SystemSaveInfo_t));//读取配置信息

    if(SystemSaveInfo.slave_state != 0x6321)//可以通过改变这个值来回复出厂设置
    {
			SystemSaveInfo.slave_state = 0x6321;//存储器特定值，如果不等于特定值，说明是第一次开机，需要写入默认值
			
			SystemSaveInfo.can2_slave_addr = 1;//默认CAN地址
			
			SystemSaveInfo.slave_addr485_Station = 1;      // 本机对场站的从机地址
			SystemSaveInfo.slave_baud_rate_Station = 9600; //默认MODBUS波特率
			
			SystemSaveInfo.slave_addr485_EMS = 1;      // 本机对EMS从站地址
			SystemSaveInfo.slave_baud_rate_EMS = 9600; // 默认MODBUS波特率
			
			// 默认密码 888888
			SystemSaveInfo.user_password = 888888;
			// 默认全部手动
			SystemSaveInfo.system_hand_or_auto_state = 0;//手动自动切换，0自动，1手动
			SystemSaveInfo.part1_hand_or_auto_state = 0;//手动自动切换，0自动，1手动
			SystemSaveInfo.part2_hand_or_auto_state = 0;//手动自动切换，0自动，1手动

			// 出场日期设置
			SystemSaveInfo.factory_release_year = 25;
			SystemSaveInfo.factory_release_month = 11;
			SystemSaveInfo.factory_release_days = 25;
			
			SystemSaveInfo.curr_license_store[0] = '\0';
			SystemSaveInfo.last_license_store[0] = '\0'; // 之前存储的授权码
			SystemSaveInfo.pref_license_store[0] = '\0'; // 
			
			SystemSaveInfo.curr_license_store[10] = '\0';
			SystemSaveInfo.last_license_store[10] = '\0'; // 之前存储的授权码
			SystemSaveInfo.pref_license_store[10] = '\0'; // 
			
			SystemSaveInfo.license_year = SystemSaveInfo.factory_release_year;
			SystemSaveInfo.license_month = SystemSaveInfo.factory_release_month;
			SystemSaveInfo.license_days = SystemSaveInfo.factory_release_days;
			SystemSaveInfo.license_hour = 12;
			
			SystemSaveInfo.license_minute = 0;
			SystemSaveInfo.license_second = 0;

			SystemSaveInfo.license_remain_day = 6666; // 默认剩余日期为0
			
			SystemInfoSave();//存储系统设置
			
			memset(tanceqiming,0,(20*20));//清空数组

			memset(cang_sxzt,0,30);//清空数组
			Save_cang_sxzt();//存储舱上线状态
			
			memset(cang_pbzt,0,30);//清空数组
			Save_cang_pbzt();//存储舱屏蔽状态
			
			memset(cu_sxzt,0,30);//清空数组
			Save_cu_sxzt();//存储簇上线状态
			
			memset(cu_tcq_sxzt,0,30);//清空数组
			Save_cutcq_sxzt();//存储探测器上线状态
			
			memset(pack_pbzt,0,30*30);//清空数组
			Save_pack_pbzt();//存储簇屏蔽状态
			
			// 清空包上线数组 初始化全部没有上线
			memset(pack_online_buff, 0, 4*33);
			Save_Pack_Set_Online_State();
			
			PointTypeMixtureOnlieStateDeInit();
			SavePointTypeSetOnlieState();

			SaveLinkageSheildState(); // 保存外联设备的屏蔽状态

		}
			//CAN设备地址
    if((SystemSaveInfo.can2_slave_addr < 1) || (SystemSaveInfo.can2_slave_addr > 300))
    {
        SystemSaveInfo.can2_slave_addr = 1;
    }
		//MODBUS设备地址
    if((SystemSaveInfo.slave_addr485_Station < 1) || (SystemSaveInfo.slave_addr485_Station > 255))
    {
        SystemSaveInfo.slave_addr485_Station = 1;
    }
		
		if((SystemSaveInfo.slave_addr485_EMS < 1) || (SystemSaveInfo.slave_addr485_EMS > 255))
    {
        SystemSaveInfo.slave_addr485_EMS = 1;
    }
		//波特率 场站
    if((SystemSaveInfo.slave_baud_rate_Station == 48) || (SystemSaveInfo.slave_baud_rate_Station ==96 ) || 
			(SystemSaveInfo.slave_baud_rate_Station == 192 ) || (SystemSaveInfo.slave_baud_rate_Station ==384 ) || (SystemSaveInfo.slave_baud_rate_Station ==1152 ))
    {
       
    }else
		{
			SystemSaveInfo.slave_baud_rate_Station = 96;
		}
		//波特率 BMS EMS
		if((SystemSaveInfo.slave_baud_rate_EMS == 48) || (SystemSaveInfo.slave_baud_rate_EMS ==96 ) || 
			(SystemSaveInfo.slave_baud_rate_EMS == 192 ) || (SystemSaveInfo.slave_baud_rate_EMS ==384 ) || (SystemSaveInfo.slave_baud_rate_EMS ==1152 ))
    {
       
    }else
		{
			SystemSaveInfo.slave_baud_rate_EMS = 96;
		}

}
void Load25Q128(void)
{
	Loa_cang_sxzt();//读取舱上线状态
	Loa_cang_pbzt();//读取舱屏蔽状态
	Loa_cu_sxzt();//读取簇上线状态
	Loa_cutcq_sxzt();//读取探测器上线状态
	Loa_pack_pbzt();//读取pack屏蔽状态
	
	LoadLinkageSheildState(); // 加载外联设备状态
	
	ReadPointTypeSetOnlieState();

	RS485Detect_Init();

	Load_Pack_Set_Online_State();
}
uint16_t CalcCrc16(uint8_t *buf, uint16_t len)
{
    uint8_t  temp;
    uint16_t crc16 = 0xffff;

    while(len--)
    {
        crc16 = crc16 ^ *buf++;
        for(temp = 0x80; temp != 0; temp = temp >> 1)
        {
            if(!(crc16 & 0x0001))
            {
                crc16 = crc16 >> 1;
            }
            else if(crc16 & 0x0001)
            {
                crc16 = crc16 >> 1;
                crc16 = crc16 ^ 0xa001;
            }
        }
    }

    return crc16;
}

void Save_cang_sxzt(void)//存储舱上线状态
{
	W25QXX_Write((uint8_t *)&cang_sxzt,0x108000,sizeof(cang_sxzt));
}
void Loa_cang_sxzt(void)//读取舱上线状态
{
	W25QXX_Read((uint8_t *)&cang_sxzt,0x108000, sizeof(cang_sxzt));
	
	for(uint8_t i = 0; i < sizeof(cang_sxzt); i++)
	{
		if(cang_sxzt[i] == 0xFF)
		{
			cang_sxzt[i] = 0;
		}
	}
	
	Save_cang_sxzt();
}

void Save_cu_sxzt(void)//存储簇上线状态
{
	W25QXX_Write((uint8_t *)&cu_sxzt,0x109000,sizeof(cu_sxzt));
}
void Loa_cu_sxzt(void)//读取簇上线状态
{
	W25QXX_Read((uint8_t *)&cu_sxzt,0x109000, sizeof(cu_sxzt));
	
	for(uint8_t i = 0; i < sizeof(cu_sxzt); i++)
	{
		if(cu_sxzt[i] == 0xFF)
		{
			cu_sxzt[i] = 0;
		}
	}
	
	Save_cu_sxzt();
}

void Save_cutcq_sxzt(void)//存储探测器上线状态
{
	W25QXX_Write((uint8_t *)&cu_tcq_sxzt,0x10a000,sizeof(cu_tcq_sxzt));
}
void Loa_cutcq_sxzt(void)//读取探测器上线状态
{
	W25QXX_Read((uint8_t *)&cu_tcq_sxzt,0x10a000, sizeof(cu_tcq_sxzt));
	
	for(uint8_t i = 0; i < sizeof(cu_tcq_sxzt); i++)
	{
		if(cu_tcq_sxzt[i] == 0xFF)
		{
			cu_tcq_sxzt[i] = 0;
		}
	}
	Save_cutcq_sxzt();
}

void Save_cang_pbzt(void)//存储舱屏蔽状态
{
	W25QXX_Write((uint8_t *)&cang_pbzt,0x10b000,sizeof(cang_pbzt));
}
void Loa_cang_pbzt(void)//读取舱屏蔽状态
{
	W25QXX_Read((uint8_t *)&cang_pbzt,0x10b000,sizeof(cang_pbzt));
	
	for(uint8_t i = 0; i < sizeof(cang_pbzt); i++)
	{
		if(cang_pbzt[i] == 0xFF)
		{
			cang_pbzt[i] = 0;
		}
	}
	Save_cang_pbzt();
}

void Save_pack_pbzt(void)//存储pcak屏蔽状态
{
	W25QXX_Write((uint8_t *)&pack_pbzt,0x10c000,sizeof(pack_pbzt));
}
void Loa_pack_pbzt(void)//读取pack屏蔽状态
{
	W25QXX_Read((uint8_t *)&pack_pbzt,0x10c000,sizeof(pack_pbzt));
}

void SaveLinkageSheildState(void)
{
	W25QXX_Write((uint8_t *)&linkage_shield_state,0x10D000,sizeof(linkage_shield_state));
}

void LoadLinkageSheildState(void)
{
	W25QXX_Read((uint8_t *)&linkage_shield_state,0x10D000,sizeof(linkage_shield_state));
}

// 2025/12/10 13:29 新增PACK上下线管理
void Save_Pack_Set_Online_State(void)
{
	W25QXX_Write((uint8_t *)&pack_online_buff,0x10E000,sizeof(pack_online_buff));
}

void Load_Pack_Set_Online_State(void)
{
	W25QXX_Read((uint8_t *)&pack_online_buff,0x10E000,sizeof(pack_online_buff));
	
	for(uint8_t i = 1; i < 4; i++)
	{
		for(uint8_t j = 1; j < 33; j++)
		{
			if(pack_online_buff[i][j] == 0xFF)
			{
				pack_online_buff[i][j] = 0;
			}
		}
	}
	Save_Pack_Set_Online_State();
}

void SystemDebugTest(void)
{
//	uint8_t test_buff[64] = {0};
//	
//	int16_t len = 0;
//	
//	ee_ReadBytes(0, (uint8_t *)&SystemSaveInfo, sizeof(SystemSaveInfo_t));//读取配置信息
//	
//	len = sprintf((char *)test_buff, "CAN总线从机地址:%d\r\n", SystemSaveInfo.SlaveAddr);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "用户密码:%d\r\n", SystemSaveInfo.Slaveyonghumima);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "存储状态:%d\r\n", SystemSaveInfo.slave_state);
//	DebugSendString(test_buff, len);

//	len = sprintf((char *)test_buff, "压力高压阈值:%d\r\n", SystemSaveInfo.yaliH);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "压力低压阈值:%d\r\n", SystemSaveInfo.yaliL);
//	DebugSendString(test_buff, sizeof(test_buff));
//	
//	len = sprintf((char *)test_buff, "手自动切换:%d\r\n", SystemSaveInfo.shouzidong);
//	DebugSendString(test_buff, len);

//	len = sprintf((char *)test_buff, "报警数量:%d\r\n", SystemSaveInfo.AlarmQuantity);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "防区总数:%d\r\n", SystemSaveInfo.fangquzongshu);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "逻辑总数:%d\r\n", SystemSaveInfo.luojiquzongshu);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "主机名称:%s\r\n", SystemSaveInfo.zhujiming);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "出场年份:%d\r\n", SystemSaveInfo.chuchangnian);
//	DebugSendString(test_buff, len);

//	len = sprintf((char *)test_buff, "出场月份%d\r\n", SystemSaveInfo.chuchangyue);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "出场日期%d\r\n", SystemSaveInfo.chuchangri);
//	DebugSendString(test_buff, len);

//	len = sprintf((char *)test_buff, "RS485设备地址:%d\r\n", SystemSaveInfo.SlaveAddr485);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "波特率:%d\r\n", SystemSaveInfo.SlaveBaudRate);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时1:%d\r\n", SystemSaveInfo.HJyanshi1);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时2:%d\r\n", SystemSaveInfo.HJyanshi2);
//	DebugSendString(test_buff, len);

//	len = sprintf((char *)test_buff, "火警延时3:%d\r\n", SystemSaveInfo.HJyanshi3);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时4:%d\r\n", SystemSaveInfo.HJyanshi4);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时5:%d\r\n", SystemSaveInfo.HJyanshi5);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时6:%d\r\n", SystemSaveInfo.HJyanshi6);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时7:%d\r\n", SystemSaveInfo.HJyanshi7);
//	DebugSendString(test_buff, len);
//	
//	len = sprintf((char *)test_buff, "火警延时8:%d\r\n", SystemSaveInfo.HJyanshi8);
//	DebugSendString(test_buff, len);
}

// 用来将设定为上线的探测器返回
// target 上线设定数组
// outcome 返回结果
// outcome_size outcome的长度
// 注：实际上为了兼容性outcome的长度与target的长度是一致的 目的是为了避免探测器长度大于输出的长度导致越界
uint8_t SystemQuerySetOnlineNum(uint8_t* target, uint8_t* outcome, uint8_t outcome_size)
{
	uint8_t len = 1; // 从下标1开始存值
	uint8_t i;
	memset(outcome,0,outcome_size);//清空数组
	
	for(i = 1;i <= outcome_size; i++) // 为了兼容屎山代码，下标从1开始
	{
		if(target[i] == 1)
		{
			outcome[len++] = i;
		}
	}
	return len;
}