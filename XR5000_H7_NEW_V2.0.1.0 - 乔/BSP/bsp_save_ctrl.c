#include "bsp_save_ctrl.h"
#include "w25qxx.h"
#include "string.h"

#include "bsp_rtc.h"

#include "bsp_debug.h"

#include "FreeRTOS.h"
#include "task.h"

/* XR5000_W25Q_MUTEX_FIX_20260804: record transactions share the driver-owned Flash mutex. */
void BspSaveCtrlInit(void)
{
	W25QXX_TransactionMutexInit();
}

static void RecordFlashLock(void)
{
	W25QXX_TransactionLock();
}

static void RecordFlashUnlock(void)
{
	W25QXX_TransactionUnlock();
}

#define DATA_FRAME_HEADER 0xAA4C535A

#define DATA_NUM_READ_LEN 256

typedef struct
{
	uint32_t frame_id;
	uint8_t bitmap_buff[DATA_NUM_READ_LEN - 4];
}BitMap_t;


typedef union
{
	BitMap_t bit_map;
	uint8_t  buff[DATA_NUM_READ_LEN];
}DataLenBase_t;

DataLenBase_t data_cache = {0};

FlashReadCache_t alarm_fault_data;

uint32_t fault_data_addr[4] = {FAULT_DATA_ADDR, FAULT_DATA_ADDR + 0x1000UL, FAULT_DATA_ADDR + 0x2000UL, FAULT_DATA_ADDR + 0x3000UL};


uint32_t alarm_data_addr[5] = {ALARM_DATA_ADDR, ALARM_DATA_ADDR + 0x1000UL, ALARM_DATA_ADDR + 0x2000UL, ALARM_DATA_ADDR + 0x3000UL, 
															 ALARM_DATA_ADDR + 0x4000UL};

uint32_t gasof_data_addr[4] = {GASER_DATA_ADDR, GASER_DATA_ADDR + 0x1000UL, GASER_DATA_ADDR + 0x2000UL, GASER_DATA_ADDR + 0x3000UL};


uint32_t other_data_addr[4] = {OTHER_DATA_ADDR, OTHER_DATA_ADDR + 0x1000UL, OTHER_DATA_ADDR + 0x2000UL, OTHER_DATA_ADDR + 0x3000UL};

uint32_t getFlashSaveFrameId(void)
{
	return data_cache.bit_map.frame_id;
}

// 用来清除记录数量
uint8_t BspFlashEraseNumberSectorCtrl(uint32_t addr)
{
	uint8_t flag = 0;
	// 读取帧头 
	W25QXX_Read(data_cache.buff, addr, DATA_NUM_READ_LEN);
	if( (data_cache.bit_map.frame_id & 0x00FFFFFF) != 0x004C535A ) // 如果帧头不对 认为需要初始化
	{
		W25QXX_Erase_Sector(addr/4096);  // 擦除该扇区
		
		memset(data_cache.buff, 0xFF, DATA_NUM_READ_LEN); // 初始化为0xFF
		data_cache.bit_map.frame_id = DATA_FRAME_HEADER; // 标记为使用
		
		BspFlashWrite(data_cache.buff, addr, DATA_NUM_READ_LEN); // 每次从头开始写入
		
		flag = 1;
	}
	
	return flag;
}

void BspFlashSaveInit(void)
{
	uint8_t init_flag;
	init_flag = BspFlashEraseNumberSectorCtrl(FAULT_NUMS_ADDR); // 初始化故障数量存储区
	if(init_flag == 1)
	{
		for(int8_t i = 0; i < 4; i++)
		{
			W25QXX_Erase_Sector(fault_data_addr[i]/4096);
			DebugPrintf("%d\t", i);
		}
		DebugPrintf("\r\n扇区1清除\r\n");
	}
	init_flag = BspFlashEraseNumberSectorCtrl(ALARM_NUMS_ADDR); // 初始化报警数量存储区
	if(init_flag == 1)
	{
		for(int8_t i = 0; i < 5; i++)
		{
			W25QXX_Erase_Sector(alarm_data_addr[i]/4096);
			DebugPrintf("%d\t", i);
		}
		DebugPrintf("\r\n扇区2清除\r\n");
	}
	
	init_flag = BspFlashEraseNumberSectorCtrl(GASER_NUMS_ADDR); // 初始化气灭数量存储区
	if(init_flag == 1)
	{
		for(int8_t i = 0; i < 4; i++)
		{
			W25QXX_Erase_Sector(gasof_data_addr[i]/4096);
			DebugPrintf("%d\t", i);
		}
		DebugPrintf("\r\n扇区3清除\r\n");
	}
	
	init_flag = BspFlashEraseNumberSectorCtrl(OTHER_NUMS_ADDR); // 初始化其它数量存储区
	if(init_flag == 1)
	{
		for(int8_t i = 0; i < 4; i++)
		{
			W25QXX_Erase_Sector(other_data_addr[i]/4096);
			DebugPrintf("%d\t", i);
		}
		DebugPrintf("\r\n扇区4清除\r\n");
	}
}

// 默认读一个扇区4096字节 存储到data_cache中 读取时默认读取一扇 从扇区中找数据
int16_t ReadNumberSaveSector(uint32_t sector_addr)
{
	uint16_t sum = 0;
	
	uint16_t i;
	uint8_t j;
	
	W25QXX_Read(data_cache.buff, sector_addr, DATA_NUM_READ_LEN);
	
	for(i = 0; i < DATA_NUM_READ_LEN - 4; i++)
	{
		if(data_cache.bit_map.bitmap_buff[i] == 0)
		{
			sum += 8;
			continue; // 进入下一次循环
		}
		for(j = 0; j < 8; j++)
		{
			if( ((data_cache.bit_map.bitmap_buff[i] >> j) & 0x01) == 0)
			{
				sum++;
			}
			else
			{
				break;
			}
		}
		if(j != 8)
		{
			break;
		}
	}
	return sum;
}


int16_t getFlashSaveDataNummber(FlashReadCtrlId id)
{
	int16_t num = 0;
	RecordFlashLock();
	switch(id)
	{
		case FAULT_FLASH_SAVE:
			num = ReadNumberSaveSector(FAULT_NUMS_ADDR);
			break;
		case FIRE_FLASH_SAVE :
			num = ReadNumberSaveSector(ALARM_NUMS_ADDR);
			break;
		case GASER_FLASH_SAVE:
			num = ReadNumberSaveSector(GASER_NUMS_ADDR);
			break;
		case OTHER_FLASH_SAVE:
			num = ReadNumberSaveSector(OTHER_NUMS_ADDR);
			break;
		default:
			break;
	}
	RecordFlashUnlock();
	return num;
}

// 
void SaveDataNumToFlash(uint32_t sector_addr, uint16_t data_sum)
{
	uint8_t array_id;
	uint8_t bit_id;
	
	if(data_sum == 0)
	{
		return ;
	}
	
	W25QXX_Read(data_cache.buff, sector_addr, DATA_NUM_READ_LEN); // 先读出来数据
	// 计算有几个字节
	array_id = (data_sum) / 8;
	for(uint16_t i = 0; i < array_id; i++)
	{
		data_cache.bit_map.bitmap_buff[i] = 0;
	}
	// 计算偏移
	bit_id = (data_sum) % 8;
	data_cache.bit_map.bitmap_buff[array_id] = 0xFFU << bit_id;

	data_cache.bit_map.frame_id = DATA_FRAME_HEADER;
	BspFlashWrite(data_cache.buff, sector_addr, DATA_NUM_READ_LEN);
}

void setFlashSaveDataNumber(FlashReadCtrlId id, uint16_t data_sum)
{
	RecordFlashLock();
	switch(id)
	{
		case FAULT_FLASH_SAVE:
			SaveDataNumToFlash(FAULT_NUMS_ADDR, data_sum);
			break;
		case FIRE_FLASH_SAVE :
			SaveDataNumToFlash(ALARM_NUMS_ADDR, data_sum);
			break;
		case GASER_FLASH_SAVE:
			SaveDataNumToFlash(GASER_NUMS_ADDR, data_sum);
			break;
		case OTHER_FLASH_SAVE:
			SaveDataNumToFlash(OTHER_NUMS_ADDR, data_sum);
			break;
		default:
			break;
	}
	RecordFlashUnlock();
}

extern FlashReadCache_t read_data[5];

void BspSaveDataToFlash(FlashReadCtrlId id, FlashSaveType type, void *type_struct)
{
	int16_t num = 0;
	RecordFlashLock();
	switch(id)
	{
		case FAULT_FLASH_SAVE: {
//			W25QXX_Read(alarm_fault_data.byte_buff, FAULT_DATA_ADDR, 4096); // 获取该地址下的数据
//			// 
//			FlashSaveDetectFault_t *pFault = (FlashSaveDetectFault_t *)type_struct;
//			// 获取写入位置
//			num = getFlashSaveDataNummber(FAULT_FLASH_SAVE);
//			// 赋值
//			alarm_fault_data.fs_sys_fault[num].fs_time = pFault->fs_time;
//			// 更新数量
//			num++;
//			setFlashSaveDataNumber(FAULT_FLASH_SAVE, num);
//			// 写入FLASH
//			BspFlashWrite(alarm_fault_data.byte_buff, FAULT_NUMS_ADDR, 4096);
/********************************************************************************************/
				// 读取数量
				num = getFlashSaveDataNummber(FAULT_FLASH_SAVE);
				if(num >= 2000) // 如果够 2000 条了 删除最先记录的500条
				{
					// 读到缓冲区中
					for(uint8_t i = 0; i < 4; i++)
					{
						BspReadFlashData(FAULT_FLASH_SAVE, read_data[i].byte_buff, i);
					}
					// 清空FLASH中的内容
					for(int8_t i = 0; i < 4; i++)
					{
						W25QXX_Erase_Sector(fault_data_addr[i]/4096);
					}
					W25QXX_Erase_Sector(FAULT_NUMS_ADDR/4096); // 清空数量存储区域
					
					for(uint8_t i = 1; i < 4; i++)
					{
						BspFlashWrite(read_data[i].byte_buff, fault_data_addr[i - 1], 4096);
					}
					num = 1500;
					setFlashSaveDataNumber(FAULT_FLASH_SAVE, num);
				}
				// 从地址偏移处开始写入
				BspFlashWrite((uint8_t *)type_struct, 
					fault_data_addr[num/500] + (num%500)*sizeof(FlashSaveDetectFault_t), // 计算偏移地址
					sizeof(FlashSaveDetectFault_t)); // 存储数据
				// 更新数量
				num++;
				setFlashSaveDataNumber(FAULT_FLASH_SAVE, num);
			}
			break;
		case FIRE_FLASH_SAVE : {
				// 读取报警数量
				num = getFlashSaveDataNummber(FIRE_FLASH_SAVE);
				if(num >= 2000)
				{
					// 读到缓冲区中
					for(uint8_t i = 0; i < 5; i++)
					{
						BspReadFlashData(FIRE_FLASH_SAVE, read_data[i].byte_buff, i);
					}
					// 清空FLASH中的内容
					for(int8_t i = 0; i < 5; i++)
					{
						W25QXX_Erase_Sector(alarm_data_addr[i]/4096);
					}
					W25QXX_Erase_Sector(ALARM_NUMS_ADDR/4096); // 清空 火警 数量存储区域
					
					for(uint8_t i = 1; i < 5; i++)
					{
						BspFlashWrite(read_data[i].byte_buff, alarm_data_addr[i - 1], 4096);
					}
					num = 1500;
					setFlashSaveDataNumber(FIRE_FLASH_SAVE, num);
				}
				
				// 从地址偏移处开始写入
				BspFlashWrite((uint8_t *)type_struct, 
					alarm_data_addr[num/FLASH_ALARM_SUM_PER_SECTOR] + (num%FLASH_ALARM_SUM_PER_SECTOR)*sizeof(FlashSaveFireAlarm_t), // 计算偏移地址
					sizeof(FlashSaveFireAlarm_t)); // 存储数据
				// 更新报警数量
				num++;
				setFlashSaveDataNumber(FIRE_FLASH_SAVE, num);
			}
			break;
		case GASER_FLASH_SAVE: {
				// 读取气灭动作数量
				num = getFlashSaveDataNummber(GASER_FLASH_SAVE);
				if(num >= 2000)
				{
					// 读到缓冲区中
					for(uint8_t i = 0; i < 4; i++)
					{
						BspReadFlashData(GASER_FLASH_SAVE, read_data[i].byte_buff, i);
					}
					// 清空FLASH中的内容
					for(int8_t i = 0; i < 4; i++)
					{
						W25QXX_Erase_Sector(gasof_data_addr[i]/4096);
					}
					W25QXX_Erase_Sector(GASER_NUMS_ADDR/4096); // 清空 气灭 数量存储区域
					
					for(uint8_t i = 1; i < 4; i++)
					{
						BspFlashWrite(read_data[i].byte_buff, gasof_data_addr[i - 1], 4096);
					}
					num = 1500;
					setFlashSaveDataNumber(GASER_FLASH_SAVE, num);
				}
			
				// 从地址偏移处开始写入
				BspFlashWrite((uint8_t *)type_struct, 
					gasof_data_addr[num/500] + (num%500)*sizeof(FlashSaveGasOutfires_t), // 计算偏移地址
					sizeof(FlashSaveGasOutfires_t)); // 存储数据
				// 更新报警数量
				num++;
				setFlashSaveDataNumber(GASER_FLASH_SAVE, num);
			}
			
			break;
		case OTHER_FLASH_SAVE: {
				// 读取气灭动作数量
				num = getFlashSaveDataNummber(OTHER_FLASH_SAVE);
				if(num >= 2000)
				{
					// 读到缓冲区中
					for(uint8_t i = 0; i < 4; i++)
					{
						BspReadFlashData(OTHER_FLASH_SAVE, read_data[i].byte_buff, i);
					}
					// 清空FLASH中的内容
					for(int8_t i = 0; i < 4; i++)
					{
						W25QXX_Erase_Sector(other_data_addr[i]/4096);
					}
					W25QXX_Erase_Sector(OTHER_NUMS_ADDR/4096); // 清空 气灭 数量存储区域
					
					for(uint8_t i = 1; i < 4; i++)
					{
						BspFlashWrite(read_data[i].byte_buff, other_data_addr[i - 1], 4096);
					}
					num = 1500;
					setFlashSaveDataNumber(OTHER_FLASH_SAVE, num);
				}
			
				// 从地址偏移处开始写入
				BspFlashWrite((uint8_t *)type_struct, 
					other_data_addr[num/500] + (num%500)*sizeof(FlashSaveOtherRecord_t), // 计算偏移地址
					sizeof(FlashSaveOtherRecord_t)); // 存储数据
				// 更新报警数量
				num++;
				setFlashSaveDataNumber(OTHER_FLASH_SAVE, num);

			}
			break;
		default:
			
			break;
	}
	RecordFlashUnlock();
}

int16_t BspReadFlashData(FlashReadCtrlId id, uint8_t *buff, uint8_t x_sector)
{
	int16_t num = 0;
	if(buff == NULL)
	{
		return -1;
	}
	RecordFlashLock();
	switch(id)
	{
		case FAULT_FLASH_SAVE: {
			W25QXX_Read(buff, fault_data_addr[x_sector], 4096);  
			break;
		}
		case FIRE_FLASH_SAVE : {
			W25QXX_Read(buff, alarm_data_addr[x_sector], 4096);  
			break;
		}
		case GASER_FLASH_SAVE: {
			W25QXX_Read(buff, gasof_data_addr[x_sector], 4096); 
			break;
		}
		case OTHER_FLASH_SAVE:
			W25QXX_Read(buff, other_data_addr[x_sector], 4096); 
			break;
		default:
			
			break;
	}
	RecordFlashUnlock();
	return num;
}

void BspClearFlashData(void)
{
	int16_t num;
	RecordFlashLock();

	// 擦除故障区域
	num = getFlashSaveDataNummber(FAULT_FLASH_SAVE);
	W25QXX_Erase_Sector(FAULT_NUMS_ADDR / 4096);  // 擦除该扇区
	memset(data_cache.buff, 0xFF, DATA_NUM_READ_LEN); // 初始化为0xFF
	data_cache.bit_map.frame_id = DATA_FRAME_HEADER; // 标记为使用	
	BspFlashWrite(data_cache.buff, FAULT_NUMS_ADDR, DATA_NUM_READ_LEN); // 每次从头开始写入
	
	for(int8_t i = (num/500) + 1; i > 0; i--)
	{
		W25QXX_Erase_Sector(i);  // 擦除该扇区
	}
	
	// 擦除气灭区域
	num = getFlashSaveDataNummber(GASER_FLASH_SAVE);
	W25QXX_Erase_Sector(GASER_NUMS_ADDR / 4096);  // 擦除该扇区
	memset(data_cache.buff, 0xFF, DATA_NUM_READ_LEN); // 初始化为0xFF
	data_cache.bit_map.frame_id = DATA_FRAME_HEADER; // 标记为使用	
	BspFlashWrite(data_cache.buff, GASER_NUMS_ADDR, DATA_NUM_READ_LEN); // 每次从头开始写入
	
	for(int8_t i = (num/500) + 6; i > 5; i--)
	{
		W25QXX_Erase_Sector(i);  // 擦除该扇区
	}
	
	// 擦除其它区域
	num = getFlashSaveDataNummber(OTHER_FLASH_SAVE);
	W25QXX_Erase_Sector(OTHER_NUMS_ADDR / 4096);  // 擦除该扇区
	memset(data_cache.buff, 0xFF, DATA_NUM_READ_LEN); // 初始化为0xFF
	data_cache.bit_map.frame_id = DATA_FRAME_HEADER; // 标记为使用	
	BspFlashWrite(data_cache.buff, OTHER_NUMS_ADDR, DATA_NUM_READ_LEN); // 每次从头开始写入

	for(int8_t i = 0; i < 4; i++)
	{
		W25QXX_Erase_Sector(other_data_addr[i]/4096);
	}
	
//	W25QXX_Read(data_cache.buff, other_data_addr[0], 8);
//	
//	for(int8_t i = 0; i < 8; i++)
//	{
//		if(data_cache.buff[i] != 0xFF)
//		{
//			DebugPrintf("%d %d 1擦除失败\r\n", i, data_cache.buff[i]);
//		}
//		else
//		{
//			DebugPrintf("successful");
//		}
//		
//	}
//	
//	W25QXX_Read(data_cache.buff, other_data_addr[1], 8);
//	
//	for(int8_t i = 0; i < 8; i++)
//	{
//		if(data_cache.buff[i] != 0xFF)
//		{
//			DebugPrintf("%d %d 2擦除失败\r\n", i, data_cache.buff[i]);
//		}
//		else
//		{
//			DebugPrintf("successful");
//		}
//		
//	}
	
	// 擦除报警区域
	num = getFlashSaveDataNummber(FIRE_FLASH_SAVE);
	W25QXX_Erase_Sector(ALARM_NUMS_ADDR / 4096);  // 擦除该扇区
	memset(data_cache.buff, 0xFF, DATA_NUM_READ_LEN); // 初始化为0xFF
	data_cache.bit_map.frame_id = DATA_FRAME_HEADER; // 标记为使用	
	BspFlashWrite(data_cache.buff, ALARM_NUMS_ADDR, DATA_NUM_READ_LEN); // 每次从头开始写入
	
	for(int8_t i = (num/400) + 17; i > 15; i--)
	{
		W25QXX_Erase_Sector(i);  // 擦除该扇区
	}
	RecordFlashUnlock();
}

extern uint8_t secs,years,months,weeks,days,hours,minutes;

void BspCommonDataSaveApp(FlashReadCtrlId addr_type, FlashSaveType save_type, uint8_t cluster_id, uint8_t pack_or_cabin)
{
	// 三种通用类型公用同一个父结构体
	FlashSaveBase_t temp_data = {0};
	
	// 设备号赋值
	temp_data.fs_detect_id.cluster_id = cluster_id; // 簇ID
	temp_data.fs_detect_id.cabin_or_pack_id = pack_or_cabin; // packID 簇不为0就是pack
	
	getBM8563TimeToSystemTime(); // 获取一下RTC时间
	
	FlashSaveTimeBuff temp_time = {0};
	
	// 给数组赋值
	setFlashTime(temp_time, years, months, days, hours, minutes, secs);

	for(uint8_t i = 0; i < 5; i++)
	{
		temp_data.fs_time_buff[i] = temp_time[i];
	}

	// 状态赋值
	temp_data.state = save_type;
	
	//DebugSendString((uint8_t *)&temp_data, 8);
	
	BspSaveDataToFlash(addr_type, save_type, (void *)&temp_data);

}

// 操作函数
void setFlashTime(FlashSaveTimeBuff time, uint8_t year, uint8_t month, uint8_t day, 
                  uint8_t hour, uint8_t minute, uint8_t second)
{
    time[0] = ((year & 0x7F) >> 4) | ((month & 0x0F) << 3);
    time[1] = ((year & 0x0F) << 4) | ((day & 0x1F) >> 1);
    time[2] = ((day & 0x01) << 7) | ((hour & 0x1F) << 2);
    time[3] = ((hour & 0x07) << 5) | ((minute & 0x3F) >> 1);
    time[4] = ((minute & 0x01) << 7) | (second & 0x3F);
}

void getFlashTime(const FlashSaveTimeBuff time, uint8_t *year, uint8_t *month, uint8_t *day,
                  uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    *year = ((time[0] & 0x07) << 4) | ((time[1] >> 4) & 0x0F);
    *month = (time[0] >> 3) & 0x0F;
    *day = ((time[1] & 0x0F) << 1) | ((time[2] >> 7) & 0x01);
    *hour = (time[2] >> 2) & 0x1F;
    *minute = ((time[3] & 0x1F) << 1) | ((time[4] >> 7) & 0x01);
    *second = time[4] & 0x3F;
}

void getFlashTime_Plus(const FlashSaveTimeBuff time, FlashSaveTime_t *time_entry)
{
	time_entry->years = ((time[0] & 0x07) << 4) | ((time[1] >> 4) & 0x0F);
	time_entry->months = (time[0] >> 3) & 0x0F;
	time_entry->days = ((time[1] & 0x0F) << 1) | ((time[2] >> 7) & 0x01);
	time_entry->hours = (time[2] >> 2) & 0x1F;
	time_entry->minute = ((time[3] & 0x1F) << 1) | ((time[4] >> 7) & 0x01);
	time_entry->second = time[4] & 0x3F;
}









