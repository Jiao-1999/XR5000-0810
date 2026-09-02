#ifndef __BSP_SAVE_CTRL_H
#define __BSP_SAVE_CTRL_H

#include "main.h"

#define FLASH_ALARM_SUM_PER_SECTOR 400

// 三字节寻址
#define FLASH_BASE_ADDR 0x000000

// 故障存储基址 0X0000
#define FAULT_BASE_ADDR (FLASH_BASE_ADDR)
// 故障条数存储地址
#define FAULT_NUMS_ADDR (FAULT_BASE_ADDR)
// 故障数据存储地址
#define FAULT_DATA_ADDR (FAULT_BASE_ADDR + 0x001000UL)

// 气灭存储基址 FLASH基址+地址偏移 五个扇区的大小 0X5000
#define GASER_BASE_ADDR (FLASH_BASE_ADDR + 0x005000UL)
// 气灭记录数量存储区域
#define GASER_NUMS_ADDR (GASER_BASE_ADDR)
// 气灭记录数据存储
#define GASER_DATA_ADDR (GASER_NUMS_ADDR + 0x001000UL)

// 其它记录存储基址 0XA0000
#define OTHER_BASE_ADDR (FLASH_BASE_ADDR + 0x00A0000UL)
// 
#define OTHER_NUMS_ADDR (OTHER_BASE_ADDR)
//
#define OTHER_DATA_ADDR (OTHER_NUMS_ADDR + 0x002000UL)

// 报警存储基址 0XF000
#define ALARM_BASE_ADDR (FLASH_BASE_ADDR + 0x010000UL)
// 报警条数存储地址
#define ALARM_NUMS_ADDR (ALARM_BASE_ADDR)
// 报警数据存储地址
#define ALARM_DATA_ADDR (ALARM_NUMS_ADDR + 0x001000UL)

// 灭火装置ID 用来记录灭火装置开始倒计时 停止倒计时
#define OUTFIRE_CLUSTER_ID 0x4D
#define OUTFIRE_PACKAGE_ID 0x48
// 灭火装置1ID 用来记录灭火装置1启动停止
#define OUTFIR1_CLUSTER_ID 0x4D
#define OUTFIR1_PACKAGE_ID (0x48 + 1)
// 灭火装置2ID 用来记录灭火装置2启动停止
#define OUTFIR2_CLUSTER_ID 0x4D
#define OUTFIR2_PACKAGE_ID (0x48 + 2)

#define LINKAGE_CLUSTER_ID 0x57
#define INOUT_GENERALLY_ID 0x58

//typedef enum
//{
//	Deflate_Package_ID = 1,
//	SoundLt_Package_ID = 2,
//	SirenBk_Package_ID = 3, // 警笛备用
//	OutFir1_Package_ID = 4,
//	OutFir2_Package_ID = 5, // 
//	CabinBK_Package_ID = 6,
//	
//	FEEDBK1_Package_ID = 7,
//	FEEDBK2_Package_ID = 8,
//	HANDPOT_Package_ID = 9,
//	
//	// 主面板上的报警器按钮
//	ALARM_ANNUNCIATOR_ID = 10,
//	// 主面板上复位按下
//	SYS_RESET_Package_ID = 11,
//	
//	SYS_SELFCHECK_Package_ID = 12,
//	
//	SYS_MAIN_POWER_KEY_ID = 13,
//	SYS_BACK_POWER_KEY_ID = 14,

//	SYS_TURN_OFF_Package_ID = 15,

//	SYS_FLASH_FAULT_ID = 16,
//	
//	SYS_HAND_AUTO_Package_ID = 17,
//	
//	PART1_HAND_AUTO_Package_ID = 18,
//	PART2_HAND_AUTO_Package_ID = 19,
//	
//	LINKAGE_FAN_Package_ID = 20,
//	
//	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1 = 21,
//	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2 = 22,
//	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3 = 23,
//	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4 = 24,
//	
//}LinkagePackage_Id;

typedef enum
{
	Deflate_Package_ID = 1,					// 泄压设备ID（气体灭火系统泄压装置）
	SoundLt_Package_ID = 2,					// 声光报警器设备ID
	SirenBk_Package_ID = 3, 				// 备用警笛设备ID
	OutFir1_Package_ID = 4,					// 外部消防设备1
	OutFir2_Package_ID = 5, 				// 外部消防设备2
	CabinBK_Package_ID = 6,					// 舱室备用输出设备

	FEEDBK1_Package_ID = 7,					// 反馈信号1（设备动作之后的状态反馈输入）
	FEEDBK2_Package_ID = 8,					// 反馈信号2
	HANDPOT_Package_ID = 9,					// 手动消防瓶启动装置（手动启动灭火瓶）

	// 主面板上的报警器按钮
	ALARM_ANNUNCIATOR_ID = 10,				// 主机面板报警声光启动按钮
	// 主面板上复位按下
	SYS_RESET_Package_ID = 11,				// 系统复位按键，按下清除当前故障、报警状态

	SYS_SELFCHECK_Package_ID = 12,			// 系统自检按钮，触发整机设备自检流程

	SYS_MAIN_POWER_KEY_ID = 13,				// 主电状态检测（市电输入）
	SYS_BACK_POWER_KEY_ID = 14,			 // 备电状态检测（蓄电池）

	SYS_TURN_OFF_Package_ID = 15,			// 系统停机按钮，整机断电关闭设备

	SYS_FLASH_FAULT_ID = 16,				// Flash存储故障（W25Q芯片读写异常）

	SYS_HAND_AUTO_Package_ID = 17,			// 整机全局手动/自动模式切换按钮
	PART1_HAND_AUTO_Package_ID = 18,		 // 1分区手动自动模式切换
	PART2_HAND_AUTO_Package_ID = 19,		 // 2分区手动自动模式切换

	LINKAGE_FAN_Package_ID = 20,			 // 联动风机（消防状态下启动或者关闭排风机）

	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1 = 21,	// 通用隔离输出1号，继电器无源输出，外接第三方设备
	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2 = 22,	// 通用隔离输出2号
	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3 = 23,	// 通用隔离输出3号
	GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4 = 24, /* general output 4 */
	SYS_CHECK_Package_ID = 25, /* XR5000_CHECK_CHANGE_20260804: system check operation record ID. */

}LinkagePackage_Id;

typedef enum
{
	FAULT_FLASH_SAVE = 0,
	FIRE_FLASH_SAVE  = 1,
	GASER_FLASH_SAVE = 2,
	OTHER_FLASH_SAVE = 3,
	
}FlashReadCtrlId;

typedef enum
{
	// 探测器外联设备
	DISCONNECT    = 0,   // 掉线故障
	DIS_RECOVERY  = 1,   // 从掉线恢复
	SHORTCIRCUIT  = 2,   // 短路
	SHO_RECOVERY  = 3,   // 从短路恢复
	LINKAGE_PRESS = 4,	 // 手动按下启动，触发联动输出
	
	// 探测器报警
	FIRGAS_ALARM = 5,    // 预警
	EAR_RECOVERY = 6,    // 从预警恢复
	SMOKE_ALARM  = 7,    // 烟雾火警
	TEMPRT_ALARM = 8,    // 温度火警
	
	// 气灭分区 - 灭火装置1
	OUTFIRE1OPEN_1 = 9,      // 灭火装置1第一次吸合
	OUTFIRE1OPEN_2 = 10,     // 灭火装置1第二次吸合
	OUTFIRE1OPEN_3 = 11,     // 灭火装置1第三次吸合
	OUTFIRE1CLOSE  = 12,     // 灭火装置1关闭
	
	// 气灭分区 - 灭火装置2
	OUTFIRE2OPEN_1 = 13,     // 灭火装置2第一次吸合
	OUTFIRE2OPEN_2 = 14,     // 灭火装置2第二次吸合
	OUTFIRE2OPEN_3 = 15,     // 灭火装置2第三次吸合
	OUTFIRE2CLOSE  = 16,     // 灭火装置2关闭
	
	// 灭火装置控制
	OUTFIRE_1_START_DELAY = 17,  // 第一次启动倒计时
	OUTFIRE_2_START_DELAY = 18,  // 第二次启动倒计时
	OUTFIRE_3_START_DELAY = 19,  // 第三次启动倒计时
	OUTFIRE_STOP          = 20,  // 灭火装置停止
	OUTFIRE_OVER          = 21,  // 灭火装置喷放结束
	OUTFIRESTART_AGAIN    = 22,  // 灭火装置再次启动
	
	// 按钮操作
	OUTFIRE_ST_PRESS = 23,   // 启动按钮按下
	OUTFIRE_SP_PRESS = 24,   // 停止按钮按下
	OUTFIRE_SL_PRESS = 25,   // 声光按钮按下
	
	// 反馈信号
	OUTFIRE_FEEDBACK1 = 26,
	OUTFIRE_FEEDBACK2 = 27,
	
	// 系统操作
	OTHER_TURN_ON          = 28,  // 开机
	OTHER_TURN_OFF         = 29,  // 关机
	OTHER_SYS_TURN_HAND    = 30,  // 联动启动切换为手动
	OTHER_SYS_TURN_AUTO    = 31,  // 手动切换为自动
	
	// 分区控制
	OTHER_PART1_TURN_HAND  = 32,  // 分区1切换为手动
	OTHER_PART1_TURN_AUTO  = 33,  // 分区1切换为自动
	OTHER_PART2_TURN_HAND  = 34,  // 分区2切换为手动
	OTHER_PART2_TURN_AUTO  = 35,  // 分区2切换为自动
	
	// 系统功能
	OTHER_SYS_RESET        = 36,  // 系统复位
	OTHER_SYS_SELF_CHECK   = 37,  /* system self-check */
	OTHER_SYS_CHECK        = 68,  /* XR5000_CHECK_CHANGE_20260804: system check operation. */
	MBUS2_HAND_ALARM       = 42,  // XR5000_MBUS2_HAND_ALARM_FIRE_HISTORY_20260729: loop2 XR2200 manual alarm.
	// XR5000_LOOP3_STATUS_CHANGE_20260730: protocol-level warning, fire and sensor-fault records.
	RS485_TEMP_WARNING = 43,
	RS485_CO_FIRE = 44,
	RS485_H2_FIRE = 45,
	RS485_TEMP_SENSOR_FAULT = 46,
	RS485_TEMP_SENSOR_RECOVERY = 47,
	RS485_SMOKE_POLLUTION_FAULT = 48,
	RS485_SMOKE_POLLUTION_RECOVERY = 49,
	/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: loop1 warning and typed sensor-fault history. */
	LOOP1_TEMP_WARNING = 50,
	LOOP1_SMOKE_WARNING = 51,
	LOOP1_TEMP_SENSOR_FAULT = 52,
	LOOP1_TEMP_SENSOR_RECOVERY = 53,
	LOOP1_SMOKE_POLLUTION_FAULT = 54,
	LOOP1_SMOKE_POLLUTION_RECOVERY = 55,
	LOOP1_SMOKE_SENSOR_FAULT = 56,
	LOOP1_SMOKE_SENSOR_RECOVERY = 57,
	LOOP1_TEMP_WARNING_RECOVERY = 58,
	LOOP1_SMOKE_WARNING_RECOVERY = 59,
	RS485_CO_SENSOR_FAULT = 60,
	RS485_CO_SENSOR_RECOVERY = 61,
	RS485_H2_SENSOR_FAULT = 62,
	RS485_H2_SENSOR_RECOVERY = 63,
	RS485_VOC_SENSOR_FAULT = 64,
	RS485_VOC_SENSOR_RECOVERY = 65,
	RS485_CH4_SENSOR_FAULT = 66,
	RS485_CH4_SENSOR_RECOVERY = 67,
	
	// 2025/10/27 09:50 新增可燃气体报警区分
	FIRGAS_ALARM_CO = 38, // 一氧化碳
	FIRGAS_ALARM_HH = 39, // 氢气
	// XR5000_LOOP3_CHANGE_20260727: loop3 VOC/CH4 forewarn flash states.
	FIRGAS_ALARM_VOC = 40,
	FIRGAS_ALARM_CH4 = 41,
	
	

}FlashSaveType;

// 2025/11/19 14:04 现在占用6字节
// 2025/11/19 14:25 现在仍然占用5字节
// 2025/11/19 14:52 不使用该结构体作为时间存储变量了 使用新的数组作为存储参考
typedef struct
{
	uint8_t years;    // 报警年
	// 记录 2025/11/19 14:00 添加
	uint8_t months;     // 1-12月 (4位)
	uint8_t days  ;       // 1-31日 (5位)
	uint8_t hours ;      // 0-23时 (5位)
	uint8_t minute;     // 0-59分 (6位)
	uint8_t second;     // 0-59秒 (6位)
}FlashSaveTime_t;

// 定义时间类型为5字节数组
typedef uint8_t FlashSaveTimeBuff[5];

typedef struct
{
	uint8_t cluster_id; // 如果该ID为0，则表示存储的是仓
	uint8_t cabin_or_pack_id; // 如果cluster_id不为零则表示是pack
}FlashSaveDetectId_t;

typedef struct
{
	// 5字节时间 年月日 时分秒 合并不同位作为时间存储 
	FlashSaveTimeBuff fs_time_buff; // 5
	// 2字节探测器ID
	FlashSaveDetectId_t fs_detect_id; // 2
	// 1字节状态 
	uint8_t state; // 1
}FlashSaveBase_t;

typedef FlashSaveBase_t FlashSaveDetectFault_t; //探测器故障
typedef FlashSaveBase_t FlashSaveGasOutfires_t; //气灭故障功能
typedef FlashSaveBase_t FlashSaveOtherRecord_t; //其他故障

typedef struct
{
	FlashSaveBase_t fs_base;  // 8serial_ctrl_id
	uint16_t data_high; // 报警值记录 1
}FlashSaveFireAlarm_t;

typedef union
{
	FlashSaveDetectFault_t fs_sys_fault[500]; // 500*8 = 4000
	FlashSaveGasOutfires_t fs_gas_outfires[500];
	FlashSaveOtherRecord_t fs_other_record[500]; // 500条通用类型记录
	
	FlashSaveFireAlarm_t fs_fire_alarm[FLASH_ALARM_SUM_PER_SECTOR]; // 400条报警记录 400*9 = 3600
	
	uint8_t byte_buff[4096]; // 读取一个扇区的字节数
}FlashReadCache_t;

// 初始化
void BspFlashSaveInit(void);
// 读取Flash中保存的条数
int16_t getFlashSaveDataNummber(FlashReadCtrlId id);
// 保存记录条数到Flash中
void setFlashSaveDataNumber(FlashReadCtrlId id, uint16_t data_sum);
// 写入数据到FLASH中 该函数会自动偏移地址
void BspSaveCtrlInit(void); /* XR5000_FLASH_SERIALIZE_FIX_20260804 */
void BspSaveDataToFlash(FlashReadCtrlId id, FlashSaveType type, void *type_struct);
// 从FLASH 第x个数据扇区中读取数据 返回值条数 -1错误
int16_t BspReadFlashData(FlashReadCtrlId id, uint8_t *buff, uint8_t x_sector);
// DEBUG use
uint32_t getFlashSaveFrameId(void);
// 清除所有记录
void BspClearFlashData(void);

void BspCommonDataSaveApp(FlashReadCtrlId addr_type, FlashSaveType save_type, uint8_t cluster_id, uint8_t pack_or_cabin);

/*********************** 2025/11/19 14:52 ***************************/
// 配合新的变量FlashSaveTimeBuff使用
void setFlashTime(FlashSaveTimeBuff time, uint8_t year, uint8_t month, uint8_t day, 
                  uint8_t hour, uint8_t minute, uint8_t second);

// 配合新的变量FlashSaveTimeBuff使用
void getFlashTime(const FlashSaveTimeBuff time, uint8_t *year, uint8_t *month, uint8_t *day,
                  uint8_t *hour, uint8_t *minute, uint8_t *second);

void getFlashTime_Plus(const FlashSaveTimeBuff time, FlashSaveTime_t *time_entry);

#endif








