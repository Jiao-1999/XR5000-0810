#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "main.h"

// 设置变量的某一位为1
#define my_set_x_bit(var, bit) ((var) |= (1 << (bit)))

// 清除变量的某一位（设为0）
#define my_clear_x_bit(var, bit) ((var) &= ~(1 << (bit)))

// 切换变量的某一位（如果是1则变0，反之亦然）
#define my_toggle_x_bit(var, bit) ((var) ^= (1 << (bit)))

// 检查变量的某一位是否为1
#define my_check_x_bit(var, bit) ((var) & (1 << (bit)))

typedef struct
{
	uint32_t user_password;     // 用户密码
	
	uint16_t can2_slave_addr;      // CAN设备地址   
	uint16_t slave_state;         // 存储状态，如果不等于特定值5A,证明还没有写入过信息  
	
	uint16_t slave_baud_rate_Station;// 波特率   
	uint16_t slave_baud_rate_EMS;    // 波特率   
	
 	uint8_t slave_addr485_Station;   // 设备地址  
	uint8_t slave_addr485_EMS;       // 设备地址  
	uint8_t	factory_release_year;			//出厂日期年    
	uint8_t	factory_release_month;		//出厂日期月
	
	uint8_t	factory_release_days;			//出厂日期日   
	uint8_t system_hand_or_auto_state;
	uint8_t part1_hand_or_auto_state;
	uint8_t part2_hand_or_auto_state;

	float out_fire_pressure1_high;
	float out_fire_pressure1_low;
	
	float out_fire_pressure2_high;
	float out_fire_pressure2_low;
	
	uint16_t license_remain_day;
	
	uint8_t curr_license_store[11];
	uint8_t last_license_store[11];
	uint8_t pref_license_store[11];
	
	uint8_t license_year;
	uint8_t license_month;
	uint8_t license_days;
	uint8_t license_hour;
	
	uint8_t license_minute;
	uint8_t license_second;
	
}SystemSaveInfo_t;

typedef enum
{
	LinkageShield = 0x10,  // 屏蔽状态
	LinkageOpen,           // 启用状态
	LinkageDisconnect,     // 掉线状态
	LinkageFault,          // 故障状态
	LinkageOnline,         // 在线状态
	LinkageShortCircuit,   // 短路状态
	
	LinkageWorking         // 继电器吸合 工作状态
}LinkageState;

typedef enum
{
	Defaule_Number    = 44U,
	SoundLight_Number = 45U,
	Siren_Number      = 46U,
	Outfire1_Number   = 47U,
	Outfire2_Number   = 48U,
	CabinSpray_Number = 49U,
	FeedBack1_Number  = 50U,
	FeedBack2_Number  = 51U,
	HandReport_Number = 52U,
	
}eSystemNumberTable; // 该枚举类型列举的探测器 以及外联设备的ID号 用来记录和显示报警记录

typedef enum
{
	LinkageDisconnectType = 52U,   // 联动设备掉线
	LinkageShortType      = 53U,   // 联动设备短路
	
	
	
	
}eSystemFaultType;


extern uint8_t tanceqiming[20][20];//XR805备命名

extern uint8_t cang_sxzt[30];           		 // 舱上线状态
extern uint8_t cang_pbzt[30];           		 // 舱屏蔽状态
extern uint8_t cu_sxzt[30];           		 // 簇上线状态
extern uint8_t cu_tcq_sxzt[30];           		 // 簇下挂探测器上线状态
extern uint8_t pack_pbzt[30][33];           		 // pack屏蔽状态

extern uint8_t pack_circuit; // 用来控制当前显示的回路

extern uint8_t pack_online_buff[4][33];

extern uint8_t linkage_shield_state[40];        // 外联设备状态（默认屏蔽）
extern uint8_t linkage_work_state[40];					// 外联设备工作状态（默认屏蔽）

extern SystemSaveInfo_t SystemSaveInfo;

void SystemInfoSave(void);
void SystemInfoLoad(void);
void Load25Q128(void);
uint16_t CalcCrc16(uint8_t *buf, uint16_t len);

void Savetanceqiming(uint8_t *buf, uint8_t tcqdrr);

void Save_cang_sxzt(void);//存储舱上线状态
void Loa_cang_sxzt(void);//读取舱上线状态
void Save_cu_sxzt(void);//存储簇上线状态
void Loa_cu_sxzt(void);//读取簇上线状态
void Save_cutcq_sxzt(void);//存储探测器上线状态
void Loa_cutcq_sxzt(void);//读取探测器上线状态
void Save_cang_pbzt(void);//存储舱屏蔽状态
void Loa_cang_pbzt(void);//读取舱屏蔽状态
void Save_pack_pbzt(void);//存储pcak屏蔽状态
void Loa_pack_pbzt(void);//读取pack屏蔽状态

void SaveLinkageSheildState(void);
void LoadLinkageSheildState(void);

void Save_Pack_Set_Online_State(void);
void Load_Pack_Set_Online_State(void);

void SystemDebugTest(void);
uint8_t SystemQuerySetOnlineNum(uint8_t* target, uint8_t* outcome, uint8_t outcome_size);

#endif // SYSTEM_H
