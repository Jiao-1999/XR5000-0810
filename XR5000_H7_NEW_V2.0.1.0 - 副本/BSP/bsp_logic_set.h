#ifndef __BSP_LOGIC_SET_H
#define __BSP_LOGIC_SET_H

#include "main.h"

typedef enum
{
	independent = 0x20,
	cooperative = 0x21,
}OutFireWorkType;

typedef enum
{
	outfire1_cluster = 0x22, // 1 对应到簇
	outfire1_cabin   = 0x23, // 1 对应到舱
	
	outfire2_cluster = 0x24, // 2 对应到簇
	outfire2_cabin   = 0x25, // 2 对应到簇
}OutFireCorrespondence;

typedef enum
{
	dry_contact_open  = 0x26,
	dry_contact_close = 0x27,
	pressure_sensor   = 0x28,
}PressureDetectType;

typedef enum
{
	hydrogen    = 0xFC,
	carbon      = 0xFD,
	smoke       = 0xFE,
	temperature = 0xFF
}LogicParameterType;

typedef enum
{
	fire_alarm = 0xFA,
	normal     = 0xFB,
}CabinJudgeState;

typedef struct
{
	uint8_t out_fire_device_sum;        // 灭火装置数量
	uint8_t device_work_method;         // 灭火装置工作方式
	uint8_t device_start_interval;      // 灭火装置启动间隔
	uint8_t device1_correspondence;     // 灭火装置1对应关系
	uint8_t device2_correspondence;     // 灭火装置2对应关系
	uint8_t pressure_detection_type;    // 压力检测类型
	
	uint8_t cluster_start_delay;        // 簇启动延时
	uint8_t cluster_spray_number;       // 簇喷放次数
	uint8_t cluster_spray_buff[5];      // 簇喷放时长 间隔时长 存储区
	
	uint8_t cabin_start_delay;          // 舱启动延时
	uint8_t cabin_spray_number;         // 舱喷放次数
	uint8_t cabin_spray_buff[5];        // 舱喷放时长 间隔时长 存储区
	
}OutFireStartLogic_t; // 灭火装置启动逻辑

typedef struct
{
	uint8_t fire_alarm_logic_buff[256]; // 火警触发逻辑数组 用来储存屏幕传过来的火警逻辑判定规则
	uint8_t buff_top;                   // 栈顶指针
	uint8_t buff_bottom;                // 栈尾指针
	uint8_t buff_pointer;               // 数组指针
	uint8_t logic_modify_flag;
	
}FireAlarmLogic_t; // 火警逻辑判定



typedef struct
{
	uint8_t detect_number;              // 探测器编号
	uint8_t alarm_type;                 // 报警类型
	
}FireAlarmJudge_t; // 用来提取判定火警的逻辑 并

typedef struct
{
	uint8_t detector_type;              // 探测器类型
	
	uint8_t hydrogen_state;             // 氢气状态
	uint8_t carbon_state;								// 一氧化碳状态
	uint8_t smoke_state;                // 烟雾状态
	uint8_t temperature_state;          // 温度状态
	
}CabinDetectorState;

extern OutFireStartLogic_t out_fire_start_ctrl;          // 灭火装置启动逻辑控制
extern FireAlarmLogic_t fire_alarm_logic_ctrl;           // 用户输入火警触发逻辑
extern FireAlarmJudge_t fire_alarm_judge[64][64];        // 程序提取用户输入的火警触发逻辑中的有效信息 为判断做准备
extern CabinDetectorState cabin_detector_state_buff[30]; // 仓状态结构体数组


extern CabinJudgeState cabin_fire_alarm_state; // 火警状态

void OutFireDeviceInternalScreenButtonSet(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state, OutFireStartLogic_t* ofsl);
void OutFireDeviceInternalScreenTexttSet(uint16_t screen_id, uint16_t control_id, uint8_t *str, OutFireStartLogic_t* ofsl);
void OutFireDeviceInternalScreenUpdataUI(uint16_t _screen_id, OutFireStartLogic_t ofsl);

void FireAlarmTriggerLogicButtonSet(uint16_t screen_id, uint16_t control_id, uint8_t  state, FireAlarmLogic_t* fals);
void FireAlarmTriggerLogicUpdataUI(uint16_t _screen_id, FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64]);

void FireAlarmJudgeBuffExtract(FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64]);

CabinJudgeState FireAlarmCompoundLogicJudgement(FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64], CabinDetectorState cds[]);

#endif
