#ifndef __BSP_INTERNAL_BOARD_H
#define __BSP_INTERNAL_BOARD_H

#include "main.h"

#include "usart.h"
#include "bsp_itcallback.h"

typedef enum
{
	KEY_MANUAL = 0x01,
	KEY_AUTO   = 0x02,
}SYS_PARTx_HAND_AUTO_STATE;

typedef enum
{
	SELFCHECK_KEY   = 1,
	SILENSE_KEY     = 2,
	RESET_KEY       = 3,
	CHECK_KEY       = 4,
	MODIFY_TIME_KEY = 5,  
	SIREN_KEY       = 6,
	
	LINKAGE_START_KEY = 7, // 外联设备启动按键
	
	PART1_SPRY_START  = 8,
	PART2_SPRY_START  = 9, // 
	
	DEVICE_CTRL_KEY = 10,
	
	SIMU_SERIAL_PORT = 11,
	
	LINKAGE_PROGREM = 12,
	DEVICE_SHIELD_KEY = 13, // XR5000_DEVICE_SHIELD_ENTRY_20260802: 设备屏蔽密码入口
	
	NONE_KEY      = 255
}InternalKeyPressValue;

// 矩阵按键键值定义
typedef enum
{
	KEY1_INFORM_CERTAIN  = 1,   // 信息确认
	KEY2_SELF_INSPECTION = 2,   // 系统自检
	KEY3_SYSTEM_SILENCE  = 3,   // 系统消音
	KEY4_SYSTEM_RESET    = 4,   // 系统复位
	KEY5_SYSTEM_CHECK    = 5,   // 系统检查
	KEY6_DIRECTION_UP    = 6,   // 方向键上
	KEY7_DIRECTION_RIGHT = 7,   // 方向键右
	KEY8_DIRECTION_DOWN  = 8,   // 方向键下
	KEY9_DIRECTION_LEFT  = 9,   // 方向键左
	KEY10_DIRECTION_OK   = 10,  // 方向键ok
	KEY11_PART1_SOUNDLT  = 11,  // 分区1声光
	KEY12_PART1_STOP     = 12,  // 分区1停止
	KEY13_PART2_SOUNDLT  = 13,  // 分区2声光
	KEY14_PART2_STOP     = 14,  // 分区2停止
	KEY15_PART1_SPRAY_ST = 15,  // 分区1喷洒启动
	KEY16_PART2_SPRAY_ST = 16,  // 分区2喷洒启动
	
	NO_MATRIX_KEY_PRESS = 0XFF,
}BspMatrixKeyPressValue;

typedef enum
{
	LED_OFF = 0,
	LED_ON = 1,
}LED_STATE;

extern void StartupLinkageDevice(void);

void SysStartStateLedCtrl(LED_STATE state);

// 系统主面板报警器故障LED控制
void SysSirenStartLedCtrl(LED_STATE state);
void SysSirenFaultLedCtrl(LED_STATE state);

// 分区1 启动LED 控制
void Part1StartLedCtrl(LED_STATE state);
void Part1StartDelayLedCtrl(LED_STATE state);
void Part1SprayLedCtrl(LED_STATE state);
void Part1FeedbackLedCtrl(LED_STATE state);
void Part1SoundLightLedCtrl(LED_STATE state);
void Part1FaultLedCtrl(LED_STATE state);

// 分区2 LED控制 2025/10/28 10:11
void Part2StartLedCtrl(LED_STATE state);
void Part2StartDelayLedCtrl(LED_STATE state);
void Part2SprayLedCtrl(LED_STATE state);
void Part2FeedbackLedCtrl(LED_STATE state);
void Part2SoundLightLedCtrl(LED_STATE state);
void Part2FaultLedCtrl(LED_STATE state);

// 备用区 2025/12/11 16:39
void SpecialSelfCheckLedCtrl(LED_STATE state);

// 新增可燃气体LED
void SpareGasAlarmLedCtrl(LED_STATE state);

void SpecialCommunicLedCtrl(LED_STATE state);

void HostUploadDataTask(void *parameter);
void InternalScreenBoradRecvDealTask(void * parameter);

uint8_t getKeyPressValue(void);
void setKeyValue(InternalKeyPressValue val);

uint8_t getDirectionKeyValue(void);
void clearMatrixKeyValue(void);

uint8_t getOutFireKeyalue(void);
void clearOutFireKeyValue(void);

uint8_t getPart1HandAutoState(void);
uint8_t getSysHandAutoState(void);
uint8_t getPart2HandAutoState(void);

// 自检状态
uint8_t getControllorSelfCheckState(void);

#endif
