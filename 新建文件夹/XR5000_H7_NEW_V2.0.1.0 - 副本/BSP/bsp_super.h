#ifndef __BSP_SUPER_H
#define __BSP_SUPER_H

#include "main.h"

typedef struct
{
	uint8_t button_count;  // 按键按下计数
	uint8_t timout_count;  // 超时计数
	uint8_t press_flag;    // 按下标志位
	
}AdminButtonCtrl_t;

typedef struct
{
	uint16_t hydrogen_warn_1;           // 氢气一级预警阈值
	uint16_t hydrogen_warn_2;           // 氢气二级预警阈值

	uint16_t carbon_mono_warn_1;        // 一氧化碳一级预警阈值
	uint16_t carbon_mono_warn_2;        // 一氧化碳二级预警阈值
	
	uint16_t temperature_warn_1;        // 温度一级预警阈值
	uint16_t temperature_warn_2;        // 温度二级预警阈值
	
	float outfire_device1_pressure;     // 灭火装置1标称压力值
	float device1_pressure_uplimit;     // 灭火装置1超压压力值
	float device1_pressure_lowlimit;    // 灭火装置1低压压力值
	
	float outfire_device2_pressure;     // 灭火装置2标称压力值
	float device2_pressure_uplimit;     // 灭火装置2超压压力值
	float device2_pressure_lowlimit;    // 灭火装置2低压压力值
	
}ThresholdSeting_t;

extern AdminButtonCtrl_t button_ctrl;
extern uint32_t super_admin_password;

extern ThresholdSeting_t fire_alarm_threshold;

void SuperAdminButtonCtrl(uint16_t screen_id, uint16_t control_id, uint8_t  state, AdminButtonCtrl_t *abc);
void SuperAdminPasswordButtonCtrl(uint16_t screen_id, uint16_t control_id, uint8_t  state, uint32_t *password);
void SuperAdminInternalScreenTextCtrl(uint16_t screen_id, uint16_t control_id, uint8_t *str, uint32_t *password);

void FireAlarmThresholdSettingInternalScreenText(uint16_t screen_id, uint16_t control_id, uint8_t *str, ThresholdSeting_t* ts);

void OutfirePressureUpdataUI(uint16_t curr_id, float pressure1, float pressure2, ThresholdSeting_t ts);

void FireAlarmThresholdUpdataUI(uint16_t _screen_id, ThresholdSeting_t ts);

#endif
