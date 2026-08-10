#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"

#define ADC_CARRY_NUM 140

#define SolenoidValveDisconnectThreshold 2150
#define SolenoidValveOnlineThreshold 750
#define SolenoidValveShortCircuitThreshold 30

#define DefauleDisconnectThreshold 2250
#define DefauleOnlineThreshold 1700
#define DefauleShortCircuitThreshold 30

#define SoundLightDisconnectThreshold 2250
#define SoundLightOnlineThreshold 1700
#define SoundLightShortCircuitThreshold 30

// ADC通道对应关系枚举类型
typedef enum
{
	Pressure1AdcSite   = 0,   // 压力检测1adc通道
	OutFire1AdcSite    = 1,   // 灭火电磁阀1adc通道 
	CabinSprayAdcSite  = 2,   // 仓喷电磁阀adc通道
	DefauleAdcSite     = 3,   // 放气adc通道
	Pressure2AdcSite   = 4,   // 压力检测2adc通道
	OutFire2AdcSite    = 5,   // 灭火电磁阀2adc通道
	SirenAdcSite       = 6,   // 警笛adc通道
	SoundLightAdcSite  = 7,   // 声光adc通道
	PowerAdcSite       = 8,   // 主电adc通道(检测主电是否存在)
	BattryAdcSite      = 9,   // 电池adc通道(检测备电电池是否存在)
	BatBoostPwrAdcSite = 10,  // 电池升压adc通道
	PwrOutVotAdcSite   = 11,  // 24V输出adc通道
	BatCurrentAdcSite  = 12,  // 电池充电/放电电流adc通道
	PwrOutCurrAdcSite  = 13,  // 主机输出电流adc通道 
}AdcChannelSite;

// 电池状态枚举类型
typedef enum
{
	normal_charge = 0,
	float_charge,
	short_circuit,
	open_circuit,
	discharge,
	undervoltage,
	overcharge,
	bat_judge
}bat_state;

extern uint32_t ADC_DMA_BUFF[140];

extern float outfire1_pressure;
extern float outfire2_pressure;

void LinkageDeviceStateInit(void);

uint16_t CaculateAdcSmoothValue(uint32_t arr[], AdcChannelSite adc_channel);

void LinkageOnlineJudgeTask(void* parameter);

void PowerOnlineJudgeTask(void* parameter);

#endif
