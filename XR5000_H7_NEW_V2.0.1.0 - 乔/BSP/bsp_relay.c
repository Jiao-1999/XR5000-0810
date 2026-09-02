#include "bsp_relay.h"

#include "bsp_debug.h"

// 火警继电器控制
void FireAlarmRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		FireAlarmP(0);
		FireAlarmN(1); // 打开继电器
	}
	else
	{
		FireAlarmP(1);
		FireAlarmN(0); // 关闭继电器
	}
}

// 预警继电器控制
void ForeWarmRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		ForeWarmP(0);
		ForeWarmN(1); // 打开继电器
	}
	else
	{
		ForeWarmP(1);
		ForeWarmN(0); // 关闭继电器
	}
}

// 故障继电器控制
void FaultRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		FaultP(0);
		FaultN(1); // 打开继电器
	}
	else
	{
		FaultP(1);
		FaultN(0); // 关闭继电器
	}
}

// 放气继电器控制
void DefauleRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		DeflateP(0);
		DeflateN(1); // 打开继电器
	}
	else
	{
		DeflateP(1);
		DeflateN(0); // 关闭继电器
	}
}

// 获取放气继电器当前吸合状态
RelayState FetchDefauleRelayState(void)
{
	if( (JDQ4P_GPIO_Port->IDR & JDQ4P_Pin) == GPIO_PIN_RESET && (JDQ4N_GPIO_Port->IDR & JDQ4N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}

// 声光继电器控制
void SoundLightRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		SoundLightP(0);
		SoundLightN(1); // 打开继电器
	}
	else
	{
		SoundLightP(1);
		SoundLightN(0); // 关闭继电器
	}
}

// 获取声光继电器当前吸合状态
RelayState FetchSoundLightRelayState(void)
{
	if( (JDQ5P_GPIO_Port->IDR & JDQ5P_Pin) == GPIO_PIN_RESET && (JDQ5N_GPIO_Port->IDR & JDQ5N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}

// 警笛继电器控制
void SirenRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		SirenP(0);
		SirenN(1); // 打开继电器
	}
	else
	{
		SirenP(1);
		SirenN(0); // 关闭继电器
	}
}

// 获取警笛继电器当前吸合状态
RelayState FetchSirenRelayState(void)
{
	if( (JDQ6P_GPIO_Port->IDR & JDQ6P_Pin) == GPIO_PIN_RESET && (JDQ6N_GPIO_Port->IDR & JDQ6N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}


// 灭火2继电器控制
void OutFire2RelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		OutFire2P(0);
		OutFire2N(1); // 打开继电器
	}
	else
	{
		OutFire2P(1);
		OutFire2N(0); // 关闭继电器
	}
}
	
// 获取灭火装置2继电器当前吸合状态
RelayState FetchOutFire2RelayState(void)
{
	if( (JDQ7P_GPIO_Port->IDR & JDQ7P_Pin) == GPIO_PIN_RESET && (JDQ7N_GPIO_Port->IDR & JDQ7N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}

// 灭火1继电器控制
void OutFire1RelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		OutFire1P(0);
		OutFire1N(1); // 打开继电器
	}
	else
	{
		OutFire1P(1);
		OutFire1N(0); // 关闭继电器
	}
}

// 获取灭火装置1继电器当前吸合状态
RelayState FetchOutFire1RelayState(void)
{
	if( (JDQ8P_GPIO_Port->IDR & JDQ8P_Pin) == GPIO_PIN_RESET && (JDQ8N_GPIO_Port->IDR & JDQ8N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}

// 仓喷继电器控制
void CabinSprayRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		CabinSprayP(0);
		CabinSprayN(1); // 打开继电器
	}
	else
	{
		CabinSprayP(1);
		CabinSprayN(0); // 关闭继电器
	}
}

// 获取仓喷继电器当前吸合状态
RelayState FetchCabinSprayRelayState(void)
{
	if( (JDQ9P_GPIO_Port->IDR & JDQ9P_Pin) == GPIO_PIN_RESET && (JDQ9N_GPIO_Port->IDR & JDQ9N_Pin) != GPIO_PIN_RESET) {
		return JDQ_ON;
	}
	return JDQ_OFF;
}

// 电池充电继电器控制
void BatteryChargeRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		ChargeON();
	}
	else
	{
		ChargeOFF();
	}
}

// 获取电池充电控制继电器状态
RelayState FetchBatteryChargeRelayState(void)
{
	return (BATCD_GPIO_Port->IDR & BATCD_Pin) ? JDQ_ON : JDQ_OFF;
}

// 电池升压继电器控制
void BatteryBoostRelayCtrl(RelayState state)
{
	if(state == JDQ_ON)
	{
		BoostON();
	}
	else
	{
		BoostOFF();
	}
}



