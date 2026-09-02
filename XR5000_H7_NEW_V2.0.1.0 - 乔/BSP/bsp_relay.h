#ifndef __BSP_RELAY_H
#define __BSP_RELAY_H

#include "main.h"

// 火警继电器控制  XR5000对应NO1 1
#define FireAlarmP(X)  ((X)==1)?(JDQ1P_GPIO_Port->BSRR = JDQ1P_Pin):(JDQ1P_GPIO_Port->BSRR = (uint32_t)JDQ1P_Pin<<16U)
// XR5000_AHT20_CHANGE_20260727: PD9 is AHTSDA now; keep relay call sites from touching it.
#define FireAlarmN(X)  do { (void)(X); } while(0)
// 预警继电器控制  XR5000对应NO2 2
// XR5000_AHT20_CHANGE_20260727: PD10 is AHTSCL now; keep relay call sites from touching it.
#define ForeWarmP(X)   do { (void)(X); } while(0)
#define ForeWarmN(X)   ((X)==1)?(JDQ2N_GPIO_Port->BSRR = JDQ2N_Pin):(JDQ2N_GPIO_Port->BSRR = (uint32_t)JDQ2N_Pin<<16U)
// 故障继电器控制  XR5000对应NO3 3
#define FaultP(X)      ((X)==1)?(JDQ3P_GPIO_Port->BSRR = JDQ3P_Pin):(JDQ3P_GPIO_Port->BSRR = (uint32_t)JDQ3P_Pin<<16U)
#define FaultN(X)      ((X)==1)?(JDQ3N_GPIO_Port->BSRR = JDQ3N_Pin):(JDQ3N_GPIO_Port->BSRR = (uint32_t)JDQ3N_Pin<<16U)
// 放气继电器控制  XR5000对应OUT1 4
#define DeflateP(X)    ((X)==1)?(JDQ4P_GPIO_Port->BSRR = JDQ4P_Pin):(JDQ4P_GPIO_Port->BSRR = (uint32_t)JDQ4P_Pin<<16U)
#define DeflateN(X)    ((X)==1)?(JDQ4N_GPIO_Port->BSRR = JDQ4N_Pin):(JDQ4N_GPIO_Port->BSRR = (uint32_t)JDQ4N_Pin<<16U)
// 声光继电器控制  XR5000对应OUT2 5
#define SoundLightP(X) ((X)==1)?(JDQ5P_GPIO_Port->BSRR = JDQ5P_Pin):(JDQ5P_GPIO_Port->BSRR = (uint32_t)JDQ5P_Pin<<16U)
#define SoundLightN(X) ((X)==1)?(JDQ5N_GPIO_Port->BSRR = JDQ5N_Pin):(JDQ5N_GPIO_Port->BSRR = (uint32_t)JDQ5N_Pin<<16U)
// 警笛继电器控制  XR5000对应OUT3 6
#define SirenP(X)      ((X)==1)?(JDQ6P_GPIO_Port->BSRR = JDQ6P_Pin):(JDQ6P_GPIO_Port->BSRR = (uint32_t)JDQ6P_Pin<<16U)
#define SirenN(X)      ((X)==1)?(JDQ6N_GPIO_Port->BSRR = JDQ6N_Pin):(JDQ6N_GPIO_Port->BSRR = (uint32_t)JDQ6N_Pin<<16U)
// 灭火2继电器控制 XR5000对应OUT4 7
#define OutFire2P(X)    ((X)==1)?(JDQ7P_GPIO_Port->BSRR = JDQ7P_Pin):(JDQ7P_GPIO_Port->BSRR = (uint32_t)JDQ7P_Pin<<16U)
#define OutFire2N(X)    ((X)==1)?(JDQ7N_GPIO_Port->BSRR = JDQ7N_Pin):(JDQ7N_GPIO_Port->BSRR = (uint32_t)JDQ7N_Pin<<16U)
// 灭火1继电器控制 XR5000对应OUT5 8
#define OutFire1P(X)    ((X)==1)?(JDQ8P_GPIO_Port->BSRR = JDQ8P_Pin):(JDQ8P_GPIO_Port->BSRR = (uint32_t)JDQ8P_Pin<<16U)
#define OutFire1N(X)    ((X)==1)?(JDQ8N_GPIO_Port->BSRR = JDQ8N_Pin):(JDQ8N_GPIO_Port->BSRR = (uint32_t)JDQ8N_Pin<<16U)
// 仓喷继电器控制  XR5000对应OUT6 9
#define CabinSprayP(X)  ((X)==1)?(JDQ9P_GPIO_Port->BSRR = JDQ9P_Pin):(JDQ9P_GPIO_Port->BSRR = (uint32_t)JDQ8P_Pin<<16U)
#define CabinSprayN(X)  ((X)==1)?(JDQ9N_GPIO_Port->BSRR = JDQ9N_Pin):(JDQ9N_GPIO_Port->BSRR = (uint32_t)JDQ8N_Pin<<16U)

// 电池充电继电器控制
#define ChargeON()      (BATCD_GPIO_Port->BSRR = BATCD_Pin)
#define ChargeOFF()     (BATCD_GPIO_Port->BSRR = (uint32_t)BATCD_Pin<<16U)                                                 
// 升压电路继电器控制
#define BoostON()       (ZBDQH_GPIO_Port->BSRR = ZBDQH_Pin)
#define BoostOFF()      (ZBDQH_GPIO_Port->BSRR = (uint32_t)ZBDQH_Pin<<16U)   


typedef enum
{
  JDQ_OFF = 0U,
  JDQ_ON
}RelayState;

typedef void (*relay_ctrl_fun)(RelayState state);

typedef struct
{
	RelayState curr_relay_state;
	relay_ctrl_fun call_back_fun;
}RelayCtrlStateRecord;

void FireAlarmRelayCtrl(RelayState state);
void ForeWarmRelayCtrl(RelayState state);
void FaultRelayCtrl(RelayState state);

void DefauleRelayCtrl(RelayState state);
RelayState FetchDefauleRelayState(void);

void SoundLightRelayCtrl(RelayState state);
RelayState FetchSoundLightRelayState(void);

void SirenRelayCtrl(RelayState state);
RelayState FetchSirenRelayState(void);

void OutFire2RelayCtrl(RelayState state);
RelayState FetchOutFire2RelayState(void);

void OutFire1RelayCtrl(RelayState state);
RelayState FetchOutFire1RelayState(void);

void CabinSprayRelayCtrl(RelayState state);
RelayState FetchCabinSprayRelayState(void);

void BatteryBoostRelayCtrl(RelayState state);
RelayState FetchBatteryChargeRelayState(void);

void BatteryChargeRelayCtrl(RelayState state);

#endif
