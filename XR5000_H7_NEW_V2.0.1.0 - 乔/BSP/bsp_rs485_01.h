#ifndef __BSP_RS485_01_H
#define __BSP_RS485_01_H

#include "main.h"
#include "usart.h"
#include "bsp_itcallback.h"
#include "system.h"

#define PACK_SLAVE_NUM 30
#define PACK_USER_SLAVE_NUM 20

#define CabinDisconnectCount 5
#define PackDisconnectCount  10

typedef enum
{
	PACK_VALVE1 = 0X01U,
	PACK_VALVE2 = 0X02U,
	PACK_VALVE3 = 0X03U,
	PACK_VALVE4 = 0X04U,
	PACK_VALVE5 = 0X05U,
	PACK_VALVE6 = 0X06U,
	PACK_VALVE7 = 0X07U,
	PACK_VALVE8 = 0X08U,
	PACK_VALVE9 = 0X09U,
	PACK_VALVEA = 0X0AU,

	CLUSTER_VALVE1 = 0X28U,
	CLUSTER_VALVE2 = 0X29U,
}ClusterCabinRegAddr;

void PackBehaviorManageSendString(uint8_t* buf, uint8_t len);

void FansPollSlaveTESK(void* parameter);

void PackPollSlaveTesk(void* parameter);

void XR805PollSlaveTESK(void* parameter);
void QueuePollSlaveTesk(void* parameter);

void QueuePollRecvDealTask(void *parameter);

void ResetAllBusDevice(void);

void PollPackManager(void);
void PollXR805Manager(void);
void PollFanManager(void);
void PollPackManager_Plus(void);
void PollPackManager_Ultra(void);

void BSP_RS485_01_Poll(void* parameter);

uint8_t getClusterPackDisconnectCount(uint8_t cluster_id, uint8_t pack_id);
void clearClusterPackDisconnectCount(uint8_t cluster_id, uint8_t pack_id);

void ClusterOrCabinCtrlCmd(uint8_t detector_id, uint8_t reg_addr, uint8_t turn_on_off);

uint16_t getPackCoConcenValue(uint8_t cluster_id, uint8_t pack_id);

void RS485_01_PollAndRecieve(void * parameter);

#endif
