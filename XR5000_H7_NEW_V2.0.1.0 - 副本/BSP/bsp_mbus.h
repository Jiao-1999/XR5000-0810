#ifndef __BSP_MBUS_H
#define __BSP_MBUS_H

#include "main.h"
#include "usart.h"
#include "bsp_itcallback.h"
#include "system.h"

#define MIXTURE_DEVICE_SUM 201
#define MIXTURE_DEVICE_MAX_ADDR 100
#define MIXTURE_DEVICE_FLASH_ADDR 0x111000UL
#define MIXTURE_DEVICE_FLASH_DATA_LEN (MIXTURE_DEVICE_MAX_ADDR + 1U)
#define MIXTURE_DEVICE_DISCONNECT_SUM 3
#define MIXTURE_DEVICE_RESPONSE_TIMEOUT_MS 80U
#define MIXTURE_DEVICE_IDENTIFY_RESPONSE_TIMEOUT_MS 200U
#define MIXTURE_DEVICE_TASK_INTERVAL_MS 10U

/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: timeout confirmation is owned by the UART7 transaction. */

typedef enum
{
	PointType_8001_AI = 1,
	PointType_800C    = 2,
	
	PointType_Error = 255,
}ePointTypeDetectName;

typedef enum
{
	PointType_Smoke  = 1,
	PointType_Temper = 2,
	
}ePointTypeDetectType;

typedef enum
{
	PointTypeData_Temper = 0,
	
	PointTypeData_Smoke  = 1,
	
	
}ePointTypeDataOrder;


typedef enum
{
	PointTypeDetectorDisconnectBit = 0,
	PointTypeDetectorTempertureBit = 1,
	PointTypeDetectorSmokeBit = 2,

}ePointTypeDetectorStateBitCtrl;


void SavePointTypeSetOnlieState(void);
void ReadPointTypeSetOnlieState(void);


void MixtureDevicePollingManage(void);

// 获取该ID是否设为上线
uint8_t getPointTypeMixtureSettingOnlieState(uint8_t detector_id);
// 判断探测器是否掉线
uint8_t getPointTypeMixtureDisconnectCount(uint8_t point_mix_id);
// 设定单一探测器上线
void PointTypeMixtureOnlieStateSingleSetting(uint8_t detector_id, uint8_t online_or_offline);
// 批量设置探测器上线
void PointTypeMixtureOnlieStateBatchSetting(uint8_t *new_online_state, uint8_t update_len);
// 下线全部探测器
void PointTypeMixtureOnlieStateDeInit(void);

// 清空掉线计数
void clearPointTypeMixtureDisconnectCount(void);

// 获取接收到的值
uint8_t getPointTypeMixtureReceiveData(ePointTypeDataOrder detect_data_type, uint8_t detect_id);
// 获取接收到的状态
uint8_t getPointTypeMixtureReceiveState(ePointTypeDataOrder detect_data_type, uint8_t detect_id);

// 获取探测器型号名
uint8_t getPointTypeMixtureDetectName(uint8_t detect_id);
uint16_t MBus1_GetNationalTypeCode(uint8_t addr); /* 获取设备实际返回并保存的0x000D国标类型码 */
uint16_t MBus1_GetProductCode(uint8_t addr); /* 获取设备实际返回并保存的0x000E内部产品码 */
// 获取探测器监测类型
uint8_t getPointTypeMixtureDetectType(uint8_t detect_id);
// 获取探测器上线状态
uint8_t getPointTypeMixtureDetectOnlineState(uint8_t detect_id);

// 设置探测器掉线记忆 以空间换时间
void setPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id, uint8_t state);
// 获取探测器掉线状态
uint8_t getPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id);

// 获取温度传感器报警历史状态
uint8_t getPointTypeMixtureDetectTempertureMemory(uint8_t detect_id);
// 温度传感器历史报警状态
void setPointTypeMixtureDetectTempertureMemory(uint8_t detect_id, uint8_t state);

// 获取烟雾传感器报警历史状态
uint8_t getPointTypeMixtureDetectSmokeMemory(uint8_t detect_id);
// 烟雾传感器历史报警状态
void setPointTypeMixtureDetectSmokeMemory(uint8_t detect_id, uint8_t state);

// 初始化/清空掉线状态
void clearPointTypeMixtureDetectAllStateMemory(void);


void MBus1SendString(uint8_t* buf, uint8_t len);
void MBus1ResetAllDevices(void);
void MBus2SendString(uint8_t* buf, uint8_t len);
uint16_t getPointTypeMixtureReceiveData16(ePointTypeDataOrder detect_data_type, uint8_t detect_id);
uint8_t getPointTypeMixtureStateClass(uint8_t detect_id);
// 获取探测器国标设备类型码(供联动逻辑显示用)
uint16_t getPointTypeMixtureNationalCode(uint8_t detector_id);

// 总线轮询同时处理接收任务
void MBus1PollSlaveAndReceiveTask(void* parameter);


#endif












