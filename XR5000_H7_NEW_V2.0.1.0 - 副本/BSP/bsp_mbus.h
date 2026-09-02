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

// ��ȡ��ID�Ƿ���Ϊ����
uint8_t getPointTypeMixtureSettingOnlieState(uint8_t detector_id);
// �ж�̽�����Ƿ����
uint8_t getPointTypeMixtureDisconnectCount(uint8_t point_mix_id);
// �趨��һ̽��������
void PointTypeMixtureOnlieStateSingleSetting(uint8_t detector_id, uint8_t online_or_offline);
// ��������̽��������
void PointTypeMixtureOnlieStateBatchSetting(uint8_t *new_online_state, uint8_t update_len);
// ����ȫ��̽����
void PointTypeMixtureOnlieStateDeInit(void);

// ��յ��߼���
void clearPointTypeMixtureDisconnectCount(void);

// ��ȡ���յ���ֵ
uint8_t getPointTypeMixtureReceiveData(ePointTypeDataOrder detect_data_type, uint8_t detect_id);
// ��ȡ���յ���״̬
uint8_t getPointTypeMixtureReceiveState(ePointTypeDataOrder detect_data_type, uint8_t detect_id);

// ��ȡ̽�����ͺ���
uint8_t getPointTypeMixtureDetectName(uint8_t detect_id);
uint16_t MBus1_GetNationalTypeCode(uint8_t addr); /* ��ȡ�豸ʵ�ʷ��ز������0x000D���������� */
uint16_t MBus1_GetProductCode(uint8_t addr); /* ��ȡ�豸ʵ�ʷ��ز������0x000E�ڲ���Ʒ�� */
// ��ȡ̽�����������
uint8_t getPointTypeMixtureDetectType(uint8_t detect_id);
// ��ȡ̽��������״̬
uint8_t getPointTypeMixtureDetectOnlineState(uint8_t detect_id);

// ����̽�������߼��� �Կռ任ʱ��
void setPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id, uint8_t state);
// ��ȡ̽��������״̬
uint8_t getPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id);

// ��ȡ�¶ȴ�����������ʷ״̬
uint8_t getPointTypeMixtureDetectTempertureMemory(uint8_t detect_id);
// �¶ȴ�������ʷ����״̬
void setPointTypeMixtureDetectTempertureMemory(uint8_t detect_id, uint8_t state);

// ��ȡ����������������ʷ״̬
uint8_t getPointTypeMixtureDetectSmokeMemory(uint8_t detect_id);
// ������������ʷ����״̬
void setPointTypeMixtureDetectSmokeMemory(uint8_t detect_id, uint8_t state);

// ��ʼ��/��յ���״̬
void clearPointTypeMixtureDetectAllStateMemory(void);


void MBus1SendString(uint8_t* buf, uint8_t len);
void MBus1ResetAllDevices(void);
void MBus2SendString(uint8_t* buf, uint8_t len);
uint16_t getPointTypeMixtureReceiveData16(ePointTypeDataOrder detect_data_type, uint8_t detect_id);
uint8_t getPointTypeMixtureStateClass(uint8_t detect_id);
// ��ȡ̽���������豸������(�������߼���ʾ��)
uint16_t getPointTypeMixtureNationalCode(uint8_t detector_id);

// ������ѯͬʱ������������
void MBus1PollSlaveAndReceiveTask(void* parameter);


#endif












