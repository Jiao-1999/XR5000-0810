#ifndef __BSP_FDCAN1_H
#define __BSP_FDCAN1_H

#include "main.h"
#include "fdcan.h"

#define IG3306_SLAVE_ADDR          0x01

#define IG3306_QUERY_ALL_STATE_CMD 0x11

#define IG3306_QUERY_FEEDBCAK_CMD  0x12

#define IG3306_CTRL_SOLENOID_CMD   0x13

#define IG3306_CTRL_SOUNDLIGHT_CMD 0x14

#define IG3306_CTRL_DRY_CONTACT_1_CMD 0x15

#define IG3306_CTRL_DRY_CONTACT_2_CMD 0x16

// 4路隔离输出
#define Isolate_Output_Register 4
// 四路电磁阀
#define Solenoid_Valve_Register 4
// 6路反馈
#define Feedback_State_Register 6
// 4路声光
#define Sounds_Lights_Register  4
// 8路干接点
#define Dry_Contact_Register 8

typedef enum
{
	IsolateOutputNomal = 0x00,
	IsolateOutputShort = 0x01
}eIsolateOutputState;

typedef enum
{
	SolenoidValveNomal = 0x00,
	SolenoidValveShort = 0x01,
	SolenoidValveOpens = 0x02,
	SolenoidValveRunDc = 0x03,
}eSolenoidValveState;

typedef enum
{
	SoundsLightsNomal = 0x00,
	SoundsLightsShort = 0x01,
	SoundsLightsOpens = 0x02,
	SoundsLightsRunDc = 0x03,
}eSoundsLightsState;

typedef enum
{
	DryContactNomal = 0x00,
	DryContactRunDc = 0x01,
}eDryContactState;

typedef enum
{
	FeedbackTriggerNomal = 0x00,
	FeedbackTriggerShort = 0x01,
	FeedbackTriggerOpens = 0x02,
	FeedbackTriggerRunDc = 0x03,
}eFeedbackTriggerState;

void FDCAN1_Start(void);

HAL_StatusTypeDef FDCAN1_SendStdDataFrame(uint32_t id, uint8_t* data, uint8_t length);

HAL_StatusTypeDef FDCAN1_SendExtDataFrame(uint32_t id, uint8_t* data, uint8_t length);

HAL_StatusTypeDef FDCAN1_SendRemoteFrame(uint32_t id, uint8_t isExtended);

void Fdcan1SendAndReceiveTask(void *parameter);

uint8_t getIsolateOutputState(uint8_t query_register_id);
uint8_t getSolenoidValveState(uint8_t query_register_id);
uint8_t getSoundsLightsState(uint8_t query_register_id);
uint8_t getfeedbackState(uint8_t query_register_id);
uint8_t getDryContactState(uint8_t query_register_id);

#endif
