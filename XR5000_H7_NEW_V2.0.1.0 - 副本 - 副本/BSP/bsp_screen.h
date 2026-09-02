#ifndef __BSP_SCREEN_H
#define __BSP_SCREEN_H

#include "main.h"
#include "bsp_logic_set.h"
#include "bsp_rtc.h"

typedef enum
{
	DefauleLinkageScreenID    = 0,
	SoundLightLinkageScreenID = 1,
	SirenLinkageScreenID      = 2,
	OutFire2LinkageScreenID   = 3,
	OutFire1LinkageScreenID   = 4,
	CabinSprayLinkageScreenID = 5,
	Feedback1LinkageScreenID  = 6,
	Feedback2LinkageScreenID  = 7,
	HandPaperLinkageScreenID  = 8,

}LinkageScreenID;

typedef enum
{
	Isolate_Output_State_ID_1 = 9,
	Isolate_Output_State_ID_2 = 10,
	Isolate_Output_State_ID_3 = 11,
	Isolate_Output_State_ID_4 = 12,
	
	Feedback_State_ID_1 = 13,
	Feedback_State_ID_2 = 14,
	Feedback_State_ID_3 = 15,
	Feedback_State_ID_4 = 16,
	Feedback_State_ID_5 = 17,
	Feedback_State_ID_6 = 18,
	
	General_Output_State_ID_1 = 19,
	General_Output_State_ID_2 = 20,
	General_Output_State_ID_3 = 21,
	General_Output_State_ID_4 = 22,
	General_Output_State_ID_5 = 23,
	General_Output_State_ID_6 = 24,
	General_Output_State_ID_7 = 25,
	General_Output_State_ID_8 = 26,
	
}IO_Linkage_State_ID;

#define LinkageUseNum 27

typedef struct
{
	uint8_t curr_fault_type;
	uint8_t last_fault_type;
}LinkageStorage;

extern LinkageStorage linkage_storage_buff[14];

void InternalScreenSendString(uint8_t* buf, uint8_t len);
void InternalScreenRecvDeal(void* parameter);
void InternalScreenRecvDealTask(void* paremeter);

void InternalLinkageMonitorButtonDeal(uint16_t _screen_id, uint8_t _control_id, uint8_t _state);
void InternalScreenLinkageMonitorUpdataUI(uint16_t _screen_id);

void InternalScreenRTCSetting(uint16_t screen_id, uint16_t control_id, uint8_t *str);
void SetInternalScreenRTCTime(void);

void SetInternalScreenRTCTime(void);

void FansStateUpdataUI(uint16_t current_screen_id, uint8_t fan_disconnect_count, uint8_t fan_state1, uint8_t fan_state2, uint8_t fan_mode);
void PowerStateUpdataUI(uint16_t curr_screen_id,uint8_t zhu_state_, uint8_t bei_state_);
// ÆÁÄ»ÇÐ»»º¯Êý
void SwitchCurrentScreenId(uint16_t target_screen);

#endif

