#include "bsp_key.h"
#include "bsp_debug.h"
#include "system.h"
#include "bsp_screen.h"

#include "cmd_process.h"

#include "cmsis_os.h"

#include "bsp_save_ctrl.h"

/**
* 此文件用来监测手报，反馈1，反馈2的状态，包括在线，掉线，短路
* 
*/


typedef struct
{
	uint8_t linkage_id;
	uint8_t buff_index;
}LinkageIdToIndex;

LinkageIdToIndex mapping[] = {
	{.linkage_id = HandPaperLinkageScreenID, .buff_index = 0},
	{.linkage_id = Feedback1LinkageScreenID, .buff_index = 1},
	{.linkage_id = Feedback2LinkageScreenID, .buff_index = 2},
};

extern uint16_t linkage_disconnect_state;
extern uint16_t linkage_beep_ctrl;
extern uint8_t silencers_state;

extern uint8_t creatNewFaultRecordToCache(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
extern uint8_t findRecoveryDevice(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
extern void deletRecoveryRecord(uint8_t recovery_index);


uint8_t hand_paper_state = 0;
uint8_t feed_back1_state = 0;
uint8_t feed_back2_state = 0;

uint8_t getHandPaperState(void)
{
	return hand_paper_state;
}

uint8_t getFeedBack1State(void)
{
	return feed_back1_state;
}

uint8_t getFeedBack2State(void)
{
	return feed_back2_state;
}

void setDealHandPaperState(void)
{
	hand_paper_state |= 0xF0;
}

void clearHandPaperState(void)
{
	hand_paper_state = 0;
}

void setDealFeedBack1State(void)
{
	feed_back1_state |= 0xF0;
}

void cleareedBack1State(void)
{
	feed_back1_state = 0;
}

void setDealFeedBack2State(void)
{
	feed_back2_state |= 0xF0;
}

void cleareedBack2State(void)
{
	feed_back2_state = 0;
}

uint8_t getCurrKeyValue(uint8_t judge_id)
{
	uint8_t curr_state = 0; // 当前状态
	switch(judge_id)
	{
		case HandPaperLinkageScreenID:
			curr_state = (ReadHandPaperOnline()<<2)|(ReadHandPaperKeyState()<<1)|(ReadHandPaperCircuitState()<<0); // 获取当前值
			break;
		case Feedback1LinkageScreenID:
			curr_state = (ReadFeedback1Online()<<2)|(ReadFeedback1KeyState()<<1)|(ReadFeedback1CircuitState()<<0); // 获取当前值
			break;
		case Feedback2LinkageScreenID:
			curr_state = (ReadFeedback2Online()<<2)|(ReadFeedback2KeyState()<<1)|(ReadFeedback2CircuitState()<<0); // 获取当前值
			break;
		default:
			break;
	}
	return curr_state;
}

uint8_t prev_state[3] = {0}; // 上一次状态

int16_t get_index(uint8_t value) {
	uint8_t totle_sum = sizeof(mapping) / sizeof(mapping[0]);
	for (uint8_t i = 0; i < totle_sum; i++) {
		if (mapping[i].linkage_id == value) {
			return mapping[i].buff_index;
		}
	}
	return -1; // 未找到
}

uint8_t BspKeyFaultSaveApp(uint8_t judge_id)
{
	uint8_t packs_cabin_id;
	switch(judge_id)
	{
		case HandPaperLinkageScreenID:
			packs_cabin_id = HANDPOT_Package_ID;
			break;
		case Feedback1LinkageScreenID:
			packs_cabin_id = FEEDBK1_Package_ID;
			break;
		case Feedback2LinkageScreenID:
			packs_cabin_id = FEEDBK2_Package_ID;
			break;
		default:
			break;
	}
	return packs_cabin_id;
}

void KeyStateJudge(uint8_t judge_id, LinkageStorage *buff_entry)
{
	uint8_t curr_state = 0; // 当前状态
	uint8_t temp_state = 0;
	
	uint8_t temp_index = 0;
	
	// 按键赋值操作
	curr_state = getCurrKeyValue(judge_id); // 获取按键值
	temp_index = get_index(judge_id); // 获取对应的存储位置
	if(curr_state != prev_state[temp_index]) // 判断值是否有变动 如果有
	{
		osDelay(20);
		temp_state = getCurrKeyValue(judge_id); // 再获取一次
		if(temp_state == curr_state) // 如果两次获取的值相同 认为按键状态变更
		{
			linkage_work_state[judge_id] = curr_state; // 存储变更后的值
			prev_state[temp_index] = curr_state;
		}
		else
		{
			linkage_work_state[judge_id] = prev_state[temp_index];
		}
	}
	
	// 键值处理操作
	if(linkage_shield_state[judge_id] == LinkageOpen) // 如果设置上线
	{
		// 给状态数组赋值
		buff_entry[judge_id].curr_fault_type = linkage_work_state[judge_id]; // 获取当前状态
		// 如果外联设备报掉线 且和上次故障类型不一样 储存新的报警信息
		if(linkage_work_state[judge_id] == FeedbackKeyDisconnect && 
			buff_entry[judge_id].last_fault_type != buff_entry[judge_id].curr_fault_type) 
		{
			uint8_t temp_linkage_pack_id;
			temp_linkage_pack_id = BspKeyFaultSaveApp(judge_id);
			// 存储到FLASH故障存储区中 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, temp_linkage_pack_id);
			// 存储到缓冲区
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, temp_linkage_pack_id, DISCONNECT);
			
			buff_entry[judge_id].last_fault_type = buff_entry[judge_id].curr_fault_type;	
			// 
			my_set_x_bit(linkage_disconnect_state, judge_id);
			my_set_x_bit(linkage_beep_ctrl, judge_id);
			silencers_state = 0;
		}
		else if(linkage_work_state[judge_id] == FeedbackKeyShort && 
			buff_entry[judge_id].last_fault_type != buff_entry[judge_id].curr_fault_type) 
		{		
			uint8_t temp_linkage_pack_id;
			
			temp_linkage_pack_id = BspKeyFaultSaveApp(judge_id);
			// 存储到FLASH故障存储区中 掉线
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, temp_linkage_pack_id);
			// 存储到缓冲区
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, temp_linkage_pack_id, SHORTCIRCUIT);
			
			buff_entry[judge_id].last_fault_type = buff_entry[judge_id].curr_fault_type;
			
			my_set_x_bit(linkage_disconnect_state, judge_id);
			my_set_x_bit(linkage_beep_ctrl, judge_id);
			silencers_state = 0;
		}
		else if(linkage_work_state[judge_id] == FeedbackKeyOnline &&
			buff_entry[judge_id].last_fault_type != linkage_work_state[judge_id])// 如果设备恢复在线
		{	
			if(buff_entry[judge_id].last_fault_type == FeedbackKeyShort)
			{
				uint8_t temp_index_ = 0xFF;
				uint8_t temp_linkage_pack_id;
				temp_linkage_pack_id = BspKeyFaultSaveApp(judge_id);
				// 存储到FLASH故障存储区中 掉线
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, temp_linkage_pack_id);
				// 存储到缓冲区
				creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, temp_linkage_pack_id, SHO_RECOVERY);
				temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, temp_linkage_pack_id, SHO_RECOVERY);
				if(0xFF != temp_index_)
				{
					deletRecoveryRecord(temp_index_);
				}
			}
			else if(buff_entry[judge_id].last_fault_type == FeedbackKeyDisconnect)
			{
				uint8_t temp_linkage_pack_id;
				uint8_t temp_index_ = 0xFF;
				
				temp_linkage_pack_id = BspKeyFaultSaveApp(judge_id);
				// 存储到FLASH故障存储区中 掉线
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, temp_linkage_pack_id);
				// 从缓冲区中删除
				temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, temp_linkage_pack_id, DIS_RECOVERY);
				if(0xFF != temp_index_)
				{
					deletRecoveryRecord(temp_index_);
				}
			}
			buff_entry[judge_id].last_fault_type = FeedbackKeyOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
			my_clear_x_bit(linkage_disconnect_state, judge_id);
			my_clear_x_bit(linkage_beep_ctrl, judge_id);
			silencers_state = 0;
		}
	}
	else
	{
		if( judge_id == HandPaperLinkageScreenID || 
				judge_id == Feedback1LinkageScreenID ||  
				judge_id == Feedback2LinkageScreenID    // 如果不是按键)
		)
		{
			my_clear_x_bit(linkage_disconnect_state, judge_id); // 清除故障标志位
			my_clear_x_bit(linkage_beep_ctrl, judge_id); // 清除标志位
			buff_entry[judge_id].last_fault_type = FeedbackKeyOnline; // 清除上一次储存的故障信息 确保下一次启动后正常存储
		}
	}
	
	// 即使禁用也判断是否按下
	if(linkage_work_state[judge_id] == FeedbackKeyPress &&
		buff_entry[judge_id].last_fault_type != linkage_work_state[judge_id])  // 如果上一个状态不是按下
	{
		// 只赋值一次
		buff_entry[judge_id].last_fault_type = linkage_work_state[judge_id];
		if(judge_id == Feedback1LinkageScreenID)
		{
			feed_back1_state |= 0x0F;
		}
		else if(judge_id == Feedback2LinkageScreenID)
		{
			feed_back2_state |= 0x0F;
		}
		else if(judge_id == HandPaperLinkageScreenID)
		{
			hand_paper_state |= 0x0F;
		}
	}
}

void KeyStateJudgeTask(void* parameter)
{
	for(;;)
	{
		for(uint8_t i = Feedback1LinkageScreenID; i < HandPaperLinkageScreenID + 1; i++)
		{
			KeyStateJudge(i, linkage_storage_buff);
		}
		
		osDelay(800);
	}
}
