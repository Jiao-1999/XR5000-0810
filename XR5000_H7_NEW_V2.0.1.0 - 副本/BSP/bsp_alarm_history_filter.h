#ifndef __BSP_ALARM_HISTORY_FILTER_H
#define __BSP_ALARM_HISTORY_FILTER_H

#include "main.h"

/* 画面78报警历史筛选：仅查询现有Flash记录，不改变存储格式。 */
void AlarmHistoryFilter_NotifyScreen(uint16_t screen_id);
void AlarmHistoryFilter_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
void AlarmHistoryFilter_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);
void AlarmHistoryFilter_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state);

#endif
