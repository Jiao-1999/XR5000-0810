#ifndef __BSP_HISTORY_FILTER_H
#define __BSP_HISTORY_FILTER_H

#include <stdint.h>

/* 画面78报警记录和画面79故障记录共用的历史筛选模块。 */
void HistoryFilter_NotifyScreen(uint16_t screen_id); /* 处理页面进入通知并清除上次残留。 */
void HistoryFilter_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state); /* 处理查询、返回和翻页。 */
void HistoryFilter_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text); /* 接收日期输入。 */
void HistoryFilter_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state); /* 接收筛选菜单。 */

#endif
