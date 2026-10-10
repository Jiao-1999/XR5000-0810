/*
 * 历史报警与历史故障筛选模块公共接口。
 *
 * 画面78用于报警历史筛选，画面79用于故障历史筛选。调用方将页面进入、按钮、
 * 文本和菜单事件转交给本模块，具体筛选条件和Flash只读扫描由模块内部完成。
 */

#ifndef __BSP_HISTORY_FILTER_H
#define __BSP_HISTORY_FILTER_H

#include <stdint.h>

/* 处理页面进入通知，并初始化对应画面的筛选和显示状态。 */
void HistoryFilter_NotifyScreen(uint16_t screen_id);
/* 处理查询、返回和翻页按钮。 */
void HistoryFilter_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
/* 接收起始日期和结束日期文本。 */
void HistoryFilter_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text);
/* 接收回路、报警类型或故障类型菜单选择。 */
void HistoryFilter_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state);

#endif
