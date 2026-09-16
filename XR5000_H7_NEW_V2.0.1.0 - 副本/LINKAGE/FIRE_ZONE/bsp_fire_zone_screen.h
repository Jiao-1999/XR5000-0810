/*==============================================================
 * 文件名称   : bsp_fire_zone_screen.h
 * 模块功能   : 防火分区画面显示与翻页（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : VIEW层：在防火分区画面的9行×6列表格中显示各分区的
 *              设备分布（分区号/分区名称/温度/烟雾/声光/输出），
 *              并处理上一页/下一页按钮。
 *              - 数据来源: bsp_fire_zone(分区表) + 各回路设备识别结果
 *              - 仅显示实际接入的设备, 未识别/离线设备不显示
 * 架构定位   : 与bsp_logic_screen同层的屏幕处理模块,
 *              由cmd_process.c的UpdateUI()/NotifyButton()调用(解耦)。
 *==============================================================*/

#ifndef __BSP_FIRE_ZONE_SCREEN_H
#define __BSP_FIRE_ZONE_SCREEN_H

#include "main.h"    /* 引入STM32基本类型: uint8_t, uint16_t等 */

/*--------------------------------------------------------------
 * 画面ID与控件ID（必须与屏工程"防火分区.tft"保持一致）
 *--------------------------------------------------------------*/

#define FIRE_ZONE_SCREEN_ID     84  /* 防火分区画面ID(上机前与屏工程核对) */

/* 按钮控件ID(已从屏工程防火分区.tft解析确认) */
#define FIRE_ZONE_BTN_PREV      22  /* 上一页 */
#define FIRE_ZONE_BTN_NEXT      23  /* 下一页 */
/* 按钮19=返回: 由屏工程页面跳转指令实现, MCU不处理 */

/* 表格控件ID映射说明(9行×6列):
 *   列0=分区号, 列1=分区名称, 列2=温度, 列3=烟雾, 列4=声光, 列5=输出
 *   行0-8对应表格第1-9行, 控件ID见.c中的映射表 */
#define FIRE_ZONE_ROWS          9   /* 每页显示分区数(表格行数) */
#define FIRE_ZONE_COLS          6   /* 表格列数 */

/*--------------------------------------------------------------
 * API声明 - 屏幕事件处理与UI刷新
 *    由cmd_process.c的UpdateUI()/NotifyButton()调用
 *--------------------------------------------------------------*/

void FireZoneScreen_UpdateUI(uint16_t screen_id);  /* 刷新画面显示(差分刷新,仅内容变化时写屏) */
void FireZoneScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state); /* 按钮处理(翻页) */

#endif /* __BSP_FIRE_ZONE_SCREEN_H */
