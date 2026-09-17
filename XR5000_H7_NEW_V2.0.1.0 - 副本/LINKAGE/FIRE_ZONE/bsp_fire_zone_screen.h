/*==============================================================
 * 文件名称   : bsp_fire_zone_screen.h
 * 模块功能   : 防火分区画面(画面84)交互处理（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : VIEW层：防火分区2×2四宫格界面(每页4个分区槽,
 *              2页共8个分区), 支持翻页/分区名编辑/设备范围划分/
 *              启用停用切换/提示条反馈。
 *              - 数据来源: bsp_fire_zone(分区表) + 各回路设备识别结果
 *              - 回显策略: 按产品码将划入设备归入对应设备类型栏,
 *                未识别设备不归栏(保存时提示)
 * 架构定位   : 与bsp_logic_screen同层的屏幕处理模块,
 *              由cmd_process.c的UpdateUI()/NotifyButton()/NotifyText()
 *              调用(解耦), 分区数据变化经OnZoneChanged回调驱动刷新。
 * 控件依据   : 屏工程"防火分区.tft"实测控件表(2026-09-16)
 *==============================================================*/

#ifndef __BSP_FIRE_ZONE_SCREEN_H
#define __BSP_FIRE_ZONE_SCREEN_H

#include "main.h"    /* 引入STM32基本类型: uint8_t, uint16_t等 */

/*--------------------------------------------------------------
 * 画面ID与页面定义
 *--------------------------------------------------------------*/

#define FIRE_ZONE_SCREEN_ID     84U  /* 防火分区画面ID(与屏工程一致) */
#define FIRE_ZONE_PAGE_ZONES    4U   /* 每页分区槽数(2×2四宫格) */
#define FIRE_ZONE_PAGE_MAX      2U   /* 总页数(分区1~4/5~8, 控件复用) */

/*--------------------------------------------------------------
 * 控件ID定义(与屏工程防火分区.tft逐一核对)
 *--------------------------------------------------------------*/

/* 分区号显示(槽0~3 = 101~104, 只读text_display) */
#define FIRE_ZONE_TXT_ZONE_0    101U
#define FIRE_ZONE_TXT_ZONE_3    104U

/* 分区名输入框(槽0~3 = 1~4, input_mode=1) */
#define FIRE_ZONE_IN_NAME_0     1U
#define FIRE_ZONE_IN_NAME_3     4U

/* 启用/停用按钮(槽0~3 = 105~108) */
#define FIRE_ZONE_BTN_ENABLE_0  105U
#define FIRE_ZONE_BTN_ENABLE_3  108U

/* 启用状态回显(槽0~3 = 111~114, 只读text_display) */
#define FIRE_ZONE_TXT_ENST_0    111U
#define FIRE_ZONE_TXT_ENST_3    114U

/* 翻页与返回按钮 */
#define FIRE_ZONE_BTN_PREV      264U  /* 上一页 */
#define FIRE_ZONE_BTN_NEXT      265U  /* 下一页 */
#define FIRE_ZONE_BTN_RETURN    266U  /* 返回(屏端switch自跳新菜单界面,MCU仅复位标志) */

/* 提示条(屏端tft暂缺此控件, 写屏指令将被忽略; 屏端补控件后即生效) */
#define FIRE_ZONE_TXT_TIP       300U

/* 设备范围输入框映射表见.c中s_fz_dev[][]:
 *   温度{5,6,7,8} 烟雾{9,10,11,12} 可燃气体{14~17}
 *   复合探测器{18~21} 手报{22~25} 声光{26~29} */

/*--------------------------------------------------------------
 * 设备类型枚举(四宫格每槽的6类设备栏)
 *--------------------------------------------------------------*/

typedef enum {
    FZ_DEV_TEMP     = 0,   /* 温度        回路1 */
    FZ_DEV_SMOKE    = 1,   /* 烟雾        回路1 */
    FZ_DEV_GAS      = 2,   /* 可燃气体    回路3 */
    FZ_DEV_COMPOUND = 3,   /* 复合探测器  回路3 */
    FZ_DEV_MANUAL   = 4,   /* 手报        回路2 (XR2200) */
    FZ_DEV_SIREN    = 5,   /* 声光        回路2 (SGBJQ) */
    FZ_DEV_TYPE_COUNT = 6
} FireZoneDevType;

/*--------------------------------------------------------------
 * API声明 - 屏幕事件处理与UI刷新
 *    由cmd_process.c的UpdateUI()/NotifyButton()/NotifyText()调用
 *--------------------------------------------------------------*/

void FireZoneScreen_UpdateUI(uint16_t screen_id);   /* 画面刷新(进入全刷/停留差分/离开复位) */
void FireZoneScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state); /* 按钮处理(翻页/启用切换/返回) */
uint8_t FireZoneScreen_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text); /* 文本输入处理(分区名/设备范围) 返回1=已处理 */
void FireZoneScreen_OnZoneChanged(void);            /* 分区数据变更回调(MODEL层通知, 置脏标志) */

#endif /* __BSP_FIRE_ZONE_SCREEN_H */
