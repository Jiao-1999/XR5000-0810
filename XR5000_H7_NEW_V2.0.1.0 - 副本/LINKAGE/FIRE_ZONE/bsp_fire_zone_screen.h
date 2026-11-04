/*==============================================================
 * 文件名称   : bsp_fire_zone_screen.h
 * 模块功能   : 防火分区画面(画面84)交互处理（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : VIEW层：防火分区单页界面(仅2个分区槽, 每分区7类设备栏),
 *              画面底部为系统级设备区(手报/声光/火灾显示盘), 支持
 *              分区名编辑/设备范围划分/启用停用切换/提示条反馈。
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
#include "bsp_fire_zone.h"     /* 引入MODEL层枚举与API(FZ_SLOT_xxx/FZ_SYS_xxx) */

/*--------------------------------------------------------------
 * 画面ID与页面定义
 *--------------------------------------------------------------*/

#define FIRE_ZONE_SCREEN_ID     84U  /* 防火分区画面ID(与屏工程一致) */
#define FIRE_ZONE_PAGE_ZONES    2U   /* 每页分区槽数(2个分区, 单页无翻页) */
#define FIRE_ZONE_PAGE_MAX      1U   /* 总页数(2分区单页, 翻页键保留但忽略) */

/*--------------------------------------------------------------
 * 控件ID定义(与屏工程防火分区.tft逐一核对)
 *--------------------------------------------------------------*/

/* 分区号显示(槽0~1 = 101/102, 只读text_display) */
#define FIRE_ZONE_TXT_ZONE_0    101U
#define FIRE_ZONE_TXT_ZONE_1    102U

/* 分区名输入框(槽0~1 = 1/2, input_mode=1) */
#define FIRE_ZONE_IN_NAME_0     1U
#define FIRE_ZONE_IN_NAME_1     2U

/* 启用/停用按钮(槽0~1 = 105/106) */
#define FIRE_ZONE_BTN_ENABLE_0  105U
#define FIRE_ZONE_BTN_ENABLE_1  106U

/* 启用状态回显(槽0~1 = 111/112, 只读text_display) */
#define FIRE_ZONE_TXT_ENST_0    111U
#define FIRE_ZONE_TXT_ENST_1    112U

/* 系统级设备输入框(画面底部系统级设备区, 均位于回路2) */
#define FIRE_ZONE_IN_SYS_MANUAL  16U  /* 系统级手报(屏端复制控件后 ID 改为 16) */
#define FIRE_ZONE_IN_SYS_SIREN   28U  /* 系统级声光 */
#define FIRE_ZONE_IN_SYS_DISPLAY 12U  /* 系统级火灾显示盘 */

/* 翻页与返回按钮(2分区单页: 翻页键屏上仍在, MCU直接忽略) */
#define FIRE_ZONE_BTN_PREV      264U  /* 上一页(保留, 无翻页) */
#define FIRE_ZONE_BTN_NEXT      265U  /* 下一页(保留, 无翻页) */
#define FIRE_ZONE_BTN_RETURN    266U  /* 返回(屏端switch自跳新菜单界面,MCU仅复位标志) */

/* 提示条(屏端tft暂缺此控件, 写屏指令将被忽略; 屏端补控件后即生效) */
#define FIRE_ZONE_TXT_TIP       300U

/* 设备范围输入框映射表见.c中s_fz_dev[][](2分区 x 7类设备栏:
 *   温度{5,6} 烟雾{9,10} 可燃气体{14,15} 复合探测器{18,19}
 *   手报{22,23} 声光{26,27} 火灾显示盘{4,8}
 *   系统级设备(画面底部): 手报24 声光28 火灾显示盘12 */

/*--------------------------------------------------------------
 * 设备类型枚举(与MODEL层FireZoneSlot严格对齐, 避免两套枚举)
 *--------------------------------------------------------------*/

typedef enum {
    FZ_DEV_TEMP     = FZ_SLOT_TEMP,       /* 温度        回路1 */
    FZ_DEV_SMOKE    = FZ_SLOT_SMOKE,      /* 烟雾        回路1 */
    FZ_DEV_GAS      = FZ_SLOT_GAS,        /* 可燃气体    回路3 */
    FZ_DEV_COMPOUND = FZ_SLOT_COMPOUND,   /* 复合探测器  回路3 */
    FZ_DEV_MANUAL   = FZ_SLOT_MANUAL,     /* 手报        回路2 (XR2200) */
    FZ_DEV_SIREN    = FZ_SLOT_SIREN,      /* 声光        回路2 (SGBJQ) */
    FZ_DEV_DISPLAY  = FZ_SLOT_DISPLAY,    /* 火灾显示盘  回路2 */
    FZ_DEV_TYPE_COUNT = FZ_SLOT_COUNT
} FireZoneDevType;

/*--------------------------------------------------------------
 * API声明 - 屏幕事件处理与UI刷新
 *    由cmd_process.c的UpdateUI()/NotifyButton()/NotifyText()调用
 *--------------------------------------------------------------*/

void FireZoneScreen_UpdateUI(uint16_t screen_id);   /* 画面刷新(进入全刷/停留差分/离开复位) */
void FireZoneScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state); /* 按钮处理(启用切换/返回; 翻页键直接忽略) */
uint8_t FireZoneScreen_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text); /* 文本输入处理(分区名/设备范围/系统级设备) 返回1=已处理 */
void FireZoneScreen_OnZoneChanged(void);            /* 分区数据变更回调(MODEL层通知, 置脏标志) */

#endif /* __BSP_FIRE_ZONE_SCREEN_H */
