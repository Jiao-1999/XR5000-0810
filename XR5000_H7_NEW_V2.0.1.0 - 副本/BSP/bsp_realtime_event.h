#ifndef __BSP_REALTIME_EVENT_H
#define __BSP_REALTIME_EVENT_H

/**
 * @file bsp_realtime_event.h
 * @brief 实时报警/故障事件管理模块对外接口。
 *
 * 本模块负责维护画面86（报警检测）与画面87（故障检测）所需的运行期事件，
 * 包括事件去重、分类计数、确认状态、自动滚动和页面跳转提示。事件只保存在
 * RAM中，系统复位后清空；本模块不替代Flash历史记录，也不参与设备控制和
 * 联动逻辑判断。
 */

#include <stdint.h>

/* 实时事件对应的屏幕画面及显示容量配置。 */
#define REALTIME_EVENT_SCREEN_ALARM       86U
#define REALTIME_EVENT_SCREEN_FAULT       87U
#define REALTIME_EVENT_CAPACITY           512U
#define REALTIME_EVENT_ROWS_PER_CATEGORY  7U

/*
 * 画面86/87方向导航和信息确认功能的编译开关：
 * 1U：启用上下左右、OK进入事件框以及信息确认；
 * 0U：禁用上述按键操作，但保留事件显示、自动滚动和页面自动跳转。
 */
#ifndef REALTIME_EVENT_NAVIGATION_ENABLE
#define REALTIME_EVENT_NAVIGATION_ENABLE  0U
#endif

/* 实时事件大类：报警与故障分别使用独立的存储区和显示页。 */
typedef enum
{
    REALTIME_EVENT_ALARM = 0,             /* 报警事件，对应画面86 */
    REALTIME_EVENT_FAULT,                 /* 故障事件，对应画面87 */
    REALTIME_EVENT_KIND_COUNT             /* 事件大类数量，仅用于边界检查 */
}RealtimeEventKind;

/* 画面86中的报警分类。 */
typedef enum
{
    REALTIME_ALARM_FIRE = 0,              /* 火灾报警 */
    REALTIME_ALARM_GAS,                   /* 可燃气体报警 */
    REALTIME_ALARM_LINKAGE,               /* 联动启动信息 */
    REALTIME_ALARM_EXTINGUISH,            /* 灭火装置动作信息 */
    REALTIME_ALARM_CATEGORY_COUNT         /* 报警分类数量 */
}RealtimeAlarmCategory;

/* 画面87中的故障分类。 */
typedef enum
{
    REALTIME_FAULT_CONTROLLER = 0,        /* 控制器/主机故障 */
    REALTIME_FAULT_DETECTOR,              /* 探测器故障 */
    REALTIME_FAULT_LINKAGE,               /* 联动模块故障 */
    REALTIME_FAULT_EXTINGUISH,            /* 灭火装置故障 */
    REALTIME_FAULT_CATEGORY_COUNT         /* 故障分类数量 */
}RealtimeFaultCategory;

/* 画面86/87的通用导航命令，与具体内屏板键值解耦。 */
typedef enum
{
    REALTIME_EVENT_NAV_NONE = 0,
    REALTIME_EVENT_NAV_UP,
    REALTIME_EVENT_NAV_RIGHT,
    REALTIME_EVENT_NAV_DOWN,
    REALTIME_EVENT_NAV_LEFT,
    REALTIME_EVENT_NAV_OK,
    REALTIME_EVENT_NAV_CONFIRM
}RealtimeEventNavCommand;

/* 事件首次产生时的时间快照。 */
typedef struct
{
    uint16_t year;                         /* 年 */
    uint8_t month;                         /* 月，1～12 */
    uint8_t day;                           /* 日，1～31 */
    uint8_t hour;                          /* 时，0～23 */
    uint8_t minute;                        /* 分，0～59 */
    uint8_t second;                        /* 秒，0～59 */
}RealtimeEventTime;

/**
 * @brief 初始化实时事件模块。
 * @note 清空全部运行期事件、滚动位置和页面跳转标志。
 */
void BspRealtimeEvent_Init(void);

/**
 * @brief 复位实时事件模块。
 * @note 当前行为与初始化一致，供系统复位流程调用。
 */
void BspRealtimeEvent_Reset(void);

/**
 * @brief 开始一次指定大类的事件同步扫描。
 * @param kind 事件大类，取值见RealtimeEventKind。
 * @note 调用后应逐项调用BspRealtimeEvent_Observe，最后调用
 *       BspRealtimeEvent_EndScan结束本轮扫描。
 */
void BspRealtimeEvent_BeginScan(RealtimeEventKind kind);

/**
 * @brief 上报本轮扫描中观察到的一条事件。
 * @param kind 事件大类。
 * @param category 对应大类中的事件分类。
 * @param key 事件唯一键，用于跨扫描识别同一事件条件。
 * @param time 事件首次产生时间。
 * @param loop 回路号；传入0表示主机本身事件。
 * @param address 设备地址或主机故障编号。
 * @param event_name 事件名称字符串；模块仅保存指针，不复制字符串，调用方必须
 *                   保证该字符串在事件保留期间始终有效。
 * @return 1表示新增了一条事件；0表示重复、重新激活、参数无效或容量已满。
 */
uint8_t BspRealtimeEvent_Observe(RealtimeEventKind kind, uint8_t category,
                                 uint32_t key, const RealtimeEventTime *time,
                                 uint8_t loop, uint8_t address,
                                 const uint8_t *event_name);

/**
 * @brief 上报使用自定义显示主体的事件，供多线盘等非回路设备使用。
 * @param kind 事件大类。
 * @param category 对应大类中的事件分类。
 * @param key 事件唯一键。
 * @param time 事件首次产生时间。
 * @param subject_name 显示主体，例如“多线盘1路”。
 * @param event_name 事件名称，例如“手动启动”。
 * @param require_confirmation 1表示需要人工确认并参与屏幕保活；
 *                             0表示无需确认的普通动作信息。
 * @return 1表示新增了一条事件；0表示重复、重新激活、参数无效或容量已满。
 * @note 保留原有BspRealtimeEvent_Observe接口及行为不变。
 */
uint8_t BspRealtimeEvent_ObserveNamed(RealtimeEventKind kind, uint8_t category,
                                      uint32_t key, const RealtimeEventTime *time,
                                      const uint8_t *subject_name,
                                      const uint8_t *event_name,
                                      uint8_t require_confirmation);

/**
 * @brief 结束一次指定大类的事件同步扫描。
 * @param kind 事件大类。
 * @note 本轮未再次观察到的活动事件会被标记为已消失，但记录仍保留到系统复位。
 */
void BspRealtimeEvent_EndScan(RealtimeEventKind kind);

/**
 * @brief 通知模块屏幕画面已经发生切换。
 * @param previous_screen 切换前的画面ID。
 * @param current_screen 切换后的画面ID。
 * @param now_tick 当前系统节拍。
 */
void BspRealtimeEvent_OnScreenChanged(uint16_t previous_screen,
                                      uint16_t current_screen,
                                      uint32_t now_tick);

/**
 * @brief 根据新事件及超时回看规则获取建议跳转画面。
 * @param current_screen 当前画面ID。
 * @param now_tick 当前系统节拍。
 * @return 86或87表示需要跳转；0表示无需跳转。报警优先级高于故障。
 */
uint16_t BspRealtimeEvent_GetNavigationTarget(uint16_t current_screen,
                                              uint32_t now_tick);

/**
 * @brief 刷新当前报警/故障画面的事件列表和统计信息。
 * @param current_screen 当前画面ID；非86/87画面时不执行刷新。
 * @param now_tick 当前系统节拍，用于自动滚动计时。
 */
void BspRealtimeEvent_Refresh(uint16_t current_screen, uint32_t now_tick);

/**
 * @brief 处理画面86/87的方向选择和信息确认命令。
 * @param current_screen 当前画面ID。
 * @param command 已转换的通用导航命令。
 * @param now_tick 当前系统节拍，用于退出人工浏览后重新开始滚动计时。
 * @return 成功把一条未确认事件改为已确认时返回1，否则返回0。
 */
uint8_t BspRealtimeEvent_HandleNavigation(uint16_t current_screen,
                                          RealtimeEventNavCommand command,
                                          uint32_t now_tick);

/**
 * @brief 获取指定分类从产生后保留至今的事件总数。
 * @param kind 事件大类。
 * @param category 事件分类。
 * @return 分类记录数；参数无效时返回0。
 * @note 计数包含当前已恢复但尚未因系统复位而清除的记录。
 */
uint16_t BspRealtimeEvent_GetCount(RealtimeEventKind kind, uint8_t category);

/**
 * @brief 判断指定事件大类是否存在未确认事件。
 * @param kind 事件大类。
 * @return 存在未确认事件或容量溢出时返回1，否则返回0。
 */
uint8_t BspRealtimeEvent_HasUnconfirmed(RealtimeEventKind kind);

/**
 * @brief 判断运行期是否保存过任何报警或故障事件。
 * @return 存在事件记录或容量溢出时返回1，否则返回0。
 * @note 用于报警/故障期间保持屏幕背光，不代表事件当前仍处于活动状态。
 */
uint8_t BspRealtimeEvent_HasAny(void);

/**
 * @brief 确认当前画面中可见的一条事件。
 * @param kind 事件大类。
 * @param category 事件分类。
 * @param row 当前分类可见区域内的行号，范围0～6。
 * @return 确认成功返回1；参数无效、该行无记录或已确认时返回0。
 * @note 由画面86/87的方向键导航与信息确认流程调用。
 */
uint8_t BspRealtimeEvent_ConfirmVisible(RealtimeEventKind kind,
                                       uint8_t category, uint8_t row);

#endif
