/*==============================================================
 * 文件名称   : bsp_logic_engine.h
 * 模块功能   : 联动逻辑引擎（头文件）
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM, CMSIS-RTOS
 * 功能描述   : 驱动规则评估和动作执行的状态机。作为RTOS任务运行。
 * 架构设计   :
 *   这是 CONTROLLER 层：
 *   - 调用 LogicExpr_Evaluate() 检查条件
 *   - 调用已注册的 Control 函数执行动作
 *   - 调用已注册的 CheckAutoMode 门控执行
 *   - 管理延时计时器和状态转换
 *   - 不知道屏幕协议或硬件细节
 *==============================================================*/

#ifndef __BSP_LOGIC_ENGINE_H
#define __BSP_LOGIC_ENGINE_H

#include "bsp_logic_expr.h"  /* LogicRule_t, RtState等 */

/*--------------------------------------------------------------
 * 常量：任务配置
 *--------------------------------------------------------------*/

#define LOGIC_ENGINE_TASK_STACK    512   /* RTOS任务堆栈大小（字） */
#define LOGIC_ENGINE_TASK_PRIO     2     /* RTOS任务优先级（数值越小优先级越低） */
#define LOGIC_ENGINE_CYCLE_MS      100   /* 引擎扫描周期（毫秒） */
#define LOGIC_ENGINE_STARTUP_DELAY 2000  /* 启动延时（毫秒，等待传感器稳定） */

/* 联动延时启动时间(毫秒): 条件满足后延时该时长再启动动作设备 */
#ifndef LOGIC_ACTION_DELAY_MS
#define LOGIC_ACTION_DELAY_MS   0U      /* XR5000_DELAY_ZERO_20261112: 联动延时去掉, 条件成立即下发 */
#endif

/*--------------------------------------------------------------
 * API：联动延时设置 与 分区手动操作
 *--------------------------------------------------------------*/

/* 联动延时启动时间设置/读取(毫秒), 缺省为 LOGIC_ACTION_DELAY_MS */
void     LogicEngine_SetActionDelay(uint32_t ms);
uint32_t LogicEngine_GetActionDelay(void);

/* 分区手动操作: 登记本区所有"条件成立"规则的启动请求, 与自动一样走本区联动延时;
 * 延时灯与画面86 倒计时据此显示, 延时到点由引擎统一下发。 */
uint8_t LogicEngine_ManualStartZone(uint8_t zone);  /* 返回登记的规则数(0=没有条件成立的规则) */
/* 分区手动停止: 停止本区所有已执行的启动类动作 */
uint8_t LogicEngine_ManualStopZone(uint8_t zone);   /* 返回实际停止下发的动作数(0=没有需要停的设备) */
/* 取消本区所有规则的延时计时(复位时调用) */
void LogicEngine_CancelZoneDelay(uint8_t zone);
/* 清除分区停止闭锁(分区复位时调用) */
void LogicEngine_ClearZoneStopLatch(uint8_t zone);

/*--------------------------------------------------------------
 * 函数指针类型 - 硬件注入
 *    引擎层调用这些函数与硬件交互。
 *    实现位于 bsp_logic_dev.c。
 *--------------------------------------------------------------*/

/* 控制设备（启动/停止指定回路指定设备的指定通道）
 * channel: 1-4=具体输出通道, 99=全部通道 */
typedef uint8_t (*ControlDevFunc_t)(uint8_t loop_no, uint8_t dev_no, uint8_t channel, uint8_t action);

/* 检查给定规则是否允许自动模式(系统总闸 + 规则所属分区) */
typedef uint8_t (*CheckAutoModeFunc_t)(const LogicRule_t *rule);
/* 联动动作事件通知回调 - 动作执行成功(控制受理)后由引擎调用, 用于黑匣子打点等观测
 * 参数: loop_no=控制回路, dev_no=被控设备地址, channel=动作通道,
 *       action=1启动/0停止. 注册方: bsp_logic_dev.c LogicDev_Register() */
typedef void (*LinkageEventFunc_t)(uint8_t loop_no, uint16_t dev_no,
                                   uint8_t channel, uint8_t action);

/* 手动启动键查询回调 - 手动模式下引擎查询"外联设备启动"键是否按下
 * 返回值: 1=键已按下(允许手动执行联动), 0=未按下
 * 注册方: bsp_logic_dev.c LogicDev_Register() */
typedef uint8_t (*ManualStartFunc_t)(void);

/*--------------------------------------------------------------
 * API：函数指针注册
 *    在初始化期间由LogicDev_Register()调用。
 *--------------------------------------------------------------*/

void LogicEngine_SetControlFunc(ControlDevFunc_t func);
void LogicEngine_SetAutoModeFunc(CheckAutoModeFunc_t func);
void LogicEngine_SetEventFunc(LinkageEventFunc_t func);   /* 注册联动动作事件回调 */
void LogicEngine_SetManualStartFunc(ManualStartFunc_t func); /* 注册手动启动键查询回调 */

/*--------------------------------------------------------------
 * API：初始化
 *    初始化运行时状态数组。必须在LogicExpr_Init()之后、
 *    RTOS任务启动之前调用。
 *--------------------------------------------------------------*/

/* V8: 联动动作执行视图(供画面86联动监测显示) */
typedef struct
{
    uint8_t  valid;         /* 1=本条有效 */
    uint8_t  rule_id;       /* 规则ID */
    uint8_t  loop_no;       /* 动作回路(固定2) */
    uint8_t  dev_no;        /* 动作设备地址 */
    uint8_t  channel;       /* 动作通道(1~4, 99=全部) */
    uint8_t  state;         /* 0=延时中 1=已下发待反馈 2=反馈已启动 3=已停止/失败 */
    uint16_t remain_s;      /* 延时剩余秒数(向上取整), 仅state=0有效 */
    uint32_t trigger_tick;  /* 触发时刻, 用于"先触发先显示"排序 */
} LogicActionView_t;

/* 取当前正在延时/已执行的联动动作, 按触发时刻升序, 最多max条 */
uint8_t LogicEngine_GetActionViews(LogicActionView_t *out, uint8_t max);
/* V8: 是否曾经触发过联动(本次上电期间), 触发后恒1, 仅LogicEngine_Init清0 */
uint8_t LogicEngine_HasEverTriggered(void);

/* 需求4: 分区联动状态位(供面板分区延时/喷洒启动灯驱动) */
#define LOGIC_ZONE_ST_DELAYING   0x01U   /* 本区有规则正在启动延时 */
#define LOGIC_ZONE_ST_ACTIVE     0x02U   /* 本区有启动类动作已下发 */

/* 汇总指定分区的联动状态, 返回 LOGIC_ZONE_ST_* 位掩码; 参数非法返回0 */
uint8_t LogicEngine_GetZoneLinkState(uint8_t zone);

void LogicEngine_Init(void);

/*--------------------------------------------------------------
 * API：RTOS任务入口
 *    FreeRTOS任务函数。创建内部定时循环：
 *    1. 等待LOGIC_ENGINE_STARTUP_DELAY毫秒等待传感器数据
 *    2. 无限循环：运行引擎 + 延时LOGIC_ENGINE_CYCLE_MS
 *
 * 在freertos.c中的用法：
 *   osThreadDef(LogicEngine, LogicEngineTask, osPriorityNormal, 0, LOGIC_ENGINE_TASK_STACK);
 *   osThreadCreate(osThread(LogicEngine), NULL);
 *--------------------------------------------------------------*/

void LogicEngineTask(void *parameter);

#endif /* __BSP_LOGIC_ENGINE_H */
