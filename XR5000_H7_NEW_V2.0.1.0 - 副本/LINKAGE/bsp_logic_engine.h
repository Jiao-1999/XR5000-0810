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

/*--------------------------------------------------------------
 * 函数指针类型 - 硬件注入
 *    引擎层调用这些函数与硬件交互。
 *    实现位于 bsp_logic_dev.c。
 *--------------------------------------------------------------*/

/* 控制设备（启动/停止指定回路指定设备的指定通道）
 * channel: 1-4=具体输出通道, 99=全部通道 */
typedef uint8_t (*ControlDevFunc_t)(uint8_t loop_no, uint8_t dev_no, uint8_t channel, uint8_t action);

/* 检查给定exec_mode下是否允许自动模式 */
typedef uint8_t (*CheckAutoModeFunc_t)(uint8_t exec_mode);
/* 联动动作事件通知回调 - 动作执行成功(控制受理)后由引擎调用, 用于黑匣子打点等观测
 * 参数: loop_no=控制回路, dev_no=被控设备地址, channel=动作通道,
 *       action=1启动/0停止. 注册方: bsp_logic_dev.c LogicDev_Register() */
typedef void (*LinkageEventFunc_t)(uint8_t loop_no, uint16_t dev_no,
                                   uint8_t channel, uint8_t action);

/*--------------------------------------------------------------
 * API：函数指针注册
 *    在初始化期间由LogicDev_Register()调用。
 *--------------------------------------------------------------*/

void LogicEngine_SetControlFunc(ControlDevFunc_t func);
void LogicEngine_SetAutoModeFunc(CheckAutoModeFunc_t func);
void LogicEngine_SetEventFunc(LinkageEventFunc_t func);   /* 注册联动动作事件回调 */

/*--------------------------------------------------------------
 * API：初始化
 *    初始化运行时状态数组。必须在LogicExpr_Init()之后、
 *    RTOS任务启动之前调用。
 *--------------------------------------------------------------*/

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
