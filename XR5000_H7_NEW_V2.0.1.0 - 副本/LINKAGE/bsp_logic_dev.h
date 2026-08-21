/*==============================================================
 * 文件名称   : bsp_logic_dev.h
 * 模块功能   : 联动逻辑设备抽象层（头文件）
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 功能描述   : 逻辑引擎的硬件抽象层。封装所有硬件交互：
 *              - 传感器状态读取（分回路getter函数）
 *              - 继电器/风机控制（SoundLightRelayCtrl, Fan1Ctrl...）
 *              - 手/自动模式查询（getSysHandAutoState...）
 *              其他模块从不直接接触硬件。
 * 架构设计   :
 *   这是 HARDWARE ABSTRACTION 层：
 *   - 实现 QueryCond（表达式层通过函数指针调用）
 *   - 实现 Control（引擎层通过函数指针调用）
 *   - 实现 CheckAutoMode（引擎层通过函数指针调用）
 *   - Register() 在初始化时将所有内容连接起来
 *==============================================================*/

#ifndef __BSP_LOGIC_DEV_H
#define __BSP_LOGIC_DEV_H

#include "bsp_logic_expr.h"  /* Cond_t, Action_t, ExecMode定义 */

/*--------------------------------------------------------------
 * API：设备状态查询
 *    检查单个条件当前是否满足。
 *    根据loop_no分回路路由，读取对应传感器状态后与alarm_level匹配。
 *
 * 参数说明：cond - 条件结构体指针（loop_no, dev_no, sensor_type, alarm_level）
 * 返回值   ：1=条件满足（报警激活）, 0=条件不满足
 *
 * 示例：cond={loop_no=3, dev_no=2, sensor_type=SENSOR_METHANE, alarm_level=LV_HIGH}
 *       如果回路3的2号探测器甲烷高报警则返回1
 *--------------------------------------------------------------*/

uint8_t LogicDev_QueryCond(const Cond_t *cond);

/*--------------------------------------------------------------
 * API：设备控制
 *    在输出设备上执行控制动作。
 *    内部调用相应的继电器/风机控制函数。
 *
 * 参数说明：loop_no - 控制回路号（1-N）
 *           dev_no  - 设备号（1-N）
 *           action  - 1=启动/打开, 0=停止/关闭
 * 返回值   ：0=成功, 1=失败（设备繁忙或不可控）
 *--------------------------------------------------------------*/

uint8_t LogicDev_Control(uint8_t loop_no, uint8_t dev_no, uint8_t action);

/*--------------------------------------------------------------
 * API：自动模式检查
 *    根据当前手/自动开关位置，检查系统是否允许自动执行。
 *
 * 参数说明：exec_mode - 执行模式（ExecMode枚举）
 * 返回值   ：1=允许自动模式, 0=手动模式（动作被阻断）
 *
 * 逻辑说明：
 *   EXEC_START_ALL / EXEC_STOP_ALL:
 *     要求系统 + 分区1 + 分区2 ALL在自动模式
 *   EXEC_START_PART / EXEC_STOP_PART:
 *     要求系统/分区1/分区2中 ANY ONE 在自动模式
 *--------------------------------------------------------------*/

uint8_t LogicDev_CheckAutoMode(uint8_t exec_mode);

/*--------------------------------------------------------------
 * API：注册
 *    将所有函数指针注册到表达式层和引擎层。
 *    必须在系统初始化期间调用一次，在引擎任务启动之前。
 *
 * 主函数初始化调用顺序：
 *   1. LogicExpr_Init();        (从Flash加载规则)
 *   2. LogicDev_Register();     (连接函数指针)
 *   3. LogicEngine_Init();      (初始化运行时状态)
 *   4. [RTOS启动] LogicEngineTask运行
 *--------------------------------------------------------------*/

void LogicDev_Register(void);

#endif /* __BSP_LOGIC_DEV_H */
