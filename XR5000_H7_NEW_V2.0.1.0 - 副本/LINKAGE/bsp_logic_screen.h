/*==============================================================
 * 文件名称   : bsp_logic_screen.h
 * 模块功能   : 火警逻辑屏幕处理（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 通过HMI屏幕协议在逻辑设定界面与表达式之间建立桥梁。
 *              将屏幕按键事件转换为表达式Token，刷新屏幕显示。
 * 架构定位   :
 *   属于 VIEW 层：
 *   - 接收大彩屏的按键事件（通过NotifyButton）
 *   - 将按键转换为条件/Token结构
 *   - 调用LogicRule_Create/Modify等接口操作
 *   - 刷新屏幕UI显示当前表达式
 *   - 不执行列表保存、表达式求值、硬件控制
 * 关键映射   : 匹配工程屏幕逻辑设定界面(screen 43)
 *              控件ID需根据实际.tft文件进行适配
 *==============================================================*/

#ifndef __BSP_LOGIC_SCREEN_H
#define __BSP_LOGIC_SCREEN_H

#include "bsp_logic_expr.h"  /* LogicRule_t, Cond_t, Token_t等结构 */

/*--------------------------------------------------------------
 * 屏幕页面ID
 *    这些宏必须与屏幕工程的项目匹配。
 *    若工程更改页面ID，需要同步修改此处。
 *--------------------------------------------------------------*/

#define LOGIC_SCREEN_EDIT       43  /* 火警逻辑设定页：编辑+预览(主界面) */
#define LOGIC_SCREEN_LIST       45  /* 逻辑查看界面：只读显示已设置规则 */
#define LOGIC_SCREEN_ACTION     43  /* 复用编辑页(无独立动作编辑页) */

/*--------------------------------------------------------------
 * 编辑页(screen 43)控件ID
 *    每个屏幕按钮有唯一的control_id。
 *    这些宏与屏幕工程逻辑设定.tft匹配。
 *--------------------------------------------------------------*/

/* 数字按钮（0-9，用于输入探测器编号或动作编号） */
#define LOGIC_BTN_DIGIT_BASE    1   /* control_id 1-9 = 数字 1-9 */
#define LOGIC_BTN_DIGIT_0       10  /* control_id 10 = 数字 0 */

/* 分隔符和操作符按钮 */
#define LOGIC_BTN_HASH          11  /* '#' 分隔符(确认当前探测器编号,任意报警) */
#define LOGIC_BTN_OR            12  /* '|' 逻辑或 */
#define LOGIC_BTN_AND           13  /* '&' 逻辑与(条件区) / 动作分隔符(动作区) */
#define LOGIC_BTN_CANCEL        14  /* 取消修改，清空编辑状态 */
#define LOGIC_BTN_DELETE        15  /* 退格删除最后一个Token */
#define LOGIC_BTN_CONFIRM       16  /* 确认/保存当前规则到RAM */

/* 括号和等号(屏幕screen 43实际控件ID) */
#define LOGIC_BTN_LPAREN        18  /* '(' 左括号 */
#define LOGIC_BTN_EQUAL         20  /* '=' 分隔符，开始选择执行动作 */
#define LOGIC_BTN_RPAREN        21  /* ')' 右括号 */

/* 功能按钮 */
#define LOGIC_BTN_VIEW_LIST     23  /* 查看逻辑(跳转到screen 45) */
#define LOGIC_BTN_NEW_RULE      72  /* 新建规则(开始新编辑) */

/* 传感器快捷按钮 — 对应屏幕逻辑设定界面新增按钮
 * 输入5位编码后按传感器按钮确认条件(高报警级别) */
#define LOGIC_BTN_PRESS         19  /* 压力(仅回路3) */
#define LOGIC_BTN_VOC           24  /* VOC挥发性有机物(仅回路3) */
#define LOGIC_BTN_CH4           25  /* CH4甲烷(仅回路3) */
#define LOGIC_BTN_H2            26  /* H2氢气(仅回路3) */
#define LOGIC_BTN_CO            27  /* CO一氧化碳(仅回路3) */
#define LOGIC_BTN_SMOKE         28  /* 烟雾(回路1+3) */
#define LOGIC_BTN_TEMP          29  /* 温度(回路1+3) */

/* 预览窗口控件ID */
#define LOGIC_TXT_PREVIEW       73  /* 预览窗口TextDisplay */

/*--------------------------------------------------------------
 * 列表页(screen 45)控件ID
 *    显示已设置的规则，每行一条规则。
 *--------------------------------------------------------------*/

#define LOGIC_TXT_LIST_BASE     10  /* 第1条规则（10-18共9条） */

/*--------------------------------------------------------------
 * 动作编号映射表（等号后数字按钮）
 *    用于将屏幕数字转换为ActDevType枚举。
 *--------------------------------------------------------------*/

/* 5位编码: 前2位回路号 + 后3位设备号, 如 01002 = 回路01的002号设备
 * 条件用探测回路, 动作用控制回路, 均通过5位数字按钮输入
 * 动作区多个动作用 & 按钮分隔(最多4个) */

/*--------------------------------------------------------------
 * API声明 - 屏幕事件处理
 *    这些函数应由cmd_process.c的NotifyButton()和NotifyText()回调中调用。
 *--------------------------------------------------------------*/

/* 处理来自大彩屏的按键按下事件
 * 调用位置：cmd_process.c中的NotifyButton()
 * 参数 screen_id: 当前屏幕页面ID
 * 参数 control_id: 按下的按钮ID
 * 参数 state: 1=按下, 0=释放 */
void LogicScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state);

/* 处理来自大彩屏的文本输入事件
 * 调用位置：cmd_process.c中的NotifyText()
 * 参数 screen_id: 当前屏幕页面ID
 * 参数 control_id: 按下的文本控件ID
 * 参数 str: 文本内容（以null结尾字符串） */
void LogicScreen_OnText(uint16_t screen_id, uint16_t control_id, uint8_t *str);

/*--------------------------------------------------------------
 * API声明 - 屏幕UI刷新
 *    在屏幕刷新循环中调用，根据当前页面刷新显示。
 * 参数 screen_id: 当前屏幕页面ID
 *--------------------------------------------------------------*/

void LogicScreen_UpdateUI(uint16_t screen_id);

/*--------------------------------------------------------------
 * API声明 - 初始化
 *    初始化屏幕处理模块内部状态。
 *--------------------------------------------------------------*/

void LogicScreen_Init(void);

#endif /* __BSP_LOGIC_SCREEN_H */
