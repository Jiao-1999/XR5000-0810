/*==============================================================
 * 文件名称   : bsp_logic_screen.c
 * 模块功能   : 火警逻辑屏幕处理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 通过HMI屏幕协议在逻辑设定界面与表达式之间建立桥梁。
 *              将屏幕按键事件转换为表达式Token，刷新屏幕显示。
 *
 * 架构定位   : VIEW层。
 *              只接收屏幕事件，处理后刷新UI。
 *              不执行列表保存、表达式求值、硬件控制。
 *
 * 5位编码   : 前2位=回路号, 后3位=设备号
 *              条件: 探测回路号+设备号 (如 01002 = 01探测回路002号设备)
 *              动作: 控制回路号+设备号 (如 01002 = 01控制回路002号设备)
 *              操作符: & (与) | (或) ( ) 括号
 *
 * 编辑状态机 : 双维度状态
 *              phase: EDIT_COND(条件区,=之前) / EDIT_ACTION(动作区,=之后)
 *              step : NONE(空闲) / NUMBER(条件编码输入中) / ACTION(动作编码输入中)
 *              - 条件区输入5位编码后按传感器按钮=带子级别条件(如01001温度)
 *              - 条件区输入5位编码后按#=任意报警条件
 *              - 动作区&按钮=多动作分隔符, 确认键=保存规则
 *
 * 表达式格式 : <5位条件编码><传感器> & <5位条件编码> = <5位动作编码> & <5位动作编码>
 *              示例: 01001温度 & 01002烟雾 = 01001 & 02003
 *
 * 依赖模块   :
 *   - bsp_logic_expr.h    : 规则CRUD与表达式求值
 *   - bsp_logic_screen.h  : 本模块API与屏幕控件
 *   - hmi_driver.h        : SetTextValue, clearTextValue
 *   - <stdio.h>           : sprintf格式化显示
 *   - <string.h>          : memset, strcat, strlen
 *==============================================================*/

#include "bsp_logic_screen.h"    /* 本模块头文件 */
#include "bsp_logic_expr.h"      /* 规则CRUD与表达式求值 */
#include "hmi_driver.h"          /* SetTextValue, clearTextValue */
#include "bsp_screen.h"          /* SwitchCurrentScreenId: 画面85选完分区跳43 */
#include "bsp_fire_zone.h"       /* FIRE_ZONE_MAX: 规则分区号范围 */
#include "bsp_debug.h"           /* DebugPrintf: test trace on UART4 */
#include "bsp_logic_dev.h"      /* LogicDev_Control: 删除规则前下发 STOP */
#include "bsp_device_registry.h" /* 国标设备类型码→中文显示名 */
#include "bsp_mbus.h"            /* 回路1设备国标码读取 */
#include "bsp_mbus_control.h"    /* 回路2设备国标码读取 */
#include "bsp_rs485_detect.h"    /* 回路3设备国标码读取 */
#include <stdio.h>               /* sprintf */
#include <string.h>              /* memset, strcat, strlen */

/*--------------------------------------------------------------
 * 第一部分：编辑状态机
 *--------------------------------------------------------------*/

/* 编辑阶段枚举（条件区/动作区） */
typedef enum
{
    EDIT_COND   = 0,  /* 条件区(=之前)：输入触发条件表达式 */
    EDIT_ACTION = 1,  /* 动作区(=之后)：输入联动执行动作 */
} EditPhase;

/* 编辑步骤枚举 */
typedef enum
{
    EDIT_STEP_NONE     = 0,  /* 空闲状态，等待输入数字或操作符 */
    EDIT_STEP_NUMBER   = 1,  /* 正在输入5位条件编码，等待#或传感器按钮确认 */
    EDIT_STEP_ACTION   = 2,  /* 正在输入5位动作编码，等待&分隔或确认键保存 */
} EditStep;

/* 编辑状态结构体 */
typedef struct
{
    uint8_t       active;        /* 1=正在编辑规则, 0=空闲 */
    EditPhase     phase;         /* 当前编辑阶段(条件区/动作区) */
    EditStep      step;          /* 当前编辑步骤 */
    uint8_t       digit_buf[12]; /* 数字输入缓冲区: 条件5位(单点)/11位(区间), 动作7位; 0xFF=区间分隔符 */
    uint8_t       digit_count;   /* 已输入数字位数 */
    LogicRule_t   rule;          /* 当前正在构造的规则 */
    uint8_t       edit_rule_id;  /* 正在编辑的规则ID，0=新建规则 */
    uint8_t       has_action;    /* 1=已设置执行动作, 0=未设置 */
    uint8_t       zone;          /* 本次编辑锁定的分区(1-2), 由画面85设定 */
} EditState;

/* 全局编辑状态（静态变量） */
static EditState s_edit;

/* 显示缓冲区（静态变量，512字节足够12个条件+括号+多动作） */
static char s_disp_buf[512];
static uint32_t s_save_msg_until = 0;  /* 保存结果提示显示截止tick, 期间实时预览不覆盖 */

/* 列表页分页状态（静态变量）
 *   s_list_page: 当前页码(0基), 最新规则在第0页最上方
 *   进入列表页时不重置页码, 保留上次浏览位置 */
static uint8_t s_list_page = 0;

/* 画面45 删除功能状态 */
static uint8_t  s_list_selected = 0xFFU;   /* 0~8=选中行, 0xFF=未选 */
static uint8_t  s_list_armed = 0U;         /* 1=已按一次删除, 等待确认 */
static uint32_t s_list_arm_tick = 0U;      /* 待确认起始时刻 */
static uint8_t  s_last_list_count = 0xFFU; /* 上次列表刷新的规则数(差分门控) */
static uint8_t  s_last_list_page  = 0xFFU; /* 上次列表刷新的页码(差分门控) */

/*--------------------------------------------------------------
 * 第二部分：内部辅助函数
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：AllocRuleId
 * 功能说明：分配一个新的可用规则ID。
 * 返回值  ：返回当前最大ID+1；如果没有规则则返回1。
 *--------------------------------------------------------------*/
static uint8_t AllocRuleId(void)
{
    LogicRule_t *rules;
    uint8_t i;
    uint8_t max_id = 0;

    rules = LogicExpr_GetTable();

    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id > max_id)
        {
            max_id = rules[i].rule_id;
        }
    }

    return (uint8_t)(max_id + 1);
}

/*--------------------------------------------------------------
 * 函数名称：NeedDigitCount
 * 功能说明：当前编辑阶段需要的数字位数。
 *           EDIT_STEP_ACTION(动作阶段)需要7位, 条件阶段需要5位。
 * 返回值  ：7=动作编码(回路2+设备3+通道2), 5=条件编码(回路2+设备3)
 *--------------------------------------------------------------*/
static uint8_t NeedDigitCount(void)
{
    /* 动作段7位; 条件段: 11位=区间, 其余按5位单点 */
    if (s_edit.step == EDIT_STEP_ACTION)
    {
        return 7U;
    }
    return (uint8_t)((s_edit.digit_count == 11U) ? 11U : 5U);
}

/*--------------------------------------------------------------
 * 函数名称：InputMaxDigits
 * 功能说明：当前阶段允许输入的最大位数(区别于NeedDigitCount的解析位数):
 *           动作段7位, 其余阶段5位.
 *--------------------------------------------------------------*/
static uint8_t InputMaxDigits(void)
{
    /* 动作段最多7位; 条件段最多11位(5位起 + '-' + 5位止) */
    return (s_edit.phase == EDIT_ACTION) ? 7U : 11U;
}

/*--------------------------------------------------------------
 * 函数名称：CondInputComplete
 * 功能说明：条件段是否已输完: 5位=单点, 11位=区间
 *--------------------------------------------------------------*/
static uint8_t CondInputComplete(void)
{
    return (uint8_t)((s_edit.digit_count == 5U) || (s_edit.digit_count == 11U));
}

/*--------------------------------------------------------------
 * 函数名称：HandleRangeKey
 * 功能说明：区间分隔符'-'的统一处理（按钮按下 / 文本控件输入两种入口）
 *           条件区已输入 1~5 位时补齐到 5 位并插入区间标记(0xFF)
 *--------------------------------------------------------------*/
static void HandleRangeKey(void)
{
    if (s_edit.phase != EDIT_COND)
    {
        DebugPrintf("[LOGIC] range ignored: not in cond phase\r\n");
        return;
    }
    if ((s_edit.digit_count == 0U) || (s_edit.digit_count > 5U))
    {
        DebugPrintf("[LOGIC] range ignored: digits=%d (need 1..5)\r\n",
                    (int)s_edit.digit_count);
        return;
    }

    /* 不足5位自动补0: 例如输入"0301"视为"03010" */
    while (s_edit.digit_count < 5U)
    {
        s_edit.digit_buf[s_edit.digit_count++] = 0U;
    }

    s_edit.digit_buf[5] = 0xFFU;   /* 区间标记, 显示为 '-' */
    s_edit.digit_count  = 6U;
    s_edit.step         = EDIT_STEP_NUMBER;
    s_edit.active       = 1;
    DebugPrintf("[LOGIC] range sep, input end addr\r\n");
}

/*--------------------------------------------------------------
 * 函数名称：ParseDigitBuf
 * 功能说明：将数字缓冲区解析为回路号、设备号和通道号。
 *           条件模式(5位): 前2位=回路号, 后3位=设备号, 通道=0
 *           动作模式(7位): 前2位=回路号, 中3位=设备号, 后2位=通道号(1-4/99)
 * 输入参数：loop_no - 输出回路号, dev_no - 输出设备号, channel - 输出通道号(可为NULL)
 * 返回值  ：1=解析成功, 0=位数不符
 *--------------------------------------------------------------*/
static uint8_t ParseDigitBuf(uint8_t *loop_no, uint16_t *dev_no,
                             uint16_t *dev_no_end, uint8_t *channel)
{
    uint8_t  need   = NeedDigitCount();
    uint8_t  l1;
    uint8_t  l2;
    uint16_t d1;
    uint16_t d2;
    uint16_t t;

    /* 位数不符直接失败 */
    if (s_edit.digit_count != need)
    {
        return 0;
    }

    /* 前2位 = 回路号 */
    l1 = (uint8_t)(s_edit.digit_buf[0] * 10 + s_edit.digit_buf[1]);
    /* 中3位 = 设备号 */
    d1 = (uint16_t)(s_edit.digit_buf[2] * 100 + s_edit.digit_buf[3] * 10 + s_edit.digit_buf[4]);

    /* ---------- 动作段: 7位, 后2位为通道号 ---------- */
    if (need == 7U)
    {
        *loop_no = l1;
        *dev_no  = d1;
        if (dev_no_end != NULL) { *dev_no_end = d1; }
        if (channel    != NULL) { *channel = (uint8_t)(s_edit.digit_buf[5] * 10 + s_edit.digit_buf[6]); }
        return 1;
    }

    /* ---------- 条件段: 5位单点 ---------- */
    if (need == 5U)
    {
        *loop_no = l1;
        *dev_no  = d1;
        if (dev_no_end != NULL) { *dev_no_end = d1; }   /* 单点 = 区间退化 */
        if (channel    != NULL) { *channel = 0; }
        return 1;
    }

    /* ---------- 条件段: 11位区间 = 5位 + '-' + 5位 ---------- */
    if ((need == 11U) && (s_edit.digit_buf[5] == 0xFFU))
    {
        l2 = (uint8_t)(s_edit.digit_buf[6] * 10 + s_edit.digit_buf[7]);
        d2 = (uint16_t)(s_edit.digit_buf[8] * 100 + s_edit.digit_buf[9] * 10 + s_edit.digit_buf[10]);

        /* 区间两端必须同回路 */
        if (l1 != l2)
        {
            DebugPrintf("[LOGIC] range reject: loop %d != %d\r\n", l1, l2);
            return 0;
        }

        /* 反向输入自动纠正 */
        if (d1 > d2)
        {
            t  = d1;
            d1 = d2;
            d2 = t;
        }

        *loop_no = l1;
        *dev_no  = d1;
        if (dev_no_end != NULL) { *dev_no_end = d2; }
        if (channel    != NULL) { *channel = 0; }
        return 1;
    }

    return 0;  /* 区间缺分隔符或位数异常 */
}

/*--------------------------------------------------------------
 * 函数名称：ClearDigitBuf
 * 功能说明：清空数字输入缓冲区。
 *--------------------------------------------------------------*/
static void ClearDigitBuf(void)
{
    memset(s_edit.digit_buf, 0, sizeof(s_edit.digit_buf));
    s_edit.digit_count = 0;
}

/*--------------------------------------------------------------
 * 函数名称：SensorSuffix
 * 功能说明：获取传感器类型的显示后缀字符串。
 * 输入参数：sensor_type - SensorType枚举值
 * 返回值  ：中文短名后缀，SENSOR_ANY返回空串
 *--------------------------------------------------------------*/
static const char *SensorSuffix(uint8_t sensor_type)
{
    switch (sensor_type)
    {
    case SENSOR_TEMPERATURE: return "温度";   /* 温度传感器 */
    case SENSOR_SMOKE:       return "烟雾";   /* 烟雾传感器 */
    case SENSOR_CARBON:      return "CO";     /* 一氧化碳传感器 */
    case SENSOR_HYDROGEN:    return "H2";     /* 氢气传感器 */
    case SENSOR_METHANE:     return "CH4";    /* 甲烷传感器 */
    case SENSOR_VOC:         return "VOC";    /* VOC传感器 */
    case SENSOR_PRESSURE:    return "压力";   /* 压力传感器 */
    default:                 return "";       /* 任意类型不显示后缀 */
    }
}

/*--------------------------------------------------------------
 * 函数名称：GetNationalCode
 * 功能说明：按回路号+设备号获取该设备的国标设备类型码，用于中文显示。
 * 输入参数：loop_no - 回路号(1~3), dev_no - 设备号
 * 返回值  ：国标设备类型码，无效返回0
 *--------------------------------------------------------------*/
static uint16_t GetNationalCode(uint8_t loop_no, uint16_t dev_no)
{
    switch (loop_no)
    {
    case 1:  return getPointTypeMixtureNationalCode((uint8_t)dev_no);
    case 2:  return MBusCtrl_GetNationalCode((uint8_t)dev_no);
    case 3:  return RS485Detect_GetNationalCode((uint8_t)dev_no);
    default: return 0U;
    }
}

/*--------------------------------------------------------------
 * 函数名称：BuildCondText
 * 功能说明：将条件转换为中文可读描述并写入buf。
 *           通配符(0xFF)显示"任意"；选了传感器类型显示"探测器+传感器名"；
 *           未选传感器按国标设备类型码显示设备中文名。
 * 输入参数：cond - 条件结构体, buf - 输出缓冲, size - 缓冲大小
 *--------------------------------------------------------------*/
static void BuildCondText(const Cond_t *cond, char *buf, uint8_t size)
{
    uint16_t code;
    const char *name;

    (void)size;
    if (cond->loop_no == LOGIC_WILDCARD && cond->dev_no == LOGIC_WILDCARD)
    {
        sprintf(buf, "任意设备");
    }
    else if (cond->sensor_type != SENSOR_ANY)
    {
        if (cond->loop_no == LOGIC_WILDCARD)
        {
            sprintf(buf, "任意回路%03d号探测器%s报警", cond->dev_no, SensorSuffix(cond->sensor_type));
        }
        else if (cond->dev_no == LOGIC_WILDCARD)
        {
            sprintf(buf, "%02d回路任意设备%s报警", cond->loop_no, SensorSuffix(cond->sensor_type));
        }
        else if (COND_IS_RANGE(cond))
        {
            sprintf(buf, "%02d回路%03d-%03d号探测器%s报警", cond->loop_no,
                    cond->dev_no, cond->dev_no_end, SensorSuffix(cond->sensor_type));
        }
        else
        {
            sprintf(buf, "%02d回路%03d号探测器%s报警", cond->loop_no, cond->dev_no, SensorSuffix(cond->sensor_type));
        }
    }
    else
    {
        if (cond->loop_no == LOGIC_WILDCARD)
        {
            sprintf(buf, "任意回路%03d号设备报警", cond->dev_no);
        }
        else if (cond->dev_no == LOGIC_WILDCARD)
        {
            sprintf(buf, "%02d回路任意设备报警", cond->loop_no);
        }
        else if (COND_IS_RANGE(cond))
        {
            /* 区间内可能混型号, 不查具体器件名 */
            sprintf(buf, "%02d回路%03d-%03d号设备报警", cond->loop_no,
                    cond->dev_no, cond->dev_no_end);
        }
        else
        {
            code = GetNationalCode(cond->loop_no, cond->dev_no);
            name = DeviceRegistry_GetNameByNationalCode(code);
            sprintf(buf, "%02d回路%03d号%s报警", cond->loop_no, cond->dev_no, name);
        }
    }
}

/*--------------------------------------------------------------
 * 函数名称：BuildActionText
 * 功能说明：将动作转换为中文可读描述并写入buf。
 *           动作1启动/0关闭，按"回路+设备号+设备+通道+启停"显示。
 *           通道99显示"全部通道"，具体通道显示"通道N"。
 * 输入参数：act - 动作结构体, buf - 输出缓冲, size - 缓冲大小
 *--------------------------------------------------------------*/
static void BuildActionText(const Action_t *act, char *buf, uint8_t size)
{
    const char *name;
    uint16_t code;
    const char *op;

    (void)size;
    op = (act->action != 0U) ? "启动" : "关闭";
    if (act->loop_no == LOGIC_WILDCARD)
    {
        sprintf(buf, "任意回路%03d号设备%s%s", act->dev_no,
                act->channel == 99U ? "全部通道" : "", op);
    }
    else if (act->dev_no == LOGIC_WILDCARD)
    {
        sprintf(buf, "%02d回路全部设备%s%s", act->loop_no,
                act->channel == 99U ? "全部通道" : "", op);
    }
    else
    {
        code = GetNationalCode(act->loop_no, act->dev_no);
        name = DeviceRegistry_GetNameByNationalCode(code);
        if (act->channel == 99U)
        {
            sprintf(buf, "%02d%03d号%s全部通道%s", act->loop_no, act->dev_no, name, op);
        }
        else
        {
            sprintf(buf, "%02d%03d号%s通道%d%s", act->loop_no, act->dev_no, name, act->channel, op);
        }
    }
}

/*--------------------------------------------------------------
 * 函数名称：CompleteConditionEx
 * 功能说明：完成当前条件的构造（解析5位编码为回路号+设备号），
 *           将其添加到条件数组，并添加对应TOK_COND Token到表达式。
 *           带传感器类型和报警级别参数（支持子级别条件）。
 *--------------------------------------------------------------*/
static void CompleteConditionEx(uint8_t sensor_type, uint8_t alarm_level)
{
    uint8_t idx;         /* 条件数组索引 */
    uint8_t loop_no;     /* 解析出的回路号 */
    uint16_t dev_no;     /* 解析出的起始设备号 */
    uint16_t dev_no_end; /* 解析出的结束设备号(单点==dev_no) */

    /* 解析条件编码: 5位单点 或 11位区间, 通道指针传NULL */
    if (!ParseDigitBuf(&loop_no, &dev_no, &dev_no_end, NULL))
    {
        return;  /* 位数不足或区间非法，无法完成 */
    }

    /* 检查是否有空间添加新条件 */
    if (s_edit.rule.cond_count >= LOGIC_COND_MAX)
    {
        return;
    }

    /* 检查是否有空间添加新Token */
    if (s_edit.rule.token_count >= LOGIC_TOKEN_MAX)
    {
        return;
    }

    /* 将当前条件添加到条件数组 */
    idx = s_edit.rule.cond_count;
    s_edit.rule.conditions[idx].loop_no     = loop_no;
    s_edit.rule.conditions[idx].dev_no      = dev_no;
    s_edit.rule.conditions[idx].dev_no_end  = dev_no_end;
    s_edit.rule.conditions[idx].sensor_type = sensor_type;
    s_edit.rule.conditions[idx].alarm_level = alarm_level;

    s_edit.rule.cond_count++;

    /* 添加对应条件Token到表达式 */
    s_edit.rule.tokens[s_edit.rule.token_count].type     = TOK_COND;
    s_edit.rule.tokens[s_edit.rule.token_count].cond_idx = idx;
    s_edit.rule.token_count++;

    DebugPrintf("[LOGIC] +Cond L%d D%03d-%03d type=0x%02X lvl=%d\r\n",
                loop_no, dev_no, dev_no_end, sensor_type, alarm_level);

    /* 重置编辑步骤，准备接收下一个输入（仍在条件区） */
    s_edit.step = EDIT_STEP_NONE;
    ClearDigitBuf();
}

/*--------------------------------------------------------------
 * 函数名称：CompleteCondition
 * 功能说明：完成当前条件的构造（任意报警版本）。
 *           传感器类型固定为SENSOR_ANY(任意类型报警)。
 *--------------------------------------------------------------*/
static void CompleteCondition(void)
{
    CompleteConditionEx(SENSOR_ANY, LV_ANY);
}

/*--------------------------------------------------------------
 * 函数名称：AddOperator
 * 功能说明：向表达式中添加操作符Token。
 * 输入参数：type - Token类型(TOK_AND, TOK_OR, TOK_LPAREN, TOK_RPAREN)
 *--------------------------------------------------------------*/
static void AddOperator(uint8_t type)
{
    /* 检查是否有空间添加新Token */
    if (s_edit.rule.token_count >= LOGIC_TOKEN_MAX)
    {
        return;
    }

    /* 添加操作符Token */
    s_edit.rule.tokens[s_edit.rule.token_count].type     = type;
    s_edit.rule.tokens[s_edit.rule.token_count].cond_idx = 0;
    s_edit.rule.token_count++;

    /* 重置编辑步骤（操作符后必须输入条件） */
    s_edit.step = EDIT_STEP_NONE;
    ClearDigitBuf();
}

/*--------------------------------------------------------------
 * 函数名称：DeleteLastToken
 * 功能说明：从表达式中删除最后一个Token。
 *           如果最后一个Token是条件，同时删除对应条件。
 *--------------------------------------------------------------*/
static void DeleteLastToken(void)
{
    /* 检查是否有Token可删除 */
    if (s_edit.rule.token_count == 0)
    {
        return;
    }

    /* 递减Token计数 */
    s_edit.rule.token_count--;

    /* 若删除的Token是条件，则条件也删除对应 */
    if (s_edit.rule.tokens[s_edit.rule.token_count].type == TOK_COND)
    {
        if (s_edit.rule.cond_count > 0)
        {
            s_edit.rule.cond_count--;
        }
    }

    /* 清空被删除的Token */
    s_edit.rule.tokens[s_edit.rule.token_count].type     = 0;
    s_edit.rule.tokens[s_edit.rule.token_count].cond_idx = 0;
}

/*--------------------------------------------------------------
 * 函数名称：DeleteLastAction
 * 功能说明：删除最后一个已输入的动作（动作区退格用）。
 *--------------------------------------------------------------*/
static void DeleteLastAction(void)
{
    if (s_edit.rule.action_count > 0)
    {
        s_edit.rule.action_count--;
        memset(&s_edit.rule.actions[s_edit.rule.action_count], 0, sizeof(Action_t));

        if (s_edit.rule.action_count == 0)
        {
            s_edit.has_action = 0;  /* 已无动作 */
        }
    }
}

/*--------------------------------------------------------------
 * 函数名称：SetAction
 * 功能说明：设置执行动作（解析7位编码为回路号+设备号+通道号）。
 *           动作方仅支持控制回路2, 通道1-4=具体通道/99=全部通道。
 *           多动作场景下每按一次&或确认键调用一次。
 *--------------------------------------------------------------*/
static void SetAction(void)
{
    uint8_t loop_no;    /* 控制回路号 */
    uint16_t dev_no;     /* 设备号 */
    uint8_t channel;     /* 输出通道号 */

    /* 解析7位编码(动作模式: 回路2+设备3+通道2) */
    if (!ParseDigitBuf(&loop_no, &dev_no, NULL, &channel))
    {
        return;  /* 位数不足，无法设置 */
    }

    /* 动作方仅支持控制回路2 */
    if (loop_no != 2U)
    {
        DebugPrintf("[LOGIC] +Act reject loop=%d (动作方仅支持控制回路2)\r\n", loop_no);
        ClearDigitBuf();
        return;
    }

    /* 通道号校验: 1-4=具体通道, 99=全部通道 */
    if (((channel < 1U) || (channel > 4U)) && (channel != 99U))
    {
        DebugPrintf("[LOGIC] +Act reject ch=%d (通道1-4或99)\r\n", channel);
        ClearDigitBuf();
        return;
    }

    /* 检查是否有空间添加动作 */
    if (s_edit.rule.action_count >= LOGIC_ACTION_MAX)
    {
        return;
    }

    /* 设置动作 */
    s_edit.rule.actions[s_edit.rule.action_count].loop_no  = loop_no;
    s_edit.rule.actions[s_edit.rule.action_count].dev_no   = dev_no;
    s_edit.rule.actions[s_edit.rule.action_count].channel  = channel;
    s_edit.rule.actions[s_edit.rule.action_count].action   = 1;  /* 默认启动 */
    s_edit.rule.action_count++;

    DebugPrintf("[LOGIC] +Act L%d D%03d ch=%d start\r\n", loop_no, dev_no, channel);

    /* 标记已设置动作，回到动作区空闲步骤等待下一个动作 */
    s_edit.has_action = 1;
    s_edit.step = EDIT_STEP_NONE;
    ClearDigitBuf();
}

/*--------------------------------------------------------------
 * 函数名称：SaveRule
 * 功能说明：保存当前编辑的规则到RAM中。
 *           未设置动作时拒绝保存（不再默认填入设备1）。
 *--------------------------------------------------------------*/
static void SaveRule(void)
{
    uint8_t create_ret;  /* LogicRule_Create return code for trace */

    /* 若当前在NUMBER步骤且有5位输入，先完成条件 */
    if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
    {
        CompleteCondition();
    }

    /* 若当前在ACTION步骤且有7位输入，先完成动作 */
    if (s_edit.step == EDIT_STEP_ACTION && (s_edit.digit_count == 7U))
    {
        SetAction();
    }

    /* 未设置动作: 拒绝保存(不再默认填入设备1), 保留编辑状态供补正 */
    if (s_edit.rule.action_count == 0)
    {
        DebugPrintf("[LOGIC] SaveRule FAIL: no action set\r\n");
        return;
    }

    /* 规则所属分区: 0=全局规则(标准编程入口), 1-2=分区限定; 超出范围拒绝保存 */
    if (s_edit.zone > (uint8_t)FIRE_ZONE_MAX)
    {
        DebugPrintf("[LOGIC] SaveRule FAIL: zone not selected\r\n");
        sprintf(s_disp_buf, "请先在分区选择页选择分区");
        SetTextValue(LOGIC_SCREEN_EDIT, LOGIC_TXT_PREVIEW, (uint8_t *)s_disp_buf);
        return;
    }
    s_edit.rule.zone = s_edit.zone;
    /* A8-3: 执行模式保留用户在动作段用'#'键选择的值(默认START_ALL=0) */
    s_edit.rule.enable = 1;
    s_edit.rule.rule_id = AllocRuleId();

    /* 保存规则到RAM表格 */
    create_ret = LogicRule_Create(s_edit.rule.rule_id, &s_edit.rule);

    if (create_ret == 0U)
    {
        /* 保存成功: 持久化到Flash供下次启动加载 */
        LogicRule_SaveAll();

        DebugPrintf("[LOGIC] SaveRule id=%d OK\r\n", s_edit.rule.rule_id);

        /* 保存成功: 在编辑页预览窗口显示提示, 不跳转列表页(停留编辑页继续编辑) */
        sprintf(s_disp_buf, "保存成功: 规则ID%d已存入", s_edit.rule.rule_id);
        s_save_msg_until = HAL_GetTick() + 3000U;  /* 提示停留3秒, 防止实时预览立即覆盖 */
        SetTextValue(LOGIC_SCREEN_EDIT, LOGIC_TXT_PREVIEW, (uint8_t *)s_disp_buf);

        /* 清空编辑状态, 准备下一次编辑 */
        memset(&s_edit.rule, 0, sizeof(LogicRule_t));
        s_edit.step = EDIT_STEP_NONE;
        s_edit.phase = EDIT_COND;
        ClearDigitBuf();
        s_edit.has_action = 0;
        s_edit.active = 0;

        /* 不跳转列表页45, 停留编辑页43继续编辑(用户可手动进入45查看) */
        /* SetScreen(LOGIC_SCREEN_LIST); */
    }
    else
    {
        /* 保存失败: 不清空编辑状态, 保留用户输入供修正
         * create_ret: 1=ID重复 2=表满 3=结构无效(条件/动作/设备号非法) */
        DebugPrintf("[LOGIC] SaveRule id=%d FAIL ret=%d (1=id 2=full 3=invalid)\r\n",
                    s_edit.rule.rule_id, create_ret);

        /* 在编辑页预览窗口显示失败原因, 方便用户诊断 */
        switch (create_ret)
        {
            case 1:
                sprintf(s_disp_buf, "保存失败: ID%d重复", s_edit.rule.rule_id);
                break;
            case 2:
                sprintf(s_disp_buf, "保存失败: 规则表已满(最多%d条)", LOGIC_RULE_MAX);
                break;
            case 3:
                sprintf(s_disp_buf, "保存失败: 设备未划入本分区或编码非法");
                break;
            default:
                sprintf(s_disp_buf, "保存失败: 未知错误%d", create_ret);
                break;
        }
        s_save_msg_until = HAL_GetTick() + 3000U;  /* 提示停留3秒 */
        SetTextValue(LOGIC_SCREEN_EDIT, LOGIC_TXT_PREVIEW, (uint8_t *)s_disp_buf);
    }
}

/*--------------------------------------------------------------
 * 第三部分：表达式显示构建
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：BuildExprDisplay
 * 功能说明：根据当前编辑的规则Token序列，转换为可读字符串并存入
 *           s_disp_buf缓冲区，供屏幕显示。
 *           显示格式: "01001温度 & 01002 = 01001 & 02003"
 *                    若正在输入步骤，附加当前输入的数字
 *--------------------------------------------------------------*/
static void BuildExprDisplay(void)
{
    uint8_t i;        /* Token循环计数器 */
    uint8_t j;        /* 动作循环计数器 */
    uint8_t idx;      /* 条件数组索引 */
    char    temp[48]; /* 临时格式化缓冲区 */

    s_disp_buf[0] = '\0';
    /* 编辑页显示当前分区, 便于确认规则归属 */
    if ((s_edit.zone >= 1U) && (s_edit.zone <= (uint8_t)FIRE_ZONE_MAX))
    {
        sprintf(temp, "[气灭分区%u] ", (unsigned)s_edit.zone);
        strcat(s_disp_buf, temp);
    }
    else
    {
        strcat(s_disp_buf, "[任意] ");  /* V7: 标准编程入口, 规则不限分区 */
    }

    /* 遍历所有Token */
    for (i = 0; i < s_edit.rule.token_count; i++)
    {
        switch (s_edit.rule.tokens[i].type)
        {
        case TOK_COND:
            /* 条件Token，显示5位编码+传感器后缀 */
            idx = s_edit.rule.tokens[i].cond_idx;
            if (idx < s_edit.rule.cond_count)
            {
                if (COND_IS_RANGE(&s_edit.rule.conditions[idx]))
                {
                    sprintf(temp, "%02d%03d-%03d%s",
                            s_edit.rule.conditions[idx].loop_no,
                            s_edit.rule.conditions[idx].dev_no,
                            s_edit.rule.conditions[idx].dev_no_end,
                            SensorSuffix(s_edit.rule.conditions[idx].sensor_type));
                }
                else
                {
                    sprintf(temp, "%02d%03d%s",
                            s_edit.rule.conditions[idx].loop_no,
                            s_edit.rule.conditions[idx].dev_no,
                            SensorSuffix(s_edit.rule.conditions[idx].sensor_type));
                }
                strcat(s_disp_buf, temp);
            }
            break;

        case TOK_AND:
            strcat(s_disp_buf, " & ");
            break;

        case TOK_OR:
            strcat(s_disp_buf, " | ");
            break;

        case TOK_LPAREN:
            strcat(s_disp_buf, "(");
            break;

        case TOK_RPAREN:
            strcat(s_disp_buf, ")");
            break;

        default:
            break;
        }
    }

    /* 显示等号前缀（进入动作区即显示，等号后接动作区内容） */
    if (s_edit.phase == EDIT_ACTION)
    {
        strcat(s_disp_buf, " = ");
    }

    /* 显示所有已确认动作（多动作用&连接） */
    if (s_edit.has_action && s_edit.rule.action_count > 0)
    {
        for (j = 0; j < s_edit.rule.action_count; j++)
        {
            if (j > 0)
            {
                strcat(s_disp_buf, " & ");
            }
            /* 动作显示7位编码(回路2+设备3+通道2) */
            sprintf(temp, "%02d%03d%02d",
                    s_edit.rule.actions[j].loop_no,
                    s_edit.rule.actions[j].dev_no,
                    s_edit.rule.actions[j].channel);
            strcat(s_disp_buf, temp);
        }

        /* 动作区已确认动作且无待定输入时, 末尾补&分隔符:
         * 按下&键确认动作后立即显示&, 与条件区"按&即显示"行为一致 */
        if (s_edit.phase == EDIT_ACTION &&
            s_edit.digit_count == 0 &&
            s_edit.step == EDIT_STEP_NONE)
        {
            strcat(s_disp_buf, " & ");
        }
    }

    /* A8-3: 动作段显示当前执行模式标签(START_ALL为默认不显示, 保持向后兼容) */
    if (s_edit.phase == EDIT_ACTION)
    {
        switch (s_edit.rule.exec_mode)
        {
        case EXEC_START_PART: strcat(s_disp_buf, " [部分启动]"); break;
        case EXEC_STOP_ALL:   strcat(s_disp_buf, " [全部停止]"); break;
        case EXEC_STOP_PART:  strcat(s_disp_buf, " [部分停止]"); break;
        default:              break;
        }
    }

    /* 显示当前输入的数字（条件区跟在条件后，动作区跟在等号/动作后） */
    if ((s_edit.step == EDIT_STEP_NUMBER || s_edit.step == EDIT_STEP_ACTION)
        && s_edit.digit_count > 0)
    {
        /* 动作区输入第2个及以后的动作时, 前面补&分隔符, 与已确认动作分开显示 */
        if (s_edit.phase == EDIT_ACTION && s_edit.rule.action_count > 0)
        {
            strcat(s_disp_buf, " & ");
        }
        for (j = 0; j < s_edit.digit_count; j++)
        {
            if (s_edit.digit_buf[j] == 0xFFU)
            {
                strcat(s_disp_buf, "-");   /* 区间分隔符: 不能按十进制打印, 否则显示255 */
            }
            else
            {
                sprintf(temp, "%d", s_edit.digit_buf[j]);
                strcat(s_disp_buf, temp);
            }
        }
        strcat(s_disp_buf, "_");  /* 输入中标记: 按&或确认键确认后消失 */
    }
    else if (s_edit.phase == EDIT_ACTION && s_edit.rule.action_count == 0)
    {
        /* 已进入动作区但还没有任何动作，显示等待输入提示 */
        strcat(s_disp_buf, "?");
    }
}

/*--------------------------------------------------------------
 * 函数名称：BuildRuleDisplay
 * 功能说明：将指定规则转换为可读字符串，用于列表页显示。
 *--------------------------------------------------------------*/
static void BuildRuleDisplay(const LogicRule_t *rule, char *buf, uint8_t size)
{
    uint8_t i;
    uint8_t j;
    uint8_t idx;
    char    temp[48];

    buf[0] = '\0';

    if (rule == NULL || rule->rule_id == 0)
    {
        return;
    }
    /* 列表页标注规则所属分区 */
    if ((rule->zone >= 1U) && (rule->zone <= (uint8_t)FIRE_ZONE_MAX))
    {
        sprintf(temp, "[气灭分区%u] ", (unsigned)rule->zone);
        strcat(buf, temp);
    }
    else
    {
        strcat(buf, "[任意] ");  /* V7: 不限分区的规则 */
    }

    /* 遍历所有Token，构造触发条件表达式 */
    for (i = 0; i < rule->token_count && strlen(buf) < (size_t)(size - 20); i++)
    {
        switch (rule->tokens[i].type)
        {
        case TOK_COND:
            idx = rule->tokens[i].cond_idx;
            if (idx < rule->cond_count)
            {
                BuildCondText(&rule->conditions[idx], temp, sizeof(temp));
                strcat(buf, temp);
            }
            break;

        case TOK_AND:
            strcat(buf, "与");
            break;

        case TOK_OR:
            strcat(buf, "或");
            break;

        case TOK_LPAREN:
            strcat(buf, "(");
            break;

        case TOK_RPAREN:
            strcat(buf, ")");
            break;

        default:
            break;
        }
    }

    /* 添加动作显示（多动作用"和"连接） */
    if (rule->action_count > 0)
    {
        strcat(buf, "触发");
        for (j = 0; j < rule->action_count; j++)
        {
            if (j > 0)
            {
                strcat(buf, "和");
            }
            BuildActionText(&rule->actions[j], temp, sizeof(temp));
            strcat(buf, temp);
        }
    }

    /* 若规则被禁用，添加标记 */
    if (rule->enable == 0)
    {
        strcat(buf, " [禁用]");
    }
}

/*--------------------------------------------------------------
 * 第四部分：API实现 - 屏幕事件处理
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：HandleSensorButton
 * 功能说明：处理传感器快捷按钮（条件区子级别确认）。
 *           NUMBER步骤输入5位编码后按传感器按钮，
 *           添加带子级别的条件（如01001温度=高报警）。
 * 输入参数：sensor_type - SensorType枚举值
 *--------------------------------------------------------------*/
static void HandleSensorButton(uint8_t sensor_type)
{
    /* 1. 正在输入且位数已够: 正常完成条件 */
    if (s_edit.phase == EDIT_COND &&
        s_edit.step == EDIT_STEP_NUMBER &&
        CondInputComplete())
    {
        /* 传感器按钮默认按高报警级别确认 */
        CompleteConditionEx(sensor_type, LV_HIGH);
        return;
    }

    /* 2. 刚用 ')' 完成条件后补按传感器类型: 回填最后一个条件的类型。
     *    典型操作: ( 03001-03010 ) 温度
     *    ')' 键会先用 SENSOR_ANY 完成条件, 之后按传感器按钮在此修正。
     *    判据: 条件区空闲 + 已有条件 + token 末位是右括号。
     *    此路径原本按传感器无效(数字缓冲为空), 因此不会破坏既有操作。 */
    if ((s_edit.phase == EDIT_COND) &&
        (s_edit.digit_count == 0U) &&
        (s_edit.rule.cond_count > 0U) &&
        (s_edit.rule.token_count > 0U) &&
        (s_edit.rule.tokens[s_edit.rule.token_count - 1U].type == TOK_RPAREN))
    {
        uint8_t idx = (uint8_t)(s_edit.rule.cond_count - 1U);

        s_edit.rule.conditions[idx].sensor_type = sensor_type;
        s_edit.rule.conditions[idx].alarm_level = LV_HIGH;

        DebugPrintf("[LOGIC] cond%d type -> 0x%02X (L%d D%03d-%03d)\r\n",
                    (int)idx, sensor_type,
                    s_edit.rule.conditions[idx].loop_no,
                    s_edit.rule.conditions[idx].dev_no,
                    s_edit.rule.conditions[idx].dev_no_end);
        return;
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicScreen_OnButton
 * 功能说明：处理来自大彩屏的按钮按下事件。
 *           实现条件区/动作区双阶段编辑状态机。
 *           5位编码输入: 数字按钮累计5位,
 *           条件区: #/传感器按钮确认条件, 动作区: &分隔多动作, 确认键保存。
 *--------------------------------------------------------------*/
/*--------------------------------------------------------------
 * 函数名称：LogicScreen_SetEntryZone
 * 功能说明：注入本次编辑的入口分区(0=全局规则/1=分区1/2=分区2)。
 *           必须在跳转到编辑页之前调用, 因为密码页验证通过后
 *           只按 target_screen 回跳, 不携带任何上下文。
 * 输入参数：zone - 0/1/2, 超出范围按0(全局)处理
 *--------------------------------------------------------------*/
void LogicScreen_SetEntryZone(uint8_t zone)
{
    if (zone > (uint8_t)FIRE_ZONE_MAX)
    {
        zone = 0U;
    }
    s_edit.zone = zone;
    /* 换入口视为新编辑会话: 清空残留, 防止跨入口串数据 */
    memset(&s_edit.rule, 0, sizeof(LogicRule_t));
    s_edit.step = EDIT_STEP_NONE;
    s_edit.phase = EDIT_COND;
    ClearDigitBuf();
    s_edit.has_action = 0;
    s_edit.active = 1;
    DebugPrintf("[LOGIC] entry zone=%d\r\n", (int)zone);
}

static uint8_t LogicListCollectIds(uint8_t *ids)
{
    uint8_t i;
    uint8_t k;
    uint8_t n = 0U;
    LogicRule_t *tbl = LogicExpr_GetTable();

    for (i = 0U; i < LOGIC_RULE_MAX; i++)
    {
        if (tbl[i].rule_id != 0U) { ids[n++] = tbl[i].rule_id; }
    }

    /* 升序: 1号在最前 */
    for (i = 0U; i < n; i++)
    {
        for (k = (uint8_t)(i + 1U); k < n; k++)
        {
            if (ids[k] < ids[i])
            {
                uint8_t tmp = ids[i];
                ids[i] = ids[k];
                ids[k] = tmp;
            }
        }
    }
    return n;
}


static void LogicListShowHint(const char *text)
{
    if ((text == NULL) || (text[0] == 0))
    {
        clearTextValue(LOGIC_SCREEN_LIST, LOGIC_TXT_LIST_HINT);
    }
    else
    {
        SetTextValue(LOGIC_SCREEN_LIST, LOGIC_TXT_LIST_HINT, (uint8_t *)text);
    }
}

static void LogicListDeleteRule(uint8_t rule_id)
{
    LogicRule_t *tbl = LogicExpr_GetTable();
    RtRuntime   *rt  = LogicExpr_GetRuntime();
    uint8_t slot = 0xFFU;
    uint8_t i;
    uint8_t j;
    char    hint[48];

    for (i = 0U; i < LOGIC_RULE_MAX; i++)
    {
        if (tbl[i].rule_id == rule_id) { slot = i; break; }
    }
    if (slot == 0xFFU) { LogicListShowHint("规则不存在"); return; }


    /* 已执行的动作先停, 否则删掉规则后输出一直保持 */
    if (rt[slot].state == RT_EXECUTED)
    {
        for (j = 0U; j < tbl[slot].action_count; j++)
        {
            if (rt[slot].action_done[j] != 0U)
            {
                (void)LogicDev_Control(tbl[slot].actions[j].loop_no,
                                       (uint8_t)tbl[slot].actions[j].dev_no,
                                       tbl[slot].actions[j].channel, 0U);
                rt[slot].action_done[j] = 0U;
            }
        }
        DebugPrintf("[LOGIC] delete: rule %d stopped\r\n", rule_id);
    }

    (void)LogicRule_Delete(rule_id);
    LogicRule_Renumber();          /* 删除后编号重排为 1..N, 避免出现空号 */
    (void)LogicRule_SaveAll();

    s_list_selected = 0xFFU;
    s_list_armed = 0U;
    s_last_list_count = 0xFFU;

    sprintf(hint, "已删除#%d号规则", rule_id);
    LogicListShowHint(hint);
    DebugPrintf("[LOGIC] delete rule %d ok, left=%d\r\n", rule_id, (int)LogicRule_GetCount());
}

void LogicScreen_OnButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    /* 只处理编辑页(43)/列表页(45)/分区选择页(85) */
    if (screen_id != LOGIC_SCREEN_EDIT && screen_id != LOGIC_SCREEN_LIST &&
        screen_id != LOGIC_SCREEN_ZONESEL)
    {
        return;
    }

    /* 只处理按下事件 */
    if (state != 1)
    {
        return;
    }

   DebugPrintf("[LOGIC-KEY] scr=%d ctrl=%d\r\n", screen_id, control_id);

    /* 画面45: 先处理删除控件(24~33), 避免与画面43 的 24~29 传感器键冲突 */
    if (screen_id == LOGIC_SCREEN_LIST)
    {
        uint8_t  ids[LOGIC_RULE_MAX];
        uint8_t  n;
        uint16_t idx;
        char     hint[40];

        if ((control_id >= LOGIC_BTN_LIST_SEL_BASE) &&
            (control_id < (LOGIC_BTN_LIST_SEL_BASE + (uint16_t)LOGIC_LIST_PAGE_SIZE)))
        {
            n   = LogicListCollectIds(ids);
            idx = (uint16_t)((uint16_t)s_list_page * LOGIC_LIST_PAGE_SIZE +
                             (control_id - LOGIC_BTN_LIST_SEL_BASE));
            s_list_armed = 0U;

            if (idx < n)
            {
                s_list_selected = (uint8_t)(control_id - LOGIC_BTN_LIST_SEL_BASE);

                sprintf(hint, "将删除#%d号规则", ids[idx]);
                LogicListShowHint(hint);
            }
            else
            {
                s_list_selected = 0xFFU;
                LogicListShowHint("该行无规则");
            }
            return;
        }

        if (control_id == LOGIC_BTN_LIST_DELETE)
        {
            if (s_list_selected == 0xFFU)
            {
                LogicListShowHint("请先选择规则");
                return;
            }

            n   = LogicListCollectIds(ids);
            idx = (uint16_t)((uint16_t)s_list_page * LOGIC_LIST_PAGE_SIZE + s_list_selected);
            if (idx >= n)
            {
                LogicListShowHint("该行无规则");
                s_list_selected = 0xFFU;
                return;
            }

            /* 第一次: 进入待确认; 超过窗口则重新计时 */
            if ((s_list_armed == 0U) ||
                ((uint32_t)(HAL_GetTick() - s_list_arm_tick) >= LOGIC_LIST_CONFIRM_MS))
            {
                s_list_armed = 1U;
                s_list_arm_tick = HAL_GetTick();
                sprintf(hint, "再按删除确认#%d", ids[idx]);
                LogicListShowHint(hint);
                return;
            }

            LogicListDeleteRule(ids[idx]);
            return;
        }
    }

    /* 画面85: 先选分区, 再进画面43编程(避免与43的数字键1/2冲突) */
    if (screen_id == LOGIC_SCREEN_ZONESEL)
    {
        if (control_id == 1U || control_id == 2U)
        {
            s_edit.zone = (uint8_t)control_id;
            /* 切区视为新的编辑会话: 清空残留, 防止跨区串数据 */
            memset(&s_edit.rule, 0, sizeof(LogicRule_t));
            s_edit.step = EDIT_STEP_NONE;
            s_edit.phase = EDIT_COND;
            ClearDigitBuf();
            s_edit.has_action = 0;
            s_edit.active = 1;
            DebugPrintf("[LOGIC] zone select -> %d\r\n", (int)s_edit.zone);
            SwitchCurrentScreenId(LOGIC_SCREEN_EDIT);
        }
        return;  /* 85 其余控件(含返回键266)不处理, 由屏端自行跳转 */
    }
    switch (control_id)
    {
    /* === 数字按钮(0-9): 累计到5位编码缓冲区 === */
    case 1: case 2: case 3: case 4: case 5:
    case 6: case 7: case 8: case 9: case 10:
        {
            uint8_t digit = (control_id == 10) ? 0 : control_id;

            /* 最多输入当前阶段所需位数: 条件5位(单点)/11位(区间), 动作7位 */
            if (s_edit.digit_count < InputMaxDigits())
            {
                s_edit.digit_buf[s_edit.digit_count++] = digit;

                /* 根据当前步骤设置编辑状态 */
                if (s_edit.step == EDIT_STEP_NONE)
                {
                    /* 空闲状态: 按编辑阶段决定输入类型 */
                    if (s_edit.phase == EDIT_ACTION)
                    {
                        /* 动作区: 开始输入动作编码 */
                        s_edit.step = EDIT_STEP_ACTION;
                    }
                    else
                    {
                        /* 条件区: 开始输入条件编码 */
                        s_edit.step = EDIT_STEP_NUMBER;
                    }
                    s_edit.active = 1;
                }
                /* NUMBER/ACTION步骤下继续输入对应编码 */
            }
            else if (s_edit.step == EDIT_STEP_NUMBER)
            {
                DebugPrintf("[LOGIC] input full %d digits, press sensor/&/# to confirm\r\n",
                            (int)s_edit.digit_count);
            }
            else
            {
                DebugPrintf("[LOGIC] input full %d digits\r\n", (int)s_edit.digit_count);
            }
        }
        break;

    /* === '-' 区间分隔符: 控件17(主) / 控件30(备用) === */
    case LOGIC_BTN_RANGE:
    case 30:
        HandleRangeKey();
        break;

    /* === '#' 分隔符: 确认当前条件编码(单点/区间) === */
    case LOGIC_BTN_HASH:
        if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
        {
            CompleteCondition();
        }
        /* A8-3: 动作段且无待定数字时, '#'键循环切换执行模式(0全启/1部启/2全停/3部停) */
        else if (s_edit.phase == EDIT_ACTION && s_edit.digit_count == 0U)
        {
            s_edit.rule.exec_mode = (uint8_t)((s_edit.rule.exec_mode + 1U) & 0x03U);
            DebugPrintf("[LOGIC] exec_mode -> %d (0全启/1部启/2全停/3部停)\r\n",
                        s_edit.rule.exec_mode);
        }
        break;

    /* === '&' 逻辑与 / 动作分隔符 === */
    case LOGIC_BTN_AND:
        if (s_edit.phase == EDIT_ACTION)
        {
            /* 动作区: & 作为多动作分隔符，确认当前编码为动作(7位) */
            if (s_edit.step == EDIT_STEP_ACTION && (s_edit.digit_count == 7U))
            {
                SetAction();
            }
        }
        else
        {
            /* 条件区: & 作为逻辑与运算符 */
            if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
            {
                CompleteCondition();
            }
            AddOperator(TOK_AND);
        }
        break;

    /* === '|' 逻辑或 === */
    case LOGIC_BTN_OR:
        if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
        {
            CompleteCondition();
        }
        AddOperator(TOK_OR);
        break;

    /* === '(' 左括号 === */
    case LOGIC_BTN_LPAREN:
        if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
        {
            CompleteCondition();
        }
        AddOperator(TOK_LPAREN);
        break;

    /* === ')' 右括号 === */
    case LOGIC_BTN_RPAREN:
        if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
        {
            CompleteCondition();
        }
        AddOperator(TOK_RPAREN);
        break;

    /* === '=' 分隔符，进入动作区 === */
    case LOGIC_BTN_EQUAL:
        if (s_edit.step == EDIT_STEP_NUMBER && CondInputComplete())
        {
            CompleteCondition();
        }
        /* 只有有条件时才进入动作选择 */
        if (s_edit.rule.cond_count > 0)
        {
            s_edit.phase = EDIT_ACTION;   /* 切换到动作区 */
            s_edit.step = EDIT_STEP_ACTION;
            ClearDigitBuf();
        }
        break;

    /* === 删除: 优先删数字，无数字时删Token/动作 === */
    case LOGIC_BTN_DELETE:
        if (s_edit.digit_count > 0)
        {
            /* 删除最后一位数字 */
            s_edit.digit_count--;
            s_edit.digit_buf[s_edit.digit_count] = 0;
        }
        else if (s_edit.phase == EDIT_ACTION && s_edit.rule.action_count > 0)
        {
            /* 动作区无数字: 删除最后一个动作 */
            DeleteLastAction();
        }
        else
        {
            /* 条件区无数字: 删除最后一个Token */
            DeleteLastToken();
            if (s_edit.rule.token_count == 0)
            {
                s_edit.step = EDIT_STEP_NONE;
            }
        }
        break;

    /* === 确认/保存: 一次确认即保存规则(待定的5位条件/7位动作由
     *     SaveRule内部先完成), 多动作之间用&键分隔确认 === */
    case LOGIC_BTN_CONFIRM:
        if (s_edit.active)
        {
            SaveRule();
        }
        break;

    /* === 新建规则: 清空编辑状态 ===
     *    只清s_edit.rule, 分区s_edit.zone保持不变: 分区由画面85决定 === */
    case LOGIC_BTN_NEW_RULE:
        memset(&s_edit.rule, 0, sizeof(LogicRule_t));
        s_edit.step = EDIT_STEP_NONE;
        s_edit.phase = EDIT_COND;
        ClearDigitBuf();
        s_edit.has_action = 0;
        s_edit.active = 1;
        break;

    /* === 取消: 清空编辑状态(分区s_edit.zone不清, 仍由画面85决定) === */
    case LOGIC_BTN_CANCEL:
        memset(&s_edit.rule, 0, sizeof(LogicRule_t));
        s_edit.step = EDIT_STEP_NONE;
        s_edit.phase = EDIT_COND;
        ClearDigitBuf();
        s_edit.has_action = 0;
        s_edit.active = 0;
        break;

    /* === 控件23: 编辑页=查看逻辑占位, 列表页=下一页 ===
     *   编辑页43: 查看逻辑按钮, 跳转由屏幕固件处理(此处预留空操作)
     *   列表页45: 下一页, 页码+1(不超过最大页) */
    case LOGIC_BTN_NEXT_PAGE:
        if (screen_id == LOGIC_SCREEN_LIST)
        {
            uint8_t total_rules = LogicRule_GetCount();
            uint8_t total_pages = (uint8_t)((total_rules + LOGIC_LIST_PAGE_SIZE - 1) / LOGIC_LIST_PAGE_SIZE);
            if (total_pages == 0) total_pages = 1;
            if (s_list_page < total_pages - 1)
            {
                s_list_page++;
            }
        }
        s_list_selected = 0xFFU;   /* 翻页清选中 */
        s_list_armed = 0U;
        LogicListShowHint("");
        break;

    /* === 控件22: 上一页(仅列表页45响应) ===
     *   页码-1, 不低于0 */
    case LOGIC_BTN_PREV_PAGE:
        if (screen_id == LOGIC_SCREEN_LIST)
        {
            if (s_list_page > 0)
            {
                s_list_page--;
            }
        }
        s_list_selected = 0xFFU;   /* 翻页清选中 */
        s_list_armed = 0U;
        LogicListShowHint("");
        break;

    /* === 传感器快捷按钮: 条件区5位编码后确认子级别条件 === */
    case LOGIC_BTN_TEMP:    /* 温度 */
        HandleSensorButton(SENSOR_TEMPERATURE);
        break;

    case LOGIC_BTN_SMOKE:   /* 烟雾 */
        HandleSensorButton(SENSOR_SMOKE);
        break;

    case LOGIC_BTN_CO:      /* 一氧化碳 */
        HandleSensorButton(SENSOR_CARBON);
        break;

    case LOGIC_BTN_H2:      /* 氢气 */
        HandleSensorButton(SENSOR_HYDROGEN);
        break;

    case LOGIC_BTN_CH4:     /* 甲烷 */
        HandleSensorButton(SENSOR_METHANE);
        break;

    case LOGIC_BTN_VOC:     /* VOC */
        HandleSensorButton(SENSOR_VOC);
        break;

    case LOGIC_BTN_PRESS:   /* 压力 */
        HandleSensorButton(SENSOR_PRESSURE);
        break;

    default:
        break;
    }

    /* 每次按键后立即刷新预览窗口(73控件) */
    if (screen_id == LOGIC_SCREEN_EDIT)
    {
        BuildExprDisplay();
        DebugPrintf("[LOGIC-DISP] '%s'\r\n", s_disp_buf);
        SetTextValue(screen_id, LOGIC_TXT_PREVIEW, (uint8_t *)s_disp_buf);
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicScreen_OnText
 * 功能说明：处理来自大彩屏的文本输入事件。
 *           当前版本不支持文本输入（所有输入通过按钮完成）
 *--------------------------------------------------------------*/
void LogicScreen_OnText(uint16_t screen_id, uint16_t control_id, uint8_t *str)
{
    if (screen_id != LOGIC_SCREEN_EDIT)
    {
        return;
    }

    /* 兜底: 控件17/30 若被做成文本控件, 输入任意内容都按区间分隔符处理,
     * 并立即清掉屏幕文本, 回到按钮式输入流程 */
    if ((control_id == LOGIC_BTN_RANGE) || (control_id == 30U))
    {
        clearTextValue(screen_id, control_id);
        HandleRangeKey();
        return;
    }

    (void)str;
}

/*--------------------------------------------------------------
 * 第五部分：API实现 - 屏幕UI刷新
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicScreen_UpdateUI
 * 功能说明：在屏幕刷新循环中调用，根据当前页面刷新显示：
 *           - 编辑页(43): 显示当前编辑的表达式预览
 *           - 列表页(45): 显示所有已设置的规则
 *--------------------------------------------------------------*/
void LogicScreen_UpdateUI(uint16_t screen_id)
{
    static uint16_t s_last_page = 0xFFFF;  /* edge detect: page enter trace */
    uint8_t i;
    uint8_t count;
    uint8_t shown;
    LogicRule_t rule;
    char    disp[256];
    uint16_t pre_len;

    /* Test trace: dump rules once when entering list page */
    if (screen_id != s_last_page)
    {
        s_last_page = screen_id;
        if (screen_id == LOGIC_SCREEN_LIST)
        {
            /* 进入列表页: 重置到第0页, 并强制下次全量刷新(差分门控失效) */
            s_list_page = 0;
            s_last_list_count = 0xFF;
            s_last_list_page = 0xFF;
            s_list_selected = 0xFFU;   /* 进入列表页清选中 */
            s_list_armed = 0U;
            LogicListShowHint("");     /* 控件35 置空白 */
            DebugPrintf("[LOGIC-UI] enter list page, rules:\r\n");
            for (i = 0; i < LOGIC_RULE_MAX; i++)
            {
                if (LogicExpr_GetTable()[i].rule_id != 0)
                {
                    LogicRule_Get(LogicExpr_GetTable()[i].rule_id, &rule);
                    BuildRuleDisplay(&rule, disp, sizeof(disp));
                    DebugPrintf("  R%d: %s\r\n", rule.rule_id, disp);
                }
            }
        }
        else if (screen_id == LOGIC_SCREEN_EDIT)
        {
            DebugPrintf("[LOGIC-UI] enter edit page\r\n");
            /* 空会话进入编辑页: 强制回到条件区(phase=EDIT_COND)。
             * 否则上次遗留的动作区状态会让数字键变成输入7位动作编码,
             * 表现为"按数字只能输7位、按'-'被判定不在条件区而忽略"。 */
            if ((s_edit.step == EDIT_STEP_NONE) && (s_edit.digit_count == 0U) &&
                (s_edit.rule.token_count == 0U))
            {
                s_edit.phase = EDIT_COND;
                s_edit.active = 1;
                DebugPrintf("[LOGIC-UI] empty session -> cond phase\r\n");
            }
        }
    }

    switch (screen_id)
    {
    /* === 编辑页: 刷新预览窗口 === */
    case LOGIC_SCREEN_EDIT:
        /* 保存结果提示停留期间不覆盖预览(实时预览每帧刷新会瞬间冲掉提示) */
        if (HAL_GetTick() < s_save_msg_until)
        {
            break;
        }
        BuildExprDisplay();
        SetTextValue(screen_id, LOGIC_TXT_PREVIEW, (uint8_t *)s_disp_buf);
        break;

    /* === 列表页: 按ID降序分页显示已设置的规则(最新规则在最上面) === */
    case LOGIC_SCREEN_LIST:
    {
        uint8_t ids[LOGIC_RULE_MAX];  /* 有效规则ID集合(升序) */
        uint8_t n;                    /* 有效规则数 */
        uint8_t start;
        uint8_t total_pages;

        /* 二次确认超时自动取消(必须在差分门控之前, 否则不刷新时永不生效) */
        if ((s_list_armed != 0U) &&
            ((uint32_t)(HAL_GetTick() - s_list_arm_tick) >= LOGIC_LIST_CONFIRM_MS))
        {
            s_list_armed = 0U;
            LogicListShowHint("已取消删除");
        }

        /* 收集并升序排序(1号在前), 与删除定位共用同一函数 */
        n = LogicListCollectIds(ids);

        /* 差分门控: 规则数和页码都没变 → 跳过全量重发, 避免画面45持续闪烁 */
        if (n == s_last_list_count && s_list_page == s_last_list_page)
        {
            break;  /* 内容未变, 不刷新 */
        }
        s_last_list_count = n;
        s_last_list_page = s_list_page;


        /* 页码越界保护: 规则被删除后页码可能越界, 回到最后一页 */
        total_pages = (uint8_t)((n + LOGIC_LIST_PAGE_SIZE - 1) / LOGIC_LIST_PAGE_SIZE);
        if (total_pages == 0) total_pages = 1;
        if (s_list_page >= total_pages)
        {
            s_list_page = (uint8_t)(total_pages - 1);
        }

        /* 按当前页码偏移显示(最新规则在第0页最上方) */
        count = n;
        start = (uint8_t)(s_list_page * LOGIC_LIST_PAGE_SIZE);
        shown = 0;
        for (i = start; i < n && shown < LOGIC_LIST_PAGE_SIZE; i++)
        {
            LogicRule_Get(ids[i], &rule);

            sprintf(disp, "#%d: ", rule.rule_id);
            pre_len = (uint16_t)strlen(disp);
            BuildRuleDisplay(&rule, disp + pre_len,
                             (uint8_t)(sizeof(disp) - pre_len));

            SetTextValue(screen_id, (uint16_t)(LOGIC_TXT_LIST_BASE + shown), (uint8_t *)disp);
            shown++;
        }

        /* 清空未使用的行 */
        for (i = shown; i < LOGIC_LIST_PAGE_SIZE; i++)
        {
            clearTextValue(screen_id, (uint16_t)(LOGIC_TXT_LIST_BASE + i));
        }
    }
    break;

    default:
        break;
    }
}

/*--------------------------------------------------------------
 * 第六部分：API实现 - 初始化
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicScreen_Init
 * 功能说明：初始化屏幕处理模块内部状态。
 *--------------------------------------------------------------*/
void LogicScreen_Init(void)
{
    memset(&s_edit, 0, sizeof(EditState));
    s_edit.zone = 0;    /* 未选分区: 直接进43保存会被拒绝并提示 */
    s_disp_buf[0] = '\0';
}
