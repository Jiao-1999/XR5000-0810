/*==============================================================
 * 文件名称   : bsp_logic_expr.h
 * 模块功能   : 火警联动逻辑表达式与规则管理（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 定义逻辑规则的数据结构与操作接口。
 *              支持 (A|B)&(C|D)|E 形式的复合表达式求值。
 * 设计要点   :
 *   - 零依赖 MODEL 层，只依赖标准类型
 *   - 数据结构（Cond_t, Token_t, Rule）
 *   - CRUD + Flash持久化
 *   - 表达式求值使用递归下降解析器
 *   本模块完全自包含，不直接调用任何外部函数，所有
 *   外部交互通过回调函数完成。
 *==============================================================*/

#ifndef __BSP_LOGIC_EXPR_H
#define __BSP_LOGIC_EXPR_H

#include "main.h"    /* 引入STM32基本类型: uint8_t, uint16_t等 */

/*--------------------------------------------------------------
 * 1. 常量定义区
 *--------------------------------------------------------------*/

#define LOGIC_RULE_MAX           64      /* RAM/Flash中可存储的最大规则数 */
#define LOGIC_COND_MAX           12      /* 每条规则最多条件数（即探测器数） */
#define LOGIC_TOKEN_MAX          31      /* 每条规则最多Token数（12个条件 + 19个操作符/括号） */
#define LOGIC_ACTION_MAX         4       /* 每条规则最多执行动作数 */
#define LOGIC_DELAY_MAX_S        600     /* 动作延时时长上限（秒，10分钟） */
#define LOGIC_WILDCARD           0xFF    /* 通配符：匹配任意探测器编号 */

/* Flash 存储区 W25Q256 SPI Flash 芯片 0x080000 */
#define LOGIC_FLASH_RULES_ADDR   0x080000U  /* 规则存储区起始地址(连续3个4KB扇区) */
#define LOGIC_FLASH_META_ADDR   0x088000U  /* 元数据存储区地址 */
#define LOGIC_MAGIC              0x4C4F4731U /* Flash存储魔数 LOG1 */

/*--------------------------------------------------------------
 * 2. 枚举类型定义区
 *--------------------------------------------------------------*/

/* 报警等级枚举 - 用于条件匹配时区分报警类型 */
typedef enum
{
    LV_ANY     = 0x00,  /* 任意等级（匹配任意报警状态） */
    LV_HIGH    = 0x01,  /* 高报警 / 火警级别 */
    LV_LOW     = 0x02,  /* 低报警 / 预警级别 */
    LV_FAULT   = 0x08,  /* 故障状态 */
    LV_OFFLINE = 0x09,  /* 离线状态 */
} AlarmLevel;

/* 传感器类型枚举 - 对应 bsp_logic_set.h 中的定义 */
typedef enum
{
    SENSOR_ANY         = 0x00,  /* 任意类型（不区分传感器类型，任何探测器报警均触发） */
    SENSOR_HAND_REPORT = 0x10,  /* 手动报警按钮 */
    SENSOR_POWER       = 0x11,  /* 电源故障/掉电报警 */
    SENSOR_PRESSURE    = 0xF9,  /* 压力传感器(仅回路3) */
    SENSOR_VOC         = 0xFA,  /* VOC挥发性有机物(仅回路3) */
    SENSOR_METHANE     = 0xFB,  /* 甲烷CH4传感器(仅回路3) */
    SENSOR_HYDROGEN    = 0xFC,  /* 氢气传感器H2（回路3，可燃气体） */
    SENSOR_CARBON      = 0xFD,  /* 一氧化碳传感器CO（回路3，有毒气体） */
    SENSOR_SMOKE       = 0xFE,  /* 烟雾传感器（回路1+3） */
    SENSOR_TEMPERATURE = 0xFF,  /* 温度传感器（回路1+3） */
} SensorType;

/* 执行模式枚举 - 控制动作启动/停止行为 */
typedef enum
{
    EXEC_START_ALL  = 0,  /* 全部启动：启动所有配置的动作（延时参数生效） */
    EXEC_START_PART = 1,  /* 部分启动：仅启动指定编号的动作 */
    EXEC_STOP_ALL   = 2,  /* 全部停止：停止所有正在执行的动作 */
    EXEC_STOP_PART  = 3,  /* 部分停止：仅停止指定编号的动作 */
} ExecMode;

/* Token类型枚举 - 表达式中的元素类型 */
typedef enum
{
    TOK_COND   = 0,  /* 条件Token，引用 conditions[idx] */
    TOK_AND    = 1,  /* 逻辑与操作符 (&) */
    TOK_OR     = 2,  /* 逻辑或操作符 (|) */
    TOK_NOT    = 3,  /* 逻辑非操作符 (!) */
    TOK_LPAREN = 4,  /* 左括号 ( */
    TOK_RPAREN = 5,  /* 右括号 ) */
} TokType;

/* 规则运行时状态枚举 */
typedef enum
{
    RT_IDLE     = 0,  /* 空闲中：规则未激活或条件未满足 */
    RT_ARMED    = 1,  /* 已布防：规则激活，正在监控条件 */
    RT_DELAYING = 2,  /* 延时中：条件满足，正在延时等待执行 */
    RT_EXECUTED = 3,  /* 已执行：所有延时动作已全部执行完毕 */
    RT_DONE     = 4,  /* 已完成：动作执行完毕，等待复位清除 */
} RtState;

/*--------------------------------------------------------------
 * 3. 数据结构定义区（packed紧凑结构，用于Flash存储对齐）
 *--------------------------------------------------------------*/

/* 条件结构体 - 每个条件占用4字节
 * 5位编码: 前2位回路号 + 后3位设备号, 如 01002 = 回路01的002号设备
 * 示例：{loop_no=1, dev_no=2, sensor_type=0xFF, alarm_level=0x01}
 *       含义："01探测回路002号设备温度高报警时触发" */
typedef struct
{
    uint8_t loop_no;     /* 探测回路号（1-N），0xFF=任意回路 */
    uint16_t dev_no;     /* 设备号（3位编码000-999），0xFF=任意设备 */
    uint8_t sensor_type; /* 传感器类型，SensorType枚举值，SENSOR_ANY=任意 */
    uint8_t alarm_level; /* 报警等级，AlarmLevel枚举值，LV_ANY=任意 */
} __attribute__((packed)) Cond_t;

/* Token结构体 - 表达式中每个元素占2字节
 * 当type==TOK_COND时，cond_idx指向conditions[]数组索引
 * 其他类型Token不使用cond_idx字段，未初始化时为0 */
typedef struct
{
    uint8_t type;      /* Token类型，TokType枚举值 */
    uint8_t cond_idx;  /* 条件数组索引（仅当type==TOK_COND时有效） */
} __attribute__((packed)) Token_t;

/* 动作结构体 - 每个动作占5字节
 * 5位编码: 前2位回路号 + 后3位设备号, 如 01002 = 回路01的002号设备
 * 示例：{loop_no=1, dev_no=2, action=1, delay_s=30}
 *       含义："30秒后启动01控制回路002号设备" */
typedef struct
{
    uint8_t  loop_no;   /* 控制回路号（1-N） */
    uint16_t dev_no;    /* 设备号（3位编码000-999） */
    uint8_t  action;    /* 0=停止, 1=启动 */
    uint16_t delay_s;   /* 延时秒数，0=无延时立即执行，上限=600 */
} __attribute__((packed)) Action_t;

/* 规则结构体 - 每条规则占138字节（packed紧凑）
 * 结构 = 元信息 + 条件 + 表达式 + 动作
 * 表达式由TOK_COND Token引用条件数组，不重复存储条件数据 */
typedef struct
{
    uint8_t  enable;                         /* 1=启用, 0=禁用 */
    uint8_t  rule_id;                        /* 规则ID（1-255），0表示空槽位 */

    uint8_t  cond_count;                     /* 条件数量（1..12） */
    Cond_t   conditions[LOGIC_COND_MAX];     /* 条件数组 */

    uint8_t  token_count;                    /* 表达式Token数量 */
    Token_t  tokens[LOGIC_TOKEN_MAX];        /* 表达式Token数组 */

    uint8_t  exec_mode;                      /* 执行模式，ExecMode枚举值 */
    uint8_t  action_count;                   /* 动作数量（1..4） */
    Action_t actions[LOGIC_ACTION_MAX];      /* 动作数组 */

    uint16_t crc;                            /* CRC16校验和，用于Flash加载时校验完整性 */
} __attribute__((packed)) LogicRule_t;

/* 运行时状态结构体 - 仅存在于RAM中，不会写入Flash */
typedef struct
{
    RtState  state;                  /* 当前运行时状态 */
    uint32_t arm_timestamp;          /* 布防时间戳（用于复位判定） */
    uint32_t delay_expire[LOGIC_ACTION_MAX]; /* 各动作延时到期时间戳 */
    uint8_t  action_done[LOGIC_ACTION_MAX];  /* 1=动作已执行, 0=未执行 */
} RtRuntime;

/*--------------------------------------------------------------
 * 4. 回调函数类型定义 - 查询条件函数
 *    由外部模块实现并注册，本模块通过它查询实时探测器状态。
 *    查询函数实现在 bsp_logic_dev.c 中，见 LogicDev_Init。
 *--------------------------------------------------------------*/

typedef uint8_t (*QueryCondFunc_t)(const Cond_t *cond);

/*--------------------------------------------------------------
 * 5. API声明 - 规则CRUD操作
 *    返回值约定：0=成功, 1=未找到, 2=参数错误, 3=规则表已满
 *--------------------------------------------------------------*/

uint8_t LogicRule_Create(uint8_t id, const LogicRule_t *rule);  /* 创建新规则 */
uint8_t LogicRule_Modify(uint8_t id, const LogicRule_t *rule);  /* 修改已有规则 */
uint8_t LogicRule_Delete(uint8_t id);                            /* 删除指定ID规则 */
uint8_t LogicRule_Get(uint8_t id, LogicRule_t *rule);            /* 查询指定ID规则 */
uint8_t LogicRule_GetCount(void);                                /* 获取有效规则数量 */
uint8_t LogicRule_SetEnable(uint8_t id, uint8_t enable);         /* 启用/禁用规则 */
void    LogicRule_ClearAll(void);                                /* 清空所有规则 */

/*--------------------------------------------------------------
 * 6. API声明 - Flash持久化操作
 *    返回值约定：0=成功, 1=读/写错误
 *--------------------------------------------------------------*/

uint8_t LogicRule_SaveAll(void);   /* 将所有规则保存到Flash */
uint8_t LogicRule_LoadAll(void);   /* 从Flash加载所有规则 */

/*--------------------------------------------------------------
 * 7. API声明 - 表达式求值
 *    对规则表达式进行递归下降求值，支持括号优先级。
 *    求值时通过回调函数查询每个条件的实时状态。
 *    返回值约定：满足条件返回1，不满足返回0。
 *--------------------------------------------------------------*/

uint8_t LogicExpr_Evaluate(const LogicRule_t *rule);

/*--------------------------------------------------------------
 * 8. API声明 - 初始化与查询函数注册
 *--------------------------------------------------------------*/

void LogicExpr_SetQueryFunc(QueryCondFunc_t func);  /* 注册查询回调函数 */
void LogicExpr_Init(void);                           /* 模块初始化，从Flash加载规则表 */

/*--------------------------------------------------------------
 * 9. API声明 - 内部状态访问（供调试与诊断使用）
 *--------------------------------------------------------------*/

LogicRule_t * LogicExpr_GetTable(void);    /* 获取规则表首地址（64个槽位） */
RtRuntime   * LogicExpr_GetRuntime(void);  /* 获取运行时状态数组首地址 */
uint8_t       LogicExpr_IsInitialized(void); /* 查询模块是否已初始化 */
uint8_t       LogicExpr_GetCrcFailCount(void); /* 获取LoadAll时CRC校验失败次数（诊断用） */

#endif /* __BSP_LOGIC_EXPR_H */
