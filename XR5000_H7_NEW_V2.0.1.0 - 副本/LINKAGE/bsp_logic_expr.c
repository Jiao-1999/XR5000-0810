/*==============================================================
 * 文件名称   : bsp_logic_expr.c
 * 模块功能   : 火警联动逻辑表达式与规则管理（实现文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 提供逻辑规则的增删改查与表达式求值功能。
 *              - 规则CRUD操作（创建/修改/删除/查询/枚举）
 *              - Flash持久化（基于W25Qxx SPI Flash）
 *              - 递归下降表达式求值（支持括号优先级）
 *              - 默认规则加载
 *
 * 架构定位   : MODEL层，Flash持久化与表达式求值核心
 *              设备无关设计，仅通过回调查询条件
 *
 * 依赖模块   :
 *   - w25qxx.h        : Flash读/写/擦除
 *   - system.h        : CalcCrc16()
 *   - cmsis_os.h      : osDelay()用于RTOS延时等待
 *   - stm32h7xx_hal.h : HAL_GetTick()
 *==============================================================*/

#include "bsp_logic_expr.h"     /* 逻辑规则数据结构与API定义 */
#include "w25qxx.h"             /* W25QXX_Read/Write/Erase_Sector */
#include "system.h"             /* CalcCrc16() */
#include "cmsis_os.h"           /* osDelay()用于RTOS延时等待 */
#include "stm32h7xx_hal.h"      /* HAL_GetTick() */
#include "bsp_debug.h"          /* DebugPrintf: test trace on UART4 */
#include <string.h>             /* memset/memcpy */

/*--------------------------------------------------------------
 * 全局变量定义
 *--------------------------------------------------------------*/

/* 规则表，64个槽位，存储于RAM中，可保存到Flash */
static LogicRule_t s_rule_table[LOGIC_RULE_MAX];

/* 运行时状态表，仅存在于RAM中，不会写入Flash表 */
static RtRuntime   s_runtime[LOGIC_RULE_MAX];

/* 模块初始化标志：0=未初始化, 1=已初始化 */
static uint8_t     s_initialized = 0;

/* 设备状态查询回调函数，由bsp_logic_dev.c注册
 * 若为NULL则表达式求值时所有条件均返回0 */
static QueryCondFunc_t s_query_func = NULL;

/* CRC校验失败计数器，LoadAll时统计CRC失败条数供诊断查询*/
static uint8_t s_crc_fail_count = 0;

/* Flash元数据结构，5字节紧凑packed布局
 * 存储魔数与规则数量，位于LOGIC_FLASH_META_ADDR处 */
typedef struct
{
    uint32_t magic;       /* 魔数 "LOG1" = 0x4C4F4731 */
    uint8_t  rule_count;  /* 写入Flash的有效规则数量 */
} __attribute__((packed)) LogicMeta_t;

/*--------------------------------------------------------------
 * 内部函数声明：仅static函数在前向声明中列出
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FindSlotById
 * 功能描述：按指定ID在规则表中查找对应槽位。
 * 输入参数：id - 要查找的规则ID（1-255）
 * 返回值   ：找到的槽位索引（0-63），未找到返回0xFF
 *--------------------------------------------------------------*/
static uint8_t FindSlotById(uint8_t id)
{
    uint8_t i;  /* 循环计数器 */

    /* 校验ID：0为保留值表示空槽位，无效 */
    if (id == 0)
    {
        return 0xFF;  /* 无效ID直接返回未找到 */
    }

    /* 遍历规则表 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* 已使用槽位且ID匹配 */
        if (s_rule_table[i].rule_id == id)
        {
            return i;  /* 返回匹配的槽位索引 */
        }
    }

    return 0xFF;  /* 遍历所有槽位均未找到 */
}

/*--------------------------------------------------------------
 * 函数名称：FindEmptySlot
 * 功能描述：在规则表中查找首个空闲槽位
 * 返回值   ：空闲槽位索引（0-63），表已满返回0xFF
 *--------------------------------------------------------------*/
static uint8_t FindEmptySlot(void)
{
    uint8_t i;  /* 循环计数器 */

    /* 遍历所有槽位 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* rule_id == 0表示空闲槽位 */
        if (s_rule_table[i].rule_id == 0)
        {
            return i;  /* 返回空闲槽位 */
        }
    }

    return 0xFF;  /* 规则表已满，无空闲槽位 */
}

/*--------------------------------------------------------------
 * 函数名称：IsRuleValid
 * 功能描述：校验规则字段是否全部合法
 * 检查项：ID有效、各计数值在合法范围内、数组
 *         Token中的条件索引在有效范围内、
 * 输入参数：rule - 待校验的规则结构体
 * 返回值   ：1=有效, 0=无效
 *
 * 设备号范围说明（分回路差异化校验）：
 *   回路2(MBus控制回路)设备地址为1-63，允许60-62等扩展地址
 *   其他回路设备号仅允许0-29
 *--------------------------------------------------------------*/
static uint8_t IsRuleValid(const LogicRule_t *rule)
{
    uint8_t i;    /* 循环计数器 */
    uint8_t lno;  /* 条件回路号临时变量 */
    uint16_t dno;  /* 条件设备号临时变量(3位编码000-999,防uint8截断误判) */

    /* 空指针检查 */
    if (rule == NULL)
    {
        return 0;  /* 空指针无效 */
    }

    /* 规则ID不能为0（0表示空槽位） */
    if (rule->rule_id == 0)
    {
        return 0;  /* 无效ID */
    }

    /* 条件数量必须在1..LOGIC_COND_MAX之间 */
    if (rule->cond_count == 0 || rule->cond_count > LOGIC_COND_MAX)
    {
        return 0;  /* 条件数量超出有效范围 */
    }

    /* Token数量必须在1..LOGIC_TOKEN_MAX之间 */
    if (rule->token_count == 0 || rule->token_count > LOGIC_TOKEN_MAX)
    {
        return 0;  /* Token数量超出有效范围 */
    }

    /* 动作数量必须在1..LOGIC_ACTION_MAX之间 */
    if (rule->action_count == 0 || rule->action_count > LOGIC_ACTION_MAX)
    {
        return 0;  /* 动作数量超出有效范围 */
    }

    /* 执行模式必须在0..3之间（ExecMode枚举范围） */
    if (rule->exec_mode > 3)
    {
        return 0;  /* 无效执行模式 */
    }

    /* 检查每个动作的延时是否在有效范围内 */
    for (i = 0; i < rule->action_count; i++)
    {
        /* 动作延时时长不能超过600秒 */
        if (rule->actions[i].delay_s > LOGIC_DELAY_MAX_S)
        {
            return 0;  /* 延时超出上限 */
        }
    }

    /* 检查条件中的回路号和设备号范围（分回路差异化校验）
     * loop_no 只能为 1-4 或 0xFF（通配符）
     * dev_no: 回路2允许0-63（支持地址60-62扩展设备）
     *         其他回路仅允许0-29或0xFF */
    for (i = 0; i < rule->cond_count; i++)
    {
        lno = rule->conditions[i].loop_no;
        dno = rule->conditions[i].dev_no;
        if (lno != LOGIC_WILDCARD && (lno == 0 || lno > 4))
        {
            return 0;  /* 回路号非法：仅允许 1-4 或 0xFF */
        }
        if (dno != LOGIC_WILDCARD)
        {
            if (lno == 2)
            {
                /* 回路2设备地址范围1-63，允许60-62扩展地址 */
                if (dno > 63)
                {
                    return 0;  /* 回路2设备号非法 */
                }
            }
            else
            {
                /* 其他回路设备号仅允许0-29 */
                if (dno >= 30)
                {
                    return 0;  /* 设备号非法 */
                }
            }
        }
    }

    /* 校验Token中的括号是否配对（左右括号必须完整匹配）*/
    {
        int8_t paren_depth = 0;  /* 括号深度计数器：>0=未匹配, <0=多余 */
        for (i = 0; i < rule->token_count; i++)
        {
            if (rule->tokens[i].type == TOK_COND)
            {
                /* 条件Token的条件索引在有效范围内 */
                if (rule->tokens[i].cond_idx >= rule->cond_count)
                {
                    return 0;  /* 条件索引越界 */
                }
            }
            else if (rule->tokens[i].type == TOK_LPAREN)
            {
                paren_depth++;  /* 左括号入栈，深度加一 */
            }
            else if (rule->tokens[i].type == TOK_RPAREN)
            {
                paren_depth--;  /* 右括号出栈，深度减一 */
                if (paren_depth < 0)
                {
                    return 0;  /* 右括号多于左括号，配对失败 */
                }
            }
        }
        /* 遍历结束后深度必须为0，否则左右括号不匹配 */
        if (paren_depth != 0)
        {
            return 0;  /* 括号未完全配对，规则无效 */
        }
    }

    return 1;  /* 所有检查均通过，规则有效 */
}

/*--------------------------------------------------------------
 * 函数名称：ResetRuntime
 * 功能描述：重置指定槽位的运行时状态为IDLE态
 * 在Create/Modify/Delete/SetEnable时均需调用
 * 以清除历史执行痕迹重新开始监控
 * 输入参数：slot - 槽位索引（0-63）
 *--------------------------------------------------------------*/
static void ResetRuntime(uint8_t slot)
{
    /* 清零运行时状态结构体 */
    memset(&s_runtime[slot], 0, sizeof(RtRuntime));

    /* 设置运行时状态为IDLE，所有动作完成标志清零，准备重新监控 */
    s_runtime[slot].state = RT_IDLE;
}

/*--------------------------------------------------------------
 * 函数名称：CalcRuleCRC
 * 功能描述：计算规则结构体的CRC16校验和
 * CRC计算时排除最后2字节（crc字段自身），避免自引用
 * 输入参数：rule - 待计算的规则
 * 返回值   ：CRC16值
 *--------------------------------------------------------------*/
static uint16_t CalcRuleCRC(const LogicRule_t *rule)
{
    /* 计算规则结构体CRC时排除crc字段自身，
     * sizeof(LogicRule_t) = 138字节，crc字段占最后2字节
     * 实际计算136字节得到CRC */
    return CalcCrc16((uint8_t *)rule, sizeof(LogicRule_t) - 2);
}

/*--------------------------------------------------------------
 * 规则CRUD操作（创建/修改/删除/查询）
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Create
 * 功能描述：创建新规则并写入规则表
 * 输入参数：id - 规则ID（1-255），不能与已有规则重复
 *           rule - 待创建的规则结构体
 * 返回值   ：0=成功, 1=ID已存在, 2=规则表已满, 3=参数无效
 *--------------------------------------------------------------*/
uint8_t LogicRule_Create(uint8_t id, const LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位索引 */

    /* 参数校验 */
    if (rule == NULL || id == 0)
    {
        return 3;  /* 参数无效 */
    }

    /* 校验规则合法性 */
    if (!IsRuleValid(rule))
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: invalid structure\r\n", id);
        return 3;  /* 规则结构体无效 */
    }

    /* 检查ID是否已存在（避免重复创建） */
    if (FindSlotById(id) != 0xFF)
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: id exists\r\n", id);
        return 1;  /* ID已存在 */
    }

    /* 在表中查找空闲槽位 */
    slot = FindEmptySlot();
    if (slot == 0xFF)
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: table full\r\n", id);
        return 2;  /* 规则表已满 */
    }

    /* 将规则写入空闲槽位 */
    s_rule_table[slot] = *rule;

    /* 强制rule_id为外部传入的id */
    s_rule_table[slot].rule_id = id;

    /* 计算并存储CRC，用于Flash加载时校验 */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置该槽位运行时状态 */
    ResetRuntime(slot);

    DebugPrintf("[LOGIC] Rule %d created: conds=%d tokens=%d actions=%d\r\n",
                id, rule->cond_count, rule->token_count, rule->action_count);
    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Modify
 * 功能描述：修改规则表中已有ID的规则
 * 输入参数：id - 待修改的规则ID，不能改变ID
 *           rule - 待更新的规则结构体
 * 返回值   ：0=成功, 1=未找到, 3=参数无效
 *--------------------------------------------------------------*/
uint8_t LogicRule_Modify(uint8_t id, const LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位索引 */

    /* 参数校验 */
    if (rule == NULL || id == 0)
    {
        return 3;  /* 参数无效 */
    }

    /* 校验规则合法性 */
    if (!IsRuleValid(rule))
    {
        return 3;  /* 规则结构体无效 */
    }

    /* 查找规则所在槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 覆盖旧规则内容 */
    s_rule_table[slot] = *rule;

    /* 保持原有规则ID */
    s_rule_table[slot].rule_id = id;

    /* 重新计算CRC */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置运行时状态以清除历史执行痕迹 */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Delete
 * 功能描述：按ID删除规则，清空对应槽位并重置运行时状态
 * 输入参数：id - 待删除的规则ID
 * 返回值   ：0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_Delete(uint8_t id)
{
    uint8_t slot;  /* 目标槽位索引 */

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 清空槽位内容（全零填充，rule_id置0表示空闲） */
    memset(&s_rule_table[slot], 0, sizeof(LogicRule_t));

    /* 重置运行时状态 */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Get
 * 功能描述：按ID读取规则的完整内容
 * 输入参数：id - 待查询的规则ID
 *           rule - 输出参数，用于接收规则内容（调用方分配）
 * 返回值   ：0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_Get(uint8_t id, LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位索引 */

    /* 检查输出指针 */
    if (rule == NULL)
    {
        return 1;  /* 输出参数无效 */
    }

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 拷贝规则到输出参数 */
    *rule = s_rule_table[slot];

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_GetCount
 * 功能描述：统计规则表中有效规则的数量
 * 返回值   ：有效规则数量（0..64）
 *--------------------------------------------------------------*/
uint8_t LogicRule_GetCount(void)
{
    uint8_t i;      /* 循环计数器 */
    uint8_t count = 0;  /* 有效规则计数器 */

    /* 遍历所有槽位 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* rule_id != 0表示该槽位为有效规则 */
        if (s_rule_table[i].rule_id != 0)
        {
            count++;  /* 累加有效规则计数 */
        }
    }

    return count;  /* 返回有效规则总数 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_SetEnable
 * 功能描述：启用或禁用指定规则，同时重置运行时状态
 * 输入参数：id - 规则ID
 *           enable - 1=启用, 0=禁用
 * 返回值   ：0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_SetEnable(uint8_t id, uint8_t enable)
{
    uint8_t slot;  /* 目标槽位索引 */

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 更新启用状态 */
    s_rule_table[slot].enable = enable;

    /* 重新计算CRC（enable字段参与CRC计算） */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置运行时状态以应用启用/禁用状态变化 */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_ClearAll
 * 功能描述：清空规则表，将所有槽位重置为空闲状态。
 *--------------------------------------------------------------*/
void LogicRule_ClearAll(void)
{
    uint8_t i;  /* 循环计数器 */

    /* 清零整个规则表 */
    memset(s_rule_table, 0, sizeof(s_rule_table));

    /* 逐个重置运行时状态 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        ResetRuntime(i);  /* 重置每个槽位运行时状态为IDLE */
    }
}

/*--------------------------------------------------------------
 * Flash持久化操作
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicRule_SaveAll
 * 功能描述：将规则表中所有有效规则保存到Flash（W25Qxx芯片）
 * 保存策略：先擦后写，按写入顺序连续存储
 * 返回值   ：0=成功, 1=失败
 *--------------------------------------------------------------*/
uint8_t LogicRule_SaveAll(void)
{
    uint8_t  i;              /* 循环计数器 */
    uint16_t write_offset;   /* Flash写入偏移量（字节单位） */
    LogicMeta_t meta;        /* 待写入的元数据 */

    /* 写入前先擦除Flash，因为Flash物理特性决定必须先擦后写 */
    /* 规则区占3个扇区：138字节*64=8832 > 8192(2扇区)，需3个4KB扇区 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR);          /* 擦除规则存储扇区1 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR + 0x1000); /* 擦除规则存储扇区2 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR + 0x2000); /* 擦除规则存储扇区3 */
    W25QXX_Erase_Sector(LOGIC_FLASH_META_ADDR);           /* 擦除元数据存储扇区 */

    /* 写入所有有效规则到Flash */
    write_offset = 0;  /* 写入偏移从0开始 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* 只写入具有有效ID的规则 */
        if (s_rule_table[i].rule_id != 0)
        {
            /* 将规则结构体写入Flash对应位置 */
            W25QXX_Write(
                (uint8_t *)&s_rule_table[i],           /* 源：RAM中的规则结构体 */
                LOGIC_FLASH_RULES_ADDR + write_offset, /* 目标：Flash存储地址 */
                sizeof(LogicRule_t)                     /* 大小：138字节 */
            );
            /* 累加写入偏移量 */
            write_offset += sizeof(LogicRule_t);
        }
    }

    /* 写入元数据：魔数 + 规则数量 */
    meta.magic = LOGIC_MAGIC;                /* "LOG1"魔数用于校验有效性 */
    meta.rule_count = LogicRule_GetCount();  /* 写入有效规则总数 */

    W25QXX_Write(
        (uint8_t *)&meta,               /* 元数据结构体地址 */
        LOGIC_FLASH_META_ADDR,          /* 元数据存储地址 */
        sizeof(LogicMeta_t)             /* 大小：5字节 */
    );

    DebugPrintf("[LOGIC] SaveAll: %d rules saved to Flash 0x080000\r\n", meta.rule_count);
    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_LoadAll
 * 功能描述：从Flash加载所有规则到RAM中
 * 加载时对每条规则进行CRC校验
 * 返回值   ：0=成功, 1=Flash无有效数据
 *--------------------------------------------------------------*/
uint8_t LogicRule_LoadAll(void)
{
    uint8_t  i;              /* 循环计数器 */
    uint16_t read_offset;    /* Flash读取偏移量（字节单位） */
    LogicMeta_t meta;        /* 从Flash读取的元数据 */
    LogicRule_t temp_rule;   /* 临时存放从Flash读取的规则 */

    s_crc_fail_count = 0;    /* 复位CRC失败计数器 */

    /* 先从Flash读取元数据 */
    W25QXX_Read(
        (uint8_t *)&meta,           /* 元数据结构体地址 */
        LOGIC_FLASH_META_ADDR,      /* 元数据在Flash中的地址 */
        sizeof(LogicMeta_t)         /* 大小：5字节 */
    );

    /* 校验魔数是否匹配，Flash未写入时为0xFFFFFFFF */
    if (meta.magic != LOGIC_MAGIC)
    {
        DebugPrintf("[LOGIC] LoadAll: Flash empty (bad magic), no rules\r\n");
        return 1;  /* Flash未初始化或数据被擦除，返回失败 */
    }

    /* 先清空RAM规则表 */
    LogicRule_ClearAll();

    /* 从Flash逐条读取规则 */
    read_offset = 0;  /* 读取偏移从0开始 */
    for (i = 0; i < meta.rule_count && i < LOGIC_RULE_MAX; i++)
    {
        /* 从Flash读取一条规则到临时变量 */
        W25QXX_Read(
            (uint8_t *)&temp_rule,                  /* 临时规则结构体地址 */
            LOGIC_FLASH_RULES_ADDR + read_offset,   /* 规则在Flash中的地址 */
            sizeof(LogicRule_t)                      /* 大小：138字节 */
        );

        /* 校验规则CRC是否与Flash一致 */
        if (CalcRuleCRC(&temp_rule) == temp_rule.crc)
        {
            /* CRC校验通过，规则数据有效，存入RAM表中 */
            /* 占用对应索引的槽位 */
            s_rule_table[i] = temp_rule;
            ResetRuntime(i);  /* 重置运行时状态以准备监控 */
        }
        else
        {
            /* CRC失败时跳过该规则，计数器自增用于诊断 */
            s_crc_fail_count++;
        }

        /* 累加读取偏移量 */
        read_offset += sizeof(LogicRule_t);
    }

    DebugPrintf("[LOGIC] LoadAll: %d rules loaded, crc_fail=%d\r\n",
                meta.rule_count, s_crc_fail_count);
    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 递归下降表达式求值实现
 *
 * 采用BNF文法定义：
 *   expr   -> term ('|' term)*          // OR表达式，左结合求值
 *   term   -> factor ('&' factor)*      // AND表达式，中等优先级
 *   factor -> '!' factor                // NOT一元运算符，最高优先级
 *          | '(' expr ')'              // 括号分组提升优先级
 *          | condition                 // 条件原子，查询探测器状态
 *
 * 语法错误时安全失败返回0，避免误触发联动
 * 优先级：括号 > AND > OR（与C语言一致）
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 内部函数前向声明
 *--------------------------------------------------------------*/
static uint8_t eval_expr(const LogicRule_t *rule, uint8_t *pos);
static uint8_t eval_term(const LogicRule_t *rule, uint8_t *pos);
static uint8_t eval_factor(const LogicRule_t *rule, uint8_t *pos);

/*--------------------------------------------------------------
 * 函数名称：eval_expr
 * 功能描述：解析并求值OR表达式（顶层）
 * OR运算采用短路求值：依次求值term，遇'|'继续
 * 若ANY term为真则结果为1，否则为0
 *
 * 输入参数：rule - 包含tokens与conditions的规则结构体
 *           pos - Token数组中的当前位置指针（按引用传递）
 * 返回值   ：1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_expr(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */

    /* 先求值第一个term */
    result = eval_term(rule, pos);

    /* 循环处理后续所有OR运算符 */
    while (*pos < rule->token_count && rule->tokens[*pos].type == TOK_OR)
    {
        (*pos)++;  /* 跳过OR token */
        /* 求值下一个term并按位或到结果 */
        result |= eval_term(rule, pos);
    }

    return result;  /* 返回所有term按OR运算的结果 */
}

/*--------------------------------------------------------------
 * 函数名称：eval_term
 * 功能描述：解析并求值AND表达式（中层）
 * AND运算采用短路求值：依次求值factor，遇'&'继续
 * 若ALL factor为真则结果为1，否则为0
 *
 * 输入参数：rule - 包含tokens与conditions的规则结构体
 *           pos - Token数组中的当前位置指针
 * 返回值   ：1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_term(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */

    /* 先求值第一个factor */
    result = eval_factor(rule, pos);

    /* 循环处理后续所有AND运算符 */
    while (*pos < rule->token_count && rule->tokens[*pos].type == TOK_AND)
    {
        (*pos)++;  /* 跳过AND token */
        /* 求值下一个factor并按位与到结果 */
        result &= eval_factor(rule, pos);
    }

    return result;  /* 返回所有factor按AND运算的结果 */
}

/*--------------------------------------------------------------
 * 函数名称：eval_factor
 * 功能描述：解析并求值factor（底层原子）factor有三种形式
 * factor有三种形式：
 *   - NOT factor: 对子factor取反后返回
 *   - ( expr ): 括号范围内的子表达式递归求值
 *   - condition: 查询探测器实时状态
 *
 * 输入参数：rule - 包含tokens与conditions的规则结构体
 *           pos - Token数组中的当前位置指针
 * 返回值   ：1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_factor(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */
    uint8_t idx;     /* 条件数组索引 */

    /* 越界检查：Token索引不能越界 */
    if (*pos >= rule->token_count)
    {
        return 0;  /* 耗尽所有Token，安全失败返回0 */
    }

    /* 情况1：NOT一元运算符 - 递归求值子factor */
    if (rule->tokens[*pos].type == TOK_NOT)
    {
        (*pos)++;  /* 跳过NOT token */
        /* 递归求值子factor并取反 */
        return (uint8_t)(!eval_factor(rule, pos));
    }

    /* 情况2：左括号 - 递归求值括号内表达式 */
    if (rule->tokens[*pos].type == TOK_LPAREN)
    {
        (*pos)++;  /* 跳过'(' token */
        /* 递归求值括号内的子表达式 */
        result = eval_expr(rule, pos);
        /* 期望匹配')'，若存在则跳过 */
        if (*pos < rule->token_count && rule->tokens[*pos].type == TOK_RPAREN)
        {
            (*pos)++;  /* 跳过')' token */
        }
        return result;  /* 返回括号内子表达式的结果 */
    }

    /* 情况3：条件 - 查询探测器状态 */
    if (rule->tokens[*pos].type == TOK_COND)
    {
        idx = rule->tokens[*pos].cond_idx;  /* 从Token中取出条件数组索引 */
        (*pos)++;  /* 跳过条件token */

        /* 越界检查：条件索引必须在有效范围内 */
        if (idx >= rule->cond_count)
        {
            return 0;  /* 索引越界，安全失败 */
        }

        /* 通过回调函数查询探测器实时状态 */
        if (s_query_func != NULL)
        {
            /* 调用查询函数，满足条件返回1 */
            return s_query_func(&rule->conditions[idx]);
        }

        return 0;  /* 未注册查询函数，默认不满足 */
    }

    /* 情况4：其他Token（RPAREN或END）- 语法错误安全失败 */
    (*pos)++;  /* 跳过未知Token */
    return 0;  /* 语法错误，安全返回0 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicExpr_Evaluate
 * 功能描述：对规则表达式进行递归下降求值
 * 求值时通过回调函数查询条件
 *
 * 输入参数：rule - 待求值的规则结构体
 * 返回值   ：1=条件满足（触发联动动作）, 0=假
 *--------------------------------------------------------------*/
uint8_t LogicExpr_Evaluate(const LogicRule_t *rule)
{
    uint8_t pos;    /* Token数组中的当前位置 */
    uint8_t result; /* 求值结果 */

    /* 参数校验 */
    if (rule == NULL || rule->token_count == 0)
    {
        return 0;  /* 无效规则，安全返回0 */
    }

    /* 初始化Token位置指针 */
    pos = 0;

    /* 从顶层开始求值（OR表达式为入口） */
    result = eval_expr(rule, &pos);

    return result;  /* 返回表达式最终求值结果 */
}

/*--------------------------------------------------------------
 * 默认规则部分（出厂配置）
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LoadDefault
 * 功能描述：加载5条出厂默认规则到规则表
 * 当Flash中无有效数据时调用本函数初始化典型联动场景
 *--------------------------------------------------------------*/
static void LoadDefault(void)
{
    LogicRule_t rule;  /* 临时规则结构体 */

    memset(&rule, 0, sizeof(LogicRule_t));

    /*--- 默认规则1：(回路1任意设备) & (回路1任意设备) → 控制回路1设备1(声光) ---*/
    rule.enable = 1;
    rule.rule_id = 1;
    rule.cond_count = 2;
    rule.conditions[0].loop_no = 1;
    rule.conditions[0].dev_no = LOGIC_WILDCARD;
    rule.conditions[0].sensor_type = SENSOR_ANY;
    rule.conditions[0].alarm_level = LV_ANY;
    rule.conditions[1].loop_no = 1;
    rule.conditions[1].dev_no = LOGIC_WILDCARD;
    rule.conditions[1].sensor_type = SENSOR_ANY;
    rule.conditions[1].alarm_level = LV_ANY;
    rule.token_count = 3;
    rule.tokens[0].type = TOK_COND;  rule.tokens[0].cond_idx = 0;
    rule.tokens[1].type = TOK_AND;   rule.tokens[1].cond_idx = 0;
    rule.tokens[2].type = TOK_COND;  rule.tokens[2].cond_idx = 1;
    rule.exec_mode = EXEC_START_ALL;
    rule.action_count = 1;
    rule.actions[0].loop_no = 1;
    rule.actions[0].dev_no = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[0] = rule;
    ResetRuntime(0);

    /*--- 默认规则2：(回路1任意设备) → 控制回路1设备3(排风) ---*/
    memset(&rule, 0, sizeof(LogicRule_t));
    rule.enable = 1;
    rule.rule_id = 2;
    rule.cond_count = 1;
    rule.conditions[0].loop_no = 1;
    rule.conditions[0].dev_no = LOGIC_WILDCARD;
    rule.conditions[0].sensor_type = SENSOR_ANY;
    rule.conditions[0].alarm_level = LV_ANY;
    rule.token_count = 1;
    rule.tokens[0].type = TOK_COND;  rule.tokens[0].cond_idx = 0;
    rule.exec_mode = EXEC_START_PART;
    rule.action_count = 1;
    rule.actions[0].loop_no = 1;
    rule.actions[0].dev_no = 3;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[1] = rule;
    ResetRuntime(1);

    /*--- 默认规则3：(回路1任意设备) → 控制回路1设备3(排风) ---*/
    memset(&rule, 0, sizeof(LogicRule_t));
    rule.enable = 1;
    rule.rule_id = 3;
    rule.cond_count = 1;
    rule.conditions[0].loop_no = 1;
    rule.conditions[0].dev_no = LOGIC_WILDCARD;
    rule.conditions[0].sensor_type = SENSOR_ANY;
    rule.conditions[0].alarm_level = LV_ANY;
    rule.token_count = 1;
    rule.tokens[0].type = TOK_COND;  rule.tokens[0].cond_idx = 0;
    rule.exec_mode = EXEC_START_PART;
    rule.action_count = 1;
    rule.actions[0].loop_no = 1;
    rule.actions[0].dev_no = 3;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[2] = rule;
    ResetRuntime(2);

    /*--- 默认规则4：(回路1任意设备) x3 → 控制回路1设备5(默认继电器) ---*/
    memset(&rule, 0, sizeof(LogicRule_t));
    rule.enable = 1;
    rule.rule_id = 4;
    rule.cond_count = 3;
    rule.conditions[0].loop_no = 1;
    rule.conditions[0].dev_no = LOGIC_WILDCARD;
    rule.conditions[0].sensor_type = SENSOR_ANY;
    rule.conditions[0].alarm_level = LV_ANY;
    rule.conditions[1].loop_no = 1;
    rule.conditions[1].dev_no = LOGIC_WILDCARD;
    rule.conditions[1].sensor_type = SENSOR_ANY;
    rule.conditions[1].alarm_level = LV_ANY;
    rule.conditions[2].loop_no = 1;
    rule.conditions[2].dev_no = LOGIC_WILDCARD;
    rule.conditions[2].sensor_type = SENSOR_ANY;
    rule.conditions[2].alarm_level = LV_ANY;
    rule.token_count = 5;
    rule.tokens[0].type = TOK_COND;  rule.tokens[0].cond_idx = 0;
    rule.tokens[1].type = TOK_AND;   rule.tokens[1].cond_idx = 0;
    rule.tokens[2].type = TOK_COND;  rule.tokens[2].cond_idx = 1;
    rule.tokens[3].type = TOK_AND;   rule.tokens[3].cond_idx = 0;
    rule.tokens[4].type = TOK_COND;  rule.tokens[4].cond_idx = 2;
    rule.exec_mode = EXEC_START_ALL;
    rule.action_count = 1;
    rule.actions[0].loop_no = 1;
    rule.actions[0].dev_no = 5;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[3] = rule;
    ResetRuntime(3);

    /*--- 默认规则5：(手动报警) → 控制回路1设备1(声光) ---*/
    memset(&rule, 0, sizeof(LogicRule_t));
    rule.enable = 1;
    rule.rule_id = 5;
    rule.cond_count = 1;
    rule.conditions[0].loop_no = 1;
    rule.conditions[0].dev_no = LOGIC_WILDCARD;
    rule.conditions[0].sensor_type = SENSOR_HAND_REPORT;
    rule.conditions[0].alarm_level = LV_ANY;
    rule.token_count = 1;
    rule.tokens[0].type = TOK_COND;  rule.tokens[0].cond_idx = 0;
    rule.exec_mode = EXEC_START_ALL;
    rule.action_count = 1;
    rule.actions[0].loop_no = 1;
    rule.actions[0].dev_no = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[4] = rule;
    ResetRuntime(4);
}

/*--------------------------------------------------------------
 * 初始化与查询函数注册
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicExpr_SetQueryFunc
 * 功能描述：注册设备状态查询回调函数
 * 该函数通常在LogicDev_Register()中调用。
 * 若不注册则表达式求值时所有条件均返回0
 *--------------------------------------------------------------*/
void LogicExpr_SetQueryFunc(QueryCondFunc_t func)
{
    s_query_func = func;  /* 存储回调函数 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicExpr_Init
 * 功能描述：模块初始化，执行以下步骤：
 * 1. 清空规则表
 * 2. 尝试从Flash加载规则
 * 3. 若Flash无效则加载默认规则并保存
 * 4. 设置初始化标志
 *
 * 本函数幂等可重复调用，多次调用不会产生副作用。
 *--------------------------------------------------------------*/
void LogicExpr_Init(void)
{
    /* 步骤1：清空RAM中的规则表 */
    LogicRule_ClearAll();

    /* 步骤2：尝试从Flash加载规则 */
    if (LogicRule_LoadAll() != 0)
    {
        /* Flash中无有效数据，加载默认规则 */
        /* 加载5条默认规则到RAM */
        LoadDefault();

        /* 将默认规则保存到Flash供下次启动 */
        LogicRule_SaveAll();
    }
    else if (LogicExpr_GetCrcFailCount() > 0)
    {
        /* Flash数据CRC校验失败（典型：结构体尺寸变更后旧数据不兼容，
         * 如dev_no由uint8改为uint16使LogicRule_t由138变154字节）
         * 旧数据无法解析，重新生成默认规则并覆盖保存 */
        DebugPrintf("[LOGIC] Init: crc_fail=%d, regenerate defaults\r\n",
                    LogicExpr_GetCrcFailCount());
        LoadDefault();
        LogicRule_SaveAll();
    }

    /* 步骤3：设置初始化完成标志 */
    s_initialized = 1;

    DebugPrintf("[LOGIC] Init done: %d rules in table\r\n", LogicRule_GetCount());
}

/*--------------------------------------------------------------
 * 函数说明：获取规则表首地址，供外部遍历64个槽位
 * 外部模块可通过该指针遍历规则表中所有规则
 *--------------------------------------------------------------*/
LogicRule_t * LogicExpr_GetTable(void)
{
    return s_rule_table;  /* 返回规则表首地址 */
}

/*--------------------------------------------------------------
 * 函数说明：获取运行时状态数组首地址
 * 外部模块可通过该指针访问所有规则的运行时状态
 *--------------------------------------------------------------*/
RtRuntime * LogicExpr_GetRuntime(void)
{
    return s_runtime;  /* 返回运行时状态数组首地址 */
}

/*--------------------------------------------------------------
 * 函数说明：查询模块是否已完成初始化
--------------------------------------------------------------*/
uint8_t LogicExpr_IsInitialized(void)
{
    return s_initialized;  /* 返回初始化标志 */
}

/*--------------------------------------------------------------
 * 函数说明：返回上次 LoadAll 时CRC校验失败次数
 * 该计数器在每次LoadAll时复位，统计CRC校验失败的规则数
 * 0=全部校验通过, >0=有规则校验失败
--------------------------------------------------------------*/
uint8_t LogicExpr_GetCrcFailCount(void)
{
    return s_crc_fail_count;
}
