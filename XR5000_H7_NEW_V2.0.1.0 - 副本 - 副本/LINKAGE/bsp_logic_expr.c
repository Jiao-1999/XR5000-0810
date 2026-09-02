/*==============================================================
 * 文件名称   : bsp_logic_expr.c
 * 模块功能   : 联动逻辑表达式管理与解析实现文件。
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 提供逻辑规则的新增删除查询与表达式求值功能。
 *              - 规则CRUD操作（创建/修改/删除/查询/枚举）；
 *              - Flash持久化存储（W25Qxx SPI Flash）；
 *              - 递归下降表达式求值，支持运算符优先级；
 *              - 默认规则加载。
 *
 * 架构定位   : MODEL层，Flash持久化与表达式求值。
 *              设备无关控制，仅通过回调查询设备。
 *
 * 依赖模块   :
 *   - w25qxx.h        : Flash读/写/擦除
 *   - system.h        : CalcCrc16()
 *   - cmsis_os.h      : osDelay()（纯RTOS延时等待）
 *   - stm32h7xx_hal.h : HAL_GetTick()
 *==============================================================*/

#include "bsp_logic_expr.h"     /* 逻辑规则数据结构与API声明 */
#include "w25qxx.h"             /* W25QXX_Read/Write/Erase_Sector */
#include "system.h"             /* CalcCrc16() */
#include "cmsis_os.h"           /* osDelay()（纯RTOS延时等待） */
#include "stm32h7xx_hal.h"      /* HAL_GetTick() */
#include "bsp_debug.h"          /* DebugPrintf: test trace on UART4 */
#include <string.h>             /* memset/memcpy */

/*--------------------------------------------------------------
 * 全局变量定义
 *--------------------------------------------------------------*/

/* 规则表：64个槽位，存储于RAM中，可保存到Flash */
static LogicRule_t s_rule_table[LOGIC_RULE_MAX];

/* 运行时状态表：仅存储于RAM中，不写入Flash */
static RtRuntime   s_runtime[LOGIC_RULE_MAX];

/* 模块初始化标志：0=未初始化, 1=已初始化 */
static uint8_t     s_initialized = 0;

/* 设备状态查询回调：由bsp_logic_dev.c注册
 * 为NULL时表达式求值时所有条件返回0 */
static QueryCondFunc_t s_query_func = NULL;

/* CRC校验失败计数：在LoadAll时统计CRC失败数量（供上层查询）*/
static uint8_t s_crc_fail_count = 0;

/* Flash元数据结构：5字节对齐packed，
 * 存储魔数与有效规则数量（位于LOGIC_FLASH_META_ADDR） */
typedef struct
{
    uint32_t magic;       /* 魔数 "LOG1" = 0x4C4F4731 */
    uint8_t  rule_count;  /* 写入Flash的有效规则数量 */
} __attribute__((packed)) LogicMeta_t;

/*--------------------------------------------------------------
 * 内部函数声明：static函数需要在使用前先声明
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：FindSlotById
 * 功能说明：按指定ID在规则表中查找对应槽位
 * 参数说明：id - 要查找的规则ID（1-255）
 * 返回值：  找到的槽位号（0-63），未找到返回0xFF
 *--------------------------------------------------------------*/
static uint8_t FindSlotById(uint8_t id)
{
    uint8_t i;  /* 循环计数器 */

    /* 校验ID为0表示空槽位，无效 */
    if (id == 0)
    {
        return 0xFF;  /* 无效ID直接返回未找到 */
    }

    /* 遍历规则表 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* 使用槽位的ID匹配 */
        if (s_rule_table[i].rule_id == id)
        {
            return i;  /* 返回匹配的槽位号 */
        }
    }

    return 0xFF;  /* 遍历所有槽位仍未找到 */
}

/*--------------------------------------------------------------
 * 函数名称：FindEmptySlot
 * 功能说明：在规则表中查找首个空槽位
 * 返回值：  空槽位序号（0-63），满表时返回0xFF
 *--------------------------------------------------------------*/
static uint8_t FindEmptySlot(void)
{
    uint8_t i;  /* 循环计数器 */

    /* 查找空槽位 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* rule_id == 0表示空槽位 */
        if (s_rule_table[i].rule_id == 0)
        {
            return i;  /* 返回空槽位 */
        }
    }

    return 0xFF;  /* 规则表已满，无空槽位 */
}

/*--------------------------------------------------------------
 * 函数名称：IsRuleValid
 * 功能说明：校验规则所有字段是否全部合法
 * 说明：ID有效、条件值在合法范围内、动作
 *         Token中的条件索引在有效范围内。
 * 参数说明：rule - 待校验的规则结构体
 * 返回值：  1=有效, 0=无效
 *
 * 设备号范围说明（分回路差异化校验）：
 *   回路2(MBus控制回路)设备地址为1-63（含60-62扩展地址）
 *   其余回路设备号限制在0-29
 *--------------------------------------------------------------*/
static uint8_t IsRuleValid(const LogicRule_t *rule)
{
    uint8_t i;    /* 循环计数器 */
    uint8_t lno;  /* 条件回路号临时变量 */
    uint16_t dno;  /* 条件设备号临时变量(3位数字000-999,故用uint8特列处理) */

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

    /* 校验每个动作的延时与通道是否在有效范围内 */
    for (i = 0; i < rule->action_count; i++)
    {
        /* 动作延时时间不能超过600秒 */
        if (rule->actions[i].delay_s > LOGIC_DELAY_MAX_S)
        {
            return 0;  /* 延时超范围 */
        }
        /* 动作仅支持控制回路2(回路1原继电器逻辑已删除,回路3无控制能力) */
        if (rule->actions[i].loop_no != 2U)
        {
            return 0;  /* 控制回路非法:仅支持控制回路2 */
        }
        /* 通道为1-4为指定通道, 99为全部通道, 否则非法 */
        if ((rule->actions[i].channel < 1U) || (rule->actions[i].channel > 4U))
        {
            if (rule->actions[i].channel != 99U)
            {
                return 0;  /* 通道号非法 */
            }
        }
    }

    /* 校验条件中的回路号和设备号范围（分回路差异化校验）
     * loop_no 只能为 1-4 或 0xFF（通配扫描）
     * dev_no: 回路2为0-63（支持地址60-62扩展设备），
     *         其余回路限制在0-29（0xFF通配） */
    for (i = 0; i < rule->cond_count; i++)
    {
        lno = rule->conditions[i].loop_no;
        dno = rule->conditions[i].dev_no;
        if (lno != LOGIC_WILDCARD && (lno == 0 || lno > 4))
        {
            return 0;  /* 回路号非法（允许 1-4 或 0xFF） */
        }
        if (dno != LOGIC_WILDCARD)
        {
            if (lno == 2)
            {
                /* 回路2设备地址范围1-63（含60-62扩展地址） */
                if (dno > 63)
                {
                    return 0;  /* 回路2设备号非法 */
                }
            }
            else
            {
                /* 其余回路设备号限制在0-29 */
                if (dno >= 30)
                {
                    return 0;  /* 设备号非法 */
                }
            }
        }
    }

    /* 校验Token中的括号是否成对（括号数量必须完全匹配）*/
    {
        int8_t paren_depth = 0;  /* 括号深度计数值：>0=未匹配, <0=多余 */
        for (i = 0; i < rule->token_count; i++)
        {
            if (rule->tokens[i].type == TOK_COND)
            {
                /* 条件Token中的索引必须在有效范围内 */
                if (rule->tokens[i].cond_idx >= rule->cond_count)
                {
                    return 0;  /* 条件索引越界 */
                }
            }
            else if (rule->tokens[i].type == TOK_LPAREN)
            {
                paren_depth++;  /* 左括号入栈深度加一 */
            }
            else if (rule->tokens[i].type == TOK_RPAREN)
            {
                paren_depth--;  /* 右括号出栈深度减一 */
                if (paren_depth < 0)
                {
                    return 0;  /* 右括号多于左括号，配对失败 */
                }
            }
        }
        /* 括号深度最终不为0，说明左右括号不匹配 */
        if (paren_depth != 0)
        {
            return 0;  /* 括号未完全配对，规则无效 */
        }
    }

    return 1;  /* 所有检查通过，规则有效 */
}

/*--------------------------------------------------------------
 * 函数名称：ResetRuntime
 * 功能说明：将指定槽位的运行时状态重置为IDLE态
 * 在Create/Modify/Delete/SetEnable时调用
 * 清除历史执行痕迹重新开始计时
 * 参数说明：slot - 槽位号（0-63）
 *--------------------------------------------------------------*/
static void ResetRuntime(uint8_t slot)
{
    /* 清除整个运行时状态结构体 */
    memset(&s_runtime[slot], 0, sizeof(RtRuntime));

    /* 设置运行时状态为IDLE（所有动作完成标志清零，准备重新计时） */
    s_runtime[slot].state = RT_IDLE;
}

/*--------------------------------------------------------------
 * 函数名称：CalcRuleCRC
 * 功能说明：计算规则结构体的CRC16校验值
 * CRC计算时排除末尾2字节（crc字段，由外部填充）
 * 参数说明：rule - 待计算的规则
 * 返回值：  CRC16值
 *--------------------------------------------------------------*/
static uint16_t CalcRuleCRC(const LogicRule_t *rule)
{
    /* 计算结构体CRC时排除crc字段（最后2字节）
     * sizeof(LogicRule_t) = 138字节，crc字段占最后2字节
     * 实际计算136字节得到CRC */
    return CalcCrc16((uint8_t *)rule, sizeof(LogicRule_t) - 2);
}

/*--------------------------------------------------------------
 * 规则CRUD操作（创建/修改/删除/查询）
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Create
 * 功能说明：创建新规则写入规则表
 * 参数说明：id - 规则ID（1-255），不能与已有规则重复
 *           rule - 待创建的规则结构体
 * 返回值：  0=成功, 1=ID已存在, 2=表已满, 3=参数无效
 *--------------------------------------------------------------*/
uint8_t LogicRule_Create(uint8_t id, const LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位号 */

    /* 参数校验 */
    if (rule == NULL || id == 0)
    {
        return 3;  /* 参数无效 */
    }

    /* 校验结构合法性 */
    if (!IsRuleValid(rule))
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: invalid structure\r\n", id);
        return 3;  /* 规则结构无效 */
    }

    /* 检查ID是否已存在，防止重复创建 */
    if (FindSlotById(id) != 0xFF)
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: id exists\r\n", id);
        return 1;  /* ID已存在 */
    }

    /* 在表中查找空槽位 */
    slot = FindEmptySlot();
    if (slot == 0xFF)
    {
        DebugPrintf("[LOGIC] Rule %d create FAIL: table full\r\n", id);
        return 2;  /* 表已满 */
    }

    /* 创建写入对应槽位 */
    s_rule_table[slot] = *rule;

    /* 强制rule_id为外部传入的id */
    s_rule_table[slot].rule_id = id;

    /* 计算并存储CRC（供Flash保存时校验） */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置该槽位运行时状态 */
    ResetRuntime(slot);

    DebugPrintf("[LOGIC] Rule %d created: conds=%d tokens=%d actions=%d\r\n",
                id, rule->cond_count, rule->token_count, rule->action_count);
    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Modify
 * 功能说明：修改规则表中指定ID的规则
 * 参数说明：id - 待修改的规则ID（不能改变ID）
 *           rule - 修改后的规则结构体
 * 返回值：  0=成功, 1=未找到, 3=参数无效
 *--------------------------------------------------------------*/
uint8_t LogicRule_Modify(uint8_t id, const LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位号 */

    /* 参数校验 */
    if (rule == NULL || id == 0)
    {
        return 3;  /* 参数无效 */
    }

    /* 校验结构合法性 */
    if (!IsRuleValid(rule))
    {
        return 3;  /* 规则结构无效 */
    }

    /* 查找规则所在槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 覆盖旧规则数据 */
    s_rule_table[slot] = *rule;

    /* 保留原规则ID */
    s_rule_table[slot].rule_id = id;

    /* 重新计算CRC */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置运行时状态（清除历史执行痕迹） */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Delete
 * 功能说明：按ID删除规则，清空对应槽位并重置运行时状态
 * 参数说明：id - 待删除的规则ID
 * 返回值：  0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_Delete(uint8_t id)
{
    uint8_t slot;  /* 目标槽位号 */

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 清空槽位数据（全部清零，rule_id为0表示空） */
    memset(&s_rule_table[slot], 0, sizeof(LogicRule_t));

    /* 重置运行时状态 */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_Get
 * 功能说明：按ID获取规则到外部缓冲区
 * 参数说明：id - 待查询的规则ID
 *           rule - 外部缓冲区（用于接收规则数据，由调用方分配）
 * 返回值：  0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_Get(uint8_t id, LogicRule_t *rule)
{
    uint8_t slot;  /* 目标槽位号 */

    /* 空指针检查 */
    if (rule == NULL)
    {
        return 1;  /* 输出缓冲区无效 */
    }

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 拷贝规则到输出 */
    *rule = s_rule_table[slot];

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_GetCount
 * 功能说明：统计规则表中有效规则的数量
 * 返回值：  有效规则数量（0..64）
 *--------------------------------------------------------------*/
uint8_t LogicRule_GetCount(void)
{
    uint8_t i;      /* 循环计数器 */
    uint8_t count = 0;  /* 有效规则计数 */

    /* 遍历所有槽位 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* rule_id != 0表示该槽位为有效规则 */
        if (s_rule_table[i].rule_id != 0)
        {
            count++;  /* 累计有效规则数 */
        }
    }

    return count;  /* 返回有效规则总数 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_SetEnable
 * 功能说明：设置用户启用/禁用，同时重置运行时状态
 * 参数说明：id - 规则ID
 *           enable - 1=启用, 0=禁用
 * 返回值：  0=成功, 1=未找到
 *--------------------------------------------------------------*/
uint8_t LogicRule_SetEnable(uint8_t id, uint8_t enable)
{
    uint8_t slot;  /* 目标槽位号 */

    /* 查找槽位 */
    slot = FindSlotById(id);
    if (slot == 0xFF)
    {
        return 1;  /* 规则未找到 */
    }

    /* 设置启用状态 */
    s_rule_table[slot].enable = enable;

    /* 重新计算CRC（enable字段参与CRC计算） */
    s_rule_table[slot].crc = CalcRuleCRC(&s_rule_table[slot]);

    /* 重置运行时状态（相应启用/禁用状态变化） */
    ResetRuntime(slot);

    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_ClearAll
 * 功能说明：清除规则表中所有槽位，置为初始状态。
 *--------------------------------------------------------------*/
void LogicRule_ClearAll(void)
{
    uint8_t i;  /* 循环计数器 */

    /* 清除所有规则 */
    memset(s_rule_table, 0, sizeof(s_rule_table));

    /* 清除所有运行时状态 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        ResetRuntime(i);  /* 重置每个槽位的运行时状态为IDLE */
    }
}

/*--------------------------------------------------------------
 * Flash持久化操作
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicRule_SaveAll
 * 功能说明：将所有规则（含无效规则）保存到Flash（W25Qxx芯片）。
 * 存储策略：先擦除写区，写完后顺序存储
 * 返回值：  0=成功, 1=失败
 *--------------------------------------------------------------*/
uint8_t LogicRule_SaveAll(void)
{
    uint8_t  i;              /* 循环计数器 */
    uint16_t write_offset;   /* Flash写入偏移（按字节定位） */
    LogicMeta_t meta;        /* 待写入的元数据 */

    /* 写入前先擦除Flash（Flash特性决定必须先擦后写） */
    /* 规则区占3个扇区（138字节*64=8832 > 8192(2扇区)），故3个4KB扇区 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR);          /* 擦除规则存储扇区1 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR + 0x1000); /* 擦除规则存储扇区2 */
    W25QXX_Erase_Sector(LOGIC_FLASH_RULES_ADDR + 0x2000); /* 擦除规则存储扇区3 */
    W25QXX_Erase_Sector(LOGIC_FLASH_META_ADDR);           /* 擦除元数据存储扇区 */

    /* 写入所有有效规则到Flash */
    write_offset = 0;  /* 写入偏移从0开始 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* 只写具有有效ID的规则 */
        if (s_rule_table[i].rule_id != 0)
        {
            /* 将规则结构体写入Flash对应位置 */
            W25QXX_Write(
                (uint8_t *)&s_rule_table[i],           /* 源：RAM中的规则结构体 */
                LOGIC_FLASH_RULES_ADDR + write_offset, /* 目标：Flash存储地址 */
                sizeof(LogicRule_t)                     /* 大小：138字节 */
            );
            /* 累计写入偏移 */
            write_offset += sizeof(LogicRule_t);
        }
    }

    /* 写入元数据：魔数 + 规则数量 */
    meta.magic = LOGIC_MAGIC;                /* "LOG1"魔数用于校验有效性 */
    meta.rule_count = LogicRule_GetCount();  /* 写入有效规则数量 */

    W25QXX_Write(
        (uint8_t *)&meta,               /* 元数据结构地址 */
        LOGIC_FLASH_META_ADDR,          /* 元数据存储地址 */
        sizeof(LogicMeta_t)             /* 大小：5字节 */
    );

    DebugPrintf("[LOGIC] SaveAll: %d rules saved to Flash 0x080000\r\n", meta.rule_count);
    return 0;  /* 成功 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_WipeAllPersistent
 * 功能说明：一次性持久化清除所有规则（RAM + Flash）。
 *           擦空后再次开机（从Flash重载旧规则）也不会触发默认规则装载。
 *           原理为SaveAll时写入 magic=LOGIC_MAGIC + rule_count=0 的元数据，
 *             LoadAll校验通过但循环0次，LogicExpr_Init不会走LoadDefault路径。
 * 用法     ：在 main.c 的 LogicExpr_Init() 之后调用一次，清除已配置和默认的
 *--------------------------------------------------------------*/
void LogicRule_WipeAllPersistent(void)
{
    LogicRule_ClearAll();     /* 1. 清除RAM规则表 + 重置所有运行时状态 */
    (void)LogicRule_SaveAll();  /* 2. 擦除Flash 4个扇区 + 写入元数据(rule_count=0) */
    DebugPrintf("[LOGIC] WipeAllPersistent: rules wiped (RAM+Flash), reboot stays empty\r\n");
}

/*--------------------------------------------------------------
 * 函数名称：LogicRule_LoadAll
 * 功能说明：从Flash装载所有规则到RAM。
 * 装载时：每条规则做CRC校验
 * 返回值：  0=成功, 1=Flash无有效规则
 *--------------------------------------------------------------*/
uint8_t LogicRule_LoadAll(void)
{
    uint8_t  i;              /* 循环计数器 */
    uint16_t read_offset;    /* Flash读取偏移（按字节定位） */
    LogicMeta_t meta;        /* 从Flash读取的元数据 */
    LogicRule_t temp_rule;   /* 临时存放从Flash读取的规则 */

    s_crc_fail_count = 0;    /* 复位CRC失败计数器 */

    /* 先从Flash读取元数据 */
    W25QXX_Read(
        (uint8_t *)&meta,           /* 元数据结构地址 */
        LOGIC_FLASH_META_ADDR,      /* 元数据在Flash中的地址 */
        sizeof(LogicMeta_t)         /* 大小：5字节 */
    );

    /* 校验魔数是否匹配，Flash未写入时为0xFFFFFFFF */
    if (meta.magic != LOGIC_MAGIC)
    {
        DebugPrintf("[LOGIC] LoadAll: Flash empty (bad magic), no rules\r\n");
        return 1;  /* Flash未初始化，读取数据被认为是装载失败 */
    }

    /* 清除RAM规则表 */
    LogicRule_ClearAll();

    /* 从Flash顺序读取规则 */
    read_offset = 0;  /* 读取偏移从0开始 */
    for (i = 0; i < meta.rule_count && i < LOGIC_RULE_MAX; i++)
    {
        /* 从Flash读取一条规则到临时缓冲 */
        W25QXX_Read(
            (uint8_t *)&temp_rule,                  /* 临时规则结构地址 */
            LOGIC_FLASH_RULES_ADDR + read_offset,   /* 规则在Flash中的地址 */
            sizeof(LogicRule_t)                      /* 大小：138字节 */
        );

        /* 校验其CRC是否与Flash一致 */
        if (CalcRuleCRC(&temp_rule) == temp_rule.crc)
        {
            /* CRC校验通过，将有效规则装入RAM中 */
            /* 占用对应的规则的槽位 */
            s_rule_table[i] = temp_rule;
            ResetRuntime(i);  /* 重置运行时状态准备装载 */
        }
        else
        {
            /* CRC失败时丢弃该规则，增加失败计数 */
            s_crc_fail_count++;
        }

        /* 累加读取偏移 */
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
 *   expr   -> term ('|' term)*          // OR表达式（最低优先级）
 *   term   -> factor ('&' factor)*      // AND表达式（中间优先级）
 *   factor -> '!' factor                // NOT一元运算符（最高优先级）
 *          | '(' expr ')'              // 括号分组（提高优先级）
 *          | condition                 // 条件原子（查询探测器状态）
 *
 * 语法错误时全部失败返回0（保证不触发动作）。
 * 优先级顺序：NOT > AND > OR（与C语言一致）。
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 内部函数前向声明
 *--------------------------------------------------------------*/
static uint8_t eval_expr(const LogicRule_t *rule, uint8_t *pos);
static uint8_t eval_term(const LogicRule_t *rule, uint8_t *pos);
static uint8_t eval_factor(const LogicRule_t *rule, uint8_t *pos);

/*--------------------------------------------------------------
 * 函数名称：eval_expr
 * 功能说明：对表达式求值OR表达式（最高层）
 * OR表达式采用多路求值，依次求值term项，用'|'分隔
 * 任一term为真则结果为1，否则为0
 *
 * 参数说明：rule - 含tokens和conditions的规则结构体
 *           pos - Token数组中的当前位置指针（按引用传递）
 * 返回值：  1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_expr(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */

    /* 先求值第一项term */
    result = eval_term(rule, pos);

    /* 循环求值剩下的OR项 */
    while (*pos < rule->token_count && rule->tokens[*pos].type == TOK_OR)
    {
        (*pos)++;  /* 跳过OR token */
        /* 求值下一项term，按位或到结果 */
        result |= eval_term(rule, pos);
    }

    return result;  /* 返回所有term按OR运算的结果 */
}

/*--------------------------------------------------------------
 * 函数名称：eval_term
 * 功能说明：对表达式求值AND表达式（中间层）
 * AND表达式采用多路求值，依次求值factor项，用'&'分隔
 * 所有factor为真才为1，否则为0
 *
 * 参数说明：rule - 含tokens和conditions的规则结构体
 *           pos - Token数组中的当前位置指针
 * 返回值：  1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_term(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */

    /* 先求值第一项factor */
    result = eval_factor(rule, pos);

    /* 循环求值剩下的AND项 */
    while (*pos < rule->token_count && rule->tokens[*pos].type == TOK_AND)
    {
        (*pos)++;  /* 跳过AND token */
        /* 求值下一项factor，按位与到结果 */
        result &= eval_factor(rule, pos);
    }

    return result;  /* 返回所有factor按AND运算的结果 */
}

/*--------------------------------------------------------------
 * 函数名称：eval_factor
 * 功能说明：对表达式求值factor（最底层原子），factor可以是表达式
 * factor支持三种形式：
 *   - NOT factor: 对factor取反后返回
 *   - ( expr ): 对括号范围内的子表达式递归求值
 *   - condition: 查询探测器实时状态
 *
 * 参数说明：rule - 含tokens和conditions的规则结构体
 *           pos - Token数组中的当前位置指针
 * 返回值：  1=真, 0=假
 *--------------------------------------------------------------*/
static uint8_t eval_factor(const LogicRule_t *rule, uint8_t *pos)
{
    uint8_t result;  /* 求值结果 */
    uint8_t idx;     /* 条件索引变量 */

    /* 越界检查：Token数量到达越界 */
    if (*pos >= rule->token_count)
    {
        return 0;  /* 到了结尾无Token，全部失败返回0 */
    }

    /* 情形1：NOT一元运算符 - 递归求值后factor取反 */
    if (rule->tokens[*pos].type == TOK_NOT)
    {
        (*pos)++;  /* 跳过NOT token */
        /* 递归求值后factor取反 */
        return (uint8_t)(!eval_factor(rule, pos));
    }

    /* 情形2：左括号 - 递归求值括号内表达式 */
    if (rule->tokens[*pos].type == TOK_LPAREN)
    {
        (*pos)++;  /* 跳过'(' token */
        /* 递归求值括号内的子表达式 */
        result = eval_expr(rule, pos);
        /* 匹配右括号，如果找不到就忽略 */
        if (*pos < rule->token_count && rule->tokens[*pos].type == TOK_RPAREN)
        {
            (*pos)++;  /* 跳过')' token */
        }
        return result;  /* 返回括号内子表达式的结果 */
    }

    /* 情形3：条件 - 查询探测器状态 */
    if (rule->tokens[*pos].type == TOK_COND)
    {
        idx = rule->tokens[*pos].cond_idx;  /* 从Token中取条件索引 */
        (*pos)++;  /* 跳过条件token */

        /* 越界检查：条件索引是否在有效范围内 */
        if (idx >= rule->cond_count)
        {
            return 0;  /* 索引越界，全部失败 */
        }

        /* 通过回调函数查询探测器实时状态 */
        if (s_query_func != NULL)
        {
            /* 调用查询函数返回该条件是否满足 */
            return s_query_func(&rule->conditions[idx]);
        }

        return 0;  /* 未注册查询函数，默认不满足 */
    }

    /* 情形4：其他Token（RPAREN/END）等 - 语法错误全部失败 */
    (*pos)++;  /* 跳过未知Token */
    return 0;  /* 语法错误，安全返回0 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicExpr_Evaluate
 * 功能说明：对规则的表达式进行递归下降求值
 * 求值时间通过回调查询设备状态
 *
 * 参数说明：rule - 待求值的规则结构体
 * 返回值：  1=条件满足（所有表达式为真）, 0=假
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

    /* 从顶层开始求值（以OR表达式为入口） */
    result = eval_expr(rule, &pos);

    return result;  /* 返回表达式最终求值结果 */
}

/*--------------------------------------------------------------
 * 默认规则部分（无需配置）
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LoadDefault
 * 功能说明：装载5条默认规则到规则表
 * 在Flash无有效数据时使用，防止初始化后无任何规则
 *--------------------------------------------------------------*/
static void LoadDefault(void)
{
    LogicRule_t rule;  /* 临时规则结构体 */

    memset(&rule, 0, sizeof(LogicRule_t));

    /*--- 默认规则1：（回路1任意设备）&（回路1任意设备） -> 控制回路1设备1(声光) ---*/
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
    rule.actions[0].loop_no = 2;
    rule.actions[0].dev_no = 1;
    rule.actions[0].channel = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[0] = rule;
    ResetRuntime(0);

    /*--- 默认规则2：（回路1任意设备） -> 控制回路1设备3(喷放) ---*/
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
    rule.actions[0].loop_no = 2;
    rule.actions[0].dev_no = 3;
    rule.actions[0].channel = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[1] = rule;
    ResetRuntime(1);

    /*--- 默认规则3：（回路1任意设备） -> 控制回路1设备3(喷放) ---*/
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
    rule.actions[0].loop_no = 2;
    rule.actions[0].dev_no = 3;
    rule.actions[0].channel = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[2] = rule;
    ResetRuntime(2);

    /*--- 默认规则4：（回路1任意设备） x3 -> 控制回路1设备5(默认继电器) ---*/
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
    rule.actions[0].loop_no = 2;
    rule.actions[0].dev_no = 5;
    rule.actions[0].channel = 1;
    rule.actions[0].action = 1;
    rule.actions[0].delay_s = 0;
    rule.crc = CalcRuleCRC(&rule);
    s_rule_table[3] = rule;
    ResetRuntime(3);

    /*--- 默认规则5：（手动报警） -> 控制回路1设备1(声光) ---*/
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
    rule.actions[0].loop_no = 2;
    rule.actions[0].dev_no = 1;
    rule.actions[0].channel = 1;
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
 * 功能说明：注册设备状态查询回调函数
 * 该函数通常由LogicDev_Register()中调用。
 * 若不注册，则表达式求值时所有条件返回0
 *--------------------------------------------------------------*/
void LogicExpr_SetQueryFunc(QueryCondFunc_t func)
{
    s_query_func = func;  /* 存储回调函数 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicExpr_Init
 * 功能说明：模块初始化，执行以下步骤：
 * 1. 清空规则表
 * 2. 尝试从Flash重载规则
 * 3. 若Flash无效则载入默认规则并保存
 * 4. 设置初始化标志
 *
 * 本函数可重复调用（幂等），多次调用不会产生副作用。
 *--------------------------------------------------------------*/
void LogicExpr_Init(void)
{
    /* 步骤1：清空RAM中的规则表 */
    LogicRule_ClearAll();

    /* 步骤2：尝试从Flash重载规则 */
    if (LogicRule_LoadAll() != 0)
    {
        /* Flash没有有效数据，载入默认规则 */
        /* 装载5条默认规则RAM */
        LoadDefault();

        /* 将默认规则保存到Flash（下次启动时） */
        LogicRule_SaveAll();
    }
    else if (LogicExpr_GetCrcFailCount() > 0)
    {
        /* Flash存在CRC校验失败，可能是结构体大小变化导致数据不匹配，
         * 如dev_no由uint8改为uint16使LogicRule_t从138变154字节，
         * 旧数据无法正常装载，则重新生成默认规则并覆盖保存 */
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
 * 函数说明：获取规则表首地址（供外部轮询64个槽位）
 * 外部模块可通过指针遍历所有已配置规则
 *--------------------------------------------------------------*/
LogicRule_t * LogicExpr_GetTable(void)
{
    return s_rule_table;  /* 返回规则表首地址 */
}

/*--------------------------------------------------------------
 * 函数说明：获取运行时状态表首地址
 * 外部模块可通过指针遍历所有规则对应的运行时状态
 *--------------------------------------------------------------*/
RtRuntime * LogicExpr_GetRuntime(void)
{
    return s_runtime;  /* 返回运行时状态表首地址 */
}

/*--------------------------------------------------------------
 * 函数说明：查询模块是否完成初始化
--------------------------------------------------------------*/
uint8_t LogicExpr_IsInitialized(void)
{
    return s_initialized;  /* 返回初始化标志 */
}

/*--------------------------------------------------------------
 * 函数说明：返回上次 LoadAll 时CRC校验失败次数
 * 该函数用于每次LoadAll时复位，统计CRC校验失败的规则数
 * 0=全部校验通过, >0=有规则校验失败
--------------------------------------------------------------*/
uint8_t LogicExpr_GetCrcFailCount(void)
{
    return s_crc_fail_count;
}
