/*==============================================================
 * 文件名称   : bsp_logic_engine.c
 * 模块功能   : 联动逻辑引擎（实现文件）
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM, CMSIS-RTOS
 * 功能描述   : 驱动规则评估和动作执行的状态机，作为RTOS任务运行。
 *
 * 状态机
 *   IDLE -> EXECUTED -> DONE -> IDLE
 *   （判定成立后延时 LOGIC_ACTION_DELAY_MS 再执行全部动作;
 *     输出一旦启动即锁存, 报警消失也不自动停止, 只能人工停止/分区复位）
 *
 *   关键改动: EXECUTED态条件消失时，先停止所有已执行的
 *   启动类动作（action=1的动作下发停止命令），再进入DONE。
 *
 * 架构定位   : CONTROLLER
 *              只负责状态机调度和动作触发，
 *              不直接操作硬件，通过回调函数与外部交互。
 *
 * 依赖模块   :
 *   - bsp_logic_expr.h    : 规则表管理和表达式求值
 *   - bsp_logic_engine.h  : 本模块API
 *   - cmsis_os.h          : osDelay等RTOS接口
 *   - stm32h7xx_hal.h     : HAL基础头文件
 *==============================================================*/

#include "bsp_logic_engine.h"   /* 本模块头文件 */
#include "bsp_logic_expr.h"     /* 规则表管理和表达式求值 */
#include "bsp_fire_zone.h"       /* FIRE_ZONE_MAX: 分区手动操作参数校验 */
#include "cmsis_os.h"           /* osDelay() */
#include "stm32h7xx_hal.h"      /* HAL_GetTick() */
#include "bsp_debug.h"          /* DebugPrintf: test trace on UART4 */
#include <string.h>             /* memset */

/*--------------------------------------------------------------
 * 全局变量区
 *--------------------------------------------------------------*/

/* 控制设备回调函数指针，由bsp_logic_dev.c注册 */
static ControlDevFunc_t    s_control_func    = NULL;

/* 检查自动模式回调函数指针，由bsp_logic_dev.c注册 */
static CheckAutoModeFunc_t s_automode_func   = NULL;
static LinkageEventFunc_t   s_event_func     = NULL;  /* 联动动作事件回调(黑匣子打点), 由bsp_logic_dev.c注册 */
static ManualStartFunc_t    s_manualstart_func = NULL;  /* 手动启动键查询回调(手动模式联动入口), 由bsp_logic_dev.c注册 */

/* Test trace: previous cycle cond state per slot, print on edge only */
static uint8_t s_prev_cond[LOGIC_RULE_MAX];
/* Test trace: auto-mode-denied message printed once per cond window */
static uint8_t s_denied_printed[LOGIC_RULE_MAX];

/* 延时启动: 每条规则的条件首次满足时刻与延时中标志 */
static uint32_t s_delay_start_tick[LOGIC_RULE_MAX];
static uint8_t  s_delaying[LOGIC_RULE_MAX];

/* 联动延时启动时间(毫秒), 运行期可改, 缺省 10 秒 */
static uint32_t s_action_delay_ms = LOGIC_ACTION_DELAY_MS;

/* 分区手动停止闭锁: 1=本分区已被停止键闭锁, 条件消失或分区复位才解除 */
static uint8_t s_zone_stop_latch[FIRE_ZONE_MAX];

/*--------------------------------------------------------------
 * 回调注册函数实现
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_SetControlFunc
 * 功能说明：注册控制设备回调函数。
 * 调用时机：LogicDev_Register()中调用。
 *--------------------------------------------------------------*/
void LogicEngine_SetControlFunc(ControlDevFunc_t func)
{
    s_control_func = func;  /* 存储回调指针 */
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_SetAutoModeFunc
 * 功能说明：注册检查自动模式回调函数。
 * 调用时机：LogicDev_Register()中调用。
 *--------------------------------------------------------------*/
void LogicEngine_SetAutoModeFunc(CheckAutoModeFunc_t func)
{
    s_automode_func = func;  /* 存储回调指针 */
}
void LogicEngine_SetEventFunc(LinkageEventFunc_t func)
{
    s_event_func = func;  /* 保存联动动作事件回调指针 */
}
void LogicEngine_SetManualStartFunc(ManualStartFunc_t func)
{
    s_manualstart_func = func;  /* 保存手动启动键查询回调指针 */
}

/*--------------------------------------------------------------
 * 内部工具函数区
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：EffectiveAction
 * 功能说明：计算动作的有效执行值. 停止类模式(EXEC_STOP_ALL/STOP_PART)
 *           统一发停止指令(0), 忽略动作表action位; 启动类模式按动作
 *           表action位执行.
 * 输入参数：rule - 规则结构指针, j - 动作下标
 * 返回值  ：1=启动, 0=停止
 *--------------------------------------------------------------*/
static uint8_t EffectiveAction(const LogicRule_t *rule, uint8_t j)
{
    if ((rule->exec_mode == EXEC_STOP_ALL) || (rule->exec_mode == EXEC_STOP_PART))
    {
        return 0U;  /* 停止模式: 统一发停止指令 */
    }
    return rule->actions[j].action;
}

/*--------------------------------------------------------------
 * 函数名称：ExecuteAction
 * 功能说明：执行单个动作（启动或停止指定设备的指定通道）。
 * 输入参数：loop_no - 控制回路号
 *           dev_no  - 设备号
 *           channel - 输出通道号(1-4具体通道, 99全部通道)
 *           action  - 1=启动, 0=停止
 * 返回值   ：0=成功, 1=失败
 *--------------------------------------------------------------*/
static uint8_t ExecuteAction(uint8_t loop_no, uint8_t dev_no, uint8_t channel, uint8_t action)
{
    uint8_t ret;  /* callback return code */

    /* 检查回调函数是否已注册 */
    if (s_control_func != NULL)
    {
        ret = s_control_func(loop_no, dev_no, channel, action);  /* 调用注册的回调函数 */
        DebugPrintf("[LOGIC] Exec loop=%d dev=%d ch=%d %s -> %s\r\n",
                    loop_no, dev_no, channel, action ? "START" : "STOP", ret ? "FAIL" : "OK");
        /* A8-1: 控制指令受理成功(ret==0)时打点黑匣子联动事件(经回调注入, 异步不阻塞引擎) */
        if ((ret == 0U) && (s_event_func != NULL))
        {
            s_event_func(loop_no, dev_no, channel, action);
        }
        return ret;
    }
    DebugPrintf("[LOGIC] Exec loop=%d dev=%d ch=%d %s -> NO CALLBACK\r\n",
                loop_no, dev_no, channel, action ? "START" : "STOP");
    return 1;  /* 未注册回调函数，返回失败 */
}

/*--------------------------------------------------------------
 * 函数名称：CheckAutoAllowed
 * 功能说明：检查当前规则是否允许自动执行动作(系统总闸 + 所属分区)。
 * 输入参数：rule - 规则结构体指针
 * 返回值   ：1=允许, 0=不允许（手动模式或被禁止）
 *--------------------------------------------------------------*/
static uint8_t CheckAutoAllowed(const LogicRule_t *rule)
{
    /* 检查回调函数是否已注册 */
    if (s_automode_func != NULL)
    {
        return s_automode_func(rule);  /* 调用注册的回调函数 */
    }
    return 0;  /* 未注册回调函数，默认不允许自动执行 */
}

/*--------------------------------------------------------------
 * 核心状态机函数区
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_Run
 * 功能说明：执行一次规则扫描周期，遍历所有规则并更新状态。
 * 遍历流程：检查每条规则条件，根据当前状态执行相应动作。
 *
 * 调用频率：RTOS任务循环，LOGIC_ENGINE_CYCLE_MS（100ms）周期调用。
 *--------------------------------------------------------------*/
static void LogicEngine_Run(void)
{
    uint8_t  i;          /* 规则循环计数器（0..63） */
    uint8_t  j;          /* 动作循环计数器（0..3） */
    uint8_t  cond_met;   /* 条件是否满足标志 */
    uint8_t  exec_now;    /* 本周期允许执行标志(自动模式允许 或 手动启动键按下) */
    uint8_t  manual_pass; /* 手动启动键放行标志(1=跳过延时立即执行) */
    LogicRule_t *rules;  /* 规则表首地址 */
    RtRuntime   *rt;     /* 运行时状态数组首地址 */

    /* 获取规则表和运行时状态数组指针 */
    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();

    /* 遍历所有规则槽位 */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        /* 跳过空槽位（rule_id == 0） */
        if (rules[i].rule_id == 0)
        {
            continue;  /* 空槽位，跳过 */
        }

        /* 检查规则是否启用 */
        if (rules[i].enable == 0)
        {
            continue;  /* 规则被禁用，跳过 */
        }

        /* 评估规则表达式条件是否满足 */
        cond_met = LogicExpr_Evaluate(&rules[i]);

        /* Test trace: print cond result only on edge (avoid 100ms spam) */
        if (cond_met != s_prev_cond[i])
        {
            s_prev_cond[i] = cond_met;
            if (cond_met)
            {
                s_denied_printed[i] = 0;  /* new cond window, allow denied msg again */
            }
            DebugPrintf("[LOGIC] R%d cond %s\r\n", rules[i].rule_id,
                        cond_met ? "MET" : "LOST");
        }

        /* 根据当前运行时状态执行相应逻辑 */
        switch (rt[i].state)
        {
        /*--- IDLE状态：等待条件满足 ---*/
        case RT_IDLE:
            /* 分区被停止键闭锁: 未经分区复位, 即使报警条件再次成立也不再自动执行。
             * 闭锁只有两条解除途径:
             *   - 分区复位键 -> FireZoneReset() -> CancelZoneDelay()/ClearZoneStopLatch()
             *   - 系统复位(整机) -> BspCmdProcessInit() 重建状态
             * zone 必须先做 1..FIRE_ZONE_MAX 校验再作数组下标, 非法时按未闭锁处理。 */
            if ((rules[i].zone >= 1U) && (rules[i].zone <= (uint8_t)FIRE_ZONE_MAX) &&
                (s_zone_stop_latch[rules[i].zone - 1U] != 0U))
            {
                s_delaying[i] = 0U;      /* 闭锁期间不累计延时 */
                s_denied_printed[i] = 0U;
                continue;                /* 无条件跳过本规则, 状态机保持不动 */
            }
            if (cond_met)
            {
                manual_pass = 0U;  /* 默认走自动延时路径 */

                /* 条件满足，检查本规则所属分区是否允许自动执行 */
                exec_now = CheckAutoAllowed(&rules[i]);
                if (exec_now == 0U)
                {
                    /* A9: 手动模式联动入口 - 自动不允许时查询外联设备启动键
                     * 手动启动键按下且条件满足才执行, 条件不满足时按键无效 */
                    if ((s_manualstart_func != NULL) && (s_manualstart_func() != 0U))
                    {
                        exec_now      = 1U;  /* 手动启动键按下, 放行执行 */
                        manual_pass   = 1U;  /* 手动放行不延时 */
                        s_delaying[i] = 0U;
                        DebugPrintf("[LOGIC] R%d manual START key -> exec\r\n",
                                    rules[i].rule_id);
                    }
                }

                if (exec_now)
                {
                    /* 首次满足: 记录延时起点 */
                    if ((s_delaying[i] == 0U) && (manual_pass == 0U))
                    {
                        s_delaying[i] = 1U;
                        s_delay_start_tick[i] = HAL_GetTick();
                        DebugPrintf("[LOGIC] R%d delay start %ums\r\n",
                                    rules[i].rule_id, (unsigned)s_action_delay_ms);
                    }
                    /* 延时已到 或 手动启动键放行: 执行全部动作 */
                    else if ((manual_pass != 0U) ||
                             ((uint32_t)(HAL_GetTick() - s_delay_start_tick[i]) >= s_action_delay_ms))
                    {
                        s_delaying[i] = 0U;
                        DebugPrintf("[LOGIC] R%d delay done, %d action(s)\r\n",
                                    rules[i].rule_id, rules[i].action_count);

                        for (j = 0; j < rules[i].action_count; j++)
                        {
                            ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                                          rules[i].actions[j].channel, EffectiveAction(&rules[i], j));
                            rt[i].action_done[j] = 1;  /* 标记动作已执行 */
                        }

                        rt[i].state = RT_EXECUTED;  /* 全部动作已执行 */
                    }
                }
                /* 不允许执行: 取消本次延时, 等待手动介入 */
                else
                {
                    s_delaying[i] = 0U;
                    /* Test trace: print once per cond window */
                    if (!s_denied_printed[i])
                    {
                        s_denied_printed[i] = 1;
                        DebugPrintf("[LOGIC] R%d cond MET but auto-mode DENIED (check hand/auto keys)\r\n",
                                    rules[i].rule_id);
                    }
                }
            }
            else
            {
                /* 条件未满足: 清延时状态 */
                s_delaying[i] = 0U;
            }
            break;

        /*--- EXECUTED状态：输出已锁存，只能人工停止 ---*/
        case RT_EXECUTED:
            s_delaying[i] = 0U;  /* 防御: 已执行态清除延时中标志 */
            /* 需求: 联动输出一旦启动就保持, 报警条件消失也不自动停止。
             * 只有以下人工操作会下发 STOP:
             *   - 分区停止键   -> LogicEngine_ManualStopZone()
             *   - 分区复位键   -> FireZoneReset() -> ManualStopZone()
             * 因此本状态不再对动作下发 STOP, 保持 EXECUTED 直到人工停止。 */
            break;

        /*--- DONE状态：人工停止后复位到IDLE ---*/
        case RT_DONE:
            /* 复位运行时状态，回到IDLE等待下一次触发 */
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
            s_delaying[i] = 0U;             /* 清除延时中标志 */
            s_delay_start_tick[i] = 0U;     /* 清除延时起点时刻 */
            break;

        /*--- 未知状态，安全复位到IDLE ---*/
        default:
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
            s_delaying[i] = 0U;             /* 清除延时中标志 */
            s_delay_start_tick[i] = 0U;     /* 清除延时起点时刻 */
            break;
        }
    }
}

/*--------------------------------------------------------------
 * 联动延时时间 与 分区手动操作
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_SetActionDelay
 * 功能说明：设置联动延时启动时间(毫秒)。0 按立即执行处理,
 *           大于 60000 截断为 60000。
 * 输入参数：ms - 延时毫秒数
 *--------------------------------------------------------------*/
void LogicEngine_SetActionDelay(uint32_t ms)
{
    if (ms > 60000U)
    {
        ms = 60000U;  /* 上限截断 */
    }
    s_action_delay_ms = ms;  /* uint32_t 单字写, 无需临界区 */
    DebugPrintf("[LOGIC] action delay set %ums\r\n", (unsigned)s_action_delay_ms);
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_GetActionDelay
 * 功能说明：读取联动延时启动时间(毫秒)。
 * 返回值  ：当前延时毫秒数
 *--------------------------------------------------------------*/
uint32_t LogicEngine_GetActionDelay(void)
{
    return s_action_delay_ms;
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_ManualStartZone
 * 功能说明：分区手动启动。立即执行本分区内所有"条件成立"的
 *           规则的全部动作(不经过延时), 供分区启动按键调用。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 *--------------------------------------------------------------*/
void LogicEngine_ManualStartZone(uint8_t zone)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    uint8_t i, j;

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return;

    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();

    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U) continue;
        if (rules[i].enable == 0U) continue;
        if (rules[i].zone != zone) continue;               /* 只处理本分区规则 */
        if (LogicExpr_Evaluate(&rules[i]) == 0U) continue;  /* 条件不成立不启动 */

        for (j = 0; j < rules[i].action_count; j++)
        {
            ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                          rules[i].actions[j].channel, EffectiveAction(&rules[i], j));
            rt[i].action_done[j] = 1U;
        }
        rt[i].state = RT_EXECUTED;
        s_delaying[i] = 0U;
        DebugPrintf("[LOGIC] R%d manual START zone%d (%d action)\r\n",
                    rules[i].rule_id, (int)zone, rules[i].action_count);
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_ManualStopZone
 * 功能说明：分区手动停止。停止本分区内所有已执行的启动类动作,
 *           清除本区延时计时。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 *--------------------------------------------------------------*/
void LogicEngine_ManualStopZone(uint8_t zone)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    uint8_t i, j;

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return;

    s_zone_stop_latch[zone - 1U] = 1U;  /* 闭锁: 条件仍成立也不再自动重启 */

    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();

    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U) continue;
        if (rules[i].zone != zone) continue;  /* 只处理本分区规则 */

        for (j = 0; j < rules[i].action_count; j++)
        {
            if ((rt[i].action_done[j] != 0U) && (EffectiveAction(&rules[i], j) == 1U))
            {
                ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                              rules[i].actions[j].channel, 0U);
                rt[i].action_done[j] = 0U;
            }
        }
        s_delaying[i] = 0U;
        rt[i].state = RT_DONE;
        DebugPrintf("[LOGIC] R%d manual STOP zone%d\r\n", rules[i].rule_id, (int)zone);
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_CancelZoneDelay
 * 功能说明：取消本分区所有规则的延时计时, 用于分区复位。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 *--------------------------------------------------------------*/
void LogicEngine_CancelZoneDelay(uint8_t zone)
{
    LogicRule_t *rules;
    uint8_t i;

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return;

    rules = LogicExpr_GetTable();

    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U) continue;
        if (rules[i].zone != zone) continue;  /* 只处理本分区规则 */

        s_delaying[i] = 0U;            /* 清除延时中标志 */
        s_delay_start_tick[i] = 0U;    /* 清除延时起点时刻 */
    }
    LogicEngine_ClearZoneStopLatch(zone);  /* 分区复位一并解除停止闭锁 */
    DebugPrintf("[LOGIC] zone%d delay cancelled\r\n", (int)zone);
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_ClearZoneStopLatch
 * 功能说明：清除分区停止闭锁。分区复位时调用, 解除"停止键"语义,
 *           之后条件成立仍可正常自动联动。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 *--------------------------------------------------------------*/
void LogicEngine_ClearZoneStopLatch(uint8_t zone)
{
    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return;

    s_zone_stop_latch[zone - 1U] = 0U;
}

/*--------------------------------------------------------------
 * 初始化函数 & RTOS任务区
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_Init
 * 功能说明：初始化引擎运行时状态，将所有规则状态设为IDLE。
 * 调用顺序：LogicExpr_Init()和LogicDev_Register()之后调用。
 *--------------------------------------------------------------*/
void LogicEngine_Init(void)
{
    uint8_t i;      /* 循环计数器 */
    RtRuntime *rt;  /* 运行时状态数组首地址 */

    /* 获取运行时状态数组指针 */
    rt = LogicExpr_GetRuntime();

    /* 初始化所有规则运行时状态为IDLE */
    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        memset(&rt[i], 0, sizeof(RtRuntime));  /* 清零 */
        rt[i].state = RT_IDLE;                  /* 设置为空闲状态 */
        s_delaying[i] = 0U;                     /* 清除延时中标志 */
        s_delay_start_tick[i] = 0U;             /* 清除延时起点时刻 */
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngineTask
 * 功能说明：联动逻辑引擎RTOS任务函数。
 *
 * 执行流程：
 * 1. 延时LOGIC_ENGINE_STARTUP_DELAY等待系统稳定
 * 2. 进入无限循环
 *    a. 检查模块是否已初始化
 *    b. 执行规则扫描周期
 *    c. 延时LOGIC_ENGINE_CYCLE_MS
 *
 * 在freertos.c中的调用方式：
 *   osThreadDef(LogicEngine, LogicEngineTask, osPriorityNormal,
 *               0, LOGIC_ENGINE_TASK_STACK);
 *   osThreadCreate(osThread(LogicEngine), NULL);
 *--------------------------------------------------------------*/
void LogicEngineTask(void *parameter)
{
    /* 避免未使用参数警告 */
    (void)parameter;

    /* 启动延时，等待传感器数据稳定
     * 各回路传感器状态需要时间填充有效数据，
     * 过早评估会导致误触发。 */
    osDelay(LOGIC_ENGINE_STARTUP_DELAY);

    /* 进入主循环，周期性执行规则扫描 */
    while (1)
    {
        /* 检查模块是否已完成初始化 */
        if (LogicExpr_IsInitialized())
        {
            /* 执行一次规则扫描周期，处理所有规则状态机 */
            LogicEngine_Run();
        }

        /* 按周期延时，默认100ms */
        osDelay(LOGIC_ENGINE_CYCLE_MS);
    }
}
