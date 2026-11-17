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
#include "bsp_logic_dev.h"       /* XR5000_COND_LEAF_LATCH_20261112: LogicDev_ClearLeafLatch */

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
static uint8_t s_blocked_printed[LOGIC_RULE_MAX];  /* V7: 分区停止闭锁提示的限频标志 */
static uint8_t s_ever_triggered = 0U;  /* V8: 本次上电是否触发过联动, 触发后恒1 */

/* 延时启动: 每条规则的条件首次满足时刻与延时中标志 */
static uint32_t s_delay_start_tick[LOGIC_RULE_MAX];
static uint8_t  s_retry_cnt[LOGIC_RULE_MAX];      /* V7: 动作下发失败后的重试计数(失败不清零, 成功/复位清) */
static uint8_t  s_delaying[LOGIC_RULE_MAX];
static uint8_t  s_manual_start_req[LOGIC_RULE_MAX]; /* 手动启动请求: 由分区启动键登记, 延时到点执行后清除 */

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
    uint32_t zone_delay_ms; /* V6: 本规则联动延时(按分区取值, zone=0为0) */
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

        /* V6: 联动延时按规则所属分区取值; zone=0(全局规则)不延时 */
        if (rules[i].zone == 1U)
        {
            zone_delay_ms = (uint32_t)FireZone_GetZoneDelay(1U) * 1000U;
        }
        else if (rules[i].zone == 2U)
        {
            zone_delay_ms = (uint32_t)FireZone_GetZoneDelay(2U) * 1000U;
        }
        else
        {
            zone_delay_ms = 0U;
        }

        /* Test trace: print cond result only on edge (avoid 100ms spam) */
        if (cond_met != s_prev_cond[i])
        {
            s_prev_cond[i] = cond_met;
            if (cond_met)
            {
                s_denied_printed[i] = 0;  /* new cond window, allow denied msg again */
                s_blocked_printed[i] = 0;
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
                /* V7 诊断: 闭锁期间原本静默跳过, 补一条限频打印, 用于区分
                 * "条件不成立" 与 "被分区停止闭锁"; 条件窗口内只打一次。 */
                if (s_blocked_printed[i] == 0U)
                {
                    s_blocked_printed[i] = 1U;
                    DebugPrintf("[LOGIC] R%d BLOCKED by zone%u stop latch (need zone reset)\r\n",
                                rules[i].rule_id, (unsigned)rules[i].zone);
                }
                s_delaying[i] = 0U;
                s_retry_cnt[i] = 0U;      /* 闭锁期间不累计延时 */
                s_manual_start_req[i] = 0U;
                s_denied_printed[i] = 0U;
                continue;                /* 无条件跳过本规则, 状态机保持不动 */
            }
            if (cond_met)
            {
                /* 条件满足，检查本规则所属分区是否允许自动执行 */
                exec_now = CheckAutoAllowed(&rules[i]);
                if (exec_now == 0U)
                {
                    /* 手动模式联动入口(两条来源, 都按本区联动延时执行):
                     *   1) 分区启动键已登记 s_manual_start_req, 延时到点由本函数下发;
                     *   2) 外联设备"联动启动"键按下, 首次检测到即登记请求。
                     * 不再跳过延时: 延时灯与画面86 倒计时按 s_delaying 显示。 */
                    if (s_manual_start_req[i] != 0U)
                    {
                        exec_now = 1U;
                    }
                    else if ((s_manualstart_func != NULL) && (s_manualstart_func() != 0U))
                    {
                        exec_now = 1U;
                        s_manual_start_req[i] = 1U;  /* 键为瞬时信号, 登记后靠请求位保持 */
                        s_delaying[i] = 0U;          /* 从按下时刻重新计时 */
                        s_retry_cnt[i] = 0U;
                        DebugPrintf("[LOGIC] R%d manual START key -> delay %ums\r\n",
                                    rules[i].rule_id, (unsigned)zone_delay_ms);
                    }
                }

                if (exec_now)
                {
                    s_ever_triggered = 1U;   /* V8: 一旦联动逻辑触发, 本次上电期间恒置位 */
                    /* 首次满足: 记录延时起点(手动/自动共用同一套计时) */
                    if (s_delaying[i] == 0U)
                    {
                        s_delaying[i] = 1U;
                        s_delay_start_tick[i] = HAL_GetTick();
                        s_retry_cnt[i] = 0U;
                        DebugPrintf("[LOGIC] R%d delay start %ums\r\n",
                                    rules[i].rule_id, (unsigned)zone_delay_ms);
                    }

                    /* 延时已到: 逐条下发; 失败的动作保留, 下一拍继续重试 */
                    if ((uint32_t)(HAL_GetTick() - s_delay_start_tick[i]) >= zone_delay_ms)
                    {
                        uint8_t all_done = 1U;   /* 本轮是否已把全部动作下发成功 */

                        for (j = 0; j < rules[i].action_count; j++)
                        {
                            if (rt[i].action_done[j] != 0U)
                            {
                                continue;        /* 已成功的动作不再重复下发 */
                            }
                            if (ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                                              rules[i].actions[j].channel, EffectiveAction(&rules[i], j)) == 0U)
                            {
                                rt[i].action_done[j] = 1;   /* 受理成功 */
                            }
                            else
                            {
                                all_done = 0U;             /* 失败: 保留 action_done=0 待重试 */
                            }
                        }

                        if (all_done != 0U)
                        {
                            s_delaying[i] = 0U;
                            s_retry_cnt[i] = 0U;
                            s_manual_start_req[i] = 0U;  /* 手动启动请求已消费 */
                            rt[i].state = RT_EXECUTED;  /* 全部动作已执行 */
                            DebugPrintf("[LOGIC] R%d all %d action(s) done\r\n",
                                        rules[i].rule_id, rules[i].action_count);
                        }
                        else
                        {
                            /* 保持 s_delaying=1 与延时起点, 下拍继续重试失败动作 */
                            if (s_retry_cnt[i] < 255U)
                            {
                                s_retry_cnt[i]++;
                            }
                            if ((s_retry_cnt[i] == 1U) || ((s_retry_cnt[i] % 50U) == 0U))
                            {
                                DebugPrintf("[LOGIC] R%d exec retry #%u (device not ready)\r\n",
                                            rules[i].rule_id, (unsigned)s_retry_cnt[i]);
                            }
                        }
                    }
                }
                /* 不允许执行: 取消本次延时, 等待手动介入 */
                else
                {
                    s_delaying[i] = 0U;
                    s_retry_cnt[i] = 0U;
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
                /* 条件未满足: 清延时状态与手动启动请求(条件消失即作废) */
                s_delaying[i] = 0U;
                s_retry_cnt[i] = 0U;
                s_manual_start_req[i] = 0U;
            }
            break;

        /*--- EXECUTED状态：输出已锁存，只能人工停止 ---*/
        case RT_EXECUTED:
            s_delaying[i] = 0U;
            s_retry_cnt[i] = 0U;  /* 防御: 已执行态清除延时中标志 */
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
            s_delaying[i] = 0U;
            s_retry_cnt[i] = 0U;             /* 清除延时中标志 */
            s_delay_start_tick[i] = 0U;     /* 清除延时起点时刻 */
            break;

        /*--- 未知状态，安全复位到IDLE ---*/
        default:
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
            s_delaying[i] = 0U;
            s_retry_cnt[i] = 0U;             /* 清除延时中标志 */
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
uint8_t LogicEngine_ManualStartZone(uint8_t zone)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    uint8_t i;
    uint8_t requested = 0U;   /* 登记的手动启动请求数 */
    uint32_t delay_ms;

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return 0U;

    s_ever_triggered = 1U;   /* V8: 手动启动也算触发过联动 */

    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();
    delay_ms = (uint32_t)FireZone_GetZoneDelay(zone) * 1000U;

    for (i = 0; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U) continue;
        if (rules[i].enable == 0U) continue;
        if (rules[i].zone != zone) continue;               /* 只处理本分区规则 */
        if (LogicExpr_Evaluate(&rules[i]) == 0U) continue;  /* 条件不成立不启动 */
        if (rt[i].state == RT_EXECUTED) continue;           /* 已启动的不重复登记 */

        /* 已停止(RT_DONE)的规则重新拉回 IDLE, 允许再次按启动键 */
        if (rt[i].state != RT_IDLE)
        {
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
        }

        /* 需求: 手动启动同样走本区联动延时, 这里只登记请求并起延时,
         * 实际下发由 LogicEngine_Run 在延时到点后统一执行;
         * 延时灯与画面86 倒计时据此显示。 */
        s_manual_start_req[i] = 1U;
        if (s_delaying[i] == 0U)
        {
            s_delaying[i] = 1U;
            s_delay_start_tick[i] = HAL_GetTick();
            s_retry_cnt[i] = 0U;
        }
        requested++;
        DebugPrintf("[LOGIC] R%d manual START zone%d -> delay %ums\r\n",
                    rules[i].rule_id, (int)zone, (unsigned)delay_ms);
    }

    return requested;
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_ManualStopZone
 * 功能说明：分区手动停止。停止本分区内所有已执行的启动类动作,
 *           清除本区延时计时。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 *--------------------------------------------------------------*/
uint8_t LogicEngine_ManualStopZone(uint8_t zone)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    uint8_t i, j;
    uint8_t stopped = 0U;    /* 实际下发 STOP 的动作数 */

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX)) return 0U;

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
                stopped++;
            }
        }
        s_delaying[i] = 0U;
        s_retry_cnt[i] = 0U;
        s_manual_start_req[i] = 0U;
        rt[i].state = RT_DONE;
        DebugPrintf("[LOGIC] R%d manual STOP zone%d\r\n", rules[i].rule_id, (int)zone);
    }

    LogicDev_ClearLeafLatchByZone(zone);   /* V7: 只清本分区探测器产生的条件锁存 */
    return stopped;
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

        s_delaying[i] = 0U;
        s_retry_cnt[i] = 0U;            /* 清除延时中标志 */
        s_delay_start_tick[i] = 0U;    /* 清除延时起点时刻 */
        s_manual_start_req[i] = 0U;
    }
    LogicDev_ClearLeafLatchByZone(zone);   /* V7: 只清本分区探测器产生的条件锁存 */
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
/*--------------------------------------------------------------
 * 函数名称：LogicEngine_GetActionViews
 * 功能说明：V8 导出联动动作执行视图，供画面86 联动监测区(控件42~48)显示。
 *           按触发时刻升序(先触发的先显示)，最多max条。
 *           state: 0=延时中 1=已下发待反馈 2=反馈已启动(本函数不填) 3=已停止/失败
 *--------------------------------------------------------------*/
uint8_t LogicEngine_GetActionViews(LogicActionView_t *out, uint8_t max)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    LogicActionView_t item;
    uint8_t  i;
    uint8_t  j;
    uint8_t  count = 0U;
    uint8_t  pos;
    uint32_t now;
    uint32_t delay_ms;
    uint32_t elapsed;
    uint32_t remain_ms;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }
    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();
    now   = HAL_GetTick();

    for (i = 0U; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U)
        {
            continue;
        }
        if (rules[i].enable == 0U)
        {
            continue;
        }
        for (j = 0U; j < rules[i].action_count; j++)
        {
            item.state    = 0U;
            item.remain_s = 0U;

            /* 分区停止闭锁: 本规则动作统一显示"已停止" */
            if ((rules[i].zone >= 1U) && (rules[i].zone <= (uint8_t)FIRE_ZONE_MAX) &&
                (s_zone_stop_latch[rules[i].zone - 1U] != 0U))
            {
                item.state = 3U;
            }
            else if (rt[i].state == RT_EXECUTED)
            {
                if (rt[i].action_done[j] == 0U)
                {
                    continue;
                }
                item.state = 1U;
            }
            else if (rt[i].state == RT_DONE)
            {
                item.state = 3U;
            }
            else if (rt[i].state == RT_IDLE)
            {
                if (s_delaying[i] != 0U)
                {
                    /* 延时按规则所属分区取值, zone=0 恒0 */
                    if (rules[i].zone == 1U)
                    {
                        delay_ms = (uint32_t)FireZone_GetZoneDelay(1U) * 1000U;
                    }
                    else if (rules[i].zone == 2U)
                    {
                        delay_ms = (uint32_t)FireZone_GetZoneDelay(2U) * 1000U;
                    }
                    else
                    {
                        delay_ms = 0U;
                    }
                    elapsed = now - s_delay_start_tick[i];
                    remain_ms = (elapsed >= delay_ms) ? 0U : (delay_ms - elapsed);
                    item.remain_s = (uint16_t)((remain_ms + 999U) / 1000U);
                    item.state = 0U;
                }
                else if (rt[i].action_done[j] != 0U)
                {
                    item.state = 3U;    /* 下发失败, 正在重试 */
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            item.valid        = 1U;
            item.rule_id      = rules[i].rule_id;
            item.loop_no      = rules[i].actions[j].loop_no;
            item.dev_no       = rules[i].actions[j].dev_no;
            item.channel      = rules[i].actions[j].channel;
            item.trigger_tick = s_delay_start_tick[i];

            /* 按触发时刻插入排序: 先触发的排前面 */
            pos = (count < max) ? count : max;
            while (pos > 0U)
            {
                if (out[pos - 1U].trigger_tick <= item.trigger_tick)
                {
                    break;
                }
                if (pos < max)
                {
                    out[pos] = out[pos - 1U];
                }
                pos--;
            }
            if (pos < max)
            {
                out[pos] = item;
            }
            if (count < max)
            {
                count++;
            }
        }
    }
    return count;
}

/* V8: 本次上电期间是否触发过联动逻辑(供画面86停止"联动检测运行中"刷新) */
uint8_t LogicEngine_HasEverTriggered(void)
{
    return (s_ever_triggered != 0U) ? 1U : 0U;
}

/*--------------------------------------------------------------
 * 函数名称：LogicEngine_GetZoneLinkState
 * 功能说明：汇总指定分区当前联动状态, 供面板分区 LED 驱动使用。
 *           位0=本区有规则处于启动延时中, 位1=本区有启动类动作已下发。
 *           停止类规则不计入"已启动", 避免停止动作点亮喷洒启动灯。
 * 输入参数：zone - 分区号(1..FIRE_ZONE_MAX)
 * 返回值  ：LOGIC_ZONE_ST_* 位掩码, 参数非法返回0
 *--------------------------------------------------------------*/
uint8_t LogicEngine_GetZoneLinkState(uint8_t zone)
{
    LogicRule_t *rules;
    RtRuntime   *rt;
    uint8_t i;
    uint8_t state = 0U;

    if ((zone < 1U) || (zone > (uint8_t)FIRE_ZONE_MAX))
    {
        return 0U;
    }

    rules = LogicExpr_GetTable();
    rt    = LogicExpr_GetRuntime();

    for (i = 0U; i < LOGIC_RULE_MAX; i++)
    {
        if (rules[i].rule_id == 0U) continue;
        if (rules[i].enable == 0U) continue;
        if (rules[i].zone != zone) continue;

        if (s_delaying[i] != 0U)
        {
            state |= LOGIC_ZONE_ST_DELAYING;
        }
        if (rt[i].state == RT_EXECUTED)
        {
            if ((rules[i].exec_mode != EXEC_STOP_ALL) &&
                (rules[i].exec_mode != EXEC_STOP_PART))
            {
                state |= LOGIC_ZONE_ST_ACTIVE;
            }
        }
    }

    return state;
}
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
        s_delaying[i] = 0U;
        s_retry_cnt[i] = 0U;                     /* 清除延时中标志 */
        s_delay_start_tick[i] = 0U;             /* 清除延时起点时刻 */
        s_manual_start_req[i] = 0U;
    }

    LogicDev_ClearLeafLatch();  /* XR5000_COND_LEAF_LATCH_20261112: 系统复位全表清一次 */
    s_ever_triggered = 0U;  /* V8: 系统复位清除"曾触发"标志 */
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
