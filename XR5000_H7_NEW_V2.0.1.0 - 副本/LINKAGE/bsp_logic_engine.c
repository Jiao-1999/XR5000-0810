/*==============================================================
 * 文件名称   : bsp_logic_engine.c
 * 模块功能   : 联动逻辑引擎（实现文件）
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM, CMSIS-RTOS
 * 功能描述   : 驱动规则评估和动作执行的状态机，作为RTOS任务运行。
 *
 * 状态机
 *   IDLE -> EXECUTED -> DONE -> IDLE
 *   （判定成立立即执行全部动作，延时机制已删除）
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
 * 功能说明：检查当前是否允许自动执行动作。
 * 输入参数：exec_mode - 执行模式，ExecMode枚举值
 * 返回值   ：1=允许, 0=不允许（手动模式或被禁止）
 *--------------------------------------------------------------*/
static uint8_t CheckAutoAllowed(uint8_t exec_mode)
{
    /* 检查回调函数是否已注册 */
    if (s_automode_func != NULL)
    {
        return s_automode_func(exec_mode);  /* 调用注册的回调函数 */
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
            if (cond_met)
            {
                /* 条件满足，检查是否允许自动执行 */
                exec_now = CheckAutoAllowed(rules[i].exec_mode);
                if (exec_now == 0U)
                {
                    /* A9: 手动模式联动入口 - 自动不允许时查询外联设备启动键
                     * 手动启动键按下且条件满足才执行, 条件不满足时按键无效 */
                    if ((s_manualstart_func != NULL) && (s_manualstart_func() != 0U))
                    {
                        exec_now = 1U;  /* 手动启动键按下, 放行执行(与自动路径一致) */
                        DebugPrintf("[LOGIC] R%d manual START key -> exec\r\n",
                                    rules[i].rule_id);
                    }
                }
                if (exec_now)
                {
                    /* 判定成立, 立即执行全部动作(延时机制已删除) */
                    DebugPrintf("[LOGIC] R%d state IDLE->EXECUTED, %d action(s)\r\n",
                                rules[i].rule_id, rules[i].action_count);

                    for (j = 0; j < rules[i].action_count; j++)
                    {
                        ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                                      rules[i].actions[j].channel, EffectiveAction(&rules[i], j));
                        rt[i].action_done[j] = 1;  /* 标记动作已执行 */
                    }

                    rt[i].state = RT_EXECUTED;  /* 全部动作已执行 */
                }
                /* 不允许自动执行时，保持在IDLE状态等待手动干预 */
                else
                {
                    /* Test trace: print once per cond window */
                    if (!s_denied_printed[i])
                    {
                        s_denied_printed[i] = 1;
                        DebugPrintf("[LOGIC] R%d cond MET but auto-mode DENIED (check hand/auto keys)\r\n",
                                    rules[i].rule_id);
                    }
                }
            }
            break;

        /*--- EXECUTED状态：所有动作已执行，等待条件消失 ---*/
        case RT_EXECUTED:
            if (!cond_met)
            {
                /* 条件消失: 停止所有已执行的启动类动作 */
                for (j = 0; j < rules[i].action_count; j++)
                {
                    if (rt[i].action_done[j] == 1 && EffectiveAction(&rules[i], j) == 1)
                    {
                        /* 停止已启动的设备，传入action=0 */
                        ExecuteAction(rules[i].actions[j].loop_no, rules[i].actions[j].dev_no,
                                      rules[i].actions[j].channel, 0);
                    }
                }
                /* 所有动作已停止，进入DONE状态 */
                rt[i].state = RT_DONE;
                DebugPrintf("[LOGIC] R%d state EXECUTED->DONE (cond lost, started actions stopped)\r\n",
                            rules[i].rule_id);
            }
            break;

        /*--- DONE状态：动作完成，自动复位到IDLE ---*/
        case RT_DONE:
            /* 复位运行时状态，回到IDLE等待下一次触发 */
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
            break;

        /*--- 未知状态，安全复位到IDLE ---*/
        default:
            memset(&rt[i], 0, sizeof(RtRuntime));
            rt[i].state = RT_IDLE;
            break;
        }
    }
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
