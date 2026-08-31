/*==============================================================
 * 文件名称   : bsp_logic_dev.c
 * 模块功能   : 联动逻辑设备抽象层（实现文件）
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 功能描述   : 将硬件传感器和继电器操作抽象为统一接口：
 *              - 查询条件状态（按回路号分回路路由到getter函数）
 *              - 控制执行设备（按回路号+设备号路由到继电器/风机）
 *              - 检查手/自动模式
 *
 * 层级定位   : HARDWARE ABSTRACTION
 *
 * 5位编码说明: 条件/动作均使用5位编码
 *              前2位=回路号, 后3位=设备号
 *              条件: loop_no=探测回路, dev_no=探测器设备号
 *              动作: loop_no=控制回路, dev_no=执行设备号
 *
 * 分回路路由:
 *   探测回路1(loop_no=1) → MBus复合探测器(bsp_mbus.c)
 *              温度/烟雾报警记忆getter，仅支持两种传感器
 *   探测回路2(loop_no=2) → MBus控制设备(bsp_mbus_control.c)
 *              通用设备状态，无传感器子级别，忽略sensor_type
 *   探测回路3(loop_no=3) → RS485探测器(bsp_rs485_detect.c)
 *              支持全7种传感器（温度/烟雾/CO/H2/VOC/CH4/压力）
 *   控制回路2(loop_no=2) → MBus控制设备(声光报警器等),
 *               经MBusCtrl_Request异步控制队列下发, 不直接占用UART2
 *               channel=1-4按通道位掩码下发, 99=全部通道
 *   控制回路1/3(loop_no=1/3) → 不支持控制(回路1板载继电器控制已删除)
 *
 * 依赖模块   :
 *   - bsp_logic_expr.h    : Cond_t, ExecMode
 *   - bsp_logic_engine.h  : SetControlFunc, SetAutoModeFunc
 *   - cmd_process.h       : zhu_state, bei_state
 *   - bsp_mbus.h          : 回路1复合探测器getter
 *   - bsp_mbus_control.h  : 回路2设备状态/在线getter
 *   - bsp_rs485_detect.h  : 回路3传感器状态/在线getter
 *   - bsp_relay.h         : SoundLightRelayCtrl, OutFire1/2RelayCtrl等
 *   - bsp_ctrl_bus.h      : Fan1CtrlOpen/Close
 *   - bsp_internal_board.h: getSysHandAutoState, KEY_AUTO
 *   - bsp_key.h           : getHandPaperState
 *==============================================================*/

#include "bsp_logic_dev.h"          /* 自身头文件 */
#include "bsp_logic_engine.h"       /* 需要LogicEngine_SetControlFunc等 */
#include "cmd_process.h"            /* zhu_state, bei_state */
#include "bsp_mbus.h"               /* 回路1复合探测器getter函数 */
#include "bsp_mbus_control.h"       /* 回路2设备状态/在线getter函数 */
#include "bsp_rs485_detect.h"       /* 回路3传感器状态/在线getter函数 */
#include "bsp_relay.h"              /* 继电器控制函数 */
#include "bsp_ctrl_bus.h"           /* 风机控制函数 */
#include "bsp_internal_board.h"     /* 手/自动模式查询 */
#include "bsp_key.h"                /* getHandPaperState() */
#include "bsp_debug.h"              /* DebugPrintf: test trace on UART4 */
#include "bsp_storage_event.h"  /* A8-1: 黑匣子联动事件存储接口(仅集成层引用, 引擎层经回调解耦) */

/*--------------------------------------------------------------
 * 第一部分：设备状态查询
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：MatchAlarmLevel
 * 功能说明：将传入的报警状态与条件中的报警级别进行匹配。
 *
 * 传感器状态getter的返回值含义：
 *   0 = 无报警（正常）
 *   1 = 低报警 / 超下限 / 报警记忆
 *   2 = 高报警 / 超上限
 *   8 = 故障
 *
 * 参数说明：state - 传感器当前状态值（0/1/2/8等）
 *           level - 条件中的报警级别（AlarmLevel枚举）
 * 返回值：   1=匹配, 0=不匹配
 *--------------------------------------------------------------*/
static uint8_t MatchAlarmLevel(uint8_t state, uint8_t level)
{
    switch (level)
    {
    case LV_ANY:
        /* 任意非零状态都匹配（只要有报警即可） */
        return (state != 0) ? 1 : 0;

    case LV_HIGH:
        /* 高报警：状态2 = 超上限 */
        return (state == 2) ? 1 : 0;

    case LV_LOW:
        /* 低报警：状态1 = 超下限 */
        return (state == 1) ? 1 : 0;

    case LV_FAULT:
        /* 故障：状态8 = 设备故障 */
        return (state == 8) ? 1 : 0;

    default:
        /* 未识别的级别，按任意报警处理 */
        return (state != 0) ? 1 : 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：IsDeviceOnline
 * 功能说明：分回路检查指定设备是否在线。
 *
 * 参数说明：loop_no - 探测回路号（1-3）
 *           dev_no  - 设备号
 * 返回值：   1=在线, 0=离线或回路未知
 *--------------------------------------------------------------*/
static uint8_t IsDeviceOnline(uint8_t loop_no, uint8_t dev_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus复合探测器在线状态 */
        return (getPointTypeMixtureDetectOnlineState(dev_no) != 0) ? 1 : 0;

    case 2:
        /* 回路2: MBus控制设备在线标志 */
        return (MBusCtrl_GetOnline(dev_no) != 0) ? 1 : 0;

    case 3:
        /* 回路3: RS485探测器在线判断（上线且未掉线） */
        return (RS485Detect_IsOnline(dev_no) != 0) ? 1 : 0;

    default:
        /* 其他回路暂未对接，视为离线 */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：GetLoopScanMax
 * 功能说明：获取指定回路的通配符扫描设备数上限。
 *
 * 参数说明：loop_no - 探测回路号（1-3）
 * 返回值：   该回路最大设备数
 *--------------------------------------------------------------*/
static uint8_t GetLoopScanMax(uint8_t loop_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus复合探测器最大地址 */
        return MIXTURE_DEVICE_MAX_ADDR;

    case 2:
        /* 回路2: MBus控制设备最大数（地址1-63） */
        return MBUS_CONTROL_MAX_DEVICES;

    case 3:
        /* 回路3: RS485探测器最大数（地址1-32） */
        return RS485_DETECT_MAX_DEVICES;

    default:
        /* 其他回路返回0，不扫描 */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：ReadSensorState
 * 功能说明：按回路号路由到对应getter函数读取传感器状态。
 *           不直接访问任何全局状态数组，全部通过getter访问。
 *
 * 分回路路由:
 *   回路1: MBus复合探测器，仅支持温度/烟雾（报警记忆0/1二值）
 *   回路2: MBus控制设备，忽略sensor_type返回通用设备状态
 *   回路3: RS485探测器，支持全7种传感器
 *
 * 参数说明：loop_no - 探测回路号（1-3）
 *           dev_no  - 设备号
 *           sensor_type - 传感器类型（SensorType枚举）
 * 返回值：   传感器状态值：0=正常, 1=低, 2=高, 8=故障
 *--------------------------------------------------------------*/
static uint8_t ReadSensorState(uint8_t loop_no, uint8_t dev_no, uint8_t sensor_type)
{
    uint8_t i;           /* 循环计数器 */
    uint8_t max_state;   /* 任意传感器最大状态值 */
    uint8_t sensor_idx;  /* RS485传感器索引 */
    uint8_t s;           /* 单个传感器状态临时变量 */

    /* 按回路号路由 */
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus复合探测器
         * 只支持温度、烟雾两种传感器
         * getter返回0/1二值（报警记忆） */
        if (dev_no > MIXTURE_DEVICE_MAX_ADDR)
        {
            return 0;  /* 设备号超出范围 */
        }

        if (sensor_type == SENSOR_TEMPERATURE)
        {
            /* 温度报警记忆 */
            return getPointTypeMixtureDetectTempertureMemory(dev_no);
        }
        else if (sensor_type == SENSOR_SMOKE)
        {
            /* 烟雾报警记忆 */
            return getPointTypeMixtureDetectSmokeMemory(dev_no);
        }
        else if (sensor_type == SENSOR_ANY)
        {
            /* 任意传感器: 温度 OR 烟雾 */
            if (getPointTypeMixtureDetectTempertureMemory(dev_no) != 0 ||
                getPointTypeMixtureDetectSmokeMemory(dev_no) != 0)
            {
                return 1;
            }
            return 0;
        }
        else
        {
            /* CH4/CO/H2/VOC/压力 — 回路1不支持，返回0 */
            return 0;
        }

    case 2:
        /* 回路2: MBus控制设备
         * 无传感器子级别，忽略sensor_type
         * 返回通用设备状态值(0/1/2/8等) */
        if (dev_no > MBUS_CONTROL_MAX_DEVICES)
        {
            return 0;  /* 设备号超出范围 */
        }
        return MBusCtrl_GetDeviceState(dev_no);

    case 3:
        /* 回路3: RS485探测器
         * 支持全7种传感器 */
        if (dev_no > RS485_DETECT_MAX_DEVICES)
        {
            return 0;  /* 设备号超出范围 */
        }

        if (sensor_type == SENSOR_ANY)
        {
            /* 任意传感器: 取所有传感器状态最大值 */
            max_state = 0;
            for (i = 0; i < (uint8_t)RS485_SENSOR_COUNT; i++)
            {
                s = RS485Detect_GetSensorState(dev_no, i);
                if (s > max_state)
                {
                    max_state = s;
                }
            }
            return max_state;
        }

        /* SensorType -> RS485SensorIndex 映射 */
        switch (sensor_type)
        {
        case SENSOR_TEMPERATURE:
            sensor_idx = RS485_SENSOR_TEMPERATURE;  /* 0 */
            break;
        case SENSOR_SMOKE:
            sensor_idx = RS485_SENSOR_SMOKE;        /* 1 */
            break;
        case SENSOR_CARBON:
            sensor_idx = RS485_SENSOR_CO;           /* 2 */
            break;
        case SENSOR_HYDROGEN:
            sensor_idx = RS485_SENSOR_H2;           /* 3 */
            break;
        case SENSOR_VOC:
            sensor_idx = RS485_SENSOR_VOC;          /* 4 */
            break;
        case SENSOR_METHANE:
            sensor_idx = RS485_SENSOR_CH4;          /* 5 */
            break;
        case SENSOR_PRESSURE:
            sensor_idx = RS485_SENSOR_PRESSURE;     /* 6 */
            break;
        default:
            /* 未识别的传感器类型，返回无报警 */
            return 0;
        }
        return RS485Detect_GetSensorState(dev_no, sensor_idx);

    default:
        /* 其他探测回路暂未对接，返回无报警 */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicDev_QueryCond
 * 功能说明：检查单个条件当前是否满足。
 *           条件使用5位编码: loop_no(探测回路号) + dev_no(设备号)
 *
 * 逻辑流程：
 * 1. 对于SENSOR_HAND_REPORT：检查getHandPaperState()
 * 2. 对于SENSOR_POWER：检查zhu_state/bei_state
 * 3. 对于其他传感器：
 *    - 如果dev_no == 0xFF（通配符）：分回路扫描该回路所有在线探测器
 *    - 如果dev_no是特定值：先分回路检查在线状态，然后读取状态并匹配报警级别
 *    - 如果是LV_OFFLINE：设备离线则返回1
 *    - 回路1的温度/烟雾/ANY：getter返回0/1二值，alarm_level统一按LV_ANY匹配
 *
 * 参数说明：cond - 条件指针（loop_no, dev_no, sensor_type, alarm_level）
 * 返回值：   1=条件满足, 0=不满足
 *--------------------------------------------------------------*/
uint8_t LogicDev_QueryCond(const Cond_t *cond)
{
    uint8_t i;              /* 通配符扫描循环计数器 */
    uint8_t state;          /* 传感器当前状态值 */
    uint8_t scan_max;       /* 通配符扫描设备数上限 */
    uint8_t effective_level;/* 实际匹配用的报警级别 */

    /* 验证条件指针 */
    if (cond == NULL)
    {
        return 0;  /* 空指针：条件不满足 */
    }

    /*--- 特殊传感器类型的处理 ---*/

    /* 手报按钮：检查getHandPaperState() */
    if (cond->sensor_type == SENSOR_HAND_REPORT)
    {
        /* getHandPaperState()返回0=未按下, 0xF0=已按下 */
        state = getHandPaperState();
        /* 任意非零状态表示手报已触发 */
        return (state != 0) ? 1 : 0;
    }

    /* 电源传感器：检查zhu_state或bei_state */
    if (cond->sensor_type == SENSOR_POWER)
    {
        /* dev_no决定检查哪个电源：
         * 1 = 主电（zhu_state）
         * 2 = 备电（bei_state）
         * 0xFF 或 0 = 检查任意电源异常 */
        if (cond->dev_no == 1)
        {
            /* 主电：zhu_state 0=正常, 1=异常 */
            return (zhu_state != 0) ? 1 : 0;
        }
        else if (cond->dev_no == 2)
        {
            /* 备电：bei_state 0=正常, 非0=异常 */
            return (bei_state != 0) ? 1 : 0;
        }
        else
        {
            /* 任意电源异常都返回1 */
            return (zhu_state != 0 || bei_state != 0) ? 1 : 0;
        }
    }

    /*--- 通用传感器处理（分回路路由）---*/

    /* 获取该回路扫描上限（同时校验回路有效性） */
    scan_max = GetLoopScanMax(cond->loop_no);
    if (scan_max == 0)
    {
        return 0;  /* 未知回路：条件不满足 */
    }

    /* 通配符：扫描该回路所有探测器 */
    if (cond->dev_no == LOGIC_WILDCARD)
    {
        /* 如果是LV_OFFLINE：检查是否有任何探测器离线 */
        if (cond->alarm_level == LV_OFFLINE)
        {
            for (i = 0; i < scan_max; i++)
            {
                if (IsDeviceOnline(cond->loop_no, i) == 0)
                {
                    return 1;  /* 找到离线探测器 */
                }
            }
            return 0;  /* 所有探测器都在线 */
        }

        /* 其他报警级别：扫描所有在线探测器 */
        for (i = 0; i < scan_max; i++)
        {
            /* 跳过离线探测器（只检查在线设备的报警状态） */
            if (IsDeviceOnline(cond->loop_no, i) == 0)
            {
                continue;  /* 设备离线，跳过 */
            }

            /* 读取该探测器的传感器状态 */
            state = ReadSensorState(cond->loop_no, i, cond->sensor_type);

            /* 回路1的温度/烟雾/ANY：报警级别统一按LV_ANY匹配 */
            if (cond->loop_no == 1 &&
                (cond->sensor_type == SENSOR_TEMPERATURE ||
                 cond->sensor_type == SENSOR_SMOKE ||
                 cond->sensor_type == SENSOR_ANY))
            {
                if (state != 0)
                {
                    return 1;  /* 回路1有报警记忆即满足 */
                }
            }
            else if (MatchAlarmLevel(state, cond->alarm_level))
            {
                return 1;  /* 找到匹配的报警探测器 */
            }
        }
        return 0;  /* 没有任何探测器匹配 */
    }

    /*--- 特定设备号的处理 ---*/

    /* 如果是LV_OFFLINE：检查指定探测器是否离线 */
    if (cond->alarm_level == LV_OFFLINE)
    {
        return (IsDeviceOnline(cond->loop_no, cond->dev_no) == 0) ? 1 : 0;
    }

    /* 分回路检查探测器是否在线 */
    if (IsDeviceOnline(cond->loop_no, cond->dev_no) == 0)
    {
        return 0;  /* 设备离线：条件不满足 */
    }

    /* 读取该探测器的传感器状态 */
    state = ReadSensorState(cond->loop_no, cond->dev_no, cond->sensor_type);

    /* 回路1的温度/烟雾/ANY：getter返回0/1二值（报警记忆），无级别区分
     * alarm_level 统一按 LV_ANY 处理 */
    effective_level = cond->alarm_level;
    if (cond->loop_no == 1 &&
        (cond->sensor_type == SENSOR_TEMPERATURE ||
         cond->sensor_type == SENSOR_SMOKE ||
         cond->sensor_type == SENSOR_ANY))
    {
        effective_level = LV_ANY;
    }

    /* 检查状态是否匹配报警级别 */
    return MatchAlarmLevel(state, effective_level);
}

/*--------------------------------------------------------------
 * 第二部分：设备控制
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_Control
 * 功能说明：按控制回路号+设备号+通道号对执行设备进行控制（启动或停止）。
 *           7位编码: loop_no(控制回路号) + dev_no(设备号) + channel(通道号)
 *
 * 控制路由(仅支持控制回路2, 回路1板载继电器控制已删除):
 *   loop_no=2, dev_no=设备地址, channel=1-4/99:
 *     经MBusCtrl_Request异步控制队列下发, 不直接占用UART2。
 *     channel=1-4: 按通道位掩码(DEVICE_OUTPUT_1~4)下发;
 *     channel=99:  全部通道(当前映射为DEVICE_OUTPUT_1)。
 *   loop_no=1/3: 不支持控制, 返回失败。
 *
 * 参数说明：loop_no - 控制回路号（仅支持2）
 *           dev_no  - 设备号（回路2设备物理地址1-63）
 *           channel - 输出通道号(1-4=具体通道, 99=全部通道)
 *           action  - 1=启动/打开, 0=停止/关闭
 * 返回值：   0=请求已接受, 1=请求失败（设备未知/不支持/队列满）
 *--------------------------------------------------------------*/
uint8_t LogicDev_Control(uint8_t loop_no, uint8_t dev_no, uint8_t channel, uint8_t action)
{
    MBusCtrlRequest request;
    uint32_t mask;

    /* 控制动作仅支持控制回路2(回路1板载继电器控制已删除, 回路3无控制能力) */
    if (loop_no != 2U)
    {
        DebugPrintf("[LOGIC-CTRL] loop %d not controllable\r\n", loop_no);
        return 1U;
    }

    /* 通道号转输出位掩码: 1-4=具体通道, 99=全部通道
     * 说明: 当前全工程唯一支持输出控制的设备为声光(XR-SGBJQ),
     * 仅支持 DEVICE_OUTPUT_1 通道, 故"全部通道"映射为 DEVICE_OUTPUT_1。
     * 未来出现多通道输出设备时, 需在此扩展为该设备支持的全部通道位。 */
    if (channel == 99U)
    {
        mask = DEVICE_OUTPUT_1;
    }
    else if ((channel >= 1U) && (channel <= 4U))
    {
        mask = (1UL << (channel - 1U));
    }
    else
    {
        DebugPrintf("[LOGIC-CTRL] invalid ch %d\r\n", channel);
        return 1U;
    }

    request.addr          = (uint8_t)dev_no;
    request.operation     = MBUS_OPERATION_SET_OUTPUT;
    request.target_mask   = mask;
    request.target_value  = (action != 0U) ? mask : 0U;

    {
        MBusCtrlResult result = MBusCtrl_Request(&request);
        DebugPrintf("[LOGIC-CTRL] loop2 dev=%d ch=%d mask=0x%X %s ret=%d\r\n",
                    dev_no, channel, (unsigned)mask, action ? "START" : "STOP", (int)result);
        /* LogicDev_Control原约定: 0=请求已接受, 1=请求失败 */
        return (result == MBUS_CTRL_ACCEPTED) ? 0U : 1U;
    }
}

/*--------------------------------------------------------------
 * 第三部分：自动模式检查
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_CheckAutoMode
 * 功能说明：根据执行模式检查系统/分区的手/自动状态是否允许执行。
 *
 * 参数说明：exec_mode - 执行模式（ExecMode枚举）
 * 返回值：   1=允许执行动作, 0=不允许（手动模式下阻止）
 *--------------------------------------------------------------*/
uint8_t LogicDev_CheckAutoMode(uint8_t exec_mode)
{
    uint8_t sys_auto;    /* 系统总自动模式状态 */
    uint8_t part1_auto;  /* 分区1自动模式状态 */
    uint8_t part2_auto;  /* 分区2自动模式状态 */

    /* 读取系统总开关和各分区的手/自动状态 */
    /* KEY_AUTO = 0x02, KEY_MANUAL = 0x01 */
    sys_auto   = (getSysHandAutoState()   == KEY_AUTO) ? 1 : 0;
    part1_auto = (getPart1HandAutoState() == KEY_AUTO) ? 1 : 0;
    part2_auto = (getPart2HandAutoState() == KEY_AUTO) ? 1 : 0;

    /* 根据执行模式判断是否允许 */
    switch (exec_mode)
    {
    case EXEC_START_ALL:
    case EXEC_STOP_ALL:
        /* 要求所有分区都在自动模式才允许 */
        return (sys_auto && part1_auto && part2_auto) ? 1 : 0;

    case EXEC_START_PART:
    case EXEC_STOP_PART:
        /* 任意一个分区在自动模式即允许 */
        return (sys_auto || part1_auto || part2_auto) ? 1 : 0;

    default:
        /* 未识别的模式，默认不允许执行 */
        return 0;  /* 未知模式：安全起见返回不允许 */
    }
}

/*--------------------------------------------------------------
 * 第四部分：初始化注册
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_Register
 * 功能说明：将本模块的函数注册到表达式层和引擎层。
 * 此函数在系统初始化期间调用一次，建立函数指针连接。
 *
 * 调用顺序（在main任务中）：
 *   1. LogicExpr_Init();      （加载Flash或默认规则）
 *   2. LogicDev_Register();   （注册函数 - 连接各层）
 *   3. LogicEngine_Init();    （初始化引擎运行时状态）
 *   4. [RTOS任务启动]          （开始周期执行引擎）
 *--------------------------------------------------------------*/
/* A8-1: 联动动作事件包装回调 - 转发到黑匣子存储(异步队列, 不阻塞引擎100ms周期) */
static void LinkageEventNotify(uint8_t loop_no, uint16_t dev_no,
                               uint8_t channel, uint8_t action)
{
    (void)loop_no;  /* 联动控制固定回路2, 黑匣子记录无回路字段, 此处仅保留语义 */
    StorageEvent_LogLinkageAction((uint8_t)dev_no, channel, action);
}

void LogicDev_Register(void)
{
    /* 注册设备状态查询函数到表达式层 */
    LogicExpr_SetQueryFunc(LogicDev_QueryCond);

    /* 注册设备控制函数到引擎层 */
    LogicEngine_SetControlFunc(LogicDev_Control);

    /* 注册自动模式检查函数到引擎层 */
    LogicEngine_SetAutoModeFunc(LogicDev_CheckAutoMode);

    /* 注册联动动作事件回调(黑匣子打点) */
    LogicEngine_SetEventFunc(LinkageEventNotify);
}
