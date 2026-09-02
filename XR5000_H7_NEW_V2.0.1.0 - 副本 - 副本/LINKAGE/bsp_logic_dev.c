/*==============================================================
 * 文件名称   : bsp_logic_dev.c
 * 模块功能   : 联动逻辑硬件抽象层(设备状态查询/控制实现文件)
 * 运行平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 功能说明   : 对硬件设备状态查询和控制操作封装为统一接口，
 *              - 查询各回路设备状态（各回路相关 getter 封装）；
 *              - 执行设备（各回路+设备路由到继电器/风机）；
 *              - 手动/自动模式
 *
 * 架构层次   : HARDWARE ABSTRACTION
 *
 * 5位数字说明: 条件/条件动作使用5位数字
 *              前2位=回路号, 后3位=设备号
 *              条件: loop_no=探测回路, dev_no=探测设备号
 *              动作: loop_no=控制回路, dev_no=执行设备号
 *
 * 各回路说明:
 *   探测回路1(loop_no=1) : MBus回路探测器(bsp_mbus.c)
 *              温度/烟雾等探测器 getter 封装支持多种传感器类型
 *   探测回路2(loop_no=2) : MBus回路设备(bsp_mbus_control.c)
 *              通过设备状态寄存器判断报警，配合 sensor_type
 *   探测回路3(loop_no=3) : RS485探测器(bsp_rs485_detect.c)
 *              支持全7种传感器类型（温度/烟雾/CO/H2/VOC/CH4/压力）
 *   控制回路2(loop_no=2) : MBus回路设备（联动输出设备），
 *               通过MBusCtrl_Request异步控制输出设备, 不直接占用UART2
 *               channel=1-4为通道位掩码控制, 99=全部通道
 *   控制回路1/3(loop_no=1/3) : 不支持控制(回路1原控制继电器逻辑已删除)
 *
 * 依赖模块   :
 *   - bsp_logic_expr.h    : Cond_t, ExecMode
 *   - bsp_logic_engine.h  : SetControlFunc, SetAutoModeFunc
 *   - cmd_process.h       : zhu_state, bei_state
 *   - bsp_mbus.h          : 回路1探测器状态 getter
 *   - bsp_mbus_control.h  : 回路2设备状态/控制 getter
 *   - bsp_rs485_detect.h  : 回路3探测器状态/控制 getter
 *   - bsp_relay.h         : SoundLightRelayCtrl, OutFire1/2RelayCtrl等
 *   - bsp_ctrl_bus.h      : Fan1CtrlOpen/Close
 *   - bsp_internal_board.h: getSysHandAutoState, KEY_AUTO
 *   - bsp_key.h           : getHandPaperState
 *==============================================================*/

#include "bsp_logic_dev.h"          /* 自身头文件 */
#include "bsp_logic_engine.h"       /* 需要LogicEngine_SetControlFunc等 */
#include "cmd_process.h"            /* zhu_state, bei_state */
#include "bsp_mbus.h"               /* 回路1探测器状态 getter等 */
#include "bsp_mbus_control.h"       /* 回路2设备状态/控制 getter等 */
#include "bsp_rs485_detect.h"       /* 回路3探测器状态/控制 getter等 */
#include "bsp_relay.h"              /* 声光/继电器控制等 */
#include "bsp_ctrl_bus.h"           /* 风机控制等 */
#include "bsp_internal_board.h"     /* 手/自动模式查询 */
#include "bsp_key.h"                /* getHandPaperState() */
#include "bsp_debug.h"              /* DebugPrintf: test trace on UART4 */
#include "bsp_storage_event.h"  /* A8-1: 黑匣子联动事件存储接口(异步队列, 不阻塞调用) */

/*==============================================================
 * D级自动化测试钩子(当前测试用, 量产必须关闭!)
 * 说明: COM8调试口打印trace, 实测传感器不可用时, 自动化测试用
 *       周期触发联动流程(可选择任意回路), COM17记录黑匣子打点
 * 钩子1: 接管回路1设备1/2的条件查询, 60s周期相位轮转
 *        (0~25s=条件MET / 25~60s=条件LOST), 周期自动触发联动.
 * 钩子2: 绕过MBus控制链路, 直接模拟控制成功ret=0(Level1无硬件测试).
 * 说明: 联动逻辑自测流程详见 联动逻辑自测方案.md Part D
 *==============================================================*/
#define LOGIC_SELF_TEST_EN           0U   /* 1=条件自测钩子开, 0=关闭 */
#define LOGIC_SELF_TEST_PERIOD       60U  /* 模拟周期(秒) */
#define LOGIC_SELF_TEST_ON_S         25U  /* 每周期报警持续(秒) */
#define LOGIC_SELF_TEST_CTRL_BYPASS  0U   /* 1=控制旁路(Level1), 0=真实MBus(Level2) */

/*--------------------------------------------------------------
 * 第一部分: 设备状态查询
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：MatchAlarmLevel
 * 功能说明：将设备报警状态值与规则要求的报警级别进行匹配。
 *
 * 状态getter返回值的含义：
 *   0 = 正常（无报警）
 *   1 = 一级报警 / 预报警 / 一般报警
 *   2 = 二级报警 / 火警
 *   8 = 故障
 *
 * 参数说明：state - 设备当前状态值（0/1/2/8等）。
 *           level - 规则要求的报警级别（AlarmLevel枚举）。
 * 返回值：   1=匹配, 0=不匹配
 *--------------------------------------------------------------*/
static uint8_t MatchAlarmLevel(uint8_t state, uint8_t level)
{
    switch (level)
    {
    case LV_ANY:
        /* 任意级别: 只要状态非0即可（仅需要非零判断即可） */
        return (state != 0) ? 1 : 0;

    case LV_HIGH:
        /* 二级报警: 状态2 = 火警 */
        return (state == 2) ? 1 : 0;

    case LV_LOW:
        /* 一级报警: 状态1 = 预报警 */
        return (state == 1) ? 1 : 0;

    case LV_FAULT:
        /* 故障: 状态8 = 设备故障 */
        return (state == 8) ? 1 : 0;

    default:
        /* 未识别级别: 按非零即报警处理 */
        return (state != 0) ? 1 : 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：IsDeviceOnline
 * 功能说明：判断某回路某设备当前是否在线。
 * 参数说明：loop_no - 探测回路号（1-3）。
 *           dev_no  - 设备号
 * 返回值：   1=在线, 0=离线（未配置）
 *--------------------------------------------------------------*/
static uint8_t IsDeviceOnline(uint8_t loop_no, uint8_t dev_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus回路探测器在线状态 */
        return (getPointTypeMixtureDetectOnlineState(dev_no) != 0) ? 1 : 0;

    case 2:
        /* 回路2: MBus回路设备在线标志 */
        return (MBusCtrl_GetOnline(dev_no) != 0) ? 1 : 0;

    case 3:
        /* 回路3: RS485探测器在线判断（无设备在线时返回0） */
        return (RS485Detect_IsOnline(dev_no) != 0) ? 1 : 0;

    default:
        /* 未知回路: 未配置在线，默认为离线 */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：GetLoopScanMax
 * 功能说明：获取指定回路的通道扫描最大设备数量。
 * 参数说明：loop_no - 探测回路号（1-3）。
 * 返回值：   该回路的设备最大数量
 *--------------------------------------------------------------*/
static uint8_t GetLoopScanMax(uint8_t loop_no)
{
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus回路探测器最大地址 */
        return MIXTURE_DEVICE_MAX_ADDR;

    case 2:
        /* 回路2: MBus回路设备最大地址（1-63） */
        return MBUS_CONTROL_MAX_DEVICES;

    case 3:
        /* 回路3: RS485探测器最大设备数（1-32） */
        return RS485_DETECT_MAX_DEVICES;

    default:
        /* 未知回路: 返回0，不执行扫描 */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：ReadSensorState
 * 功能说明：对指定回路设备调用相应getter，获取当前报警状态。
 *           不直接处理通配扫描全流程状态查询，全部通过getter实现
 *
 * 各回路说明:
 *   回路1: MBus回路探测器（只支持温度/烟雾 getter，返回0/1值）
 *   回路2: MBus回路设备（按 sensor_type 查询设备状态）
 *   回路3: RS485探测器（支持全7种传感器）
 *
 * 参数说明：loop_no - 探测回路号（1-3）。
 *           dev_no  - 设备号
 *           sensor_type - 传感器类型（SensorType枚举）。
 * 返回值：   报警状态值（0=正常, 1=报警, 2=火警, 8=故障）
 *--------------------------------------------------------------*/
static uint8_t ReadSensorState(uint8_t loop_no, uint8_t dev_no, uint8_t sensor_type)
{
    uint8_t i;           /* 循环计数变量 */
    uint8_t max_state;   /* 通配查询的最大状态值 */
    uint8_t sensor_idx;  /* RS485传感器索引 */
    uint8_t s;           /* 单传感器状态（临时变量） */

    /* 按回路分发处理 */
    switch (loop_no)
    {
    case 1:
        /* 回路1: MBus回路探测器
         * 只支持温度/烟雾两种传感器类型
         * getter返回0/1值（非报警值） */
        if (dev_no > MIXTURE_DEVICE_MAX_ADDR)
        {
            return 0;  /* 设备号超出范围 */
        }

        if (sensor_type == SENSOR_TEMPERATURE)
        {
            /* 温度报警状态查询 */
            return getPointTypeMixtureDetectTempertureMemory(dev_no);
        }
        else if (sensor_type == SENSOR_SMOKE)
        {
            /* 烟雾报警状态查询 */
            return getPointTypeMixtureDetectSmokeMemory(dev_no);
        }
        else if (sensor_type == SENSOR_ANY)
        {
            /* 通配查询: 温度 OR 烟雾 */
            if (getPointTypeMixtureDetectTempertureMemory(dev_no) != 0 ||
                getPointTypeMixtureDetectSmokeMemory(dev_no) != 0)
            {
                return 1;
            }
            return 0;
        }
        else
        {
            /* CH4/CO/H2/VOC/压力 等 回路1不支持，返回0 */
            return 0;
        }

    case 2:
        /* 回路2: MBus回路设备
         * 通过设备状态寄存器查询，配合 sensor_type
         * 返回设备状态值(0/1/2/8等) */
        if (dev_no > MBUS_CONTROL_MAX_DEVICES)
        {
            return 0;  /* 设备号超出范围 */
        }
        return MBusCtrl_GetDeviceState(dev_no);

    case 3:
        /* 回路3: RS485探测器
         * 支持全7种传感器类型 */
        if (dev_no > RS485_DETECT_MAX_DEVICES)
        {
            return 0;  /* 设备号超出范围 */
        }

        if (sensor_type == SENSOR_ANY)
        {
            /* 通配查询: 取所有传感器的最大状态值 */
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
            /* 未识别的传感器类型: 返回0（不报警） */
            return 0;
        }
        return RS485Detect_GetSensorState(dev_no, sensor_idx);

    default:
        /* 未知探测回路: 超出范围，返回0（不报警） */
        return 0;
    }
}

/*--------------------------------------------------------------
 * 函数名称：LogicDev_QueryCond
 * 功能说明：查询某条件是否满足。（条件使用5位数字: loop_no(探测回路) + dev_no(设备号)）
 *
 * 实现要点：
 * 1. 手动报警SENSOR_HAND_REPORT: 调用getHandPaperState()
 * 2. 电源SENSOR_POWER: 查询zhu_state/bei_state
 * 3. 通用传感器查询
 *    - 若dev_no == 0xFF通配扫描: 只对在线设备查询各设备报警状态
 *    - 若dev_no为具体值: 对回路设备查询报警状态，若匹配报警级别
 *    - 若报警LV_OFFLINE: 设备离线返回1
 *    - 回路1温度/烟雾/ANY的getter返回0/1值（非报警值）alarm_level统一用LV_ANY匹配
 *
 * 参数说明：cond - 条件指针（loop_no, dev_no, sensor_type, alarm_level）。
 * 返回值：   1=条件满足, 0=条件不满足
 *--------------------------------------------------------------*/
uint8_t LogicDev_QueryCond(const Cond_t *cond)
{
    uint8_t i;              /* 通配扫描循环变量 */
    uint8_t state;          /* 设备当前状态值 */
    uint8_t scan_max;       /* 通配扫描最大设备数 */
    uint8_t effective_level;/* 实际匹配用的报警级别 */

    /* 校验输入指针 */
    if (cond == NULL)
    {
        return 0;  /* 空指针：条件不满足 */
    }

#if (LOGIC_SELF_TEST_EN != 0U)
    /* D级测试钩子1: 回路1设备1/2按时间周期相位模拟(0~25s报警/25~60s正常),
     * 绕过真实传感器getter, 用于测试规则 01001&01002=0200101 */
    if ((cond->loop_no == 1U) && ((cond->dev_no == 1U) || (cond->dev_no == 2U)))
    {
        uint32_t phase_sec = (HAL_GetTick() / 1000U) % LOGIC_SELF_TEST_PERIOD;
        return (phase_sec < LOGIC_SELF_TEST_ON_S) ? 1U : 0U;
    }
#endif
    /*--- 按传感器类型的处理 ---*/

    /* 手动报警按钮: 调用getHandPaperState() */
    if (cond->sensor_type == SENSOR_HAND_REPORT)
    {
        /* getHandPaperState()返回0=未按下, 0xF0=已按下 */
        state = getHandPaperState();
        /* 若状态非0直接返回报警 */
        return (state != 0) ? 1 : 0;
    }

    /* 电源报警: 查询zhu_state/bei_state */
    if (cond->sensor_type == SENSOR_POWER)
    {
        /* dev_no 表示哪个电源（1或2:
         * 1 = 主电（zhu_state）
         * 2 = 备电（bei_state）
         * 0xFF 或 0 = 任何电源异常 */
        if (cond->dev_no == 1)
        {
            /* 主电: zhu_state 0=正常, 1=异常 */
            return (zhu_state != 0) ? 1 : 0;
        }
        else if (cond->dev_no == 2)
        {
            /* 备电: bei_state 0=正常, 非0=异常 */
            return (bei_state != 0) ? 1 : 0;
        }
        else
        {
            /* 任何电源异常: 取或 */
            return (zhu_state != 0 || bei_state != 0) ? 1 : 0;
        }
    }

    /*--- 通用探测器（非手动/电源）---*/

    /* 获取该回路扫描上限，同时校验回路有效性 */
    scan_max = GetLoopScanMax(cond->loop_no);
    if (scan_max == 0)
    {
        return 0;  /* 未知回路: 不执行扫描 */
    }

    /* 通过通配扫描该回路的所有探测器 */
    if (cond->dev_no == LOGIC_WILDCARD)
    {
        /* 若查LV_OFFLINE: 检查是否存在任何离线探测器 */
        if (cond->alarm_level == LV_OFFLINE)
        {
            for (i = 0; i < scan_max; i++)
            {
                if (IsDeviceOnline(cond->loop_no, i) == 0)
                {
                    return 1;  /* 找到了离线探测器 */
                }
            }
            return 0;  /* 所有探测器都在线 */
        }

        /* 报警扫描: 遍历扫描所有在线探测器 */
        for (i = 0; i < scan_max; i++)
        {
            /* 跳过离线探测器: 只查询在线设备的状态 */
            if (IsDeviceOnline(cond->loop_no, i) == 0)
            {
                continue;  /* 设备离线，跳过 */
            }

            /* 获取探测器当前状态 */
            state = ReadSensorState(cond->loop_no, i, cond->sensor_type);

            /* 回路1温度/烟雾/ANY: 只要非0即报警，统一LV_ANY匹配 */
            if (cond->loop_no == 1 &&
                (cond->sensor_type == SENSOR_TEMPERATURE ||
                 cond->sensor_type == SENSOR_SMOKE ||
                 cond->sensor_type == SENSOR_ANY))
            {
                if (state != 0)
                {
                    return 1;  /* 回路1报警即视为满足 */
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

    /* 若查LV_OFFLINE: 判断指定探测器是否离线 */
    if (cond->alarm_level == LV_OFFLINE)
    {
        return (IsDeviceOnline(cond->loop_no, cond->dev_no) == 0) ? 1 : 0;
    }

    /* 若回路指定探测器是否在线 */
    if (IsDeviceOnline(cond->loop_no, cond->dev_no) == 0)
    {
        return 0;  /* 设备离线，不能参与判断 */
    }

    /* 获取探测器当前状态 */
    state = ReadSensorState(cond->loop_no, cond->dev_no, cond->sensor_type);

    /* 回路1温度/烟雾/ANY的getter返回0/1值（非报警值），需特殊处理
     * alarm_level 统一用 LV_ANY 匹配 */
    effective_level = cond->alarm_level;
    if (cond->loop_no == 1 &&
        (cond->sensor_type == SENSOR_TEMPERATURE ||
         cond->sensor_type == SENSOR_SMOKE ||
         cond->sensor_type == SENSOR_ANY))
    {
        effective_level = LV_ANY;
    }

    /* 最后根据报警状态匹配级别 */
    return MatchAlarmLevel(state, effective_level);
}

/*--------------------------------------------------------------
 * 第二部分: 设备控制
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_Control
 * 功能说明：对控制回路+设备号+通道号执行设备启动/停止控制动作。
 *           7位数字: loop_no(控制回路) + dev_no(设备号) + channel(通道号)
 *
 * 控制回路(只支持控制回路2, 回路1原控制继电器逻辑已删除):
 *   loop_no=2, dev_no=设备地址, channel=1-4/99:
 *     通过MBusCtrl_Request异步控制输出设备, 不直接占用UART2。
 *     channel=1-4: 为通道位掩码(DEVICE_OUTPUT_1~4)控制;
 *     channel=99:  全部通道(当前映射为DEVICE_OUTPUT_1)。
 *   loop_no=1/3: 不支持控制, 直接失败返回
 *
 * 参数说明：loop_no - 控制回路号（只支持2）。
 *           dev_no  - 设备号（回路2设备地址1-63）。
 *           channel - 控制通道号(1-4=指定通道, 99=全部通道)
 *           action  - 1=启动/报警, 0=停止/关闭
 * 返回值：   0=控制受理成功, 1=控制失败（设备未知/不支持/参数非法）
 *--------------------------------------------------------------*/
uint8_t LogicDev_Control(uint8_t loop_no, uint8_t dev_no, uint8_t channel, uint8_t action)
{
    MBusCtrlRequest request;
    uint32_t mask;

    /* 控制动作只支持控制回路2(回路1原控制继电器逻辑已删除, 回路3无控制能力) */
    if (loop_no != 2U)
    {
        DebugPrintf("[LOGIC-CTRL] loop %d not controllable\r\n", loop_no);
        return 1U;
    }

    /* 通道号转换为位掩码: 1-4=指定通道, 99=全部通道
     * 说明: 当前全部回路唯一支持输出控制的设备为声光报警器(XR-SGBJQ),
     * 只支持 DEVICE_OUTPUT_1 通道, 故"全部通道"映射为 DEVICE_OUTPUT_1。
     * 未出现多通道输出设备时, 此处需再扩展为该设备支持的全部通道位 */
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
#if (LOGIC_SELF_TEST_CTRL_BYPASS != 0U)
        /* D级测试钩子2: 绕过MBus真实控制, 直接模拟控制成功(ret=0),
         * 用于Level1无硬件自动化测试(配合条件模拟+C1联动打点) */
        DebugPrintf("[LOGIC-CTRL] SELF-TEST-BYPASS loop2 dev=%d ch=%d mask=0x%X %s -> OK\r\n",
                    dev_no, channel, (unsigned)mask, action ? "START" : "STOP");
        return 0U;
#else
        MBusCtrlResult result = MBusCtrl_Request(&request);
        DebugPrintf("[LOGIC-CTRL] loop2 dev=%d ch=%d mask=0x%X %s ret=%d\r\n",
                    dev_no, channel, (unsigned)mask, action ? "START" : "STOP", (int)result);
        /* LogicDev_Control原约定: 0=控制受理成功, 1=控制失败 */
        return (result == MBUS_CTRL_ACCEPTED) ? 0U : 1U;
#endif /* LOGIC_SELF_TEST_CTRL_BYPASS */
    }
}

/*--------------------------------------------------------------
 * 第三部分: 自动模式判断
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_CheckAutoMode
 * 功能说明：根据执行模式和系统/分区的手/自动状态判断是否允许执行。
 *
 * 参数说明：exec_mode - 执行模式（ExecMode枚举）。
 * 返回值：   1=允许执行动作, 0=禁止（当前为手动模式或禁止状态）
 *--------------------------------------------------------------*/
uint8_t LogicDev_CheckAutoMode(uint8_t exec_mode)
{
    uint8_t sys_auto;    /* 系统级自动模式状态 */
    uint8_t part1_auto;  /* 分区1自动模式状态 */
    uint8_t part2_auto;  /* 分区2自动模式状态 */

    /* 获取系统级及各分区的 手/自动 状态 */
    /* KEY_AUTO = 0x02, KEY_MANUAL = 0x01 */
    sys_auto   = (getSysHandAutoState()   == KEY_AUTO) ? 1 : 0;
    part1_auto = (getPart1HandAutoState() == KEY_AUTO) ? 1 : 0;
    part2_auto = (getPart2HandAutoState() == KEY_AUTO) ? 1 : 0;

    /* 根据执行模式判断是否可以 */
    switch (exec_mode)
    {
    case EXEC_START_ALL:
    case EXEC_STOP_ALL:
        /* 要求三个分区都处于自动模式才能执行 */
        return (sys_auto && part1_auto && part2_auto) ? 1 : 0;

    case EXEC_START_PART:
    case EXEC_STOP_PART:
        /* 任意一个分区处于自动模式即可执行 */
        return (sys_auto || part1_auto || part2_auto) ? 1 : 0;

    default:
        /* 未识别的模式类型，禁止执行 */
        return 0;  /* 未知模式，默认全部禁止执行 */
    }
}

/*--------------------------------------------------------------
 * 第四部分: 初始化注册
 *--------------------------------------------------------------*/

/*--------------------------------------------------------------
 * 函数名称：LogicDev_Register
 * 功能说明：将本模块的回调接口注入到引擎中，完成功能注册。
 *           该函数需在系统初始化阶段调用一次，完成回调注册。
 *
 * 调用顺序（main函数中调用）：
 *   1. LogicExpr_Init();      装载规则（Flash默认配置）
 *   2. LogicDev_Register();   注入回调函数 - 立即生效）
 *   3. LogicEngine_Init();    引擎初始化（加载时间状态）
 *   4. [RTOS环境启动]          引擎启动（开始执行循环）
 *--------------------------------------------------------------*/
/* A8-1: 联动动作打点实现 - 转存到黑匣子存储(异步调用, 不阻塞引擎100ms轮询) */
static void LinkageEventNotify(uint8_t loop_no, uint16_t dev_no,
                               uint8_t channel, uint8_t action)
{
    (void)loop_no;  /* 联动控制固定为回路2, 黑匣子记录仅回路号固定, 故不传递 */
    StorageEvent_LogLinkageAction((uint8_t)dev_no, channel, action);
}

/* A9: 手动联动启动查询回调 - 查询"外联设备启动"按键状态(由cmd_process.c的LINKAGE_START_KEY
 * 按键处理/StartupLinkageDevice()设置). 手动模式下手动联动需要此回调返回非零才执行动作.
 * 注意: 该回调只读查询标志, 标志由风机链路(BspFanStartCrtlApp)消费并清
 * 复位, 因此只能轮询, 不能在此处清除标志 */
static uint8_t LinkageManualStartQuery(void)
{
    extern uint8_t linkage_start_key_press_flag;  /* cmd_process.c 中定义的按键标志 */
    return (linkage_start_key_press_flag == 1U) ? 1U : 0U;
}

void LogicDev_Register(void)
{
    /* 注册设备状态查询回调（条件表达式使用） */
    LogicExpr_SetQueryFunc(LogicDev_QueryCond);

    /* 注册设备控制回调（动作执行使用） */
    LogicEngine_SetControlFunc(LogicDev_Control);

    /* 注册自动模式查询回调（执行前判断） */
    LogicEngine_SetAutoModeFunc(LogicDev_CheckAutoMode);

    /* 注册联动动作打点回调（黑匣子记录） */
    LogicEngine_SetEventFunc(LinkageEventNotify);

    /* 注册手动启动查询回调（手动模式执行判断） */
    LogicEngine_SetManualStartFunc(LinkageManualStartQuery);
}
