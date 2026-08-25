/* ============================================================================
 * 模块名称: MBus/回路2设备控制模块 (MBus Control Module)
 * 功能描述: 实现回路2(UART2) MBus总线设备的轮询调度、Modbus RTU通信、
 *          状态管理、声光报警器控制、火灾显示盘事件上报、Flash持久化。
 * 通信协议: Modbus RTU, 功能码04(读输入寄存器)/05(写单线圈)/10(写多寄存器),
 *          UART2/115200/8N1, MBUS2SITE=1
 * 设备类型: 声光报警器(XR-SGBJQ,地址60)/手动报警器(XR2200,地址61)/火灾显示盘(XR1530,地址62)
 * 轮询流程: 每10个任务周期(200ms)轮询一个在线设备, 发送04功能码读1个寄存器
 * 控制服务: 声光报警器通过05功能码控制, 火灾显示盘通过10功能码上报事件
 * 掉线检测: 发送失败(队列满)或超时无响应均计掉线, 连续10次判定掉线
 * 回路标识: 回路2, Flash存储地址0x110000, 故障簇ID=0x52(82簇)
 * ============================================================================ */

#include "bsp_mbus_control.h"
#include "bsp_device_registry.h"
#include "bsp_mbus.h"          
#include "bsp_itcallback.h"     
#include "system.h"             
#include "w25qxx.h"             
#include "cmsis_os.h"           

/* ============================================================
 * 内部常量与数据结构
 * ============================================================ */

static MBusCtrlDevice g_mbus_ctrl_devices[MBUS_CONTROL_MAX_DEVICES]; /* 设备实例数组(索引=地址) */
static uint8_t g_mbus_ctrl_polling_addr = 1; /* 当前轮询地址(1~63循环) */

#define MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN       16U  /* 火灾显示盘事件队列最大长度 */
#define MBUS_FIRE_DISPLAY_RESPONSE_WAIT_TICKS   15U  /* 火灾显示盘响应等待超时(tick) */
#define MBUS_FIRE_DISPLAY_MAX_RETRY             3U   /* 火灾显示盘最大重试次数 */
#define MBUS_SOUND_LIGHT_RESPONSE_WAIT_TICKS    15U  /* 声光报警器响应等待超时(tick) */
#define MBUS_SOUND_LIGHT_MAX_RETRY              3U   /* 声光报警器最大重试次数 */
#define MBUS_CONTROL_REQUEST_QUEUE_LEN           8U   /* 通用控制请求静态队列长度 */

/* 火灾显示盘事件定义(环形队列元素) */
typedef struct
{
    uint8_t loop;          /* 回路编号 */
    uint8_t addr;          /* 探测器地址 */
    uint8_t detector_type; /* 探测器类型(MBUS_FIRE_DISPLAY_DETECT_*) */
    uint8_t alarm_type;    /* 报警类型(MBUS_FIRE_DISPLAY_ALARM_*) */
} MBusFireDisplayEvent;

/* 火灾显示盘事件环形队列 */
static MBusFireDisplayEvent g_fire_display_event_queue[MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN];
static uint8_t g_fire_display_event_head = 0;    /* 队列头指针(出队位置) */
static uint8_t g_fire_display_event_tail = 0;    /* 队列尾指针(入队位置) */
static uint8_t g_fire_display_event_count = 0;   /* 当前队列中的事件数 */
static uint8_t g_fire_display_wait_response = 0; /* 是否等待显示盘响应 */
static uint8_t g_fire_display_wait_ticks = 0;    /* 等待响应已计tick数 */
static uint8_t g_fire_display_retry_count = 0;   /* 当前事件重试计数 */

/* 通用控制队列元素。final_outputs是合并update_mask后的完整目标快照。 */
typedef struct
{
    MBusCtrlRequest request;
    uint32_t final_outputs;
    uint8_t driver_id;
} MBusQueuedControl;

/* 固定RAM环形队列，不使用动态内存，也不在调用者栈中保存待执行请求。 */
static MBusQueuedControl g_control_queue[MBUS_CONTROL_REQUEST_QUEUE_LEN];
static uint8_t g_control_queue_head = 0U;
static uint8_t g_control_queue_tail = 0U;
static uint8_t g_control_queue_count = 0U;
static MBusQueuedControl g_active_control;
static uint8_t g_active_control_valid = 0U;
static uint8_t g_control_wait_response = 0U;
static uint8_t g_control_wait_ticks = 0U;
static uint8_t g_control_retry_count = 0U;

/* 每个物理地址独立保存目标值、确认值和异步结果，便于后续扩展更多输出设备。 */
static uint32_t g_control_target_outputs[MBUS_CONTROL_MAX_DEVICES];
static uint32_t g_control_confirmed_outputs[MBUS_CONTROL_MAX_DEVICES];
static uint8_t g_control_confirmed_valid[MBUS_CONTROL_MAX_DEVICES];
static uint8_t g_control_pending_count[MBUS_CONTROL_MAX_DEVICES];
static uint8_t g_control_status[MBUS_CONTROL_MAX_DEVICES];

/* 固定地址设备类型映射表(索引=地址, 值=MBusCtrlDevType) */
/* 设备中文名称映射表(索引=MBusCtrlDevType) */
#define MBUS2_IDENTIFY_FAIL_THRESHOLD 3U
#define MBUS2_IDENTIFY_RETRY_MS 1000U
#define MBUS2_STAGE_NATIONAL 0U
#define MBUS2_STAGE_PRODUCT  1U
#define MBUS2_STAGE_COMPLETE 2U

static uint8_t MBusCtrl_MapProductType(uint16_t product_code)
{
    uint8_t parser;
    if(DeviceRegistry_IsSupportedOnLoop(product_code, DEVICE_REGISTRY_LOOP2) == 0U) return MBUS_CONTROL_DEV_UNKNOWN;
    parser = DeviceRegistry_GetParserType(product_code);
    if(parser == DEVICE_PARSER_SGBJQ) return MBUS_CONTROL_DEV_SGBJQ;
    if(parser == DEVICE_PARSER_XR2200) return MBUS_CONTROL_DEV_XR2200;
    if(parser == DEVICE_PARSER_XR1503) return MBUS_CONTROL_DEV_FIRE_DISPLAY;
    if(parser == DEVICE_PARSER_GCM1002) return MBUS_CONTROL_DEV_GCM1002;
    if(parser == DEVICE_PARSER_FIM1017) return MBUS_CONTROL_DEV_FIM1017;
    return MBUS_CONTROL_DEV_UNKNOWN;
}

static uint8_t MBusCtrl_FindAddressByType(uint8_t dev_type)
{
    uint8_t addr;
    for(addr = 1U; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
        if(g_mbus_ctrl_devices[addr].online != 0U && g_mbus_ctrl_devices[addr].type_confirmed != 0U &&
           g_mbus_ctrl_devices[addr].dev_type == dev_type) return addr;
    return 0U;
}

static void MBusCtrl_MarkIdentifyFailure(uint8_t addr, DeviceIdentifyError error)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return;
    if(error == DEVICE_IDENTIFY_NATIONAL_UNKNOWN || error == DEVICE_IDENTIFY_CODE_MISMATCH)
        g_mbus_ctrl_devices[addr].identify_stage = MBUS2_STAGE_NATIONAL;
    if(g_mbus_ctrl_devices[addr].identify_fail_count < MBUS2_IDENTIFY_FAIL_THRESHOLD)
        g_mbus_ctrl_devices[addr].identify_fail_count++;
    g_mbus_ctrl_devices[addr].last_identify_tick = osKernelGetTickCount();
    if(g_mbus_ctrl_devices[addr].identify_fail_count >= MBUS2_IDENTIFY_FAIL_THRESHOLD)
        DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP2, addr, error);
}
/* ============================================================
 * 火灾显示盘事件处理: 10功能码写多寄存器, 环形队列+重试机制
 * ============================================================ */

/* 发送一条火灾显示盘事件帧: 10功能码, 写2个寄存器共8字节(回路/地址/探测器类型/报警类型) */
static void MBusCtrl_SendFireDisplayEvent(void)
{
    uint8_t frame[17] = {0};
    uint16_t crc16;
    MBusFireDisplayEvent *event = &g_fire_display_event_queue[g_fire_display_event_head];

    frame[0] = MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_FIRE_DISPLAY);
    if(frame[0] == 0U) return;
    frame[1] = 0x10;
    frame[3] = 0x02;
    frame[5] = 0x04;
    frame[6] = 0x08;
    frame[8] = event->loop;
    frame[10] = event->addr;
    frame[12] = event->detector_type;
    frame[14] = event->alarm_type;
    crc16 = CalcCrc16(frame, 15);
    frame[15] = crc16 & 0xFF;
    frame[16] = crc16 >> 8;
    MBus2SendString(frame, sizeof(frame));
    g_fire_display_wait_response = 1;
    g_fire_display_wait_ticks = 0;
}

/* 火灾显示盘事件服务: 检查队列→发送事件→等待响应→超时重试→放弃 */
static uint8_t MBusCtrl_ServiceFireDisplayEvents(void)
{
    if (g_fire_display_event_count == 0)
        return 0;
    if (MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_FIRE_DISPLAY) == 0U)
        return 0;
    if (g_fire_display_wait_response == 0)
    {
        MBusCtrl_SendFireDisplayEvent();
    }
    else if (++g_fire_display_wait_ticks >= MBUS_FIRE_DISPLAY_RESPONSE_WAIT_TICKS)
    {
        if (g_fire_display_retry_count++ < MBUS_FIRE_DISPLAY_MAX_RETRY)
            MBusCtrl_SendFireDisplayEvent();
        else
        {
            /* XR5000_FIRE_DISPLAY_GENERIC_NAMES_20260729: do not block MBus2 after a display write failure. */
            g_fire_display_event_head = (g_fire_display_event_head + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
            g_fire_display_event_count--;
            g_fire_display_wait_response = 0;
            g_fire_display_retry_count = 0;
        }
    }
    return 1;
}

/* ============================================================
 * 通用异步控制: 单一外部入口、静态队列、产品驱动分派
 * ============================================================ */

/* 完成当前事务并更新该地址的最终状态；若同地址仍有排队请求则继续显示等待。 */
static void MBusCtrl_FinishActiveControl(MBusCtrlStatus result)
{
    uint8_t addr = g_active_control.request.addr;
    if(addr > 0U && addr < MBUS_CONTROL_MAX_DEVICES)
    {
        if(g_control_pending_count[addr] > 0U) g_control_pending_count[addr]--;
        g_control_status[addr] = g_control_pending_count[addr] > 0U ?
                                 MBUS_CTRL_STATUS_PENDING : (uint8_t)result;
    }
    g_active_control_valid = 0U;
    g_control_wait_response = 0U;
    g_control_wait_ticks = 0U;
    g_control_retry_count = 0U;
}

/* 请求执行前再次检查设备，防止排队期间设备被下线或重新识别。 */
static uint8_t MBusCtrl_CanExecute(const MBusQueuedControl *control)
{
    uint8_t addr = control->request.addr;
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return 0U;
    if(g_mbus_ctrl_devices[addr].online == 0U || g_mbus_ctrl_devices[addr].type_confirmed == 0U) return 0U;
    if(g_mbus_ctrl_devices[addr].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD) return 0U;
    return DeviceRegistry_GetControlDriver(g_mbus_ctrl_devices[addr].product_code) == control->driver_id ? 1U : 0U;
}

/* 当前只登记XR-SGBJQ驱动：05功能码写线圈0，输出1表示声光整体状态。 */
static uint8_t MBusCtrl_BuildControlFrame(const MBusQueuedControl *control, uint8_t *frame, uint8_t *length)
{
    uint16_t crc16;
    if(control == 0 || frame == 0 || length == 0) return 0U;
    if(control->driver_id != DEVICE_CONTROL_DRIVER_SGBJQ ||
       control->request.operation != MBUS_OPERATION_SET_OUTPUT) return 0U;

    frame[0] = control->request.addr;
    frame[1] = 0x05U;
    frame[2] = 0x00U;
    frame[3] = 0x00U;
    frame[4] = (control->final_outputs & MBUS_OUTPUT_SOUND_LIGHT) != 0U ? 0xFFU : 0x00U;
    frame[5] = 0x00U;
    crc16 = CalcCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc16 & 0xFFU);
    frame[7] = (uint8_t)(crc16 >> 8);
    *length = 8U;
    return 1U;
}

/* 发送当前控制事务；所有UART2控制报文只能由回路2任务调用这里发送。 */
static uint8_t MBusCtrl_SendActiveControl(void)
{
    uint8_t frame[8] = {0};
    uint8_t length = 0U;
    if(MBusCtrl_BuildControlFrame(&g_active_control, frame, &length) == 0U) return 0U;
    MBus2SendString(frame, length);
    g_control_wait_response = 1U;
    g_control_wait_ticks = 0U;
    g_control_status[g_active_control.request.addr] = MBUS_CTRL_STATUS_SENDING;
    return 1U;
}

/* 取出一条请求并执行；控制事务期间暂停普通轮询，超时后自动释放总线。 */
static uint8_t MBusCtrl_ServiceControlRequest(void)
{
    if(g_active_control_valid == 0U)
    {
        taskENTER_CRITICAL();
        if(g_control_queue_count == 0U)
        {
            taskEXIT_CRITICAL();
            return 0U;
        }
        g_active_control = g_control_queue[g_control_queue_head];
        g_control_queue_head = (uint8_t)((g_control_queue_head + 1U) % MBUS_CONTROL_REQUEST_QUEUE_LEN);
        g_control_queue_count--;
        taskEXIT_CRITICAL();
        g_active_control_valid = 1U;

        if(MBusCtrl_CanExecute(&g_active_control) == 0U || MBusCtrl_SendActiveControl() == 0U)
        {
            MBusCtrl_FinishActiveControl(MBUS_CTRL_STATUS_RESPONSE_ERROR);
            return 0U;
        }
    }
    else if(g_control_wait_response != 0U && ++g_control_wait_ticks >= MBUS_SOUND_LIGHT_RESPONSE_WAIT_TICKS)
    {
        if(g_control_retry_count++ < MBUS_SOUND_LIGHT_MAX_RETRY)
        {
            if(MBusCtrl_SendActiveControl() == 0U)
                MBusCtrl_FinishActiveControl(MBUS_CTRL_STATUS_RESPONSE_ERROR);
        }
        else
        {
            MBusCtrl_FinishActiveControl(MBUS_CTRL_STATUS_TIMEOUT);
        }
    }
    return g_active_control_valid != 0U ? 1U : 0U;
}

/* 唯一正式外部控制入口：校验能力并入队，不在调用任务中直接访问UART2。 */
MBusCtrlResult MBusCtrl_Request(const MBusCtrlRequest *request)
{
    uint8_t addr;
    uint8_t driver_id;
    uint32_t capabilities;
    uint32_t supported_outputs;
    uint32_t final_outputs;

    if(request == 0) return MBUS_CTRL_INVALID_PARAMETER;
    addr = request->addr;
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return MBUS_CTRL_INVALID_ADDR;
    if(g_mbus_ctrl_devices[addr].online == 0U) return MBUS_CTRL_NOT_CONFIGURED;
    if(g_mbus_ctrl_devices[addr].type_confirmed == 0U) return MBUS_CTRL_UNIDENTIFIED;
    if(g_mbus_ctrl_devices[addr].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD) return MBUS_CTRL_DISCONNECTED;
    if(request->operation != MBUS_OPERATION_SET_OUTPUT) return MBUS_CTRL_UNSUPPORTED;

    capabilities = DeviceRegistry_GetCapabilities(g_mbus_ctrl_devices[addr].product_code);
    supported_outputs = DeviceRegistry_GetSupportedOutputs(g_mbus_ctrl_devices[addr].product_code);
    driver_id = DeviceRegistry_GetControlDriver(g_mbus_ctrl_devices[addr].product_code);
    if((capabilities & DEVICE_CAP_OUTPUT_CONTROL) == 0U || driver_id == DEVICE_CONTROL_DRIVER_NONE)
        return MBUS_CTRL_UNSUPPORTED;
    if(request->target_mask == 0U || (request->target_mask & ~supported_outputs) != 0U ||
       (request->target_value & ~request->target_mask) != 0U)
        return MBUS_CTRL_INVALID_PARAMETER;

    taskENTER_CRITICAL();
    final_outputs = (g_control_target_outputs[addr] & ~request->target_mask) |
                    (request->target_value & request->target_mask);
    /* 相同目标已经确认，或相同目标正在排队时，不重复占用队列。 */
    if(final_outputs == g_control_target_outputs[addr] &&
       ((g_control_pending_count[addr] > 0U) ||
        (g_control_confirmed_valid[addr] != 0U && g_control_confirmed_outputs[addr] == final_outputs)))
    {
        taskEXIT_CRITICAL();
        return MBUS_CTRL_ACCEPTED;
    }
    if(g_control_queue_count >= MBUS_CONTROL_REQUEST_QUEUE_LEN)
    {
        taskEXIT_CRITICAL();
        return MBUS_CTRL_QUEUE_FULL;
    }
    g_control_target_outputs[addr] = final_outputs;
    g_control_queue[g_control_queue_tail].request = *request;
    g_control_queue[g_control_queue_tail].final_outputs = final_outputs;
    g_control_queue[g_control_queue_tail].driver_id = driver_id;
    g_control_queue_tail = (uint8_t)((g_control_queue_tail + 1U) % MBUS_CONTROL_REQUEST_QUEUE_LEN);
    g_control_queue_count++;
    if(g_control_pending_count[addr] < 0xFFU) g_control_pending_count[addr]++;
    g_control_status[addr] = MBUS_CTRL_STATUS_PENDING;
    taskEXIT_CRITICAL();
    return MBUS_CTRL_ACCEPTED;
}

MBusCtrlStatus MBusCtrl_GetStatus(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return MBUS_CTRL_STATUS_IDLE;
    return (MBusCtrlStatus)g_control_status[addr];
}

/* 手报自动控制也复用通用入口，避免另建一套声光控制状态机。 */
static void MBusCtrl_UpdateManualSoundLightTarget(uint8_t manual_state)
{
    uint8_t sound_light_addr;
    MBusCtrlRequest request = {0};
    if(manual_state != 1U && manual_state != 2U) return;
    sound_light_addr = MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_SGBJQ);
    if(sound_light_addr == 0U) return;
    request.addr = sound_light_addr;
    request.operation = MBUS_OPERATION_SET_OUTPUT;
    request.target_mask = MBUS_OUTPUT_SOUND_LIGHT;
    request.target_value = manual_state == 2U ? MBUS_OUTPUT_SOUND_LIGHT : 0U;
    (void)MBusCtrl_Request(&request);
}

/* ============================================================
 * 初始化: 清零设备表, 从Flash恢复在线状态
 * ============================================================ */

void MBusCtrl_Init(void)
{
    /* 通用控制状态全部使用静态RAM，上电初始化时统一清零。 */
    g_control_queue_head = 0U;
    g_control_queue_tail = 0U;
    g_control_queue_count = 0U;
    g_active_control_valid = 0U;
    g_control_wait_response = 0U;
    g_control_wait_ticks = 0U;
    g_control_retry_count = 0U;
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        g_mbus_ctrl_devices[i].online = 0;
        g_mbus_ctrl_devices[i].disconnect_count = 0;
        g_mbus_ctrl_devices[i].dev_type = MBUS_CONTROL_DEV_UNKNOWN;
        g_mbus_ctrl_devices[i].sensor_state = 0;
        g_mbus_ctrl_devices[i].disconnect_memory = 0;
        g_mbus_ctrl_devices[i].product_code = 0U;
        g_mbus_ctrl_devices[i].national_type_code = 0U;
        g_mbus_ctrl_devices[i].type_confirmed = 0U;
        g_mbus_ctrl_devices[i].identify_stage = MBUS2_STAGE_NATIONAL;
        g_mbus_ctrl_devices[i].identify_fail_count = 0U;
        g_mbus_ctrl_devices[i].identify_request_pending = 0U;
        g_mbus_ctrl_devices[i].last_identify_tick = 0U;
        g_control_target_outputs[i] = 0U;
        g_control_confirmed_outputs[i] = 0U;
        g_control_confirmed_valid[i] = 0U;
        g_control_pending_count[i] = 0U;
        g_control_status[i] = MBUS_CTRL_STATUS_IDLE;
    }
    MBusCtrl_LoadOnlineState();
}

/* ============================================================
 * 在线状态管理: 屏幕下发设置/清除设备在线
 * ============================================================ */

/* 设置单个设备在线状态: 下线时同时清零掉线计数/传感器状态/掉线记忆 */
void MBusCtrl_SetOnline(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return;
    g_mbus_ctrl_devices[addr].online = state;
    if (state == 0)
    {
        g_mbus_ctrl_devices[addr].disconnect_count = 0;
        g_mbus_ctrl_devices[addr].sensor_state = 0;
        g_mbus_ctrl_devices[addr].disconnect_memory = 0;
        g_mbus_ctrl_devices[addr].product_code = 0U;
        g_mbus_ctrl_devices[addr].national_type_code = 0U;
        g_mbus_ctrl_devices[addr].dev_type = MBUS_CONTROL_DEV_UNKNOWN;
        g_mbus_ctrl_devices[addr].type_confirmed = 0U;
        g_mbus_ctrl_devices[addr].identify_stage = MBUS2_STAGE_NATIONAL;
        g_mbus_ctrl_devices[addr].identify_fail_count = 0U;
        g_mbus_ctrl_devices[addr].identify_request_pending = 0U;
        g_control_target_outputs[addr] = 0U;
        g_control_confirmed_outputs[addr] = 0U;
        g_control_confirmed_valid[addr] = 0U;
        g_control_pending_count[addr] = 0U;
        g_control_status[addr] = MBUS_CTRL_STATUS_IDLE;
        DeviceRegistry_SetProductUnknown(DEVICE_REGISTRY_LOOP2, addr, 0U);
    }
}

/* 批量设置在线状态(start~end地址范围) */
void MBusCtrl_SetOnlineRange(uint8_t start, uint8_t end, uint8_t state)
{
    for (uint8_t addr = start; addr <= end; addr++)
    {
        MBusCtrl_SetOnline(addr, state);
    }
}

/* ============================================================
 * 状态查询接口
 * ============================================================ */

/* 获取在线标志(屏幕下发值, 非实时在线状态) */
uint8_t MBusCtrl_GetOnline(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].online;
}

/* 判断是否掉线(连续无响应次数>=阈值) */
uint8_t MBusCtrl_IsDisconnected(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD ? 1 : 0;
}

/* 获取回路2在线设备总数 */
uint8_t MBusCtrl_GetOnlineCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online)
            count++;
    }
    return count;
}

uint8_t MBusCtrl_IsIdentified(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0U;
    return g_mbus_ctrl_devices[addr].type_confirmed != 0U ? 1U : 0U;
}

uint16_t MBusCtrl_GetNationalTypeCode(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return 0U;
    return g_mbus_ctrl_devices[addr].national_type_code;
}

uint16_t MBusCtrl_GetProductCode(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return 0U;
    return g_mbus_ctrl_devices[addr].product_code;
}

uint8_t MBusCtrl_GetActiveCount(void)
{
    uint8_t count = 0U;
    uint8_t addr;
    for(addr = 1U; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
    {
        if(g_mbus_ctrl_devices[addr].online != 0U &&
           g_mbus_ctrl_devices[addr].type_confirmed != 0U &&
           g_mbus_ctrl_devices[addr].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD)
        {
            count++;
        }
    }
    return count;
}

/* 统计回路2掉线设备数量(上线但掉线计数>=阈值) */
uint8_t MBusCtrl_GetDisconnectCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online &&
            g_mbus_ctrl_devices[i].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD)
            count++;
    }
    return count;
}

/* 统计回路2报警设备数(在线且未掉线且处于报警状态) */
uint8_t MBusCtrl_GetAlarmCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online &&
            g_mbus_ctrl_devices[i].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD &&
            MBusCtrl_IsAlarmState(i))
            count++;
    }
    return count;
}

/* 根据地址获取设备中文名称(XR-SGBJQ/XR2200/FireDisplay) */
const char* MBusCtrl_GetDeviceName(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES || g_mbus_ctrl_devices[addr].type_confirmed == 0U) return NULL;
    return DeviceRegistry_GetName(g_mbus_ctrl_devices[addr].product_code);
}

/* 获取设备传感器状态值(04功能码读取的寄存器值) */
uint8_t MBusCtrl_GetDeviceState(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].sensor_state;
}

/* 获取设备国标设备类型码(供联动逻辑显示用), 地址无效返回0 */
uint16_t MBusCtrl_GetNationalCode(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].national_type_code;
}

/* 获取设备类型(MBusCtrlDevType) */
uint8_t MBusCtrl_GetDeviceType(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return MBUS_CONTROL_DEV_UNKNOWN;
    return g_mbus_ctrl_devices[addr].dev_type;
}

/* 判断设备是否处于报警状态: SGBJQ(state=1)/XR2200(state=2)/FireDisplay(state=1) */
uint8_t MBusCtrl_IsAlarmState(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    uint8_t state = g_mbus_ctrl_devices[addr].sensor_state;
    switch (g_mbus_ctrl_devices[addr].dev_type)
    {
        case MBUS_CONTROL_DEV_SGBJQ:  return (state == 1);
        case MBUS_CONTROL_DEV_XR2200: return (state == 2);
        case MBUS_CONTROL_DEV_FIRE_DISPLAY: return (state == 1);
        default: return 0;
    }
}

/* ============================================================
 * Flash持久化: 在线状态表存储与加载
 * ============================================================ */

/* 将当前在线状态表保存到Flash(地址0x110000) */
void MBusCtrl_SaveOnlineState(void)
{
    uint8_t online_state[MBUS_CONTROL_MAX_DEVICES];
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        online_state[i] = g_mbus_ctrl_devices[i].online;
    }
    W25QXX_Write(online_state, MBUS_CONTROL_FLASH_ADDR, sizeof(online_state));
}

/* 从Flash加载在线状态表(0xFF视为未配置, 设为离线) */
void MBusCtrl_LoadOnlineState(void)
{
    uint8_t online_state[MBUS_CONTROL_MAX_DEVICES];
    W25QXX_Read(online_state, MBUS_CONTROL_FLASH_ADDR, sizeof(online_state));
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (online_state[i] == 0xFF)
        {
            g_mbus_ctrl_devices[i].online = 0;
        }
        else
        {
            g_mbus_ctrl_devices[i].online = online_state[i];
        }
    }
}

/* ============================================================
 * 轮询管理: 04功能码读取设备状态寄存器
 * ============================================================ */

/* 轮询下一个在线设备: 构建04功能码帧→发送到MBus2队列→发送失败则计掉线 */
static void MBusControlPollingManage(void)
{
    uint8_t modbus_buff[8]; uint16_t crc16; uint8_t found = 0U; uint32_t now = osKernelGetTickCount();
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        g_mbus_ctrl_polling_addr++;
        if (g_mbus_ctrl_polling_addr >= MBUS_CONTROL_MAX_DEVICES) g_mbus_ctrl_polling_addr = 1U;
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].online != 0U &&
           (DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP2, g_mbus_ctrl_polling_addr) == 0U ||
            (now - g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].last_identify_tick) >= MBUS2_IDENTIFY_RETRY_MS))
        { found = 1U; break; }
    }
    if(found == 0U) return;
    if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].type_confirmed == 0U)
    {
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending != 0U)
        {
            MBusCtrl_MarkIdentifyFailure(g_mbus_ctrl_polling_addr,
                g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_stage == MBUS2_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
            if(DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP2, g_mbus_ctrl_polling_addr) != 0U)
            {
                g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 0U;
                return;
            }
        }
        modbus_buff[2]=0U;
        modbus_buff[3]=g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_stage == MBUS2_STAGE_NATIONAL ? 0x0DU : 0x0EU;
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 1U;
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].last_identify_tick = now;
    }
    else
    {
        modbus_buff[2]=0U; modbus_buff[3]=0U;
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD)
            g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].disconnect_count++;
    }
    modbus_buff[0]=g_mbus_ctrl_polling_addr; modbus_buff[1]=0x04U; modbus_buff[4]=0U; modbus_buff[5]=1U;
    crc16=CalcCrc16(modbus_buff,6U); modbus_buff[6]=crc16 & 0xFFU; modbus_buff[7]=crc16 >> 8;
    if(SendDataToMBus2Queue(modbus_buff,8U) != 1 && g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].type_confirmed == 0U)
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 0U;
}

/* ============================================================
 * 接收数据处理: 校验CRC→解析04/05/10/06/03功能码响应
 * ============================================================ */

/* 处理MBus2从设备响应数据: 04功能码更新传感器状态, 05功能码确认声光控制, 10功能码确认显示盘事件 */
static void MBus2ReceiveSlaveDataDeal(void)
{
    uint16_t crc16 = 0x0000;
    if (uartbuff[MBUS2SITE].recepetion_flag == 1)
    {
        uartbuff[MBUS2SITE].recepetion_flag = 0;
        if (uartbuff[MBUS2SITE].recepetion_len < 2)
        {
            return;
        }

        crc16 = (uartbuff[MBUS2SITE].recepetion_buff[uartbuff[MBUS2SITE].recepetion_len - 1] << 8) |
                uartbuff[MBUS2SITE].recepetion_buff[uartbuff[MBUS2SITE].recepetion_len - 2];
        if (CalcCrc16(uartbuff[MBUS2SITE].recepetion_buff, uartbuff[MBUS2SITE].recepetion_len - 2) == crc16)
        {
            uint8_t dev_addr = uartbuff[MBUS2SITE].recepetion_buff[0];

            if (dev_addr == 0 || dev_addr >= MBUS_CONTROL_MAX_DEVICES)
                return;

            if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x04 && uartbuff[MBUS2SITE].recepetion_len == 7U && uartbuff[MBUS2SITE].recepetion_buff[2] == 2U)
            {
                uint16_t data = (uartbuff[MBUS2SITE].recepetion_buff[3] << 8) | uartbuff[MBUS2SITE].recepetion_buff[4];
                if(g_mbus_ctrl_devices[dev_addr].identify_request_pending != 0U)
                {
                    g_mbus_ctrl_devices[dev_addr].identify_request_pending = 0U;
                    if(g_mbus_ctrl_devices[dev_addr].identify_stage == MBUS2_STAGE_NATIONAL)
                    {
                        g_mbus_ctrl_devices[dev_addr].national_type_code = data;
                        g_mbus_ctrl_devices[dev_addr].identify_stage = MBUS2_STAGE_PRODUCT;
                        g_mbus_ctrl_devices[dev_addr].identify_fail_count = 0U;
                    }
                    else
                    {
                        uint8_t type = MBusCtrl_MapProductType(data);
                        if(type == MBUS_CONTROL_DEV_UNKNOWN)
                            MBusCtrl_MarkIdentifyFailure(dev_addr, DEVICE_IDENTIFY_PRODUCT_UNKNOWN);
                        else if(DeviceRegistry_IsNationalProductMatch(g_mbus_ctrl_devices[dev_addr].national_type_code, data) == 0U)
                            MBusCtrl_MarkIdentifyFailure(dev_addr, DeviceRegistry_IsNationalTypeKnown(g_mbus_ctrl_devices[dev_addr].national_type_code) != 0U ? DEVICE_IDENTIFY_CODE_MISMATCH : DEVICE_IDENTIFY_NATIONAL_UNKNOWN);
                        else
                        {
                            g_mbus_ctrl_devices[dev_addr].product_code = data;
                            g_mbus_ctrl_devices[dev_addr].dev_type = type;
                            g_mbus_ctrl_devices[dev_addr].type_confirmed = 1U;
                            g_mbus_ctrl_devices[dev_addr].identify_stage = MBUS2_STAGE_COMPLETE;
                            g_mbus_ctrl_devices[dev_addr].identify_fail_count = 0U;
                            g_mbus_ctrl_devices[dev_addr].disconnect_count = 0U;
                            DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP2, dev_addr, DEVICE_IDENTIFY_OK);
                        }
                    }
                }
                else
                {
                    g_mbus_ctrl_devices[dev_addr].disconnect_count = 0U;
                    g_mbus_ctrl_devices[dev_addr].sensor_state = (uint8_t)data;
                    if(g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_XR2200) MBusCtrl_UpdateManualSoundLightTarget((uint8_t)data);
                    else if(g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_SGBJQ && data <= 1U)
                    {
                        g_control_confirmed_valid[dev_addr] = 1U;
                        g_control_confirmed_outputs[dev_addr] = data != 0U ? MBUS_OUTPUT_SOUND_LIGHT : 0U;
                    }
                }
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x84U && g_mbus_ctrl_devices[dev_addr].identify_request_pending != 0U)
            {
                g_mbus_ctrl_devices[dev_addr].identify_request_pending = 0U;
                MBusCtrl_MarkIdentifyFailure(dev_addr,
                    g_mbus_ctrl_devices[dev_addr].identify_stage == MBUS2_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x05 &&
                     g_active_control_valid != 0U &&
                     g_control_wait_response != 0U &&
                     dev_addr == g_active_control.request.addr &&
                     g_active_control.driver_id == DEVICE_CONTROL_DRIVER_SGBJQ &&
                     uartbuff[MBUS2SITE].recepetion_len >= 8U &&
                     uartbuff[MBUS2SITE].recepetion_buff[2] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[3] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[4] == ((g_active_control.final_outputs & MBUS_OUTPUT_SOUND_LIGHT) != 0U ? 0xFFU : 0x00U) &&
                     uartbuff[MBUS2SITE].recepetion_buff[5] == 0x00U)
            {
                /* 仅接受当前地址、当前目标值完全一致的05回显，避免把轮询或其他设备应答误认成功。 */
                g_mbus_ctrl_devices[dev_addr].disconnect_count = 0U;
                g_control_confirmed_valid[dev_addr] = 1U;
                g_control_confirmed_outputs[dev_addr] = g_active_control.final_outputs;
                MBusCtrl_FinishActiveControl(MBUS_CTRL_STATUS_SUCCESS);
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x85U &&
                     g_active_control_valid != 0U && dev_addr == g_active_control.request.addr)
            {
                /* 当前05控制请求收到Modbus异常响应，立即结束事务并恢复普通轮询。 */
                MBusCtrl_FinishActiveControl(MBUS_CTRL_STATUS_RESPONSE_ERROR);
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x06)
            {
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x10 &&
                     g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_FIRE_DISPLAY &&
                     g_fire_display_wait_response != 0 &&
                     uartbuff[MBUS2SITE].recepetion_len >= 8)
            {
                g_mbus_ctrl_devices[dev_addr].disconnect_count = 0;
                g_fire_display_event_head = (g_fire_display_event_head + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
                g_fire_display_event_count--;
                g_fire_display_wait_response = 0;
                g_fire_display_wait_ticks = 0;
                g_fire_display_retry_count = 0;
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x03)
            {
            }
            crc16 = 0;
        }
    }
}

/* ============================================================
 * RTOS轮询任务: 主循环→接收处理→控制服务(声光/显示盘)→轮询调度, 间隔20ms
 * 轮询间隔: 每10个周期(200ms)发起一次设备轮询
 * 控制服务: 声光报警器优先于火灾显示盘, 有控制事务时暂停轮询
 * ============================================================ */

void MBusControlPollSlaveAndReceiveTask(void* parameter)
{
    uint8_t modbusbuf[8] = {0};
    uint8_t mbus_poll_delay_count = 0;

    for (;;)
    {
        uint8_t mbus2_control_busy = 0U;

        MBus2ReceiveSlaveDataDeal();
        /* 已开始的事务优先完成；所有通用控制均在本任务串行执行。 */
        if (g_active_control_valid != 0U)
            mbus2_control_busy = MBusCtrl_ServiceControlRequest();
        else if (g_fire_display_wait_response != 0U)
            mbus2_control_busy = MBusCtrl_ServiceFireDisplayEvents();
        else if (MBusCtrl_ServiceControlRequest())
            mbus2_control_busy = 1U;
        else if (MBusCtrl_ServiceFireDisplayEvents())
            mbus2_control_busy = 1U;

        if (mbus2_control_busy != 0U)
        {
            osDelay(20);
            continue;
        }

        mbus_poll_delay_count++;
        if (mbus_poll_delay_count == 10)
        {
            mbus_poll_delay_count = 0;
            MBusControlPollingManage();
        }

        if (ReceiveDataFromMBus2Queue(modbusbuf) == 1)
        {
            MBus2SendString(modbusbuf, sizeof(modbusbuf));
        }

        osDelay(20);
    }
}

/* ============================================================
 * FreeRTOS消息队列接口: MBus2轮询数据队列操作
 * ============================================================ */

/* 将Modbus帧发送到MBus2队列(非阻塞, 队列满返回0) */
int8_t SendDataToMBus2Queue(uint8_t *buf, uint8_t buf_len)
{
    if (xMBus2QueueHandle == NULL)
        return 0;
    if (xQueueSend(xMBus2QueueHandle, buf, 0) == pdFALSE)
        return 0;
    return 1;
}

/* 从MBus2队列接收数据(非阻塞, 队列空返回0) */
int8_t ReceiveDataFromMBus2Queue(uint8_t *buf)
{
    if (xMBus2QueueHandle == NULL)
        return 0;
    if (xQueueReceive(xMBus2QueueHandle, buf, 0) == pdFALSE)
        return 0;
    return 1;
}

/* ============================================================
 * 火灾显示盘事件发布: 入队事件到环形队列
 * ============================================================ */

/* 向火灾显示盘事件队列推送一条事件(回路/地址/探测器类型/报警类型) */
uint8_t MBusCtrl_PostFireDisplayEvent(uint8_t loop, uint8_t addr, uint8_t detector_type, uint8_t alarm_type)
{
    if (loop == 0 || addr == 0 || detector_type < MBUS_FIRE_DISPLAY_DETECT_TEMP ||
        detector_type > MBUS_FIRE_DISPLAY_DETECT_MANUAL || alarm_type < MBUS_FIRE_DISPLAY_ALARM_FIRE ||
        alarm_type > MBUS_FIRE_DISPLAY_ALARM_FAULT)
        return 0;
    taskENTER_CRITICAL();
    if (g_fire_display_event_count >= MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN)
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    g_fire_display_event_queue[g_fire_display_event_tail].loop = loop;
    g_fire_display_event_queue[g_fire_display_event_tail].addr = addr;
    g_fire_display_event_queue[g_fire_display_event_tail].detector_type = detector_type;
    g_fire_display_event_queue[g_fire_display_event_tail].alarm_type = alarm_type;
    g_fire_display_event_tail = (g_fire_display_event_tail + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
    g_fire_display_event_count++;
    taskEXIT_CRITICAL();
    return 1;
}


/* 测试注入: 设置MBus2控制设备传感器状态(bsp_test_inject调用) */
void MBusCtrl_InjectSensorState(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return;
    g_mbus_ctrl_devices[addr].online = 1;
    g_mbus_ctrl_devices[addr].disconnect_count = 0;
    g_mbus_ctrl_devices[addr].sensor_state = state;
}
