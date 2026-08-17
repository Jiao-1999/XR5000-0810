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

/* 声光报警器控制状态(独立于设备在线状态, 保留最后目标值) */
static uint8_t g_sound_light_target_valid = 0;   /* 目标值是否有效 */
static uint8_t g_sound_light_target_state = 0;   /* 目标状态(0=关闭, 1=开启) */
static uint8_t g_sound_light_confirmed_valid = 0;/* 确认值是否有效 */
static uint8_t g_sound_light_confirmed_state = 0;/* 已确认的当前状态 */
static uint8_t g_sound_light_request_pending = 0;/* 是否有待发送的控制请求 */
static uint8_t g_sound_light_wait_response = 0;  /* 是否等待声光报警器响应 */
static uint8_t g_sound_light_wait_ticks = 0;     /* 等待响应已计tick数 */
static uint8_t g_sound_light_retry_count = 0;    /* 当前控制重试计数 */

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
 * 声光报警器控制: 05功能码写单线圈, 由XR2200手动报警器状态触发
 * ============================================================ */

/* 根据XR2200手动报警器状态更新声光报警器目标值: state=2(报警)→开启, state=1(正常)→关闭 */
static void MBusCtrl_UpdateManualSoundLightTarget(uint8_t manual_state)
{
    uint8_t target_state;

    if (manual_state == 2U)
        target_state = 1U;
    else if (manual_state == 1U)
        target_state = 0U;
    else
        return;

    /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: only valid XR2200 states change the retained target. */
    g_sound_light_target_valid = 1U;
    g_sound_light_target_state = target_state;
    if (g_sound_light_confirmed_valid == 0U || g_sound_light_confirmed_state != target_state)
        g_sound_light_request_pending = 1U;
}

/* 发送声光报警器控制帧: 05功能码, 线圈值0xFF00(开启)或0x0000(关闭) */
static void MBusCtrl_SendSoundLightControl(void)
{
    uint8_t frame[8] = {0};
    uint16_t crc16;

    frame[0] = MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_SGBJQ);
    if(frame[0] == 0U) return;
    frame[1] = 0x05;
    frame[4] = g_sound_light_target_state ? 0xFF : 0x00;
    crc16 = CalcCrc16(frame, 6);
    frame[6] = crc16 & 0xFF;
    frame[7] = crc16 >> 8;
    MBus2SendString(frame, sizeof(frame));
    g_sound_light_wait_response = 1U;
    g_sound_light_wait_ticks = 0U;
}

/* 声光报警器控制服务: 检查请求→发送控制帧→等待响应→超时重试→放弃 */
static uint8_t MBusCtrl_ServiceSoundLightControl(void)
{
    if (g_sound_light_target_valid == 0U || g_sound_light_request_pending == 0U)
        return 0U;
    if (MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_SGBJQ) == 0U)
        return 0U;

    if (g_sound_light_wait_response == 0U)
    {
        MBusCtrl_SendSoundLightControl();
    }
    else if (++g_sound_light_wait_ticks >= MBUS_SOUND_LIGHT_RESPONSE_WAIT_TICKS)
    {
        if (g_sound_light_retry_count++ < MBUS_SOUND_LIGHT_MAX_RETRY)
        {
            MBusCtrl_SendSoundLightControl();
        }
        else
        {
            /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: abandon this cycle so polling cannot be blocked forever. */
            g_sound_light_request_pending = 0U;
            g_sound_light_wait_response = 0U;
            g_sound_light_wait_ticks = 0U;
            g_sound_light_retry_count = 0U;
        }
    }
    return 1U;
}

/* ============================================================
 * 初始化: 清零设备表, 从Flash恢复在线状态
 * ============================================================ */

void MBusCtrl_Init(void)
{
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
                    { g_sound_light_confirmed_valid = 1U; g_sound_light_confirmed_state = (uint8_t)data; }
                }
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x84U && g_mbus_ctrl_devices[dev_addr].identify_request_pending != 0U)
            {
                g_mbus_ctrl_devices[dev_addr].identify_request_pending = 0U;
                MBusCtrl_MarkIdentifyFailure(dev_addr,
                    g_mbus_ctrl_devices[dev_addr].identify_stage == MBUS2_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x05 &&
                     g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_SGBJQ &&
                     g_sound_light_wait_response != 0U &&
                     uartbuff[MBUS2SITE].recepetion_len >= 8U &&
                     uartbuff[MBUS2SITE].recepetion_buff[2] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[3] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[4] == (g_sound_light_target_state ? 0xFFU : 0x00U) &&
                     uartbuff[MBUS2SITE].recepetion_buff[5] == 0x00U)
            {
                /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: accept only the exact 0x05 coil echo. */
                g_mbus_ctrl_devices[dev_addr].disconnect_count = 0;
                g_sound_light_confirmed_valid = 1U;
                g_sound_light_confirmed_state = g_sound_light_target_state;
                g_sound_light_request_pending = 0U;
                g_sound_light_wait_response = 0U;
                g_sound_light_wait_ticks = 0U;
                g_sound_light_retry_count = 0U;
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
        /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: finish the active transaction before starting another. */
        if (g_sound_light_wait_response != 0U)
            mbus2_control_busy = MBusCtrl_ServiceSoundLightControl();
        else if (g_fire_display_wait_response != 0U)
            mbus2_control_busy = MBusCtrl_ServiceFireDisplayEvents();
        else if (MBusCtrl_ServiceSoundLightControl())
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

