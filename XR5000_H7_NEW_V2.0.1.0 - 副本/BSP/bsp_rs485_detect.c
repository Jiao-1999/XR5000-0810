/* ============================================================================
 * 模块名称: RS485探测器管理模块 (RS485 Detector Management)
 * 功能描述: 实现回路3(UART5) RS485总线探测器的轮询调度、Modbus RTU通信、
 *          传感器数据解析、设备状态管理、掉线/报警检测、Flash持久化。
 * 通信协议: Modbus RTU, 功能码04(读输入寄存器), UART5/9600/8N1
 * 轮询流程: 先探测设备类型(读0x000E/0x000F), 确认后读取传感器数据
 *           (V2.21: 从0x000C起读19个寄存器, 覆盖0x000C~0x001E)
 * 故障记录: 掉线/报警由本模块检测, 故障记录由cmd_process.c统一处理
 * 回路标识: 回路3, Flash存储地址0x10F000, 故障簇ID=0x53(83簇)
 * ============================================================================ */

#include "bsp_rs485_detect.h"
#include "bsp_device_registry.h"
#include "bsp_device_threshold.h"

#include "FreeRTOS.h"          
#include "cmsis_os.h"          
#include "queue.h"             

#include "bsp_itcallback.h"    
#include "usart.h"             
#include "bsp_save_ctrl.h"     
#include "system.h"            
#include "w25qxx.h"            

/* ============================================================
 * 第一层: 可配置映射表与传感器寄存器布局
 * ============================================================ */

/* 类型码映射表: 0x000E寄存器产品型号值 → 设备类型枚举 */
/* V2.21传感器寄存器布局: 常规轮询从0x000C起读, byte_offset按Modbus响应数据区偏移计算。 */
static const RS485SensorRegDef XR805_SENSOR_LAYOUT[] = {
    {0x000C, 3,  RS485_SENSOR_TEMPERATURE, 1},
    {0x0018, 27, RS485_SENSOR_TEMPERATURE, 0},
    {0x000D, 5,  RS485_SENSOR_SMOKE,       1},
    {0x0019, 29, RS485_SENSOR_SMOKE,       0},
    {0x000E, 7,  RS485_SENSOR_CO,          1},
    {0x001A, 31, RS485_SENSOR_CO,          0},
    {0x000F, 9,  RS485_SENSOR_H2,          1},
    {0x001B, 33, RS485_SENSOR_H2,          0},
    {0x0010, 11, RS485_SENSOR_VOC,         1},
    {0x001C, 35, RS485_SENSOR_VOC,         0},
    {0x0011, 13, RS485_SENSOR_CH4,         1},
    {0x001D, 37, RS485_SENSOR_CH4,         0},
};
#define XR805_SENSOR_COUNT  (sizeof(XR805_SENSOR_LAYOUT) / sizeof(XR805_SENSOR_LAYOUT[0]))

static const RS485SensorRegDef XR8303_SENSOR_LAYOUT[] = {
    {0x000C, 3,  RS485_SENSOR_TEMPERATURE, 1},
    {0x0018, 27, RS485_SENSOR_TEMPERATURE, 0},
    {0x000D, 5,  RS485_SENSOR_SMOKE,       1},
    {0x0019, 29, RS485_SENSOR_SMOKE,       0},
    {0x000E, 7,  RS485_SENSOR_CO,          1},
    {0x001A, 31, RS485_SENSOR_CO,          0},
    {0x000F, 9,  RS485_SENSOR_H2,          1},
    {0x001B, 33, RS485_SENSOR_H2,          0},
    {0x0010, 11, RS485_SENSOR_VOC,         1},
    {0x001C, 35, RS485_SENSOR_VOC,         0},
    {0x0012, 15, RS485_SENSOR_PRESSURE,    1},
    {0x001E, 39, RS485_SENSOR_PRESSURE,    0},
};
#define XR8303_SENSOR_COUNT  (sizeof(XR8303_SENSOR_LAYOUT) / sizeof(XR8303_SENSOR_LAYOUT[0]))

/* ============================================================
 * 第二层: 全局设备实例表与轮询状态
 * ============================================================ */

static RS485DetectDevice g_devices[RS485_DETECT_MAX_DEVICES]; /* 设备实例数组(索引=地址) */

static uint8_t g_online_count = 0;       /* 当前上线设备数量 */
static uint8_t g_poll_current_addr = 1;  /* 当前轮询地址(1~64循环) */
static uint32_t g_last_poll_time = 0;    /* 上次轮询的系统tick */

/* UART5事务锁: 同一时刻仅允许一个请求占用UART5 */
static uint8_t g_transaction_pending = 0;       /* 是否有进行中的事务 */
static uint8_t g_transaction_addr = 0;          /* 当前事务的目标地址 */
static uint8_t g_transaction_type_detect = 0; /* identify stage, zero for normal polling */
static uint8_t g_transaction_threshold = 0;   /* threshold 03/06 transaction, never counts as disconnect */
static uint8_t g_transaction_tx_active = 0U;
static uint8_t g_transaction_waiting_response = 0U;
static uint32_t g_transaction_tx_start_tick = 0U;
static uint32_t g_transaction_start_tick = 0;
static uint32_t g_last_transaction_end_tick = 0;
static uint8_t g_rx_frame[64];
static uint8_t g_rx_chunk[64];
static uint8_t g_transaction_request[8];
static uint16_t g_rx_frame_len = 0U;
static uint8_t g_identify_fail_count[RS485_DETECT_MAX_DEVICES];
static uint32_t g_last_identify_tick[RS485_DETECT_MAX_DEVICES];
#define RS485_IDENTIFY_FAIL_THRESHOLD 3U
#define RS485_IDENTIFY_RETRY_MS 1000U   /* 事务启动时的系统tick(用于超时判断) */
#define RS485_STAGE_NATIONAL 1U
#define RS485_STAGE_PRODUCT  2U
#define RS485_STAGE_SENSOR   3U
#define RS485_STAGE_COMPLETE 4U

static void release_transaction(void)
{
    g_transaction_pending = 0U;
    g_transaction_addr = 0U;
    g_transaction_type_detect = 0U;
    g_transaction_threshold = 0U;
    g_transaction_tx_active = 0U;
    g_transaction_waiting_response = 0U;
    g_transaction_tx_start_tick = 0U;
    g_transaction_start_tick = 0U;
    g_last_transaction_end_tick = osKernelGetTickCount();
    g_rx_frame_len = 0U;
}

static void begin_transaction(const uint8_t frame[8], uint8_t addr,
                              uint8_t type_detect, uint8_t threshold,
                              uint32_t start_tick)
{
    RS485DetectUartClearRx();
    g_rx_frame_len = 0U;
    memcpy(g_transaction_request, frame, sizeof(g_transaction_request));
    g_transaction_pending = 1U;
    g_transaction_addr = addr;
    g_transaction_type_detect = type_detect;
    g_transaction_threshold = threshold;
    g_transaction_tx_active = 1U;
    g_transaction_waiting_response = 0U;
    g_transaction_tx_start_tick = start_tick;
    g_transaction_start_tick = 0U;
    RS485DetectUartPrepareTx();
}

static HAL_StatusTypeDef start_transaction(const uint8_t frame[8], uint8_t addr,
                                           uint8_t type_detect, uint8_t threshold,
                                           uint32_t start_tick)
{
    HAL_StatusTypeDef status;

    begin_transaction(frame, addr, type_detect, threshold, start_tick);
    status = HAL_UART_Transmit_IT(&huart5, g_transaction_request, sizeof(g_transaction_request));
    g_rs4853_uart_diag.last_tx_status = (uint8_t)status;
    return status;
}

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/* 根据0x000E寄存器产品型号值查找对应的设备类型枚举 */
static uint8_t lookup_device_type(uint16_t product_code)
{
    uint8_t parser;
    if(DeviceRegistry_IsSupportedOnLoop(product_code, DEVICE_REGISTRY_LOOP3) == 0U) return RS485_DETECT_TYPE_UNKNOWN;
    parser = DeviceRegistry_GetParserType(product_code);
    if(parser == DEVICE_PARSER_XR805) return RS485_DETECT_TYPE_XR805;
    if(parser == DEVICE_PARSER_XR8303) return RS485_DETECT_TYPE_XR8303;
    if(parser == DEVICE_PARSER_XR8305) return RS485_DETECT_TYPE_XR8305;
    if(parser == DEVICE_PARSER_DLYGWG) return RS485_DETECT_TYPE_DLYGWG;
    return RS485_DETECT_TYPE_UNKNOWN;
}

/* 根据设备类型返回对应的传感器寄存器布局表及条目数 */
static const RS485SensorRegDef* get_sensor_layout(uint8_t device_type, uint8_t *count)
{
    if (device_type == RS485_DETECT_TYPE_XR805)
    {
        *count = XR805_SENSOR_COUNT;
        return XR805_SENSOR_LAYOUT;
    }
    else if (device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
    {
        *count = XR8303_SENSOR_COUNT;
        return XR8303_SENSOR_LAYOUT;
    }
    *count = 0;
    return NULL;
}

/* 根据设备类型返回常规04功能码读取数量: V2.21从0x000C起读19个寄存器, 覆盖数值0x000C~0x0012和状态0x0018~0x001E。 */
static uint8_t get_register_count(uint8_t device_type)
{
    if (device_type == RS485_DETECT_TYPE_XR805)
        return 19;
    else if (device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
        return 19;
    else if (device_type == RS485_DETECT_TYPE_DLYGWG)
        return 1;  /* 0x0000 ~ 0x000B */
    return 0;
}

/* ============================================================
 * Flash持久化: 在线状态表存储与加载
 * ============================================================ */

/* 将当前在线状态表保存到Flash(地址0x10F000) */
void RS485Detect_SaveOnlineState(void)
{
    uint8_t online_states[RS485_DETECT_MAX_DEVICES];
    for (uint8_t i = 0; i < RS485_DETECT_MAX_DEVICES; i++)
    {
        online_states[i] = g_devices[i].online;
    }
    W25QXX_Write(online_states, RS485_DETECT_FLASH_ADDR, sizeof(online_states));
}

/* 从Flash加载在线状态表(0xFF视为未配置, 设为离线) */
void RS485Detect_LoadOnlineState(void)
{
    uint8_t online_states[RS485_DETECT_MAX_DEVICES];
    W25QXX_Read(online_states, RS485_DETECT_FLASH_ADDR, sizeof(online_states));

    g_online_count = 0;
    for (uint8_t i = 0; i < RS485_DETECT_MAX_DEVICES; i++)
    {
        if (online_states[i] == 0xFF)
        {
            g_devices[i].online = 0;
        }
        else
        {
            g_devices[i].online = online_states[i];
        }
        if (g_devices[i].online)
        {
            g_online_count++;
        }
    }
}

/* ============================================================
 * 初始化: 清零设备表, 从Flash恢复在线状态
 * ============================================================ */

void RS485Detect_Init(void)
{
    memset(g_devices, 0, sizeof(g_devices));
    for(uint8_t i = 0U; i < RS485_DETECT_MAX_DEVICES; i++) g_devices[i].identify_stage = RS485_STAGE_NATIONAL;
    g_online_count = 0;
    g_poll_current_addr = 1;
    g_last_poll_time = 0;
    g_transaction_pending = 0;
    g_transaction_addr = 0;
    g_transaction_type_detect = 0;
    g_transaction_threshold = 0;
    g_transaction_tx_active = 0U;
    g_transaction_waiting_response = 0U;
    g_transaction_tx_start_tick = 0U;
    g_transaction_start_tick = 0;
    g_last_transaction_end_tick = osKernelGetTickCount();
    g_rx_frame_len = 0U;
    RS485DetectUartClearRx();
    DeviceThreshold_Init();

    RS485Detect_LoadOnlineState();
}

/* ============================================================
 * 在线状态管理: 屏幕下发设置/清除设备在线
 * ============================================================ */

/* 设置单个设备在线状态: 上线时重置类型/传感器/掉线状态, 下线时清空并中止当前事务 */
void RS485Detect_SetOnline(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return;

    if (state == 0)
    {
        g_devices[addr].sensor_data_valid = 0; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
        g_devices[addr].disconnect_count = 0;
        g_devices[addr].recovery_count = 0;
        g_devices[addr].disconnect_memory = 0;
        g_identify_fail_count[addr] = 0U;
        DeviceRegistry_SetProductUnknown(DEVICE_REGISTRY_LOOP3, addr, 0U);
        if (g_transaction_pending != 0U && g_transaction_addr == addr)
        {
            if(g_transaction_threshold != 0U) DeviceThreshold_HandleTimeout();
            release_transaction();
        }
    }

    if (g_devices[addr].online != state)
    {
        g_devices[addr].online = state;
        if (state)
        {
            g_online_count++;
            g_devices[addr].type_confirmed = 0;
            g_devices[addr].identify_stage = RS485_STAGE_NATIONAL;
            g_devices[addr].national_type_code = 0U;
            g_devices[addr].product_code = 0U;
            g_devices[addr].sensor_enable_confirmed = 0; 
            g_devices[addr].sensor_data_valid = 0; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
            g_devices[addr].device_type = RS485_DETECT_TYPE_UNKNOWN;
            g_devices[addr].disconnect_count = 0;
            g_devices[addr].recovery_count = 0;
            g_devices[addr].disconnect_memory = 0;
        }
        else
        {
            if (g_online_count > 0)
                g_online_count--;
            g_devices[addr].type_confirmed = 0;
            g_devices[addr].identify_stage = RS485_STAGE_NATIONAL;
            g_devices[addr].sensor_enable_confirmed = 0;
            g_devices[addr].sensor_data_valid = 0; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
            g_devices[addr].disconnect_count = 0;
            g_devices[addr].recovery_count = 0;
            g_devices[addr].disconnect_memory = 0;
        }
    }
}

/* 批量设置在线状态(start~end地址范围) */
void RS485Detect_SetOnlineRange(uint8_t start, uint8_t end, uint8_t state)
{
    if (start == 0) start = 1;
    if (end >= RS485_DETECT_MAX_DEVICES) end = RS485_DETECT_MAX_DEVICES - 1;

    for (uint8_t i = start; i <= end; i++)
    {
        RS485Detect_SetOnline(i, state);
    }
}

/* ============================================================
 * 状态查询接口
 * ============================================================ */

/* 获取在线标志(屏幕下发值, 非实时在线状态) */
uint8_t RS485Detect_GetOnline(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].online;
}

/* 获取设备类型(需等类型探测完成) */
uint8_t RS485Detect_GetType(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return RS485_DETECT_TYPE_UNKNOWN;
    return g_devices[addr].device_type;
}

uint16_t RS485Detect_GetNationalTypeCode(uint8_t addr)
{
    if(addr == 0U || addr >= RS485_DETECT_MAX_DEVICES) return 0U;
    return g_devices[addr].national_type_code;
}

uint16_t RS485Detect_GetProductCode(uint8_t addr)
{
    if(addr == 0U || addr >= RS485_DETECT_MAX_DEVICES) return 0U;
    return g_devices[addr].product_code;
}

/* 获取指定传感器的数值(温度/烟雾/CO等) */
uint8_t DeviceThreshold_GetLoop3Identity(uint8_t address, DeviceThresholdIdentity *identity)
{
    if(identity == NULL || address == 0U || address >= RS485_DETECT_MAX_DEVICES) return 0U;
    identity->online = RS485Detect_IsOnline(address);
    identity->identified = g_devices[address].type_confirmed;
    identity->device_type = g_devices[address].device_type;
    identity->national_code = g_devices[address].national_type_code;
    identity->product_code = g_devices[address].product_code;
    identity->sensor_enable = g_devices[address].sensor_enable;
    return 1U;
}

uint16_t RS485Detect_GetSensorValue(uint8_t addr, uint8_t sensor_idx)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return 0;
    return g_devices[addr].sensor_values[sensor_idx];
}

/* 获取温度值(有符号, 支持负温) */
int16_t RS485Detect_GetTemperature(uint8_t addr)
{
    return (int16_t)RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE);
}
/* 获取传感器启用位掩码(0x000F寄存器值) */
uint16_t RS485Detect_GetSensorEnable(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_enable;
}

/* 获取传感器状态(0=正常, 1=预警, 2=报警, 3=故障, 8=传感器故障等) */
uint8_t RS485Detect_GetSensorState(uint8_t addr, uint8_t sensor_idx)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return 0;
    return g_devices[addr].sensor_states[sensor_idx];
}

/* 判断是否掉线(连续无响应次数>=阈值) */
uint8_t RS485Detect_IsDisconnected(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return (g_devices[addr].disconnect_count >= RS485_DETECT_DISCONNECT_THRESHOLD) ? 1 : 0;
}

/* 获取回路3在线设备总数 */
uint8_t RS485Detect_GetOnlineCount(void)
{
    return g_online_count;
}

uint8_t RS485Detect_GetActiveCount(void)
{
    uint8_t count = 0U;
    uint8_t addr;
    for(addr = 1U; addr < RS485_DETECT_MAX_DEVICES; addr++)
    {
        if(g_devices[addr].online != 0U && g_devices[addr].type_confirmed != 0U &&
           g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD)
        {
            count++;
        }
    }
    return count;
}

/* 统计回路3掉线设备数量(上线但掉线计数>=阈值) */
uint8_t RS485Detect_GetDisconnectCount(void)
{
    uint8_t count = 0;
    uint8_t i;
    for (i = 1; i < RS485_DETECT_MAX_DEVICES; i++)
    {
        if (g_devices[i].online
            && g_devices[i].disconnect_count >= RS485_DETECT_DISCONNECT_THRESHOLD)
        {
            count++;
        }
    }
    return count;
}

/* 按V2.21统一Modbus协议判断显示/统计用报警状态：
 * 温度/烟雾 state=1 进入火警；CO/H2 state=2低报、state=3高报进入预警。
 * VOC/CH4仅保留原始状态作辅助判断，不进入主机预警/火警显示和报警数量统计。
 * 压力当前只做实时显示，暂不参与消防报警统计。
 */
uint8_t RS485Detect_IsAlarmState(uint8_t device_type, uint8_t sensor_idx, uint8_t state)
{
    (void)device_type;

    if(sensor_idx == RS485_SENSOR_TEMPERATURE || sensor_idx == RS485_SENSOR_SMOKE)
        return state == 1U;
    if(sensor_idx == RS485_SENSOR_CO || sensor_idx == RS485_SENSOR_H2)
        return state == 2U || state == 3U;

    return 0U;
}

/* 按V2.21统一Modbus协议判断传感器故障状态：有效传感器 state=8 表示故障。 */
uint8_t RS485Detect_IsFaultState(uint8_t device_type, uint8_t sensor_idx, uint8_t state)
{
    (void)device_type;
    (void)sensor_idx;

    return state == 8U;
}
/* 统计回路3当前显示/统计用报警设备数: 温度/烟雾火警 + CO/H2低报高报; VOC/CH4不计入显示报警数量。 */
uint8_t RS485Detect_GetAlarmCount(void)
{
    uint8_t count = 0;
    uint8_t alarm_sensors[] = {
        RS485_SENSOR_TEMPERATURE,
        RS485_SENSOR_SMOKE,
        RS485_SENSOR_CO,
        RS485_SENSOR_H2,
    };
    uint8_t alarm_count = sizeof(alarm_sensors) / sizeof(alarm_sensors[0]);
    uint8_t i, j;

    for (i = 1; i < RS485_DETECT_MAX_DEVICES; i++)
    {
        if (!g_devices[i].online)
            continue;
        for (j = 0; j < alarm_count; j++)
        {
            uint8_t sensor = alarm_sensors[j];
            uint8_t state = g_devices[i].sensor_states[sensor];
            uint8_t is_alarm = RS485Detect_IsAlarmState(g_devices[i].device_type, sensor, state);
            if (is_alarm != 0U)
            {
                count++;
                break;
            }
        }
    }
    return count;
}

/* ============================================================
 * 轮询层: Modbus RTU请求构建与发送
 * ============================================================ */

/* 构建传感器数据读取Modbus帧(04功能码, 读reg_count个寄存器) */
static void build_modbus_read_cmd(uint8_t *buf, uint8_t addr, uint8_t reg_count)
{
    uint16_t crc16;

    buf[0] = addr;
    buf[1] = 0x04;
    buf[2] = 0x00;
    buf[3] = 0x0C;
    buf[4] = 0x00;
    buf[5] = reg_count;

    crc16 = CalcCrc16(buf, 6);
    buf[6] = crc16 & 0xFF;
    buf[7] = crc16 >> 8;
}

/* 构建类型探测Modbus帧(04功能码, 读0x000E和0x000F共2个寄存器) */
static void build_type_detect_cmd(uint8_t *buf, uint8_t addr, uint16_t reg)
{
    uint16_t crc16;
    (void)reg;

    buf[0] = addr;
    buf[1] = 0x04;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x03;

    crc16 = CalcCrc16(buf, 6);
    buf[6] = crc16 & 0xFF;
    buf[7] = crc16 >> 8;
}

static void check_and_record_fault(uint8_t addr); /* 前向声明: 掉线/报警检测 */

static void mark_communication_miss(uint8_t addr)
{
    RS485DetectDevice *dev = &g_devices[addr];

    dev->recovery_count = 0U;
    if(dev->disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD)
        dev->disconnect_count++;
    check_and_record_fault(addr);
}

static void mark_communication_success(uint8_t addr)
{
    RS485DetectDevice *dev = &g_devices[addr];

    if(dev->disconnect_count >= RS485_DETECT_DISCONNECT_THRESHOLD)
    {
        if(dev->recovery_count < RS485_DETECT_RECOVERY_SUCCESS_THRESHOLD)
            dev->recovery_count++;
        if(dev->recovery_count >= RS485_DETECT_RECOVERY_SUCCESS_THRESHOLD)
        {
            dev->disconnect_count = 0U;
            dev->recovery_count = 0U;
        }
    }
    else
    {
        dev->disconnect_count = 0U;
        dev->recovery_count = 0U;
    }
    check_and_record_fault(addr);
}

static void mark_identify_failure(uint8_t addr, DeviceIdentifyError error)
{
    if(addr == 0U || addr >= RS485_DETECT_MAX_DEVICES) return;
    if(error == DEVICE_IDENTIFY_NATIONAL_UNKNOWN || error == DEVICE_IDENTIFY_CODE_MISMATCH)
        g_devices[addr].identify_stage = RS485_STAGE_NATIONAL;
    if(g_identify_fail_count[addr] < RS485_IDENTIFY_FAIL_THRESHOLD) g_identify_fail_count[addr]++;
    g_last_identify_tick[addr] = osKernelGetTickCount();
    if(g_identify_fail_count[addr] >= RS485_IDENTIFY_FAIL_THRESHOLD)
        DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP3, addr, error);
}

/* 检查当前事务是否超时: 超时则计掉线+触发故障检测 */
static void mark_transaction_timeout(void)
{
    uint8_t addr;
    uint8_t was_type_detect;
    if (g_transaction_pending == 0U) return;
    if (g_transaction_waiting_response == 0U) return;
    if ((osKernelGetTickCount() - g_transaction_start_tick) < RS485_DETECT_RESPONSE_TIMEOUT_MS) return;
    addr = g_transaction_addr;
    was_type_detect = g_transaction_type_detect;
    if(g_transaction_threshold != 0U)
    {
        DeviceThreshold_HandleTimeout();
        g_poll_current_addr = (addr > 1U) ? (uint8_t)(addr - 1U) : (RS485_DETECT_MAX_DEVICES - 1U);
        release_transaction();
        return;
    }
    release_transaction();
    if (addr > 0U && addr < RS485_DETECT_MAX_DEVICES && g_devices[addr].online != 0U)
    {
        if(was_type_detect != 0U)
            mark_identify_failure(addr, DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE);
        else
        {
            DeviceThreshold_NotifyNormalPoll();
            mark_communication_miss(addr);
        }
    }
}

/* 中断发送完成后才开始计算从机应答超时；发送阶段超时不计为设备掉线。 */
static void process_transaction_tx_state(void)
{
    uint8_t addr;

    if(g_transaction_pending == 0U || g_transaction_tx_active == 0U)
    {
        (void)RS485DetectUartTakeTxComplete();
        return;
    }

    if(RS485DetectUartTakeTxComplete() != 0U)
    {
        g_transaction_tx_active = 0U;
        g_transaction_waiting_response = 1U;
        g_transaction_start_tick = osKernelGetTickCount();
        return;
    }

    if((osKernelGetTickCount() - g_transaction_tx_start_tick) < RS485_DETECT_TX_COMPLETE_TIMEOUT_MS)
        return;

    addr = g_transaction_addr;
    (void)HAL_UART_AbortTransmit(&huart5);
    g_rs4853_uart_diag.last_tx_status = (uint8_t)HAL_TIMEOUT;
    g_rs4853_uart_diag.tx_fail_count++;
    g_rs4853_uart_diag.tx_complete_timeout_count++;
    if(g_transaction_threshold != 0U)
    {
        DeviceThreshold_HandleTimeout();
        g_poll_current_addr = (addr > 1U) ? (uint8_t)(addr - 1U) : (RS485_DETECT_MAX_DEVICES - 1U);
    }
    release_transaction();
}

/* 轮询下一个在线设备: 未确认类型则发类型探测帧, 已确认则发传感器数据帧 */
static void poll_next_device(void)
{
    uint8_t modbusbuf[8];
    uint8_t found = 0;
    uint8_t start_addr = g_poll_current_addr;
    uint8_t addr;
    uint8_t type_detect;
    HAL_StatusTypeDef tx_status;

    if (g_online_count == 0U || g_transaction_pending != 0U)
        return;

    for (uint8_t attempt = 0; attempt < RS485_DETECT_MAX_DEVICES; attempt++)
    {
        g_poll_current_addr++;
        if (g_poll_current_addr >= RS485_DETECT_MAX_DEVICES)
            g_poll_current_addr = 1;

        if (g_devices[g_poll_current_addr].online != 0U &&
            (DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP3, g_poll_current_addr) == 0U ||
             (osKernelGetTickCount() - g_last_identify_tick[g_poll_current_addr]) >= RS485_IDENTIFY_RETRY_MS))
        {
            found = 1;
            break;
        }

        if (g_poll_current_addr == start_addr)
            break;
    }

    if (found == 0U)
        return;

    addr = g_poll_current_addr;
    type_detect = (g_devices[addr].type_confirmed == 0U) ? g_devices[addr].identify_stage : 0U;

    if (type_detect != 0U)
    {
        build_type_detect_cmd(modbusbuf, addr, RS485_DETECT_NATIONAL_TYPE_REG);
    }
    else
    {
        uint8_t reg_count = get_register_count(g_devices[addr].device_type);
        if (reg_count == 0U)
            return;
        build_modbus_read_cmd(modbusbuf, addr, reg_count);
    }

    tx_status = start_transaction(modbusbuf, addr, type_detect, 0U, osKernelGetTickCount());
    if (tx_status != HAL_OK)
    {
        /* XR5000_UART5_EXCLUSIVE_FIX_20260730: unsent requests are not detector misses. */
        g_rs4853_uart_diag.tx_fail_count++;
        release_transaction();
    }
}
/* 解析传感器数据帧: 按设备类型对应的布局表, 将响应字节填入sensor_values和sensor_states */
static void parse_sensor_data(uint8_t addr, const uint8_t *bytes, uint8_t device_type)
{
    uint8_t sensor_count;
    const RS485SensorRegDef *layout = get_sensor_layout(device_type, &sensor_count);

    if (!layout || sensor_count == 0)
        return;

    uint16_t enable = g_devices[addr].sensor_enable;

    for (uint8_t i = 0; i < sensor_count; i++)
    {
        uint8_t offset = layout[i].byte_offset;
        uint8_t sensor_idx = layout[i].sensor_index;

        /* 根据sensor_index查0x000F中的bit位 */
        uint8_t bit;
        switch (sensor_idx)
        {
            case RS485_SENSOR_TEMPERATURE: bit = 5; break;
            case RS485_SENSOR_SMOKE:       bit = 0; break;
            case RS485_SENSOR_CO:          bit = 4; break;
            case RS485_SENSOR_H2:          bit = 2; break;
            case RS485_SENSOR_VOC:         bit = 3; break;
            case RS485_SENSOR_CH4:         bit = 1; break;
            case RS485_SENSOR_PRESSURE:    bit = 6; break;
            default: continue;
        }

        if (!(enable & (1 << bit)))
        {
            /* 传感器未启用，清零 */
            if (layout[i].is_value)
                g_devices[addr].sensor_values[sensor_idx] = 0;
            else
                g_devices[addr].sensor_states[sensor_idx] = 0;
            continue;
        }

        if (layout[i].is_value)
        {
            uint16_t value = (bytes[offset] << 8) | bytes[offset + 1];
            g_devices[addr].sensor_values[sensor_idx] = value;
        }
        else
        {
            uint16_t state = (bytes[offset] << 8) | bytes[offset + 1];
            g_devices[addr].sensor_states[sensor_idx] = (uint8_t)state;
        }
    }
}

/* 故障检测: 检查掉线状态和报警状态, 更新disconnect_memory和alarm_memory(故障记录由cmd_process.c统一处理) */
static void check_and_record_fault(uint8_t addr)
{
    RS485DetectDevice *dev = &g_devices[addr];

    /* 掉线检测 */
    if (dev->disconnect_count >= RS485_DETECT_DISCONNECT_THRESHOLD)
    {
        if (dev->disconnect_memory == 0)
        {
            dev->disconnect_memory = 1;
            // XR5000_LOOP3_CHANGE_20260726: Flash logging is handled in cmd_process.c.
        }
    }
    else
    {
        if (dev->disconnect_memory == 1)
        {
            dev->disconnect_memory = 0;
            // XR5000_LOOP3_CHANGE_20260726: Flash logging is handled in cmd_process.c.
        }
    }

    /* 报警记忆检测：只处理进入主机显示/统计的温度、烟雾、CO、H2；VOC/CH4不进入报警显示区。 */
    uint8_t alarm_sensors[] = {
        RS485_SENSOR_TEMPERATURE,
        RS485_SENSOR_SMOKE,
        RS485_SENSOR_CO,
        RS485_SENSOR_H2,
    };
    uint8_t alarm_count = sizeof(alarm_sensors) / sizeof(alarm_sensors[0]);

    for (uint8_t i = 0; i < alarm_count; i++)
    {
        uint8_t idx = alarm_sensors[i];
        uint8_t state = dev->sensor_states[idx];

        if (RS485Detect_IsAlarmState(dev->device_type, idx, state) != 0U)
        {
            if (dev->alarm_memory[idx] == 0)
            {
                dev->alarm_memory[idx] = 1;
                // XR5000_LOOP3_CHANGE_20260726: Flash logging is handled in cmd_process.c.
            }
        }
        else
        {
            if (dev->alarm_memory[idx] == 1)
            {
                dev->alarm_memory[idx] = 0;
                // XR5000_LOOP3_CHANGE_20260726: Flash logging is handled in cmd_process.c.
            }
        }
    }
}

/* 完成当前事务: 更新通信恢复状态并释放UART5事务锁 */
static void complete_transaction(uint8_t addr)
{
    if(g_transaction_threshold == 0U && g_transaction_type_detect == 0U)
    {
        DeviceThreshold_NotifyNormalPoll();
        mark_communication_success(addr);
    }
    release_transaction();
}

/* 处理已经完成组帧和CRC校验的单个响应。 */
static void process_received_frame(const uint8_t *buf, uint16_t len)
{
    uint8_t addr;
    uint8_t func;
    uint8_t byte_count;
    uint8_t expected_byte_count;

    if (g_transaction_pending == 0U || len < 5U)
        return;

    addr = buf[0];
    func = buf[1];

    /* XR5000_UART5_EXCLUSIVE_FIX_20260730: only the owned request may consume a response. */
    if (addr != g_transaction_addr || addr == 0U || addr >= RS485_DETECT_MAX_DEVICES)
        return;

    if(g_transaction_threshold != 0U)
    {
        if(DeviceThreshold_HandleResponse(buf, len) != 0U)
        {
            g_poll_current_addr = (addr > 1U) ? (uint8_t)(addr - 1U) : (RS485_DETECT_MAX_DEVICES - 1U);
            release_transaction();
        }
        return;
    }

    if (func == 0x84U)
    {
        uint8_t was_type_detect = g_transaction_type_detect;
        if(len != 5U)
        {
            g_rs4853_uart_diag.rx_invalid_length_count++;
            return;
        }
        g_rs4853_uart_diag.rx_protocol_exception_count++;
        if (was_type_detect != 0U)
        {
            mark_identify_failure(addr, DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE);
            release_transaction();
        }
        else
            complete_transaction(addr);
        return;
    }

    if (func != 0x04U || len < 5U)
        return;

    byte_count = buf[2];
    if (len != (uint16_t)(byte_count + 5U))
    {
        g_rs4853_uart_diag.rx_invalid_length_count++;
        return;
    }

    if (g_transaction_type_detect != 0U)
    {
        expected_byte_count = 6U;
    }
    else
    {
        uint8_t device_type = g_devices[addr].device_type;
        if (device_type == RS485_DETECT_TYPE_UNKNOWN)
            return;
        expected_byte_count = get_register_count(device_type) * 2U;
    }

    if (byte_count != expected_byte_count)
    {
        g_rs4853_uart_diag.rx_invalid_length_count++;
        return;
    }

    if (g_transaction_type_detect != 0U)
    {
        uint16_t national_code = ((uint16_t)buf[3] << 8) | buf[4];
        uint16_t product_code = ((uint16_t)buf[5] << 8) | buf[6];
        uint16_t sensor_mask = ((uint16_t)buf[7] << 8) | buf[8];
        uint8_t detected_type = lookup_device_type(product_code);
        g_devices[addr].national_type_code = national_code;
        if(detected_type == RS485_DETECT_TYPE_UNKNOWN)
            mark_identify_failure(addr, DEVICE_IDENTIFY_PRODUCT_UNKNOWN);
        else if(DeviceRegistry_IsNationalProductMatch(national_code, product_code) == 0U)
            mark_identify_failure(addr, DeviceRegistry_IsNationalTypeKnown(national_code) != 0U ? DEVICE_IDENTIFY_CODE_MISMATCH : DEVICE_IDENTIFY_NATIONAL_UNKNOWN);
        else if(DeviceRegistry_RequiresSensorMask(product_code) != 0U && DeviceRegistry_IsSensorMaskValid(product_code, sensor_mask) == 0U)
            mark_identify_failure(addr, DEVICE_IDENTIFY_SENSOR_TYPE_UNKNOWN);
        else
        {
            g_devices[addr].product_code = product_code;
            g_devices[addr].device_type = detected_type;
            g_devices[addr].sensor_enable = sensor_mask;
            g_devices[addr].sensor_enable_confirmed = 1U;
            g_devices[addr].type_confirmed = 1U;
            g_devices[addr].identify_stage = RS485_STAGE_COMPLETE;
            g_identify_fail_count[addr] = 0U;
            DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP3, addr, DEVICE_IDENTIFY_OK);
        }
        complete_transaction(addr);
    }
    else
    {
        uint8_t device_type = g_devices[addr].device_type;
        parse_sensor_data(addr, buf, device_type);
        g_devices[addr].sensor_data_valid = 1U; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
        complete_transaction(addr);
    }
}

static uint8_t is_expected_response_function(uint8_t function)
{
    uint8_t request_function = g_transaction_request[1];

    if(g_transaction_threshold != 0U)
        return (function == request_function || function == (uint8_t)(request_function | 0x80U)) ? 1U : 0U;
    return (function == 0x04U || function == 0x84U) ? 1U : 0U;
}

static void resync_rx_frame(void)
{
    uint16_t pos;

    for(pos = 1U; pos < g_rx_frame_len; pos++)
    {
        if(g_rx_frame[pos] == g_transaction_addr) break;
    }
    if(pos < g_rx_frame_len)
    {
        memmove(g_rx_frame, &g_rx_frame[pos], g_rx_frame_len - pos);
        g_rx_frame_len = (uint16_t)(g_rx_frame_len - pos);
    }
    else
    {
        g_rx_frame_len = 0U;
    }
    g_rs4853_uart_diag.rx_resync_count++;
}

static void try_process_rx_frame(void)
{
    uint16_t expected_length;
    uint16_t crc16;
    uint8_t function;

    while(g_transaction_pending != 0U && g_rx_frame_len > 0U)
    {
        if(g_rx_frame[0] != g_transaction_addr)
        {
            resync_rx_frame();
            continue;
        }
        if(g_rx_frame_len < 2U) return;
        function = g_rx_frame[1];
        if(is_expected_response_function(function) == 0U)
        {
            resync_rx_frame();
            continue;
        }
        if(g_rx_frame_len < 3U) return;

        if((function & 0x80U) != 0U)
            expected_length = 5U;
        else if(function == 0x06U)
            expected_length = 8U;
        else if(g_rx_frame[2] == 0U)
            expected_length = 8U; /* 04/03查询回显，而不是合法响应。 */
        else
            expected_length = (uint16_t)g_rx_frame[2] + 5U;

        if(expected_length > sizeof(g_rx_frame) || expected_length < 5U)
        {
            g_rs4853_uart_diag.rx_invalid_length_count++;
            resync_rx_frame();
            continue;
        }
        if(g_rx_frame_len < expected_length) return;

        if(expected_length == sizeof(g_transaction_request) &&
           (function == 0x04U || function == 0x03U) &&
           memcmp(g_rx_frame, g_transaction_request, sizeof(g_transaction_request)) == 0)
        {
            g_rs4853_uart_diag.rx_echo_count++;
            g_rx_frame_len = 0U;
            return;
        }

        crc16 = ((uint16_t)g_rx_frame[expected_length - 1U] << 8) |
                g_rx_frame[expected_length - 2U];
        if(CalcCrc16(g_rx_frame, expected_length - 2U) != crc16)
        {
            g_rs4853_uart_diag.rx_crc_error_count++;
            resync_rx_frame();
            continue;
        }

        process_received_frame(g_rx_frame, expected_length);
        g_rx_frame_len = 0U;
        return;
    }
}

/* 将任意DMA片段作为连续字节流消费，允许一帧跨多个回调。 */
static void receive_data_deal(void)
{
    uint16_t count;
    uint16_t i;

    while((count = RS485DetectUartRead(g_rx_chunk, sizeof(g_rx_chunk))) != 0U)
    {
        for(i = 0U; i < count; i++)
        {
            if(g_transaction_pending == 0U)
            {
                g_rx_frame_len = 0U;
                continue;
            }
            if(g_rx_frame_len >= sizeof(g_rx_frame))
            {
                g_rs4853_uart_diag.rx_invalid_length_count++;
                resync_rx_frame();
            }
            g_rx_frame[g_rx_frame_len++] = g_rx_chunk[i];
            try_process_rx_frame();
        }
    }
}
/* 判断是否真正在线(上线且未掉线) */
uint8_t RS485Detect_IsOnline(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return (g_devices[addr].online && g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD);
}

/* 是否已收到有效的传感器数据帧 */
uint8_t RS485Detect_HasSensorData(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_data_valid;
}

/* 是否已确认传感器启用状态(已成功读取0x000F寄存器) */
uint8_t RS485Detect_HasSensorEnableData(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_enable_confirmed;
}

/* ============================================================
 * RTOS轮询任务: 主循环→接收处理→超时检测→定时轮询, 间隔10ms
 * ============================================================ */

void RS485DetectPollAndReceiveTask(void *parameter)
{
    uint32_t current_tick;

    (void)parameter;
    g_last_poll_time = osKernelGetTickCount();

    for (;;)
    {
        (void)RS485DetectUartEnsureRx();
        process_transaction_tx_state();
        receive_data_deal();
        mark_transaction_timeout();

        current_tick = osKernelGetTickCount();
        if (g_transaction_pending == 0U &&
            (current_tick - g_last_poll_time) >= RS485_DETECT_POLL_INTERVAL_MS &&
            (current_tick - g_last_transaction_end_tick) >= RS485_DETECT_INTER_FRAME_GUARD_MS)
        {
            uint8_t threshold_frame[8];
            uint8_t threshold_addr = 0U;
            HAL_StatusTypeDef tx_status;
            g_last_poll_time = current_tick;
            if(DeviceThreshold_BuildNextFrame(threshold_frame, &threshold_addr) != 0U)
            {
                tx_status = start_transaction(threshold_frame, threshold_addr, 0U, 1U, current_tick);
                if(tx_status != HAL_OK)
                {
                    g_rs4853_uart_diag.tx_fail_count++;
                    DeviceThreshold_HandleTimeout();
                    g_poll_current_addr = (threshold_addr > 1U) ? (uint8_t)(threshold_addr - 1U) : (RS485_DETECT_MAX_DEVICES - 1U);
                    release_transaction();
                }
            }
            else
            {
                poll_next_device();
            }
        }

        osDelay(RS485_DETECT_TASK_INTERVAL_MS);
    }
}


/* 测试注入: 设置RS485探测器传感器状态(bsp_test_inject调用) */
void RS485Detect_InjectSensorState(uint8_t addr, uint8_t sensor_idx, uint8_t state)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return;
    g_devices[addr].online = 1;
    g_devices[addr].disconnect_count = 0;
    g_devices[addr].recovery_count = 0;
    g_devices[addr].type_confirmed = 1;
    g_devices[addr].sensor_data_valid = 1;
    g_devices[addr].sensor_states[sensor_idx] = state;
}

/* 获取国标设备类型码(供逻辑屏显示), 地址非法返回0 */
uint16_t RS485Detect_GetNationalCode(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].national_type_code;
}
