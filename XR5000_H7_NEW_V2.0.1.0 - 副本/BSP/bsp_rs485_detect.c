/* ============================================================================
 * 文件功能: RS485探测器轮询管理 (RS485 Detector Management)
 * 功能描述: 负责回路3(UART5) RS485探测器轮询, 采用Modbus RTU协议
 *          轮询结果通过回调函数通知上层模块/更新设备状态Flash存储
 * 通信协议: Modbus RTU, 功能码04(读取输入寄存器), UART5/115200/8N1
 * 寄存器定义: 使用设备寄存器表(查0x000E/0x000F), 按产品类型差异化布局
 *           (XR805含16个寄存器, XR8303/XR8305含12个寄存器)
 * 数据流向: 轮询/超时/状态更新, 逐层上报至cmd_process.c处理
 * 回路编号: 回路3, Flash存储地址0x10F000, 产品ID=0x53(83进制)
 * ============================================================================ */

#include "bsp_rs485_detect.h"
#include "bsp_device_registry.h"

#include "FreeRTOS.h"          
#include "cmsis_os.h"          
#include "queue.h"             

#include "bsp_itcallback.h"    
#include "usart.h"             
#include "bsp_save_ctrl.h"     
#include "system.h"            
#include "w25qxx.h"            

/* ============================================================
 * 第一层: 传感器寄存器布局表 & 设备类型映射
 * ============================================================ */

/* 产品类型映射: 0x000E寄存器值对应设备类型 */
/* XR805传感器寄存器布局: 04功能码读16个寄存器(0x0000~0x000F), 含温度/烟雾/CH4/CO/VOC/H2共6种传感器 */
static const RS485SensorRegDef XR805_SENSOR_LAYOUT[] = {
    {0x0000, 3,  RS485_SENSOR_TEMPERATURE, 1},  /* 温度值 */
    {0x0001, 5,  RS485_SENSOR_TEMPERATURE, 0},  /* 温度状态 */
    {0x0002, 7,  RS485_SENSOR_SMOKE,       1},  /* 烟雾值 */
    {0x0003, 9,  RS485_SENSOR_SMOKE,       0},  /* 烟雾状态 */
    {0x0004, 11, RS485_SENSOR_CH4,         1},  /* CH4值 */
    {0x0005, 13, RS485_SENSOR_CH4,         0},  /* CH4状态 */
    {0x0006, 15, RS485_SENSOR_CO,          1},  /* CO值 */
    {0x0007, 17, RS485_SENSOR_CO,          0},  /* CO状态 */
    {0x0008, 19, RS485_SENSOR_VOC,         1},  /* VOC值 */
    {0x0009, 21, RS485_SENSOR_VOC,         0},  /* VOC状态 */
    {0x000A, 23, RS485_SENSOR_H2,          1},  /* H2值 */
    {0x000B, 25, RS485_SENSOR_H2,          0},  /* H2状态 */
};
#define XR805_SENSOR_COUNT  (sizeof(XR805_SENSOR_LAYOUT) / sizeof(XR805_SENSOR_LAYOUT[0]))

/* XR8303/XR8305传感器寄存器布局: 04功能码读12个寄存器(0x0000~0x000B), 含温度/烟雾/CO/H2/VOC/压力共6种传感器 */
static const RS485SensorRegDef XR8303_SENSOR_LAYOUT[] = {
    {0x0000, 3,  RS485_SENSOR_TEMPERATURE, 1},  /* 温度值 */
    {0x0001, 5,  RS485_SENSOR_TEMPERATURE, 0},  /* 温度状态 */
    {0x0002, 7,  RS485_SENSOR_SMOKE,       1},  /* 烟雾值 */
    {0x0003, 9,  RS485_SENSOR_SMOKE,       0},  /* 烟雾状态 */
    {0x0004, 11, RS485_SENSOR_CO,          1},  /* CO值 */
    {0x0005, 13, RS485_SENSOR_CO,          0},  /* CO状态 */
    {0x0006, 15, RS485_SENSOR_H2,          1},  /* H2值 */
    {0x0007, 17, RS485_SENSOR_H2,          0},  /* H2状态 */
    {0x0008, 19, RS485_SENSOR_VOC,         1},  /* VOC值 */
    {0x0009, 21, RS485_SENSOR_VOC,         0},  /* VOC状态 */
    {0x000A, 23, RS485_SENSOR_PRESSURE,    1},  /* 压力值 */
    {0x000B, 25, RS485_SENSOR_PRESSURE,    0},  /* 压力状态 */
};
#define XR8303_SENSOR_COUNT  (sizeof(XR8303_SENSOR_LAYOUT) / sizeof(XR8303_SENSOR_LAYOUT[0]))

/* ============================================================
 * 第二层: 全局设备状态变量
 * ============================================================ */

static RS485DetectDevice g_devices[RS485_DETECT_MAX_DEVICES]; /* 设备状态数组(地址=索引) */

static uint8_t g_online_count = 0;       /* 当前在线设备总数 */
static uint8_t g_poll_current_addr = 1;  /* 当前轮询地址(1~33循环) */
static uint32_t g_last_poll_time = 0;    /* 上次轮询时间tick */

/* UART5事务控制: 防止并发发送请求占用UART5 */
static uint8_t g_transaction_pending = 0;       /* 是否有事务正在进行中 */
static uint8_t g_transaction_addr = 0;          /* 当前事务的目标地址 */
static uint8_t g_transaction_type_detect = 0;   /* 当前事务是否为类型探测(1=类型探测, 0=普通数据轮询) */
static uint32_t g_transaction_start_tick = 0;
static uint8_t g_identify_fail_count[RS485_DETECT_MAX_DEVICES];
static uint32_t g_last_identify_tick[RS485_DETECT_MAX_DEVICES];
#define RS485_IDENTIFY_FAIL_THRESHOLD 3U
#define RS485_IDENTIFY_RETRY_MS 1000U   /* 重试类型探测间隔tick(避免连续发送阻塞) */
#define RS485_STAGE_NATIONAL 1U
#define RS485_STAGE_PRODUCT  2U
#define RS485_STAGE_SENSOR   3U
#define RS485_STAGE_COMPLETE 4U

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/* 通过0x000E寄存器值查询产品类型是否属于回路3支持设备 */
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

/* 获取传感器布局: 按设备类型返回对应的传感器寄存器表 */
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

/* 获取寄存器数量: 按设备类型返回04功能码读取的寄存器数(XR805=16, XR8303/XR8305=12) */
static uint8_t get_register_count(uint8_t device_type)
{
    if (device_type == RS485_DETECT_TYPE_XR805)
        return 16;  /* 0x0000 ~ 0x000F */
    else if (device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
        return 12;
    else if (device_type == RS485_DETECT_TYPE_DLYGWG)
        return 1;  /* 0x0000 ~ 0x000B */
    return 0;
}

/* ============================================================
 * Flash存储: 在线状态持久化存储
 * ============================================================ */

/* 将当前在线状态保存到Flash(地址0x10F000) */
void RS485Detect_SaveOnlineState(void)
{
    uint8_t online_states[RS485_DETECT_MAX_DEVICES];
    for (uint8_t i = 0; i < RS485_DETECT_MAX_DEVICES; i++)
    {
        online_states[i] = g_devices[i].online;
    }
    W25QXX_Write(online_states, RS485_DETECT_FLASH_ADDR, sizeof(online_states));
}

/* 从Flash恢复在线状态(0xFF表示未初始化, 视为离线) */
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
 * 初始化: 重置设备状态, 从Flash恢复在线
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
    g_transaction_start_tick = 0;

    RS485Detect_LoadOnlineState();
}

/* ============================================================
 * 在线状态管理: 设置/获取设备在线状态
 * ============================================================ */

/* 设置设备在线状态: 同时更新类型/产品码/识别标志, 触发DeviceRegistry回调 */
void RS485Detect_SetOnline(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return;

    if (state == 0)
    {
        g_devices[addr].sensor_data_valid = 0; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
        g_devices[addr].disconnect_count = 0;
        g_devices[addr].disconnect_memory = 0;
        g_identify_fail_count[addr] = 0U;
        DeviceRegistry_SetProductUnknown(DEVICE_REGISTRY_LOOP3, addr, 0U);
        if (g_transaction_pending != 0U && g_transaction_addr == addr)
        {
            g_transaction_pending = 0;
            g_transaction_addr = 0;
            g_transaction_type_detect = 0;
            g_transaction_start_tick = 0;
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
 * 查询接口
 * ============================================================ */

/* 获取在线状态(地址无效返回0) */
uint8_t RS485Detect_GetOnline(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].online;
}

/* 获取设备类型(地址无效返回UNKNOWN) */
uint8_t RS485Detect_GetType(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return RS485_DETECT_TYPE_UNKNOWN;
    return g_devices[addr].device_type;
}

/* 获取传感器原始值(温度/烟雾/CO等) */
uint16_t RS485Detect_GetSensorValue(uint8_t addr, uint8_t sensor_idx)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return 0;
    return g_devices[addr].sensor_values[sensor_idx];
}

/* 获取温度值(带符号, 单位0.1℃) */
int16_t RS485Detect_GetTemperature(uint8_t addr)
{
    return (int16_t)RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE);
}
/* 获取传感器使能位掩码(0x000F寄存器值) */
uint16_t RS485Detect_GetSensorEnable(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_enable;
}

/* 获取传感器状态值(0=正常, 1=预警, 2=报警, 3=故障, 8=传感器故障需清洗) */
uint8_t RS485Detect_GetSensorState(uint8_t addr, uint8_t sensor_idx)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return 0;
    return g_devices[addr].sensor_states[sensor_idx];
}

/* 获取设备国标设备类型码(供联动逻辑显示用), 地址无效返回0 */
uint16_t RS485Detect_GetNationalCode(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].national_type_code;
}

/* XR5000_INJECT_EXT_20260818: Inject sensor state for test only.
 * Force device online and clear disconnect counter, then write sensor
 * state to simulate slave alarm without real RS485 hardware. */
void RS485Detect_InjectSensorState(uint8_t addr, uint8_t sensor_idx, uint8_t state)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES || sensor_idx >= RS485_SENSOR_COUNT)
        return;
    g_devices[addr].online = 1;
    g_devices[addr].disconnect_count = 0;
    g_devices[addr].type_confirmed = 1;
    g_devices[addr].sensor_data_valid = 1;
    g_devices[addr].sensor_states[sensor_idx] = state;
}

/* 判断断线状态(断线计数>=阈值) */
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

/* 获取回路3在线且活跃设备数(在线且已识别且断线计数<阈值) */
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

/* 判断报警状态: XR805(state=1/2), XR8303/XR8305(温度/烟雾/CO/H2/VOC各有不同阈值) */
uint8_t RS485Detect_IsAlarmState(uint8_t device_type, uint8_t sensor_idx, uint8_t state)
{
    if(device_type == RS485_DETECT_TYPE_XR805) return state == 1U || state == 2U;
    if(device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
    {
        if(sensor_idx == RS485_SENSOR_TEMPERATURE) return state == 1U || state == 2U;
        if(sensor_idx == RS485_SENSOR_SMOKE) return state == 1U;
        if(sensor_idx == RS485_SENSOR_CO || sensor_idx == RS485_SENSOR_H2) return state == 1U || state == 2U;
        if(sensor_idx == RS485_SENSOR_VOC) return state == 1U;
    }
    return 0U;
}

/* 判断故障状态: XR805(state=9), XR8303/XR8305(温度state=3/烟雾state=8) */
uint8_t RS485Detect_IsFaultState(uint8_t device_type, uint8_t sensor_idx, uint8_t state)
{
    if(device_type == RS485_DETECT_TYPE_XR805) return state == 9U;
    if(device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
    {
        if(sensor_idx == RS485_SENSOR_TEMPERATURE) return state == 3U;
        if(sensor_idx == RS485_SENSOR_SMOKE) return state == 8U;
    }
    return 0U;
}
/* 获取回路3总报警设备数(仅计算在线设备, 任一传感器报警即计数) */
uint8_t RS485Detect_GetAlarmCount(void)
{
    uint8_t count = 0;
    uint8_t alarm_sensors[] = {
        RS485_SENSOR_TEMPERATURE,
        RS485_SENSOR_SMOKE,
        RS485_SENSOR_CO,
        RS485_SENSOR_H2,
        RS485_SENSOR_CH4,
        RS485_SENSOR_VOC,
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
 * 第三层: Modbus RTU协议收发
 * ============================================================ */

/* 构建Modbus读数据命令(04功能码, 指定reg_count寄存器数) */
static void build_modbus_read_cmd(uint8_t *buf, uint8_t addr, uint8_t reg_count)
{
    uint16_t crc16;

    buf[0] = addr;
    buf[1] = 0x04;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = reg_count;

    crc16 = CalcCrc16(buf, 6);
    buf[6] = crc16 & 0xFF;
    buf[7] = crc16 >> 8;
}

/* 构建类型探测Modbus命令(04功能码, 读0x000E或0x000F单寄存器) */
static void build_type_detect_cmd(uint8_t *buf, uint8_t addr, uint16_t reg)
{
    uint16_t crc16;

    buf[0] = addr;
    buf[1] = 0x04;
    buf[2] = (reg >> 8) & 0xFF;
    buf[3] = reg & 0xFF;
    buf[4] = 0x00;
    buf[5] = 0x01;

    crc16 = CalcCrc16(buf, 6);
    buf[6] = crc16 & 0xFF;
    buf[7] = crc16 >> 8;
}

static void check_and_record_fault(uint8_t addr); /* 前向声明: 断线/故障记录 */
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

/* 检查当前事务: 超时则重置事务+增加断线计数 */
static void mark_transaction_timeout(void)
{
    uint8_t addr;
    uint8_t was_type_detect;
    if (g_transaction_pending == 0U) return;
    if ((osKernelGetTickCount() - g_transaction_start_tick) < RS485_DETECT_RESPONSE_TIMEOUT_MS) return;
    addr = g_transaction_addr;
    was_type_detect = g_transaction_type_detect;
    g_transaction_pending = 0;
    g_transaction_addr = 0;
    g_transaction_type_detect = 0;
    if (addr > 0U && addr < RS485_DETECT_MAX_DEVICES && g_devices[addr].online != 0U)
    {
        if(was_type_detect != 0U)
            mark_identify_failure(addr, was_type_detect == RS485_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : was_type_detect == RS485_STAGE_PRODUCT ? DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE : DEVICE_IDENTIFY_SENSOR_READ_FAILED);
        else
        {
            if (g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD)
                g_devices[addr].disconnect_count++;
            check_and_record_fault(addr);
        }
    }
}

/* 轮询下一个在线设备: 未识别则先类型探测, 已识别则读传感器数据 */
static void poll_next_device(void)
{
    uint8_t modbusbuf[8];
    uint8_t found = 0;
    uint8_t start_addr = g_poll_current_addr;
    uint8_t addr;
    uint8_t type_detect;

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
        uint16_t reg = type_detect == RS485_STAGE_NATIONAL ? RS485_DETECT_NATIONAL_TYPE_REG : type_detect == RS485_STAGE_PRODUCT ? RS485_DETECT_TYPE_REG : RS485_DETECT_SENSOR_ENABLE_REG;
        build_type_detect_cmd(modbusbuf, addr, reg);
    }
    else
    {
        uint8_t reg_count = get_register_count(g_devices[addr].device_type);
        if (reg_count == 0U)
            return;
        build_modbus_read_cmd(modbusbuf, addr, reg_count);
    }

    uartbuff[DEBUGSITE].recepetion_flag = 0;
    uartbuff[DEBUGSITE].recepetion_len = 0;

    g_transaction_pending = 1;
    g_transaction_addr = addr;
    g_transaction_type_detect = type_detect;
    g_transaction_start_tick = osKernelGetTickCount();

    if (HAL_UART_Transmit(&huart5, modbusbuf, sizeof(modbusbuf), RS485_DETECT_TX_TIMEOUT_MS) != HAL_OK)
    {
        /* XR5000_UART5_EXCLUSIVE_FIX_20260730: unsent requests are not detector misses. */
        g_transaction_pending = 0;
        g_transaction_addr = 0;
        g_transaction_type_detect = 0;
    }
}
/* 解析传感器数据: 将设备返回的原始字节按布局解析到sensor_values和sensor_states */
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

        /* 匹配sensor_index到0x000F中的bit位 */
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
            /* 传感器未使能, 跳过 */
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

/* 故障检查: 检查断线和传感器报警, 设置disconnect_memory和alarm_memory(实际记录在cmd_process.c中处理) */
static void check_and_record_fault(uint8_t addr)
{
    RS485DetectDevice *dev = &g_devices[addr];

    /* 断线检查 */
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

    /* 传感器报警检查: 温度/烟雾/CO/H2/CH4/VOC等 */
    uint8_t alarm_sensors[] = {
        RS485_SENSOR_TEMPERATURE,
        RS485_SENSOR_SMOKE,
        RS485_SENSOR_CO,
        RS485_SENSOR_H2,
        RS485_SENSOR_CH4,
        RS485_SENSOR_VOC,
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

/* 完成事务: 清除断线计数, 释放UART5占用 */
static void complete_transaction(uint8_t addr)
{
    g_devices[addr].disconnect_count = 0;
    g_transaction_pending = 0;
    g_transaction_addr = 0;
    g_transaction_type_detect = 0;
    g_transaction_start_tick = 0;
}

/* 接收数据处理: 校验CRC后根据事务类型分发解析 */
static void receive_data_deal(void)
{
    uint16_t crc16;
    uint8_t *buf = uartbuff[DEBUGSITE].recepetion_buff;
    uint16_t len = uartbuff[DEBUGSITE].recepetion_len;
    uint8_t addr;
    uint8_t func;
    uint8_t byte_count;
    uint8_t expected_byte_count;

    if (uartbuff[DEBUGSITE].recepetion_flag != 1U)
        return;
    uartbuff[DEBUGSITE].recepetion_flag = 0;

    if (g_transaction_pending == 0U || len < 4U)
        return;

    crc16 = (buf[len - 1U] << 8) | buf[len - 2U];
    if (CalcCrc16(buf, len - 2U) != crc16)
        return;

    addr = buf[0];
    func = buf[1];

    /* XR5000_UART5_EXCLUSIVE_FIX_20260730: only the owned request may consume a response. */
    if (addr != g_transaction_addr || addr == 0U || addr >= RS485_DETECT_MAX_DEVICES)
        return;

    if (func == 0x84U)
    {
        uint8_t was_type_detect = g_transaction_type_detect;
        g_transaction_pending = 0U; g_transaction_addr = 0U;
        g_transaction_type_detect = 0U; g_transaction_start_tick = 0U;
        if (was_type_detect != 0U)
            mark_identify_failure(addr, was_type_detect == RS485_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : was_type_detect == RS485_STAGE_PRODUCT ? DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE : DEVICE_IDENTIFY_SENSOR_READ_FAILED);
        else
        {
            if(g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD) g_devices[addr].disconnect_count++;
            check_and_record_fault(addr);
        }
        return;
    }

    if (func != 0x04U || len < 5U)
        return;

    byte_count = buf[2];
    if (len != (uint16_t)(byte_count + 5U))
    {
        /* A CRC-valid 0x04 frame from the polled device proves communication.
         * Do not convert an unexpected payload length into a false disconnect. */
        if(g_transaction_type_detect == 0U) complete_transaction(addr);
        return;
    }

    if (g_transaction_type_detect != 0U)
    {
        expected_byte_count = 2U;
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
        /* Keep the last valid sensor data, but release this normal polling
         * transaction without increasing disconnect_count. */
        if(g_transaction_type_detect == 0U) complete_transaction(addr);
        return;
    }

    if (g_transaction_type_detect != 0U)
    {
        uint16_t identify_value = (buf[3] << 8) | buf[4];
        if(g_transaction_type_detect == RS485_STAGE_NATIONAL)
        {
            g_devices[addr].national_type_code = identify_value;
            g_devices[addr].identify_stage = RS485_STAGE_PRODUCT;
            g_identify_fail_count[addr] = 0U;
            complete_transaction(addr);
        }
        else if(g_transaction_type_detect == RS485_STAGE_PRODUCT)
        {
            uint8_t detected_type = lookup_device_type(identify_value);
            if(detected_type == RS485_DETECT_TYPE_UNKNOWN)
                mark_identify_failure(addr, DEVICE_IDENTIFY_PRODUCT_UNKNOWN);
            else if(DeviceRegistry_IsNationalProductMatch(g_devices[addr].national_type_code, identify_value) == 0U)
                mark_identify_failure(addr, DeviceRegistry_IsNationalTypeKnown(g_devices[addr].national_type_code) != 0U ? DEVICE_IDENTIFY_CODE_MISMATCH : DEVICE_IDENTIFY_NATIONAL_UNKNOWN);
            else
            {
                g_devices[addr].product_code = identify_value;
                g_devices[addr].device_type = detected_type;
                g_identify_fail_count[addr] = 0U;
                if(DeviceRegistry_RequiresSensorMask(identify_value) != 0U)
                    g_devices[addr].identify_stage = RS485_STAGE_SENSOR;
                else
                {
                    g_devices[addr].sensor_enable = 0U;
                    g_devices[addr].sensor_enable_confirmed = 1U;
                    g_devices[addr].type_confirmed = 1U;
                    g_devices[addr].identify_stage = RS485_STAGE_COMPLETE;
                    DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP3, addr, DEVICE_IDENTIFY_OK);
                }
                complete_transaction(addr);
                return;
            }
            g_transaction_pending = 0U; g_transaction_addr = 0U; g_transaction_type_detect = 0U; g_transaction_start_tick = 0U;
        }
        else
        {
            if(DeviceRegistry_IsSensorMaskValid(g_devices[addr].product_code, identify_value) != 0U)
            {
                g_devices[addr].sensor_enable = identify_value;
                g_devices[addr].sensor_enable_confirmed = 1U;
                g_devices[addr].type_confirmed = 1U;
                g_devices[addr].identify_stage = RS485_STAGE_COMPLETE;
                g_identify_fail_count[addr] = 0U;
                DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP3, addr, DEVICE_IDENTIFY_OK);
                complete_transaction(addr);
            }
            else
            {
                mark_identify_failure(addr, DEVICE_IDENTIFY_SENSOR_TYPE_UNKNOWN);
                g_transaction_pending = 0U; g_transaction_addr = 0U; g_transaction_type_detect = 0U; g_transaction_start_tick = 0U;
            }
        }
    }
    else
    {
        uint8_t device_type = g_devices[addr].device_type;
        parse_sensor_data(addr, buf, device_type);
        g_devices[addr].sensor_data_valid = 1U; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
        complete_transaction(addr);
        check_and_record_fault(addr);
    }
}
/* 判断在线且有效(非断线状态) */
uint8_t RS485Detect_IsOnline(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return (g_devices[addr].online && g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD);
}

/* 判断传感器数据是否有效 */
uint8_t RS485Detect_HasSensorData(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_data_valid;
}

/* 判断传感器使能配置是否已确认(读取过0x000F寄存器) */
uint8_t RS485Detect_HasSensorEnableData(uint8_t addr)
{
    if (addr == 0 || addr >= RS485_DETECT_MAX_DEVICES)
        return 0;
    return g_devices[addr].sensor_enable_confirmed;
}

/* ============================================================
 * RTOS轮询任务: 10ms周期检查超时/轮询/接收处理, 耗时约10ms
 * ============================================================ */

void RS485DetectPollAndReceiveTask(void *parameter)
{
    uint32_t current_tick;

    (void)parameter;
    g_last_poll_time = osKernelGetTickCount();

    for (;;)
    {
        receive_data_deal();
        mark_transaction_timeout();

        current_tick = osKernelGetTickCount();
        if (g_transaction_pending == 0U &&
            (current_tick - g_last_poll_time) >= RS485_DETECT_POLL_INTERVAL_MS)
        {
            g_last_poll_time = current_tick;
            poll_next_device();
        }

        osDelay(RS485_DETECT_TASK_INTERVAL_MS);
    }
}

