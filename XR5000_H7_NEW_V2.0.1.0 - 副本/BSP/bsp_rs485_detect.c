/* ============================================================================
 * 模块名称: RS485探测器管理模块 (RS485 Detector Management)
 * 功能描述: 实现回路3(UART5) RS485总线探测器的轮询调度、Modbus RTU通信、
 *          传感器数据解析、设备状态管理、掉线/报警检测、Flash持久化。
 * 通信协议: Modbus RTU, 功能码04(读输入寄存器), UART5/115200/8N1
 * 轮询流程: 先探测设备类型(读0x000E/0x000F), 确认后读取传感器数据
 *           (XR805读16个寄存器, XR8303/XR8305读12个寄存器)
 * 故障记录: 掉线/报警由本模块检测, 故障记录由cmd_process.c统一处理
 * 回路标识: 回路3, Flash存储地址0x10F000, 故障簇ID=0x53(83簇)
 * ============================================================================ */

#include "bsp_rs485_detect.h"

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
static const RS485TypeMapEntry type_map[] = {
    {1, RS485_DETECT_TYPE_XR805},
    {2, RS485_DETECT_TYPE_XR805},
    {3, RS485_DETECT_TYPE_XR805},
    {7, RS485_DETECT_TYPE_XR8303},
    {8, RS485_DETECT_TYPE_XR8305},
};
#define TYPE_MAP_SIZE  (sizeof(type_map) / sizeof(type_map[0]))

/* XR805传感器寄存器布局: 04功能码读取16个寄存器(0x0000~0x000F), 响应帧含温度/烟雾/CH4/CO/VOC/H2的数值和状态 */
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

/* XR8303/XR8305传感器寄存器布局: 04功能码读取12个寄存器(0x0000~0x000B), 响应帧含温度/烟雾/CO/H2/VOC/压力的数值和状态 */
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
 * 第二层: 全局设备实例表与轮询状态
 * ============================================================ */

static RS485DetectDevice g_devices[RS485_DETECT_MAX_DEVICES]; /* 设备实例数组(索引=地址) */

static uint8_t g_online_count = 0;       /* 当前上线设备数量 */
static uint8_t g_poll_current_addr = 1;  /* 当前轮询地址(1~33循环) */
static uint32_t g_last_poll_time = 0;    /* 上次轮询的系统tick */

/* UART5事务锁: 同一时刻仅允许一个请求占用UART5 */
static uint8_t g_transaction_pending = 0;       /* 是否有进行中的事务 */
static uint8_t g_transaction_addr = 0;          /* 当前事务的目标地址 */
static uint8_t g_transaction_type_detect = 0;   /* 当前事务是否为类型探测(1=类型探测, 0=传感器数据) */
static uint32_t g_transaction_start_tick = 0;   /* 事务启动时的系统tick(用于超时判断) */

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/* 根据0x000E寄存器产品型号值查找对应的设备类型枚举 */
static uint8_t lookup_device_type(uint8_t product_code)
{
    for (uint8_t i = 0; i < TYPE_MAP_SIZE; i++)
    {
        if (type_map[i].product_code == product_code)
        {
            return type_map[i].device_type;
        }
    }
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

/* 根据设备类型返回04功能码需读取的寄存器数量(XR805=16, XR8303/XR8305=12) */
static uint8_t get_register_count(uint8_t device_type)
{
    if (device_type == RS485_DETECT_TYPE_XR805)
        return 16;  /* 0x0000 ~ 0x000F */
    else if (device_type == RS485_DETECT_TYPE_XR8303 || device_type == RS485_DETECT_TYPE_XR8305)
        return 12;  /* 0x0000 ~ 0x000B */
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
        g_devices[addr].disconnect_memory = 0;
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

/* 获取指定传感器的数值(温度/烟雾/CO等) */
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

/* 判断传感器状态是否为报警: XR805(state=1/2), XR8303/XR8305(温度/烟雾/CO/H2/VOC各有不同阈值) */
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

/* 判断传感器状态是否为故障: XR805(state=9), XR8303/XR8305(温度state=3/烟雾state=8) */
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
/* 统计回路3当前报警设备数(遍历在线设备, 任一传感器报警即计数) */
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
 * 轮询层: Modbus RTU请求构建与发送
 * ============================================================ */

/* 构建传感器数据读取Modbus帧(04功能码, 读reg_count个寄存器) */
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

/* 构建类型探测Modbus帧(04功能码, 读0x000E和0x000F共2个寄存器) */
static void build_type_detect_cmd(uint8_t *buf, uint8_t addr)
{
    uint16_t crc16;

    buf[0] = addr;
    buf[1] = 0x04;
    buf[2] = (RS485_DETECT_TYPE_REG >> 8) & 0xFF;
    buf[3] = RS485_DETECT_TYPE_REG & 0xFF;
    buf[4] = 0x00;
    buf[5] = 0x02; /* 读2个寄存器（0x000E + 0x000F） */

    crc16 = CalcCrc16(buf, 6);
    buf[6] = crc16 & 0xFF;
    buf[7] = crc16 >> 8;
}

static void check_and_record_fault(uint8_t addr); /* 前向声明: 掉线/报警检测 */

/* 检查当前事务是否超时: 超时则计掉线+触发故障检测 */
static void mark_transaction_timeout(void)
{
    uint8_t addr;

    if (g_transaction_pending == 0U)
        return;

    if ((osKernelGetTickCount() - g_transaction_start_tick) < RS485_DETECT_RESPONSE_TIMEOUT_MS)
        return;

    addr = g_transaction_addr;
    g_transaction_pending = 0;
    g_transaction_addr = 0;
    g_transaction_type_detect = 0;

    if (addr > 0U && addr < RS485_DETECT_MAX_DEVICES && g_devices[addr].online != 0U)
    {
        if (g_devices[addr].disconnect_count < RS485_DETECT_DISCONNECT_THRESHOLD)
        {
            g_devices[addr].disconnect_count++;
        }
        check_and_record_fault(addr);
    }
}

/* 轮询下一个在线设备: 未确认类型则发类型探测帧, 已确认则发传感器数据帧 */
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

        if (g_devices[g_poll_current_addr].online != 0U)
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
    type_detect = (g_devices[addr].type_confirmed == 0U) ? 1U : 0U;

    if (type_detect != 0U)
    {
        build_type_detect_cmd(modbusbuf, addr);
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

    /* 报警检测（温度、烟雾、CO、H2、CH4、VOC） */
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

/* 完成当前事务: 清零掉线计数, 释放UART5事务锁 */
static void complete_transaction(uint8_t addr)
{
    g_devices[addr].disconnect_count = 0;
    g_transaction_pending = 0;
    g_transaction_addr = 0;
    g_transaction_type_detect = 0;
    g_transaction_start_tick = 0;
}

/* 接收数据处理: 校验CRC→解析响应帧→类型探测或传感器数据分发→完成事务 */
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
        complete_transaction(addr);

        if (was_type_detect != 0U)
        {
            g_devices[addr].device_type = RS485_DETECT_TYPE_XR8303;
            g_devices[addr].type_confirmed = 1;
        }
        check_and_record_fault(addr);
        return;
    }

    if (func != 0x04U || len < 5U)
        return;

    byte_count = buf[2];
    if (len != (uint16_t)(byte_count + 5U))
        return;

    if (g_transaction_type_detect != 0U)
    {
        expected_byte_count = 4U;
    }
    else
    {
        uint8_t device_type = g_devices[addr].device_type;
        if (device_type == RS485_DETECT_TYPE_UNKNOWN)
            return;
        expected_byte_count = get_register_count(device_type) * 2U;
    }

    if (byte_count != expected_byte_count)
        return;

    if (g_transaction_type_detect != 0U)
    {
        uint16_t product_code = (buf[3] << 8) | buf[4];
        uint8_t detected_type = lookup_device_type((uint8_t)product_code);

        if (detected_type != RS485_DETECT_TYPE_UNKNOWN)
        {
            g_devices[addr].device_type = detected_type;
            g_devices[addr].type_confirmed = 1;
        }
        g_devices[addr].sensor_enable = (buf[5] << 8) | buf[6];
        g_devices[addr].sensor_enable_confirmed = 1;
        complete_transaction(addr);
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

