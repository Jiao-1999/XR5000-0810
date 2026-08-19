#ifndef __BSP_RS485_DETECT_H
#define __BSP_RS485_DETECT_H

#include "main.h"

/* ============================================================================
 * 模块名称: RS485探测器管理模块 (RS485 Detector Management)
 * 功能描述: 管理回路3(UART5) RS485总线探测器(XR805/XR8303/XR8305)的轮询、
 *          数据解析、状态管理、Flash持久化。
 * 通信协议: Modbus RTU, 功能码04(读输入寄存器), 波特率115200/8N1
 * 轮询机制: FreeRTOS任务每150ms轮询一个在线设备, 先探测类型再读取传感器数据
 * 设备类型: XR805(烟雾+温度+CO+CH4+H2+VOC), XR8303(烟雾+温度+CO+H2+VOC+压力)
 * 掉线检测: 连续5次无响应判定掉线
 * 回路标识: 回路3, Flash存储地址0x10F000, 故障簇ID=0x53(83簇)
 * ============================================================================ */

/* -------------------- 可配置常量 -------------------- */
#define RS485_DETECT_MAX_DEVICES    34    /* 最大设备数(地址1~32, 预留扩展) */
#define RS485_DETECT_DISCONNECT_THRESHOLD  5  /* 连续无响应次数阈值, 超过判定掉线 */
#define RS485_DETECT_POLL_INTERVAL_MS      150 /* 轮询间隔(ms), 每轮一个设备 */
#define RS485_DETECT_RESPONSE_TIMEOUT_MS   120 /* 单次请求超时(ms), 超时计掉线 */
#define RS485_DETECT_TASK_INTERVAL_MS        10 /* 任务循环周期(ms) */
#define RS485_DETECT_TX_TIMEOUT_MS           30 /* 阻塞发送超时(ms) */

#define RS485_DETECT_LOOP_ID  3  /* 回路编号, 用于Flash存储和故障记录 */
#define RS485_DETECT_FLASH_ID 0x53U /* 故障簇ID(83簇), 避免与旧簇3历史冲突 */

/* Flash存储地址（在线状态表持久化） */
#define RS485_DETECT_FLASH_ADDR  0x10F000UL

/* 类型探测寄存器地址(04功能码读取) */
#define RS485_DETECT_TYPE_REG    0x000E  /* 产品型号寄存器 */
#define RS485_DETECT_SENSOR_ENABLE_REG  0x000F  /* 传感器启用位掩码寄存器 */

/* -------------------- 设备类型枚举 -------------------- */
typedef enum {
    RS485_DETECT_TYPE_UNKNOWN = 0,  /* 未知/未探测 */
    RS485_DETECT_TYPE_XR805   = 1,  /* XR805: 烟雾+温度+CO+CH4+H2+VOC */
    RS485_DETECT_TYPE_XR8303  = 2,  /* XR8303: 烟雾+温度+CO+H2+VOC+压力 */
    RS485_DETECT_TYPE_XR8305  = 3,  /* XR8305: 与XR8303同布局 */
} RS485DetectDeviceType;

/* -------------------- 传感器索引(统一对外) -------------------- */
typedef enum {
    RS485_SENSOR_TEMPERATURE = 0,  /* 温度 */
    RS485_SENSOR_SMOKE       = 1,  /* 烟雾 */
    RS485_SENSOR_CO          = 2,  /* 一氧化碳 */
    RS485_SENSOR_H2          = 3,  /* 氢气 */
    RS485_SENSOR_VOC         = 4,  /* 挥发性有机物 */
    RS485_SENSOR_CH4         = 5,  /* 甲烷(仅XR805) */
    RS485_SENSOR_PRESSURE    = 6,  /* 压力(仅XR8303/XR8305) */
    RS485_SENSOR_COUNT             /* 传感器总数 */
} RS485SensorIndex;

/* -------------------- 类型码映射条目(产品型号→设备类型) -------------------- */
typedef struct {
    uint8_t  product_code;   /* 0x000E寄存器读取的产品型号值 */
    uint8_t  device_type;    /* 映射到的RS485DetectDeviceType */
} RS485TypeMapEntry;

/* -------------------- 传感器寄存器定义(寄存器地址→响应帧偏移→传感器索引) -------------------- */
typedef struct {
    uint16_t reg_addr;       /* 寄存器地址 */
    uint8_t  byte_offset;    /* 响应帧中的字节偏移 */
    uint8_t  sensor_index;   /* 对应RS485SensorIndex */
    uint8_t  is_value;       /* 1=传感器数值, 0=传感器状态 */
} RS485SensorRegDef;

/* -------------------- 设备实例(每个在线设备维护一份) -------------------- */
typedef struct {
    uint8_t  online;              /* 在线标志(屏幕下发设置) */
    uint8_t  device_type;         /* 设备类型(RS485DetectDeviceType) */
    uint8_t  type_confirmed;      /* 类型已确认(已成功读取0x000E) */
    uint8_t  sensor_enable_confirmed; /* 传感器启用状态已确认(已成功读取0x000F) */
    uint8_t  sensor_data_valid;      /* 传感器实时数据有效(已收到完整传感器帧) */
    uint8_t  disconnect_count;       /* 掉线累计计数(连续无响应次数) */
    uint8_t  disconnect_memory;      /* 掉线记忆(0=在线, 1=已记录掉线) */
    uint16_t sensor_enable;          /* 0x000F传感器启用位掩码 */
    uint16_t sensor_values[RS485_SENSOR_COUNT];  /* 传感器数值(温度/烟雾/CO等) */
    uint8_t  sensor_states[RS485_SENSOR_COUNT];  /* 传感器状态(0=正常/1=预警/2=报警/...) */
    uint8_t  alarm_memory[RS485_SENSOR_COUNT];   /* 报警记忆(0=正常, 1=已记录报警) */
} RS485DetectDevice;

/* ==================== 对外接口 ==================== */

/* ---- 初始化 ---- */
void RS485Detect_Init(void);

/* ---- 在线状态设置(屏幕下发) ---- */
void RS485Detect_SetOnline(uint8_t addr, uint8_t state);          /* 设置单个设备在线状态 */
void RS485Detect_SetOnlineRange(uint8_t start, uint8_t end, uint8_t state); /* 批量设置在线状态 */

/* ---- 状态查询 ---- */
uint8_t  RS485Detect_GetOnline(uint8_t addr);                     /* 获取在线标志(屏幕下发值) */
uint8_t  RS485Detect_GetType(uint8_t addr);                       /* 获取设备类型 */
uint16_t RS485Detect_GetSensorValue(uint8_t addr, uint8_t sensor_idx); /* 获取传感器数值 */
int16_t  RS485Detect_GetTemperature(uint8_t addr);                /* 获取温度值(有符号) */
uint16_t RS485Detect_GetSensorEnable(uint8_t addr);               /* 获取传感器启用位掩码 */
uint8_t  RS485Detect_GetSensorState(uint8_t addr, uint8_t sensor_idx); /* 获取传感器状态 */
void     RS485Detect_InjectSensorState(uint8_t addr, uint8_t sensor_idx, uint8_t state); /* 注入传感器状态(仅测试用) */
uint8_t  RS485Detect_IsDisconnected(uint8_t addr);                /* 判断是否掉线(计数>=阈值) */
uint8_t  RS485Detect_GetOnlineCount(void);                        /* 回路3在线设备总数 */
uint8_t  RS485Detect_GetDisconnectCount(void);                    /* 回路3掉线设备总数 */
uint8_t  RS485Detect_GetAlarmCount(void);                         /* 回路3报警设备总数 */
uint8_t  RS485Detect_IsAlarmState(uint8_t device_type, uint8_t sensor_idx, uint8_t state);  /* 判断传感器状态是否为报警 */
uint8_t  RS485Detect_IsFaultState(uint8_t device_type, uint8_t sensor_idx, uint8_t state);  /* 判断传感器状态是否为故障 */

/* ---- 便捷查询 ---- */
uint16_t RS485Detect_GetSensorEnable(uint8_t addr);               /* 获取传感器启用位掩码 */
uint8_t  RS485Detect_IsOnline(uint8_t addr);                      /* 判断是否真正在线(上线且未掉线) */
uint8_t  RS485Detect_HasSensorData(uint8_t addr);                 /* 是否已收到有效传感器数据 */
uint8_t  RS485Detect_HasSensorEnableData(uint8_t addr);           /* 是否已确认传感器启用状态 */

/* ---- Flash持久化 ---- */
void RS485Detect_SaveOnlineState(void);  /* 将在线状态表保存到Flash(0x10F000) */
void RS485Detect_LoadOnlineState(void);  /* 从Flash加载在线状态表 */

/* ---- RTOS轮询任务 ---- */
void RS485DetectPollAndReceiveTask(void *parameter); /* 回路3轮询任务(发送查询→接收解析→超时检测) */

#endif

