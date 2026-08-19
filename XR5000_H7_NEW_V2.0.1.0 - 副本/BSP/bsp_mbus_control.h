#ifndef __BSP_MBUS_CONTROL_H
#define __BSP_MBUS_CONTROL_H

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ============================================================================
 * 模块名称: MBus/回路2设备控制模块 (MBus Control Module)
 * 功能描述: 管理回路2(UART2) MBus总线设备(声光报警器/手动报警器/火灾显示盘)的
 *          轮询调度、Modbus RTU通信、状态管理、Flash持久化。
 * 通信协议: Modbus RTU, 功能码04(读输入寄存器)/05(写单线圈)/10(写多寄存器),
 *          UART2/115200/8N1, MBUS2SITE=1
 * 轮询机制: FreeRTOS任务每200ms(10×20ms)轮询一个在线设备, 地址范围1~63
 * 设备类型: 声光报警器(XR-SGBJQ,地址60)/手动报警器(XR2200,地址61)/火灾显示盘(XR1530,地址62)
 * 掉线检测: 连续10次无响应判定掉线
 * 回路标识: 回路2, Flash存储地址0x110000, 故障簇ID=0x52(82簇)
 * ============================================================================ */

/* -------------------- 可配置常量 -------------------- */
#define MBUS_CONTROL_MAX_DEVICES         64    /* 最大设备数(地址1~63, 预留扩展) */
#define MBUS_CONTROL_DISCONNECT_THRESHOLD 10   /* 连续无响应次数阈值, 超过判定掉线 */
#define MBUS_CONTROL_FLASH_ADDR          0x110000UL /* Flash在线状态存储地址 */
#define MBUS_CONTROL_LOOP_ID             2     /* 回路编号, 用于故障记录显示 */
#define MBUS_CONTROL_FLASH_ID            0x52U /* 故障簇ID(82簇) */

/* 回路2固定设备地址 */
#define MBUS_CONTROL_SOUND_LIGHT_ADDR    60U   /* 声光报警器(XR-SGBJQ)地址 */
#define MBUS_CONTROL_MANUAL_ALARM_ADDR   61U   /* 手动报警器(XR2200)地址 */
#define MBUS_CONTROL_FIRE_DISPLAY_DEFAULT_ADDR 62U /* 火灾显示盘(XR1530)地址 */

/* 火灾显示盘探测器类型码(用于10功能码事件上报) */
#define MBUS_FIRE_DISPLAY_DETECT_TEMP          1U  /* 温度探测器 */
#define MBUS_FIRE_DISPLAY_DETECT_SMOKE         2U  /* 烟雾探测器 */
#define MBUS_FIRE_DISPLAY_DETECT_CO            3U  /* CO探测器 */
#define MBUS_FIRE_DISPLAY_DETECT_H2            4U  /* H2探测器 */
#define MBUS_FIRE_DISPLAY_DETECT_MANUAL        5U  /* 手动报警按钮 */

/* 火灾显示盘报警类型 */
#define MBUS_FIRE_DISPLAY_ALARM_FIRE           1U  /* 火警 */
#define MBUS_FIRE_DISPLAY_ALARM_FAULT          2U  /* 故障 */

/* -------------------- 设备类型枚举 -------------------- */
typedef enum {
    MBUS_CONTROL_DEV_UNKNOWN = 0,        /* 未知类型 */
    MBUS_CONTROL_DEV_SGBJQ   = 1,        /* XR-SGBJQ 声光报警器 */
    MBUS_CONTROL_DEV_XR2200  = 2,        /* XR2200 手动报警器 */
    MBUS_CONTROL_DEV_FIRE_DISPLAY = 3,   /* XR1530 火灾显示盘 */
} MBusCtrlDevType;

/* -------------------- 设备实例(每个地址维护一份) -------------------- */
typedef struct {
    uint8_t online;              /* 在线标志(屏幕下发设置) */
    uint8_t disconnect_count;    /* 掉线累计计数(连续无响应次数) */
    uint8_t dev_type;            /* 设备类型(MBusCtrlDevType) */
    uint8_t sensor_state;        /* 传感器状态(04功能码读取的寄存器值) */
    uint8_t disconnect_memory;   /* 掉线记忆(0=在线, 1=已记录掉线) */
} MBusCtrlDevice;

/* FreeRTOS消息队列句柄(MBus2轮询数据发送队列) */
extern QueueHandle_t xMBus2QueueHandle;

/* ==================== 对外接口 ==================== */

/* ---- 初始化 ---- */
void MBusCtrl_Init(void);

/* ---- 在线状态设置(屏幕下发) ---- */
void MBusCtrl_SetOnline(uint8_t addr, uint8_t state);                    /* 设置单个设备在线状态 */
void MBusCtrl_SetOnlineRange(uint8_t start, uint8_t end, uint8_t state); /* 批量设置在线状态(start~end) */

/* ---- 状态查询 ---- */
uint8_t MBusCtrl_GetOnline(uint8_t addr);            /* 获取在线标志(屏幕下发值) */
uint8_t MBusCtrl_IsDisconnected(uint8_t addr);       /* 判断是否掉线(计数>=阈值) */
uint8_t MBusCtrl_GetOnlineCount(void);               /* 回路2在线设备总数 */
uint8_t MBusCtrl_GetDisconnectCount(void);           /* 回路2掉线设备总数 */
uint8_t MBusCtrl_GetAlarmCount(void);                /* 回路2报警设备总数 */

/* ---- 设备信息查询 ---- */
const char* MBusCtrl_GetDeviceName(uint8_t addr);    /* 根据地址获取设备名称(中文) */
uint8_t MBusCtrl_GetDeviceState(uint8_t addr);       /* 获取设备传感器状态值 */
void MBusCtrl_InjectSensorState(uint8_t addr, uint8_t state);       /* 注入传感器状态(仅测试用) */
uint8_t MBusCtrl_GetDeviceType(uint8_t addr);        /* 获取设备类型(MBusCtrlDevType) */
uint8_t MBusCtrl_IsAlarmState(uint8_t addr);         /* 判断设备是否处于报警状态 */

/* ---- 火灾显示盘事件上报(10功能码写多寄存器) ---- */
uint8_t MBusCtrl_PostFireDisplayEvent(uint8_t loop, uint8_t addr, uint8_t detector_type, uint8_t alarm_type);

/* ---- Flash持久化 ---- */
void MBusCtrl_SaveOnlineState(void);                 /* 将在线状态表保存到Flash(0x110000) */
void MBusCtrl_LoadOnlineState(void);                 /* 从Flash加载在线状态表 */

/* ---- FreeRTOS消息队列接口 ---- */
int8_t SendDataToMBus2Queue(uint8_t *buf, uint8_t buf_len);  /* 将Modbus帧发送到MBus2队列 */
int8_t ReceiveDataFromMBus2Queue(uint8_t *buf);              /* 从MBus2队列接收数据 */

/* ---- RTOS轮询任务 ---- */
void MBusControlPollSlaveAndReceiveTask(void* parameter);    /* 回路2轮询任务(发送查询→接收解析→控制服务) */

#endif

