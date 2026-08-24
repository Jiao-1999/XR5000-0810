/**
 * @file    bsp_storage_tx.h
 * @brief   主控存储发送模块 - 通过LPUART1(PB6/PB7)向存储侧单片机发送事件记录
 *
 * @details
 *   硬件接口:
 *     主控(基于STM32H723)通过LPUART1串口与存储侧单片机通信,
 *     存储侧单片机负责管理SPI Flash W25Q256, 可选通过USB上报.
 *
 *   通信参数:
 *     LPUART1 PB6=TX, PB7=RX, 115200 8N1
 *     本模块自行初始化LPUART1, 不依赖CubeMX配置
 *
 *   帧格式(主控->存储侧):
 *     [帧头0xA5][长度][命令码][数据载荷][CRC16低][CRC16高][帧尾0x5A]
 *     长度 = 命令码字节数(1) + 数据载荷字节数
 *     CRC16计算范围: 从长度字节到最后一字节载荷, 多项式0xA001, 小端存储
 *
 *   事件记录格式(符合GB4717-2024附录B表B.2):
 *     共17字节: 控制器号+单元+设备+通道+设备类型+事件+状态+年月日时分秒
 *
 *   存储命令码:
 *     0x01=存储事件, 0x02=首警, 0x03=火警, 0x04=故障
 *     (首警/火警/故障由存储侧独立区段管理, 互不覆盖)
 *
 *   关键API:
 *     StorageTx_QueueRecord() 中断/任务安全(非阻塞入队)
 *     StorageTx_TaskLoop() 专用FreeRTOS任务(阻塞出队发送)
 *     队列深度STX_QUEUE_DEPTH, 满则丢最旧
 */
#ifndef __BSP_STORAGE_TX_H
#define __BSP_STORAGE_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/*==============================================================
 * 协议常量定义
 *============================================================*/
#define STX_FRAME_HEAD      0xA5    /* 帧头 */
#define STX_FRAME_TAIL      0x5A    /* 帧尾 */
#define STX_MAX_PAYLOAD     255     /* 最大数据载荷长度 */
#define STX_TIMEOUT_MS      100     /* 应答超时(ms) */
#define STX_RETRY_COUNT     3       /* 发送失败重试次数 */
#define STX_QUEUE_DEPTH     32      /* 发送队列深度(条数) */

/* 存储命令码 */
#define STX_CMD_STORE_EVENT        0x01    /* 存储事件(普通) */
#define STX_CMD_STORE_FIRST_ALARM  0x02    /* 存储首警(独立区段) */
#define STX_CMD_STORE_FIRE_ALARM   0x03    /* 存储火警(独立区段) */
#define STX_CMD_STORE_FAULT        0x04    /* 存储故障(独立区段) */
#define STX_CMD_QUERY_CAPACITY     0x05    /* 查询剩余容量 */
#define STX_CMD_HEARTBEAT          0x06    /* 心跳 */

/* 应答码 */
#define STX_ACK_OK          0x00    /* 成功 */
#define STX_ACK_ERR_CRC     0x01    /* 校验错误 */
#define STX_ACK_ERR_FULL    0x02    /* 存储已满 */
#define STX_ACK_ERR_BUSY    0x03    /* 忙 */

/*==============================================================
 * 事件记录结构 - 符合GB4717-2024附录B表B.2
 *============================================================*/
#pragma pack(push, 1)
/* 事件记录结构体 共17字节(紧凑打包, 字段顺序与协议一致) */
typedef struct {
    uint16_t controller_no;   /* 控制器号         2字节(小端) */
    uint8_t  unit_no;         /* 单元号           1字节 */
    uint8_t  device_no;       /* 设备号           1字节 */
    uint8_t  channel_no;      /* 通道号           1字节 */
    uint16_t dev_type;        /* 设备类型代码     2字节(小端, 见附录C.16) */
    uint16_t event_code;      /* 事件代码         2字节(小端, 见附录C.17) */
    uint16_t state_code;      /* 状态代码         2字节(小端, 见附录C.18) */
    uint8_t  year;            /* 年=完整年份-2000 1字节 */
    uint8_t  month;           /* 月               1字节 */
    uint8_t  day;             /* 日               1字节 */
    uint8_t  hour;            /* 时               1字节 */
    uint8_t  minute;          /* 分               1字节 */
    uint8_t  second;          /* 秒               1字节 */
} EventRecord_t;  /* 共17字节 */
#pragma pack(pop)

/*==============================================================
 * 设备类型代码(GB4717-2024附录C表C.16)
 *============================================================*/
#define DEV_TYPE_CONTROLLER     1     /* 控制器 */
#define DEV_TYPE_SOUND_LIGHT    17    /* 声光报警器 */
#define DEV_TYPE_SMOKE          21    /* 感烟探测器 */
#define DEV_TYPE_TEMPERATURE    31    /* 感温探测器 */
#define DEV_TYPE_CO             53    /* 一氧化碳探测器 */
#define DEV_TYPE_HAND_REPORT    61    /* 手动报警按钮 */
#define DEV_TYPE_H2             55    /* 氢气探测器 */
#define DEV_TYPE_FIRE_ALARM     82    /* 火灾报警器件 */
#define DEV_TYPE_CONTROL_DEV    163   /* 控制设备 */
#define DEV_TYPE_STORAGE        18    /* 运行数据存储单元(表C.16, 存储故障上报用) */
#define DEV_TYPE_MULTI_SENSOR   50    /* 多传感复合探测器(表C.16) */

/*==============================================================
 * 事件代码(GB4717-2024附录C表C.17)
 *============================================================*/
#define EVT_NORMAL              1     /* 正常 */
#define EVT_FIRST_FIRE          2     /* 首警 */
#define EVT_FIRE                3     /* 火警 */
#define EVT_GAS_LOW             5     /* 气体低报 */
#define EVT_GAS_HIGH            6     /* 气体高报 */
#define EVT_START               19    /* 启动 */
#define EVT_FEEDBACK            26    /* 反馈 */
#define EVT_STOP                29    /* 停动 */
#define EVT_SHIELD              72    /* 屏蔽 */
#define EVT_SHIELD_RELEASE      73    /* 解除屏蔽 */
#define EVT_FAULT               80    /* 故障 */
#define EVT_FAULT_RECOVER       100   /* 故障恢复 */
#define EVT_MANUAL              125   /* 手动 */
#define EVT_AUTO                126   /* 自动 */
/* 以下为GB4717-2024表C.17补充事件代码(P1-1整改, 2026-08-24) */
#define EVT_SUPERVISED          70    /* 监管 */
#define EVT_SUPERVISED_RELEASE  71    /* 监管解除 */
#define EVT_POWER_ON            120   /* 开机 */
#define EVT_POWER_OFF           121   /* 关机 */
#define EVT_RESET               122   /* 复位 */
#define EVT_SELF_CHECK          123   /* 自检 */
#define EVT_SELF_CHECK_FAIL     124   /* 自检失败 */
#define EVT_CONFIRM_BUTTON      128   /* 信息确认按钮动作 */
#define EVT_CHECK_BUTTON        129   /* 检查功能按钮动作 */
#define EVT_LINKAGE_START_BUTTON 130  /* 联动启动按钮动作 */
#define EVT_CLOCK_ADJUST        131   /* 调整时钟 */

/*==============================================================
 * API声明
 *============================================================*/

/**
 * @brief  初始化存储发送模块
 * @note   本函数初始化LPUART1(PB6=TX, PB7=RX, 115200 8N1)及相关GPIO,
 *         本模块自行配置LPUART1, 不依赖CubeMX, 完全使用HAL寄存器操作;
 *         同时创建FreeRTOS发送队列(失败不入队).
 *         可重复调用, 已初始化则直接返回.
 */
void StorageTx_Init(void);

/**
 * @brief  同步发送一条事件记录到存储侧(阻塞)
 * @param  cmd: 命令码(STX_CMD_STORE_xxx)
 * @param  record: 17字节事件记录指针
 * @retval 0=成功, 1=无应答/超时, 2=CRC错误(会重试), 3=存储已满, 4=忙
 * @note   内含3次重试, 每次重试前清空RX残留并重新组帧发送
 */
uint8_t StorageTx_SendRecord(uint8_t cmd, const EventRecord_t *record);

/**
 * @brief  异步入队一条事件记录(非阻塞)
 * @param  cmd: 命令码(STX_CMD_STORE_xxx)
 * @param  record: 17字节事件记录指针
 * @retval 0=入队成功, 1=队列满(已丢最旧重试), 2=未初始化
 * @note   适合中断/任务调用, 真正发送由StorageTxTask完成
 */
uint8_t StorageTx_QueueRecord(uint8_t cmd, const EventRecord_t *record);

/**
 * @brief  存储发送任务主循环(需在专用FreeRTOS任务中调用)
 * @note   阻塞等待队列, 有记录则发送; 发送完短暂让出CPU
 */
void StorageTx_TaskLoop(void);

/**
 * @brief  查询存储侧剩余容量
 * @param  remaining: 输出剩余容量条数
 * @retval 0=成功, 非0=失败(已重试3次)
 */
uint8_t StorageTx_QueryCapacity(uint32_t *remaining);

/**
 * @brief  填充记录时间戳(读RTC)
 * @param  rec: 待填充的记录
 */
void StorageTx_FillTimestamp(EventRecord_t *rec);

/**
 * @brief  构造完整事件记录结构
 * @param  rec: 待填充记录
 * @param  dev_no: 设备号
 * @param  dev_type: 设备类型代码
 * @param  event_code: 事件代码
 * @param  state_code: 状态代码
 */
void StorageTx_BuildRecord(EventRecord_t *rec,
                           uint8_t dev_no,
                           uint16_t dev_type,
                           uint16_t event_code,
                           uint16_t state_code);

/**
 * @brief  发送心跳到存储侧
 * @retval 0=成功
 */
uint8_t StorageTx_Heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STORAGE_TX_H */
