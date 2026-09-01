/**
 * @file    bsp_storage_rx.h
 * @brief   存储接收模块头文件 - 定义数据结构和函数接口
 * @details 通信接口: USART1 (PA9=TX/PA10=RX, 115200 8N1, 中断接收)
 *          存储介质: W25Q256 SPI Flash (32MB, 基地址0x00000000, 掉电保存)
 *          核心功能: 接收主机侧事件记录帧, CRC校验后写FLASH+读回校验, 回复ACK
 *
 *   帧格式(主机->存储侧):
 *     [0xA5][长度][命令码][17字节EventRecord_t][CRC16低][CRC16高][0x5A]
 *     长度   = 命令码(1) + 负载长度
 *     CRC16校验范围: 长度 + 命令码 + 负载, 多项式0xA001, 低位在前
 *
 *   帧格式(存储侧->主机):
 *     [0xA5][长度][命令码+应答数据][应答码][CRC16低][CRC16高][0x5A]
 *     应答码: 0x00=成功 0x01=CRC错误 0x02=存储满 0x03=忙 0x04=读回校验错
 *
 *   命令列表:
 *     0x01=存储事件, 0x02=存储首警, 0x03=存储火警, 0x04=存储故障
 *     0x05=查询容量, 0x06=心跳
 *
 *   EventRecord_t (17字节, #pragma pack(1)):
 *     偏移0:  controller_no(2) + unit_no(1) + device_no(1) + channel_no(1)
 *     偏移5:  dev_type(2) + event_code(2) + state_code(2)
 *     偏移11: year(1) + month(1) + day(1) + hour(1) + minute(1) + second(1) = 17字节
 *     event_code取值: 2=火警, 3=故障 (参考GB4717事件编码)
 */
#ifndef __BSP_STORAGE_RX_H
#define __BSP_STORAGE_RX_H

#include "sys.h"

/*==============================================================
 * 帧格式常量
 *============================================================*/
#define STX_FRAME_HEAD      0xA5    /* 帧头 */
#define STX_FRAME_TAIL      0x5A    /* 帧尾 */
#define STX_MAX_PAYLOAD     255     /* 负载最大长度 */
#define STX_TIMEOUT_MS      100     /* 接收超时(ms) */

/* 命令码 */
#define STX_CMD_STORE_EVENT        0x01    /* 存储通用事件 */
#define STX_CMD_STORE_FIRST_ALARM  0x02    /* 存储首警 */
#define STX_CMD_STORE_FIRE_ALARM   0x03    /* 存储火警 */
#define STX_CMD_STORE_FAULT        0x04    /* 存储故障 */
#define STX_CMD_QUERY_CAPACITY     0x05    /* 查询剩余容量 */
#define STX_CMD_HEARTBEAT          0x06    /* 心跳 */
#define STX_CMD_TEST_LOG           0x07    /* 测试日志透传(不写Flash, USB转发) */

/* 应答码 */
#define STX_ACK_OK          0x00    /* 成功 */
#define STX_ACK_ERR_CRC     0x01    /* CRC校验失败 */
#define STX_ACK_ERR_FULL    0x02    /* 存储已满 */
#define STX_ACK_ERR_BUSY    0x03    /* 忙 */
#define STX_ACK_ERR_VERIFY  0x04    /* 读回校验失败 */

/*==============================================================
 * 事件记录结构体 (17字节, 参考GB4717-2024附录B)
 *============================================================*/
#pragma pack(push, 1)
typedef struct {
    uint16_t controller_no;   /* 控制器编号       2字节 */
    uint8_t  unit_no;         /* 部件编号         1字节 */
    uint8_t  device_no;       /* 设备编号         1字节 */
    uint8_t  channel_no;      /* 通道编号         1字节 */
    uint16_t dev_type;        /* 设备类型编码     2字节 */
    uint16_t event_code;      /* 事件类型编码     2字节 */
    uint16_t state_code;      /* 状态信息编码     2字节 */
    uint8_t  year;            /* 年=实际年份-2000 1字节 */
    uint8_t  month;           /* 月               1字节 */
    uint8_t  day;             /* 日               1字节 */
    uint8_t  hour;            /* 时               1字节 */
    uint8_t  minute;          /* 分               1字节 */
    uint8_t  second;          /* 秒               1字节 */
} EventRecord_t;  /* 共17字节 */
#pragma pack(pop)

/*==============================================================
 * API函数声明
 *============================================================*/

/**
 * @brief  初始化存储模块 (初始化W25Q256, 恢复写指针)
 * @retval 0=成功, 1=W25Q256初始化失败
 */
/*==============================================================
 * 分区定义(P0-1/P0-2整改: 4类记录独立分区环形FIFO, 2026-08-24)
 * W25Q256 32MB布局:
 *   首警区 1MB(0x000000) + 火警区 2MB(0x100000) + 故障区 2MB(0x300000)
 *   + 通用区 约27MB(0x500000) + 元数据区 64KB(0x1FF0000, A/B双库指针持久化)
 * 各区写满后环形覆盖最旧记录, 互不侵占(GB4717 B.1.2.2)
 *============================================================*/
#define STX_ZONE_FIRST     0     /* 首警区(独立, 命令0x02) */
#define STX_ZONE_FIRE      1     /* 火警区(独立, 命令0x03) */
#define STX_ZONE_FAULT     2     /* 故障区(独立, 命令0x04) */
#define STX_ZONE_GENERAL   3     /* 通用区(其他事件, 命令0x01) */
#define STX_ZONE_COUNT     4     /* 分区总数 */

uint8_t StorageRx_Init(void);

/**
 * @brief  USART1接收处理函数 (由USART1中断逐字节调用)
 * @param  data: 接收到的字节
 * @note   内部状态机接收, 帧完整后置位s_frame_ready标志
 */
void StorageRx_OnByte(uint8_t data);

/**
 * @brief  存储主处理函数 (在main主循环while(1)中调用)
 * @note   处理接收到的完整帧, 写入W25Q256并读回校验, 回复ACK
 */
void StorageRx_Process(void);
/**
 * @brief  后台分步预擦下一个meta bank (main主循环每圈调用)
 * @note   P0-A修复: 每圈至多擦除1个扇区(典型45ms/最坏400ms), 分8圈完成一个bank,
 *         避免MetaSave切库时整库擦除(最坏约3.2s)阻塞写路径丢帧.
 */
void StorageRx_MetaPrepare(void);

/**
 * @brief  查询存储记录数
 * @retval 存储记录数
 */
uint32_t StorageRx_GetRecordCount(uint8_t zone);   /* 查询指定分区现存条数(P1-5整改) */
uint32_t StorageRx_GetTotalCount(void);            /* 查询全部分区总条数 */

/**
 * @brief  查询剩余存储容量
 * @retval 剩余容量
 */
uint32_t StorageRx_GetRemainingCount(void);

/**
 * @brief  按索引读取存储记录 (供导出模块使用)
 * @param  index: 记录索引(0开始)
 * @param  rec: 读取到的记录
 * @retval 0=成功, 1=失败
 */
uint8_t StorageRx_ReadRecord(uint8_t zone, uint32_t index, EventRecord_t *rec);

/**
 * @brief  擦除全部存储 (复位写指针为0)
 */
void StorageRx_EraseAll(void);

/*==============================================================
 * 调试全局变量 (供 main.c 及 USB_CDC 使用, 位于接收链路)
 *============================================================*/
extern volatile uint32_t g_stx_rx_byte_count;     /* 累计收到的字节数(每次中断+1) */
extern volatile uint8_t  g_stx_last_byte;          /* 最后一次收到的字节(用于排错) */
extern volatile uint8_t  g_stx_rx_idx_snap;        /* s_rx_idx 快照(指示接收进度, 0=空闲) */
extern volatile uint8_t  g_stx_frame_ready_snap;   /* s_frame_ready 快照(1=有完整帧待处理) */
extern volatile uint32_t g_stx_process_count;      /* StorageRx_Process 实际处理过的帧数(确认主控帧已到存储侧并处理) */

/* ISR帧到达事件(中断捕获, main循环消费): 用于"中断处理函数日志"确认 */
extern volatile uint8_t  g_stx_isr_frame_event;    /* 1=ISR收到完整帧待打印, 0=空闲 */
extern volatile uint8_t  g_stx_isr_last_cmd;       /* ISR最后一次收到的帧cmd */
extern volatile uint32_t g_stx_isr_frame_count;    /* ISR累计收到的完整帧数 */

#endif /* __BSP_STORAGE_RX_H */
