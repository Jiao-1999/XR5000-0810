/**
 * @file    bsp_fecbus.h
 * @brief   GB4717-2024 附录C FECbus RS485 协议
 *
 * @details
 *   通信链路 (双串口拆分):
 *     USART3 (PB10=TX, PB11=RX) -> RS485 收发器 A: 从机通道
 *       RX 收从机主动上报, TX 回 0FH 应答给上报方
 *     USART1 (PB14=TX, PB15=RX) -> RS485 收发器 B: 主机通道
 *       TX 本机主动下发(设置/广播/查询/事件上报), RX 收从机 0FH 应答
 *     两串口均 19200 8N1 (GB4717 附录C 规定), 两串口 RX 共用同一环形缓冲/状态机(FecbusRx_OnByte)
 *
 *   调试链路:
 *     RS485 收发器接 USB-RS485 转换器, 上位机用 PC 端 COM6 口, 与控制器互发 FECbus 帧
 *
 *   帧格式 (GB4717-2024 附录C 表C.22):
 *     [0x7E][FT][DA][PA][SA][MN][TN][DLC][数据0..N][CRC0][CRC1][0x7E]
 *     FT  = 帧类型 (0=请求/上报, 1=应答; 废除原自定义 3=ACK/4=NAK)
 *     DA  = 目的地址 (0=广播)
 *     PA  = 优先级 (1=紧急, 2=重要, 3=一般)
 *     SA  = 源地址 (控制器固定取 1)
 *     MN  = 报文编号 (GB4717 限定 1~63 循环)
 *     TN  = 帧序号 (多帧上报用 1/2/3, 0=单帧/广播等)
 *     DLC = 数据长度 (GB4717 限定 1~8)
 *     CRC16 多项式 0xA001, 小端存储
 *     CRC 校验范围(C.6.1.6): 帧头0x7E + 报文头 + 数据0~7 (含0x7E帧头, 不含帧尾/CRC自身)
 *
 *   上报事件 (控制器->上位机, 表C.3/C.5 事件通告 4 帧制):
 *    第1帧 TN=01: [功能码][控制器][单元][设备][通道][类型低][类型高][事件代码低] (DLC=8)
 *    第2帧 TN=02: [事件代码高][状态低][状态高][年-2000][月][日][时][分]        (DLC=8)
 *    第3帧 TN=03: [秒]                                                    (DLC=1)
 *    第4帧 TN=00: [0x0F][0x00]                                            (DLC=2, 分组报文结束帧)
 *
 *   状态应答 (设备->上位机, 收到单播通告后):
 *     TN=00: [0x0F][0x00]  (DLC=2, 状态应答)
 *
 *   注意事项:
 *     - 周期上报需周期性 HAL_IWDG_Refresh(&hiwdg1) (3个周期任务约 ~3s, IWDG ~8.2s)
 *     - 主动下发(USART1)与被动回应(USART3)物理隔离, 发送互斥锁 s_tx_mutex 仍保留
 *       (防止多任务并发主动下发交错; 0FH 回应由接收中断驱动, 与主动下发不同串口不冲突)
 *     - 事件上报可走 bsp_storage_event 存储链路 (LPUART1)
 */
#ifndef __BSP_FECBUS_H
#define __BSP_FECBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/*==============================================================
 * 协议常量
 *============================================================*/
#define FECBUS_FRAME_HEAD       0x7E    /* 帧头 */
#define FECBUS_FRAME_TAIL       0x7E    /* 帧尾 (与帧头相同) */

#define FECBUS_DEFAULT_BAUDRATE 19200   /* GB4717 附录C 规定的波特率 */

/* 帧类型 FT (GB4717 附录C 表C.1: 0=请求/上报, 1=应答) */
#define FECBUS_FT_REQUEST       0       /* 请求/上报帧 */
#define FECBUS_FT_UNCONFIRMED   0       /* 兼容保留, 与请求帧相同 */
#define FECBUS_FT_RESPONSE      1       /* 应答帧 */
/* 原自定义 ACK(3)/NAK(4) 已删除: 不符合 GB4717 */

/* 优先级 PA */
#define FECBUS_PA_URGENT        1       /* 紧急 (火警/故障/监管/启动等) */
#define FECBUS_PA_IMPORTANT     2       /* 重要 */
#define FECBUS_PA_NORMAL        3       /* 一般 (周期/广播帧) */
#define FECBUS_PA_SYNC          0       /* 同步节拍专用 PA (A10, 表C.3 行0: PA=00H) */

/* 上报功能码 (控制器->上位机) */
#define FECBUS_FUNC_RESET       1       /* 复位请求 (按键) */
#define FECBUS_FUNC_SILENCE     2       /* 消音请求 (按键) */
#define FECBUS_FUNC_SELFTEST    3       /* 自检请求 (按键) */
#define FECBUS_FUNC_URGENT_EVT  5       /* 紧急事件上报 (火灾) */
#define FECBUS_FUNC_NORMAL_EVT  6       /* 一般事件上报 (故障) */
#define FECBUS_FUNC_IMPORTANT_EVT 7     /* 重要事件上报 (监管) */

/* 装置通告功能码 (装置->控制器, B1: 表C.2 17/18/19H, 4帧组装) */
#define FECBUS_FUNC_NOTIFY_URGENT 0x11  /* 紧急通告(火警/首警等) */
#define FECBUS_FUNC_NOTIFY_NORMAL 0x12  /* 一般通告(故障等) */
#define FECBUS_FUNC_NOTIFY_DEBUG  0x13  /* 调试通告(不入库, 仅打印回显) */

/* 查询功能码 (B2: 表C.2 33~45H)
 * 角色定位(决策点8): 国标方向为 控->装 查询; 本工程「接收并应答」= 供上位机/PC
 * 查询本控制器, 属厂商扩展, 须写入双方协议补充说明。 */
#define FECBUS_FUNC_POLL_CONN     0x21  /* 巡检装置连接状态(控->装, 表C.7; 批次6用) */
#define FECBUS_FUNC_DEV_STATUS    0x22  /* 查设备状态(表C.18 状态位聚合) */
#define FECBUS_FUNC_DEV_CONFIG    0x23  /* 查设备配置(返回层级目标数量, C.4.5.13) */
#define FECBUS_FUNC_DEV_ID        0x24  /* 查设备标识(7字节, 生产者规定 C.4.5.12) */
#define FECBUS_FUNC_DEV_PARAM     0x25  /* 查设备参量(阈值, 表C.19) */
#define FECBUS_FUNC_DEV_COMMENT   0x26  /* 查设备注释(UTF-8, 表C.9; 批次7) */
#define FECBUS_FUNC_DEV_PROGRAM   0x27  /* 查设备编程(UTF-8, 表C.10; 批次8留桩) */
#define FECBUS_FUNC_REG_INFO      0x28  /* 查注册登记信息(UTF-8, 表C.11; 批次7) */
#define FECBUS_FUNC_QUERY_CUR_EVT 0x29  /* 查当前事件(表C.12; 批次7) */
#define FECBUS_FUNC_QUERY_HIS_EVT 0x2A  /* 查历史事件(表C.13; 批次8留桩) */
#define FECBUS_FUNC_STOP_QUERY    0x2B  /* 停止查询(清查询会话) */
#define FECBUS_FUNC_PROTO_VER     0x2C  /* 查协议版本号 */
#define FECBUS_FUNC_DEV_LIST      0x2D  /* 查设备列表(表C.14; 批次7) */

/* 状态应答功能码 (GB4717 附录C: 0FH = 状态应答) */
#define FECBUS_FUNC_RESP        0x0F    /* 状态应答帧功能码 */

/* 0FH 状态应答码 (A3, GB4717 附录C C.4.2.5)
 * 用途约束: 0FH 仅三类场景 --
 *   (a) 异常应答: 状态码 1~8 (CRC错/无效服务/单元故障/忙/未知命令/地址不存在/参数错/处理中)
 *   (b) 分组报文结束: 状态码 0 (END_GROUP)
 *   (c) 事件全部结束: 状态码 9 (EVT_END)
 *   成功应答一律走 ReplyEcho(回显功能码)/数据帧, 不发 0FH。 */
#define FECBUS_STAT_END_GROUP       0   /* 分组报文结束 */
#define FECBUS_STAT_CRC_ERR         1   /* CRC 校验错误 */
#define FECBUS_STAT_INVALID_SVC     2   /* 无效服务/不支持的操作 */
#define FECBUS_STAT_UNIT_FAULT      3   /* 单元故障 */
#define FECBUS_STAT_BUSY            4   /* 设备忙 */
#define FECBUS_STAT_UNKNOWN_CMD     5   /* 未知命令/功能码 */
#define FECBUS_STAT_ADDR_NOT_EXIST  6   /* 地址不存在 */
#define FECBUS_STAT_PARAM_ERR       7   /* 参数错误 */
#define FECBUS_STAT_PROCESSING      8   /* 处理中(稍后再试) */
#define FECBUS_STAT_EVT_END         9   /* 事件全部结束 */
#define FECBUS_STAT_DATA_OK         10  /* 数据正确(保留) */

/* 周期广播功能码 */
#define FECBUS_FUNC_SYNC_BEAT   0x00    /* 同步心跳帧 (1s 周期) */
#define FECBUS_FUNC_CLOCK_BC    0x04    /* 时钟广播帧 (10s 周期) */
#define FECBUS_FUNC_HEARTBEAT   0x14    /* 装->控心跳帧 (接收用; 控->装周期巡检已改 0x21, A4) */

/* 地址 */
#define FECBUS_DA_BROADCAST     0       /* 广播地址 */
#define FECBUS_SA_CONTROLLER    1       /* 控制器源地址固定为 1 */

/* 帧号 TN */
#define FECBUS_TN_SINGLE        0       /* 单帧/广播帧 */
#define FECBUS_TN_FRAME1        1       /* 第1帧 */
#define FECBUS_TN_FRAME2        2       /* 第2帧 */
#define FECBUS_TN_FRAME3        3       /* 第3帧 (事件数据帧:秒; 分组末帧为 TN=0 结束帧) */

/* 应答与重试参数 */
#define FECBUS_RETRY_COUNT      3       /* 单播发送重试次数 */
#define FECBUS_ACK_TIMEOUT_MS   1000    /* 应答等待超时 1s */

/* 最大帧长: 头(9) + 数据(255) + CRC(2) + 尾(1) = 267 */
#define FECBUS_MAX_FRAME_LEN    270

/*==============================================================
 * 数据结构
 *============================================================*/

/**
 * @brief  FECbus 事件上报项 (可队列缓存, 发送侧按"火灾"组帧上报)
 *         上报前先查队列容量, 队列满则丢弃旧项.
 *         每个事件占约 16 字节, 队列深度 16, 最大缓存 256 字节
 */
typedef struct {
    uint8_t  func_code;     /* 功能码 (5/6/7/1/2/3) */
    uint8_t  da;            /* 目的地址 (暂用0广播) */
    uint8_t  pa;            /* 优先级 */
    uint8_t  dev_no;        /* 设备编号 */
    uint8_t  unit_no;       /* 回路编号 */
    uint8_t  channel_no;    /* 分区编号 */
    uint16_t dev_type;      /* 设备类型编号 (DEV_TYPE_xxx) */
    uint16_t event_code;    /* 事件编码 (EVT_xxx) */
    uint16_t state_code;    /* 状态编码 */
} FecbusEventItem_t;

/*==============================================================
 * API 声明
 *============================================================*/

/**
 * @brief  初始化 FECbus 协议
 * @note   - 创建发送队列/发送互斥锁
 *         - 启动 USART3 逐字节 IT 接收 (bsp_fecbus_rx.c)
 *         - 注意: 波特率切换由 CubeMX 的 MX_USART3_UART_Init() 配置
 */
void Fecbus_Init(void);

/**
 * @brief  发送测试事件(三帧) - 用于验证FECbus发送链路
 * @note   XR5000_FECBUS_TEST_EVENT_20260811: 固定组一帧火灾上报三帧,
 *         便于串口抓帧验证, 不参与业务上报(勿在FecbusPeriodicTask中循环调用).
 */
void Fecbus_SendTestEvent(void);

/**
 * @brief   入队上报 FECbus 事件 (异步)
 * @param   item: 事件项 (参见 FecbusEventItem_t)
 * @retval  0=入队成功, 1=队列满(丢弃旧项), 2=未初始化, 3=参数错误
 * @note    只在任务上下文调用, 不能中断/临界区内调用
 */
uint8_t Fecbus_QueueEvent(const FecbusEventItem_t *item);

/**
 * @brief   发送任务主循环 (周期, 由 FreeRTOS 任务循环调用)
 * @note    从队列取事件组帧发送, 具体发送流程:
 *          1. 发送事件 (三帧)
 *          2. 单播发送后等待应答 (超时 1s, 最多重试 3 次)
 *          3. 空闲时轮询接收解析 (Fecbus_RxPoll)
 */
void Fecbus_TxTaskLoop(void);

/**
 * @brief   周期广播任务主循环 (周期, 由 FreeRTOS 任务循环调用)
 * @note    - 1s 发送同步心跳帧 (0x00)
 *          - 5s 发送心跳帧 (0x14)
 *          - 10s 发送时钟广播帧 (0x04)
 *          周期内喂 IWDG, 保证周期广播不触发复位
 */
void Fecbus_PeriodicTaskLoop(void);

/**
 * @brief  发送一帧原始 FECbus 帧 (带发送互斥保护, 供接收层回 0FH 应答)
 * @param  ft/da/pa/mn/tn: 帧头字段
 * @param  payload: 数据区 (首字节为功能码), payload_len: 1..8
 * @retval 0=成功, 1=互斥锁超时
 */
uint8_t Fecbus_SendRawFrame(uint8_t ft, uint8_t da, uint8_t pa,
                            uint8_t mn, uint8_t tn,
                            const uint8_t *payload, uint8_t payload_len);

/**
 * @brief  计算 MODBUS CRC16 (多项式 0xA001, 小端存储)
 * @note   导出供接收层校验收到的帧
 */
uint16_t Fecbus_CalcCRC16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FECBUS_H */
