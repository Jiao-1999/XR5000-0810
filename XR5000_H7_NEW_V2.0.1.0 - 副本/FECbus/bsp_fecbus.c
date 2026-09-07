/**
 * @file    bsp_fecbus.c
 * @brief   GB4717-2024 附录C FECbus RS485 协议发送侧
 *
 * @details
 *   通信链路: USART3 (PB10=TX, PB11=RX) -> RS485 收发器 -> USB-RS485 转换器 -> PC COM6
 *   波特率: 19200 8N1 (Fecbus_Init 时配置, CubeMX 初始 9600)
 *   调试: 用串口助手打开 COM6 收发 FECbus 帧 (波特率选 19200 8N1)
 *   DMA:   DMA2_Stream1 RX 沿用 CubeMX 配置, 本模块不使用 DMA 收发,
 *          发送采用查询 TXE (协议简单/数据帧短小, 无阻塞风险)
 *
 *   发送:
 *     - 等待 TXE -> 写 TDR -> 等待 TC
 *     - 多任务并发发送用 g_fecbus_tx_mutex 递归互斥保护
 *     - 周期广播在周期任务中直接发送
 *
 *   接收 (协议层):
 *     - WaitByte 等 RXNE, 注意 ORE (长间隔收发时先清 RX)
 *     - 应答帧: [0x7E][FT][DA][PA][SA][MN][00][02][0x0F][0x00][CRC0][CRC1][0x7E]
 *
 *   重试与看门狗:
 *     - 单播发送后等 ACK 超时 1s, 最多重试 3 次
 *     - 发送 3 帧事件, 每帧耗时 ~3s 加 3 次 = 9s, 注意 IWDG (~8.2s)
 *     - 发送期间周期调用 HAL_IWDG_Refresh(&hiwdg1)
 *
 *   参考实现: bsp_storage_tx.c (LPUART1 存储通信链路, 含 CRC16 计算)
 */
#include "bsp_fecbus.h"
#include "bsp_fecbus_rx.h"
#include "bsp_rtc.h"
#include "iwdg.h"
#include "usart.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>

/*==============================================================
 * 常量定义
 *============================================================*/
#define FECBUS_QUEUE_DEPTH       16      /* 事件队列深度(上限) */
#define FECBUS_TX_MUTEX_TIMEOUT  5000    /* 发送互斥锁超时 5s */

/*==============================================================
 * 全局变量
 *============================================================*/
static uint8_t             s_initialized = 0;
static QueueHandle_t       s_tx_queue    = NULL;
static SemaphoreHandle_t   s_tx_mutex    = NULL;   /* 收发互斥锁 */
static uint8_t             s_msg_seq     = 0;      /* 报文编号 MN: 取用时 1~63 循环 */

/**
 * @brief  获取下一个报文编号 (GB4717: MN 范围 1~63)
 * @retval 下一个 MN 值
 */
static uint8_t Fecbus_NextSeq(void)
{
    s_msg_seq++;
    if (s_msg_seq > 63) { s_msg_seq = 1; }   /* GB4717: MN 范围 1..63 */
    return s_msg_seq;
}

/*==============================================================
 * 发送基础函数
 *============================================================*/

/**
 * @brief  发送单字节 (查询方式)
 * @param  data: 待发送字节
 * @注意   先等 TXE(发送数据寄存器空)再写 TDR, 再等 TC(发送完成)再退出.
 *         查询带超时避免死等. 参考 bsp_storage_tx.c 实现.
 */
static void Fecbus_SendByte(UART_HandleTypeDef *huart, uint8_t data)
{
    uint32_t timeout = 100000;
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) && timeout--) { ; }
    if (timeout == 0) return;

    huart->Instance->TDR = (uint32_t)data;

    timeout = 100000;
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) && timeout--) { ; }
}

/**
 * @brief  清 RX 残留: 清除错误标志并读出 RDR 残留字节
 * @注意   每帧发送前调用, 避免残留数据触发 ORE 影响下次 RXNE.
 *         先写 ICR 清 PE/FE/NE/ORE/IDLE 标志, 再读 RDR 清 RXNE.
 */
static void Fecbus_FlushRx(UART_HandleTypeDef *huart)
{
    huart->Instance->ICR = 0x0000001FU;
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        (void)huart->Instance->RDR;
    }
}

/*==============================================================
 * CRC16 (多项式 0xA001, MODBUS CRC16, 小端存储)
 *============================================================*/
/**
 * @brief   计算 MODBUS CRC16 (多项式 0xA001)
 * @param   data: 待校验数据指针
 * @param   len:  数据长度
 * @retval  CRC16 值 (发送时低字节在前, 小端存储)
 * @注意    初值 0xFFFF, 逐字节异或后右移 8 位, 最低位为 1 时异或 0xA001.
 *          参考 bsp_storage_tx.c StorageTx_CRC16 实现, 保证与存储侧设备一致.
 */
uint16_t Fecbus_CalcCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*==============================================================
 * 发送组帧
 *============================================================*/

/**
 * @brief  组装并发送一帧 FECbus 帧 (发送基础)
 * @param  ft:  帧类型 (FECBUS_FT_UNCONFIRMED 等)
 * @param  da:  目的地址 (0=广播)
 * @param  pa:  优先级 (FECBUS_PA_URGENT / IMPORTANT / NORMAL)
 * @param  mn:  报文编号
 * @param  tn:  帧序号 (1/2/3 多帧, 单帧=0)
 * @param  payload: 数据区内容
 * @param  payload_len: 数据区长度
 * @注意   帧格式: [0x7E][FT][DA][PA][SA][MN][TN][DLC][payload...][CRC0][CRC1][0x7E]
 *         SA 固定 1 (控制器)
 *         CRC 范围(C.6.1.6): 帧头0x7E + 报文头 + 数据 (即 buf[0]..payload末, 共 8+payload_len 字节)
 *         小端存储. 发送时逐字节查询发送.
 */
static void Fecbus_SendFrame(UART_HandleTypeDef *huart, uint8_t ft, uint8_t da, uint8_t pa,
                             uint8_t mn, uint8_t tn,
                             const uint8_t *payload, uint16_t payload_len)
{
    uint8_t  buf[FECBUS_MAX_FRAME_LEN];
    uint16_t idx = 0;
    uint16_t crc;

    /* GB4717 附录C: DLC 范围 1~8, 超长直接丢弃 */
    if (payload_len > 8) {
        return;
    }

    /* 帧头 */
    buf[idx++] = FECBUS_FRAME_HEAD;

    /* 帧头字段 */
    buf[idx++] = ft;
    buf[idx++] = da;
    buf[idx++] = pa;
    buf[idx++] = FECBUS_SA_CONTROLLER;  /* SA=1 固定 */
    buf[idx++] = mn;
    buf[idx++] = tn;
    buf[idx++] = (uint8_t)payload_len;  /* DLC */

    /* 数据区 */
    if (payload_len > 0 && payload != NULL) {
        memcpy(&buf[idx], payload, payload_len);
        idx += payload_len;
    }

    /* CRC 范围(C.6.1.6): 帧头0x7E + 报文头 + 数据, 即 buf[0]..buf[idx-1], 共 idx 字节 */
    crc = Fecbus_CalcCRC16(buf, idx);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    /* 帧尾 */
    buf[idx++] = FECBUS_FRAME_TAIL;

    /* 逐字节发送 */
    for (uint16_t i = 0; i < idx; i++) {
        Fecbus_SendByte(huart, buf[i]);
    }
}

/**
 * @brief  发送一帧原始 FECbus 帧 (带发送互斥保护)
 * @note   供 FecbusRx_ReplyStatus()/FecbusRx_ReplyEcho() 回发应答帧使用(D3).
 *         整帧发送期间持有 g_fecbus_tx_mutex, 与其它发送者互斥.
 */
uint8_t Fecbus_SendRawFrame(uint8_t ft, uint8_t da, uint8_t pa,
                            uint8_t mn, uint8_t tn,
                            const uint8_t *payload, uint8_t payload_len)
{
    if (!s_initialized) {
        return 1;
    }
    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) {
        return 1;
    }
    /* 0FH 应答回上报总线: 走 USART3 (从机通道) */
    Fecbus_SendFrame(&huart3, ft, da, pa, mn, tn, payload, payload_len);
    xSemaphoreGiveRecursive(s_tx_mutex);
    return 0;
}

/*==============================================================
 * 发送事件通告组 + 按组等待应答 (D4, 表C.6)
 *============================================================*/

/**
 * @brief   按组发送事件通告4帧 + 等待组应答 (D4)
 * @param   ft/da/pa/mn: 帧头字段(4帧共用一个 MN)
 * @param   f1/f2/f3: TN=1/2/3 数据帧; f4: TN=0 分组报文结束帧[0FH][00H]
 * @retval  0=收到组应答, 1=超时(整组重试<=3次仍无应答), 2=NAK(对端异常应答, 终止重试)
 * @note    - 广播(da=0)不等待应答: 连发4帧即返回0
 *          - 单播: 帧1~4连发(不逐帧等待), 末帧后统一等1个组应答(表C.6); 超时重发整组
 *          - C.3.3: 重试单位=整组请求报文, <=3次; 每轮发送前喂 IWDG
 *          - 应答识别(D4): 回显帧(dlc=1,data[0]=功能码) 或 0FH状态帧(dlc=2, status=0)
 */
static uint8_t Fecbus_SendEventGroup(uint8_t ft, uint8_t da, uint8_t pa, uint8_t mn,
                                     const uint8_t *f1, const uint8_t *f2,
                                     const uint8_t *f3, const uint8_t *f4)
{
    uint8_t retry;

    /* 广播帧不等待应答 (走主机通道 USART1): 连发4帧即返回 */
    if (da == FECBUS_DA_BROADCAST) {
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME1, f1, 8);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME2, f2, 8);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME3, f3, 1);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_SINGLE, f4, 2);
        return 0;
    }

    for (retry = 0; retry < FECBUS_RETRY_COUNT; retry++) {
        uint32_t tickstart;
        uint8_t  ack;
        /* 每轮(整组)发送前喂 IWDG */
        HAL_IWDG_Refresh(&hiwdg1);
        Fecbus_FlushRx(&huart1);    /* 清 USART1 硬件 RX 残留 (主动下发通道) */
        FecbusRx_Flush();           /* 清接收环形缓冲 (丢弃发送前残留数据) */
        FecbusRx_ResetAck();        /* 清应答标志 */
        FecbusRx_SetAckFunc(f1[0]); /* D4: 期望对端回显的功能码 = 事件功能码 */

        /* 帧1~4 连发 (不逐帧等待, 走主机通道 USART1) */
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME1, f1, 8);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME2, f2, 8);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_FRAME3, f3, 1);
        Fecbus_SendFrame(&huart1, ft, da, pa, mn, FECBUS_TN_SINGLE, f4, 2);

        /* 组应答等待 (表C.6: 每组1个应答), 超时1s */
        tickstart = HAL_GetTick();
        while ((HAL_GetTick() - tickstart) < FECBUS_ACK_TIMEOUT_MS) {
            Fecbus_RxPoll();        /* 及时解析接收到的应答帧 */
            ack = FecbusRx_CheckAckEx(mn, f1[0]);
            if (ack == 1) return 0; /* 收到组应答(回显功能码 或 0FH结束) */
            if (ack == 2) return 2; /* NAK: 参数错等, 终止重试 */
            vTaskDelay(1);
        }
    }

    return 1;  /* 整组重试后仍未收到应答 */
}

/*==============================================================
 * 事件上报
 *============================================================*/

/**
 * @brief   组帧发送事件上报 (A7/D2: 表C.3/C.5 事件通告 4 帧制)
 * @param   item: 事件项
 * @note    - 第1帧 TN=01: [功能码][控制器][单元][设备][通道][类型低][类型高][事件代码低] (DLC=8)
 *          - 第2帧 TN=02: [事件代码高][状态低][状态高][年-2000][月][日][时][分]        (DLC=8)
 *          - 第3帧 TN=03: [秒]                                                    (DLC=1)
 *          - 第4帧 TN=00: [0x0F][0x00]                                            (DLC=2, 分组报文结束帧)
 *
 *          4帧共用一个 MN (同一报文), 帧号 TN 区分次序; 按组应答(表C.6), 整组重试<=3次
 */
static void Fecbus_SendEvent(const FecbusEventItem_t *item)
{
    uint8_t mn = Fecbus_NextSeq();
    uint8_t frame1[8];
    uint8_t frame2[8];
    uint8_t frame3[1];
    uint8_t endfrm[2];
    uint8_t pa = item->pa;
    uint8_t da = item->da;
    uint8_t ft = FECBUS_FT_UNCONFIRMED;

    /* 第1帧(表C.5): [功能码][控制器][单元][设备][通道][类型低][类型高][事件代码低] */
    frame1[0] = item->func_code;
    frame1[1] = FECBUS_SA_CONTROLLER;        /* 控制器编号=1 */
    frame1[2] = item->unit_no;
    frame1[3] = item->dev_no;
    frame1[4] = item->channel_no;
    frame1[5] = (uint8_t)(item->dev_type & 0xFF);
    frame1[6] = (uint8_t)((item->dev_type >> 8) & 0xFF);
    frame1[7] = (uint8_t)(item->event_code & 0xFF);   /* A7: 事件代码低字节(原预留位) */

    /* 当前时间 (读 RTC): 年/月/日/时/分/秒 (A7: 补月/日) */
    getBM8563TimeToSystemTime();

    /* 第2帧(表C.5): [事件代码高][状态低][状态高][年-2000][月][日][时][分] */
    frame2[0] = (uint8_t)((item->event_code >> 8) & 0xFF);
    frame2[1] = (uint8_t)(item->state_code & 0xFF);
    frame2[2] = (uint8_t)((item->state_code >> 8) & 0xFF);
    frame2[3] = (uint8_t)(SystemTime.year - 2000);
    frame2[4] = SystemTime.month;
    frame2[5] = SystemTime.day;
    frame2[6] = SystemTime.hours;
    frame2[7] = SystemTime.minutes;

    /* 第3帧(表C.5): [秒] */
    frame3[0] = SystemTime.seconds;

    /* 第4帧(D2, 表C.3/C.5): 分组报文结束帧 [0x0F][0x00] */
    endfrm[0] = FECBUS_FUNC_RESP;   /* 0x0F */
    endfrm[1] = 0x00;               /* 状态应答码 0=分组报文结束 */

    /* 整组4帧持锁发送, 防止与其它发送者交错 */
    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) {
        return;
    }

    /* D4: 按组发送(帧1~4连发) + 组应答等待 + 整组重试 */
    (void)Fecbus_SendEventGroup(ft, da, pa, mn, frame1, frame2, frame3, endfrm);

    xSemaphoreGiveRecursive(s_tx_mutex);
}

/**
 * @brief   A9(表C.3): 复位/消音/自检单帧上报 TN=0 DLC=2 data=[功能码][控制器编号=1]
 * @param   func: 功能码 (1=复位/2=消音/3=自检)
 * @param   pa:   优先级 (复位/消音=PA_URGENT(01H), 自检=PA_NORMAL(03H), 表C.3)
 * @note    国标为单帧, 不走事件4帧布局; 广播发送(da=0)不等待应答.
 */
static void Fecbus_SendSimple(uint8_t func, uint8_t pa)
{
    uint8_t payload[2];
    uint8_t mn = Fecbus_NextSeq();

    payload[0] = func;
    payload[1] = FECBUS_SA_CONTROLLER;   /* 控制器编号=1 */

    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) return;
    HAL_IWDG_Refresh(&hiwdg1);
    Fecbus_FlushRx(&huart1);
    Fecbus_SendFrame(&huart1, FECBUS_FT_UNCONFIRMED, FECBUS_DA_BROADCAST,
                     pa, mn, FECBUS_TN_SINGLE, payload, 2);  /* 广播走主机通道 USART1 */
    xSemaphoreGiveRecursive(s_tx_mutex);
}

/*==============================================================
 * 周期广播
 *============================================================*/

/**
 * @brief   发送同步心跳帧 (功能码 0x00, 1s 周期)
 * @note    广播帧, DLC=1, 数据=[0x00]; A10(表C.3 行0): PA=00H, MN=00H
 *          广播维持不等应答(多装置同时应答会在RS485冲突; 严格合规需单播轮询, 本轮不做)
 */
static void Fecbus_SendSyncBeat(void)
{
    uint8_t payload[1] = { FECBUS_FUNC_SYNC_BEAT };
    /* A10(表C.3 行0): 同步节拍特例 PA=00H, MN=00H (不经 Fecbus_NextSeq 循环序号) */
    uint8_t mn = 0;

    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) return;
    HAL_IWDG_Refresh(&hiwdg1);
    Fecbus_FlushRx(&huart1);
    Fecbus_SendFrame(&huart1, FECBUS_FT_UNCONFIRMED, FECBUS_DA_BROADCAST,
                     FECBUS_PA_SYNC, mn, FECBUS_TN_SINGLE,
                     payload, 1);  /* 周期广播走主机通道 USART1 */
    xSemaphoreGiveRecursive(s_tx_mutex);
}

/**
 * @brief   巡检装置连接状态 (控->装, 功能码 0x21, 5s 周期, 表C.7)
 * @note    A4/A6: 原「心跳帧 0x14」改为国标巡检 0x21; PA 由 03H 改 02H(重要, 表C.7 行33)。
 *          广播帧, DLC=1, 数据=[0x21]。装置侧回显应答(表C.2 标有应答)。
 */
static void Fecbus_SendHeartbeat(void)
{
    uint8_t payload[1] = { FECBUS_FUNC_POLL_CONN };   /* A4: 0x14 -> 0x21 巡检连接 */
    uint8_t mn = Fecbus_NextSeq();

    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) return;
    HAL_IWDG_Refresh(&hiwdg1);
    Fecbus_FlushRx(&huart1);
    Fecbus_SendFrame(&huart1, FECBUS_FT_UNCONFIRMED, FECBUS_DA_BROADCAST,
                     FECBUS_PA_IMPORTANT, mn, FECBUS_TN_SINGLE,   /* A6: PA 03H -> 02H(重要) */
                     payload, 1);  /* 周期广播走主机通道 USART1 */
    xSemaphoreGiveRecursive(s_tx_mutex);
}

/**
 * @brief   发送时钟广播帧 (功能码 0x04, 10s 周期)
 * @note    广播帧, DLC=8, 数据=[0x04][编号][年-2000][月][日][时][分][秒]
 */
static void Fecbus_SendClockBC(void)
{
    uint8_t payload[8];  /* GB4717 04H 数据: 功能码 + 编号 + 时间(6B) = 8B */
    uint8_t mn = Fecbus_NextSeq();

    getBM8563TimeToSystemTime();
    payload[0] = FECBUS_FUNC_CLOCK_BC;
    payload[1] = FECBUS_SA_CONTROLLER;      /* 编号=控制器编号 1 (GB4717 04H 数据格式) */
    payload[2] = (uint8_t)(SystemTime.year - 2000);
    payload[3] = SystemTime.month;
    payload[4] = SystemTime.day;
    payload[5] = SystemTime.hours;
    payload[6] = SystemTime.minutes;
    payload[7] = SystemTime.seconds;

    if (xSemaphoreTakeRecursive(s_tx_mutex, FECBUS_TX_MUTEX_TIMEOUT) != pdTRUE) return;
    HAL_IWDG_Refresh(&hiwdg1);
    Fecbus_FlushRx(&huart1);
    Fecbus_SendFrame(&huart1, FECBUS_FT_UNCONFIRMED, FECBUS_DA_BROADCAST,
                     FECBUS_PA_NORMAL, mn, FECBUS_TN_SINGLE,
                     payload, 8);
    xSemaphoreGiveRecursive(s_tx_mutex);
}

/*==============================================================
 * 对外 API
 *============================================================*/

/**
 * @brief  初始化 FECbus 协议
 * @note   - 创建发送队列/发送互斥锁
 *         - USART3 波特率由 CubeMX 的 MX_USART3_UART_Init() (usart.c) 配置为 19200,
 *           此处不重复调用 HAL_UART_Init, 避免重复初始化 GPIO/DMA
 *         - 使能接收 RE 位 (H7 部分库默认只开发送未开接收)
 *         - 注意: 初始化前需先执行 MX_USART3_UART_Init()
 */
void Fecbus_Init(void)
{
    if (s_initialized) return;

    /* 使能接收: 打开 USART1/USART3 RE 位 (双串口: USART1 主机通道 / USART3 从机通道) */
    SET_BIT(huart1.Instance->CR1, USART_CR1_RE);
    SET_BIT(huart3.Instance->CR1, USART_CR1_RE);

    /* 创建事件队列 */
    if (s_tx_queue == NULL) {
        s_tx_queue = xQueueCreate(FECBUS_QUEUE_DEPTH, sizeof(FecbusEventItem_t));
    }

    /* 创建递归互斥锁 (发送/接收共用物理链路, 需互斥保护) */
    if (s_tx_mutex == NULL) {
        s_tx_mutex = xSemaphoreCreateRecursiveMutex();
    }

    /* 启动 USART3 RX IT + 环形缓冲 (bsp_fecbus_rx.c) */
    Fecbus_RxInit();

    s_initialized = 1;
}

/**
 * @brief  发送测试事件(三帧) - 用于验证FECbus发送链路
 * @note   XR5000_FECBUS_TEST_EVENT_20260811:
 *         - 固定组一帧火灾上报三帧, 便于串口抓帧验证
 *         - 不参与业务上报, 勿在周期任务(FecbusPeriodicTask)中循环调用
 *         - 组帧参数: 功能码5(火灾) 回路1 设备号2 分区31(温度) 状态3(火警), 共3帧
 */
void Fecbus_SendTestEvent(void)
{
    FecbusEventItem_t item;

    item.func_code  = FECBUS_FUNC_URGENT_EVT; /* 功能码5: 紧急事件上报 */
    item.da         = FECBUS_DA_BROADCAST;     /* 广播发送 */
    item.pa         = FECBUS_PA_URGENT;        /* 优先级: 紧急 */
    item.dev_no     = 2;                       /* 测试设备编号 */
    item.unit_no    = 1;                       /* 回路编号 */
    item.channel_no = 0;                       /* 分区编号 */
    item.dev_type   = 31;                      /* DEV_TYPE_TEMPERATURE 温度传感器 */
    item.event_code = 3;                       /* EVT_FIRE 火警 */
    item.state_code = 0;                       /* 状态编码 */

    Fecbus_SendEvent(&item);
}

/**
 * @brief   入队上报 FECbus 事件 (异步)
 * @param   item: 事件项 (参见 FecbusEventItem_t)
 * @retval  0=入队成功, 1=队列满(丢弃旧项), 2=未初始化, 3=参数错误
 */
uint8_t Fecbus_QueueEvent(const FecbusEventItem_t *item)
{
    if (!s_initialized || s_tx_queue == NULL) {
        return 2;
    }
    if (item == NULL) {
        return 3;
    }

    /* 直接入队(非阻塞) */
    if (xQueueSend(s_tx_queue, item, 0) == pdTRUE) {
        return 0;
    }

    /* 队列满: 丢弃最旧一条再入队 */
    FecbusEventItem_t old;
    xQueueReceive(s_tx_queue, &old, 0);
    if (xQueueSend(s_tx_queue, item, 0) == pdTRUE) {
        return 1;  /* 入队成功, 丢弃旧项 */
    }

    return 1;
}

/**
 * @brief   发送任务主循环 (周期, 由 FreeRTOS 任务循环调用)
 * @note    从队列取事件组帧发送, 空闲时轮询接收解析, 周期内喂 IWDG
 */
void Fecbus_TxTaskLoop(void)
{
    if (!s_initialized || s_tx_queue == NULL) {
        vTaskDelay(1000);
        return;
    }

    FecbusEventItem_t item;

    /* 等待队列事件(100ms超时) */
    if (xQueueReceive(s_tx_queue, &item, 100) == pdTRUE) {
        HAL_IWDG_Refresh(&hiwdg1);
        /* A9(表C.3): 复位/消音/自检走单帧路径(DLC=2), 其余走事件4帧组 */
        if (item.func_code == FECBUS_FUNC_RESET) {
            Fecbus_SendSimple(FECBUS_FUNC_RESET, FECBUS_PA_URGENT);
        } else if (item.func_code == FECBUS_FUNC_SILENCE) {
            Fecbus_SendSimple(FECBUS_FUNC_SILENCE, FECBUS_PA_URGENT);
        } else if (item.func_code == FECBUS_FUNC_SELFTEST) {
            Fecbus_SendSimple(FECBUS_FUNC_SELFTEST, FECBUS_PA_NORMAL);
        } else {
            Fecbus_SendEvent(&item);
        }
        vTaskDelay(10);  /* 让出 CPU */
    }

    /* FECbus RX 解析: 中断入环, 此处解析 (协议层) */
    Fecbus_RxPoll();
}

/**
 * @brief   周期广播任务主循环 (周期, 由 FreeRTOS 任务循环调用)
 * @note    - 1s 周期: 同步心跳帧 (0x00)
 *          - 5s 周期: 心跳帧 (0x14)
 *          - 10s 周期: 时钟广播帧 (0x04)
 *          每周期喂 IWDG
 */
void Fecbus_PeriodicTaskLoop(void)
{
    if (!s_initialized) {
        vTaskDelay(1000);
        return;
    }

    static uint32_t beat_cnt = 0;     /* 1s 计数 */
    static uint32_t hb_cnt   = 0;     /* 5s 计数 */
    static uint32_t clk_cnt  = 0;     /* 10s 计数 */

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg1);

        /* 1s 同步心跳 */
        if (beat_cnt >= 1) {
            Fecbus_SendSyncBeat();
            beat_cnt = 0;
        }

        /* 5s 心跳 */
        if (hb_cnt >= 5) {
            Fecbus_SendHeartbeat();
            hb_cnt = 0;
        }

        /* 10s 时钟广播 */
        if (clk_cnt >= 10) {
            Fecbus_SendClockBC();
            clk_cnt = 0;
        }

        beat_cnt++;
        hb_cnt++;
        clk_cnt++;

        vTaskDelay(1000);  /* 1s 周期 */
    }
}
