/**
 * @file    bsp_storage_tx.c
 * @brief   主控存储发送模块 - 主控通过LPUART1(PB6/PB7, 115200 8N1)向
 *          存储侧单片机发送事件记录.
 *
 * @details
 *   硬件接口:
 *     UART:  LPUART1 PB6=TX, PB7=RX, 115200 8N1
 *            本模块自行初始化(CubeMX未配置LPUART1), 完全使用HAL寄存器操作.
 *     存储:  存储侧单片机管理SPI Flash W25Q256, 可选USB上报.
 *
 *   帧格式(主控 -> 存储侧):
 *     [0xA5][len][cmd][payload...][CRC16_lo][CRC16_hi][0x5A]
 *     len  = 命令码字节数(1) + 数据载荷字节数  (不含帧头/帧尾/CRC)
 *     CRC16计算范围: 从len字节到最后一字节payload, 多项式0xA001, 小端存储.
 *
 *   关键背景(ORE处理):
 *     H7 LPUART的ORE(过载)标志一旦置位会永久阻塞RXNE, 导致RX通道永久失效.
 *     这是已修复的关键bug: WaitByte中检测并清除ORE, FlushRx在每次发送前清
 *     错误标志并排空RDR残留, 避免上一次事务超时后才到达的应答触发ORE.
 *
 *   依赖关系:
 *     bsp_rtc.h   - SystemTime, getBM8563TimeToSystemTime() 用于时间戳
 *     FreeRTOS    - 队列与任务
 *
 *   使用方式:
 *     StorageTx_QueueRecord()  中断/任务安全(非阻塞入队)
 *     StorageTx_TaskLoop()     专用FreeRTOS任务(阻塞出队发送)
 */
#include "bsp_storage_tx.h"
#include "bsp_rtc.h"            /* SystemTime, getBM8563TimeToSystemTime() */
#include "bsp_debug.h"          /* XR5000_STX_DIAG_20260811: 调试串口输出(COM4) */
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*==============================================================
 * 内部定义
 *============================================================*/
/* 存储发送UART: LPUART1 (PB6=TX, PB7=RX, 115200 8N1), 本模块自行初始化 */
static UART_HandleTypeDef s_hlpuart1;
#define STX_UART             (&s_hlpuart1)
static uint8_t s_initialized = 0;           /* 初始化完成标志 */
static QueueHandle_t s_tx_queue = NULL;     /* 发送队列句柄 */

/* 队列项: 命令码 + 事件记录 */
typedef struct {
    uint8_t cmd;
    EventRecord_t record;
} StorageTx_QueueItem_t;

/*==============================================================
 * 内部函数原型
 *============================================================*/
static void StorageTx_InitLPUART1(void);
static void StorageTx_SendByte(uint8_t data);
static uint8_t StorageTx_WaitByte(uint8_t *data, uint32_t timeout_ms);
static void StorageTx_FlushRx(void);
static uint16_t StorageTx_CRC16(const uint8_t *data, uint16_t len);
static void StorageTx_SendFrame(uint8_t cmd, const uint8_t *payload, uint16_t payload_len);

/*==============================================================
 * LPUART1初始化 (PB6=TX, PB7=RX, 115200 8N1)
 *============================================================*/
/**
 * @brief  初始化LPUART1及PB6/PB7引脚, 配置波特率并使能UE/TE/RE位
 * @说明   配置流程:
 *           1. 选择LPUART1内核时钟源 = D3PCLK1 (PCLK4)
 *           2. 使能GPIOB与LPUART1时钟
 *           3. 配置PB6/PB7为复用推挽AF8(LPUART)
 *           4. HAL_UART_Init配置115200/8N1/TX+RX/无流控
 *         关键: HAL_UART_Init返回值必须检查; 失败时RE位不会置位会导致
 *         RX永久无数据, 故失败时手动置UE/TE/RE兜底. 此外无论成功与否都
 *         显式置RE(H7 LPUART1在某些时钟配置下HAL可能不置RE).
 */
static void StorageTx_InitLPUART1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* 配置LPUART1内核时钟源 = D3PCLK1 (PCLK4) */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_D3PCLK1;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* 使能GPIOB与LPUART1时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_LPUART1_CLK_ENABLE();

    /* PB6=TX, PB7=RX, 复用功能AF8 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 115200 8N1, TX+RX, 无流控 */
    s_hlpuart1.Instance = LPUART1;
    s_hlpuart1.Init.BaudRate = 115200;
    s_hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    s_hlpuart1.Init.StopBits = UART_STOPBITS_1;
    s_hlpuart1.Init.Parity = UART_PARITY_NONE;
    s_hlpuart1.Init.Mode = UART_MODE_TX_RX;
    s_hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    /* XR5000_FIX_20260805: HAL_UART_Init 返回值必须检查;若失败 RE 位不会置位,
       会导致 RX 永久无数据。失败时打印并显式置 TE/RE/UE。 */
    if (HAL_UART_Init(&s_hlpuart1) != HAL_OK)
    {
        /* 兜底:手动使能 UE/TE/RE,保证即使 HAL 初始化部分失败 RX 仍可用 */
        SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_UE);
        SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_TE | USART_CR1_RE);
    }
    /* 防御性:无论 HAL_UART_Init 是否成功,都显式确保 RE 位被置位
       (H7 LPUART1 在某些时钟配置下 HAL_UART_Init 可能不置 RE) */
    SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_RE);
}

/*==============================================================
 * 初始化
 *============================================================*/

/**
 * @brief  初始化存储发送模块(LPUART1 + 发送队列)
 * @说明   幂等: 已初始化则直接返回. 先初始化LPUART1, 再创建发送队列.
 */
void StorageTx_Init(void)
{
    if (s_initialized) return;

    /* 初始化LPUART1 (PB6/PB7, 115200 8N1) */
    StorageTx_InitLPUART1();

    /* 创建发送队列 */
    if (s_tx_queue == NULL) {
        s_tx_queue = xQueueCreate(STX_QUEUE_DEPTH, sizeof(StorageTx_QueueItem_t));
    }

    s_initialized = 1;
}

/*==============================================================
 * 字节级收发
 *============================================================*/

/**
 * @brief  阻塞发送一个字节
 * @param  data: 待发送字节
 * @说明   轮询TXE(发送数据寄存器空)后写TDR, 再轮询TC(发送完成)确保字节发完.
 *         超时计数避免硬件异常时永久卡死.
 */
static void StorageTx_SendByte(uint8_t data)
{
    /* 等待TXE=1 (发送数据寄存器空) */
    uint32_t timeout = 100000;
    while (!__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_TXE) && timeout--) {;}
    if (timeout == 0) return;

    /* 写TDR(发送数据寄存器) */
    STX_UART->Instance->TDR = (uint32_t)data;

    /* 等待TC=1 (发送完成) */
    timeout = 100000;
    while (!__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_TC) && timeout--) {;}
}

/**
 * @brief   带超时接收一个字节(非阻塞轮询)
 * @param   data: 输出接收到的字节
 * @param   timeout_ms: 超时时间(ms)
 * @retval  0=成功收到字节, 1=超时
 * @说明    XR5000_FIX_20260805: H7 LPUART 的 ORE(过载)标志一旦置位会永久
 *          阻塞 RXNE, 导致后续字节永远收不到、WaitByte 必然超时。这里在
 *          轮询中检测并清除 ORE, 避免一次过载就使整个 RX 通道永久卡死。
 *          这是已修复的关键bug, ORE处理不可省略。
 */
static uint8_t StorageTx_WaitByte(uint8_t *data, uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_RXNE)) {
        /* ORE 置位时 RXNE 不会再次置位,必须先清 ORE 才能继续接收新字节 */
        if (__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(STX_UART, UART_CLEAR_OREF);
        }
        if ((HAL_GetTick() - tickstart) > timeout_ms) {
            return 1;  /* 超时 */
        }
    }
    *data = (uint8_t)(STX_UART->Instance->RDR & 0xFF);
    return 0;  /* 成功 */
}

/**
 * @brief  清空RX路径: 清错误标志并排空RDR残留字节
 * @说明    XR5000_FIX_20260805: 必须在每次发起查询/发送前调用,清除上一次事务
 *          未读残留(含超时后才到达的应答字节)。否则残留字节与新应答叠加会
 *          触发 ORE,进而阻塞 RXNE 导致本次事务必然失败。
 *          先写ICR清PE/FE/NE/ORE/IDLE错误标志, 再循环读RDR清RXNE.
 */
static void StorageTx_FlushRx(void)
{
    /* 清除 PE/FE/NE/ORE/IDLE 错误标志 (ICR 写1清零) */
    STX_UART->Instance->ICR = 0x0000001FU;
    /* 排空 RDR 中的残留字节 (读 RDR 自动清 RXNE) */
    while (__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_RXNE)) {
        (void)STX_UART->Instance->RDR;
    }
}

/*==============================================================
 * CRC16 (多项式0xA001, MODBUS CRC16)
 *============================================================*/
/**
 * @brief   计算MODBUS CRC16(多项式0xA001)
 * @param   data: 待校验数据
 * @param   len:  数据长度
 * @retval  CRC16值(小端存储)
 * @说明    初值0xFFFF, 逐字节异或后按位移8次, 最低位为1则右移异或0xA001.
 *          与存储侧约定一致, 用于帧校验.
 */
static uint16_t StorageTx_CRC16(const uint8_t *data, uint16_t len)
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
    return crc;  /* 小端存储 */
}

/*==============================================================
 * 帧发送
 *============================================================*/

/**
 * @brief  组装并发送一个完整STX帧
 * @param  cmd: 命令码
 * @param  payload: 数据载荷指针(可为NULL)
 * @param  payload_len: 数据载荷长度
 * @说明   帧结构: [0xA5][len][cmd][payload...][CRC16_lo][CRC16_hi][0x5A]
 *         len = 命令码(1) + payload_len; CRC16范围从len字节到最后一字节payload,
 *         小端存储. 组装到本地buf后逐字节发送.
 */
static void StorageTx_SendFrame(uint8_t cmd, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t buf[260];  /* 最大: 1+1+1+255+2+1 = 261 */
    uint16_t idx = 0;
    uint16_t crc;
    uint16_t crc_calc_len;  /* CRC范围: len + cmd + payload */

    /* 帧头 */
    buf[idx++] = STX_FRAME_HEAD;

    /* len = 命令码(1) + payload_len */
    buf[idx++] = (uint8_t)(1 + payload_len);

    /* 命令码 */
    buf[idx++] = cmd;

    /* 数据载荷 */
    if (payload_len > 0 && payload != NULL) {
        memcpy(&buf[idx], payload, payload_len);
        idx += payload_len;
    }

    /* CRC16: 范围 = len字节(1) + cmd字节(1) + payload */
    crc_calc_len = 1 + 1 + payload_len;
    crc = StorageTx_CRC16(&buf[1], crc_calc_len);  /* 从len字节开始计算 */

    /* CRC小端存储 */
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    /* 帧尾 */
    buf[idx++] = STX_FRAME_TAIL;

    /* 逐字节发送整帧 */
    for (uint16_t i = 0; i < idx; i++) {
        StorageTx_SendByte(buf[i]);
    }
}

/*==============================================================
 * 公开API
 *============================================================*/

/**
 * @brief   同步发送一条事件记录到存储侧(阻塞, 含3次重试)
 * @param   cmd: 命令码(STX_CMD_STORE_xxx)
 * @param   record: 17字节事件记录指针
 * @retval  0=成功, 1=无应答/超时, 2=CRC错误(已重试), 3=存储已满(不重试), 4=忙
 * @说明    每次重试前调用FlushRx清空RX残留, 防止上次超时后才到达的应答触发ORE.
 *          应答帧格式: [0xA5][len][cmd_echo][ack_code][CRC16][0x5A].
 *          存储已满(STX_ACK_ERR_FULL)直接返回不重试.
 */
uint8_t StorageTx_SendRecord(uint8_t cmd, const EventRecord_t *record)
{
    if (!s_initialized || record == NULL) {
        return 1;
    }

    uint8_t ret = 1;
    uint8_t retry;

    for (retry = 0; retry < STX_RETRY_COUNT; retry++) {
        /* XR5000_FIX_20260805: 每次发送前清空 RX 残留,防止上一次超时后才
         * 到达的应答字节与新应答叠加触发 ORE。 */
        StorageTx_FlushRx();
        /* 发送请求帧 */
        StorageTx_SendFrame(cmd, (const uint8_t *)record, sizeof(EventRecord_t));

        /* 等待应答: [0xA5][len][cmd_echo][ack_code][CRC16][0x5A] */
        uint8_t head;
        if (StorageTx_WaitByte(&head, STX_TIMEOUT_MS) != 0) {
            ret = 1;  /* 无应答 */
            continue;
        }
        if (head != STX_FRAME_HEAD) {
            ret = 1;
            continue;
        }

        uint8_t len;
        if (StorageTx_WaitByte(&len, STX_TIMEOUT_MS) != 0) {
            ret = 1;
            continue;
        }

        uint8_t cmd_echo;
        if (StorageTx_WaitByte(&cmd_echo, STX_TIMEOUT_MS) != 0) {
            ret = 1;
            continue;
        }

        uint8_t ack_code;
        if (StorageTx_WaitByte(&ack_code, STX_TIMEOUT_MS) != 0) {
            ret = 1;
            continue;
        }

        /* 消费CRC与帧尾(不校验, 仅读出) */
        uint8_t dummy;
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* CRC低字节 */
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* CRC高字节 */
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* 帧尾 */

        /* 检查应答码 */
        if (ack_code == STX_ACK_OK) {
            return 0;  /* 成功 */
        } else if (ack_code == STX_ACK_ERR_CRC) {
            ret = 2;  /* CRC错误, 重试 */
        } else if (ack_code == STX_ACK_ERR_FULL) {
            return 3;  /* 存储已满, 不重试 */
        } else if (ack_code == STX_ACK_ERR_BUSY) {
            ret = 4;  /* 忙 */
        } else {
            ret = 1;
        }
    }

    /* XR5000_STX_DIAG_20260811: 调试-发送结果统计(0=ACK成功,1=超时,2=CRC错,3=满,4=忙) */
    DebugPrintf("[STX-TX] cmd=%d result=%d\r\n", cmd, ret);

    return ret;
}

/**
 * @brief   查询存储侧剩余容量(阻塞, 含3次重试)
 * @param   remaining: 输出剩余容量条数
 * @retval  0=成功, 1=全部重试失败
 * @说明    XR5000_FIX_20260805: 与 SendRecord 一致加重试,吸收首次事务的竞态
 *          失败(如启动后首查、偶发超时)。每次重试前 FlushRx 清空残留。
 *          应答帧格式: [0xA5][len][cmd_echo][remaining 4字节小端][CRC16][0x5A].
 */
uint8_t StorageTx_QueryCapacity(uint32_t *remaining)
{
    if (!s_initialized || remaining == NULL) {
        return 1;
    }

    /* XR5000_FIX_20260805: 与 SendRecord 一致加重试,吸收首次事务的竞态失败
     * (如启动后首查、偶发超时)。每次重试前 FlushRx 清空残留。 */
    uint8_t retry;
    for (retry = 0; retry < STX_RETRY_COUNT; retry++) {
        StorageTx_FlushRx();
        /* 发送查询命令 */
        StorageTx_SendFrame(STX_CMD_QUERY_CAPACITY, NULL, 0);

        /* 等待应答: [0xA5][len][cmd_echo][remaining 4字节小端][CRC16][0x5A] */
        uint8_t head;
        if (StorageTx_WaitByte(&head, STX_TIMEOUT_MS) != 0) continue;
        if (head != STX_FRAME_HEAD) continue;

        uint8_t len;
        if (StorageTx_WaitByte(&len, STX_TIMEOUT_MS) != 0) continue;

        uint8_t cmd_echo;
        if (StorageTx_WaitByte(&cmd_echo, STX_TIMEOUT_MS) != 0) continue;

        /* 读取4字节剩余容量(小端) */
        uint8_t cap[4];
        uint8_t fail = 0;
        for (int i = 0; i < 4; i++) {
            if (StorageTx_WaitByte(&cap[i], STX_TIMEOUT_MS) != 0) { fail = 1; break; }
        }
        if (fail) continue;

        *remaining = (uint32_t)cap[0] | ((uint32_t)cap[1] << 8) |
                     ((uint32_t)cap[2] << 16) | ((uint32_t)cap[3] << 24);

        /* 消费CRC与帧尾 */
        uint8_t dummy;
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);

        return 0;  /* 成功 */
    }

    return 1;  /* 全部重试失败 */
}

/**
 * @brief   发送心跳到存储侧(不等应答)
 * @retval  0=已发送, 1=未初始化
 * @说明    心跳帧无载荷, 发送后不等待应答.
 */
uint8_t StorageTx_Heartbeat(void)
{
    if (!s_initialized) {
        return 1;
    }
    StorageTx_SendFrame(STX_CMD_HEARTBEAT, NULL, 0);
    /* 心跳不等待应答 */
    return 0;
}

/**
 * @brief   填充记录时间戳(读取BM8563实时钟)
 * @param   rec: 待填充时间戳的记录
 * @说明    先读RTC到SystemTime, 再写入记录. 年份存储为完整年份-2000
 *         (如2026 -> 26).
 */
void StorageTx_FillTimestamp(EventRecord_t *rec)
{
    if (rec == NULL) return;

    /* 读取RTC(BM8563)到SystemTime */
    getBM8563TimeToSystemTime();

    /* 填充时间戳(年 = 完整年份 - 2000, 如 2026 -> 26) */
    rec->year   = (uint8_t)(SystemTime.year - 2000);
    rec->month  = SystemTime.month;
    rec->day    = SystemTime.day;
    rec->hour   = SystemTime.hours;
    rec->minute = SystemTime.minutes;
    rec->second = SystemTime.seconds;
}

/**
 * @brief   构造完整事件记录结构(含RTC时间戳)
 * @param   rec: 待填充记录
 * @param   dev_no: 设备号
 * @param   dev_type: 设备类型代码
 * @param   event_code: 事件代码
 * @param   state_code: 状态代码
 * @说明    先清零记录, 再填固定字段(控制器号=1, 单元号=1, 通道号=0)与
 *         传入字段, 最后调用FillTimestamp填入RTC时间.
 */
void StorageTx_BuildRecord(EventRecord_t *rec,
                           uint8_t dev_no,
                           uint16_t dev_type,
                           uint16_t event_code,
                           uint16_t state_code)
{
    if (rec == NULL) return;

    memset(rec, 0, sizeof(EventRecord_t));

    /* 控制器号: 固定1 */
    rec->controller_no = 1;

    /* 单元号: 固定1 */
    rec->unit_no = 1;

    /* 设备号 */
    rec->device_no = dev_no;

    /* 通道号: 固定0 */
    rec->channel_no = 0;

    /* 设备类型代码 */
    rec->dev_type = dev_type;

    /* 事件代码 */
    rec->event_code = event_code;

    /* 状态代码 */
    rec->state_code = state_code;

    /* 填充时间戳 */
    StorageTx_FillTimestamp(rec);
}

/*==============================================================
 * 队列API (任务安全)
 *============================================================*/

/**
 * @brief   异步入队一条事件记录(非阻塞, 任务安全)
 * @param   cmd: 命令码(STX_CMD_STORE_xxx)
 * @param   record: 17字节事件记录指针
 * @retval  0=入队成功, 1=队列满(已丢最旧后入队或仍失败), 2=未初始化
 * @说明    适合中断/任务调用. 队列满时丢弃最旧一条再重试入队, 保证最新
 *          事件优先. 真正发送由StorageTx_TaskLoop完成.
 */
uint8_t StorageTx_QueueRecord(uint8_t cmd, const EventRecord_t *record)
{
    if (!s_initialized || s_tx_queue == NULL || record == NULL) {
        return 2;  /* 未初始化 */
    }

    StorageTx_QueueItem_t item;
    item.cmd = cmd;
    memcpy(&item.record, record, sizeof(EventRecord_t));

    /* XR5000_STX_DIAG_20260811: 调试-队列记录信息打印 */
    DebugPrintf("[STX-Q ] cmd=%d ctl=%d unit=%d dev=%d ch=%d type=%d evt=%d st=%d\r\n",
                cmd, record->controller_no, record->unit_no, record->device_no,
                record->channel_no, record->dev_type, record->event_code, record->state_code);

    /* 尝试入队(非阻塞) */
    if (xQueueSend(s_tx_queue, &item, 0) == pdTRUE) {
        return 0;  /* 入队成功 */
    }

    /* 队列满: 丢弃最旧一条后重试 */
    StorageTx_QueueItem_t old;
    xQueueReceive(s_tx_queue, &old, 0);
    if (xQueueSend(s_tx_queue, &item, 0) == pdTRUE) {
        return 1;  /* 已入队, 最旧被丢弃 */
    }

    return 1;  /* 入队失败 */
}

/**
 * @brief   存储发送任务主循环(需在专用FreeRTOS任务中调用)
 * @说明    未初始化时休眠1秒返回. 阻塞等待队列, 收到记录后调用SendRecord
 *          发送(内含重试与ACK), 发送完短暂让出CPU(10ms)给其它任务.
 */
void StorageTx_TaskLoop(void)
{
    if (!s_initialized || s_tx_queue == NULL) {
        /* 未初始化, 休眠 */
        vTaskDelay(1000);
        return;
    }

    StorageTx_QueueItem_t item;

    /* 阻塞等待队列项到达 */
    if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
        /* 发送记录(内含重试与应答) */
        StorageTx_SendRecord(item.cmd, &item.record);

        /* 让出CPU给其它任务 */
        vTaskDelay(10);
    }
}
