/**
 * @file    bsp_storage_tx.c
 * @brief   存储通信发送模块 - 经LPUART1(PB6/PB7, 115200 8N1)与
 *          存储端MCU通信的发送/接收/重试模块.
 *
 * @details
 *   硬件链路:
 *     UART:  LPUART1 PB6=TX, PB7=RX, 115200 8N1
 *            手动初始化LPUART1(CubeMX未配置LPUART1), 采用HAL寄存器级驱动.
 *     Flash: 存储端MCU管理SPI Flash W25Q256, 经USB转接.
 *
 *   帧格式(主机 -> 存储端):
 *     [0xA5][len][cmd][payload...][CRC16_lo][CRC16_hi][0x5A]
 *     len  = 命令码字节数(1) + 数据载荷字节数  (不含帧头/帧尾/CRC)
 *     CRC16校验范围: 从len字节到payload末尾, 多项式0xA001, 小端存储.
 *
 *   硬件问题(ORE溢出):
 *     H7 LPUART的ORE(溢出错误)标志位可能被错误置位而阻塞RXNE, 导致RX接收无效.
 *     本模块的应对bug: WaitByte中检测到ORE, 调用FlushRx清空一次接收缓冲
 *     并重新等待RDR数据, 通过多次重试策略避免ORE.
 *
 *   时间戳来源:
 *     bsp_rtc.h   - SystemTime, getBM8563TimeToSystemTime() 获取RTC时间
 *     FreeRTOS    - 任务调度/队列阻塞
 *
 *   调用方式:
 *     StorageTx_QueueRecord()  中断/任务上下文(队列发送)
 *     StorageTx_TaskLoop()     在FreeRTOS任务中循环(阻塞等待队列)
 */
#include "bsp_storage_tx.h"
#include "bsp_rtc.h"            /* SystemTime, getBM8563TimeToSystemTime() */
#include "bsp_debug.h"          /* XR5000_STX_DIAG_20260811: 调试打印输出(COM4) */
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*==============================================================
 * 内部变量
 *============================================================*/
/* 存储通信UART: LPUART1 (PB6=TX, PB7=RX, 115200 8N1), 手动初始化驱动 */
static UART_HandleTypeDef s_hlpuart1;
#define STX_UART             (&s_hlpuart1)
static uint8_t s_initialized = 0;           /* 初始化标志 */
static uint8_t s_boot_completed = 0;        /* P0-4: 启动后首次发送成功标志(未成功前不报存储故障) */
static uint8_t s_fault_reported = 0;        /* P0-4: 存储故障已上报标志(去重,避免故障刷屏) */
static QueueHandle_t s_tx_queue = NULL;     /* 发送消息队列句柄 */

/* 队列项: 命令码 + 事件记录 */
typedef struct {
    uint8_t cmd;
    EventRecord_t record;
} StorageTx_QueueItem_t;

/*==============================================================
 * 内部函数声明
 *============================================================*/
static void StorageTx_InitLPUART1(void);
static void StorageTx_SendByte(uint8_t data);
static uint8_t StorageTx_WaitByte(uint8_t *data, uint32_t timeout_ms);
static void StorageTx_FlushRx(void);
static uint16_t StorageTx_CRC16(const uint8_t *data, uint16_t len);
static void StorageTx_SendFrame(uint8_t cmd, const uint8_t *payload, uint16_t payload_len);

/*==============================================================
 * LPUART1驱动层 (PB6=TX, PB7=RX, 115200 8N1)
 *============================================================*/
/**
 * @brief  手动初始化LPUART1的PB6/PB7引脚, 同时设置UE/TE/RE位
 * @note   初始化步骤:
 *           1. 配置LPUART1时钟源 = D3PCLK1 (PCLK4)
 *           2. 使能GPIOB和LPUART1时钟
 *           3. 设置PB6/PB7为复用功能AF8(LPUART)
 *           4. HAL_UART_Init配置115200/8N1/TX+RX/无流控
 *         问题: HAL_UART_Init不使能USART; 需要手动设置UE/TE/RE位后
 *         RX才能接收数据, 否则无法接收, 需手动操作UE/TE/RE寄存器. 此处手动
 *        开启RE(H7 LPUART1有些特殊寄存器位可能被HAL库遗漏的RE).
 */
static void StorageTx_InitLPUART1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* 配置LPUART1时钟源 = D3PCLK1 (PCLK4) */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_D3PCLK1;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* 使能GPIOB和LPUART1时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_LPUART1_CLK_ENABLE();

    /* PB6=TX, PB7=RX, 配置复用功能AF8 */
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
    /* XR5000_FIX_20260805: HAL_UART_Init 不会使能USART; 需要手动设置 RE 位使能接收,
       否则 RX 无法接收数据. 因此手动设置 TE/RE/UE位 */
    if (HAL_UART_Init(&s_hlpuart1) != HAL_OK)
    {
        /* 注:使能 UE/TE/RE,确保 HAL 初始化完成后 RX 能够接收 */
        SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_UE);
        SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_TE | USART_CR1_RE);
    }
    /* 追加:即使 HAL_UART_Init 成功,仍需确保 RE 位已使能
       (H7 LPUART1 有些特殊寄存器位可能被 HAL_UART_Init 遗漏的 RE) */
    SET_BIT(s_hlpuart1.Instance->CR1, USART_CR1_RE);
}

/*==============================================================
 * 初始化
 *============================================================*/

/**
 * @brief  初始化存储通信模块(LPUART1 + 消息队列)
 * @note   注意: 该函数可重复调用. 首次调用初始化LPUART1, 后续直接返回.
 */
void StorageTx_Init(void)
{
    if (s_initialized) return;

    /* 初始化LPUART1 (PB6/PB7, 115200 8N1) */
    StorageTx_InitLPUART1();

    /* 创建发送消息队列 */
    if (s_tx_queue == NULL) {
        s_tx_queue = xQueueCreate(STX_QUEUE_DEPTH, sizeof(StorageTx_QueueItem_t));
    }

    s_initialized = 1;
}

/*==============================================================
 * 字节级收发
 *============================================================*/

/**
 * @brief  发送单个字节(阻塞轮询)
 * @param  data: 待发送的字节数据
 * @note   等待TXE(发送数据寄存器空)后写TDR, 再等待TC(发送完成)后返回.
 *         超时后放弃发送, 不阻塞调用者.
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
 * @brief   等待接收一个字节(轮询超时)
 * @param   data: 接收到的字节存放位置
 * @param   timeout_ms: 超时时间(ms)
 * @retval  0=接收成功, 1=超时
 * @note    XR5000_FIX_20260805: H7 LPUART 的 ORE(溢出)标志可能被错误置位而
 *          阻塞 RXNE, 导致接收卡死. 本函数在等待 RXNE 循环中检测到 ORE,
 *          则清除一次 ORE 标志并重新等待, 通过多次重试避免 ORE 阻塞问题.
 */
static uint8_t StorageTx_WaitByte(uint8_t *data, uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(STX_UART, UART_FLAG_RXNE)) {
        /* ORE 置位时 RXNE 不会再被置位,检测到 ORE 则清除后继续等待 */
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
 * @brief  清空RX缓冲: 清除标志位并读取RDR清空数据
 * @note    XR5000_FIX_20260805: 每次发送查询/记录前调用,防止上一次残留
 *          未读完的应答数据导致后续应答解析偏移或触发 ORE,
 *          进而阻塞 RXNE 使后续接收永久失效。
 *          先写ICR清PE/FE/NE/ORE/IDLE标志位, 再循环读RDR清RXNE.
 */
static void StorageTx_FlushRx(void)
{
    /* 清 PE/FE/NE/ORE/IDLE 错误标志 (ICR 写1清除) */
    STX_UART->Instance->ICR = 0x0000001FU;
    /* 清空 RDR 中的残余数据 (读 RDR 自动清 RXNE) */
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
 * @note    初值0xFFFF, 每字节异或后移位8次, 最低位为1则异或0xA001.
 *          小端存储约定一致, 用于帧校验.
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
 * @brief  封装并发送一条完整STX帧
 * @param  cmd: 命令码
 * @param  payload: 数据载荷指针(可NULL)
 * @param  payload_len: 数据载荷长度
 * @note   帧结构: [0xA5][len][cmd][payload...][CRC16_lo][CRC16_hi][0x5A]
 *         len = 命令码(1) + payload_len; CRC16范围从len字节到最后一个payload,
 *         小端存储. 封装到buf后逐字节发送.
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
 * 发送API
 *============================================================*/

/**
 * @brief   同步发送一条事件记录到存储端(阻塞, 最多3次重试)
 * @param   cmd: 命令码(STX_CMD_STORE_xxx)
 * @param   record: 17字节事件记录指针
 * @retval  0=成功, 1=无应答/超时, 2=CRC错误(存储端), 3=存储已满(不重试), 4=忙
 * @note    每次发送前调用FlushRx清RX缓冲, 防止上次残留应答导致解析偏移触发ORE.
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
        /* XR5000_FIX_20260805: 每次发送前清空 RX 缓冲,防止上一次残留
         * 的应答数据导致后续应答解析偏移或触发 ORE */
        StorageTx_FlushRx();
        /* 发送数据帧 */
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

        /* 跳过CRC和帧尾(不校验, 信任存储端) */
        uint8_t dummy;
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* CRC低字节 */
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* CRC高字节 */
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);  /* 帧尾 */

        /* 解析应答码 */
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

    /* XR5000_STX_DIAG_20260811: 调试-打印发送结果(0=ACK成功,1=超时,2=CRC错,3=满,4=忙) */
    DebugPrintf("[STX-TX] cmd=%d result=%d\r\n", cmd, ret);

    return ret;
}

/**
 * @brief   查询存储端剩余容量(阻塞, 最多3次重试)
 * @param   remaining: 输出剩余容量值
 * @retval  0=成功, 1=全部重试失败
 * @note    XR5000_FIX_20260805: 同 SendRecord 一样需要处理,容易遇到静态
 *          失败(设备状态碰撞、偶发超时)故每次发送前 FlushRx 清空缓冲.
 *          应答帧格式: [0xA5][len][cmd_echo][remaining 4字节小端][CRC16][0x5A].
 */
uint8_t StorageTx_QueryCapacity(uint32_t *remaining)
{
    if (!s_initialized || remaining == NULL) {
        return 1;
    }

    /* XR5000_FIX_20260805: 同 SendRecord 一样需要处理,容易遇到静态失败
     * (设备状态碰撞、偶发超时)故每次发送前 FlushRx 清空缓冲 */
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

        /* 跳过CRC和帧尾 */
        uint8_t dummy;
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);
        StorageTx_WaitByte(&dummy, STX_TIMEOUT_MS);

        return 0;  /* 成功 */
    }

    return 1;  /* 全部重试失败 */
}

/**
 * @brief   发送心跳到存储端(无应答等待)
 * @retval  0=已发送, 1=未初始化
 * @note    只发送帧不等待应答, 发送后立即返回.
 */
uint8_t StorageTx_Heartbeat(void)
{
    if (!s_initialized) {
        return 1;
    }
    StorageTx_SendFrame(STX_CMD_HEARTBEAT, NULL, 0);
    /* 不等待应答 */
    return 0;
}

/**
 * @brief   填充记录时间戳(读取BM8563实时时钟)
 * @param   rec: 待填充时间的记录
 * @note    先读RTC到SystemTime, 再写入记录. 年份存储为公历年-2000
 *         (如2026 -> 26).
 */
void StorageTx_FillTimestamp(EventRecord_t *rec)
{
    if (rec == NULL) return;

    /* 读取RTC(BM8563)到SystemTime */
    getBM8563TimeToSystemTime();

    /* 填充时间值(年 = 公历年 - 2000, 如 2026 -> 26) */
    rec->year   = (uint8_t)(SystemTime.year - 2000);
    rec->month  = SystemTime.month;
    rec->day    = SystemTime.day;
    rec->hour   = SystemTime.hours;
    rec->minute = SystemTime.minutes;
    rec->second = SystemTime.seconds;
}

/**
 * @brief  按事件代码自动填充状态位图(GB4717-2024表C.18)
 * @param  rec: 待填充的记录(根据rec->event_code查表填rec->state_code)
 * @note   P1-2整改: state_code位图定义(表C.18, bit0~bit10):
 *           bit3(0x0008)=有报警   bit4(0x0010)=有启动   bit5(0x0020)=有反馈
 *           bit7(0x0080)=有故障   bit8(0x0100)=有屏蔽   bit2(0x0004)=电源故障
 *         映射: 火警类事件置bit3, 故障置bit7, 屏蔽置bit8,
 *         关机置bit2, 启动类置bit4, 反馈置bit5, 其余事件清零.
 */
void StorageTx_FillStateMask(EventRecord_t *rec)
{
    uint16_t mask = 0;

    if (rec == NULL) return;

    switch (rec->event_code)
    {
    case EVT_FIRST_FIRE:            /* 2 首警 */
    case EVT_FIRE:                  /* 3 火警 */
    case EVT_CONFIRM_BUTTON:        /* 128 信息确认(确认时系统处于报警状态) */
        mask = 0x0008;              /* bit3 有报警 */
        break;

    case EVT_START:                 /* 19 联动设备启动 */
    case EVT_LINKAGE_START_BUTTON:  /* 130 联动启动按钮 */
        mask = 0x0010;              /* bit4 有启动 */
        break;

    case EVT_FEEDBACK:              /* 26 反馈 */
        mask = 0x0020;              /* bit5 有反馈 */
        break;

    case EVT_FAULT:                 /* 80 故障 */
        mask = 0x0080;              /* bit7 有故障 */
        break;

    case EVT_SHIELD:                /* 72 屏蔽 */
        mask = 0x0100;              /* bit8 有屏蔽 */
        break;

    case EVT_POWER_OFF:             /* 121 关机(掉电前记录) */
        mask = 0x0004;              /* bit2 电源故障 */
        break;

    case EVT_NORMAL:                /* 1 正常 */
    case EVT_FAULT_RECOVER:         /* 100 故障恢复 */
    case EVT_SHIELD_RELEASE:        /* 73 解除屏蔽 */
    case EVT_POWER_ON:              /* 120 开机 */
    case EVT_RESET:                 /* 122 复位 */
    case EVT_SELF_CHECK:            /* 123 自检 */
    case EVT_SELF_CHECK_FAIL:       /* 124 自检失败 */
    case EVT_CHECK_BUTTON:          /* 129 检查按钮 */
    case EVT_CLOCK_ADJUST:          /* 131 时钟调整 */
    case EVT_MANUAL:                /* 125 手动 */
    case EVT_AUTO:                  /* 126 自动 */
    case EVT_SUPERVISED:            /* 70 监管 */
    case EVT_SUPERVISED_RELEASE:    /* 71 监管解除 */
    default:
        mask = 0x0000;              /* 其余事件不置状态位 */
        break;
    }

    rec->state_code = mask;
}

/**
 * @brief   构建完整事件记录结构(含RTC时间戳)
 * @param   rec: 输出记录
 * @param   dev_no: 设备号
 * @param   dev_type: 设备类型码
 * @param   event_code: 事件代码
 * @param   state_code: 状态代码
 * @note    填充固定字段(控制器号=1, 单元号=1, 通道号=0)和
 *         可变字段, 最后调用FillTimestamp填入RTC时间.
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

    /* 设备类型码 */
    rec->dev_type = dev_type;

    /* 事件代码 */
    rec->event_code = event_code;

    /* 状态代码 */
    rec->state_code = state_code;

    /* 填充时间戳 */
    StorageTx_FillTimestamp(rec);
}

/*==============================================================
 * 异步API (线程安全)
 *============================================================*/

/**
 * @brief   异步发送一条事件记录(入队列, 线程安全)
 * @param   cmd: 命令码(STX_CMD_STORE_xxx)
 * @param   record: 17字节事件记录指针
 * @retval  0=入队成功, 1=入队失败(队列满丢弃最旧记录), 2=未初始化
 * @note    适合中断/任务上下文. 队列满时丢弃最旧一条记录再尝试, 保证最新
 *          事件不丢失. 实际发送由StorageTx_TaskLoop处理.
 */
uint8_t StorageTx_QueueRecord(uint8_t cmd, const EventRecord_t *record)
{
    if (!s_initialized || s_tx_queue == NULL || record == NULL) {
        return 2;  /* 未初始化 */
    }

    StorageTx_QueueItem_t item;
    item.cmd = cmd;
    memcpy(&item.record, record, sizeof(EventRecord_t));

    /* XR5000_STX_DIAG_20260811: 调试-打印队列记录详情 */
    DebugPrintf("[STX-Q ] cmd=%d ctl=%d unit=%d dev=%d ch=%d type=%d evt=%d st=%d\r\n",
                cmd, record->controller_no, record->unit_no, record->device_no,
                record->channel_no, record->dev_type, record->event_code, record->state_code);

    /* 尝试入队(不等待) */
    if (xQueueSend(s_tx_queue, &item, 0) == pdTRUE) {
        return 0;  /* 入队成功 */
    }

    /* 队列满: 丢弃最旧一条再尝试 */
    StorageTx_QueueItem_t old;
    xQueueReceive(s_tx_queue, &old, 0);
    if (xQueueSend(s_tx_queue, &item, 0) == pdTRUE) {
        return 1;  /* 成功, 丢弃最旧记录 */
    }

    return 1;  /* 入队失败 */
}

/**
 * @brief   存储通信任务主循环(在专用FreeRTOS任务中调用)
 * @note    未初始化时延时1秒返回. 阻塞等待队列, 收到记录后调用SendRecord
 *          同步发送(内含等待ACK), 发送完成后让出CPU(10ms)给其他任务.
 */
void StorageTx_TaskLoop(void)
{
    if (!s_initialized || s_tx_queue == NULL) {
        /* 未初始化, 等待 */
        vTaskDelay(1000);
        return;
    }

    StorageTx_QueueItem_t item;

    /* 阻塞等待队列消息 */
    if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
        uint8_t send_ret;

        /* 发送记录(内含等待应答) */
        send_ret = StorageTx_SendRecord(item.cmd, &item.record);

        /* P0-4整改: 存储写入故障上报(失败码: 1超时/2CRC错/3满/4忙) */
        if (send_ret == 0U)
        {
            /* 发送成功: 标记启动完成; 若此前报过存储故障, 清标志(故障恢复) */
            s_boot_completed = 1;
            if (s_fault_reported != 0U)
            {
                s_fault_reported = 0;
                DebugPrintf("[STX-TX] storage fault recovered\r\n");
            }
        }
        else
        {
            /* 发送失败: 首次成功后才上报(开机阶段存储侧未就绪属正常, 不误报) */
            if ((s_boot_completed != 0U) && (s_fault_reported == 0U))
            {
                EventRecord_t fault_rec;

                memset(&fault_rec, 0, sizeof(EventRecord_t));
                fault_rec.controller_no = 1;                 /* 控制器号固定1 */
                fault_rec.unit_no       = 1;
                fault_rec.device_no     = 1;                 /* 存储单元设备号1 */
                fault_rec.dev_type      = DEV_TYPE_STORAGE;  /* 18=运行数据存储单元(表C.16) */
                fault_rec.event_code    = EVT_FAULT;         /* 80=故障 */
                fault_rec.state_code    = 0x0080;            /* bit7有故障(表C.18) */
                StorageTx_FillTimestamp(&fault_rec);         /* 填充RTC时间戳 */

                /* 故障记录入队(只入队不发送, 无递归风险); 下轮发送若成功即清标志 */
                (void)StorageTx_QueueRecord(STX_CMD_STORE_FAULT, &fault_rec);
                s_fault_reported = 1;                        /* 去重: 故障期间只报一次 */
                DebugPrintf("[STX-TX] storage fault reported (ret=%d evt=80 type=18)\r\n", send_ret);
            }
        }

        /* 让出CPU给其他任务 */
        vTaskDelay(10);
    }
}
