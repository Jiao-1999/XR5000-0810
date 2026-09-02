/*==============================================================
 * 文件名    : bsp_fecbus_rx.c
 * 功能描述  : FECbus 接收解析协议层（环形缓冲 + 帧解析 + 分发）
 * 适用平台  : STM32H723ZGT6, Keil MDK-ARM
 * 实现方式  : USART3 逐字节 IT 接收 -> 环形缓冲 -> 状态机解析
 *              (0x7E 帧头 + CRC16 校验 + 功能码分发) -> 业务分发
 *              并回 0FH 状态应答帧完成协议层闭环
 * 适用范围  : 仅协议层框架，不涉及具体业务上报/写入；
 *             业务功能 case 仅留 TODO 注释 + 统一回 0FH 应答
 * 相关模块  :
 *   - bsp_fecbus.h   : FT/常量定义, Fecbus_SendRawFrame/Fecbus_CalcCRC16
 *   - bsp_debug.h    : DebugPrintf 调试打印
 *   - usart.h        : huart3
 *==============================================================*/

#include "bsp_fecbus.h"
#include "bsp_fecbus_rx.h"
#include "bsp_debug.h"
#include "usart.h"
#include <string.h>

/*--------------------------------------------------------------
 * 常量定义
 *--------------------------------------------------------------*/
#define FECBUS_RX_RING_SIZE  512   /* 环形缓冲大小 */
#define FECBUS_HEADER_LEN    7     /* FT DA PA SA MN TN DLC = 7字节 */
#define FECBUS_RX_MAX_DLC    8     /* GB4717: DLC 范围 1~8 */
#define FECBUS_CRC_LEN       2     /* CRC16 2字节 */

/*--------------------------------------------------------------
 * 接收状态机
 *--------------------------------------------------------------*/
typedef enum {
    RX_STATE_WAIT_HEAD = 0,   /* 等待帧头 0x7E */
    RX_STATE_HEADER,          /* 接收帧头 7字节 */
    RX_STATE_DATA,            /* 收 DLC 个数据 */
    RX_STATE_CRC,             /* 收 CRC 2字节 */
    RX_STATE_TAIL             /* 收帧尾 0x7E */
} RxState_t;

/*--------------------------------------------------------------
 * 全局/静态变量
 *--------------------------------------------------------------*/
/* USART3 IT 接收当前字节（bsp_itcallback.c 中断中调用 FecbusRx_OnByte） */
volatile uint8_t g_fecbus_rx_byte;
/* USART1 IT 接收当前字节（主机通道收从机应答, 与 USART3 共用环形缓冲/状态机） */
volatile uint8_t g_fecbus_rx_byte_u1;

/* 环形缓冲（中断写 / 任务读） */
static uint8_t           s_rx_ring[FECBUS_RX_RING_SIZE];
static volatile uint16_t s_rx_head;   /* 写指针（中断更新） */
static volatile uint16_t s_rx_tail;   /* 读指针（任务更新） */

/* 帧解析状态 */
static uint8_t           s_state    = RX_STATE_WAIT_HEAD;
static FecbusRxFrame_t   s_frame;
static uint8_t           s_hdr_idx;
static uint8_t           s_data_idx;
static uint8_t           s_crc_idx;
static uint16_t          s_rx_crc;

/* 0FH 应答标志（供发送侧 Fecbus_SendFrameWithAck 查询） */
static volatile uint8_t  s_ack_ok;
static volatile uint8_t  s_ack_mn;

/*--------------------------------------------------------------
 * 内部函数声明
 *--------------------------------------------------------------*/
static void FecbusRx_FeedByte(uint8_t byte);
static void FecbusRx_Dispatch(const FecbusRxFrame_t *f);
static void FecbusRx_PrintFrame(const FecbusRxFrame_t *f);

/*--------------------------------------------------------------
 * 中断入口
 *--------------------------------------------------------------*/
void FecbusRx_OnByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1) % FECBUS_RX_RING_SIZE);
    if (next == s_rx_tail) {
        return;  /* 环形缓冲已满，丢弃本次数据 */
    }
    s_rx_ring[s_rx_head] = byte;
    s_rx_head = next;
}

/*--------------------------------------------------------------
 * 初始化：启动 USART3 逐字节 IT 接收
 *--------------------------------------------------------------*/
void Fecbus_RxInit(void)
{
    /* 清空环形缓冲/状态机 */
    s_rx_head = 0;
    s_rx_tail = 0;
    s_state   = RX_STATE_WAIT_HEAD;
    s_ack_ok  = 0;

    /* 启动逐字节 IT 接收（1字节，RxCpltCallback 在 bsp_itcallback.c 中处理）
     * USART3 = 从机通道(收从机上报), USART1 = 主机通道(收从机应答) */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_fecbus_rx_byte, 1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_fecbus_rx_byte_u1, 1);
}

/*--------------------------------------------------------------
 * 缓冲控制
 *--------------------------------------------------------------*/
void FecbusRx_Flush(void)
{
    /* 清空环形缓冲（关中断防止与接收中断竞争） */
    __disable_irq();
    s_rx_head = 0;
    s_rx_tail = 0;
    __enable_irq();
    s_state = RX_STATE_WAIT_HEAD;
}

void FecbusRx_ResetAck(void)
{
    s_ack_ok = 0;
}

uint8_t FecbusRx_CheckAck(uint8_t mn)
{
    uint8_t ok = 0;
    __disable_irq();
    if (s_ack_ok != 0 && s_ack_mn == mn) {
        ok = 1;
        s_ack_ok = 0;   /* 只匹配一次，取用后清除 */
    }
    __enable_irq();
    return ok;
}

/*--------------------------------------------------------------
 * 轮询解析
 *--------------------------------------------------------------*/
void Fecbus_RxPoll(void)
{
    while (s_rx_head != s_rx_tail) {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1) % FECBUS_RX_RING_SIZE);
        FecbusRx_FeedByte(byte);
    }
}

/*--------------------------------------------------------------
 * 帧解析状态机
 *--------------------------------------------------------------*/
static void FecbusRx_FeedByte(uint8_t byte)
{
    switch (s_state) {
    case RX_STATE_WAIT_HEAD:
        if (byte == FECBUS_FRAME_HEAD) {
            s_state = RX_STATE_HEADER;
            s_hdr_idx = 0;
        }
        break;

    case RX_STATE_HEADER:
        switch (s_hdr_idx) {
        case 0: s_frame.ft = byte; break;
        case 1: s_frame.da = byte; break;
        case 2: s_frame.pa = byte; break;
        case 3: s_frame.sa = byte; break;
        case 4: s_frame.mn = byte; break;
        case 5: s_frame.tn = byte; break;
        default:  /* 第7字节 = DLC */
            s_frame.dlc = byte;
            if (s_frame.dlc == 0 || s_frame.dlc > FECBUS_RX_MAX_DLC) {
                /* DLC 非法（GB4717: 1~8），丢弃重新等待帧头 */
                s_state = RX_STATE_WAIT_HEAD;
            } else {
                s_data_idx = 0;
                s_state = RX_STATE_DATA;
            }
            break;
        }
        s_hdr_idx++;
        break;

    case RX_STATE_DATA:
        s_frame.data[s_data_idx++] = byte;
        if (s_data_idx >= s_frame.dlc) {
            s_crc_idx = 0;
            s_rx_crc  = 0;
            s_state = RX_STATE_CRC;
        }
        break;

    case RX_STATE_CRC:
        if (s_crc_idx == 0) {
            s_rx_crc = byte;              /* CRC 低字节在前 */
        } else {
            uint8_t calc_buf[1 + FECBUS_HEADER_LEN + FECBUS_RX_MAX_DLC];
            uint16_t calc_len = (uint16_t)(FECBUS_HEADER_LEN + s_frame.dlc);

            s_rx_crc |= (uint16_t)((uint16_t)byte << 8);

            /* 校验范围: FT .. 数据末（不含帧头/帧尾/CRC） */
            calc_buf[0] = s_frame.ft;
            calc_buf[1] = s_frame.da;
            calc_buf[2] = s_frame.pa;
            calc_buf[3] = s_frame.sa;
            calc_buf[4] = s_frame.mn;
            calc_buf[5] = s_frame.tn;
            calc_buf[6] = s_frame.dlc;
            memcpy(&calc_buf[7], s_frame.data, s_frame.dlc);

            if (Fecbus_CalcCRC16(calc_buf, calc_len) == s_rx_crc) {
                s_state = RX_STATE_TAIL;  /* CRC 正确，等待帧尾 */
            } else {
                DebugPrintf("[FECBUS-RX] CRC err mn=%d\r\n", s_frame.mn);
                s_state = RX_STATE_WAIT_HEAD;
            }
        }
        s_crc_idx++;
        break;

    case RX_STATE_TAIL:
        if (byte == FECBUS_FRAME_TAIL) {
            /* 完整帧有效 -> 分发处理 */
            FecbusRx_Dispatch(&s_frame);
        }
        s_state = RX_STATE_WAIT_HEAD;
        break;

    default:
        s_state = RX_STATE_WAIT_HEAD;
        break;
    }
}

/*--------------------------------------------------------------
 * 打印接收帧
 *--------------------------------------------------------------*/
static void FecbusRx_PrintFrame(const FecbusRxFrame_t *f)
{
    uint8_t i;
    DebugPrintf("[FECBUS-RX] ft=%d da=%d pa=%d sa=%d mn=%d tn=%d dlc=%d func=%02X",
                f->ft, f->da, f->pa, f->sa, f->mn, f->tn, f->dlc, f->data[0]);
    DebugPrintf(" data:");
    for (i = 0; i < f->dlc; i++) {
        DebugPrintf(" %02X", f->data[i]);
    }
    DebugPrintf("\r\n");
}

/*--------------------------------------------------------------
 * 0FH 状态应答（协议层闭环）
 *--------------------------------------------------------------*/
void FecbusRx_Reply(const FecbusRxFrame_t *req, uint8_t status,
                    const uint8_t *data, uint8_t len)
{
    uint8_t payload[2 + FECBUS_RX_MAX_DLC];  /* 功能码 + 状态 + 附加数据 */
    uint8_t dlc;

    if (len > FECBUS_RX_MAX_DLC) {
        len = FECBUS_RX_MAX_DLC;
    }
    dlc = (uint8_t)(2 + len);

    payload[0] = FECBUS_FUNC_RESP;   /* 0x0F */
    payload[1] = status;             /* 应答状态 */
    if (len > 0 && data != NULL) {
        memcpy(&payload[2], data, len);
    }

    /* 组帧: [0x7E][FT=1][DA=请求SA][PA=请求优先级][SA=1][MN=请求MN][TN=0][DLC][payload][CRC][0x7E] */
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, req->pa,
                        req->mn, FECBUS_TN_SINGLE, payload, dlc);

    DebugPrintf("[FECBUS-RX] reply 0FH status=%02X dlc=%d\r\n", status, dlc);
}

/*--------------------------------------------------------------
 * 帧分发：协议层闭环 + 业务 TODO 接口
 *--------------------------------------------------------------*/
static void FecbusRx_Dispatch(const FecbusRxFrame_t *f)
{
    /* 1) 应答帧(FT=1): 匹配 0FH 应答并置标志，供发送侧查询 */
    if (f->ft == FECBUS_FT_RESPONSE) {
        if (f->data[0] == FECBUS_FUNC_RESP) {
            s_ack_ok = 1;
            s_ack_mn = f->mn;
            DebugPrintf("[FECBUS-RX] RESP ok mn=%d\r\n", f->mn);
        }
        return;
    }

    /* 2) 请求帧: 仅处理发给控制器(DA=1)的帧 */
    if (f->da != FECBUS_SA_CONTROLLER) {
        return;
    }

    FecbusRx_PrintFrame(f);

    /* 3) 按功能码分发（业务接入点，仅留 TODO 注释） */
    switch (f->data[0]) {
    case 0x11:  /* 火灾报警事件上报 (3帧 TN=1/2/3) */
        /* TODO: 多帧重组 + 事件入库 StorageEvent_LogFire */
        break;
    case 0x12:  /* 故障报警事件上报 */
        /* TODO: 多帧重组 + 事件入库 StorageEvent_LogFault */
        break;
    case 0x13:  /* 其他报警事件上报 */
        /* TODO: 多帧重组 + 事件入库 */
        break;
    case 0x22:  /* 系统状态位下发 */
        /* TODO: 更新系统状态位(运行/故障/火警/监管/消音等) */
        break;
    case 0x23:  /* 联动规则下载 */
        /* TODO: 规则写入 Flash 0x080000 (LogicRule_t) */
        break;
    case 0x24:  /* 设备ID+产品编号下发 */
        /* TODO: 写入本机设备ID + 产品编号 */
        break;
    case 0x25:  /* 报警阈值下发 */
        /* TODO: 更新报警阈值 (bsp_super) */
        break;
    case 0x27:  /* 联动规则相关（同 0x23） */
        /* TODO: 规则相关处理 (同 0x23) */
        break;
    case 0x28:  /* 在线设备表下发 */
        /* TODO: 更新回路1/2/3 在线设备表 */
        break;
    case 0x29:  /* 查询RAM最新事件 */
        /* TODO: 回传 RAM 最新事件 */
        break;
    case 0x2A:  /* 查询存储侧事件 */
        /* TODO: LPUART1 查存储侧 + TN 多帧重组回传 */
        break;
    case 0x2B:  /* 中止多帧会话 */
        /* TODO: 中止当前多帧接收会话 */
        break;
    case 0x2C:  /* 查询协议版本 */
        /* TODO: 回传版本号 = 2 */
        break;
    case 0x2D:  /* 在线设备列表 */
        /* TODO: 回传在线设备列表 */
        break;
    default:
        DebugPrintf("[FECBUS-RX] unknown func=%02X\r\n", f->data[0]);
        return;
    }

    /* 4) 协议层闭环: 按 GB4717 回 0FH 状态应答（默认0x00, 附加数据后续业务填充） */
    FecbusRx_Reply(f, 0x00, NULL, 0);
}

/*==============================================================
 * 文件结束
 *==============================================================*/
