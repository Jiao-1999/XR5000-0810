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
#include "bsp_storage_event.h"   /* B1: 装置通告事件入库 API */
#include "system.h"              /* B2: SystemSaveInfo (0x22 手/自动状态位) */
#include "bsp_super.h"           /* B2: fire_alarm_threshold (0x25 设备参量) */
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

/* 应答标志（供发送侧 Fecbus_SendEventGroup 按组查询, D4） */
static volatile uint8_t  s_ack_ok;    /* 收到匹配组应答: 回显帧 或 0FH status=0 */
static volatile uint8_t  s_ack_mn;    /* 应答帧 MN */
static volatile uint8_t  s_ack_nak;   /* 收到 0FH 异常应答(status!=0): 终止重试 */
static volatile uint8_t  s_ack_func;  /* 本次发送期望被回显的功能码(SetAckFunc 设置) */

/*--------------------------------------------------------------
 * B1: 装置通告(0x11/0x12/0x13)多帧组装会话
 *--------------------------------------------------------------*/
#define FECBUS_MAX_DEV          62     /* 装置地址 2~63 (C.3.2.2), 会话按 sa-2 索引 */
#define FECBUS_ASM_TIMEOUT_MS   1000   /* 组内帧间隔超时 1s: 超时清会话(装置侧会重发整组) */

typedef struct {
    uint8_t  active;                   /* 会话进行中标志 */
    uint8_t  sa;                       /* 装置源地址 */
    uint8_t  mn;                       /* 组报文编号(4帧共用) */
    uint8_t  func;                     /* 通告功能码 0x11/0x12/0x13 */
    uint8_t  buf[17];                  /* 组装缓冲: 编号4+类型2+事件2+状态2+时间6+spare1 */
    uint8_t  next_tn;                  /* 期望下一帧序 1->2->3->0 */
    uint32_t tick;                     /* 最近一帧到达时刻(超时基准) */
} FecbusAsmSession_t;
static FecbusAsmSession_t s_asm[FECBUS_MAX_DEV];

/*--------------------------------------------------------------
 * B2: 多帧查询会话(0x2A 停等用; 0x2B 停止查询时清除)
 *--------------------------------------------------------------*/
typedef struct {
    uint8_t  active;                   /* 会话进行中标志 */
    uint8_t  func;                     /* 查询功能码 */
    uint8_t  mn;                       /* 报文编号 */
    uint8_t  sa;                       /* 查询方源地址 */
    uint16_t total;                    /* 待回传总字节数 */
    uint16_t offset;                   /* 已回传偏移 */
    uint8_t  tn;                       /* 当前帧序 */
    uint8_t  wait_ack;                 /* 0x2A 停等标志(批次8用) */
    uint32_t tick;                     /* 超时基准 */
} FecbusQuerySession_t;
static FecbusQuerySession_t s_query;

/* B2: RAM 最新事件缓存(0x29 查当前事件用; B1 组装完成时更新) */
static uint8_t s_last_evt[16];
static uint8_t s_last_evt_valid;

/*--------------------------------------------------------------
 * B2: 单帧查询(0x2C/2B/22/23/24/25)常量与外部数据源
 *--------------------------------------------------------------*/
#define FECBUS_PROTO_VER   0x0002   /* 协议版本号(0x2C 回 [00][02]; 兼作 D1 新旧固件区分) */

/* 0x24 设备标识(7字节, 生产者规定 C.4.5.12): 厂商占位, 待定正式标识 */
static const uint8_t s_dev_id[7] = { 'X', 'R', '5', '0', '0', '0', '0' };

/* 系统级状态获取(cmd_process.c 定义, 无头文件声明, 此处 extern; 用于 0x22 状态位聚合) */
extern uint8_t getCurrentSystemRunState(void);        /* 0=静默 1=底点动作 2=报警 */
extern uint8_t getCurrentSystemFaultState(void);      /* 1=系统故障 */
extern uint8_t getCurrentStartStopKeyState(void);     /* 启动/停止按键状态(非0=启动) */
extern uint8_t getOutFireSprayState(void);            /* 喷洒状态(非0=喷洒) */
extern uint8_t getCurrentMainBackupPowerState(void);  /* 1=主电正常 2=备电供电 3=备电故障 */
extern uint8_t MBusCtrl_GetOnlineCount(void);         /* 回路2(MBus)在线设备数 */
extern uint8_t RS485Detect_GetOnlineCount(void);      /* 回路3(RS485)在线设备数 */
/* 系统级状态聚合变量(bsp_internal_board.c 定义, 全局非static, 此处 extern; 0x22 bit8 屏蔽) */
extern uint8_t shielding_state;     /* 屏蔽聚合: 非0=有设备被屏蔽 (bsp_device_disable.c disabled_count 驱动, 表C.18 bit8) */

/* 在线设备枚举(0x2D 设备列表 / 0x28 注册登记用; 各 .h 已声明, 此处 extern 复用) */
#define FECBUS_MBUS_MAX_ADDR    64   /* 回路2 MBus 地址上界(有效 1~63) */
#define FECBUS_RS485_MAX_ADDR   34   /* 回路3 RS485 地址上界(有效 1~32) */
extern uint8_t  MBusCtrl_GetOnline(uint8_t addr);             /* 回路2 在线标志 */
extern uint16_t MBusCtrl_GetNationalTypeCode(uint8_t addr);   /* 回路2 国标类型码(表C.16) */
extern uint8_t  RS485Detect_GetOnline(uint8_t addr);          /* 回路3 在线标志 */
extern uint16_t RS485Detect_GetNationalTypeCode(uint8_t addr);/* 回路3 国标类型码(表C.16) */

/*--------------------------------------------------------------
 * 内部函数声明
 *--------------------------------------------------------------*/
static void FecbusRx_FeedByte(uint8_t byte);
static void FecbusRx_Dispatch(const FecbusRxFrame_t *f);
static void FecbusRx_PrintFrame(const FecbusRxFrame_t *f);
static uint8_t FecbusRx_LogEvent(uint8_t func, const uint8_t *buf);   /* B1: 事件入库分派 */
static void FecbusRx_AsmFeed(const FecbusRxFrame_t *f);               /* B1: 多帧组装状态机 */
static void FecbusRx_ReplyData(const FecbusRxFrame_t *req, uint8_t func,
                               const uint8_t *d, uint8_t len);        /* B2: 单帧带数据应答 */
static uint16_t FecbusRx_BuildDevStatus(void);                        /* B2: 0x22 状态位聚合 */
static uint16_t FecbusRx_BuildConfigCount(uint8_t level);             /* B2: 0x23 层级目标数量 */
static uint16_t FecbusRx_BuildParam(uint8_t type, uint8_t *ptype_out);/* B2: 0x25 参量阈值 */
static uint8_t FecbusRx_ReplyMulti(const FecbusRxFrame_t *req, uint8_t func,
                                   const uint8_t *data, uint16_t len); /* B2: 多帧文本应答(表C.9~C.11) */
static void FecbusRx_ReplyCurEvent(const FecbusRxFrame_t *req);        /* B2: 0x29 当前事件4帧 */
static uint16_t FecbusRx_BuildDevList(uint8_t level, uint8_t *out, uint16_t cap); /* B2: 0x2D 列表 */

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
    s_ack_nak = 0;

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
    s_ack_ok  = 0;
    s_ack_nak = 0;   /* D4: 一并清异常应答标志 */
}

void FecbusRx_SetAckFunc(uint8_t func)
{
    s_ack_func = func;   /* D4: 记录本次发送期望被回显的功能码 */
}

/* D4: 查询组应答结果(表C.6)
 *   retval 0=尚无应答, 1=收到组应答(回显功能码 或 0FH status=0), 2=NAK(0FH status!=0) */
uint8_t FecbusRx_CheckAckEx(uint8_t mn, uint8_t func)
{
    uint8_t r = 0;
    (void)func;   /* 期望功能码已由 s_ack_func 在 Dispatch 侧比对; 此处以 mn 匹配为准 */
    __disable_irq();
    if (s_ack_nak != 0 && s_ack_mn == mn) {
        r = 2;
        s_ack_nak = 0;   /* NAK: 取用后清除, 发送侧据此终止重试 */
        s_ack_ok  = 0;
    } else if (s_ack_ok != 0 && s_ack_mn == mn) {
        r = 1;
        s_ack_ok = 0;    /* 只匹配一次，取用后清除 */
    }
    __enable_irq();
    return r;
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
            uint16_t calc_len = (uint16_t)(1 + FECBUS_HEADER_LEN + s_frame.dlc);

            s_rx_crc |= (uint16_t)((uint16_t)byte << 8);

            /* 校验范围(C.6.1.6): 帧头0x7E + 报文头 + 数据（含0x7E, 不含帧尾/CRC自身） */
            calc_buf[0] = FECBUS_FRAME_HEAD;   /* D1: CRC 校验含 0x7E 帧头 */
            calc_buf[1] = s_frame.ft;
            calc_buf[2] = s_frame.da;
            calc_buf[3] = s_frame.pa;
            calc_buf[4] = s_frame.sa;
            calc_buf[5] = s_frame.mn;
            calc_buf[6] = s_frame.tn;
            calc_buf[7] = s_frame.dlc;
            memcpy(&calc_buf[8], s_frame.data, s_frame.dlc);

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
 * 应答原语 (D3: 拆分「回显应答」与「0FH 状态应答」)
 *--------------------------------------------------------------*/

/**
 * @brief  A2: 按被应答功能码确定回显帧 PA (不信任 req->pa)
 * @param  func: 被应答的功能码
 * @retval 该功能码回显应答应使用的 PA
 * @note   0x11 紧急通告 -> PA_URGENT(01H); 0x12/0x13 及其余查询/设置 -> PA_NORMAL(03H)
 */
static uint8_t FecbusRx_EchoPa(uint8_t func)
{
    switch (func) {
    case 0x11:  return FECBUS_PA_URGENT;   /* 紧急通告 (表C.4/C.8) */
    case 0x12:  return FECBUS_PA_NORMAL;   /* 一般通告 */
    case 0x13:  return FECBUS_PA_NORMAL;   /* 调试通告 */
    default:    return FECBUS_PA_NORMAL;   /* 查询/设置回显 (表C.4/C.8) */
    }
}

/**
 * @brief  D3/B1: 正常应答 = 回显指定功能码 (FT=1, DLC=1, data=[func])
 * @param  req:  请求帧 (取其 SA/MN 反填; PA 按 func 查表)
 * @param  func: 要回显的功能码
 * @note   多帧通告的结束帧 data[0]=0FH, 需显式回显组功能码(0x11/12/13), 故拆出本函数。
 */
static void FecbusRx_ReplyEchoFunc(const FecbusRxFrame_t *req, uint8_t func)
{
    uint8_t payload[1];
    payload[0] = func;   /* 回显功能码 */

    /* 组帧: [0x7E][FT=1][DA=请求SA][PA=查表][SA=1][MN=请求MN][TN=0][DLC=1][功能码][CRC][0x7E] */
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FecbusRx_EchoPa(func),
                        req->mn, FECBUS_TN_SINGLE, payload, 1);

    DebugPrintf("[FECBUS-RX] reply echo func=%02X\r\n", func);
}

/**
 * @brief  D3: 正常应答 = 回显请求功能码 (单帧请求: data[0]=功能码)
 * @param  req: 请求帧
 */
static void FecbusRx_ReplyEcho(const FecbusRxFrame_t *req)
{
    FecbusRx_ReplyEchoFunc(req, req->data[0]);
}

/**
 * @brief  D3: 0FH 状态应答 (FT=1, DLC=2, data=[0x0F][status])
 * @param  req:    请求帧 (取其 SA/MN 反填; PA 固定 03H)
 * @param  status: 状态码 (A3: FECBUS_STAT_*; 仅异常/分组结束/事件结束三类)
 * @note   走 Fecbus_SendRawFrame -> USART3 从机通道。
 */
void FecbusRx_ReplyStatus(const FecbusRxFrame_t *req, uint8_t status)
{
    uint8_t payload[2];
    payload[0] = FECBUS_FUNC_RESP;   /* 0x0F */
    payload[1] = status;             /* 状态码 */

    /* 组帧: [0x7E][FT=1][DA=请求SA][PA=03H][SA=1][MN=请求MN][TN=0][DLC=2][0FH][status][CRC][0x7E] */
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                        req->mn, FECBUS_TN_SINGLE, payload, 2);

    DebugPrintf("[FECBUS-RX] reply 0FH status=%02X\r\n", status);
}

/*--------------------------------------------------------------
 * B2: 单帧查询应答原语 + 数据源聚合
 *--------------------------------------------------------------*/
/**
 * @brief  B2: 单帧带数据应答 (FT=1, DLC=1+len, data=[func][d...])
 * @param  req:  请求帧 (取其 SA/MN 反填; PA=03H)
 * @param  func: 回显的查询功能码
 * @param  d:    附加数据(len 字节, 可为 NULL 当 len=0)
 * @note   走 Fecbus_SendRawFrame -> USART3 从机通道。用于 0x2C/22/23/24/25 单帧查询。
 */
static void FecbusRx_ReplyData(const FecbusRxFrame_t *req, uint8_t func,
                               const uint8_t *d, uint8_t len)
{
    uint8_t payload[1 + 8];
    uint8_t i;
    if (len > 8) { len = 8; }                 /* 数据区上限 8(DLC 1~8), 保守截断 */
    payload[0] = func;
    for (i = 0; i < len; i++) { payload[1 + i] = d[i]; }

    /* 组帧: [0x7E][FT=1][DA=请求SA][PA=03H][SA=1][MN=请求MN][TN=0][DLC=1+len][func][d...][CRC][0x7E] */
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                        req->mn, FECBUS_TN_SINGLE, payload, (uint8_t)(1 + len));

    DebugPrintf("[FECBUS-RX] reply data func=%02X len=%d\r\n", func, len);
}

/**
 * @brief  B2: 0x22 设备状态位聚合 (表C.18)
 * @retval 16位状态字 (bit0 手/自动 ... bit9 喷洒)
 * @note   已接源: bit0 手自动, bit1 备电供电, bit2 电源故障(主/备电), bit3 报警,
 *         bit4 启动, bit7 故障, bit8 屏蔽(shielding_state), bit9 喷洒。
 *         bit5 反馈(源待定)/bit6 监管(工程未实现)/bit10 应急: 无有效源暂置0, 见内注释。
 */
static uint16_t FecbusRx_BuildDevStatus(void)
{
    uint16_t st    = 0;
    uint8_t  power = getCurrentMainBackupPowerState();   /* 1主电正常 2备电供电 3备电故障 */

    if (!SystemSaveInfo.system_hand_or_auto_state) { st |= (uint16_t)(1u << 0); }  /* bit0=1 自动(表C.18:0手动/1自动; state:0自动/1手动, 故取反) */
    if (power == 2)                               { st |= (uint16_t)(1u << 1); }  /* bit1 备电供电 */
    if (power == 2 || power == 3)                 { st |= (uint16_t)(1u << 2); }  /* bit2 电源故障(2主电异常/3备电故障, 表C.18) */
    if (getCurrentSystemRunState() == 2)          { st |= (uint16_t)(1u << 3); }  /* bit3 报警 */
    if (getCurrentStartStopKeyState())            { st |= (uint16_t)(1u << 4); }  /* bit4 启动 */
    /* bit5 反馈: TODO 源待定 — feedbacked_state 系死变量(仅init清零无置位); 候选 A) part_1_feedback||part_2_feedback(分区喷洒反馈,part_2待接) B) getFeedBack1State()||getFeedBack2State()(联动反馈输入端子,两路活) */
    /* bit6 监管: 工程未实现监管状态(regul_alarm_state 死变量,全工程无置位), 置0 */
    if (getCurrentSystemFaultState())             { st |= (uint16_t)(1u << 7); }  /* bit7 故障 */
    if (shielding_state)                          { st |= (uint16_t)(1u << 8); }  /* bit8 屏蔽 */
    if (getOutFireSprayState())                   { st |= (uint16_t)(1u << 9); }  /* bit9 喷洒 */
    /* bit10 应急: 工程无独立应急照明聚合状态源, 维持置0 (表C.18) */
    return st;
}

/**
 * @brief  B2: 0x23 按查询层级返回目标数量 (C.4.5.13)
 * @param  level: 1=控制器 2=单元 3=设备 4=通道
 * @retval 对应层级目标数量
 */
static uint16_t FecbusRx_BuildConfigCount(uint8_t level)
{
    switch (level) {
    case 1:  return 1;   /* 控制器数: 本机 1 台 */
    case 2:  return 1;   /* 单元数: 单单元 (TODO 如扩展多单元再聚合) */
    case 3:              /* 设备数: 回路2(MBus)+回路3(RS485)在线总数 */
        return (uint16_t)(MBusCtrl_GetOnlineCount() + RS485Detect_GetOnlineCount());
    case 4:              /* 通道数: 每设备 1 通道 (TODO 如多通道设备再聚合) */
        return (uint16_t)(MBusCtrl_GetOnlineCount() + RS485Detect_GetOnlineCount());
    default: return 0;
    }
}

/**
 * @brief  B2: 0x25 按参量类型返回阈值 (表C.19)
 * @param  type:      请求参量类型 (2=温度0.1℃ 3=压力0.1MPa 5=气体浓度0.1%LEL)
 * @param  ptype_out: 回显的参量类型; 未识别类型置 0
 * @retval 阈值(uint16); 未识别类型返回 0 且 *ptype_out=0
 */
static uint16_t FecbusRx_BuildParam(uint8_t type, uint8_t *ptype_out)
{
    switch (type) {
    case 2:  /* 温度 0.1℃ */
        *ptype_out = 2;
        return fire_alarm_threshold.temperature_warn_1;
    case 3:  /* 压力 0.1MPa (取装置1 压力上限) */
        *ptype_out = 3;
        return (uint16_t)(fire_alarm_threshold.device1_pressure_uplimit * 10.0f);
    case 5:  /* 气体浓度 0.1%LEL (取氢气一级预警) */
        *ptype_out = 5;
        return fire_alarm_threshold.hydrogen_warn_1;
    default:
        *ptype_out = 0;
        return 0;
    }
}

/**
 * @brief  B2: 多帧文本应答 (表C.9~C.11)
 * @param  req:  请求帧 (取 SA/MN 反填; ctrl/unit/dev 取 req->data[1..3])
 * @param  func: 查询功能码
 * @param  data: 回传文本(UTF-8/字节流, len=0 时可为 NULL); len: 字节数(<=255)
 * @retval 0=成功
 * @note   len<=3: 单帧 TN=0 [func][ctrl][unit][dev][len][X0..X2] (DLC=5+len), 不发结束帧。
 *         len>3 : 首帧 TN=1 [func][ctrl][unit][dev][总字节数][X0][X1][X2] (DLC=8)
 *                 + 数据帧 TN=2..n(每帧<=8字节) + 结束帧 TN=0 [0FH][00H] (DLC=2)。
 */
static uint8_t FecbusRx_ReplyMulti(const FecbusRxFrame_t *req, uint8_t func,
                                   const uint8_t *data, uint16_t len)
{
    uint8_t  ctrl = (req->dlc >= 2) ? req->data[1] : 1;
    uint8_t  unit = (req->dlc >= 3) ? req->data[2] : 1;
    uint8_t  dev  = (req->dlc >= 4) ? req->data[3] : 0;
    uint8_t  frm[8];
    uint8_t  endfrm[2];
    uint16_t off;
    uint8_t  tn;
    uint8_t  i;

    if (len > 255) { len = 255; }   /* 总字节数字段 1 字节上限 */

    if (len <= 3) {
        /* 单帧 TN=0: [func][ctrl][unit][dev][len][X0..X2] DLC=5+len, 不发结束帧 */
        frm[0] = func; frm[1] = ctrl; frm[2] = unit; frm[3] = dev; frm[4] = (uint8_t)len;
        for (i = 0; i < len; i++) { frm[5 + i] = data[i]; }
        Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                            req->mn, FECBUS_TN_SINGLE, frm, (uint8_t)(5 + len));
        DebugPrintf("[FECBUS-RX] reply multi-single func=%02X len=%d\r\n", func, len);
        return 0;
    }

    /* 多帧首帧 TN=1: [func][ctrl][unit][dev][总字节数][X0][X1][X2] DLC=8 */
    frm[0] = func; frm[1] = ctrl; frm[2] = unit; frm[3] = dev; frm[4] = (uint8_t)len;
    frm[5] = data[0]; frm[6] = data[1]; frm[7] = data[2];
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                        req->mn, FECBUS_TN_FRAME1, frm, 8);

    /* 数据帧 TN=2..n: 每帧<=8字节 */
    off = 3;
    tn  = FECBUS_TN_FRAME2;
    while (off < len) {
        uint8_t n = (uint8_t)((len - off) > 8 ? 8 : (len - off));
        for (i = 0; i < n; i++) { frm[i] = data[off + i]; }
        Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                            req->mn, tn, frm, n);
        off += n;
        tn++;
    }

    /* 结束帧 TN=0: [0FH][00H] */
    endfrm[0] = FECBUS_FUNC_RESP; endfrm[1] = 0x00;
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL,
                        req->mn, FECBUS_TN_SINGLE, endfrm, 2);
    DebugPrintf("[FECBUS-RX] reply multi func=%02X len=%d\r\n", func, len);
    return 0;
}

/**
 * @brief  B2: 0x29 查当前事件应答 (表C.12): 复用 RAM 最新事件缓存 s_last_evt 组 4 帧
 * @param  req: 请求帧
 * @note   帧1 TN=1 [29H][控制器..事件低]; 帧2 TN=2 [事件高..分]; 帧3 TN=3 [秒]; 帧4 TN=0 [0FH][00H]。
 *         无缓存事件 -> 状态帧(单元故障 0x03)。
 */
static void FecbusRx_ReplyCurEvent(const FecbusRxFrame_t *req)
{
    uint8_t f1[8], f2[8], f3[1], fe[2];

    if (!s_last_evt_valid) {
        FecbusRx_ReplyStatus(req, FECBUS_STAT_UNIT_FAULT);   /* 暂无当前事件 */
        return;
    }
    f1[0] = FECBUS_FUNC_QUERY_CUR_EVT;      /* 29H */
    memcpy(&f1[1], &s_last_evt[0], 7);      /* 控制器/单元/设备/通道/类型低/类型高/事件低 */
    memcpy(f2, &s_last_evt[7], 8);          /* 事件高/状态低/状态高/年/月/日/时/分 */
    f3[0] = s_last_evt[15];                 /* 秒 */
    fe[0] = FECBUS_FUNC_RESP; fe[1] = 0x00; /* 结束帧 */

    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL, req->mn, FECBUS_TN_FRAME1, f1, 8);
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL, req->mn, FECBUS_TN_FRAME2, f2, 8);
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL, req->mn, FECBUS_TN_FRAME3, f3, 1);
    Fecbus_SendRawFrame(FECBUS_FT_RESPONSE, req->sa, FECBUS_PA_NORMAL, req->mn, FECBUS_TN_SINGLE, fe, 2);
    DebugPrintf("[FECBUS-RX] reply cur event 0x29\r\n");
}

/**
 * @brief  B2: 0x2D 构建层级在线设备列表 -> out[], 返回字节数
 * @param  level:   1=控制器 2=单元 3=设备 4=通道
 * @param  out/cap: 输出缓冲及容量
 * @note   控制器/单元: 本机各 1 -> [1]。设备/通道: 枚举 回路2(MBus 1~63)+回路3(RS485 1~32) 在线地址。
 *         TODO(表C.14/C.16): 如需每设备携带 national_type_code, 扩展条目为 [addr][type_lo][type_hi]
 *         (MBusCtrl_GetNationalTypeCode / RS485Detect_GetNationalTypeCode 已 extern 备用)。
 */
static uint16_t FecbusRx_BuildDevList(uint8_t level, uint8_t *out, uint16_t cap)
{
    uint16_t n = 0;
    uint8_t  addr;

    if (level == 1 || level == 2) {          /* 控制器/单元: 本机各 1 */
        if (cap >= 1) { out[n++] = 1; }
        return n;
    }
    for (addr = 1; addr < FECBUS_MBUS_MAX_ADDR && n < cap; addr++) {   /* 回路2 MBus */
        if (MBusCtrl_GetOnline(addr)) { out[n++] = addr; }
    }
    for (addr = 1; addr < FECBUS_RS485_MAX_ADDR && n < cap; addr++) {  /* 回路3 RS485 */
        if (RS485Detect_GetOnline(addr)) { out[n++] = addr; }
    }
    return n;
}

/*--------------------------------------------------------------
 * B1: 装置通告事件入库分派 + 多帧组装状态机
 *--------------------------------------------------------------*/
/**
 * @brief  B1: 按事件代码将组装完成的装置通告入库(黑匣子)
 * @param  func: 通告功能码(0x11/0x12/0x13)
 * @param  buf:  组装缓冲(16字节有效):
 *               [0]控制器[1]单元[2]设备[3]通道[4]类型低[5]类型高[6]事件低[7]事件高
 *               [8]状态低[9]状态高[10]年[11]月[12]日[13]时[14]分[15]秒
 * @retval 0=已入库/已处理(调用方回显应答); FECBUS_STAT_PARAM_ERR(7)=未识别事件码
 * @note   事件代码分派依据 GB4717 附录C 事件代码表; 0x13 调试通告仅打印不入库。
 */
static uint8_t FecbusRx_LogEvent(uint8_t func, const uint8_t *buf)
{
    uint8_t  unit_no    = buf[1];
    uint8_t  dev_no     = buf[2];
    uint8_t  channel_no = buf[3];
    uint16_t dev_type   = (uint16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    uint16_t event_code = (uint16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
    uint16_t state_code = (uint16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));

    /* 0x13 调试通告: 仅打印, 不入库, 回显应答 */
    if (func == FECBUS_FUNC_NOTIFY_DEBUG) {
        DebugPrintf("[FECBUS-B1] debug notify dev=%d evt=%d\r\n", dev_no, event_code);
        return 0;
    }

    switch (event_code) {
    case 2:   /* 首警 */
    case 3:   /* 火警 */
        StorageEvent_LogFire(dev_no, dev_type, unit_no, channel_no);
        return 0;
    case 19:  /* 启动 */
        StorageEvent_LogStart(dev_no, dev_type);
        return 0;
    case 26:  /* 反馈 */
    case 27:
    case 28:
        StorageEvent_LogFeedback(dev_no, dev_type, state_code);
        return 0;
    case 70:  /* 监管 */
        StorageEvent_LogSupervise(dev_no, dev_type, 0);
        return 0;
    case 71:  /* 监管解除 */
        StorageEvent_LogSupervise(dev_no, dev_type, 1);
        return 0;
    case 72:  /* 屏蔽 */
        StorageEvent_LogShield(dev_no, dev_type, 0);
        return 0;
    case 73:  /* 解除屏蔽 */
        StorageEvent_LogShield(dev_no, dev_type, 1);
        return 0;
    case 125: /* 手动 */
        StorageEvent_LogManualAuto(dev_no, 1);
        return 0;
    case 126: /* 自动 */
        StorageEvent_LogManualAuto(dev_no, 0);
        return 0;
    default:
        break;
    }

    /* 范围类事件码 */
    if (event_code >= 80 && event_code <= 90) {          /* 故障 */
        StorageEvent_LogFault(dev_no, dev_type, unit_no, channel_no, 0);
        return 0;
    }
    if (event_code >= 100 && event_code <= 110) {        /* 故障恢复 */
        StorageEvent_LogFault(dev_no, dev_type, unit_no, channel_no, 1);
        return 0;
    }
    if (event_code >= 20 && event_code <= 30) {          /* 其余启动/联动类: 暂仅打印 */
        DebugPrintf("[FECBUS-B1] evt=%d dev=%d (log TBD)\r\n", event_code, dev_no);
        return 0;
    }

    /* 未识别事件码: 不入库, 回参数错(0x07) */
    DebugPrintf("[FECBUS-B1] unknown evt=%d dev=%d\r\n", event_code, dev_no);
    return FECBUS_STAT_PARAM_ERR;
}

/**
 * @brief  B1: 按帧序 TN 组装装置通告4帧, 完成则入库并应答
 * @param  f: 当前接收帧(首帧 TN=1 由功能码 switch 进入; 续帧 TN=2/3/0 由会话路由进入)
 * @note   - 会话按装置地址 sa(2~63) 索引, 组内帧间隔超时 1s 则清会话
 *          - 结束帧校验 [0x0F][0x00]; 组装完成 -> LogEvent 入库 + ReplyEchoFunc(回显组功能码)
 *          - 乱序/超时/MN不符/结束帧异常 -> 清会话 + ReplyStatus(参数错)
 */
static void FecbusRx_AsmFeed(const FecbusRxFrame_t *f)
{
    FecbusAsmSession_t *s;
    uint8_t idx;

    /* 装置地址 2~63 -> 会话索引 0~61 (C.3.2.2) */
    if (f->sa < 2 || f->sa > 63) {
        FecbusRx_ReplyStatus(f, FECBUS_STAT_ADDR_NOT_EXIST);
        return;
    }
    idx = (uint8_t)(f->sa - 2);
    s = &s_asm[idx];

    switch (f->tn) {
    case FECBUS_TN_FRAME1:  /* TN=1: 组首帧, 新建/重置会话 */
        if (f->dlc != 8) {
            FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
            return;
        }
        s->active  = 1;
        s->sa      = f->sa;
        s->mn      = f->mn;
        s->func    = f->data[0];           /* 通告功能码 0x11/12/13 */
        s->next_tn = FECBUS_TN_FRAME2;
        s->tick    = HAL_GetTick();
        /* buf[0..6] = data[1..7]: 控制器/单元/设备/通道/类型低/类型高/事件低 */
        memcpy(&s->buf[0], &f->data[1], 7);
        break;

    case FECBUS_TN_FRAME2:  /* TN=2 */
        if (!s->active || s->mn != f->mn || s->next_tn != FECBUS_TN_FRAME2 ||
            f->dlc != 8 || (HAL_GetTick() - s->tick) > FECBUS_ASM_TIMEOUT_MS) {
            s->active = 0;
            FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
            return;
        }
        /* buf[7..14] = data[0..7]: 事件高/状态低/状态高/年/月/日/时/分 */
        memcpy(&s->buf[7], &f->data[0], 8);
        s->next_tn = FECBUS_TN_FRAME3;
        s->tick    = HAL_GetTick();
        break;

    case FECBUS_TN_FRAME3:  /* TN=3 */
        if (!s->active || s->mn != f->mn || s->next_tn != FECBUS_TN_FRAME3 ||
            f->dlc != 1 || (HAL_GetTick() - s->tick) > FECBUS_ASM_TIMEOUT_MS) {
            s->active = 0;
            FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
            return;
        }
        s->buf[15] = f->data[0];           /* 秒 */
        s->next_tn = FECBUS_TN_SINGLE;
        s->tick    = HAL_GetTick();
        break;

    case FECBUS_TN_SINGLE:  /* TN=0: 分组报文结束帧 [0x0F][0x00] */
        if (!s->active || s->mn != f->mn || s->next_tn != FECBUS_TN_SINGLE ||
            f->dlc != 2 || f->data[0] != FECBUS_FUNC_RESP || f->data[1] != 0x00 ||
            (HAL_GetTick() - s->tick) > FECBUS_ASM_TIMEOUT_MS) {
            s->active = 0;
            FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
            return;
        }
        /* 组装完成: 缓存最新事件 + 入库 + 回显应答(回显组功能码, 非结束帧的 0x0F) */
        s->active = 0;
        memcpy(s_last_evt, s->buf, 16);   /* B2: 缓存为 RAM 最新事件(0x29 查当前事件用) */
        s_last_evt_valid = 1;
        {
            uint8_t st = FecbusRx_LogEvent(s->func, s->buf);
            if (st == 0) {
                FecbusRx_ReplyEchoFunc(f, s->func);
            } else {
                FecbusRx_ReplyStatus(f, st);
            }
        }
        break;

    default:  /* 非法 TN */
        s->active = 0;
        FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
        break;
    }
}

/*--------------------------------------------------------------
 * 帧分发：协议层闭环 + 业务 TODO 接口
 *--------------------------------------------------------------*/
static void FecbusRx_Dispatch(const FecbusRxFrame_t *f)
{
    /* 1) 应答帧(FT=1): D4 识别回显帧(正常应答) 与 0FH 状态帧(结束/异常) */
    if (f->ft == FECBUS_FT_RESPONSE) {
        if (f->dlc == 1 && f->data[0] == s_ack_func) {
            /* 回显应答: 对端原样回显功能码 = 正常应答(表C.6) */
            s_ack_ok = 1;
            s_ack_mn = f->mn;
            DebugPrintf("[FECBUS-RX] ACK echo func=%02X mn=%d\r\n", f->data[0], f->mn);
        } else if (f->dlc >= 2 && f->data[0] == FECBUS_FUNC_RESP) {
            /* 0FH 状态帧: status=0(分组报文结束)->ACK; status!=0->NAK(终止重试) */
            s_ack_mn = f->mn;
            if (f->data[1] == 0x00) {
                s_ack_ok = 1;
                DebugPrintf("[FECBUS-RX] RESP end mn=%d\r\n", f->mn);
            } else {
                s_ack_nak = 1;
                DebugPrintf("[FECBUS-RX] RESP nak status=%02X mn=%d\r\n", f->data[1], f->mn);
            }
        }
        return;
    }

    /* 2) 请求帧: 仅处理发给控制器(DA=1)的帧 */
    if (f->da != FECBUS_SA_CONTROLLER) {
        return;
    }

    FecbusRx_PrintFrame(f);

    /* 3) B1 多帧通告续帧路由: 该 sa 有进行中会话且 MN 匹配 -> 交组装状态机
     *    (续帧 TN=2/3/0 的 data[0] 非功能码, 不能走下面的功能码 switch) */
    if (f->tn != FECBUS_TN_FRAME1 && f->sa >= 2 && f->sa <= 63) {
        FecbusAsmSession_t *sp = &s_asm[f->sa - 2];
        if (sp->active && sp->mn == f->mn) {
            FecbusRx_AsmFeed(f);
            return;
        }
    }

    /* 4) 按功能码分发（首帧 TN=1 / 单帧请求） */
    switch (f->data[0]) {
    case FECBUS_FUNC_NOTIFY_URGENT:  /* 0x11 装置紧急通告 (B1: 4帧组装) */
    case FECBUS_FUNC_NOTIFY_NORMAL:  /* 0x12 装置一般通告 */
    case FECBUS_FUNC_NOTIFY_DEBUG:   /* 0x13 装置调试通告 */
        FecbusRx_AsmFeed(f);         /* 首帧(TN=1)入口; 续帧由分发前会话路由处理 */
        return;
    case FECBUS_FUNC_HEARTBEAT:  /* 0x14 装->控心跳: 回显应答 (TODO: 更新该装置在线时间戳) */
        FecbusRx_ReplyEcho(f);
        return;
    case FECBUS_FUNC_POLL_CONN:  /* 0x21 巡检连接(上位机查询本控制器时): 回显 [21H] DLC=1 */
        FecbusRx_ReplyEcho(f);
        return;
    case FECBUS_FUNC_DEV_STATUS: {  /* 0x22 查设备状态: [22H][状态0][状态1] DLC=3 (表C.18) */
        uint16_t st = FecbusRx_BuildDevStatus();
        uint8_t  d[2];
        d[0] = (uint8_t)(st & 0xFF);
        d[1] = (uint8_t)(st >> 8);
        FecbusRx_ReplyData(f, FECBUS_FUNC_DEV_STATUS, d, 2);
        return;
    }
    case FECBUS_FUNC_DEV_CONFIG: {  /* 0x23 查设备配置: [23H][数量0][数量1] DLC=3 (C.4.5.13) */
        uint8_t  level = (f->dlc >= 2) ? f->data[1] : 3;   /* 缺省查设备数 */
        uint16_t cnt   = FecbusRx_BuildConfigCount(level);
        uint8_t  d[2];
        d[0] = (uint8_t)(cnt & 0xFF);
        d[1] = (uint8_t)(cnt >> 8);
        FecbusRx_ReplyData(f, FECBUS_FUNC_DEV_CONFIG, d, 2);
        return;
    }
    case FECBUS_FUNC_DEV_ID:  /* 0x24 查设备标识: [24H][标识0..6] DLC=8 (7字节, C.4.5.12) */
        FecbusRx_ReplyData(f, FECBUS_FUNC_DEV_ID, s_dev_id, 7);
        return;
    case FECBUS_FUNC_DEV_PARAM: {  /* 0x25 查设备参量: [25H][参量类型][数值0][数值1] DLC=4 (表C.19) */
        uint8_t  ptype    = 0;
        uint8_t  req_type = (f->dlc >= 2) ? f->data[1] : 2;   /* 缺省查温度 */
        uint16_t val      = FecbusRx_BuildParam(req_type, &ptype);
        uint8_t  d[3];
        if (ptype == 0) {   /* 未识别参量类型 -> 参数错(0x07) */
            FecbusRx_ReplyStatus(f, FECBUS_STAT_PARAM_ERR);
            return;
        }
        d[0] = ptype;
        d[1] = (uint8_t)(val & 0xFF);
        d[2] = (uint8_t)(val >> 8);
        FecbusRx_ReplyData(f, FECBUS_FUNC_DEV_PARAM, d, 3);
        return;
    }
    case FECBUS_FUNC_DEV_COMMENT:  /* 0x26 查设备注释(表C.9, UTF-8) */
        /* TODO: 工程暂无独立设备注释表; 接入后回传真实注释。现回空注释单帧(总字节数=0) */
        FecbusRx_ReplyMulti(f, FECBUS_FUNC_DEV_COMMENT, NULL, 0);
        return;
    case FECBUS_FUNC_DEV_PROGRAM:  /* 0x27 查设备编程(表C.10, 布局同C.9, 编程信息UTF-8) */
        /* TODO(任务三): 联动规则(bsp_logic_expr.c) -> UTF-8 编程信息文本, 经 ReplyMulti 回传。
         *   本轮留桩: 编程信息源未就绪 -> 回 0FH 处理中(0x08), 上位机稍后重查。 */
        FecbusRx_ReplyStatus(f, FECBUS_STAT_PROCESSING);
        return;
    case FECBUS_FUNC_REG_INFO:  /* 0x28 查注册登记信息(表C.11, 布局同C.9, UTF-8) */
        /* TODO: 接入回路 online 表完整注册登记 UTF-8 文本; 现回设备标识(7B)作占位注册信息 */
        FecbusRx_ReplyMulti(f, FECBUS_FUNC_REG_INFO, s_dev_id, 7);
        return;
    case FECBUS_FUNC_QUERY_CUR_EVT:  /* 0x29 查当前事件(表C.12): RAM 最新事件 4 帧 */
        FecbusRx_ReplyCurEvent(f);
        return;
    case FECBUS_FUNC_QUERY_HIS_EVT:  /* 0x2A 查历史事件(表C.13, 布局同C.12 + 逐条停等 C.4.5.9) */
        /* TODO(任务一): LPUART1 查存储侧历史事件(StorageTx), 逐条组4帧停等回传,
         *   每条待对端回显应答后发下一条, 全部发完回 [0FH][09H](FECBUS_STAT_EVT_END)。
         *   本轮留桩: 登记停等会话骨架(批次7 s_query 字段) + 回 0FH 处理中(0x08)。 */
        s_query.active   = 1;                        /* 预留停等会话骨架 */
        s_query.func     = FECBUS_FUNC_QUERY_HIS_EVT;
        s_query.mn       = f->mn;
        s_query.sa       = f->sa;
        s_query.total    = 0;                        /* 待任务一填: 历史事件总条数/字节数 */
        s_query.offset   = 0;
        s_query.tn       = FECBUS_TN_FRAME1;
        s_query.wait_ack = 1;                        /* 停等: 待存储侧回数据后逐条发送 */
        s_query.tick     = HAL_GetTick();
        FecbusRx_ReplyStatus(f, FECBUS_STAT_PROCESSING);
        return;
    case FECBUS_FUNC_STOP_QUERY:  /* 0x2B 停止查询: 清查询会话 + [2BH] DLC=1 回显 */
        s_query.active   = 0;     /* B2: 中止进行中的多帧查询会话 */
        s_query.wait_ack = 0;
        FecbusRx_ReplyEcho(f);
        return;
    case FECBUS_FUNC_PROTO_VER: {  /* 0x2C 查协议版本: [2CH][ver0][ver1] DLC=3 */
        uint8_t d[2];
        d[0] = (uint8_t)(FECBUS_PROTO_VER & 0xFF);
        d[1] = (uint8_t)(FECBUS_PROTO_VER >> 8);
        FecbusRx_ReplyData(f, FECBUS_FUNC_PROTO_VER, d, 2);
        return;
    }
    case FECBUS_FUNC_DEV_LIST: {  /* 0x2D 查设备列表(表C.14): 层级在线设备地址列表 */
        uint8_t  lvl = (f->dlc >= 2) ? f->data[1] : 3;   /* 缺省查设备级 */
        uint8_t  lst[100];
        uint16_t n   = FecbusRx_BuildDevList(lvl, lst, sizeof(lst));
        FecbusRx_ReplyMulti(f, FECBUS_FUNC_DEV_LIST, lst, n);
        return;
    }
    default:
        /* D3/A3: 未知功能码 -> 0FH 异常应答 (状态码 5=未知命令) */
        DebugPrintf("[FECBUS-RX] unknown func=%02X\r\n", f->data[0]);
        FecbusRx_ReplyStatus(f, FECBUS_STAT_UNKNOWN_CMD);
        return;
    }

    /* D3: 各功能码 case 内显式应答(ReplyEcho/ReplyData/ReplyMulti/ReplyStatus), 取消统一 0FH 兜底。
     *     A3: 成功应答走回显/数据帧不发 0FH; B1/B2 全部实现(0x27/0x2A 跨模块留桩回 0FH 处理中)。 */
}

/*==============================================================
 * 文件结束
 *==============================================================*/
