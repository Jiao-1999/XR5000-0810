/**
 * @file    gb4717_export.c
 * @brief   GB4717-2024附录B 火灾报警控制器记录导出协议实现
 * @details 通过USB CDC虚拟串口(PA11-D-/PA12-D+)向PC导出存储侧记录,
 *          实现GB4717-2024附录B的B.1/B.3/B.4记录读取协议.
 *          底层调用USB_CDC接口收发数据, 协议帧解析/CRC16/记录读取均在本模块完成.
 *
 *   通信硬件: USB CDC虚拟串口(映射为PC端COM17), 115200 8N1
 *   协议帧头/帧尾: 均为0x40
 *   CRC16: MODBUS多项式0xA001, 低字节在前
 *
 *   请求帧(PC->存储侧):
 *     [0x40][设备ID 8字节][版本1][地址1][类型1][命令长度1][命令数据n][CRC16低][CRC16高][0x40]
 *     其中设备ID=0x00×8, 版本=0x02, 地址=0x7E, 类型=0x7F
 *
 *   响应帧(存储侧->PC):
 *     [0x40][记录总数3字节大端][控制器地址1][控制器类型2][产品编号20字节ASCII]
 *     [事件记录17字节(可选)][CRC16低][CRC16高][0x40]
 *
 *   命令码:
 *     0x01=顺序读取记录, 0x02=重发上一帧, 0x03=读首警(event_code=2), 0x04=读火警(event_code=3)
 */
#include "gb4717_export.h"
#include "usb_cdc.h"
#include <string.h>

/*==============================================================
 * 协议常量定义 (GB4717-2024附录B)
 *============================================================*/
#define GB4717_START_MARK       0x40    /* 帧头/帧尾标志字节 */
#define GB4717_VERSION          0x02    /* 协议版本号 */
#define GB4717_TOOL_ADDR        0x7E    /* 工具(上位机)地址 */
#define GB4717_TOOL_TYPE        0x7F    /* 工具(上位机)类型 */

#define GB4717_CMD_READ_DATA    1       /* 命令码: 顺序读取记录 */
#define GB4717_CMD_RESEND       2       /* 命令码: 重发上一帧响应 */
#define GB4717_CMD_READ_FIRST   3       /* 命令码: 读取首警记录(event_code=2) */
#define GB4717_CMD_READ_FIRE    4       /* 命令码: 读取火警记录(event_code=3) */

#define GB4717_CONTROLLER_ADDR  0x01    /* 本控制器地址 */
#define GB4717_CONTROLLER_TYPE  0x0001  /* 本控制器类型 */

/* 产品编号(20字节ASCII, 用于响应帧中的设备标识) */
static const uint8_t s_product_no[20] = {
    'X','R','5','0','0','0','-','S','T','O',
    'R','A','G','E','-','V','1','.','0','\0'
};

/*==============================================================
 * 内部变量
 *============================================================*/
/* 接收状态机状态枚举: 用于解析PC发来的请求帧 */
typedef enum {
    RX_STATE_WAIT_START = 0,  /* 等待帧头0x40 */
    RX_STATE_DEVICE_ID,       /* 接收8字节设备ID */
    RX_STATE_VERSION,         /* 接收版本号 */
    RX_STATE_ADDR,            /* 接收地址 */
    RX_STATE_TYPE,            /* 接收类型 */
    RX_STATE_CMD_LEN,         /* 接收命令长度 */
    RX_STATE_CMD_DATA,        /* 接收命令数据 */
    RX_STATE_CRC_LO,          /* 接收CRC低字节 */
    RX_STATE_CRC_HI,          /* 接收CRC高字节 */
    RX_STATE_END_MARK         /* 等待帧尾0x40 */
} RxState_t;

static RxState_t s_rx_state = RX_STATE_WAIT_START;  /* 当前接收状态 */
static uint8_t  s_rx_idx = 0;          /* 接收字节计数(用于设备ID/命令数据) */
static uint8_t  s_cmd_len = 0;         /* 当前请求的命令长度字段 */
static uint8_t  s_cmd_data = 0;        /* 当前请求的命令数据(命令码) */
static uint8_t  s_cmd_ready = 0;       /* 命令接收完成标志 */

static uint32_t s_read_index_seq = 0;  /* 顺序读取游标(每次CMD_READ_DATA自增,到末尾归零) */
static uint32_t s_read_index_first = 0;/* 首警读取游标(按event_code=2筛选) */
static uint32_t s_read_index_fire = 0; /* 火警读取游标(按event_code=3筛选) */

static uint8_t  s_last_resp[64];       /* 上一次响应帧缓存(用于CMD_RESEND重发) */
static uint16_t s_last_resp_len = 0;   /* 上一次响应帧长度 */

/*==============================================================
 * 内部函数
 *============================================================*/
/**
 * @brief  计算MODBUS CRC16 (多项式0xA001, 低字节在前)
 * @param  data: 参与计算的数据指针
 * @param  len:  数据长度
 * @retval CRC16值(低字节在前存放)
 * @note   与bsp_storage_rx的CRC算法一致, 用于请求/响应帧校验
 */
static uint16_t GB4717_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief  打包并发送GB4717响应帧到PC
 * @param  cmd: 命令码(仅用于记录, 实际不放入响应帧)
 * @param  rec: 要发送的事件记录指针; 为NULL表示无记录(如读取到末尾)
 * @note   响应帧字节布局:
 *           [0]  0x40 帧头
 *           [1-3] 记录总数(3字节大端, 高字节在前)
 *           [4]  控制器地址(0x01)
 *           [5-6] 控制器类型(0x0001, 大端)
 *           [7-26] 产品编号(20字节ASCII)
 *           [27-43] 事件记录(17字节EventRecord_t, rec!=NULL时才有)
 *           [N]  CRC16低字节
 *           [N+1] CRC16高字节
 *           [N+2] 0x40 帧尾
 *         CRC范围: 从记录总数到事件记录(即buf[1]~buf[idx-1])
 *         同时缓存本帧到s_last_resp, 供CMD_RESEND重发
 */
static void GB4717_SendResponse(uint8_t cmd, const EventRecord_t *rec)
{
    uint8_t buf[64];
    uint16_t idx = 0;
    uint16_t crc;
    uint32_t total = StorageRx_GetRecordCount();
    uint8_t has_record = (rec != NULL) ? 1 : 0;

    buf[idx++] = GB4717_START_MARK;
    buf[idx++] = (uint8_t)((total >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((total >> 8) & 0xFF);
    buf[idx++] = (uint8_t)(total & 0xFF);
    buf[idx++] = GB4717_CONTROLLER_ADDR;
    buf[idx++] = (uint8_t)((GB4717_CONTROLLER_TYPE >> 8) & 0xFF);
    buf[idx++] = (uint8_t)(GB4717_CONTROLLER_TYPE & 0xFF);
    memcpy(&buf[idx], s_product_no, 20);
    idx += 20;
    if (has_record)
    {
        memcpy(&buf[idx], rec, sizeof(EventRecord_t));
        idx += sizeof(EventRecord_t);
    }
    crc = GB4717_CRC16(&buf[1], idx - 1);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    buf[idx++] = GB4717_START_MARK;

    if (idx <= sizeof(s_last_resp))
    {
        memcpy(s_last_resp, buf, idx);
        s_last_resp_len = idx;
    }
    USB_CDC_SendData(buf, idx);
    (void)cmd;
}

/**
 * @brief  按事件编码筛选记录(用于首警/火警查询)
 * @param  target_event_code: 目标事件编码(2=首警, 3=火警)
 * @param  search_from: 输入输出参数, 指向本次搜索起始索引; 命中后更新为命中索引+1
 * @param  rec: 输出命中的记录
 * @retval 0=找到匹配记录, 1=未找到(同时将*search_from归零)
 * @note   从*search_from开始顺序扫描全部已存记录, 命中第一个event_code匹配项即返回.
 *         游标机制保证每次调用继续向后搜索, 避免重复返回同一条.
 */
static uint8_t GB4717_FindRecord(uint16_t target_event_code, uint32_t *search_from, EventRecord_t *rec)
{
    uint32_t total = StorageRx_GetRecordCount();
    uint32_t i;
    for (i = *search_from; i < total; i++)
    {
        if (StorageRx_ReadRecord(i, rec) == 0)
        {
            if (rec->event_code == target_event_code)
            {
                *search_from = i + 1;
                return 0;
            }
        }
        else break;
    }
    *search_from = 0;
    return 1;
}

/**
 * @brief  执行GB4717命令处理(根据命令码读取记录并发送响应)
 * @param  cmd: 命令码
 *           GB4717_CMD_READ_DATA(1): 顺序读取下一条记录, 游标s_read_index_seq自增,
 *                                    读到末尾则归零并发送空记录响应;
 *           GB4717_CMD_READ_FIRST(3): 按event_code=2筛选首警记录;
 *           GB4717_CMD_READ_FIRE(4):  按event_code=3筛选火警记录;
 *           GB4717_CMD_RESEND(2):     重发上一帧响应(从s_last_resp缓存);
 *           其他:                     发送空记录响应.
 */
static void GB4717_ProcessCommand(uint8_t cmd)
{
    EventRecord_t rec;
    switch (cmd)
    {
        case GB4717_CMD_READ_DATA:
            if (StorageRx_ReadRecord(s_read_index_seq, &rec) == 0)
            {
                s_read_index_seq++;
                GB4717_SendResponse(cmd, &rec);
            }
            else
            {
                s_read_index_seq = 0;
                GB4717_SendResponse(cmd, NULL);
            }
            break;
        case GB4717_CMD_READ_FIRST:
            if (GB4717_FindRecord(2, &s_read_index_first, &rec) == 0)
                GB4717_SendResponse(cmd, &rec);
            else
                GB4717_SendResponse(cmd, NULL);
            break;
        case GB4717_CMD_READ_FIRE:
            if (GB4717_FindRecord(3, &s_read_index_fire, &rec) == 0)
                GB4717_SendResponse(cmd, &rec);
            else
                GB4717_SendResponse(cmd, NULL);
            break;
        case GB4717_CMD_RESEND:
            if (s_last_resp_len > 0)
                USB_CDC_SendData(s_last_resp, s_last_resp_len);
            break;
        default:
            GB4717_SendResponse(cmd, NULL);
            break;
    }
}

/*==============================================================
 * 对外接口
 *============================================================*/
/**
 * @brief  初始化GB4717导出模块
 * @note   复位接收状态机、命令缓存、各读取游标和响应缓存.
 *         在主程序启动时调用一次.
 */
void GB4717_ExportInit(void)
{
    s_rx_state = RX_STATE_WAIT_START;
    s_rx_idx = 0;
    s_cmd_len = 0;
    s_cmd_data = 0;
    s_cmd_ready = 0;
    s_read_index_seq = 0;
    s_read_index_first = 0;
    s_read_index_fire = 0;
    s_last_resp_len = 0;
}

/*==============================================================
 * GB4717导出主循环处理 - 从USB CDC读取字节并按状态机解析
 *============================================================*/
/**
 * @brief  GB4717导出主循环处理
 * @note   在main的while(1)中调用. 完成两件事:
 *         1) 若上一轮已收到完整命令(s_cmd_ready=1), 则执行命令处理;
 *         2) 否则从USB CDC逐字节读取并喂入接收状态机, 直到命令完整或缓冲空.
 *
 *   请求帧解析流程(状态机):
 *     WAIT_START -> 收到0x40 -> DEVICE_ID(读8字节) -> VERSION(1) -> ADDR(1)
 *     -> TYPE(1) -> CMD_LEN(1) -> CMD_DATA(n) -> CRC_LO(1) -> CRC_HI(1)
 *     -> END_MARK(收到0x40则置位s_cmd_ready) -> 回到WAIT_START
 */
void GB4717_ExportProcess(void)
{
    uint16_t byte;

    if (s_cmd_ready)
    {
        GB4717_ProcessCommand(s_cmd_data);
        s_cmd_ready = 0;
        return;
    }

    while (USB_CDC_Available() > 0)
    {
        byte = USB_CDC_ReadByte();
        if (byte == 0xFFFF) break;

        switch (s_rx_state)
        {
            case RX_STATE_WAIT_START:
                if ((uint8_t)byte == GB4717_START_MARK)
                {
                    s_rx_idx = 0;
                    s_rx_state = RX_STATE_DEVICE_ID;
                }
                break;
            case RX_STATE_DEVICE_ID:
                s_rx_idx++;
                if (s_rx_idx >= 8) s_rx_state = RX_STATE_VERSION;
                break;
            case RX_STATE_VERSION:
                s_rx_state = RX_STATE_ADDR;
                break;
            case RX_STATE_ADDR:
                s_rx_state = RX_STATE_TYPE;
                break;
            case RX_STATE_TYPE:
                s_rx_state = RX_STATE_CMD_LEN;
                break;
            case RX_STATE_CMD_LEN:
                s_cmd_len = (uint8_t)byte;
                s_rx_idx = 0;
                if (s_cmd_len == 0) s_rx_state = RX_STATE_CRC_LO;
                else s_rx_state = RX_STATE_CMD_DATA;
                break;
            case RX_STATE_CMD_DATA:
                s_cmd_data = (uint8_t)byte;
                s_rx_idx++;
                if (s_rx_idx >= s_cmd_len) s_rx_state = RX_STATE_CRC_LO;
                break;
            case RX_STATE_CRC_LO:
                s_rx_state = RX_STATE_CRC_HI;
                break;
            case RX_STATE_CRC_HI:
                s_rx_state = RX_STATE_END_MARK;
                break;
            case RX_STATE_END_MARK:
                if ((uint8_t)byte == GB4717_START_MARK)
                    s_cmd_ready = 1;
                s_rx_state = RX_STATE_WAIT_START;
                break;
            default:
                s_rx_state = RX_STATE_WAIT_START;
                break;
        }

        if (s_cmd_ready) break;
    }
}
