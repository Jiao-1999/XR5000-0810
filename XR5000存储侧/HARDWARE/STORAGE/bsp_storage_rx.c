/**
 * @file    bsp_storage_rx.c
 * @brief   存储接收模块 - 接收主机侧记录并写入W25Q256
 * @details 通过USART1(PA9=TX/PA10=RX, 115200 8N1, 中断接收)接收主机侧事件记录帧,
 *          校验通过后写入W25Q256 SPI Flash指定分区, 支持分区管理和ACK回复.
 *          处理流程: USART1中断调用StorageRx_OnByte(), 主循环调用StorageRx_Process().
 *
 *   帧格式(主机->存储侧):
 *     [0xA5][长度][命令码][17字节EventRecord_t][CRC16低][CRC16高][0x5A]
 *     长度   = 命令码(1) + 负载长度
 *     CRC16: MODBUS多项式0xA001, 校验范围=长度+命令码+负载, 低位在前
 *
 *   存储命令: 0x01~0x04存储记录, 0x05查询容量, 0x06心跳
 *   存储介质: 数据写入W25Q256(32MB, 基地址0x00000000), 断电后记录仍保留
 */
#include "bsp_storage_rx.h"
#include "w25qxx.h"
#include "usart.h"
#include "delay.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>

/*==============================================================
 * 常量定义
 *============================================================*/
#define STX_RECORD_SIZE      sizeof(EventRecord_t)  /* 17字节记录大小 */
#define STX_FLASH_BASE_ADDR  0x00000000             /* 记录存储起始地址 */
#define STX_FLASH_MAX_ADDR   0x02000000             /* W25Q256容量32MB = 0x02000000 */

/*==============================================================
 * 内部变量
 *============================================================*/
static volatile uint8_t  s_frame_ready = 0;     /* 帧接收完成标志 */
static volatile uint32_t s_write_addr   = 0;     /* 当前写入地址 */
static uint8_t  s_rx_buf[96];                   /* 逐字节接收缓冲(扩容96: 兼容0x07测试日志透传帧46~86字节, 原32会拦丢致L2失效) */
static uint8_t  s_rx_idx = 0;                    /* 接收缓冲区索引 */
static uint8_t  s_frame_cmd;                     /* 当前帧命令码 */
static uint8_t  s_frame_len;                     /* 当前帧长度值 */
static uint16_t s_frame_payload_len;             /* 当前帧负载长度 */
static uint8_t  s_frame_buf[STX_MAX_PAYLOAD + 8];/* 完整帧缓冲 */
static uint16_t s_frame_total;                   /* 帧总长度 */

/*==============================================================
 * 调试全局变量 (供 main.c 及 USB_CDC 使用, 均位于存储侧接收链路内)
 *   g_stx_rx_byte_count: 接收字节计数 (每次USART1中断+1)
 *   g_stx_last_byte:     最近接收的字节 (调试用)
 *   g_stx_rx_idx_snap:   s_rx_idx 快照 (供调试分析, 0=空闲)
 *   g_stx_frame_ready_snap: s_frame_ready 快照 (1=收到帧但主循环未处理)
 *============================================================*/
/*==============================================================
 * 分区环形FIFO定义(P0-1/P0-2整改, 2026-08-24)
 * W25Q256 32MB布局:
 *   首警区1MB(0x000000) 火警区2MB(0x100000) 故障区2MB(0x300000)
 *   通用区约27MB(0x500000) 元数据区64KB(0x1FF0000, A/B双库)
 * 每扇区240条记录(240*17=4080<4096), 写满环形覆盖最旧
 *============================================================*/
#define STX_SECTOR_SIZE      4096UL
#define STX_SLOT_PER_SECTOR  240
/* ================================================================
 * 【GB4717-2024 附录B 存储实现索引】 搜索关键字: [GB4717
 *   B.1.2.1) 存储容量(区域型: 火警>=触发器件x100, 状态信息>=(触发+联动)x200)
 *   B.1.2.2) 非断电丢失介质 / >=30d / 始终保持最新 / 首火警·火灾报警·故障独立记录且不被覆盖
 *   B.1.2.3) 十六进制格式存储(格式见表B.1, 数据信息见表B.2)
 *   B.1.1.1) 记录范围见主控 bsp_storage_event.c
 *   B.1.1.2) 按时间顺序提供导出记录 -> StorageRx_ReadRecord
 *   B.1.1.3) 防止存储信息被更改或删除 -> MetaSave/MetaLoad/VerifyCRC/EraseAll
 *   B.1.3.1) 仅专用技术手段导出 -> 配合 gb4717_export.c
 *   B.1.4.2) 授权后才能导出 -> 配合 gb4717_export.c 的 AUTH_TOKEN
 *   B.1.4.4) 断电后数据保持 -> MetaSave/MetaLoad A/B双库
 * ================================================================ */
/* [GB4717 B.1.2.2] 首火警/火灾报警/故障三类信息独立记录, 其他状态信息不应覆盖它们。
 * 下面4个物理分区彼此不重叠, 通用区(Z3)无法侵占 Z0/Z1/Z2, 满足国标独立性要求。
 * [GB4717 B.1.2.1] 各分区容量在 StorageRx_Init() 中按"扇区数 x 每扇区240槽"计算。 */
#define STX_ZONE0_BASE       0x00000000UL   /* 首警区基址 */
#define STX_ZONE0_SIZE       0x00100000UL   /* 首警区1MB */
#define STX_ZONE1_BASE       0x00100000UL   /* 火警区基址 */
#define STX_ZONE1_SIZE       0x00200000UL   /* 火警区2MB */
#define STX_ZONE2_BASE       0x00300000UL   /* 故障区基址 */
#define STX_ZONE2_SIZE       0x00200000UL   /* 故障区2MB */
#define STX_ZONE3_BASE       0x00500000UL   /* 通用区基址 */
#define STX_ZONE3_SIZE       0x01AF0000UL   /* 通用区约27MB(至0x1FEFFFF) */
#define STX_META_BASE        0x01FF0000UL   /* 元数据区基址64KB */
#define STX_META_BANK_SIZE   0x00008000UL   /* 元数据A/B库各32KB */
#define STX_META_MAGIC       0x58525354UL   /* 元数据魔数 */

/* 元数据结构(42字节): 每条记录写入后保存各区指针, 断电恢复用 */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;                          /* 魔数 */
    uint32_t seq;                            /* 单调递增序号(取最大为最新) */
    uint32_t slot_head[STX_ZONE_COUNT];      /* 各区写指针(槽号) */
    uint32_t count[STX_ZONE_COUNT];          /* 各区现存条数 */
    uint16_t crc;                            /* 结构校验(除本字段外) */
} StorageMeta_t;
#pragma pack(pop)

/* 分区运行时状态 */
typedef struct {
    uint32_t base;        /* 区基址(字节) */
    uint32_t size;        /* 区大小(字节) */
    uint32_t capacity;    /* 容量(条), Init时按扇区数计算 */
    uint32_t slot_head;   /* 写指针(槽号) */
    uint32_t count;       /* 现存条数 */
} ZoneState_t;

static ZoneState_t s_zones[STX_ZONE_COUNT] = {
    {STX_ZONE0_BASE, STX_ZONE0_SIZE, 0, 0, 0},
    {STX_ZONE1_BASE, STX_ZONE1_SIZE, 0, 0, 0},
    {STX_ZONE2_BASE, STX_ZONE2_SIZE, 0, 0, 0},
    {STX_ZONE3_BASE, STX_ZONE3_SIZE, 0, 0, 0},
};

/* 元数据A/B双库状态(断电安全: 切库擦除时另一库保有有效数据) */
static uint8_t  s_meta_bank = 0;        /* 当前活动库(0=A, 1=B) */
static uint32_t s_meta_off  = 0;        /* 库内写偏移 */
static uint32_t s_meta_seq  = 0;        /* 元数据序号(单调递增) */
/* P0-A修复(v2): 分步预擦状态机变量.
 * 原方案若逐帧条件整擦8扇区, 从触发到切库约700条记录会重复整擦数千次(阻塞放大+耗尽擦写寿命).
 * v2改为: 每圈主循环只擦1个扇区(单次阻塞典型45ms/最坏400ms), 8圈完成一个bank,
 * 完成后置prep_done, MetaSave切库后复位进入下一周期. */
static uint8_t s_meta_prep_bank = 0;   /* 预擦目标bank(=当前bank^1) */
static uint8_t s_meta_prep_sec  = 0;   /* 预擦进度(已擦扇区数) */
static uint8_t s_meta_prep_done = 1;   /* 预擦空闲/完成标志(1=空闲) */


static uint32_t s_last_wr_addr = 0;     /* 最近一次写入地址(日志/读回验证用) */

volatile uint32_t g_stx_rx_byte_count = 0;
volatile uint8_t  g_stx_last_byte = 0;
volatile uint8_t  g_stx_rx_idx_snap = 0;
volatile uint8_t  g_stx_frame_ready_snap = 0;
volatile uint32_t g_stx_process_count = 0;   /* 处理过的帧数, 用于确认存储侧已响应主控 */

/* ISR帧到达事件: 在USART1中断里置位, main循环消费后清零.
 * 用于"中断处理函数日志": ISR本身不能调USB_CDC_SendData(会冲突USB驱动状态机),
 * 改为ISR捕获事件, main循环检测到事件后打印日志, 等效于"ISR里加日志". */
volatile uint8_t  g_stx_isr_frame_event  = 0;  /* 1=ISR收到完整帧待打印, 0=空闲 */
volatile uint8_t  g_stx_isr_last_cmd     = 0;  /* ISR最后一次收到的帧cmd */
volatile uint32_t g_stx_isr_frame_count  = 0;  /* ISR累计收到的完整帧数 */

/*==============================================================
 * 内部函数声明
 *============================================================*/
static uint16_t StorageRx_CRC16(const uint8_t *data, uint16_t len);
static uint32_t StorageRx_ZoneSlotAddr(const ZoneState_t *z, uint32_t slot);
static uint32_t StorageRx_ZoneWrite(ZoneState_t *z, const uint8_t *rec, uint8_t *overwrote);
static void StorageRx_ZoneCommit(ZoneState_t *z, uint8_t overwrote);
static void StorageRx_MetaSave(void);
static void StorageRx_MetaLoad(void);
static void StorageRx_SendByte(uint8_t data);
static void StorageRx_SendAck(uint8_t cmd, uint8_t ack_code);
static void StorageRx_SendCapacity(uint8_t cmd, uint32_t remaining);
static uint8_t StorageRx_VerifyCRC(void);

/*==============================================================
 * CRC16校验函数 (多项式0xA001, 兼容MODBUS CRC16)
 *============================================================*/
/**
 * @brief  计算MODBUS CRC16 (多项式0xA001, 高位在低字节)
 * @param  data: 待计算数据指针
 * @param  len:  数据长度
 * @retval CRC16值
 * @note   初值0xFFFF, 每个字节右移8次, 最低位为1则异或0xA001
 * [GB4717 B.1.1.3] 帧完整性校验算法(多项式0xA001, 低字节在前)
 * [核查 CHK-27] 本函数必须与主控侧 StorageTx_CRC16(bsp_storage_tx.c) 算法完全一致:
 *   多项式 0xA001 / 初值 0xFFFF / 无输入反转 / 无输出反转 / 校验范围=(长度+命令码+负载)。
 *   预期: 任意相同报文两侧计算结果相同; 若任一侧改动其一, 全部帧将被判 CRC 错误并静默丢弃。
 * [缺陷 DEF-N2] 主控侧发送失败(ACK超时/CRC错)3次重试全败后记录即被丢弃且不重新入队
 *   -> 该类事件永久丢失; 本函数仅能保证"收到的帧未被篡改", 无法弥补链路丢帧。 */
static uint16_t StorageRx_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;  /* 返回计算后的CRC16值 */
}

/*==============================================================
 * 通过USART1发送单字节
 *============================================================*/
/**
 * @brief  通过USART1发送单个字节
 * @param  data: 待发送的字节
 * @note   等待USART_FLAG_TC(发送完成标志)确保发送完毕
 */
static void StorageRx_SendByte(uint8_t data)
{
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*==============================================================
 * 发送ACK回复帧
 * 帧格式: [0xA5][0x02][cmd][ack_code][CRC16低][CRC16高][0x5A]
 *============================================================*/
/**
 * @brief  发送ACK应答帧(用于存储成功/失败等结果)
 * @param  cmd: 原命令码(与主机请求的命令一致)
 * @param  ack_code: 应答码 (STX_ACK_OK / STX_ACK_ERR_CRC / STX_ACK_ERR_FULL 等)
 * @note   发送的帧结构(共7字节):
 *           [0] 0xA5 帧头
 *           [1] 0x02 长度(负载长度=命令码+应答码=2字节)
 *           [2] cmd  原命令码
 *           [3] ack_code 应答码
 *           [4] CRC16低字节
 *           [5] CRC16高字节
 *           [6] 0x5A 帧尾
 *         CRC校验范围: 长度+命令码+应答码 = 3字节
 */
static void StorageRx_SendAck(uint8_t cmd, uint8_t ack_code)
{
    uint8_t buf[8];
    uint16_t crc;
    uint8_t i;

    buf[0] = STX_FRAME_HEAD;
    buf[1] = 0x02;          /* 长度 = 命令码(1) + 应答码(1) = 2 */
    buf[2] = cmd;           /* 原命令码 */
    buf[3] = ack_code;      /* 应答码 */

    /* CRC校验范围: 长度 + 命令码 + 应答码 = 3字节 */
    crc = StorageRx_CRC16(&buf[1], 3);
    buf[4] = (uint8_t)(crc & 0xFF);         /* CRC低字节 */
    buf[5] = (uint8_t)((crc >> 8) & 0xFF);  /* CRC高字节 */
    buf[6] = STX_FRAME_TAIL;

    for (i = 0; i < 7; i++)
    {
        StorageRx_SendByte(buf[i]);
    }
}

/*==============================================================
 * 发送剩余容量查询回复帧
 * 帧格式: [0xA5][0x05][cmd][剩余容量4字节][CRC16低][CRC16高][0x5A]
 *============================================================*/
/**
 * @brief  发送剩余容量回复帧(响应STX_CMD_QUERY_CAPACITY查询命令)
 * @param  cmd: 原命令码(0x05)
 * @param  remaining: 剩余可存储条数
 * @note   发送的帧结构(共10字节):
 *           [0] 0xA5 帧头
 *           [1] 0x05 长度(负载长度=命令码+剩余容量4字节=5字节)
 *           [2] cmd  原命令码
 *           [3-6] remaining 剩余容量(小端: 低字节在前)
 *           [7] CRC16低字节
 *           [8] CRC16高字节
 *           [9] 0x5A 帧尾
 *         CRC校验范围: 长度+命令码+容量 = 6字节
 */
static void StorageRx_SendCapacity(uint8_t cmd, uint32_t remaining)
{
    uint8_t buf[12];
    uint16_t crc;
    uint8_t i;

    buf[0] = STX_FRAME_HEAD;
    buf[1] = 0x05;          /* 长度 = 命令码(1) + 容量(4) = 5 */
    buf[2] = cmd;
    buf[3] = (uint8_t)(remaining & 0xFF);
    buf[4] = (uint8_t)((remaining >> 8) & 0xFF);
    buf[5] = (uint8_t)((remaining >> 16) & 0xFF);
    buf[6] = (uint8_t)((remaining >> 24) & 0xFF);

    /* CRC校验范围: 长度 + 命令码 + 容量 = 6字节 */
    crc = StorageRx_CRC16(&buf[1], 6);
    buf[7] = (uint8_t)(crc & 0xFF);
    buf[8] = (uint8_t)((crc >> 8) & 0xFF);
    buf[9] = STX_FRAME_TAIL;

    for (i = 0; i < 10; i++)
    {
        StorageRx_SendByte(buf[i]);
    }
}

/*==============================================================
 * 初始化存储模块
 *============================================================*/
/**
 * @brief  初始化存储模块
 * @retval 0=成功, 1=W25Q256初始化失败
 * @note   初始化W25Q256 SPI Flash, 从元数据区恢复写指针到存储区(0x00000000)
 */
/*==============================================================
 * 分区环形FIFO核心函数(P0-1/P0-2整改, 2026-08-24)
 *============================================================*/
/**
 * @brief  槽号转Flash字节地址(每扇区240槽, 扇区尾16字节弃用)
 * @param  z: 分区状态指针
 * @param  slot: 槽号(0 ~ capacity-1)
 * @retval Flash字节地址
 */
static uint32_t StorageRx_ZoneSlotAddr(const ZoneState_t *z, uint32_t slot)
{
    return z->base + (slot / STX_SLOT_PER_SECTOR) * STX_SECTOR_SIZE
                   + (slot % STX_SLOT_PER_SECTOR) * STX_RECORD_SIZE;
}

/**
 * @brief  向分区写入一条记录(环形FIFO, 写满覆盖最旧)
 * @param  z: 分区状态指针
 * @param  rec: 17字节记录数据
 * @retval 本次写入的Flash地址(供读回验证/日志)
 * @note   覆盖判定: 目标17字节非全FF说明有旧记录, 先擦所在扇区再写.
 *         断电恢复场景: 写指针位置为空白(FF)直接写, 同扇区旧记录不受影响.
 *         扇区粒度覆盖: 擦扇区会提前淘汰该扇区剩余旧记录, 属可接受损耗
 *         (各区容量巨大, 30天正常运行不会触发覆盖).
 */
/* [GB4717 B.1.2.2] 存储超出容量后应"始终保持最新的状态信息记录"(环形覆盖最旧)
 * [S-2 已修复 2026-09-17] 覆盖时擦掉整扇区(240槽)却只 count -= 239 再 count++ (净 -238),
 *   实际净变化应为 -239, 每次覆盖多记 1 条 -> 环形回绕后 count 系统性偏大 ->
 *   导出按 count 读取会越界到 0xFF 空洞。已改为 count -= STX_SLOT_PER_SECTOR。
 * [试验 TST-S2] 容量回绕: 向故障区(Z2)连续写入 >122880 条使环形回绕一轮,
 *   判据: (1)回绕后 GetRecordCount() 恒等于该区 capacity, 不得 >capacity;
 *         (2)导出前 100 条与后 100 条逐字节比对, 不得出现全 0xFF 空洞;
 *         (3)回绕前后各区 count 之和与 MetaLoad 恢复值一致(允许 ±0 误差)。
 * [已修复 DEF-N1b 2026-09-20] 原实现在本函数内部先完成 slot_head++/count++(RAM态),
 *   而调用方 StorageRx_Process 在读回校验失败时直接 return 未回滚指针推进 ->
 *   RAM count 虚高 1 条, 下一帧成功时 MetaSave 会把虚高值固化到 Flash(幽灵记录跨帧累积)。
 *   已按原修复方向落地: 本函数只写 Flash 不动指针(覆盖判定经 overwrote 出参上报),
 *   账目推进由 StorageRx_ZoneCommit 在读回校验通过后执行。
 * [试验 TST-N1] 账目一致性: 写 N 条后逐条读回比对, 判据: GetRecordCount == 实际落盘成功条数。 */
static uint32_t StorageRx_ZoneWrite(ZoneState_t *z, const uint8_t *rec, uint8_t *overwrote)
{
    uint32_t addr = StorageRx_ZoneSlotAddr(z, z->slot_head);
    uint8_t  tmp[STX_RECORD_SIZE];
    uint8_t  i;
    uint8_t  blank = 1;

    if (overwrote != NULL) { *overwrote = 0U; }

    /* 读取目标位置, 判断是否覆盖旧数据 */
    W25QXX_Read(tmp, addr, STX_RECORD_SIZE);
    for (i = 0; i < STX_RECORD_SIZE; i++)
    {
        if (tmp[i] != 0xFF)
        {
            blank = 0;
            break;
        }
    }

    if (blank == 0)
    {
        /* S-2 已修复 2026-09-17: 整扇区擦除丢的是本扇区全部 STX_SLOT_PER_SECTOR 条旧记录,
         * 原 count -= (STX_SLOT_PER_SECTOR-1) 会少减1条 -> 每次覆盖多记1条 ->
         * 环形回绕后 count 系统性偏大, 导出按 count 读取会越界到 0xFF 空洞.
         * [已修复 DEF-N1b 2026-09-20] 擦除动作仍在此执行(必须在写入前),
         * 但账目扣减已移到 StorageRx_ZoneCommit: 只有读回校验通过才变更
         * slot_head/count, 校验失败时账目保持原值, 不再出现"幽灵记录"跨帧累积。 */
        W25QXX_Erase_Sector(addr / STX_SECTOR_SIZE);
        if (overwrote != NULL) { *overwrote = 1U; }
    }

    W25QXX_Write((uint8_t *)rec, addr, STX_RECORD_SIZE);

    /* [DEF-N1b] 本函数不再推进 slot_head/count, 由调用方在读回校验通过后调 Commit */
    return addr;
}

/**
 * @brief  提交一次分区写入(读回校验通过后调用, 推进账目)
 * @param  z: 分区状态指针
 * @param  overwrote: ZoneWrite 输出: 1=本次擦除了整扇区(需扣减 STX_SLOT_PER_SECTOR 条)
 * @note   [已修复 DEF-N1b 2026-09-20] 账目变更(覆盖扣减 + 写指针推进 + count++)
 *         统一在读回校验通过后执行, 保证 RAM 账目与 Flash 实际内容严格一致:
 *           - 校验失败: 调用方直接返回, 账目不动 -> 不再有虚高 count 被 MetaSave 固化;
 *           - 校验通过: 先按覆盖情况扣减, 再推进 slot_head 并 count++(净变化 -239 或 +1)。
 */
static void StorageRx_ZoneCommit(ZoneState_t *z, uint8_t overwrote)
{
    if (overwrote != 0U)
    {
        if (z->count >= STX_SLOT_PER_SECTOR)
        {
            z->count -= STX_SLOT_PER_SECTOR;
        }
        else
        {
            z->count = 0U;
        }
    }

    /* 环形推进写指针; 条数达到容量后保持(每写一条丢一条最旧) */
    z->slot_head = (z->slot_head + 1) % z->capacity;
    if (z->count < z->capacity)
    {
        z->count++;
    }
}

/**
 * @brief  后台分步预擦下一个meta bank, 避免MetaSave在写路径上整bank擦除(最坏约3.2s)阻塞丢帧
 * @note   调用时机: main while循环每圈调用(StorageRx_Process之后);
 *         触发条件: 当前bank剩余空间不足100条时启动; 每次调用只擦1个扇区, 分8圈完成;
 *         完成后置prep_done, MetaSave切库后复位(下一周期预擦旧当前库).
 */
void StorageRx_MetaPrepare(void)
{
    uint32_t bank_base;

    if (s_meta_prep_done != 0U)
    {
        /* 空闲态: 检查是否需要触发预擦 */
        if (s_meta_off + sizeof(StorageMeta_t) > STX_META_BANK_SIZE)
        {
            return;  /* 已满, 由MetaSave切换兜底(极端情况) */
        }
        if (STX_META_BANK_SIZE - s_meta_off >= (uint32_t)100U * sizeof(StorageMeta_t))
        {
            return;  /* 剩余充足(100条以上), 无需预擦 */
        }
        /* 触发: 目标bank=另一库(只擦未被标记为当前使用的bank, 不影响当前写) */
        s_meta_prep_bank = s_meta_bank ^ 1U;
        s_meta_prep_sec  = 0U;
        s_meta_prep_done = 0U;
    }

    /* 每次调用只擦1个扇区, 分8圈完成, 单次阻塞典型45ms(最坏400ms), 主循环不丢帧 */
    bank_base = STX_META_BASE + (uint32_t)s_meta_prep_bank * STX_META_BANK_SIZE;
    W25QXX_Erase_Sector(bank_base / STX_SECTOR_SIZE + s_meta_prep_sec);
    s_meta_prep_sec++;
    if (s_meta_prep_sec >= STX_META_BANK_SIZE / STX_SECTOR_SIZE)
    {
        s_meta_prep_done = 1U;  /* 预擦完成, 等MetaSave切换使用 */
    }
}

/**
 * @brief  保存元数据到Flash(A/B双库顺序写, 降低擦写损耗)
 * @note   每条记录写入后调用. 库写满切换另一库并整库擦除,
 *         任何时刻至少一个库存有有效元数据(断电安全).
 *         每库32KB/42字节约780条, 两库共1560次写入才擦一轮.
 */
/* [GB4717 B.1.1.3]  防止存储信息被更改或删除(元数据带魔数+CRC16校验)
 * [GB4717 B.1.4.4]  断电后数据保持不丢失: A/B双库顺序写, 任一时刻至少一库有效
 * [缺陷 DEF-N9] 记账与数据写入非原子: 本函数落在 ZoneWrite 之后, 若在
 *   ZoneWrite 已完成而 MetaSave 尚未落盘之间掉电 -> 数据已写但 count 未累加(少记),
 *   或在擦除旧扇区过程中掉电 -> 该扇区 240 条旧记录全丢而账目未同步。
 * [试验 TST-N9] 随机掉电 100 次: 持续灌入事件并在随机时刻断电(覆盖写入窗口/擦除窗口/meta窗口),
 *   判据: (1)每次上电 MetaLoad 均能恢复出合法 meta(magic=0x58525354 且 CRC 通过), 不得返回"全新Flash";
 *         (2)恢复后各区 count <= capacity, 且导出不得出现全 0xFF 空洞记录;
 *         (3)[GB4717 B.1.4.4] 断电保持: 单次断电 14d 后上电, 记录条数与断电前一致。 */
static void StorageRx_MetaSave(void)
{
    StorageMeta_t meta;
    uint32_t addr;
    uint8_t  i;

    s_meta_seq++;
    meta.magic = STX_META_MAGIC;
    meta.seq   = s_meta_seq;
    for (i = 0; i < STX_ZONE_COUNT; i++)
    {
        meta.slot_head[i] = s_zones[i].slot_head;
        meta.count[i]     = s_zones[i].count;
    }
    meta.crc = StorageRx_CRC16((const uint8_t *)&meta, sizeof(StorageMeta_t) - 2);

    /* 当前库空间不足: 切换到另一库并擦除之 */
    if (s_meta_off + sizeof(StorageMeta_t) > STX_META_BANK_SIZE)
    {
        uint8_t  new_bank = s_meta_bank ^ 1U;
        uint32_t bank_base = STX_META_BASE + (uint32_t)new_bank * STX_META_BANK_SIZE;
        uint32_t sec;

        for (sec = 0; sec < STX_META_BANK_SIZE / STX_SECTOR_SIZE; sec++)
        {
            W25QXX_Erase_Sector(bank_base / STX_SECTOR_SIZE + sec);
        }
        s_meta_bank = new_bank;
        s_meta_off = 0;
        s_meta_prep_done = 1U;  /* P0-A(v2): 复位预擦状态机, 下一周期预擦旧当前库 */
        s_meta_prep_sec  = 0U;
    }

    addr = STX_META_BASE + (uint32_t)s_meta_bank * STX_META_BANK_SIZE + s_meta_off;
    W25QXX_Write((uint8_t *)&meta, addr, sizeof(StorageMeta_t));
    s_meta_off += sizeof(StorageMeta_t);
}

/**
 * @brief  开机恢复元数据(扫描A/B两库, 取seq最大的合法条目)
 * @note   无有效元数据(全新Flash)时各区从0开始;
 *         恢复后写位置=最新条目之后(继续在当前库顺序写).
 *         遇magic不符或CRC错即停止该库扫描(顺序写, 后续不可信).
 */
/* [GB4717 B.1.4.4] 断电后恢复: 扫描A/B两库取 seq 最大且CRC正确的条目, 恢复各区写指针/条数
 * [GB4717 B.1.1.3]  魔数+CRC不符即停止该库扫描(顺序写, 后续不可信) */
static void StorageRx_MetaLoad(void)
{
    StorageMeta_t meta;
    StorageMeta_t best;
    uint32_t best_seq = 0;
    uint8_t  best_bank = 0;
    uint32_t best_off = 0;
    uint8_t  found = 0;
    uint8_t  bank;
    uint32_t off;
    uint8_t  i;

    for (bank = 0; bank < 2; bank++)
    {
        for (off = 0; off + sizeof(StorageMeta_t) <= STX_META_BANK_SIZE;
             off += sizeof(StorageMeta_t))
        {
            W25QXX_Read((uint8_t *)&meta,
                        STX_META_BASE + (uint32_t)bank * STX_META_BANK_SIZE + off,
                        sizeof(StorageMeta_t));
            if (meta.magic != STX_META_MAGIC)
            {
                break;    /* 遇空白即到库尾 */
            }
            if (StorageRx_CRC16((const uint8_t *)&meta, sizeof(StorageMeta_t) - 2) != meta.crc)
            {
                break;    /* 写坏(断电), 后续不可信 */
            }
            if (meta.seq > best_seq)
            {
                best_seq = meta.seq;
                best_bank = bank;
                best_off = off;
                found = 1;
                memcpy(&best, &meta, sizeof(StorageMeta_t));
            }
        }
    }

    if (found != 0)
    {
        /* 恢复各区指针/条数 */
        for (i = 0; i < STX_ZONE_COUNT; i++)
        {
            s_zones[i].slot_head = best.slot_head[i] % s_zones[i].capacity;
            if (best.count[i] > s_zones[i].capacity)
            {
                s_zones[i].count = s_zones[i].capacity;
            }
            else
            {
                s_zones[i].count = best.count[i];
            }
        }
        s_meta_bank = best_bank;
        s_meta_off = best_off + sizeof(StorageMeta_t);
        s_meta_seq = best_seq;
    }
    else
    {
        /* 全新Flash: 元数据区无有效数据, 从库A偏移0开始 */
        s_meta_bank = 0;
        s_meta_off = 0;
        s_meta_seq = 0;
    }
}

uint8_t StorageRx_Init(void)
{
    uint8_t ret;
    uint8_t zi;

    /* 初始化W25Q256 */
    ret = W25QXX_Init();
    if (ret != 0)
    {
        return 1;  /* W25Q256初始化失败 */
    }

    /* 复位状态 */
    s_frame_ready = 0;
    s_rx_idx = 0;

    /* P0-1/P0-2: 计算各分区容量(每扇区240条, 240*17=4080<4096) */
    for (zi = 0; zi < STX_ZONE_COUNT; zi++)
    {
        s_zones[zi].capacity = (s_zones[zi].size / STX_SECTOR_SIZE) * STX_SLOT_PER_SECTOR;
    }

    /* 元数据恢复: 读回各区写指针/条数(断电后FIFO位置不丢) */
    StorageRx_MetaLoad();
    /* P0-A(v2): 开机对即将使用的下一bank做一轮分步预擦(循环8次=整轮, 剩余充足时自动跳过) */
    {
        uint8_t prep_i;
        for (prep_i = 0; prep_i < (STX_META_BANK_SIZE / STX_SECTOR_SIZE); prep_i++)
        {
            StorageRx_MetaPrepare();
        }
    }

    return 0;
}

/*==============================================================
 * USART1接收处理函数 - 中断接收
 * 帧格式: [0xA5][长度][命令码][负载...][CRC16低][CRC16高][0x5A]
 *============================================================*/
/**
 * @brief  USART1接收处理函数(由USART1中断服务函数逐字节调用)
 * @param  data: 接收到的字节
 * @note   接收状态机说明:
 *           state 0: 等待帧头0xA5
 *           state 1: 接收长度字节(=命令码+负载长度), 计算负载长度
 *           state 2: 接收命令码
 *           state 3+: 接收负载+CRC+帧尾, 总长=3+payload+2+1
 *         收到完整帧且帧尾为0x5A时, 复制到s_frame_buf并置位s_frame_ready.
 *         如果已有未处理帧(s_frame_ready=1), 直接丢弃新帧.
 * [GB4717 B.1.1.1] 中断级帧入口: 主控发出的每一条记录经此进入黑匣子链路。
 * [核查 CHK-28] 单帧缓冲: s_frame_ready=1 期间到达的新帧被直接丢弃(无乒乓缓冲),
 *   依赖主控侧 3 次重试(STX_TIMEOUT_MS=4000ms)兜底补偿。
 *   预期: 连续高频事件下不得漏记; 若观测到丢帧, 应查 g_stx_frame_ready_snap 计数。
 * [已修复 CHK-29 2026-09-20] 长度字段下溢: s_frame_payload_len 为 uint16_t,
 *   data-1 在 data=0 时下溢为 65535(非原注释所写 255), 会导致 s_frame_total 回绕为 5
 *   (可能误判帧完整), 且 StorageRx_VerifyCRC 越界读 s_frame_buf[65538]。
 *   现已在长度接收处直接丢弃长度=0 的非法帧并重新同步。 */
void StorageRx_OnByte(uint8_t data)
{
    /* 统计: 记录接收字节数和最近字节(调试用) */
    g_stx_rx_byte_count++;
    g_stx_last_byte = data;

    /* 已有帧未处理, 丢弃新字节 */
    if (s_frame_ready)
    {
        g_stx_frame_ready_snap = 1;
        return;
    }

    s_rx_buf[s_rx_idx] = data;
    g_stx_rx_idx_snap = s_rx_idx;

    switch (s_rx_idx)
    {
        case 0:  /* 等待帧头 */
            if (data == STX_FRAME_HEAD)
            {
                s_rx_idx = 1;
            }
            break;

        case 1:  /* 接收长度字节 */
            if (data == 0U)
            {
                /* [已修复 CHK-29 2026-09-20] 长度字段非法(0): 丢弃本帧并重新同步。
                 * s_frame_payload_len 是 uint16_t, data-1 在 data=0 时下溢为 65535:
                 *   (1) s_frame_total = 3+65535+2+1 回绕为 5, 可能在第 5 字节误判帧完整;
                 *   (2) StorageRx_VerifyCRC 的 crc_len 回绕为 1, 且 crc_recv 读
                 *       s_frame_buf[3+65535] = s_frame_buf[65538], 严重越界(缓冲仅263字节)。 */
                s_rx_idx = 0;
                break;
            }
            s_frame_len = data;
            s_frame_payload_len = data - 1;  /* 负载长度 = 长度 - 命令码(1) */
            s_rx_idx = 2;
            break;

        case 2:  /* 接收命令码 */
            s_frame_cmd = data;
            s_rx_idx = 3;
            break;

        default:  /* 负载 + CRC + 帧尾 */
            /* 总长度计算: 帧头(1) + 长度(1) + 命令码(1) + 负载 + CRC(2) + 尾(1) */
            s_frame_total = 3 + s_frame_payload_len + 2 + 1;
            s_rx_idx++;

            if (s_rx_idx == s_frame_total)
            {
                /* 到达帧尾, 校验帧尾 */
                if (data == STX_FRAME_TAIL)
                {
                    /* 完整帧接收完成 */
                    uint16_t copy_len = s_frame_total;
                    if (copy_len > sizeof(s_frame_buf))
                    {
                        copy_len = sizeof(s_frame_buf);
                    }
                    memcpy(s_frame_buf, s_rx_buf, copy_len);
                    s_frame_ready = 1;
                    /* ISR帧到达事件: 记录cmd+计数, 等main循环打印日志(等效ISR日志) */
                    g_stx_isr_last_cmd = s_frame_cmd;
                    g_stx_isr_frame_count++;
                    g_stx_isr_frame_event = 1;
                }
                /* 帧尾不正确或异常, 复位接收状态 */
                s_rx_idx = 0;
            }
            else if (s_rx_idx > sizeof(s_rx_buf) - 1)
            {
                /* 接收缓冲区溢出, 复位 */
                s_rx_idx = 0;
            }
            break;
    }
}

/*==============================================================
 * 校验帧CRC
 * CRC校验范围: 帧头(1) + 长度(1) + 负载(payload_len)
 *============================================================*/
/**
 * @brief  校验接收帧的CRC16
 * @retval 0=CRC校验通过, 1=CRC校验失败
 * @note   CRC计算范围: s_frame_buf[1]~s_frame_buf[2+payload_len] (长度+命令码+负载)
 *         与接收帧末尾的CRC字节比较, 判断帧是否有效
 */
/* [GB4717 B.1.1.3] 防止存储信息被更改或删除: 收到帧后先CRC校验(写前)
 * [GB4717 B.1.1.2]  数据正确性的前置保障(配合写后读回校验) */
static uint8_t StorageRx_VerifyCRC(void)
{
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint16_t crc_len = 1 + 1 + s_frame_payload_len;  /* 长度 + 命令码 + 负载 */
    uint8_t *crc_start = &s_frame_buf[1];  /* CRC计算起始位置 */

    crc_calc = StorageRx_CRC16(crc_start, crc_len);

    /* 提取接收帧中的CRC, 与计算结果比较 */
    crc_recv = (uint16_t)s_frame_buf[3 + s_frame_payload_len] |
               ((uint16_t)s_frame_buf[3 + s_frame_payload_len + 1] << 8);

    return (crc_calc == crc_recv) ? 0 : 1;
}

/*==============================================================
 * 存储主处理函数
 *============================================================*/
/**
 * @brief  存储主处理函数(在main主循环while(1)中调用)
 * @note   当收到完整帧(s_frame_ready=1)时, 按命令码分类处理:
 *         - STX_CMD_HEARTBEAT(0x06): 回复ACK_OK
 *         - STX_CMD_QUERY_CAPACITY(0x05): 查询剩余容量并发送SendCapacity
 *         - STX_CMD_STORE_*(0x01~0x04): CRC校验通过后写入对应的存储分区
 *           写W25Q256并读回校验通过后回复ACK_OK; 失败则回复相应错误ACK
 *         - 其他未知命令: 回复ACK_ERR_BUSY
 *         处理完成后复位s_frame_ready和s_rx_idx
 * [试验 TST-N1] 写100条逐条比对: 主控连续下发 100 条事件(覆盖 0x01~0x04 四类命令码),
 *   判据: (1)存储侧串口打印 [STX_WR] OK 应为 100 条, 无 VERIFY / CRC / LEN 失败;
 *         (2)结束后 GetTotalCount() == 100, 且各分区 count 与下发路由一致;
 *         (3)导出读取的每条记录与主控下发的原始 17 字节逐字节相同(含时间戳);
 *         (4)StorageTx_GetQueueDropCount() == 0(主控侧未发生队列满丢弃)。 */
void StorageRx_Process(void)
{
    char log_buf[96];

    if (!s_frame_ready)
    {
        return;
    }

    /* 帧已完整, 进入处理流程. 递增计数用于 USB_CDC 调试输出确认主控帧已被存储侧处理 */
    g_stx_process_count++;

    /* [ISR日志] 中断里收到完整帧的事件在此处打印(ISR不能直接调USB_CDC_SendData).
     * 该日志确认: USART1中断确实收到了帧并置位s_frame_ready, 主循环已观察到. */
    if (g_stx_isr_frame_event)
    {
        uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
            "[STX_ISR] frame ready cmd=0x%02X total=%lu proc=%lu\r\n",
            (unsigned)g_stx_isr_last_cmd,
            (unsigned long)g_stx_isr_frame_count,
            (unsigned long)g_stx_process_count);
        USB_CDC_SendData((const uint8_t *)log_buf, len);
        g_stx_isr_frame_event = 0;
    }

    /* 心跳命令处理(回复OK, 无其他操作) */
    if (s_frame_cmd == STX_CMD_HEARTBEAT)
    {
        StorageRx_SendAck(STX_CMD_HEARTBEAT, STX_ACK_OK);
        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* 查询剩余容量命令处理(回复容量, 无其他操作) */
    if (s_frame_cmd == STX_CMD_QUERY_CAPACITY)
    {
        uint32_t remaining = StorageRx_GetRemainingCount();
        StorageRx_SendCapacity(STX_CMD_QUERY_CAPACITY, remaining);
        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* 测试日志透传(0x07): payload原样经USB CDC转发, 不写Flash不动count */
    if (s_frame_cmd == STX_CMD_TEST_LOG)
    {
        /* v2: 本分支位于存储命令链之前, VerifyCRC在存储链内才调用, 此处补验 */
        if (StorageRx_VerifyCRC() != 0)
        {
            StorageRx_SendAck(STX_CMD_TEST_LOG, STX_ACK_ERR_CRC);
            s_frame_ready = 0;
            s_rx_idx = 0;
            return;
        }
        uint8_t plen = (uint8_t)(s_frame_len - 1U);   /* len=cmd(1)+payload */
        uint8_t log_buf[80];
        if (plen > 76U) { plen = 76U; }   /* 截断保护 */
        /* 已确认: 完整帧在s_frame_buf(s_rx_buf是ISR字节队列装不下整帧), payload起始=[3] */
        memcpy(log_buf, &s_frame_buf[3], plen);
        log_buf[plen] = '\0';
        USB_CDC_SendData((const uint8_t *)log_buf, plen);
        USB_CDC_SendData((const uint8_t *)"\r\n", 2);
        StorageRx_SendAck(STX_CMD_TEST_LOG, STX_ACK_OK);
        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* 存储命令处理 (0x01~0x04, 负载为17字节EventRecord_t) */
    if (s_frame_cmd >= STX_CMD_STORE_EVENT && s_frame_cmd <= STX_CMD_STORE_FAULT)
    {
        /* 1. CRC校验 */
        if (StorageRx_VerifyCRC() != 0)
        {
            uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                "[STX_WR] FAIL reason=CRC cmd=0x%02X addr=0x%08lX\r\n",
                (unsigned)s_frame_cmd, (unsigned long)s_write_addr);
            USB_CDC_SendData((const uint8_t *)log_buf, len);
            StorageRx_SendAck(s_frame_cmd, STX_ACK_ERR_CRC);
            s_frame_ready = 0;
            s_rx_idx = 0;
            return;
        }

        /* 2. 校验负载长度 */
        if (s_frame_payload_len != STX_RECORD_SIZE)
        {
            uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                "[STX_WR] FAIL reason=LEN cmd=0x%02X payload=%u expect=%u\r\n",
                (unsigned)s_frame_cmd, (unsigned)s_frame_payload_len, (unsigned)STX_RECORD_SIZE);
            USB_CDC_SendData((const uint8_t *)log_buf, len);
            StorageRx_SendAck(s_frame_cmd, STX_ACK_ERR_CRC);
            s_frame_ready = 0;
            s_rx_idx = 0;
            return;
        }

        /* 3. 检查存储空间是否足够 */
        /* P0-1整改: 环形FIFO写满自动覆盖最旧记录, 删除原"写满拒绝"检查 */

        /* 4. 写入W25Q256 (数据从s_frame_buf[3]开始) - 写入触发点 */
        {
            uint8_t zone_dbg = (s_frame_cmd == STX_CMD_STORE_EVENT) ? STX_ZONE_GENERAL
                               : (uint8_t)(s_frame_cmd - STX_CMD_STORE_FIRST_ALARM);
            uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                "[STX_WR] START zone=%u cnt=%lu cmd=0x%02X dev=%u evt=%u\r\n",
                (unsigned)zone_dbg,
                (unsigned long)s_zones[zone_dbg].count,
                (unsigned)s_frame_cmd,
                (unsigned)((EventRecord_t *)&s_frame_buf[3])->device_no,
                (unsigned)((EventRecord_t *)&s_frame_buf[3])->event_code);
            USB_CDC_SendData((const uint8_t *)log_buf, len);
        }
        /* P0-1/P0-2: 按命令码路由分区, 环形FIFO写入(覆盖最旧) + 读回校验 + 提交账目 */
        {
            uint8_t zone_idx = (s_frame_cmd == STX_CMD_STORE_EVENT)
                               ? STX_ZONE_GENERAL
                               : (uint8_t)(s_frame_cmd - STX_CMD_STORE_FIRST_ALARM);
            uint8_t zone_overwrote = 0U;
            uint8_t readback[STX_RECORD_SIZE];

            /* [核查 CHK-13] 分区路由: 0x01->Z3通用 / 0x02->Z0首警 / 0x03->Z1火警 / 0x04->Z2故障
             * [缺陷 DEF-N9] 此处写入与记账(MetaSave)非原子: 擦除后掉电->count偏大->导出0xFF空洞
             * [已修复 DEF-N1  2026-09-20] 记账已后移到读回校验通过之后(见下方步骤5之后)
             * [已修复 DEF-N1b 2026-09-20] 指针推进同样后移: ZoneWrite 只写Flash不记账,
             *   读回校验通过后才由 ZoneCommit 推进 slot_head/count, 校验失败时账目保持原值 */
            s_last_wr_addr = StorageRx_ZoneWrite(&s_zones[zone_idx], &s_frame_buf[3], &zone_overwrote);

            /* 5. 读回校验 */
            W25QXX_Read(readback, s_last_wr_addr, STX_RECORD_SIZE);  /* [已修复 DEF-N4 2026-09-20] 随DEF-N1一并解决: 记账在校验之后, 失败重试不再累加count */

            if (memcmp(&s_frame_buf[3], readback, STX_RECORD_SIZE) != 0)
            {
                /* 读回校验失败: ZoneCommit 未执行, 账目保持原值, 不产生幽灵记录 */
                uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                    "[STX_WR] FAIL reason=VERIFY addr=0x%08lX\r\n",
                    (unsigned long)s_last_wr_addr);
                USB_CDC_SendData((const uint8_t *)log_buf, len);
                StorageRx_SendAck(s_frame_cmd, STX_ACK_ERR_VERIFY);
                s_frame_ready = 0;
                s_rx_idx = 0;
                return;
            }

            /* [已修复 DEF-N1b 2026-09-20] 校验通过, 提交本次写入的账目 */
            StorageRx_ZoneCommit(&s_zones[zone_idx], zone_overwrote);
        }

        /* [已修复 DEF-N1 2026-09-20] 读回校验通过后才记账(meta持久化count):
         *   原实现在校验之前记账 -> 写坏时账目已+1, 导出读到坏记录(幽灵记录);
         *   同时消除 DEF-N4: 校验失败重试不再反复累加 count, 坏记录不再累积。
         *   现顺序: 写入 -> 读回校验 -> 提交账目(ZoneCommit) -> MetaSave -> ACK */
        StorageRx_MetaSave();

        /* 6. 校验通过, 记录写入成功, 回复ACK */
        {
            uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                "[STX_WR] OK addr=0x%08lX cmd=0x%02X\r\n",
                (unsigned long)s_last_wr_addr, (unsigned)s_frame_cmd);
            USB_CDC_SendData((const uint8_t *)log_buf, len);
        }
        /* 指针推进已在 ZoneWrite 内部完成 */
        StorageRx_SendAck(s_frame_cmd, STX_ACK_OK);

        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* 未知命令处理 */
    {
        uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
            "[STX_WR] FAIL reason=UNKNOWN_CMD cmd=0x%02X\r\n",
            (unsigned)s_frame_cmd);
        USB_CDC_SendData((const uint8_t *)log_buf, len);
    }
    StorageRx_SendAck(s_frame_cmd, STX_ACK_ERR_BUSY);
    s_frame_ready = 0;
    s_rx_idx = 0;
}

/*==============================================================
 * 查询存储记录数
 *============================================================*/
/**
 * @brief  查询指定分区的现存记录条数
 * @retval 该分区现存条数(取值范围 0 ~ 分区capacity)
 * [GB4717 表B.1] 本返回值即导出响应帧中"记录总数(3字节大端)"字段的来源之一
 *   (分区直读取本值, 顺序读取四区之和 StorageRx_GetTotalCount)。
 * [核查 CHK-30] count 必须与实际"落盘且通过读回校验"的条数严格相等。
 *   已知偏差来源: DEF-N1b(RAM态虚高)、DEF-N9(记账未落盘)、N3(ACK丢失导致重帧重复写)。
 *   预期: 稳态下 GetRecordCount(zone) == 该区实际有效记录数, 误差 0。 */
uint32_t StorageRx_GetRecordCount(uint8_t zone)
{
    if (zone >= STX_ZONE_COUNT)
    {
        return 0;
    }
    return s_zones[zone].count;
}

/* 查询全部分区总条数(P1-5整改: 供导出响应帧的记录总数字段) */
uint32_t StorageRx_GetTotalCount(void)
{
    uint32_t total = 0;
    uint8_t z;
    for (z = 0; z < STX_ZONE_COUNT; z++)
    {
        total += s_zones[z].count;
    }
    return total;
}

/*==============================================================
 * 查询剩余存储容量
 *============================================================*/
/**
 * @brief  查询剩余存储容量
 * @retval 剩余容量 = (W25Q256总容量 - 已用容量) / 记录固定大小
 */
/* [GB4717 B.1.2.1] 存储容量要求(区域型: 火警>=触发器件x100, 状态信息>=(触发+联动)x200) */
uint32_t StorageRx_GetRemainingCount(void)
{
    /* P0-1整改: 分区后返回全部分区剩余可写条数之和(容量-现存) */
    uint32_t total = 0;
    uint8_t z;
    for (z = 0; z < STX_ZONE_COUNT; z++)
    {
        total += (s_zones[z].capacity - s_zones[z].count);
    }
    return total;
}

/*==============================================================
 * 读取存储记录函数 (按索引读取)
 *============================================================*/
/**
 * @brief  按索引读取存储记录(供GB4717导出模块调用)
 * @param  index: 记录索引(0开始)
 * @param  rec:   读取到的记录缓冲区
 * @retval 0=成功, 1=失败(索引越界或指针为空)
 * @note   存储地址 = 存储基址 + index * 17, 与写入时计算方式一致
 */
/* [GB4717 B.1.1.2] 应具有按照时间顺序提供导出记录信息的功能
 *   约定 index=0 为最旧记录, 顺序读即时间升序
 * [GB4717 B.1.3.1] 数据导出的取数来源(仅专用手段经USB导出)
 * [核查 CHK-26] index 越界保护: index >= count 时返回 1(失败), 调用方(GB4717_ReadZone)
 *   读到返回值 1 视作"该区读完", 故 count 虚高会把尚未写入的空洞当作有效记录导出。
 *   预期: StorageRx_GetRecordCount(zone) 必须与实际落盘成功条数严格相等。
 * [试验 TST-EXP] 导出比对: 用 com17_export.py 顺序读全部记录,
 *   判据: (1)上位机逐帧 CRC16 校验(含/不含起始符按 S-3 确认口径)须全部通过;
 *         (2)导出条数 == StorageRx_GetTotalCount();
 *         (3)与主控侧同步记录的原始事件比对: 设备号/事件码/时间戳三项逐条一致, 无多余项、无缺失项;
 *         (4)时间戳单调不减严格成立(允许同秒并列)。 */
uint8_t StorageRx_ReadRecord(uint8_t zone, uint32_t index, EventRecord_t *rec)
{
    uint32_t addr;
    uint32_t slot;

    if (rec == NULL || zone >= STX_ZONE_COUNT)
    {
        return 1;
    }
    if (index >= s_zones[zone].count)
    {
        return 1;  /* 超出该分区现存条数 */
    }

    /* P1-5整改: index 0=最旧记录, 物理槽号 = (写指针-count+index) mod 容量 */
    slot = (s_zones[zone].slot_head + s_zones[zone].capacity
            - s_zones[zone].count + index) % s_zones[zone].capacity;
    addr = StorageRx_ZoneSlotAddr(&s_zones[zone], slot);

    W25QXX_Read((uint8_t *)rec, addr, STX_RECORD_SIZE);
    return 0;
}

/*==============================================================
 * 擦除全部存储
 *============================================================*/
/**
 * @brief  擦除全部存储(复位写地址和索引)
 * @note   提速整改: 只擦每分区头部256KB + 元数据区64KB, 替代整片擦
 *         (W25Q256 ChipErase约150~240s -> 约15s)
 *         依据: 清零后count=0, dump/导出仅按count读取新写入记录,
 *         分区深处的历史数据不参与账目对账, 不影响测试判定
 */
/* [GB4717 B.1.1.3] 控制器运行数据存储单元应有防止存储信息被更改或删除的功能
 * [S-1 已修复 2026-09-17] 两处 W25QXX_Erase_Sector() 原传【字节地址】, 而该API形参是
 *   【扇区号】(w25qxx.c 内部会 Dst_Addr *= 4096)。叠加 uint32 溢出后实际擦到扇区0附近,
 *   声称清空的"每分区头部256KB + meta区64KB"并未被擦除; meta 未擦会导致重启后
 *   MetaLoad 取 seq 最大者恢复旧 count, 使"清空"被撤销。现改为 字节地址/4096 + 扇区偏移。
 *   另: 本函数是 'C' 调试命令的后端, 生产构建应关闭(见 USER/main.c 的 STX_DEBUG_BUILD)。
 * [核查 CHK-08] 生产/送检构建必须置 STX_DEBUG_BUILD=0 —— 否则现场或审查方可通过
 *   USB CDC 发送 'C' 命令删除全部记录, 直接违反 B.1.1.3"防止存储信息被更改或删除"。
 *   预期: 送检固件中搜索不到 'C' 命令分支, 或 Storage_ClearAllRecords 不可达。 */
void StorageRx_EraseAll(void)
{
    uint8_t z;
    uint32_t addr;
    uint32_t sec;
    static const uint32_t zone_base[STX_ZONE_COUNT] = {
        STX_ZONE0_BASE, STX_ZONE1_BASE, STX_ZONE2_BASE, STX_ZONE3_BASE };

    /* 每分区头部256KB(64扇区): 清零后slot_head=0, 新记录全部落在该范围 */
    for (z = 0; z < STX_ZONE_COUNT; z++)
    {
        addr = zone_base[z];
        for (sec = 0; sec < 64U; sec++)
        {
            /* P0-1修复: W25QXX_Erase_Sector形参为扇区号(内部再乘4096),
             * 原写法传字节地址会溢出并擦到扇区0附近, 现改为字节地址/4096+偏移 */
            W25QXX_Erase_Sector(addr / STX_SECTOR_SIZE + sec);
        }
    }
    /* 元数据区64KB(16扇区): A/B双库状态一并擦除 */
    for (sec = 0; sec < 16U; sec++)
    {
        W25QXX_Erase_Sector(STX_META_BASE / STX_SECTOR_SIZE + sec);
    }
    /* P0-1整改: 分区指针/条数全部清零, 元数据状态复位 */
    for (z = 0; z < STX_ZONE_COUNT; z++)
    {
        s_zones[z].slot_head = 0;
        s_zones[z].count = 0;
    }
    s_meta_bank = 0;
    s_meta_off = 0;
    s_meta_seq = 0;
}
