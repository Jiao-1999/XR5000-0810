/**
 * @file    bsp_storage_rx.c
 * @brief   ?洢??????? - ????????洢?????д??W25Q256
 * @details ???USART1(PA9=TX/PA10=RX, 115200 8N1, ?????ж????)???????????????????,
 *          ??????д??W25Q256 SPI Flash?????У??, ?????ACK???.
 *          ????????: USART1?ж????StorageRx_OnByte(), ?????????StorageRx_Process().
 *
 *   ????(????->?洢??):
 *     [0xA5][????][??????][???17???EventRecord_t][CRC16??][CRC16??][0x5A]
 *     ???? = ??????(1) + ??????
 *     CRC16: MODBUS?????0xA001, ??Χ=????+??????+???, ????????
 *
 *   ??????: 0x01~0x04?洢???, 0x05???????, 0x06????
 *   ?洢????: ???д??W25Q256(32MB, ???0x00000000), д?????洢??????
 */
#include "bsp_storage_rx.h"
#include "w25qxx.h"
#include "usart.h"
#include "delay.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>

/*==============================================================
 * ???????
 *============================================================*/
#define STX_RECORD_SIZE      sizeof(EventRecord_t)  /* 17??? */
#define STX_FLASH_BASE_ADDR  0x00000000             /* ????洢?????? */
#define STX_FLASH_MAX_ADDR   0x02000000             /* W25Q256????32MB = 0x02000000 */

/*==============================================================
 * ???????
 *============================================================*/
static volatile uint8_t  s_frame_ready = 0;     /* ???????? */
static volatile uint32_t s_write_addr   = 0;     /* ???д???? */
static uint8_t  s_rx_buf[32];                   /* ?????????(23?????) */
static uint8_t  s_rx_idx = 0;                    /* ??????????? */
static uint8_t  s_frame_cmd;                     /* ?????????? */
static uint8_t  s_frame_len;                     /* ??????????? */
static uint16_t s_frame_payload_len;             /* ?????????? */
static uint8_t  s_frame_buf[STX_MAX_PAYLOAD + 8];/* ????????? */
static uint16_t s_frame_total;                   /* ???????? */

/*==============================================================
 * ??????? (?? main.c ??? USB_CDC ???, ??λ??????洢????·????)
 *   g_stx_rx_byte_count: ??????????????? (???USART1?ж?+1)
 *   g_stx_last_byte:     ??????????? (???????)
 *   g_stx_rx_idx_snap:   s_rx_idx ???? (????????????, 0=????)
 *   g_stx_frame_ready_snap: s_frame_ready ???? (1=??????????????)
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
 * ???????????
 *============================================================*/
static uint16_t StorageRx_CRC16(const uint8_t *data, uint16_t len);
static uint32_t StorageRx_ZoneSlotAddr(const ZoneState_t *z, uint32_t slot);
static uint32_t StorageRx_ZoneWrite(ZoneState_t *z, const uint8_t *rec);
static void StorageRx_MetaSave(void);
static void StorageRx_MetaLoad(void);
static void StorageRx_SendByte(uint8_t data);
static void StorageRx_SendAck(uint8_t cmd, uint8_t ack_code);
static void StorageRx_SendCapacity(uint8_t cmd, uint32_t remaining);
static uint8_t StorageRx_VerifyCRC(void);

/*==============================================================
 * CRC16У?? (?????0xA001, ??MODBUS CRC16)
 *============================================================*/
/**
 * @brief  ????MODBUS CRC16 (?????0xA001, ????????)
 * @param  data: ???????????????
 * @param  len:  ???????
 * @retval CRC16?
 * @note   ????0xFFFF, ?????????λ????8??, ???λ?1????0xA001
 */
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
    return crc;  /* ??????????? */
}

/*==============================================================
 * ???USART1??????????
 *============================================================*/
/**
 * @brief  ???USART1??????????????
 * @param  data: ?????????
 * @note   ???USART_FLAG_TC(???????)??????
 */
static void StorageRx_SendByte(uint8_t data)
{
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*==============================================================
 * ????ACK????
 * ???: [0xA5][0x02][cmd][ack_code][CRC16??][CRC16??][0x5A]
 *============================================================*/
/**
 * @brief  ????ACK????(????洢????/?????????)
 * @param  cmd: ??????(???????????????????)
 * @param  ack_code: ????? (STX_ACK_OK / STX_ACK_ERR_CRC / STX_ACK_ERR_FULL ??)
 * @note   ??????????(??7???):
 *           [0] 0xA5 ??
 *           [1] 0x02 ????(??????+?????=2???)
 *           [2] cmd  ?????????
 *           [3] ack_code ?????
 *           [4] CRC16?????
 *           [5] CRC16?????
 *           [6] 0x5A ?β
 *         CRC??Χ: ????+??????+????? = 3???
 */
static void StorageRx_SendAck(uint8_t cmd, uint8_t ack_code)
{
    uint8_t buf[8];
    uint16_t crc;
    uint8_t i;

    buf[0] = STX_FRAME_HEAD;
    buf[1] = 0x02;          /* ???? = ??????(1) + ?????(1) = 2 */
    buf[2] = cmd;           /* ????????? */
    buf[3] = ack_code;      /* ????? */

    /* CRC??Χ: ???? + ?????? + ????? = 3??? */
    crc = StorageRx_CRC16(&buf[1], 3);
    buf[4] = (uint8_t)(crc & 0xFF);         /* CRC?? */
    buf[5] = (uint8_t)((crc >> 8) & 0xFF);  /* CRC?? */
    buf[6] = STX_FRAME_TAIL;

    for (i = 0; i < 7; i++)
    {
        StorageRx_SendByte(buf[i]);
    }
}

/*==============================================================
 * ???????????????
 * ???: [0xA5][0x05][cmd][???????4???][CRC16??][CRC16??][0x5A]
 *============================================================*/
/**
 * @brief  ???????????????(????STX_CMD_QUERY_CAPACITY????????)
 * @param  cmd: ??????(0x05)
 * @param  remaining: ????洢?????
 * @note   ??????????(??10???):
 *           [0] 0xA5 ??
 *           [1] 0x05 ????(??????+????4???=5???)
 *           [2] cmd  ?????????
 *           [3-6] remaining ???????(С??: ????????)
 *           [7] CRC16?????
 *           [8] CRC16?????
 *           [9] 0x5A ?β
 *         CRC??Χ: ????+??????+???? = 6???
 */
static void StorageRx_SendCapacity(uint8_t cmd, uint32_t remaining)
{
    uint8_t buf[12];
    uint16_t crc;
    uint8_t i;

    buf[0] = STX_FRAME_HEAD;
    buf[1] = 0x05;          /* ???? = ??????(1) + ????(4) = 5 */
    buf[2] = cmd;
    buf[3] = (uint8_t)(remaining & 0xFF);
    buf[4] = (uint8_t)((remaining >> 8) & 0xFF);
    buf[5] = (uint8_t)((remaining >> 16) & 0xFF);
    buf[6] = (uint8_t)((remaining >> 24) & 0xFF);

    /* CRC??Χ: ???? + ?????? + ???? = 6??? */
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
 * ??????洢???????
 *============================================================*/
/**
 * @brief  ??????洢???????
 * @retval 0=???, 1=W25Q256????????
 * @note   ?????W25Q256 SPI Flash, ??λ?????д????洢???(0x00000000)
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
static uint32_t StorageRx_ZoneWrite(ZoneState_t *z, const uint8_t *rec)
{
    uint32_t addr = StorageRx_ZoneSlotAddr(z, z->slot_head);
    uint8_t  tmp[STX_RECORD_SIZE];
    uint8_t  i;
    uint8_t  blank = 1;

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
        /* 覆盖旧记录: 擦除目标扇区(该扇区旧记录均为待淘汰的最旧数据) */
        W25QXX_Erase_Sector(addr / STX_SECTOR_SIZE);
    }

    W25QXX_Write((uint8_t *)rec, addr, STX_RECORD_SIZE);

    /* 环形推进写指针; 条数达到容量后保持(每写一条丢一条最旧) */
    z->slot_head = (z->slot_head + 1) % z->capacity;
    if (z->count < z->capacity)
    {
        z->count++;
    }

    return addr;
}

/**
 * @brief  保存元数据到Flash(A/B双库顺序写, 降低擦写损耗)
 * @note   每条记录写入后调用. 库写满切换另一库并整库擦除,
 *         任何时刻至少一个库存有有效元数据(断电安全).
 *         每库32KB/42字节约780条, 两库共1560次写入才擦一轮.
 */
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

    /* ?????W25Q256 */
    ret = W25QXX_Init();
    if (ret != 0)
    {
        return 1;  /* W25Q256???????? */
    }

    /* ??λ?? */
    s_frame_ready = 0;
    s_rx_idx = 0;

    /* P0-1/P0-2: 计算各分区容量(每扇区240条, 240*17=4080<4096) */
    for (zi = 0; zi < STX_ZONE_COUNT; zi++)
    {
        s_zones[zi].capacity = (s_zones[zi].size / STX_SECTOR_SIZE) * STX_SLOT_PER_SECTOR;
    }

    /* 元数据恢复: 读回各区写指针/条数(断电后FIFO位置不丢) */
    StorageRx_MetaLoad();

    return 0;
}

/*==============================================================
 * USART1????????? - ?????
 * ????: [0xA5][????][??????][???...][CRC16??][CRC16??][0x5A]
 *============================================================*/
/**
 * @brief  USART1?????????(??USART1?ж???????????????)
 * @param  data: ??????????
 * @note   ???????????:
 *           ????0: ?????0xA5
 *           ????1: ??????????(=??????+??????), ??????????
 *           ????2: ?????????
 *           ????3+: ???????+CRC+?β, ????=3+payload+2+1
 *         ???????????β?0x5A?, ?????s_frame_buf????λs_frame_ready.
 *         ??????δ??????(s_frame_ready=1), ???????????????.
 */
void StorageRx_OnByte(uint8_t data)
{
    /* ????: ????????????????(??????????) */
    g_stx_rx_byte_count++;
    g_stx_last_byte = data;

    /* ???????δ??????, ?????????? */
    if (s_frame_ready)
    {
        g_stx_frame_ready_snap = 1;
        return;
    }

    s_rx_buf[s_rx_idx] = data;
    g_stx_rx_idx_snap = s_rx_idx;

    switch (s_rx_idx)
    {
        case 0:  /* ????? */
            if (data == STX_FRAME_HEAD)
            {
                s_rx_idx = 1;
            }
            break;

        case 1:  /* ??????? */
            s_frame_len = data;
            s_frame_payload_len = data - 1;  /* ?????? = ???? - ??????(1) */
            s_rx_idx = 2;
            break;

        case 2:  /* ?????? */
            s_frame_cmd = data;
            s_rx_idx = 3;
            break;

        default:  /* ??? + CRC + ?β */
            /* ???????????: ?(1) + ????(1) + ????(1) + ??? + CRC(2) + β(1) */
            s_frame_total = 3 + s_frame_payload_len + 2 + 1;
            s_rx_idx++;

            if (s_rx_idx == s_frame_total)
            {
                /* ????????, ????β */
                if (data == STX_FRAME_TAIL)
                {
                    /* ?????????? */
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
                /* ?????β??????, ????λ???? */
                s_rx_idx = 0;
            }
            else if (s_rx_idx > sizeof(s_rx_buf) - 1)
            {
                /* ???????????, ??λ */
                s_rx_idx = 0;
            }
            break;
    }
}

/*==============================================================
 * У???CRC
 * CRC??Χ: ????(1) + ??????(1) + ???(payload_len)
 *============================================================*/
/**
 * @brief  У?鵱????CRC16
 * @retval 0=CRCУ?????, 1=CRCУ?????
 * @note   CRC????Χ: s_frame_buf[1]~s_frame_buf[2+payload_len] (????+????+???)
 *         ?????CRC????????????, ????????
 */
static uint8_t StorageRx_VerifyCRC(void)
{
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint16_t crc_len = 1 + 1 + s_frame_payload_len;  /* ???? + ???? + ??? */
    uint8_t *crc_start = &s_frame_buf[1];  /* ???????ο?? */

    crc_calc = StorageRx_CRC16(crc_start, crc_len);

    /* ?????CRC????????, ???????? */
    crc_recv = (uint16_t)s_frame_buf[3 + s_frame_payload_len] |
               ((uint16_t)s_frame_buf[3 + s_frame_payload_len + 1] << 8);

    return (crc_calc == crc_recv) ? 0 : 1;
}

/*==============================================================
 * ?洢?????????????
 *============================================================*/
/**
 * @brief  ?洢?????????????(??main??while(1)?е???)
 * @note   ????????(s_frame_ready=1)??, ??????????????:
 *         - STX_CMD_HEARTBEAT(0x06): ????ACK_OK
 *         - STX_CMD_QUERY_CAPACITY(0x05): ??????????????SendCapacity
 *         - STX_CMD_STORE_*(0x01~0x04): CRCУ?????????У????洢??У???
 *           дW25Q256?????У???????д??????ACK_OK; ????????????ACK
 *         - ????????: ??ACK_ERR_BUSY
 *         ????????λs_frame_ready??s_rx_idx
 */
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

    /* ????????????(?????, ??????) */
    if (s_frame_cmd == STX_CMD_HEARTBEAT)
    {
        StorageRx_SendAck(STX_CMD_HEARTBEAT, STX_ACK_OK);
        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* ???????????????(?????, ???????????) */
    if (s_frame_cmd == STX_CMD_QUERY_CAPACITY)
    {
        uint32_t remaining = StorageRx_GetRemainingCount();
        StorageRx_SendCapacity(STX_CMD_QUERY_CAPACITY, remaining);
        s_frame_ready = 0;
        s_rx_idx = 0;
        return;
    }

    /* ?????洢???? (0x01~0x04, ????17???EventRecord_t) */
    if (s_frame_cmd >= STX_CMD_STORE_EVENT && s_frame_cmd <= STX_CMD_STORE_FAULT)
    {
        /* 1. CRCУ?? */
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

        /* 2. ????????? */
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

        /* 3. ???洢??????? */
        /* P0-1整改: 环形FIFO写满自动覆盖最旧记录, 删除原"写满拒绝"检查 */

        /* 4. д??W25Q256 (????s_frame_buf[3]???) - 写入触发点 */
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
        /* P0-1/P0-2: 按命令码路由分区, 环形FIFO写入(覆盖最旧) + 指针持久化 */
        {
            uint8_t zone_idx = (s_frame_cmd == STX_CMD_STORE_EVENT)
                               ? STX_ZONE_GENERAL
                               : (uint8_t)(s_frame_cmd - STX_CMD_STORE_FIRST_ALARM);
            s_last_wr_addr = StorageRx_ZoneWrite(&s_zones[zone_idx], &s_frame_buf[3]);
            StorageRx_MetaSave();
        }

        /* 5. ???У?? */
        {
            uint8_t readback[STX_RECORD_SIZE];
            W25QXX_Read(readback, s_last_wr_addr, STX_RECORD_SIZE);

            if (memcmp(&s_frame_buf[3], readback, STX_RECORD_SIZE) != 0)
            {
                /* ???????? */
                uint16_t len = (uint16_t)snprintf(log_buf, sizeof(log_buf),
                    "[STX_WR] FAIL reason=VERIFY addr=0x%08lX\r\n",
                    (unsigned long)s_last_wr_addr);
                USB_CDC_SendData((const uint8_t *)log_buf, len);
                StorageRx_SendAck(s_frame_cmd, STX_ACK_ERR_VERIFY);
                s_frame_ready = 0;
                s_rx_idx = 0;
                return;
            }
        }

        /* 6. У?????, ????д???, ????ACK */
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

    /* δ????? */
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
 * ?????洢?????
 *============================================================*/
/**
 * @brief  ?????洢?????
 * @retval ??洢????? = ???д??? / ????????????(17)
 */
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
 * ???????洢?????
 *============================================================*/
/**
 * @brief  ???????洢?????
 * @retval ??????? = (W25Q256?????? - ??????) / ????????????
 */
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
 * ?????????????? (?????????)
 *============================================================*/
/**
 * @brief  ??????????????(??GB4717??????????)
 * @param  index: ???????(0???)
 * @param  rec:   ?????????????
 * @retval 0=???, 1=???(?????????????????д??Χ)
 * @note   ?????? = ?洢??? + index * 17, ???????????д??????
 */
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
 * ????????洢
 *============================================================*/
/**
 * @brief  ????????洢(???????W25Q256, ??λд??????)
 * @note   ????W25QXX_Erase_Chip??????, ??????(????)
 */
void StorageRx_EraseAll(void)
{
    uint8_t z;

    W25QXX_Erase_Chip();
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
