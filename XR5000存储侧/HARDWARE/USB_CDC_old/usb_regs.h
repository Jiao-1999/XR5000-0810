/**
 * @file    usb_regs.h
 * @brief   STM32F103 USB 全速设备寄存器定义
 * @details stm32f10x.h 提供的 USB 外设寄存器访问宏不够用, 此处直接定义
 *          USB_TypeDef 寄存器映射, 并补充寄存器位定义和 PMA(包缓冲区)访问宏.
 *
 *   USB 基地址: 0x40005C00 (APB1)
 *   PMA 基地址: 0x40006000 (512 字节)
 *
 *   寄存器偏移:
 *     USB_EP0R..EP7R : 0x00 + 4*n
 *     USB_CNTR       : 0x40
 *     USB_ISTR       : 0x44
 *     USB_FNR        : 0x48
 *     USB_DADDR      : 0x4C
 *     USB_BTABLE     : 0x50
 */
#ifndef __USB_REGS_H
#define __USB_REGS_H

#include "stm32f10x.h"

/*==============================================================
 * 基地址
 *============================================================*/
#define USB_BASE                (0x40005C00u)
#define USB_PMA_BASE            (0x40006000u)

/*==============================================================
 * 寄存器访问宏
 *============================================================*/
#define REG16(addr)             (*((volatile uint16_t *)(addr)))

#define USB                     ((USB_TypeDef *)(USB_BASE))
#define USB_CNTR                REG16(USB_BASE + 0x40)
#define USB_ISTR                REG16(USB_BASE + 0x44)
#define USB_FNR                 REG16(USB_BASE + 0x48)
#define USB_DADDR               REG16(USB_BASE + 0x4C)
#define USB_BTABLE              REG16(USB_BASE + 0x50)

#define USB_EP(n)               REG16(USB_BASE + 0x00 + 4u*(n))

/*==============================================================
 * 端点类型
 *============================================================*/
#define EP_TYPE_BULK            0x0000u
#define EP_TYPE_CONTROL         0x0200u
#define EP_TYPE_ISO             0x0400u
#define EP_TYPE_INTERRUPT       0x0600u

/* 端点状态位 (STAT_TX/STAT_RX) */
#define EP_STAT_DIS             0x0000u     /* 禁用 */
#define EP_STAT_STALL           0x0010u     /* STALL (TX) / 0x1000 (RX) */
#define EP_STAT_NAK             0x0020u     /* NAK (TX) / 0x2000 (RX) */
#define EP_STAT_VALID           0x0030u     /* 有效 (TX) / 0x3000 (RX) */

#define EP_STAT_TX_VALID        0x0030u
#define EP_STAT_TX_NAK          0x0020u
#define EP_STAT_TX_STALL        0x0010u
#define EP_STAT_TX_DIS          0x0000u

#define EP_STAT_RX_VALID        0x3000u
#define EP_STAT_RX_NAK          0x2000u
#define EP_STAT_RX_STALL        0x1000u
#define EP_STAT_RX_DIS          0x0000u

/* EP_KIND 功能位 */
#define EP_KIND_DBL_BUF         0x0100u
#define EP_KIND_STATUS_OUT      0x0100u

/* 状态机位掩码 (写 EPnR 时保持不变的位) */
#define EP_T_MASK               (0x0030u)   /* STAT_TX */
#define EP_R_MASK               (0x3000u)   /* STAT_RX */
#define EP_T_TOGGLE             (0x0040u)   /* DTOG_TX */
#define EP_R_TOGGLE             (0x4000u)   /* DTOG_RX */
#define EP_CTR_TX               (0x0080u)
#define EP_CTR_RX               (0x8000u)
#define EP_SETUP                (0x0800u)

#define EP_MASK_REG             (0x0F1Fu)   /* 保留位: EA/STAT_TX0/KIND/类型/SETUP */

/*==============================================================
 * PMA(包缓冲区)访问宏
 *   STM32F103 PMA 为 16 位宽, 但 CPU 通过 APB1 总线以 32 位对齐访问,
 *   每个 16 位 PMA 字占用 4 字节 CPU 地址空间 (2 字节数据 + 2 字节填充)。
 *   因此: CPU 地址 = USB_PMA_BASE + 4 * PMA字索引
 *   BTABLE ADDR 字段存储的是 PMA 字索引 (不是字节偏移)。
 *============================================================*/
#define PMA_BUF(offset)         REG16(USB_PMA_BASE + 4u*(offset))

/* BTABLE 中每个端点占 4 个 16 位字 (4×4=16 字节 CPU 地址空间) */
#define PMA_ADDR_TX(ep)         REG16(USB_PMA_BASE + 16u*(ep) + 0u)
#define PMA_COUNT_TX(ep)        REG16(USB_PMA_BASE + 16u*(ep) + 4u)
#define PMA_ADDR_RX(ep)         REG16(USB_PMA_BASE + 16u*(ep) + 8u)
#define PMA_COUNT_RX(ep)        REG16(USB_PMA_BASE + 16u*(ep) + 12u)

/*==============================================================
 * USB_TypeDef 寄存器映射 (与 stm32f10x.h 保持一致)
 *============================================================*/
typedef struct {
    volatile uint16_t EP0R;      /* 0x00 */
    volatile uint16_t RESERVED0; /* 0x02 */
    volatile uint16_t EP1R;      /* 0x04 */
    volatile uint16_t RESERVED1; /* 0x06 */
    volatile uint16_t EP2R;      /* 0x08 */
    volatile uint16_t RESERVED2; /* 0x0A */
    volatile uint16_t EP3R;      /* 0x0C */
    volatile uint16_t RESERVED3; /* 0x0E */
    volatile uint16_t EP4R;      /* 0x10 */
    volatile uint16_t RESERVED4; /* 0x12 */
    volatile uint16_t EP5R;      /* 0x14 */
    volatile uint16_t RESERVED5; /* 0x16 */
    volatile uint16_t EP6R;      /* 0x18 */
    volatile uint16_t RESERVED6; /* 0x1A */
    volatile uint16_t EP7R;      /* 0x1C */
    volatile uint16_t RESERVED7; /* 0x1E */
    volatile uint16_t RESERVED8; /* 0x20 - 0x3F */
    volatile uint16_t CNTR;      /* 0x40 */
    volatile uint16_t ISTR;      /* 0x44 */
    volatile uint16_t FNR;       /* 0x48 */
    volatile uint16_t DADDR;     /* 0x4C */
    volatile uint16_t BTABLE;    /* 0x50 */
} USB_TypeDef;

#endif /* __USB_REGS_H */
