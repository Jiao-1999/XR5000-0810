/**
 * @file    usb_cdc.c
 * @brief   STM32F103 USB CDC virtual serial port driver (register-level)
 * @details STM32F103 USB full-speed device peripheral, CDC ACM class.
 *          PC recognizes a COM port, use pyserial to send/receive data.
 *
 *   Endpoint layout:
 *     EP0 (control)     : SETUP + CDC class requests (SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE)
 *     EP1 (interrupt IN) : CDC notification (not actively used, keep NAK)
 *     EP2 (bulk OUT)     : PC -> device (receive GB4717 commands)
 *     EP3 (bulk IN)      : device -> PC (send GB4717 responses)
 *
 *   PMA buffer layout (word offsets, each word = 2 bytes, PMA = 256 words = 512 bytes):
 *     0x00-0x0F  BTABLE (4 endpoints x 4 words = 16 words)
 *     0x10-0x2F  EP0 RX  (64 bytes = 32 words)
 *     0x30-0x4F  EP0 TX  (64 bytes = 32 words)
 *     0x50-0x53  EP1 TX  (8 bytes = 4 words)
 *     0x54-0x73  EP2 RX  (64 bytes = 32 words)
 *     0x74-0x93  EP3 TX  (64 bytes = 32 words)
 *
 *   Init flow (proven saewave/ST UM0424 pattern):
 *     USB_CDC_Init: FRES pulse -> clear regs -> CNTR=RESETM only -> NVIC enable
 *       (PDWN stays 0 = pull-up active, DADDR.EF NOT set here)
 *     Host sees pull-up, sends RESET -> RESET ISR fires
 *     RESET handler: configure EPs -> CNTR=CTRM|RESETM|SUSPM -> DADDR=EF
 *     Host sends SETUP -> CTR ISR -> enumerate
 */
#include "usb_cdc.h"
#include "usb_regs.h"
#include <string.h>

/*==============================================================
 * PMA buffer layout (word offsets, PMA_ADDR values)
 *============================================================*/
#define PMA_BTABLE               0x00
#define PMA_EP0_RX_ADDR          0x10
#define PMA_EP0_TX_ADDR          0x30
#define PMA_EP1_TX_ADDR          0x50
#define PMA_EP2_RX_ADDR          0x54
#define PMA_EP3_TX_ADDR          0x74

#define EP0_PACKET_SIZE          64
#define EP1_PACKET_SIZE          8
#define EP2_PACKET_SIZE          64
#define EP3_PACKET_SIZE          64

/*==============================================================
 * State variables
 *============================================================*/
static volatile uint8_t  s_usb_configured = 0;    /* 1 after SET_CONFIGURATION */
static volatile uint8_t  s_usb_address = 0;       /* pending device address */
static volatile uint8_t  s_usb_address_pending = 0;

/* RX ring buffer (EP2 OUT -> ring buf -> ReadByte) */
static volatile uint8_t  s_rx_buf[USB_CDC_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint16_t s_rx_count = 0;

/* TX (EP3 IN) */
static volatile uint8_t  s_tx_buf[EP3_PACKET_SIZE];
static volatile uint16_t s_tx_len = 0;
static volatile uint8_t  s_tx_busy = 0;
static volatile uint16_t s_tx_remaining = 0;
static const uint8_t    *s_tx_ptr = NULL;

/* Line Coding (PC sets baud rate etc., we accept but don't use) */
static uint8_t s_line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08}; /* 115200, 1 stop, no parity, 8 bit */

/*==============================================================
 * USB descriptors
 *============================================================*/

/* Device descriptor (18 bytes) */
static const uint8_t s_dev_desc[18] = {
    0x12,                   /* bLength */
    0x01,                   /* bDescriptorType (Device) */
    0x10, 0x01,             /* bcdUSB = 1.10 */
    0x02,                   /* bDeviceClass (CDC) */
    0x00,                   /* bDeviceSubClass */
    0x00,                   /* bDeviceProtocol */
    0x40,                   /* bMaxPacketSize0 = 64 */
    0x83, 0x04,             /* idVendor = 0x0483 (ST) */
    0x57, 0x40,             /* idProduct = 0x4057 (CDC) */
    0x00, 0x02,             /* bcdDevice = 2.00 */
    0x01,                   /* iManufacturer */
    0x02,                   /* iProduct */
    0x03,                   /* iSerialNumber */
    0x01                    /* bNumConfigurations */
};

/* Configuration descriptor (67 bytes, config + interface + CDC functional + endpoints) */
static const uint8_t s_cfg_desc[67] = {
    /* ---- Configuration descriptor (9 bytes) ---- */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
    /* total 67(0x43), 2 interfaces, config value 1, string index 0, bus powered, 100mA */

    /* ---- Interface 0: CDC communication (9 bytes) ---- */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* interface 0, alt 0, 1 endpoint (EP1 interrupt), class=CDC, subclass=ACM, protocol=AT, string */

    /* ---- CDC functional descriptors ---- */
    /* Header (5 bytes) */
    0x05, 0x24, 0x00, 0x10, 0x01,                                /* bcdCDC = 1.10 */
    /* Call Management (5 bytes) */
    0x05, 0x24, 0x01, 0x00, 0x01,                                /* bmCapabilities=0, bDataInterface=1 */
    /* ACM (4 bytes) */
    0x04, 0x24, 0x02, 0x02,                                      /* bmCapabilities=0x02 */
    /* Union (5 bytes) */
    0x05, 0x24, 0x06, 0x00, 0x01,                                /* bMasterInterface=0, bSlaveInterface=1 */

    /* ---- EP1 descriptor: interrupt IN (7 bytes) ---- */
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10,
    /* EP1 IN, interrupt, wMaxPacketSize=8, bInterval=10ms */

    /* ---- Interface 1: CDC data (9 bytes) ---- */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* interface 1, alt 0, 2 endpoints (EP2 OUT + EP3 IN), class=CDC Data, string */

    /* ---- EP2 descriptor: bulk OUT (7 bytes) ---- */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    /* EP2 OUT, bulk, wMaxPacketSize=64, bInterval=0 */

    /* ---- EP3 descriptor: bulk IN (7 bytes) ---- */
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00
    /* EP3 IN, bulk, wMaxPacketSize=64, bInterval=0 */
};

/* String descriptor 0 (Language ID) */
static const uint8_t s_str0[4] = {
    0x04, 0x03, 0x09, 0x04   /* LangID = 0x0409 (English US) */
};

/* String descriptor 1 (manufacturer) - "XR5000" */
static const uint8_t s_str1[14] = {
    0x0E, 0x03,
    'X', 0, 'R', 0, '5', 0, '0', 0, '0', 0, '0', 0
};

/* String descriptor 2 (product) - "XR5000 GB4717 Export" (20 chars x 2 + 2 header = 42 bytes) */
static const uint8_t s_str2[42] = {
    0x2A, 0x03,
    'X', 0, 'R', 0, '5', 0, '0', 0, '0', 0, '0', 0, ' ', 0,
    'G', 0, 'B', 0, '4', 0, '7', 0, '1', 0, '7', 0, ' ', 0,
    'E', 0, 'x', 0, 'p', 0, 'o', 0, 'r', 0, 't', 0
};

/* String descriptor 3 (serial) - "00010" */
static const uint8_t s_str3[12] = {
    0x0C, 0x03,
    '0', 0, '0', 0, '0', 0, '1', 0, '0', 0
};

/*==============================================================
 * PMA read/write helpers
 *============================================================*/

/* Write data to PMA (src -> PMA[buf_offset], len bytes) */
static void PMA_Write(uint16_t buf_offset, const uint8_t *src, uint16_t len)
{
    uint16_t i;
    uint16_t n = (len + 1) / 2;
    for (i = 0; i < n; i++)
    {
        volatile uint16_t *pma = (volatile uint16_t *)(USB_PMA_BASE + 4u * (buf_offset + i));
        uint16_t w;
        if (2 * i + 1 < len)
            w = src[2 * i] | (src[2 * i + 1] << 8);
        else
            w = src[2 * i];
        *pma = w;
    }
}

/* Read data from PMA (PMA[buf_offset] -> dst, len bytes) */
static void PMA_Read(uint8_t *dst, uint16_t buf_offset, uint16_t len)
{
    uint16_t i;
    uint16_t n = (len + 1) / 2;
    for (i = 0; i < n; i++)
    {
        volatile uint16_t *pma = (volatile uint16_t *)(USB_PMA_BASE + 4u * (buf_offset + i));
        uint16_t w = *pma;
        dst[2 * i] = (uint8_t)(w & 0xFF);
        if (2 * i + 1 < len)
            dst[2 * i + 1] = (uint8_t)(w >> 8);
    }
}

/*==============================================================
 * Endpoint status toggle-write helpers (STM32 standard toggle-write)
 *   STM32F103 EPnR STAT/DTOG bits are toggle-write semantics:
 *     write 1 = toggle, write 0 = keep.
 *   CTR bits are read-only: write 1 = keep, write 0 = clear.
 *   type/kind/ea are normal read-write.
 *
 *   Key: to keep a STAT bit unchanged, write 0 (not the current value).
 *        Writing back current value 1 would toggle it!
 *============================================================*/

/* Debug: record received SETUP data */
volatile uint8_t g_dbg_setup[8] = {0};
volatile uint8_t g_dbg_setup_count = 0;
volatile uint8_t g_dbg_bmReqType = 0;
volatile uint8_t g_dbg_bRequest = 0;

/* Debug: EP0 state after init (pyOCD readback) */
volatile uint16_t g_dbg_ep0r_after_init = 0xBEEF;

/* Debug: CPU readback of PMA BTABLE values */
volatile uint16_t g_dbg_pma_addr_tx0 = 0xDEAD;
volatile uint16_t g_dbg_pma_count_tx0 = 0xDEAD;
volatile uint16_t g_dbg_pma_addr_rx0 = 0xDEAD;
volatile uint16_t g_dbg_pma_count_rx0 = 0xDEAD;

/* Debug: PMA read/write test */
volatile uint16_t g_dbg_pma_test_write = 0xBEEF;
volatile uint16_t g_dbg_pma_test_read = 0xDEAD;

/* Debug: interrupt type counters */
volatile uint32_t g_dbg_ctr_count = 0;
volatile uint32_t g_dbg_susp_count = 0;
volatile uint32_t g_dbg_esof_count = 0;
volatile uint32_t g_dbg_reset_isr_count = 0;

/* Debug: ISTR history (last 16 ISR entries) */
volatile uint16_t g_dbg_istr_hist[16] = {0};
volatile uint8_t g_dbg_istr_hist_idx = 0;

/* Debug: RESET handler end state snapshot */
volatile uint16_t g_dbg_ep0r_after_reset = 0xDEAD;
volatile uint16_t g_dbg_cntr_after_reset = 0xDEAD;
volatile uint16_t g_dbg_daddr_after_reset = 0xDEAD;

/* Keep mask: EA + EP_KIND + EP_TYPE (normal read-write bits, keep directly) */
#define EP_KEEP_MASK  (0x070Fu)

static void EP_SetTxStatus(uint8_t ep, uint16_t status)
{
    uint16_t cur = USB_EP(ep);
    uint16_t toggle = (cur ^ status) & EP_T_MASK;
    if (toggle)
    {
        uint16_t v = (cur & EP_KEEP_MASK) | toggle | EP_CTR_TX | EP_CTR_RX;
        USB_EP(ep) = v;
        (void)USB_EP(ep);
    }
}

static void EP_SetRxStatus(uint8_t ep, uint16_t status)
{
    uint16_t cur = USB_EP(ep);
    uint16_t toggle = (cur ^ status) & EP_R_MASK;
    if (toggle)
    {
        uint16_t v = (cur & EP_KEEP_MASK) | toggle | EP_CTR_TX | EP_CTR_RX;
        USB_EP(ep) = v;
        (void)USB_EP(ep);
    }
}

/* Clear CTR_TX (write 0), keep STAT/DTOG (write 0), keep CTR_RX (write 1) */
static void EP_ClearCTR_TX(uint8_t ep)
{
    uint16_t cur = USB_EP(ep);
    uint16_t v = (cur & EP_KEEP_MASK) | EP_CTR_RX;
    USB_EP(ep) = v;
    (void)USB_EP(ep);
}

/* Clear CTR_RX (write 0), keep STAT/DTOG (write 0), keep CTR_TX (write 1) */
static void EP_ClearCTR_RX(uint8_t ep)
{
    uint16_t cur = USB_EP(ep);
    uint16_t v = (cur & EP_KEEP_MASK) | EP_CTR_TX;
    USB_EP(ep) = v;
    (void)USB_EP(ep);
}

static uint16_t EP_GetRxCount(uint8_t ep)
{
    uint16_t v = PMA_COUNT_RX(ep);
    return v & 0x03FF;
}

static void EP_SetTxCount(uint8_t ep, uint16_t count)
{
    PMA_COUNT_TX(ep) = count;
}

/*==============================================================
 * Endpoint init
 *============================================================*/
static void EP_Init(uint8_t ep, uint16_t type, uint16_t tx_status, uint16_t rx_status)
{
    /* Set BTABLE entries for this endpoint */
    switch (ep)
    {
        case 0:
            PMA_ADDR_TX(0) = PMA_EP0_TX_ADDR;
            PMA_ADDR_RX(0) = PMA_EP0_RX_ADDR;
            PMA_COUNT_TX(0) = 0;
            /* EP0 RX: 64 bytes. BL_SIZE=1(32+), NUM_BLOCK=2 (2x32=64) */
            PMA_COUNT_RX(0) = 0x8000 | (2 << 10);
            break;
        case 1:
            PMA_ADDR_TX(1) = PMA_EP1_TX_ADDR;
            PMA_COUNT_TX(1) = 0;
            break;
        case 2:
            PMA_ADDR_RX(2) = PMA_EP2_RX_ADDR;
            PMA_COUNT_RX(2) = 0x8000 | (2 << 10);   /* 64 bytes: BL_SIZE=1, NUM_BLOCK=2 */
            break;
        case 3:
            PMA_ADDR_TX(3) = PMA_EP3_TX_ADDR;
            PMA_COUNT_TX(3) = 0;
            break;
        default:
            break;
    }

    /* Single toggle-write: type/ea normal write, STAT toggle-write, DTOG/CTR keep */
    {
        uint16_t cur = USB_EP(ep);
        uint16_t toggle_tx = (cur ^ tx_status) & EP_T_MASK;
        uint16_t toggle_rx = (cur ^ rx_status) & EP_R_MASK;
        uint16_t v = type | ep | toggle_tx | toggle_rx | EP_CTR_TX | EP_CTR_RX;
        USB_EP(ep) = v;
        (void)USB_EP(ep);
    }
    if (ep == 0)
    {
        g_dbg_ep0r_after_init = USB_EP(0);

        g_dbg_pma_addr_tx0 = PMA_ADDR_TX(0);
        g_dbg_pma_count_tx0 = PMA_COUNT_TX(0);
        g_dbg_pma_addr_rx0 = PMA_ADDR_RX(0);
        g_dbg_pma_count_rx0 = PMA_COUNT_RX(0);

        /* PMA read/write test: write known value to EP0 TX buffer, read back */
        {
            uint8_t test_wr[4] = {0x12, 0x34, 0x56, 0x78};
            uint8_t test_rd[4] = {0};
            PMA_Write(PMA_EP0_TX_ADDR, test_wr, 4);
            PMA_Read(test_rd, PMA_EP0_TX_ADDR, 4);
            g_dbg_pma_test_write = (uint16_t)((test_wr[0] << 8) | test_wr[1]);
            g_dbg_pma_test_read = (uint16_t)((test_rd[0] << 8) | test_rd[1]);
        }
    }
}

/*==============================================================
 * EP0 control transfer handling
 *============================================================*/
static uint8_t s_ep0_buf[64];
static const uint8_t *s_ep0_ptr = NULL;
static uint16_t s_ep0_remaining = 0;

/* EP0 send data (IN direction): send first packet, queue remainder */
static void EP0_Send(const uint8_t *data, uint16_t total_len, uint16_t req_len)
{
    uint16_t send_len = total_len < req_len ? total_len : req_len;
    if (send_len > EP0_PACKET_SIZE) send_len = EP0_PACKET_SIZE;

    PMA_Write(PMA_EP0_TX_ADDR, data, send_len);
    EP_SetTxCount(0, send_len);
    EP_SetTxStatus(0, EP_STAT_TX_VALID);

    s_ep0_remaining = (send_len < req_len) ? (total_len - send_len) : 0;
    s_ep0_ptr = data + send_len;
}

/* SET_ADDRESS deferred: send ZLP first, set DADDR after IN complete */
static void EP0_SetAddress(uint8_t addr)
{
    s_usb_address = addr;
    s_usb_address_pending = 1;
    EP_SetTxCount(0, 0);
    EP_SetTxStatus(0, EP_STAT_TX_VALID);
}

/* EP0 SETUP packet handler */
static void EP0_HandleSetup(void)
{
    uint8_t setup[8];
    PMA_Read(setup, PMA_EP0_RX_ADDR, 8);

    /* Debug: record SETUP data */
    {
        uint8_t i;
        for (i = 0; i < 8; i++) g_dbg_setup[i] = setup[i];
        g_dbg_bmReqType = setup[0];
        g_dbg_bRequest = setup[1];
        g_dbg_setup_count++;
    }

    uint8_t  bmReqType = setup[0];
    uint8_t  bRequest   = setup[1];
    uint16_t wValue     = setup[2] | (setup[3] << 8);
    uint16_t wIndex     = setup[4] | (setup[5] << 8);
    uint16_t wLength    = setup[6] | (setup[7] << 8);
    (void)wIndex;

    /* Standard device requests */
    if ((bmReqType & 0x60) == 0x00)
    {
        switch (bRequest)
        {
            case 0x00: /* GET_STATUS */
                s_ep0_buf[0] = 0; s_ep0_buf[1] = 0;
                EP0_Send(s_ep0_buf, 2, wLength);
                return;
            case 0x01: /* CLEAR_FEATURE */
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            case 0x03: /* SET_FEATURE */
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            case 0x05: /* SET_ADDRESS */
                EP0_SetAddress((uint8_t)wValue);
                return;
            case 0x06: /* GET_DESCRIPTOR */
            {
                uint8_t desc_type = (wValue >> 8) & 0xFF;
                uint8_t desc_idx  = wValue & 0xFF;
                switch (desc_type)
                {
                    case 0x01: /* Device */
                        EP0_Send(s_dev_desc, 18, wLength);
                        return;
                    case 0x02: /* Configuration */
                        EP0_Send(s_cfg_desc, 67, wLength);
                        return;
                    case 0x03: /* String */
                        switch (desc_idx)
                        {
                            case 0: EP0_Send(s_str0, 4, wLength); return;
                            case 1: EP0_Send(s_str1, 14, wLength); return;
                            case 2: EP0_Send(s_str2, 42, wLength); return;
                            case 3: EP0_Send(s_str3, 12, wLength); return;
                            default: break;
                        }
                        break;
                    default:
                        break;
                }
                break;
            }
            case 0x08: /* GET_CONFIGURATION */
                s_ep0_buf[0] = s_usb_configured ? 1 : 0;
                EP0_Send(s_ep0_buf, 1, wLength);
                return;
            case 0x09: /* SET_CONFIGURATION */
                s_usb_configured = (wValue != 0) ? 1 : 0;
                if (s_usb_configured)
                {
                    EP_SetTxStatus(1, EP_STAT_TX_NAK);
                    EP_SetRxStatus(2, EP_STAT_RX_VALID);
                    EP_SetTxStatus(3, EP_STAT_TX_NAK);
                }
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            case 0x0B: /* SET_INTERFACE */
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            default:
                break;
        }
    }
    /* CDC class requests (bmReqType = 0x21 host-to-device class) */
    else if ((bmReqType & 0x7F) == 0x21)
    {
        switch (bRequest)
        {
            case 0x20: /* SET_LINE_CODING (OUT data stage) */
                EP_SetRxStatus(0, EP_STAT_RX_VALID);
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            case 0x21: /* GET_LINE_CODING (IN data stage) */
                EP0_Send(s_line_coding, 7, wLength);
                return;
            case 0x22: /* SET_CONTROL_LINE_STATE */
                s_usb_configured = (wValue & 0x01) ? 1 : s_usb_configured;
                EP_SetTxCount(0, 0);
                EP_SetTxStatus(0, EP_STAT_TX_VALID);
                return;
            default:
                break;
        }
    }

    /* 未处理的请求: STALL EP0 TX方向
     * USB规范要求EP0在STALL后仍能接收新SETUP包, 因此RX保持VALID */
    EP_SetTxStatus(0, EP_STAT_TX_STALL);
    EP_SetRxStatus(0, EP_STAT_RX_VALID);
}

/* EP0 CTR handler */
static void EP0_HandleCTR(void)
{
    uint16_t ep0 = USB_EP(0);

    if (ep0 & EP_SETUP)   /* SETUP received */
    {
        EP_ClearCTR_RX(0);
        EP0_HandleSetup();
    }
    else if (ep0 & EP_CTR_TX)   /* IN transfer complete */
    {
        EP_ClearCTR_TX(0);
        if (s_ep0_remaining > 0)
        {
            uint16_t send_len = s_ep0_remaining > EP0_PACKET_SIZE ? EP0_PACKET_SIZE : s_ep0_remaining;
            PMA_Write(PMA_EP0_TX_ADDR, s_ep0_ptr, send_len);
            EP_SetTxCount(0, send_len);
            EP_SetTxStatus(0, EP_STAT_TX_VALID);
            s_ep0_ptr += send_len;
            s_ep0_remaining -= send_len;
        }
        else
        {
            EP_SetRxStatus(0, EP_STAT_RX_VALID);
            if (s_usb_address_pending)
            {
                USB_DADDR = (uint16_t)(s_usb_address | 0x80);
                s_usb_address_pending = 0;
            }
        }
    }
    else if (ep0 & EP_CTR_RX)   /* OUT transfer complete */
    {
        EP_ClearCTR_RX(0);
        /* SET_LINE_CODING data stage, read 7 bytes */
        {
            uint16_t n = EP_GetRxCount(0);
            if (n >= 7)
            {
                PMA_Read(s_line_coding, PMA_EP0_RX_ADDR, 7);
            }
        }
        EP_SetRxStatus(0, EP_STAT_RX_VALID);
    }
}

/*==============================================================
 * EP2 bulk OUT handler (PC -> device)
 *============================================================*/
static void EP2_HandleCTR(void)
{
    uint16_t n = EP_GetRxCount(2);
    uint16_t i;
    uint8_t  buf[EP2_PACKET_SIZE];

    EP_ClearCTR_RX(2);

    if (n > EP2_PACKET_SIZE) n = EP2_PACKET_SIZE;
    PMA_Read(buf, PMA_EP2_RX_ADDR, n);

    for (i = 0; i < n; i++)
    {
        if (s_rx_count < USB_CDC_RX_BUF_SIZE)
        {
            s_rx_buf[s_rx_head] = buf[i];
            s_rx_head = (s_rx_head + 1) % USB_CDC_RX_BUF_SIZE;
            s_rx_count++;
        }
    }

    EP_SetRxStatus(2, EP_STAT_RX_VALID);
}

/*==============================================================
 * EP3 bulk IN handler (device -> PC)
 *============================================================*/
static void EP3_HandleCTR(void)
{
    EP_ClearCTR_TX(3);

    s_tx_busy = 0;
    if (s_tx_remaining > 0)
    {
        uint16_t n = s_tx_remaining > EP3_PACKET_SIZE ? EP3_PACKET_SIZE : s_tx_remaining;
        PMA_Write(PMA_EP3_TX_ADDR, s_tx_ptr, n);
        EP_SetTxCount(3, n);
        s_tx_ptr += n;
        s_tx_remaining -= n;
        s_tx_busy = 1;
        EP_SetTxStatus(3, EP_STAT_TX_VALID);
    }
    else
    {
        EP_SetTxStatus(3, EP_STAT_TX_NAK);
    }
}

/*==============================================================
 * EP1 handler (interrupt IN, defensive)
 *============================================================*/
static void EP1_HandleCTR(void)
{
    uint16_t ep1 = USB_EP(1);
    if (ep1 & EP_CTR_TX)
    {
        EP_ClearCTR_TX(1);
        EP_SetTxStatus(1, EP_STAT_TX_NAK);
    }
    if (ep1 & EP_CTR_RX)
    {
        EP_ClearCTR_RX(1);
    }
}

/*==============================================================
 * USB interrupt service routine
 *============================================================*/
volatile uint32_t g_usb_isr_count = 0;
volatile uint32_t g_usb_reset_count = 0;

void USB_CDC_ISR(void)
{
    uint16_t istr = USB_ISTR;
    uint16_t ep;

    g_usb_isr_count++;

    g_dbg_istr_hist[g_dbg_istr_hist_idx] = istr;
    g_dbg_istr_hist_idx = (g_dbg_istr_hist_idx + 1) & 0x0F;

    if (istr & USB_ISTR_CTR)   g_dbg_ctr_count++;
    if (istr & USB_ISTR_SUSP)  g_dbg_susp_count++;
    if (istr & USB_ISTR_ESOF)  g_dbg_esof_count++;

    /* RESET interrupt: USB bus reset, reconfigure endpoints.
     * This is the FIRST interrupt that fires after host detects pull-up.
     * EPs and DADDR.EF are configured HERE (not in USB_CDC_Init) so the
     * device is fully ready before responding to any host traffic. */
    if (istr & USB_ISTR_RESET)
    {
        g_usb_reset_count++;
        g_dbg_reset_isr_count++;

        /* Clear RESET flag (write 0 to clear, write 1 to keep others) */
        USB_ISTR = (uint16_t)~USB_ISTR_RESET;

        /* Reset state variables */
        s_usb_configured = 0;
        s_usb_address = 0;
        s_usb_address_pending = 0;
        s_rx_head = s_rx_tail = s_rx_count = 0;
        s_tx_busy = 0;
        s_tx_remaining = 0;
        s_tx_len = 0;
        s_ep0_remaining = 0;

        /* Disable function during configuration */
        USB_DADDR = 0;
        USB_BTABLE = PMA_BTABLE;

        /* Configure EP0 (control, TX=NAK, RX=VALID to accept SETUP) */
        EP_Init(0, EP_TYPE_CONTROL, EP_STAT_TX_NAK, EP_STAT_RX_VALID);
        /* EP1/EP2/EP3 disabled, activate on SET_CONFIGURATION */
        EP_Init(1, EP_TYPE_INTERRUPT, EP_STAT_TX_DIS, EP_STAT_RX_DIS);
        EP_Init(2, EP_TYPE_BULK, EP_STAT_TX_DIS, EP_STAT_RX_DIS);
        EP_Init(3, EP_TYPE_BULK, EP_STAT_TX_DIS, EP_STAT_RX_DIS);

        /* Enable CTR + RESET + SUSP + ERR interrupts (endpoints are now ready) */
        USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;

        /* Enable device function (EF=1, address=0) - endpoints ready, can respond to SETUP.
         * Do NOT clear ISTR here: a SETUP may have arrived during EP configuration,
         * and its CTR flag must remain set so the next ISR entry processes it. */
        USB_DADDR = 0x80;

        g_dbg_ep0r_after_reset = USB_EP(0);
        g_dbg_cntr_after_reset = USB_CNTR;
        g_dbg_daddr_after_reset = USB_DADDR;

        return;
    }

    /* CTR (correct transfer) interrupt: process endpoint transfer */
    if (istr & USB_ISTR_CTR)
    {
        ep = istr & USB_ISTR_EP_ID;

        switch (ep)
        {
            case 0: EP0_HandleCTR(); break;
            case 1: EP1_HandleCTR(); break;
            case 2: EP2_HandleCTR(); break;
            case 3: EP3_HandleCTR(); break;
            default: break;
        }
        USB_ISTR = (uint16_t)~USB_ISTR_CTR;
    }

    /* Other interrupts (suspend/wakeup/SOF/ESOF/error/PMA overrun): just clear */
    if (istr & (USB_ISTR_SUSP | USB_ISTR_WKUP | USB_ISTR_SOF | USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR))
    {
        USB_ISTR = ~(USB_ISTR_SUSP | USB_ISTR_WKUP | USB_ISTR_SOF | USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR);
    }
}

/*==============================================================
 * Public API
 *============================================================*/

void USB_CDC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* USB clock: PLL / 1.5 = 72MHz / 1.5 = 48MHz (full-speed USB clock) */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

    /* FRES pulse to reset USB peripheral. Do NOT use PDWN (breaks PHY on
     * some F103 clones). Pull-up stays active (PDWN=0 default). */
    USB_CNTR = USB_CNTR_FRES;
    USB_BTABLE = PMA_BTABLE;
    USB_DADDR = 0;
    USB_ISTR = 0;
    USB_CNTR = USB_CNTR_RESETM;     /* FRES=0, enable RESET interrupt */

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USB_CDC_SendData(const uint8_t *data, uint16_t len)
{
    if (len == 0) return;

    while (s_tx_busy || s_tx_remaining > 0)
    {
    }

    __disable_irq();
    s_tx_ptr = data;
    s_tx_remaining = len;

    if (!s_tx_busy && s_usb_configured)
    {
        uint16_t n = s_tx_remaining > EP3_PACKET_SIZE ? EP3_PACKET_SIZE : s_tx_remaining;
        PMA_Write(PMA_EP3_TX_ADDR, s_tx_ptr, n);
        EP_SetTxCount(3, n);
        s_tx_ptr += n;
        s_tx_remaining -= n;
        s_tx_busy = 1;
        EP_SetTxStatus(3, EP_STAT_TX_VALID);
    }
    __enable_irq();

    while (s_tx_busy || s_tx_remaining > 0)
    {
    }
}

uint16_t USB_CDC_Available(void)
{
    return s_rx_count;
}

uint16_t USB_CDC_ReadByte(void)
{
    uint16_t ret;
    if (s_rx_count == 0) return 0xFFFF;

    __disable_irq();
    ret = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % USB_CDC_RX_BUF_SIZE;
    s_rx_count--;
    __enable_irq();

    return ret;
}

void USB_CDC_Poll(void)
{
    /* 防御: 确保USB中断屏蔽位正确 (CTRM|RESETM|SUSPM|ERRM)
     * 某些未知条件可能清除CTRM, 导致SETUP包到达时无法触发CTR中断
     * 每次Poll检查并在CTRM被清除时恢复正确的中断屏蔽配置 */
    if ((USB_CNTR & USB_CNTR_CTRM) == 0)
    {
        USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    }

    /* TX pump: send pending data on EP3 */
    if (s_usb_configured && !s_tx_busy && s_tx_remaining > 0)
    {
        uint16_t n = s_tx_remaining > EP3_PACKET_SIZE ? EP3_PACKET_SIZE : s_tx_remaining;
        PMA_Write(PMA_EP3_TX_ADDR, s_tx_ptr, n);
        EP_SetTxCount(3, n);
        s_tx_ptr += n;
        s_tx_remaining -= n;
        s_tx_busy = 1;
        EP_SetTxStatus(3, EP_STAT_TX_VALID);
    }

    /* 自动重新枚举: 仅在从未收到USB RESET时触发
     * (主机完全未检测到设备D+上拉). 一旦收到RESET, 主机已知设备存在,
     * 后续SETUP/枚举由主机主导, 不再强制断开重连, 避免干扰枚举过程 */
    if (g_usb_reset_count == 0)
    {
        static uint32_t s_no_reset_timer = 0;
        if (++s_no_reset_timer > 5000000u)
        {
            volatile uint32_t j;
            uint32_t crh_save;
            s_no_reset_timer = 0;

            /* 断开: GPIO驱动PA12(D+)拉低, 覆盖内部上拉 */
            crh_save = GPIOA->CRH;
            GPIOA->CRH = (crh_save & 0xFFF0FFFF) | 0x00010000; /* PA12推挽输出10MHz */
            GPIOA->BRR = (1 << 12);  /* PA12=0 (D+低=断开) */
            for (j = 0; j < 50000u; j++);  /* ~700us断开 */

            /* 重连: 恢复PA12为输入(内部上拉激活) */
            GPIOA->BSRR = (1 << 12);  /* PA12=1(预充电) */
            GPIOA->CRH = crh_save;    /* 恢复原始配置 */
            for (j = 0; j < 50000u; j++);  /* ~700us稳定 */
        }
    }
}

uint8_t USB_CDC_IsConfigured(void)
{
    return s_usb_configured;
}
