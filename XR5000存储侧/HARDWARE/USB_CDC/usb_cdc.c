/**
 * @file    usb_cdc.c
 * @brief   USB CDC 虚拟串口封装层实现 (基于 ST 官方 USB-FS-Device_Driver)
 * @details 本文件封装 ST 官方 USB-FS-Device_Driver, 对外提供与旧版 usb_cdc
 *          完全一致的接口. 环形接收区用字节流低位循环(防止帧错位\r\n拆帧),
 *          透传给主机侧目标为 GB4717 协议透传处理.
 */
#include "usb_cdc.h"
#include "delay.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_istr.h" /* USB_Istr() 声明 */

/*==============================================================
 * 外部变量 (供 main.c 调试 LED 指示 USB ISR 活动)
 *============================================================*/
volatile uint32_t g_usb_isr_count   = 0;
volatile uint32_t g_usb_reset_count = 0;

/*==============================================================
 * 接收环形缓冲 (字节流型, 存放帧)
 *============================================================*/
static volatile uint8_t  s_rx_buf[USB_CDC_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0; /* 写入位置 */
static volatile uint16_t s_rx_tail = 0; /* 读取位置 */
static volatile uint16_t s_rx_count = 0; /* 当前字节数 */

/*==============================================================
 * 内部函数声明
 *============================================================*/
void RESET_Callback(void); /* 供 usb_istr.c 通过 RESET_CALLBACK 宏调用 */

/*==============================================================
 * API 实现
 *============================================================*/

void USB_CDC_Init(void)
{
    /* USB 软断开 -> 延时 -> 软重连, 模拟拔插, 触发 PC 重新枚举 */
    USB_Port_Set(0);
    delay_ms(700);
    USB_Port_Set(1);

    /* 配置 USB 48MHz 时钟 (PLL/1.5 = 48MHz) */
    Set_USBClock();

    /* 配置 USB 低优先级中断 + 唤醒中断 (EXTI Line18) */
    USB_Interrupts_Config();

    /* 初始化接收缓冲区 */
    s_rx_head  = 0;
    s_rx_tail  = 0;
    s_rx_count = 0;

    /* 初始化 USB 协议栈 (含枚举处理) */
    USB_Init();
}

void USB_CDC_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        USB_USART_SendData(data[i]);
    }
}

uint16_t USB_CDC_Available(void)
{
    return s_rx_count;
}

uint16_t USB_CDC_ReadByte(void)
{
    uint8_t b;
    if (s_rx_count == 0)
    {
        return 0xFFFF;
    }
    b = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1) % USB_CDC_RX_BUF_SIZE);
    __disable_irq();
    s_rx_count--;
    __enable_irq();
    return (uint16_t)b;
}

/* P1-6: peek API: GB4717 (0x40 frames) vs character command (R/C) byte stream splitting */
uint16_t USB_CDC_PeekByte(void)
{
    if (s_rx_count == 0)
    {
        return 0xFFFF;
    }
    return (uint16_t)s_rx_buf[s_rx_tail];
}

void USB_CDC_ISR(void)
{
    g_usb_isr_count++;
    USB_Istr();
}

void USB_CDC_Poll(void)
{
    /* 接收在 USB 中断异步回调, 此处无需处理 */
}

uint8_t USB_CDC_IsConfigured(void)
{
    return (bDeviceState == CONFIGURED) ? 1 : 0;
}

void USB_CDC_PushRx(uint8_t b)
{
    if (s_rx_count >= USB_CDC_RX_BUF_SIZE)
    {
        /* 缓冲已满, 丢弃当前字节 */
        return;
    }
    s_rx_buf[s_rx_head] = b;
    s_rx_head = (uint16_t)((s_rx_head + 1) % USB_CDC_RX_BUF_SIZE);
    __disable_irq();
    s_rx_count++;
    __enable_irq();
}

/*==============================================================
 * USB RESET 回调 (由 usb_istr.c 通过 RESET_CALLBACK 宏调用)
 * 累计统计 USB 复位次数, 便于调试监控
 *============================================================*/
void RESET_Callback(void)
{
    g_usb_reset_count++;
}
