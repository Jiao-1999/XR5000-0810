/**
 * @file    usb_cdc.c
 * @brief   USB CDC 虚拟串口封装层实现 (基于 ST 官方 USB-FS-Device_Driver)
 * @details 本文件封装 ST 官方 USB-FS-Device_Driver, 对外提供与旧版 usb_cdc
 *          完全一致的接口. 接收路径改用字节流环形缓冲区(不再依赖\r\n分帧),
 *          以适配存储侧项目的 GB4717 协议透传需求.
 */
#include "usb_cdc.h"
#include "delay.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_istr.h"   /* USB_Istr() 声明 */

/*==============================================================
 * 外部变量 (供 main.c 调试 LED 指示 USB ISR 活动)
 *============================================================*/
volatile uint32_t g_usb_isr_count   = 0;
volatile uint32_t g_usb_reset_count = 0;

/*==============================================================
 * 接收环形缓冲区 (字节流, 不分帧)
 *============================================================*/
static volatile uint8_t  s_rx_buf[USB_CDC_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;   /* 写入位置 */
static volatile uint16_t s_rx_tail = 0;   /* 读取位置 */
static volatile uint16_t s_rx_count = 0;  /* 当前字节数 */

/*==============================================================
 * 内部函数声明
 *============================================================*/
void RESET_Callback(void);  /* 供 usb_istr.c 通过 RESET_CALLBACK 宏调用 */

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

    /* 启动 USB 协议栈 (进入枚举流程) */
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

void USB_CDC_ISR(void)
{
    g_usb_isr_count++;
    USB_Istr();
}

void USB_CDC_Poll(void)
{
    /* 收发均由 USB 中断异步驱动, 此处无需处理 */
}

uint8_t USB_CDC_IsConfigured(void)
{
    return (bDeviceState == CONFIGURED) ? 1 : 0;
}

void USB_CDC_PushRx(uint8_t b)
{
    if (s_rx_count >= USB_CDC_RX_BUF_SIZE)
    {
        /* 缓冲区满, 丢弃最新字节 */
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
 * 用于统计 USB 复位次数, 方便调试
 *============================================================*/
void RESET_Callback(void)
{
    g_usb_reset_count++;
}
