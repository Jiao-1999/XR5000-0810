#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_istr.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"

/*==============================================================
 * 移植说明 (2026-08-05):
 *   - 移除了 USB_LP_CAN1_RX0_IRQHandler 和 USBWakeUp_IRQHandler,
 *     改由 stm32f10x_it.c 统一注册, 避免符号冲突.
 *   - 移除了所有 printf 调用, 避免 USB 库调试输出污染存储侧
 *     USART1(PA9/PA10) 与存储侧板的通信.
 *   - USB_To_USART_Send_Data 仍保留 \r\n 分帧逻辑供 usb_printf 使用,
 *     但实际接收路径已改由 usb_endp.c 的 EP3_OUT_Callback 直接调用
 *     USB_CDC_PushRx 写入字节流环形缓冲(不再经过本函数).
 *============================================================*/

_usb_usart_fifo uu_txfifo;                  //USB串口发送FIFO结构体
u8  USART_PRINTF_Buffer[USB_USART_REC_LEN]; //usb_printf发送缓冲区

/* 保留旧变量声明以兼容 usb_prop.c 等引用, 但实际接收路径不使用 */
u8  USB_USART_RX_BUF[USB_USART_REC_LEN];
u16 USB_USART_RX_STA = 0;

extern LINE_CODING linecoding;              //USB虚拟串口配置信息

/*==============================================================
 * USB 通用部分代码 (来自 ST 官方例程, 已去除 printf)
 *============================================================*/

/* USB 时钟配置: USBclk = 48MHz @ HCLK=72MHz */
void Set_USBClock(void)
{
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);    //USBclk = PLLclk/1.5 = 48MHz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE); //USB时钟使能
}

/* USB 进入低功耗模式 */
void Enter_LowPowerMode(void)
{
    bDeviceState = SUSPENDED;
}

/* USB 退出低功耗模式 */
void Leave_LowPowerMode(void)
{
    DEVICE_INFO *pInfo = &Device_Info;
    if (pInfo->Current_Configuration != 0) bDeviceState = CONFIGURED;
    else bDeviceState = ATTACHED;
}

/* USB 中断配置 (低优先级 USB IRQ + EXTI Line18 唤醒) */
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClearITPendingBit(EXTI_Line18);
    EXTI_InitStructure.EXTI_Line = EXTI_Line18;                  //USB resume from suspend
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable the USB interrupt (低优先级, 抢占1 子优先级0) */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt (抢占0, 最高优先级) */
    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

/* USB 接口上拉配置 (战舰V3硬件恒上拉, 此函数为空实现) */
void USB_Cable_Config(FunctionalState NewState)
{
    (void)NewState;
}

/* USB 使能连接/断线 (通过控制 CNTR PD 位 + GPIO 拉低 D+ 模拟拔插) */
void USB_Port_Set(u8 enable)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能PORTA时钟
    if (enable) _SetCNTR(_GetCNTR() & (~(1 << 1)));       //退出断电模式
    else
    {
        _SetCNTR(_GetCNTR() | (1 << 1));                  //断电模式
        GPIOA->CRH &= 0XFFF00FFF;
        GPIOA->CRH |= 0X00033000;
        PAout(12) = 0;
    }
}

/* 获取 STM32 唯一 ID 用于 USB 序列号 */
void Get_SerialNum(void)
{
    u32 Device_Serial0, Device_Serial1, Device_Serial2;
    Device_Serial0 = *(u32*)(0x1FFFF7E8);
    Device_Serial1 = *(u32*)(0x1FFFF7EC);
    Device_Serial2 = *(u32*)(0x1FFFF7F0);
    Device_Serial0 += Device_Serial2;
    if (Device_Serial0 != 0)
    {
        IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2],  8);
        IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
    }
}

/* 将 32 位值转换为 unicode 字符串 */
void IntToUnicode(u32 value, u8 *pbuf, u8 len)
{
    u8 idx = 0;
    for (idx = 0; idx < len; idx++)
    {
        if (((value >> 28)) < 0xA)
        {
            pbuf[2 * idx] = (value >> 28) + '0';
        }
        else
        {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }
        value = value << 4;
        pbuf[2 * idx + 1] = 0;
    }
}

/*==============================================================
 * CDC 类应用层回调
 *============================================================*/

/* USB COM 口配置回调 (USB 协议栈在 SET_LINE_CODING 后调用) */
bool USART_Config(void)
{
    uu_txfifo.readptr = 0;  //清空读指针
    uu_txfifo.writeptr = 0; //清空写指针
    USB_USART_RX_STA = 0;   //USB USART接收状态清零
    return (TRUE);
}

/* 处理从 USB 虚拟串口接收到的数据 (\r\n 分帧, 仅供 usb_printf 路径使用)
 * 注意: 实际接收已改由 usb_endp.c 的 EP3_OUT_Callback 直接调用
 *       USB_CDC_PushRx 写入字节流缓冲, 本函数不再被调用. */
void USB_To_USART_Send_Data(u8* data_buffer, u8 Nb_bytes)
{
    u8 i;
    u8 res;
    for (i = 0; i < Nb_bytes; i++)
    {
        res = data_buffer[i];
        if ((USB_USART_RX_STA & 0x8000) == 0)        //接收未完成
        {
            if (USB_USART_RX_STA & 0x4000)            //接收到了0x0d
            {
                if (res != 0x0a) USB_USART_RX_STA = 0; //接收错误, 重新开始
                else USB_USART_RX_STA |= 0x8000;       //接收完成了
            }
            else                                      //还没收到0X0D
            {
                if (res == 0x0d) USB_USART_RX_STA |= 0x4000;
                else
                {
                    USB_USART_RX_BUF[USB_USART_RX_STA & 0X3FFF] = res;
                    USB_USART_RX_STA++;
                    if (USB_USART_RX_STA > (USB_USART_REC_LEN - 1)) USB_USART_RX_STA = 0;
                }
            }
        }
    }
}

/* 发送一个字节数据到 USB 虚拟串口 (写入 TX FIFO, 由 SOF 中断发出) */
void USB_USART_SendData(u8 data)
{
    uu_txfifo.buffer[uu_txfifo.writeptr] = data;
    uu_txfifo.writeptr++;
    if (uu_txfifo.writeptr == USB_USART_TXFIFO_SIZE) //超过buf大小了, 归零
    {
        uu_txfifo.writeptr = 0;
    }
}

/* usb 虚拟串口 printf 函数 (确保一次发送数据不超 USB_USART_REC_LEN 字节) */
void usb_printf(char* fmt, ...)
{
    u16 i, j;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char*)USART_PRINTF_Buffer, fmt, ap);
    va_end(ap);
    i = strlen((const char*)USART_PRINTF_Buffer); //此次发送数据的长度
    for (j = 0; j < i; j++)
    {
        USB_USART_SendData(USART_PRINTF_Buffer[j]);
    }
}
