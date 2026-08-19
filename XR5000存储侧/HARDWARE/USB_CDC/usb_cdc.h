/**
 * @file    usb_cdc.h
 * @brief   USB CDC 虚拟串口封装层 (基于 ST 官方 USB-FS-Device_Driver)
 * @details 本文件对外提供与旧版 usb_cdc 完全一致的接口, 内部改用 ST 官方
 *          USB-FS-Device_Driver + CDC 类实现, 替换之前手写的 USB 协议栈.
 *          硬件连接:
 *            PA11 = USB D- (USBDM)
 *            PA12 = USB D+ (USBDP)
 *          数据流:
 *            接收(PC->MCU): EP3 OUT 中断 -> 字节流环形缓冲 -> USB_CDC_ReadByte
 *            发送(MCU->PC): USB_CDC_SendData -> TX FIFO -> SOF 中断周期触发 EP1 IN
 *          注意: 本封装不占用 USART1, 不影响存储侧板通信.
 */
#ifndef __USB_CDC_H
#define __USB_CDC_H

#include "sys.h"

/*==============================================================
 * 配置
 *============================================================*/
#define USB_CDC_RX_BUF_SIZE     256     /* 接收环形缓冲区大小(字节) */
#define USB_CDC_TX_FIFO_SIZE    1024    /* 发送 FIFO 大小(字节, 与 hw_config.h 一致) */

/*==============================================================
 * API (与旧版接口完全一致, main.c 无需修改)
 *============================================================*/

/**
 * @brief  初始化 USB CDC 虚拟串口
 * @note   内部执行: USB 软断开/重连 -> 设置 USB 时钟 -> 配置 USB 中断 -> USB_Init
 *         成功后 PC 端会枚举出新的 COM 口
 */
void USB_CDC_Init(void);

/**
 * @brief  发送数据到 PC (非阻塞, 写入 FIFO 后由 SOF 中断自动发出)
 * @param  data: 数据指针
 * @param  len:  数据长度
 */
void USB_CDC_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  查询接收缓冲区中可读字节数
 * @retval 可读字节数
 */
uint16_t USB_CDC_Available(void);

/**
 * @brief  从接收缓冲区读取一个字节
 * @retval 0x00-0xFF = 读到的字节, 0xFFFF = 缓冲区为空
 */
uint16_t USB_CDC_ReadByte(void);

/**
 * @brief  USB 低优先级中断服务函数 (由 stm32f10x_it.c 的 USB_LP_CAN1_RX0_IRQHandler 调用)
 * @note   内部调用 ST 官方 USB_Istr(), 并更新 g_usb_isr_count 用于调试
 */
void USB_CDC_ISR(void);

/**
 * @brief  USB CDC 主循环处理 (在 main while(1) 中调用)
 * @note   当前实现为空, 收发均由 USB 中断异步驱动, 此函数保留以兼容旧接口
 */
void USB_CDC_Poll(void);

/**
 * @brief  查询 USB 是否已配置完成(PC 端已打开 COM 口会触发枚举完成)
 * @retval 1 = 已配置(CONFIGURED), 0 = 未配置
 */
uint8_t USB_CDC_IsConfigured(void);

/**
 * @brief  将一个字节压入接收环形缓冲区 (供 usb_endp.c 的 EP3_OUT_Callback 调用)
 * @param  b: 待压入的字节
 * @note   缓冲区满时丢弃最新字节
 */
void USB_CDC_PushRx(uint8_t b);

#endif /* __USB_CDC_H */
