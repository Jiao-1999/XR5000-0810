/**
 * @file    usb_cdc.h
 * @brief   STM32F103 USB CDC 虚拟串口驱动
 * @details 使用 USB 全速设备外设(CDC) ACM 类, PC 端识别为串口(COM 口).
 *          负责 GB4717 数据导出, 通过 USB 口与 PC 通信(PA11/PA12).
 *
 *   硬件接口:
 *     PA11 = USB D- (USBDM)
 *     PA12 = USB D+ (USBDP)
 *     PA11/PA12 是 USB 专用引脚, 连接时需要让出 GPIO 控制权
 *
 *   端点布局:
 *     EP0 : 控制端点 (枚举, SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE)
 *     EP1 : 中断 IN  (CDC 通知, 暂不主动使用)
 *     EP2 : 批量 OUT (PC -> 设备, 接收 GB4717 命令)
 *     EP3 : 批量 IN  (设备 -> PC, 发送 GB4717 响应)
 *
 *   注意:
 *     - STM32F103 USB 全速(12Mbps), CDC 类设备由主机自动枚举
 *     - PC 端用 pyserial 打开 COM 口, 波特率可任意设置(忽略 SET_LINE_CODING)
 *     - 数据收发对接 soft_uart.h 模块, 与主串口协议(GB4717)数据透传
 */
#ifndef __USB_CDC_H
#define __USB_CDC_H

#include "sys.h"

/*==============================================================
 * 缓冲区
 *============================================================*/
#define USB_CDC_RX_BUF_SIZE     128     /* 接收环形缓冲区大小(字节) */
#define USB_CDC_TX_BUF_SIZE     64      /* 发送 EP3 IN 缓冲大小(字节) */

/*==============================================================
 * API 函数接口
 *============================================================*/

/**
 * @brief  初始化 USB CDC 虚拟串口
 * @note   内部执行 USB 复位(FRES), 配置 PA11/PA12 和 PMA 缓冲, 使能 USB 中断
 *         成功后 PC 端会枚举出新的 COM 口
 */
void USB_CDC_Init(void);

/**
 * @brief  发送数据到 PC (非阻塞, 等待上一包完成)
 * @param  data: 数据指针
 * @param  len:  数据长度
 * @note   内部循环等待发送完成, 通过 USB_CDC_Poll() 或中断驱动完成剩余 EP3 IN 传输.
 *         若上一包未发送完, 阻塞等待直到可发送.
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
 * @brief  USB 中断服务函数 (由 stm32f10x_it.c 的 USB_LP_CAN1_RX0_IRQHandler 调用)
 * @note   处理 USB 复位/挂起/唤醒/正确传输(CTR) 等中断
 */
void USB_CDC_ISR(void);

/**
 * @brief  USB CDC 主循环处理 (在 main while(1) 中调用)
 * @note   持续推送待发送数据到 EP3 IN 端点(完成剩余传输), 并处理自动重新枚举
 */
void USB_CDC_Poll(void);

/**
 * @brief  查询 USB 是否已配置完成
 * @retval 1 = 已配置(PC 端已打开 COM 口), 0 = 未配置
 */
uint8_t USB_CDC_IsConfigured(void);

#endif /* __USB_CDC_H */
