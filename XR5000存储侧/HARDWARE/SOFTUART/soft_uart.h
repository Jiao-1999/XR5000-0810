/**
 * @file    soft_uart.h
 * @brief   软件串口模块 - PA12(TX)/PA11(RX), 9600 8N1
 * @details 使用TIM3 3倍过采样实现全双工软件串口, 提供额外的串口通信通道.
 *          PA11/PA12与USB D-/D+复用, 当USB CDC未启用时可作为备用串口使用.
 *          PA12(TX)推挽输出, PA11(RX)上拉输入
 *          TIM3中断频率 = 3 * 9600 = 28800Hz
 *          帧格式: 起始位(0) + 8数据位(LSB先) + 停止位(1), 无校验位
 */
#ifndef __SOFT_UART_H
#define __SOFT_UART_H

#include "sys.h"

#define SOFT_UART_BAUD      9600
#define SOFT_UART_RX_BUF_SIZE  64   /* 接收环形缓冲大小 */

/**
 * @brief  初始化软件串口 (配置GPIO/TIM3/NVIC并启动定时器)
 */
void SoftUART_Init(void);

/**
 * @brief  发送一个字节 (阻塞等待发送完成)
 * @param  data: 待发送字节
 */
void SoftUART_SendByte(uint8_t data);

/**
 * @brief  发送多字节
 * @param  data: 数据指针
 * @param  len:  字节数
 */
void SoftUART_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  查询接收缓冲中可用字节数
 * @retval 可读字节数
 */
uint16_t SoftUART_Available(void);

/**
 * @brief  读取一个字节 (无数据返回0xFFFF)
 * @retval 读取到的字节; 0xFFFF表示缓冲为空
 */
uint16_t SoftUART_ReadByte(void);

/**
 * @brief  TIM3中断服务函数 (由stm32f10x_it.c的TIM3_IRQHandler调用)
 * @note   每次中断同时处理TX发送移位和RX接收采样
 */
void SoftUART_TIM3_ISR(void);

#endif /* __SOFT_UART_H */
