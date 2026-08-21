/*==============================================================
 * 文件名    : bsp_fecbus_rx.h
 * 功能描述  : FECbus 接收解析协议层
 * 适用平台  : STM32H723ZGT6, Keil MDK-ARM
 * 实现方式  : 基于 USART3 逐字节中断接收
 *              - 中断把字节写入环形缓冲；
 *              - 帧解析状态机：0x7E 帧头 + CRC16 校验 + 功能码分发；
 *              - 业务功能按 switch 分发预留 TODO 接口；
 *              - 0FH 状态应答帧完成协议层闭环。
 * 适用范围  : 仅协议层框架，不涉及具体业务上报/写入。
 *==============================================================*/
#ifndef __BSP_FECBUS_RX_H
#define __BSP_FECBUS_RX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 协议层帧参数 */
#define FECBUS_RX_MAX_DLC   8   /* GB4717: DLC 范围 1~8 */

/* 接收帧结构（解析结果，不含帧头/帧尾/CRC） */
typedef struct {
    uint8_t ft;
    uint8_t da;
    uint8_t pa;
    uint8_t sa;
    uint8_t mn;
    uint8_t tn;
    uint8_t dlc;
    uint8_t data[FECBUS_RX_MAX_DLC];  /* 有效数据，data[0]=功能码 */
} FecbusRxFrame_t;

/* USART3 IT 接收当前字节（bsp_itcallback.c 中断中调用 FecbusRx_OnByte） */
extern volatile uint8_t g_fecbus_rx_byte;

/**
 * @brief  中断接收一字节，写入环形缓冲
 * @param  byte: USART3 接收到的一字节
 * @note   在 HAL_UART_RxCpltCallback 的 USART3 分支中调用
 */
void FecbusRx_OnByte(uint8_t byte);

/**
 * @brief  启动 USART3 逐字节 IT 接收
 * @note   在 Fecbus_Init() 中调用一次即可
 */
void Fecbus_RxInit(void);

/**
 * @brief  轮询解析环形缓冲中已收到的字节
 * @note   在 Fecbus_TxTaskLoop 空闲时调用（约 100ms 周期）
 */
void Fecbus_RxPoll(void);

/**
 * @brief  清空接收环形缓冲
 * @note   发送单播前调用，丢弃历史残留数据
 */
void FecbusRx_Flush(void);

/**
 * @brief  清零 0FH 应答标志
 * @note   发送单播前调用，供 Fecbus_SendFrameWithAck 使用
 */
void FecbusRx_ResetAck(void);

/**
 * @brief  查询是否收到与指定 mn 匹配的 0FH 应答
 * @param  mn: 期望的报文编号
 * @retval 1=收到匹配应答（标志已清除），0=未收到
 */
uint8_t FecbusRx_CheckAck(uint8_t mn);

/**
 * @brief  回发 0FH 状态应答帧（协议层闭环）
 * @param  req:    请求帧（取其 SA/PA/MN 反填应答）
 * @param  status: 应答状态，0x00=正常
 * @param  data:   附加数据（可空，无附加传 NULL/0）
 * @param  len:    附加数据长度（<=8）
 */
void FecbusRx_Reply(const FecbusRxFrame_t *req, uint8_t status, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FECBUS_RX_H */
