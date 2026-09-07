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
/* USART1 IT 接收当前字节（主机通道收从机应答, 与 USART3 共用解析状态机） */
extern volatile uint8_t g_fecbus_rx_byte_u1;

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
 * @brief  清零应答标志(组应答 s_ack_ok / 异常应答 s_ack_nak)
 * @note   发送单元前调用，供 Fecbus_SendEventGroup 使用(D4)
 */
void FecbusRx_ResetAck(void);

/**
 * @brief  设置本次发送期望被回显的功能码(D4)
 * @param  func: 期望对端回显的功能码(事件/查询功能码)
 * @note   发送前调用; Dispatch 收到 FT=1 且 dlc=1 且 data[0]==func 视为正常应答
 */
void FecbusRx_SetAckFunc(uint8_t func);

/**
 * @brief  查询组应答结果(D4, 表C.6)
 * @param  mn:   期望的报文编号
 * @param  func: 期望回显功能码(冗余校验; 实际比对在 Dispatch 侧完成)
 * @retval 0=尚无应答; 1=收到组应答(回显功能码 或 0FH status=0); 2=NAK(0FH status!=0)
 */
uint8_t FecbusRx_CheckAckEx(uint8_t mn, uint8_t func);

/**
 * @brief  D3: 回发 0FH 状态应答帧 (FT=1, DLC=2, data=[0x0F][status])
 * @param  req:    请求帧（取其 SA/MN 反填应答; PA 固定 03H）
 * @param  status: 状态码 (A3: FECBUS_STAT_*; 仅异常/分组结束/事件结束三类)
 * @note   正常应答用内部 FecbusRx_ReplyEcho(回显功能码); 本函数仅发 0FH 状态帧。
 */
void FecbusRx_ReplyStatus(const FecbusRxFrame_t *req, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FECBUS_RX_H */
