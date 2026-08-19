#include "bsp_debug.h"
#include "stdarg.h"
#include "string.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/*==============================================================
 * 文件名称   : bsp_debug.c
 * 模块功能   : 调试打印(非阻塞队列+独立任务架构)
 * 模块说明   : 解决旧版 DebugPrintf 在屏幕接收任务上下文调用导致
 *              栈溢出(tx2buf[400]占栈)+阻塞轮询卡死屏幕接收的问题。
 *
 *              新架构:
 *              - DebugPrintf: 调用端格式化到栈小缓冲(128B,非400B),
 *                关中断极短(几us)压入ring队列后立即返回。零阻塞、零大栈。
 *              - DebugPrintTask: 低优先级独立任务(栈1024),从队列取出
 *                用UART4轮询发送。即使UART4异常卡住,也不影响屏幕接收
 *                等高实时任务。
 *              - ring满则丢弃新消息(调试可接受),绝不阻塞调用者。
 *
 * 硬件平台   : STM32H723ZGT6, UART4(PC10=TX/PC11=RX)作调试口
 * 架构定位   : 基础服务层,所有任务可安全调用
 *==============================================================*/

#if DEBUG_OUTPUT_ENABLED

/* --- ring buffer 配置: 16块 x 128字节,共2KB --- */
#define DBG_BLK_NUM      16
#define DBG_BLK_SIZE     128

/* 环形缓冲区: 每块存一条已格式化好的消息 */
static uint8_t  s_ring[DBG_BLK_NUM][DBG_BLK_SIZE];
static uint16_t s_ring_len[DBG_BLK_NUM];   /* 每块实际有效长度 */
static volatile uint16_t s_head = 0;        /* 生产者写入位置 */
static volatile uint16_t s_tail = 0;        /* 消费者读取位置 */

/*--------------------------------------------------------------
 * 压入一条消息(多生产者,关中断极短保护)
 * 满 则丢弃新消息,绝不阻塞调用者
 *--------------------------------------------------------------*/
static void ring_push(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    taskDISABLE_INTERRUPTS();              /* 关中断,仅几us,不影响UART8接收 */
    next = (uint16_t)((s_head + 1) % DBG_BLK_NUM);
    if (next == s_tail)
    {
        /* 队列满:丢弃新消息(调试场景可接受) */
        taskENABLE_INTERRUPTS();
        return;
    }
    if (len > DBG_BLK_SIZE) len = DBG_BLK_SIZE;
    for (i = 0; i < len; i++)
    {
        s_ring[s_head][i] = data[i];
    }
    s_ring_len[s_head] = len;
    s_head = next;
    taskENABLE_INTERRUPTS();
}

/*--------------------------------------------------------------
 * 取出一条消息(仅DebugPrintTask单消费者调用)
 * 空 返回0,有则拷贝到buf并返回长度
 *--------------------------------------------------------------*/
static uint16_t ring_pop(uint8_t *buf, uint16_t maxlen)
{
    uint16_t i;
    uint16_t len;

    taskDISABLE_INTERRUPTS();
    if (s_tail == s_head)
    {
        taskENABLE_INTERRUPTS();
        return 0;                          /* 队列空 */
    }
    len = s_ring_len[s_tail];
    if (len > maxlen) len = maxlen;
    for (i = 0; i < len; i++)
    {
        buf[i] = s_ring[s_tail][i];
    }
    s_tail = (uint16_t)((s_tail + 1) % DBG_BLK_NUM);
    taskENABLE_INTERRUPTS();
    return len;
}

/*--------------------------------------------------------------
 * 底层UART4发送(轮询,短超时,忙则跳过该字节)
 * 仅 由 DebugPrintTask 调用,阻塞也不影响其他任务
 *--------------------------------------------------------------*/
static void uart_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        uint32_t to = 5000;                /* 短超时,远小于原100000 */
        while (!(UART4->ISR & USART_ISR_TXE_TXFNF) && to--) { ; }
        if (to == 0) continue;            /* 超时跳过该字节,不阻塞 */
        UART4->TDR = (uint32_t)buf[i];
    }
    {
        uint32_t to = 5000;
        while (!(UART4->ISR & USART_ISR_TC) && to--) { ; }
    }
}
#endif /* DEBUG_OUTPUT_ENABLED */

/*--------------------------------------------------------------
 * 直接发送(保留兼容旧接口,同步发送)
 * 注意: 在高实时任务上下文慎用,建议用 DebugPrintf
 *--------------------------------------------------------------*/
void DebugSendString(uint8_t *buf, uint8_t len)
{
#if DEBUG_OUTPUT_ENABLED
    /* 直接压入队列,由后台任务异步输出 */
    if (len > 0) ring_push(buf, len);
#else
    (void)buf;
    (void)len;
#endif
}

/*--------------------------------------------------------------
 * 调试打印入口(任意任务可安全调用)
 * 调用端格式化到栈小缓冲(128B)后压队列立即返回
 * 栈占用: 128B缓冲+vsnprintf内部,约300~400B(原版700B+)
 * 配合屏幕任务栈1536,安全无溢出
 *--------------------------------------------------------------*/
void DebugPrintf(const char *format, ...)
{
#if DEBUG_OUTPUT_ENABLED
    char buf[DBG_BLK_SIZE];                /* 栈仅128B(原版400B) */
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len < 0) len = 0;
    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;

    ring_push((const uint8_t *)buf, (uint16_t)len);
#else
    (void)format;
#endif
}

/*--------------------------------------------------------------
 * 打印任务: 低优先级,从ring队列取出消息用UART4发送
 * 独立栈(1024B),独立优先级,即使卡死也不影响屏幕接收等
 * 高实时任务
 *--------------------------------------------------------------*/
void DebugPrintTask(void *parameter)
{
#if DEBUG_OUTPUT_ENABLED
    uint8_t buf[DBG_BLK_SIZE];
    uint16_t len;

    (void)parameter;
    for (;;)
    {
        len = ring_pop(buf, sizeof(buf));
        if (len > 0)
        {
            uart_send(buf, len);
        }
        else
        {
            osDelay(5);                    /* 队列空,让出CPU */
        }
    }
#else
    (void)parameter;
    for (;;)
    {
        osDelay(1000);
    }
#endif
}
