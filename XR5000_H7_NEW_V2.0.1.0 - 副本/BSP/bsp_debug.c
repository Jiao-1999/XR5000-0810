�?include "bsp_debug.h"
#include "stdarg.h"
#include "string.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/*==============================================================
 * �ļ�����   : bsp_debug.c
 * ģ�鹦��   : ���Դ�ӡ(����������+��������ܹ�?
 * ģ��˵��   : ����ɰ�?DebugPrintf ����Ļ�������������ĵ��õ���
 *              ջ���?tx2buf[400]ռջ)+������ѯ������Ļ���յ����⡣
 *
 *              �¼ܹ�:
 *              - DebugPrintf: ���ö˸�ʽ����ջС����(128B,��400B),
 *                ���жϼ���(��us)ѹ��ring���к��������ء������������ջ��?
 *              - DebugPrintTask: �����ȼ���������(ջ1024),�Ӷ���ȡ��
 *                ��UART4��ѯ���͡���ʹUART4�쳣��ס,Ҳ��Ӱ����Ļ����
 *                �ȸ�ʵʱ����
 *              - ring����������Ϣ(���Կɽ���),�������������ߡ�
 *
 * Ӳ��ƽ̨   : STM32H723ZGT6, UART4(PC10=TX/PC11=RX)�����Կ�
 * �ܹ���λ   : ���������?��������ɰ�ȫ����?
 *==============================================================*/

#if DEBUG_OUTPUT_ENABLED

/* --- ring buffer ����: 16�� x 128�ֽ�,��2KB --- */
#define DBG_BLK_NUM      16
#define DBG_BLK_SIZE     128

/* ���λ�����: ÿ���һ���Ѹ�ʽ���õ����?*/
static uint8_t  s_ring[DBG_BLK_NUM][DBG_BLK_SIZE];
static uint16_t s_ring_len[DBG_BLK_NUM];   /* ÿ��ʵ����Ч���� */
static volatile uint16_t s_head = 0;        /* ������д��λ�� */
static volatile uint16_t s_tail = 0;        /* �����߶�ȡλ�� */

/*--------------------------------------------------------------
 * ѹ��һ����Ϣ(��������,���жϼ��̱���)
 * �� ��������Ϣ,��������������
 *--------------------------------------------------------------*/
static void ring_push(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    taskDISABLE_INTERRUPTS();              /* ���ж�,����us,��Ӱ��UART8���� */
    next = (uint16_t)((s_head + 1) % DBG_BLK_NUM);
    if (next == s_tail)
    {
        /* ������:��������Ϣ(���Գ����ɽ���) */
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
 * ȡ��һ����Ϣ(��DebugPrintTask�������ߵ���)
 * �� ����0,���򿽱���buf�����س���
 *--------------------------------------------------------------*/
static uint16_t ring_pop(uint8_t *buf, uint16_t maxlen)
{
    uint16_t i;
    uint16_t len;

    taskDISABLE_INTERRUPTS();
    if (s_tail == s_head)
    {
        taskENABLE_INTERRUPTS();
        return 0;                          /* ���п� */
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
 * �ײ�UART4����(��ѯ,�̳�ʱ,æ���������ֽ�)
 * �� �� DebugPrintTask ����,����Ҳ��Ӱ����������
 *--------------------------------------------------------------*/
static void uart_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        uint32_t to = 5000;                /* �̳�ʱ,ԶС��ԭ100000 */
        while (!(UART4->ISR & USART_ISR_TXE_TXFNF) && to--) { ; }
        if (to == 0) continue;            /* ��ʱ�������ֽ�,������ */
        UART4->TDR = (uint32_t)buf[i];
    }
    {
        uint32_t to = 5000;
        while (!(UART4->ISR & USART_ISR_TC) && to--) { ; }
    }
}
#endif /* DEBUG_OUTPUT_ENABLED */

/*--------------------------------------------------------------
 * ֱ�ӷ���(������ݾɽӿ�?ͬ������)
 * ע��: �ڸ�ʵʱ��������������,������ DebugPrintf
 *--------------------------------------------------------------*/
void DebugSendString(uint8_t *buf, uint8_t len)
{
#if DEBUG_OUTPUT_ENABLED
    /* ֱ��ѹ�����?�ɺ�̨�����첽���?*/
    if (len > 0) ring_push(buf, len);
#else
    (void)buf;
    (void)len;
#endif
}

/*--------------------------------------------------------------
 * ���Դ�ӡ���?��������ɰ�ȫ����?
 * ���ö˸�ʽ����ջС����(128B)��ѹ������������
 * ջռ��: 128B����+vsnprintf�ڲ�,Լ300~400B(ԭ��700B+)
 * �����Ļ�����?536,��ȫ�����?
 *--------------------------------------------------------------*/
void DebugPrintf(const char *format, ...)
{
#if DEBUG_OUTPUT_ENABLED
    char buf[DBG_BLK_SIZE];                /* ջ��128B(ԭ��400B) */
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
 * ��ӡ����: �����ȼ�,��ring����ȡ����Ϣ��UART4����
 * ����ջ(1024B),�������ȼ�,��ʹ����Ҳ��Ӱ����Ļ���յ�
 * ��ʵʱ����
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
            osDelay(5);                    /* ���п�,�ó�CPU */
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
