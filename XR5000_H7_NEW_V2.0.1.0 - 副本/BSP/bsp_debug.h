#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "main.h"

/* XR5000_UART4_DEBUG_20260811: �������ͨ��UART4ֱ�ӼĴ�������(PC10/PC11, 115200 8N1) */
#define DEBUG_OUTPUT_ENABLED 1

void DebugSendString(uint8_t *buf, uint8_t len);
void DebugPrintf(const char *format, ...);
void DebugPrintTask(void *parameter);  /* async log pump task on UART4 */

#endif
