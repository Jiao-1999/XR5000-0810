#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "main.h"

/* XR5000_UART4_DEBUG_20260811: 调试输出通过UART4直接寄存器发送(PC10/PC11, 115200 8N1) */
#define DEBUG_OUTPUT_ENABLED 1   /* XR5000_UART4_TO_FECBUS_20261110: UART4 改作 FECbus 总线, 调试输出停用。
                                  * 恢复调试输出: 改回 1 并把 bsp_debug.c 的 UART4 改回调试口。 */

void DebugSendString(uint8_t *buf, uint8_t len);
void DebugPrintf(const char *format, ...);
void DebugPrintTask(void *parameter);  /* async log pump task on UART4 */

#endif
