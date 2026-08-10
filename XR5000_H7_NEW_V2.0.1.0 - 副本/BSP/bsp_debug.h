#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "main.h"

/* XR5000_UART5_EXCLUSIVE_FIX_20260730: move debug output before enabling this adapter. */
#define DEBUG_OUTPUT_ENABLED 0

void DebugSendString(uint8_t *buf, uint8_t len);
void DebugPrintf(const char *format, ...);

#endif
