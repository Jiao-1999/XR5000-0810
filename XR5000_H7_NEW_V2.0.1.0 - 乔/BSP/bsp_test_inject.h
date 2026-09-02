#ifndef __BSP_TEST_INJECT_H
#define __BSP_TEST_INJECT_H

#include "main.h"

/* XR5000_TEST_INJECT_20260811: COM4命令注入模块
 * 通过UART4(PC10/PC11)接收PC端文本命令, 直接写传感器状态数组,
 * 触发PointTypeDetectorDataDeal()轮询检测变化, 走真实报警/故障链路。
 * 不依赖DMA, 寄存器轮询RX, 与DebugPrintf发送共存。 */

void TestInjectTask(void *argument);

#endif
