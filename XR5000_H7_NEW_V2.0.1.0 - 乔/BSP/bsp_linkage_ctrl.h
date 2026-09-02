#ifndef __BSP_LINKAGE_CTRL_H
#define __BSP_LINKAGE_CTRL_H

#include "main.h"

typedef enum
{
	StartDelay = 0U,
	FirstSpray = 1U,
	Interval_1 = 2U,
	SecndSpray = 3U,
	Interval_2 = 4U,
	ThirdSpray = 5U,
	
	SprayEnded = 0xFFU

}CurrentSprayState; // 当前喷放状态 伪状态机实现

void LinkageRelayCtrlTask(void *parameter);

#endif
