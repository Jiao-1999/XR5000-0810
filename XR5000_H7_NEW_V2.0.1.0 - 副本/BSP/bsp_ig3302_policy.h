#ifndef __BSP_IG3302_POLICY_H
#define __BSP_IG3302_POLICY_H

#include "main.h"

/* 返回全部已上线IG3302风机的固定联动目标：0=全部停止，1=全部启动。
 * 回路1/3火警的停止命令始终高于回路3 CO/H2 报警的启动命令。 */
uint8_t IG3302Policy_GetAllFanTarget(void);

#endif
