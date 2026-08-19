#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void TIM4_Int_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void FUN_CTRL(void);

#define FUN      PAout(1)
#endif
