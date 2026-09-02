#ifndef __BSP_AHT20_H
#define __BSP_AHT20_H

#include "main.h"

void AHT20_Init(void);
uint8_t AHT20_Update(void);
uint8_t AHT20_IsValid(void);
int16_t AHT20_GetTemperature(void);
uint16_t AHT20_GetHumidity(void);

#endif
