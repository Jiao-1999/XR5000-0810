#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "main.h"

#define ReadHandPaperOnline()       ((IN3A_GPIO_Port->IDR & IN3A_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadHandPaperKeyState()     ((IN3B_GPIO_Port->IDR & IN3B_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadHandPaperCircuitState() ((IN3C_GPIO_Port->IDR & IN3C_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))

#define ReadFeedback1Online()       ((IN2A_GPIO_Port->IDR & IN2A_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadFeedback1KeyState()     ((IN2B_GPIO_Port->IDR & IN2B_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadFeedback1CircuitState() ((IN2C_GPIO_Port->IDR & IN2C_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))

#define ReadFeedback2Online()       ((IN1A_GPIO_Port->IDR & IN1A_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadFeedback2KeyState()     ((IN1B_GPIO_Port->IDR & IN1B_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ReadFeedback2CircuitState() ((IN1C_GPIO_Port->IDR & IN1C_Pin)?(GPIO_PIN_SET):(GPIO_PIN_RESET))

typedef enum
{
	FeedbackKeyOnline     = 0x03,
	FeedbackKeyPress      = 0x01,
//	FeedbackKeyLoosen     = 0x03, // 松开和在线实际上是一个值，故只用在线状态
	FeedbackKeyShort      = 0x00,
	FeedbackKeyDisconnect = 0x07
}FeedbackKeyState;

uint8_t getHandPaperState(void);
uint8_t getFeedBack1State(void);
uint8_t getFeedBack2State(void);

void setDealHandPaperState(void);
void setDealFeedBack1State(void);
void setDealFeedBack2State(void);

void clearHandPaperState(void);
void cleareedBack1State(void);
void cleareedBack2State(void);


void KeyStateJudgeTask(void* parameter);

#endif

