#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define SENDBUFF_SIAE           100
#define USART_REC_LEN  			100

extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

void uart_init(u32 bound);
void Usart_SendData(unsigned char *_p_data, unsigned char _len);

#endif
