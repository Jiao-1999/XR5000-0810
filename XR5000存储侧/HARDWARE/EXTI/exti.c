#include "exti.h"

#include "delay.h"
#include "usart.h"	
#include "stm32f10x_exti.h"

void EXTIX_Init(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟 

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line7;	
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);
}

//外部中断0服务程序 
void EXTI7_IRQHandler(void)
{
	
	EXTI_ClearITPendingBit(EXTI_Line7); //清除LINE0上的中断标志位  
}
 
 
