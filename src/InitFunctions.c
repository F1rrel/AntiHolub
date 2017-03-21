#include "InitFunctions.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include <stdio.h>

void initPIR(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_Init_PIR;

	GPIO_Init_PIR.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init_PIR.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_PIR.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init_PIR.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB,&GPIO_Init_PIR);
}

void initLED(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_Init_LED;

	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_LED.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init_LED.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(GPIOA,&GPIO_Init_LED);
}
