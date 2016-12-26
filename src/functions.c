/*
 * functions.c
 *
 *  Created on: 26. 12. 2016
 *      Author: Filip
 */

#include "functions.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include <stdio.h>


void init_button(void)
{
	// Button - PC13
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;			// PC13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		// Input
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// No Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void init_led(void)
{
	// LED - PA5
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			// PA5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// Output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	// 40MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		// No Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
