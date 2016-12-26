/*
 * functions.c
 *
 *  Created on: 26. 12. 2016
 *      Author: Filip
 */

#include "functions.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t PIR;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void init_PIR(void)
{
	// PIR - PA6
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Enable clock ------------------------------------------------------------------*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	// GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// SYSCFG

	// GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;			// PA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		// Input
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// Pull up
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	// 40MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// EXTI
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);	// Tell system that you will use PA6 for EXTI_Line6

	EXTI_InitStructure.EXTI_Line = EXTI_Line6;						// PA6 is connected to EXTI_Line6
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;						// Enable interrupt
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				// Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	// Triggers on rising and falling edge
	EXTI_Init(&EXTI_InitStructure);									// Add to EXTI

	// Add IRQ vector to NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				// PA6 is on EXTI9_5_IRQn vector
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	// Set priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			// Set sub priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// Enable interrupt
	NVIC_Init(&NVIC_InitStructure);									// Add to NVIC
}

void init_led(void)
{
	// LED - PA7
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;			// PA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// Output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	// 40MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		// No Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
	 if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		/* Do your stuff when PA6 is changed */
		PIR = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);

		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line6);
	 }
}
