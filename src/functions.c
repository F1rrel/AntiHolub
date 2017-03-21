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
	#define CLOCK_CYCLES_PER_SECOND  16000000	// 16MHz
	#define MAX_RELOAD               0xFFFF		// 16 bits timer (65535)
	#define positionLow				 1200
	#define positionHigh			 2050
	#define positionCentre			 1625
	#define resolution				 19
/* Private macro -------------------------------------------------------------*/
	static uint32_t sysTickCounter;
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void init_GPIO(void)
{
	// enable clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);		// GPIOA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);		// GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// SYSCFG

	// PIR - PA6
	GPIO_InitTypeDef PIR_InitStructure;
	PIR_InitStructure.GPIO_Pin = GPIO_Pin_6;			// PA6
	PIR_InitStructure.GPIO_Mode = GPIO_Mode_IN;			// Input
	PIR_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		// Pull up
	PIR_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	PIR_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	// 40MHz
	GPIO_Init(GPIOA, &PIR_InitStructure);

	// Button - PC13 //PA10
	GPIO_InitTypeDef Button_InitStructure;
	Button_InitStructure.GPIO_Pin = GPIO_Pin_13;			// PA10
	Button_InitStructure.GPIO_Mode = GPIO_Mode_IN;			// Input
//	Button_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		// 40MHz
	Button_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		// Pull down
	Button_InitStructure.GPIO_OType = GPIO_OType_PP;		// PushPull
	GPIO_Init(GPIOC, &Button_InitStructure);

	// Configure PA0 as PWM output
	GPIO_InitTypeDef Timer_gpioStructure;
	Timer_gpioStructure.GPIO_Pin = GPIO_Pin_0;
	Timer_gpioStructure.GPIO_OType = GPIO_OType_PP;
	Timer_gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	Timer_gpioStructure.GPIO_Mode = GPIO_Mode_AF;
	Timer_gpioStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_Init(GPIOA, &Timer_gpioStructure);

}

void init_PIR(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			// Set sub priority (second highest)
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// Enable interrupt
	NVIC_Init(&NVIC_InitStructure);									// Add to NVIC
}

void init_Button(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// EXTI
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);	// Tell system that you will use PA10 for EXTI_Line10

	EXTI_InitStructure.EXTI_Line = EXTI_Line13;						// PA10 is connected to EXTI_Line10
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;						// Enable interrupt
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				// Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;			// Triggers on falling edge
	EXTI_Init(&EXTI_InitStructure);									// Add to EXTI

	// Add IRQ vector to NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			// PA10 is on EXTI15_10_IRQn vector
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	// Set priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			// Set sub priority (highest)
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// Enable interrupt
	NVIC_Init(&NVIC_InitStructure);									// Add to NVIC
}

void init_PWM(void)
{
	/*	wanted pulse 20ms = 50Hz
	 * 	Prescaler = 32 for 1Mhz clock (1 tick = 1 us)
	 * 	Period = 20000 for prescaler / wanted frequency (1000000/50)
	 * 	Pulse = 1500 = 1.5ms for middle position (1ms lowest and 2ms highest position)
	 */

	/* Configure Timer -----------------------------------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef Timer_InitStructure;
	Timer_InitStructure.TIM_Prescaler = 32;
	Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_InitStructure.TIM_Period = 19999;
	Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &Timer_InitStructure);
	TIM_Cmd(TIM2, ENABLE);

	/* Configure Timer OC --------------------------------------------------*/
	TIM_OCInitTypeDef TimerOC_InitStructure;
	TimerOC_InitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TimerOC_InitStructure.TIM_Pulse = 1000;
	TimerOC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TimerOC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC1Init(TIM2, &TimerOC_InitStructure);	// TIM2 channel 1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

}

uint16_t RNG(void)
{
	uint16_t number;
	number = TIM2->CNT;		// take value from TIM2 counter
	if (number > 15) {		// if value is big, take first 4 bits (value 0-15)
		number = TIM2->CNT & 0b1111;
	}
	return (number + 4);
}

void servo(void)
{
	uint16_t position = TIM2->CCR1;
	uint16_t number = RNG();

	if (position > positionCentre) {
		TIM2->CCR1 = position - (uint32_t)(((position-positionLow)/resolution)*number);
	}
	else
	{
		TIM2->CCR1 = position + (uint32_t)(((positionHigh - position)/resolution)*number);
	}


}

void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	while (SysTick_Config(SystemCoreClock / 1000) != 0) {
	} // One SysTick interrupt now equals 1ms
}

/**
 * This method needs to be called in the SysTick_Handler
 */
void TimeTick_Decrement(void) {
	if (sysTickCounter != 0x00) {
		sysTickCounter--;
	}
}

void delay_ms(uint32_t ms) {
	sysTickCounter = ms;
	while (sysTickCounter != 0) {
	}
}

void lowPowerRunMode(void) {
	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
	GPIO_InitTypeDef GPIO_InitStructure;
	  /* Enable GPIOs clock */
	  RCC_AHBPeriphClockCmd(/*RCC_AHBPeriph_GPIOA | */RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
							RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH |
							RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_GPIOG, ENABLE);

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	  GPIO_Init(GPIOE, &GPIO_InitStructure);
	  GPIO_Init(GPIOH, &GPIO_InitStructure);
	  GPIO_Init(GPIOF, &GPIO_InitStructure);
	  GPIO_Init(GPIOG, &GPIO_InitStructure);
	  /*GPIO_Init(GPIOA, &GPIO_InitStructure);*/
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Disable GPIOs clock */
	  RCC_AHBPeriphClockCmd(/*RCC_AHBPeriph_GPIOA | */RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
							RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH |
							RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_GPIOG, DISABLE);


	PWR_VoltageScalingConfig(PWR_VoltageScaling_Range2);		// Configures the voltage scaling range
	/* Wait Until the Voltage Regulator is ready */
	while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;

	PWR_EnterLowPowerRunMode(ENABLE);
	/* Wait until the system enters RUN LP and the Regulator is in LP mode */
	while (PWR_GetFlagStatus(PWR_FLAG_REGLP) == RESET) ;

}





