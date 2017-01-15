/*
 * functions.h
 *
 *  Created on: 26. 12. 2016
 *      Author: Filip
 */
#include <stddef.h>
#include "stm32l1xx.h"

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_
	void init_PIR(void);
	void init_Button(void);
	void init_GPIO(void);
	void init_PWM(void);
	uint16_t RNG(void);
	void servo(void);
	void delay_ms(uint32_t ms);
	void SysTick_Init(void);
	void TimeTick_Decrement(void);
	void lowPowerRunMode(void);

#endif /* FUNCTIONS_H_ */
