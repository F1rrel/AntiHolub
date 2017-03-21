/*
 * main.c
 *
 *  Created on: 26. 12. 2016
 *      Author: Filip
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include "functions.h"
#include "stm32l1xx_it.h"

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
	#define positionHome			 800
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	extern uint16_t PIR;
	extern uint16_t Button;
	uint16_t count = 0;
	uint8_t step = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  SysTick_Init();
  init_GPIO();
  init_PIR();
  init_Button();
  init_PWM();
  //lowPowerRunMode();
  
  /* Infinite loop */
  while (1)
  {
	  switch (step) {

	  	  case 0:
	  		  TIM2->CCR1 = positionHome;
	  		  delay_ms(500);
	  		  step = 1;
	  		  break;

		  case 1:
			  if (PIR == 1) {
				  step = 10;
			  }
			  break;

		  case 10:
			  if ((PIR == 0) && (count > 40)) {
				  count = 0;
				  step = 20;
				  break;
			  }
			  count ++;
			  servo();
			  delay_ms(250);
			  break;

		  case 20:
			  TIM2->CCR1 = positionHome;
			  step = 1;
			  delay_ms(10000);
			  break;

		  case 100:	// Sleep mode
			  if (Button == 1) {
				  Button = 0;
				  step = 1;
			  }
			  break;
	  }

	  // Go to sleep mode
	  if ((Button == 1) && (step != 100)) {
		  TIM2->CCR1 = positionHome;
		  Button = 0;
		  count = 0;
		  step = 100;
	  }
  }

}


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
