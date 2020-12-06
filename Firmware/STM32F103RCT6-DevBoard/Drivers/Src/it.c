/*
 * it.c
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#include "it.h"



void EXTI0_IRQHandler(void)
{
	//Delay_ms();

	if(EXTI->PR & (1 << 0))
	{
		EXTI->PR |= (1 << 0);
	}

	GPIO_TogglePin(GPIOA, GPIO_PIN_3);
}


void SysTick_Handler(void)
{
	GPIO_TogglePin(GPIOA, GPIO_PIN_3);
}
