/*
 * it.c
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#include "it.h"


extern UART_HandleTypeDef USART1Handle;


void EXTI0_IRQHandler(void)
{
	/*
	if(EXTI->PR & (1 << 0))
	{
		EXTI->PR |= (1 << 0);
	}

	GPIO_TogglePin(GPIOA, GPIO_PIN_3);
	*/
}

void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART1Handle);
}


void SysTick_Handler(void)
{
	GPIO_TogglePin(GPIOA, GPIO_PIN_3);
}
