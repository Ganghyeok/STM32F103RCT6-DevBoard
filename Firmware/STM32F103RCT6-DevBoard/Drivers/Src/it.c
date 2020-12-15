/*
 * it.c
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#include "it.h"


extern UART_HandleTypeDef USART1Handle;
extern TIM_HandleTypeDef TIMHandle;

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


void TIM6_IRQHandler(void)
{
	CLEAR_BIT(TIMHandle.Instance->SR, TIM_SR_UIF);
	GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}
