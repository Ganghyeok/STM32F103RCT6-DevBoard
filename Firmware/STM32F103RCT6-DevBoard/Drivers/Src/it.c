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


void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ISER[0] |= (1 << IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ISER[1] |= (1 << (IRQNumber % 32));
		}

	}
	else if(En_or_Di == DISABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ICER[0] |= (1 << IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ICER[1] |= (1 << (IRQNumber % 32));
		}
	}

	// IRQ Priority configuration

	NVIC->IPR[IRQNumber] |= (IRQPriority << 4);
}
