/*
 * core.c
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#include "core.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

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
