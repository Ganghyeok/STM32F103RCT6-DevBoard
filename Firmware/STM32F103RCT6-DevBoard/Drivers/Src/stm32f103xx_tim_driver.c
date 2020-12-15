/*
 * stm32f103xx_tim_driver.c
 *
 *  Created on: 2020. 12. 12.
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx_tim_driver.h"




/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TIM_BaseInit(TIM_HandleTypeDef *pTIMHandle)
{
	uint32_t temp = 0;

	// 1. Check state of TIMx is RESET
	if(pTIMHandle->State != TIM_STATE_RESET)
	{
		// State of TIMx is not RESET
		return;
	}

	// 2. TIMx Clock Enable
	TIM_PeripheralClockControl(pTIMHandle->Instance, ENABLE);

	// 3. Configure counter mode
	temp |= pTIMHandle->Init.CounterMode;

	// 4. Decide the use of Auto-reload preload
	temp |= pTIMHandle->Init.AutoReloadPreload;

	MODIFY_REG(pTIMHandle->Instance->CR1, (TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_ARPE), temp);

	// 5. Configure ARR value
	pTIMHandle->Instance->ARR = (uint32_t)pTIMHandle->Init.Period;

	// 6. Configure Prescaler value
	pTIMHandle->Instance->PSC = pTIMHandle->Init.Prescaler;

	// 7. Configure Repetition counter value
	pTIMHandle->Instance->RCR = pTIMHandle->Init.RepetitionCounter;

	// 8. Generate update event to reload some registers
	pTIMHandle->Instance->EGR |= TIM_EGR_UG;
}



void TIM_GPIOInit(TIM_TypeDef *TIMx)
{




}



void TIM_PeripheralClockControl(TIM_TypeDef *TIMx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_ENABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_ENABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_ENABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_ENABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_ENABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_ENABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_ENABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_DISABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_DISABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_DISABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_DISABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_DISABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_DISABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_DISABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_DISABLE();
	}
}
