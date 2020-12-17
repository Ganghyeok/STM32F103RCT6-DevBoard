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



void TIM_LowInit(TIM_TypeDef *TIMx)
{
	GPIO_HandleTypeDef TIMx_GPIOHandle;

	memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

	// 1. Check TIM peripheral number
	if(TIMx == TIM1)
	{
		TIMx_GPIOHandle.Instance = GPIOA;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(GPIOA, &TIMx_GPIOHandle.Init);
	}
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



void TIM_PWM_ConfigChannel(TIM_HandleTypeDef *pTIMHandle, TIM_OC_InitTypeDef *sConfig, uint32_t Channel)
{
	switch (Channel)
	{
		case TIM_CHANNEL_1:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC1E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR1, (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M), sConfig->OCMode);

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC1P, sConfig->OCPolarity);

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR1 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR1 |= TIM_CCMR1_OC1PE;

			break;
		}

		case TIM_CHANNEL_2:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC2E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR1, (TIM_CCMR1_CC2S | TIM_CCMR1_OC2M), (sConfig->OCMode << 8U));

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC2P, (sConfig->OCPolarity << 4U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR2 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR1 |= TIM_CCMR1_OC2PE;

			break;
		}

		case TIM_CHANNEL_3:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC3E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR2, (TIM_CCMR2_CC3S | TIM_CCMR2_OC3M), sConfig->OCMode);

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC3P, (sConfig->OCPolarity << 8U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR3 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR2 |= TIM_CCMR2_OC3PE;

			break;
		}

		case TIM_CHANNEL_4:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC4E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR2, (TIM_CCMR2_CC4S | TIM_CCMR2_OC4M), (sConfig->OCMode << 8U));

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC4P, (sConfig->OCPolarity << 12U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR4 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR2 |= TIM_CCMR2_OC4PE;

			break;
		}
		default :
			break;
	}

}



void TIM_PWM_Start(TIM_HandleTypeDef *pTIMHandle, uint32_t Channel)
{
	// Enable the channel
	if(Channel == TIM_CHANNEL_1)		pTIMHandle->Instance->CCER |= TIM_CCER_CC1E;
	else if(Channel == TIM_CHANNEL_2)	pTIMHandle->Instance->CCER |= TIM_CCER_CC2E;
	else if(Channel == TIM_CHANNEL_3)	pTIMHandle->Instance->CCER |= TIM_CCER_CC3E;
	else if(Channel == TIM_CHANNEL_4)	pTIMHandle->Instance->CCER |= TIM_CCER_CC4E;

	// Enable the Main output
	pTIMHandle->Instance->BDTR |= TIM_BDTR_MOE;

	// Enable the TIM1
	pTIMHandle->Instance->CR1 |= TIM_CR1_CEN;
}
