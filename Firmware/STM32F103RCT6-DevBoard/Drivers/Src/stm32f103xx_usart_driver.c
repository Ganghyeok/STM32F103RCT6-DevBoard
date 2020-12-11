/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx_usart_driver.h"



/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void USART_Init(UART_HandleTypeDef *pUSARTHandle)
{
	// Temporary register for USART configuration
	uint32_t config= 0;

	// USARTx Clock Enable
	USART_PeripheralClockControl(pUSARTHandle->Instance, ENABLE);

	// USARTx Disable for configuration
	USART_PeripheralControl(pUSARTHandle->Instance, DISABLE);

	/* --------------------- USART_CR1 configuration --------------------- */

	// 1. Enable USART Tx and Rx engines according to the Mode configuration item
	config |= (uint32_t)pUSARTHandle->Init.Mode | pUSARTHandle->Init.WordLength | pUSARTHandle->Init.Parity;
	MODIFY_REG(pUSARTHandle->Instance->CR1, (uint32_t)(USART_CR1_TE | USART_CR1_RE | USART_CR1_M | USART_CR1_PCE | USART_CR1_PS), config);
	config = 0;

	/* --------------------- USART_CR2 configuration --------------------- */
	config |= (uint32_t)pUSARTHandle->Init.StopBits;
	MODIFY_REG(pUSARTHandle->Instance->CR2, (uint32_t)USART_CR2_STOP, config);
	config = 0;

	/* --------------------- USART_CR3 configuration --------------------- */
	config |= (uint32_t)pUSARTHandle->Init.HwFlowCtl;
	MODIFY_REG(pUSARTHandle->Instance->CR3, (uint32_t)(USART_CR3_CTSE | USART_CR3_RTSE), config);
	config = 0;

	/* --------------------- USART_BRR configuration --------------------- */
	USART_SetBaudRate(pUSARTHandle->Instance, pUSARTHandle->Init.BaudRate);


	// USARTx Enable for configuration
	USART_PeripheralControl(pUSARTHandle->Instance, ENABLE);
}


void __weak USARTx_Init(UART_HandleTypeDef * pUSARTHandle)
{

}


void USART_GPIOInit(USART_TypeDef *USARTx)
{
	GPIO_HandleTypeDef GPIOHandle;

	memset(&GPIOHandle, 0, sizeof(GPIOHandle));

	// 1. Check USART peripheral number
	if(USARTx == USART1)
	{
		// USART1 Tx
		GPIOHandle.Instance = GPIOA;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_9;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART1 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_10;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);
	}
}


void USART_PeripheralClockControl(USART_TypeDef *USARTx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(USARTx == USART1)		RCC_USART1_CLK_ENABLE();
		else if(USARTx == USART2)	RCC_USART2_CLK_ENABLE();
		else if(USARTx == USART3)	RCC_USART3_CLK_ENABLE();
		else if(USARTx == UART4)	RCC_UART4_CLK_ENABLE();
		else if(USARTx == UART5)	RCC_UART5_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(USARTx == USART1)		RCC_USART1_CLK_DISABLE();
		else if(USARTx == USART2)	RCC_USART2_CLK_DISABLE();
		else if(USARTx == USART3)	RCC_USART3_CLK_DISABLE();
		else if(USARTx == UART4)	RCC_UART4_CLK_DISABLE();
		else if(USARTx == UART5)	RCC_UART5_CLK_DISABLE();
	}
}


void USART_PeripheralControl(USART_TypeDef *USARTx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)				SET_BIT(USARTx->CR1, USART_CR1_UE);
	else if(En_or_Di == DISABLE)		CLEAR_BIT(USARTx->CR1, USART_CR1_UE);
}


void USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t BaudRate)
{
	uint32_t pclk;
	uint32_t usartDiv;
	uint32_t mantissa, fraction;
	uint8_t tmp;

	// 1. Get PCLK of USARTx
	if(USARTx == USART1)
	{
		// Case of USART1
		pclk = RCC_GetPCLKxValue(PCLK2);
	}
	else
	{
		// Case of USART2/3/4/5
		pclk = RCC_GetPCLKxValue(PCLK1);
	}

	// 2. Calculate USARTDIV vlaue by given PCLK, BaudRate
	usartDiv = ((float)pclk / (16 * BaudRate)) * 100;

	// 3. Extract Mantissa part from USARTDIV
	mantissa = usartDiv / 100;

	// 4. Extract Fraction part from USARTDIV
	tmp = usartDiv - (mantissa * 100);

	fraction = (uint8_t)((((tmp * 16) + 50) / 100) & 0xF);

	// 5. Write Mantissa part and Fraction part value to USART_BRR
	CLEAR_REG(USARTx->BRR);

	USARTx->BRR |= ((mantissa << 4) | fraction);
}


void USART_Transmit(UART_HandleTypeDef *pUSARTHandle, uint8_t *TxBuffer, uint16_t Size)
{
	// 1. Check that state of USARTx is READY
	if(pUSARTHandle->State != USART_STATE_READY)
	{
		// state of USARTx is not ready
		return;
	}

	// 2. Change state of USARTx to BUSY_TX
	pUSARTHandle->State = USART_STATE_BUSY_TX;

	// 3. Configure TxBufferSize, TxCount variable
	pUSARTHandle->TxXferSize = Size;
	pUSARTHandle->TxXferCount = Size;

	// 4. Write data to USART_DR until TxCount value reaches to zero
	while(pUSARTHandle->TxXferCount > 0)
	{
		WAIT_FLAG_SET(pUSARTHandle->Instance->SR, USART_SR_TXE);

		pUSARTHandle->Instance->DR = (uint8_t)(*TxBuffer & 0xFF);

		TxBuffer++;
		pUSARTHandle->TxXferCount--;
	}

	// 5. Wait until TC flag is set in the SR
	WAIT_FLAG_SET(pUSARTHandle->Instance->SR, USART_SR_TC);

	// 6. Change state of USARTx to READY
	pUSARTHandle->State = USART_STATE_READY;
}


