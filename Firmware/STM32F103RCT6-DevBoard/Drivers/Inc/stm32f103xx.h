/*
 * stm32f103xx.h
 *
 *  Created on: Dec 2, 2020
 *      Author: Ganghyeok Lim
 */

#ifndef STM32F103XX_H_
#define STM32F103XX_H_


#include <stdint.h>
#include <stddef.h>

/*
 *  Generic Macros
 */
#define __IO									volatile
#define __vo									volatile
#define __weak									__attribute__((weak))
#define __naked									__attribute__((naked))
#define HIGH									1
#define LOW										0
#define TRUE									1
#define FALSE									0
#define ENABLE									1
#define DISABLE									0
#define SET										1
#define RESET									0
#define CLEAR									0
#define GPIO_PIN_SET							SET
#define GPIO_PIN_RESET							RESET
#define FLAG_SET								SET
#define FLAG_RESET								RESET


/**************************************************************************************************************
 * 																											  *
 * 									Base Addresses of Memories, Bus, Peripherals							  *
 * 																											  *
 **************************************************************************************************************/

/*
 *  Base Addresses of Flash and SRAM Memories
 */
#define CORE_PERIPH_BASE						0xE0000000U
#define FLASH_BASE								0x08000000U
#define SRAM_BASE								0x20000000U
#define ROM_BASE								0x1FFFF000U


/*
 *  Base Addresses of AHBx and APBx Bus Peripheral
 */
#define PERIPH_BASE								0x40000000U
#define APB1_PERIPH_BASE						PERIPH_BASE
#define APB2_PERIPH_BASE						0x40010000U
#define AHB_PERIPH_BASE							0x40018000U


/*
 *  Base Addresses of Cortex-M3 Internal Peripheral
 */
#define NVIC_BASE								(CORE_PERIPH_BASE + 0xE100)


/*
 *  Base Addresses of Peripherals which are hanging on AHB Bus
 */
#define SDIO_BASE								(AHB_PERIPH_BASE + 0x0000)
#define DMA1_BASE								(AHB_PERIPH_BASE + 0x8000)
#define DMA2_BASE								(AHB_PERIPH_BASE + 0x8400)
#define RCC_BASE								(AHB_PERIPH_BASE + 0x9000)
#define FLITF_BASE								(AHB_PERIPH_BASE + 0xA000)
#define CRC_BASE								(AHB_PERIPH_BASE + 0xB000)
#define FSMC_BASE								(AHB_PERIPH_BASE + 0x5FFE9000)


/*
 *  Base Addresses of Peripherals which are hanging on APB1 Bus
 */
#define TIM2_BASE								(APB1_PERIPH_BASE + 0x0000)
#define TIM3_BASE								(APB1_PERIPH_BASE + 0x0400)
#define TIM4_BASE								(APB1_PERIPH_BASE + 0x0800)
#define TIM5_BASE								(APB1_PERIPH_BASE + 0x0C00)
#define TIM6_BASE								(APB1_PERIPH_BASE + 0x1000)
#define TIM7_BASE								(APB1_PERIPH_BASE + 0x1400)
#define RTC_BASE								(APB1_PERIPH_BASE + 0x2800)
#define WWDG_BASE								(APB1_PERIPH_BASE + 0x2C00)
#define IWDG_BASE								(APB1_PERIPH_BASE + 0x3000)
#define SPI2_BASE								(APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASE								(APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASE								(APB1_PERIPH_BASE + 0x4400)
#define USART3_BASE								(APB1_PERIPH_BASE + 0x4800)
#define UART4_BASE								(APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASE								(APB1_PERIPH_BASE + 0x5000)
#define I2C1_BASE								(APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE								(APB1_PERIPH_BASE + 0x5800)
#define USB_REG_BASE							(APB1_PERIPH_BASE + 0x5C00)
#define SHRD_USB_CAN_SRAM_BASE					(APB1_PERIPH_BASE + 0x6000)
#define BXCAN_BASE								(APB1_PERIPH_BASE + 0x6400)
#define BKP_BASE								(APB1_PERIPH_BASE + 0x6C00)
#define PWR_BASE								(APB1_PERIPH_BASE + 0x7000)
#define DAC_BASE								(APB1_PERIPH_BASE + 0x7400)


/*
 *  Base Addresses of Peripherals which are hanging on APB2 Bus
 */
#define AFIO_BASE								(APB2_PERIPH_BASE + 0x0000)
#define EXTI_BASE								(APB2_PERIPH_BASE + 0x0400)
#define GPIOA_BASE								(APB2_PERIPH_BASE + 0x0800)
#define GPIOB_BASE								(APB2_PERIPH_BASE + 0x0C00)
#define GPIOC_BASE								(APB2_PERIPH_BASE + 0x1000)
#define GPIOD_BASE								(APB2_PERIPH_BASE + 0x1400)
#define GPIOE_BASE								(APB2_PERIPH_BASE + 0x1800)
#define GPIOF_BASE								(APB2_PERIPH_BASE + 0x1C00)
#define GPIOG_BASE								(APB2_PERIPH_BASE + 0x2000)
#define ADC1_BASE								(APB2_PERIPH_BASE + 0x2400)
#define ADC2_BASE								(APB2_PERIPH_BASE + 0x2800)
#define TIM1_BASE								(APB2_PERIPH_BASE + 0x2C00)
#define SPI1_BASE								(APB2_PERIPH_BASE + 0x3000)
#define TIM8_BASE								(APB2_PERIPH_BASE + 0x3400)
#define USART1_BASE								(APB2_PERIPH_BASE + 0x3800)
#define ADC3_BASE								(APB2_PERIPH_BASE + 0x3C00)



/**************************************************************************************************************
 * 																											  *
 * 									Peripheral Register Definition Structures								  *
 * 																											  *
 **************************************************************************************************************/

typedef struct
{
	__IO uint32_t CR;							/* !< RCC Clock control register,												Address offset : 0x00 >	*/
	__IO uint32_t CFGR;							/* !< RCC Clock configuration register,											Address offset : 0x04 >	*/
	__IO uint32_t CIR;							/* !< RCC Clock interrupt register,												Address offset : 0x08 >	*/
	__IO uint32_t APB2RSTR;						/* !< RCC APB2 Peripheral reset register,										Address offset : 0x0C >	*/
	__IO uint32_t APB1RSTR;						/* !< RCC APB1 Peripheral reset register,										Address offset : 0x10 >	*/
	__IO uint32_t AHBENR;						/* !< RCC AHB Peripheral clock enable register,									Address offset : 0x14 >	*/
	__IO uint32_t APB2ENR;						/* !< RCC APB2 Peripheral clock enable register,								Address offset : 0x18 >	*/
	__IO uint32_t APB1ENR;						/* !< RCC APB1 Peripheral clock enable  register,								Address offset : 0x1C >	*/
	__IO uint32_t BDCR;							/* !< RCC Backup domain control register,										Address offset : 0x20 >	*/
	__IO uint32_t CSR;							/* !< RCC Control/Status register,												Address offset : 0x24 >	*/
	__IO uint32_t AHBSTR;						/* !< RCC AHB Peripheral clock reset register,									Address offset : 0x28 >	*/
	__IO uint32_t CFGR2;						/* !< RCC Clock configuration register2,										Address offset : 0x2C >	*/
} RCC_RegDef_t;


typedef struct
{
	__IO uint32_t CRL;							/* !< GPIO Port configuration register low										Address offset : 0x00 > */
	__IO uint32_t CRH;							/* !< GPIO Port configuration register high 									Address offset : 0x04 > */
	__IO uint32_t IDR;							/* !< GPIO Input data register													Address offset : 0x08 > */
	__IO uint32_t ODR;							/* !< GPIO Output data register													Address offset : 0x0C > */
	__IO uint32_t BSRR;							/* !< GPIO Port bit set/reset register											Address offset : 0x10 > */
	__IO uint32_t BRR;							/* !< GPIO Port bit reset register												Address offset : 0x14 > */
	__IO uint32_t LCKR;							/* !< GPIO Port configuration lock register										Address offset : 0x18 > */
} GPIO_RegDef_t;


typedef struct
{
	__IO uint32_t EVCR;							/* !< AFIO Event control register												Address offset : 0x00 > */
	__IO uint32_t MAPR;							/* !< AFIO AF remap and debug I/O configuration register						Address offset : 0x04 > */
	__IO uint32_t EXTICR1;						/* !< AFIO External interrupt configuration register 1							Address offset : 0x08 > */
	__IO uint32_t EXTICR2;						/* !< AFIO External interrupt configuration register 2							Address offset : 0x0C > */
	__IO uint32_t EXTICR3;						/* !< AFIO External interrupt configuration register 3							Address offset : 0x10 > */
	__IO uint32_t EXTICR4;						/* !< AFIO External interrupt configuration register 4							Address offset : 0x14 > */
	__IO uint32_t MAPR2;						/* !< AFIO AF remap and debug I/O configuration register 2						Address offset : 0x1C > */
} AFIO_RegDef_t;


typedef struct
{
	__IO uint32_t ISER[8];						/* !< NVIC Interrupt Set-enable registers										Address offset : 0x000 > */
	__IO uint32_t RESERVED1[24];				/* !< RESERVED1																						   > */
	__IO uint32_t ICER[8];						/* !< NVIC Interrupt Clear-enable registers										Address offset : 0x080 > */
	__IO uint32_t RESERVED2[24];				/* !< RESERVED2																						   > */
	__IO uint32_t ISPR[8];						/* !< NVIC Interrupt Set-pending registers										Address offset : 0x100 > */
	__IO uint32_t RESERVED3[24];				/* !< RESERVED3																						   > */
	__IO uint32_t ICPR[8];						/* !< NVIC Interrupt Clear-pending registers									Address offset : 0x180 > */
	__IO uint32_t RESERVED4[24];				/* !< RESERVED4																						   > */
	__IO uint32_t IABR[8];						/* !< NVIC Interrupt Active bit registers										Address offset : 0x200 > */
	__IO uint32_t RESERVED5[56];				/* !< RESERVED5																						   > */
	__IO uint32_t IPR[60];						/* !< NVIC Interrupt priority registers											Address offset : 0x300 > */
	__IO uint32_t RESERVED6[643];				/* !< RESERVED6																						   > */
	__IO uint32_t STIR;							/* !< NVIC Software trigger interrupt registers									Address offset : 0xE00 > */
} NVIC_RegDef_t;


typedef struct
{
	__IO uint32_t IMR;							/* !< EXTI Interrupt mask register												Address offset : 0x00 > */
	__IO uint32_t EMR;							/* !< EXTI Event mask register													Address offset : 0x04 > */
	__IO uint32_t RTSR;							/* !< EXTI Rising trigger selection register									Address offset : 0x08 > */
	__IO uint32_t FTSR;							/* !< EXTI Falling trigger selection register									Address offset : 0x0C > */
	__IO uint32_t SWIER;						/* !< EXTI Software interrupt event register									Address offset : 0x10 > */
	__IO uint32_t PR;							/* !< EXTI Pending register														Address offset : 0x14 > */
} EXTI_RegDef_t;


typedef struct
{
	__IO uint32_t SR;							/* !< ADC Status register														Address offset : 0x00 > */
	__IO uint32_t CR1;							/* !< ADC Control register 1													Address offset : 0x04 > */
	__IO uint32_t CR2;							/* !< ADC Control register 2													Address offset : 0x08 > */
	__IO uint32_t SMPR1;						/* !< ADC Sample time register 1												Address offset : 0x0C > */
	__IO uint32_t SMPR2;						/* !< ADC Sample time register 2												Address offset : 0x10 > */
	__IO uint32_t JOFR1;						/* !< ADC Injected channel data offset register 1								Address offset : 0x14 > */
	__IO uint32_t JOFR2;						/* !< ADC Injected channel data offset register 2								Address offset : 0x18 > */
	__IO uint32_t JOFR3;						/* !< ADC Injected channel data offset register 3								Address offset : 0x1C > */
	__IO uint32_t JOFR4;						/* !< ADC Injected channel data offset register 4								Address offset : 0x20 > */
	__IO uint32_t HTR;							/* !< ADC Watchdog high threshold register										Address offset : 0x24 > */
	__IO uint32_t LTR;							/* !< ADC Watchdog low threshold register										Address offset : 0x28 > */
	__IO uint32_t SQR1;							/* !< ADC Regular sequence register 1											Address offset : 0x2C > */
	__IO uint32_t SQR2;							/* !< ADC Regular sequence register 2											Address offset : 0x30 > */
	__IO uint32_t SQR3;							/* !< ADC Regular sequence register 3											Address offset : 0x34 > */
	__IO uint32_t JSQR;							/* !< ADC Injected sequence register											Address offset : 0x38 > */
	__IO uint32_t JDR1;							/* !< ADC Injected data register 1												Address offset : 0x3C > */
	__IO uint32_t JDR2;							/* !< ADC Injected data register 2												Address offset : 0x40 > */
	__IO uint32_t JDR3;							/* !< ADC Injected data register 3												Address offset : 0x44 > */
	__IO uint32_t JDR4;							/* !< ADC Injected data register 4												Address offset : 0x48 > */
	__IO uint32_t DR;							/* !< ADC Regular data register													Address offset : 0x4C > */
} ADC_RegDef_t;


typedef struct
{
	__IO uint32_t CR1;							/* !< TIM1/8 TIM1 and TIM8 Control register 1									Address offset : 0x00 > */
	__IO uint32_t CR2;							/* !< TIM1/8 TIM1 and TIM8 Control register 2									Address offset : 0x04 > */
	__IO uint32_t SMCR;							/* !< TIM1/8 TIM1 and TIM8 Slave mode control register							Address offset : 0x08 > */
	__IO uint32_t DIER;							/* !< TIM1/8 TIM1 and TIM8 DMA/Interrupt enable register						Address offset : 0x0C > */
	__IO uint32_t SR;							/* !< TIM1/8 TIM1 and TIM8 Status register										Address offset : 0x10 > */
	__IO uint32_t EGR;							/* !< TIM1/8 TIM1 and TIM8 Event generation register							Address offset : 0x14 > */
	__IO uint32_t CCMR1;						/* !< TIM1/8 TIM1 and TIM8 Capture/Compare mode register 1						Address offset : 0x18 > */
	__IO uint32_t CCMR2;						/* !< TIM1/8 TIM1 and TIM8 Capture/Compare mode register 2						Address offset : 0x1C > */
	__IO uint32_t CCER;							/* !< TIM1/8 TIM1 and TIM8 Capture/Compare enable register						Address offset : 0x20 > */
	__IO uint32_t CNT;							/* !< TIM1/8 TIM1 and TIM8 Counter												Address offset : 0x24 > */
	__IO uint32_t PSC;							/* !< TIM1/8 TIM1 and TIM8 Prescaler											Address offset : 0x28 > */
	__IO uint32_t ARR;							/* !< TIM1/8 TIM1 and TIM8 Auto-reload register									Address offset : 0x2C > */
	__IO uint32_t RCR;							/* !< TIM1/8 TIM1 and TIM8 Repetition counter register							Address offset : 0x30 > */
	__IO uint32_t CCR1;							/* !< TIM1/8 TIM1 and TIM8 Capture/Compare register 1							Address offset : 0x34 > */
	__IO uint32_t CCR2;							/* !< TIM1/8 TIM1 and TIM8 Capture/Compare register	2							Address offset : 0x38 > */
	__IO uint32_t CCR3;							/* !< TIM1/8 TIM1 and TIM8 Capture/Compare register 3							Address offset : 0x3C > */
	__IO uint32_t CCR4;							/* !< TIM1/8 TIM1 and TIM8 Capture/Compare register	4							Address offset : 0x40 > */
	__IO uint32_t BDTR;							/* !< TIM1/8 TIM1 and TIM8 Break and dead-time register							Address offset : 0x44 > */
	__IO uint32_t DCR;							/* !< TIM1/8 TIM1 and TIM8 DMA Control register									Address offset : 0x48 > */
	__IO uint32_t DMAR;							/* !< TIM1/8 TIM1 and TIM8 DMA Address for full transfer						Address offset : 0x4C > */
} TIM1_8_RegDef_t;


typedef struct
{
	__IO uint32_t CR1;							/* !< TIMx Control register 1													Address offset : 0x00 > */
	__IO uint32_t CR2;							/* !< TIMx Control register 2													Address offset : 0x04 > */
	__IO uint32_t SMCR;							/* !< TIMx Slave mode control register											Address offset : 0x08 > */
	__IO uint32_t DIER;							/* !< TIMx DMA/Interrupt enable register										Address offset : 0x0C > */
	__IO uint32_t SR;							/* !< TIMx Status register														Address offset : 0x10 > */
	__IO uint32_t EGR;							/* !< TIMx Event generation register											Address offset : 0x14 > */
	__IO uint32_t CCMR1;						/* !< TIMx Capture/Compare mode register 1										Address offset : 0x18 > */
	__IO uint32_t CCMR2;						/* !< TIMx Capture/Compare mode register 2										Address offset : 0x1C > */
	__IO uint32_t CCER;							/* !< TIMx Capture/Compare enable register										Address offset : 0x20 > */
	__IO uint32_t CNT;							/* !< TIMx Counter																Address offset : 0x24 > */
	__IO uint32_t PSC;							/* !< TIMx Prescaler															Address offset : 0x28 > */
	__IO uint32_t ARR;							/* !< TIMx Auto-reload register													Address offset : 0x2C > */
	__IO uint32_t RESERVED1;					/* !< RESERVED1																						  > */
	__IO uint32_t CCR1;							/* !< TIMx Capture/Compare register 1											Address offset : 0x34 > */
	__IO uint32_t CCR2;							/* !< TIMx Capture/Compare register 2											Address offset : 0x38 > */
	__IO uint32_t CCR3;							/* !< TIMx Capture/Compare register 3											Address offset : 0x3C > */
	__IO uint32_t CCR4;							/* !< TIMx Capture/Compare register 4											Address offset : 0x40 > */
	__IO uint32_t RESERVED2;					/* !< RESERVED2																	 					  > */
	__IO uint32_t DCR;							/* !< TIMx DMA Control register													Address offset : 0x48 > */
	__IO uint32_t DMAR;							/* !< TIMx DMA Address for full transfer										Address offset : 0x4C > */
} TIM2_5_RegDef_t;


typedef struct
{
	__IO uint32_t SR;							/* !< USART Status register														Address offset : 0x00 > */
	__IO uint32_t DR;							/* !< USART Data register														Address offset : 0x04 > */
	__IO uint32_t BRR;							/* !< USART Baud rate register													Address offset : 0x08 > */
	__IO uint32_t CR1;							/* !< USART Control register 1													Address offset : 0x0C > */
	__IO uint32_t CR2;							/* !< USART Control register 2													Address offset : 0x10 > */
	__IO uint32_t CR3;							/* !< USART Control register 3													Address offset : 0x14 > */
	__IO uint32_t GTPR;							/* !< USART Guard time and prescaler register									Address offset : 0x18 > */
} USART_RegDef_t;


/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Definitions											  *
 * 							(Peripheral base addresses type casted to xxx_RegDef_t)							  *
 * 																											  *
 **************************************************************************************************************/

#define RCC										((RCC_RegDef_t*)RCC_BASE)
#define GPIOA									((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB									((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC									((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD									((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE									((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF									((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG									((GPIO_RegDef_t*)GPIOG_BASE)
#define AFIO									((AFIO_RegDef_t*)AFIO_BASE)
#define NVIC									((NVIC_RegDef_t*)NVIC_BASE)
#define EXTI									((EXTI_RegDef_t*)EXTI_BASE)
#define ADC1									((ADC_RegDef_t*)ADC1_BASE)
#define ADC2									((ADC_RegDef_t*)ADC2_BASE)
#define TIM1									((TIM1_8_RegDef_t*)TIM1_BASE)
#define TIM8									((TIM1_8_RegDef_t*)TIM8_BASE)
#define TIM2									((TIM2_5_RegDef_t*)TIM2_BASE)
#define TIM3									((TIM2_5_RegDef_t*)TIM3_BASE)
#define TIM4									((TIM2_5_RegDef_t*)TIM4_BASE)
#define TIM5									((TIM2_5_RegDef_t*)TIM5_BASE)
#define USART1									((USART_RegDef_t*)USART1_BASE)
#define USART2									((USART_RegDef_t*)USART2_BASE)
#define USART3									((USART_RegDef_t*)USART3_BASE)
#define UART4									((USART_RegDef_t*)UART4_BASE)
#define UART5									((USART_RegDef_t*)UART5_BASE)


/**************************************************************************************************************
 * 																											  *
 * 												 Macro for IRQs					 							  *
 * 																											  *
 **************************************************************************************************************/

/*
 *  IRQ(Interrupt Request) Number of STM32F103xx MCU
 */
#define IRQ_NO_RCC								5
#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_ADC1_2							18
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_TIM1_BRK							24
#define IRQ_NO_TIM1_UP							25
#define IRQ_NO_TIM1_TRG_COM						26
#define IRQ_NO_TIM1_CC							27
#define IRQ_NO_TIM2								28
#define IRQ_NO_TIM3								29
#define IRQ_NO_TIM4								30
#define IRQ_NO_I2C1_EV							31
#define IRQ_NO_I2C1_ER							32
#define IRQ_NO_I2C2_EV							33
#define IRQ_NO_I2C2_ER							34
#define IRQ_NO_SPI1								35
#define IRQ_NO_SPI2								36
#define IRQ_NO_USART1							37
#define IRQ_NO_USART2							38
#define IRQ_NO_USART3							39
#define IRQ_NO_EXTI15_10						40
#define IRQ_NO_TIM8_BRK							43
#define IRQ_NO_TIM8_UP							44
#define IRQ_NO_TIM8_TRG_COM						45
#define IRQ_NO_TIM8_CC							46
#define IRQ_NO_ADC3								47
#define IRQ_NO_TIM5								50
#define IRQ_NO_SPI3								51
#define IRQ_NO_UART4							52
#define IRQ_NO_UART5							53
#define IRQ_NO_TIM6								54
#define IRQ_NO_TIM7								55


/*
 *  Priority of IRQ
 *  (The lower the value, the greater the priority of corresponding interrupt)
 */
#define NVIC_PRIOR_0							0
#define NVIC_PRIOR_1							1
#define NVIC_PRIOR_2							2
#define NVIC_PRIOR_3							3
#define NVIC_PRIOR_4							4
#define NVIC_PRIOR_5							5
#define NVIC_PRIOR_6							6
#define NVIC_PRIOR_7							7
#define NVIC_PRIOR_8							8
#define NVIC_PRIOR_9							9
#define NVIC_PRIOR_10							10
#define NVIC_PRIOR_11							11
#define NVIC_PRIOR_12							12
#define NVIC_PRIOR_13							13
#define NVIC_PRIOR_14							14
#define NVIC_PRIOR_15							15


#endif /* STM32F103XX_H_ */
