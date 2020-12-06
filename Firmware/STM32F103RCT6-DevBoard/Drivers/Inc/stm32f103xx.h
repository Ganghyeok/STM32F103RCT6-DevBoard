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
#include <string.h>


/**************************************************************************************************************
 * 																											  *
 * 													Generic Macros											  *
 * 																											  *
 **************************************************************************************************************/

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

#define UNUSED(X)								(void)X			/* To avoid gcc/g++ warnings */

/**************************************************************************************************************
 * 																											  *
 * 									Base Addresses of Memories, Bus, Peripherals							  *
 * 																											  *
 **************************************************************************************************************/

/* Memory mapping of Core Hardware */
#define SCS_BASE			(0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE			(0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE			(0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE			(0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE		(0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE		(SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE			(SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE			(SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

/* Memory mapping of Processor Hardware */
#define FLASH_BASE            0x08000000UL /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       0x0807FFFFUL /*!< FLASH END address of bank1 */
#define SRAM_BASE             0x20000000UL /*!< SRAM base address in the alias region */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          0x22000000UL /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region */

#define FSMC_BASE             0x60000000UL /*!< FSMC base address */
#define FSMC_R_BASE           0xA0000000UL /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x00000000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x00000400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x00000800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x00000C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x00001000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x00001400UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x00002800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x00003000UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x00003800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x00003C00UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x00004400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x00004800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x00004C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x00005000UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x00005800UL)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x00006400UL)
#define BKP_BASE              (APB1PERIPH_BASE + 0x00006C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x00007000UL)
#define DAC_BASE              (APB1PERIPH_BASE + 0x00007400UL)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x00000000UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x00000400UL)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x00001000UL)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x00001400UL)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x00001800UL)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x00001C00UL)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x00002000UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x00002400UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x00002800UL)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x00002C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x00003000UL)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x00003400UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x00003800UL)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x00003C00UL)

#define SDIO_BASE             (PERIPH_BASE + 0x00018000UL)

#define DMA1_BASE             (AHBPERIPH_BASE + 0x00000000UL)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x00000030UL)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x00000044UL)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x00000058UL)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x0000006CUL)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x00000080UL)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x00000400UL)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x00000408UL)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x0000041CUL)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x00000430UL)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x00000444UL)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x00000458UL)
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000UL)

#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000UL) /*!< Flash registers base address */
#define FLASHSIZE_BASE        0x1FFFF7E0UL    /*!< FLASH Size register base address */
#define UID_BASE              0x1FFFF7E8UL    /*!< Unique device ID register base address */
#define OB_BASE               0x1FFFF800UL    /*!< Flash Option Bytes base address */


#define FSMC_BANK1            (FSMC_BASE)               /*!< FSMC Bank1 base address */
#define FSMC_BANK1_1          (FSMC_BANK1)              /*!< FSMC Bank1_1 base address */
#define FSMC_BANK1_2          (FSMC_BANK1 + 0x04000000UL) /*!< FSMC Bank1_2 base address */
#define FSMC_BANK1_3          (FSMC_BANK1 + 0x08000000UL) /*!< FSMC Bank1_3 base address */
#define FSMC_BANK1_4          (FSMC_BANK1 + 0x0C000000UL) /*!< FSMC Bank1_4 base address */

#define FSMC_BANK2            (FSMC_BASE + 0x10000000UL)  /*!< FSMC Bank2 base address */
#define FSMC_BANK3            (FSMC_BASE + 0x20000000UL)  /*!< FSMC Bank3 base address */
#define FSMC_BANK4            (FSMC_BASE + 0x30000000UL)  /*!< FSMC Bank4 base address */

#define FSMC_BANK1_R_BASE     (FSMC_R_BASE + 0x00000000UL)    /*!< FSMC Bank1 registers base address */
#define FSMC_BANK1E_R_BASE    (FSMC_R_BASE + 0x00000104UL)    /*!< FSMC Bank1E registers base address */
#define FSMC_BANK2_3_R_BASE   (FSMC_R_BASE + 0x00000060UL)    /*!< FSMC Bank2/Bank3 registers base address */
#define FSMC_BANK4_R_BASE     (FSMC_R_BASE + 0x000000A0UL)    /*!< FSMC Bank4 registers base address */

#define DBGMCU_BASE          0xE0042000UL /*!< Debug MCU registers base address */

/* USB device FS */
#define USB_BASE              (APB1PERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APB1PERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */




/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Register Structures									  *
 * 																											  *
 **************************************************************************************************************/

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;


typedef struct
{
  __IO uint32_t SR;               /*!< ADC status register,    used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address         */
  __IO uint32_t CR1;              /*!< ADC control register 1, used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x04  */
  __IO uint32_t CR2;              /*!< ADC control register 2, used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x08  */
  uint32_t  RESERVED[16];
  __IO uint32_t DR;               /*!< ADC data register,      used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x4C  */
} ADC_Common_TypeDef;


/**
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;


/**
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;


/**
  * @brief Alternate Function I/O
  */
typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;
} AFIO_TypeDef;


/**
  * @brief External Interrupt/Event Controller
  */
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;


/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;


/**
  * @brief TIM Timers
  */
typedef struct
{
  __IO uint32_t CR1;             /*!< TIM control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;             /*!< TIM control register 2,                      Address offset: 0x04 */
  __IO uint32_t SMCR;            /*!< TIM slave Mode Control register,             Address offset: 0x08 */
  __IO uint32_t DIER;            /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  __IO uint32_t SR;              /*!< TIM status register,                         Address offset: 0x10 */
  __IO uint32_t EGR;             /*!< TIM event generation register,               Address offset: 0x14 */
  __IO uint32_t CCMR1;           /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
  __IO uint32_t CCMR2;           /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
  __IO uint32_t CCER;            /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  __IO uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */
  __IO uint32_t PSC;             /*!< TIM prescaler register,                      Address offset: 0x28 */
  __IO uint32_t ARR;             /*!< TIM auto-reload register,                    Address offset: 0x2C */
  __IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
  __IO uint32_t CCR1;            /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  __IO uint32_t CCR2;            /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  __IO uint32_t CCR3;            /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  __IO uint32_t CCR4;            /*!< TIM capture/compare register 4,              Address offset: 0x40 */
  __IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
  __IO uint32_t DCR;             /*!< TIM DMA control register,                    Address offset: 0x48 */
  __IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
  __IO uint32_t OR;              /*!< TIM option register,                         Address offset: 0x50 */
}TIM_TypeDef;


/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;


/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Declarations											  *
 * 							(Peripheral base addresses type casted to xxx_TypeDef *)						  *
 * 																											  *
 **************************************************************************************************************/

/* Peripheral Declarations of Processor Hardware */
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                ((TIM_TypeDef *)TIM4_BASE)
#define TIM5                ((TIM_TypeDef *)TIM5_BASE)
#define TIM6                ((TIM_TypeDef *)TIM6_BASE)
#define TIM7                ((TIM_TypeDef *)TIM7_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define WWDG                ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2                ((SPI_TypeDef *)SPI2_BASE)
#define SPI3                ((SPI_TypeDef *)SPI3_BASE)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define USART3              ((USART_TypeDef *)USART3_BASE)
#define UART4               ((USART_TypeDef *)UART4_BASE)
#define UART5               ((USART_TypeDef *)UART5_BASE)
#define I2C1                ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                ((I2C_TypeDef *)I2C2_BASE)
#define USB                 ((USB_TypeDef *)USB_BASE)
#define CAN1                ((CAN_TypeDef *)CAN1_BASE)
#define BKP                 ((BKP_TypeDef *)BKP_BASE)
#define PWR                 ((PWR_TypeDef *)PWR_BASE)
#define DAC1                ((DAC_TypeDef *)DAC_BASE)
#define DAC                 ((DAC_TypeDef *)DAC_BASE) /* Kept for legacy purpose */
#define AFIO                ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *)GPIOG_BASE)
#define ADC1                ((ADC_TypeDef *)ADC1_BASE)
#define ADC2                ((ADC_TypeDef *)ADC2_BASE)
#define ADC3                ((ADC_TypeDef *)ADC3_BASE)
#define ADC12_COMMON        ((ADC_Common_TypeDef *)ADC1_BASE)
#define TIM1                ((TIM_TypeDef *)TIM1_BASE)
#define SPI1                ((SPI_TypeDef *)SPI1_BASE)
#define TIM8                ((TIM_TypeDef *)TIM8_BASE)
#define USART1              ((USART_TypeDef *)USART1_BASE)
#define SDIO                ((SDIO_TypeDef *)SDIO_BASE)
#define DMA1                ((DMA_TypeDef *)DMA1_BASE)
#define DMA2                ((DMA_TypeDef *)DMA2_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *)DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *)DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *)DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *)DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *)DMA2_Channel5_BASE)
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define CRC                 ((CRC_TypeDef *)CRC_BASE)
#define FLASH               ((FLASH_TypeDef *)FLASH_R_BASE)
#define OB                  ((OB_TypeDef *)OB_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *)FSMC_BANK1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *)FSMC_BANK1E_R_BASE)
#define FSMC_Bank2_3        ((FSMC_Bank2_3_TypeDef *)FSMC_BANK2_3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *)FSMC_BANK4_R_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *)DBGMCU_BASE)


/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define FLASH_ACR_LATENCY_Pos               (0U)
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000007 */
#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk              /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_0                 (0x1UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000001 */
#define FLASH_ACR_LATENCY_1                 (0x2UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000002 */
#define FLASH_ACR_LATENCY_2                 (0x4UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000004 */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                     (0U)
#define RCC_CR_HSION_Msk                     (0x1UL << RCC_CR_HSION_Pos)        /*!< 0x00000001 */
#define RCC_CR_HSION                         RCC_CR_HSION_Msk                  /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos                    (1U)
#define RCC_CR_HSIRDY_Msk                    (0x1UL << RCC_CR_HSIRDY_Pos)       /*!< 0x00000002 */
#define RCC_CR_HSIRDY                        RCC_CR_HSIRDY_Msk                 /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_Pos                   (3U)
#define RCC_CR_HSITRIM_Msk                   (0x1FUL << RCC_CR_HSITRIM_Pos)     /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                       RCC_CR_HSITRIM_Msk                /*!< Internal High Speed clock trimming */
#define RCC_CR_HSICAL_Pos                    (8U)
#define RCC_CR_HSICAL_Msk                    (0xFFUL << RCC_CR_HSICAL_Pos)      /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                        RCC_CR_HSICAL_Msk                 /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSEON_Pos                     (16U)
#define RCC_CR_HSEON_Msk                     (0x1UL << RCC_CR_HSEON_Pos)        /*!< 0x00010000 */
#define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos                    (17U)
#define RCC_CR_HSERDY_Msk                    (0x1UL << RCC_CR_HSERDY_Pos)       /*!< 0x00020000 */
#define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP_Pos                    (18U)
#define RCC_CR_HSEBYP_Msk                    (0x1UL << RCC_CR_HSEBYP_Pos)       /*!< 0x00040000 */
#define RCC_CR_HSEBYP                        RCC_CR_HSEBYP_Msk                 /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON_Pos                     (19U)
#define RCC_CR_CSSON_Msk                     (0x1UL << RCC_CR_CSSON_Pos)        /*!< 0x00080000 */
#define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< Clock Security System enable */
#define RCC_CR_PLLON_Pos                     (24U)
#define RCC_CR_PLLON_Msk                     (0x1UL << RCC_CR_PLLON_Pos)        /*!< 0x01000000 */
#define RCC_CR_PLLON                         RCC_CR_PLLON_Msk                  /*!< PLL enable */
#define RCC_CR_PLLRDY_Pos                    (25U)
#define RCC_CR_PLLRDY_Msk                    (0x1UL << RCC_CR_PLLRDY_Pos)       /*!< 0x02000000 */
#define RCC_CR_PLLRDY                        RCC_CR_PLLRDY_Msk                 /*!< PLL clock ready flag */


/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                      (0U)
#define RCC_CFGR_SW_Msk                      (0x3UL << RCC_CFGR_SW_Pos)         /*!< 0x00000003 */
#define RCC_CFGR_SW                          RCC_CFGR_SW_Msk                   /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                        (0x1UL << RCC_CFGR_SW_Pos)         /*!< 0x00000001 */
#define RCC_CFGR_SW_1                        (0x2UL << RCC_CFGR_SW_Pos)         /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                      0x00000000U                       /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                      0x00000001U                       /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                      0x00000002U                       /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                     (2U)
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                  /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                       (0x1UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                       (0x2UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                    (4U)
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                      (0x1UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                      (0x2UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                      (0x4UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                      (0x8UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                   0x00000080U                       /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                   0x00000090U                       /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                   0x000000A0U                       /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                  0x000000B0U                       /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                  0x000000C0U                       /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                 0x000000D0U                       /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                 0x000000E0U                       /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                 0x000000F0U                       /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                     (0x1UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1                     (0x2UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2                     (0x4UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                  0x00000400U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                  0x00000500U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                  0x00000600U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                   (11U)
#define RCC_CFGR_PPRE2_Msk                   (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                     (0x1UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1                     (0x2UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2                     (0x4UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                  0x00002000U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                  0x00002800U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                  0x00003000U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                 0x00003800U                       /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define RCC_CFGR_ADCPRE_Pos                  (14U)
#define RCC_CFGR_ADCPRE_Msk                  (0x3UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x0000C000 */
#define RCC_CFGR_ADCPRE                      RCC_CFGR_ADCPRE_Msk               /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_ADCPRE_0                    (0x1UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00004000 */
#define RCC_CFGR_ADCPRE_1                    (0x2UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00008000 */

#define RCC_CFGR_ADCPRE_DIV2                 0x00000000U                       /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4                 0x00004000U                       /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC_Pos                  (16U)
#define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_Pos                (17U)
#define RCC_CFGR_PLLXTPRE_Msk                (0x1UL << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk             /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL_Pos                 (18U)
#define RCC_CFGR_PLLMULL_Msk                 (0xFUL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
#define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk              /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMULL_0                   (0x1UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL_1                   (0x2UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL_2                   (0x4UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL_3                   (0x8UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00200000 */

#define RCC_CFGR_PLLXTPRE_HSE                0x00000000U                      /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2           0x00020000U                      /*!< HSE clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMULL2                    0x00000000U                       /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3_Pos                (18U)
#define RCC_CFGR_PLLMULL3_Msk                (0x1UL << RCC_CFGR_PLLMULL3_Pos)   /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL3                    RCC_CFGR_PLLMULL3_Msk             /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4_Pos                (19U)
#define RCC_CFGR_PLLMULL4_Msk                (0x1UL << RCC_CFGR_PLLMULL4_Pos)   /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL4                    RCC_CFGR_PLLMULL4_Msk             /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5_Pos                (18U)
#define RCC_CFGR_PLLMULL5_Msk                (0x3UL << RCC_CFGR_PLLMULL5_Pos)   /*!< 0x000C0000 */
#define RCC_CFGR_PLLMULL5                    RCC_CFGR_PLLMULL5_Msk             /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6_Pos                (20U)
#define RCC_CFGR_PLLMULL6_Msk                (0x1UL << RCC_CFGR_PLLMULL6_Pos)   /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL6                    RCC_CFGR_PLLMULL6_Msk             /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7_Pos                (18U)
#define RCC_CFGR_PLLMULL7_Msk                (0x5UL << RCC_CFGR_PLLMULL7_Pos)   /*!< 0x00140000 */
#define RCC_CFGR_PLLMULL7                    RCC_CFGR_PLLMULL7_Msk             /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8_Pos                (19U)
#define RCC_CFGR_PLLMULL8_Msk                (0x3UL << RCC_CFGR_PLLMULL8_Pos)   /*!< 0x00180000 */
#define RCC_CFGR_PLLMULL8                    RCC_CFGR_PLLMULL8_Msk             /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9_Pos                (18U)
#define RCC_CFGR_PLLMULL9_Msk                (0x7UL << RCC_CFGR_PLLMULL9_Pos)   /*!< 0x001C0000 */
#define RCC_CFGR_PLLMULL9                    RCC_CFGR_PLLMULL9_Msk             /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10_Pos               (21U)
#define RCC_CFGR_PLLMULL10_Msk               (0x1UL << RCC_CFGR_PLLMULL10_Pos)  /*!< 0x00200000 */
#define RCC_CFGR_PLLMULL10                   RCC_CFGR_PLLMULL10_Msk            /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11_Pos               (18U)
#define RCC_CFGR_PLLMULL11_Msk               (0x9UL << RCC_CFGR_PLLMULL11_Pos)  /*!< 0x00240000 */
#define RCC_CFGR_PLLMULL11                   RCC_CFGR_PLLMULL11_Msk            /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12_Pos               (19U)
#define RCC_CFGR_PLLMULL12_Msk               (0x5UL << RCC_CFGR_PLLMULL12_Pos)  /*!< 0x00280000 */
#define RCC_CFGR_PLLMULL12                   RCC_CFGR_PLLMULL12_Msk            /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13_Pos               (18U)
#define RCC_CFGR_PLLMULL13_Msk               (0xBUL << RCC_CFGR_PLLMULL13_Pos)  /*!< 0x002C0000 */
#define RCC_CFGR_PLLMULL13                   RCC_CFGR_PLLMULL13_Msk            /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14_Pos               (20U)
#define RCC_CFGR_PLLMULL14_Msk               (0x3UL << RCC_CFGR_PLLMULL14_Pos)  /*!< 0x00300000 */
#define RCC_CFGR_PLLMULL14                   RCC_CFGR_PLLMULL14_Msk            /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15_Pos               (18U)
#define RCC_CFGR_PLLMULL15_Msk               (0xDUL << RCC_CFGR_PLLMULL15_Pos)  /*!< 0x00340000 */
#define RCC_CFGR_PLLMULL15                   RCC_CFGR_PLLMULL15_Msk            /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16_Pos               (19U)
#define RCC_CFGR_PLLMULL16_Msk               (0x7UL << RCC_CFGR_PLLMULL16_Pos)  /*!< 0x00380000 */
#define RCC_CFGR_PLLMULL16                   RCC_CFGR_PLLMULL16_Msk            /*!< PLL input clock*16 */
#define RCC_CFGR_USBPRE_Pos                  (22U)
#define RCC_CFGR_USBPRE_Msk                  (0x1UL << RCC_CFGR_USBPRE_Pos)     /*!< 0x00400000 */
#define RCC_CFGR_USBPRE                      RCC_CFGR_USBPRE_Msk               /*!< USB Device prescaler */

/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos                     (24U)
#define RCC_CFGR_MCO_Msk                     (0x7UL << RCC_CFGR_MCO_Pos)        /*!< 0x07000000 */
#define RCC_CFGR_MCO                         RCC_CFGR_MCO_Msk                  /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0                       (0x1UL << RCC_CFGR_MCO_Pos)        /*!< 0x01000000 */
#define RCC_CFGR_MCO_1                       (0x2UL << RCC_CFGR_MCO_Pos)        /*!< 0x02000000 */
#define RCC_CFGR_MCO_2                       (0x4UL << RCC_CFGR_MCO_Pos)        /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK                 0x00000000U                        /*!< No clock */
#define RCC_CFGR_MCO_SYSCLK                  0x04000000U                        /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI                     0x05000000U                        /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE                     0x06000000U                        /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLLCLK_DIV2             0x07000000U                        /*!< PLL clock divided by 2 selected as MCO source */

 /* Reference defines */
 #define RCC_CFGR_MCOSEL                      RCC_CFGR_MCO
 #define RCC_CFGR_MCOSEL_0                    RCC_CFGR_MCO_0
 #define RCC_CFGR_MCOSEL_1                    RCC_CFGR_MCO_1
 #define RCC_CFGR_MCOSEL_2                    RCC_CFGR_MCO_2
 #define RCC_CFGR_MCOSEL_NOCLOCK              RCC_CFGR_MCO_NOCLOCK
 #define RCC_CFGR_MCOSEL_SYSCLK               RCC_CFGR_MCO_SYSCLK
 #define RCC_CFGR_MCOSEL_HSI                  RCC_CFGR_MCO_HSI
 #define RCC_CFGR_MCOSEL_HSE                  RCC_CFGR_MCO_HSE
 #define RCC_CFGR_MCOSEL_PLL_DIV2             RCC_CFGR_MCO_PLLCLK_DIV2

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define RCC_CIR_LSIRDYF_Pos                  (0U)
#define RCC_CIR_LSIRDYF_Msk                  (0x1UL << RCC_CIR_LSIRDYF_Pos)     /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                      RCC_CIR_LSIRDYF_Msk               /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF_Pos                  (1U)
#define RCC_CIR_LSERDYF_Msk                  (0x1UL << RCC_CIR_LSERDYF_Pos)     /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                      RCC_CIR_LSERDYF_Msk               /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF_Pos                  (2U)
#define RCC_CIR_HSIRDYF_Msk                  (0x1UL << RCC_CIR_HSIRDYF_Pos)     /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                      RCC_CIR_HSIRDYF_Msk               /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF_Pos                  (3U)
#define RCC_CIR_HSERDYF_Msk                  (0x1UL << RCC_CIR_HSERDYF_Pos)     /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                      RCC_CIR_HSERDYF_Msk               /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF_Pos                  (4U)
#define RCC_CIR_PLLRDYF_Msk                  (0x1UL << RCC_CIR_PLLRDYF_Pos)     /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                      RCC_CIR_PLLRDYF_Msk               /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF_Pos                     (7U)
#define RCC_CIR_CSSF_Msk                     (0x1UL << RCC_CIR_CSSF_Pos)        /*!< 0x00000080 */
#define RCC_CIR_CSSF                         RCC_CIR_CSSF_Msk                  /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE_Pos                 (8U)
#define RCC_CIR_LSIRDYIE_Msk                 (0x1UL << RCC_CIR_LSIRDYIE_Pos)    /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                     RCC_CIR_LSIRDYIE_Msk              /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE_Pos                 (9U)
#define RCC_CIR_LSERDYIE_Msk                 (0x1UL << RCC_CIR_LSERDYIE_Pos)    /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                     RCC_CIR_LSERDYIE_Msk              /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE_Pos                 (10U)
#define RCC_CIR_HSIRDYIE_Msk                 (0x1UL << RCC_CIR_HSIRDYIE_Pos)    /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                     RCC_CIR_HSIRDYIE_Msk              /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE_Pos                 (11U)
#define RCC_CIR_HSERDYIE_Msk                 (0x1UL << RCC_CIR_HSERDYIE_Pos)    /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                     RCC_CIR_HSERDYIE_Msk              /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE_Pos                 (12U)
#define RCC_CIR_PLLRDYIE_Msk                 (0x1UL << RCC_CIR_PLLRDYIE_Pos)    /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                     RCC_CIR_PLLRDYIE_Msk              /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC_Pos                  (16U)
#define RCC_CIR_LSIRDYC_Msk                  (0x1UL << RCC_CIR_LSIRDYC_Pos)     /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                      RCC_CIR_LSIRDYC_Msk               /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC_Pos                  (17U)
#define RCC_CIR_LSERDYC_Msk                  (0x1UL << RCC_CIR_LSERDYC_Pos)     /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                      RCC_CIR_LSERDYC_Msk               /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC_Pos                  (18U)
#define RCC_CIR_HSIRDYC_Msk                  (0x1UL << RCC_CIR_HSIRDYC_Pos)     /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                      RCC_CIR_HSIRDYC_Msk               /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC_Pos                  (19U)
#define RCC_CIR_HSERDYC_Msk                  (0x1UL << RCC_CIR_HSERDYC_Pos)     /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                      RCC_CIR_HSERDYC_Msk               /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC_Pos                  (20U)
#define RCC_CIR_PLLRDYC_Msk                  (0x1UL << RCC_CIR_PLLRDYC_Pos)     /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                      RCC_CIR_PLLRDYC_Msk               /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC_Pos                     (23U)
#define RCC_CIR_CSSC_Msk                     (0x1UL << RCC_CIR_CSSC_Pos)        /*!< 0x00800000 */
#define RCC_CIR_CSSC                         RCC_CIR_CSSC_Msk                  /*!< Clock Security System Interrupt Clear */


/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define RCC_APB2RSTR_AFIORST_Pos             (0U)
#define RCC_APB2RSTR_AFIORST_Msk             (0x1UL << RCC_APB2RSTR_AFIORST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_AFIORST                 RCC_APB2RSTR_AFIORST_Msk          /*!< Alternate Function I/O reset */
#define RCC_APB2RSTR_IOPARST_Pos             (2U)
#define RCC_APB2RSTR_IOPARST_Msk             (0x1UL << RCC_APB2RSTR_IOPARST_Pos) /*!< 0x00000004 */
#define RCC_APB2RSTR_IOPARST                 RCC_APB2RSTR_IOPARST_Msk          /*!< I/O port A reset */
#define RCC_APB2RSTR_IOPBRST_Pos             (3U)
#define RCC_APB2RSTR_IOPBRST_Msk             (0x1UL << RCC_APB2RSTR_IOPBRST_Pos) /*!< 0x00000008 */
#define RCC_APB2RSTR_IOPBRST                 RCC_APB2RSTR_IOPBRST_Msk          /*!< I/O port B reset */
#define RCC_APB2RSTR_IOPCRST_Pos             (4U)
#define RCC_APB2RSTR_IOPCRST_Msk             (0x1UL << RCC_APB2RSTR_IOPCRST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_IOPCRST                 RCC_APB2RSTR_IOPCRST_Msk          /*!< I/O port C reset */
#define RCC_APB2RSTR_IOPDRST_Pos             (5U)
#define RCC_APB2RSTR_IOPDRST_Msk             (0x1UL << RCC_APB2RSTR_IOPDRST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_IOPDRST                 RCC_APB2RSTR_IOPDRST_Msk          /*!< I/O port D reset */
#define RCC_APB2RSTR_ADC1RST_Pos             (9U)
#define RCC_APB2RSTR_ADC1RST_Msk             (0x1UL << RCC_APB2RSTR_ADC1RST_Pos) /*!< 0x00000200 */
#define RCC_APB2RSTR_ADC1RST                 RCC_APB2RSTR_ADC1RST_Msk          /*!< ADC 1 interface reset */

#define RCC_APB2RSTR_ADC2RST_Pos             (10U)
#define RCC_APB2RSTR_ADC2RST_Msk             (0x1UL << RCC_APB2RSTR_ADC2RST_Pos) /*!< 0x00000400 */
#define RCC_APB2RSTR_ADC2RST                 RCC_APB2RSTR_ADC2RST_Msk          /*!< ADC 2 interface reset */

#define RCC_APB2RSTR_TIM1RST_Pos             (11U)
#define RCC_APB2RSTR_TIM1RST_Msk             (0x1UL << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_TIM1RST                 RCC_APB2RSTR_TIM1RST_Msk          /*!< TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST_Pos             (12U)
#define RCC_APB2RSTR_SPI1RST_Msk             (0x1UL << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST                 RCC_APB2RSTR_SPI1RST_Msk          /*!< SPI 1 reset */
#define RCC_APB2RSTR_USART1RST_Pos           (14U)
#define RCC_APB2RSTR_USART1RST_Msk           (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST               RCC_APB2RSTR_USART1RST_Msk        /*!< USART1 reset */


#define RCC_APB2RSTR_IOPERST_Pos             (6U)
#define RCC_APB2RSTR_IOPERST_Msk             (0x1UL << RCC_APB2RSTR_IOPERST_Pos) /*!< 0x00000040 */
#define RCC_APB2RSTR_IOPERST                 RCC_APB2RSTR_IOPERST_Msk          /*!< I/O port E reset */

#define RCC_APB2RSTR_IOPFRST_Pos             (7U)
#define RCC_APB2RSTR_IOPFRST_Msk             (0x1UL << RCC_APB2RSTR_IOPFRST_Pos) /*!< 0x00000080 */
#define RCC_APB2RSTR_IOPFRST                 RCC_APB2RSTR_IOPFRST_Msk          /*!< I/O port F reset */
#define RCC_APB2RSTR_IOPGRST_Pos             (8U)
#define RCC_APB2RSTR_IOPGRST_Msk             (0x1UL << RCC_APB2RSTR_IOPGRST_Pos) /*!< 0x00000100 */
#define RCC_APB2RSTR_IOPGRST                 RCC_APB2RSTR_IOPGRST_Msk          /*!< I/O port G reset */
#define RCC_APB2RSTR_TIM8RST_Pos             (13U)
#define RCC_APB2RSTR_TIM8RST_Msk             (0x1UL << RCC_APB2RSTR_TIM8RST_Pos) /*!< 0x00002000 */
#define RCC_APB2RSTR_TIM8RST                 RCC_APB2RSTR_TIM8RST_Msk          /*!< TIM8 Timer reset */
#define RCC_APB2RSTR_ADC3RST_Pos             (15U)
#define RCC_APB2RSTR_ADC3RST_Msk             (0x1UL << RCC_APB2RSTR_ADC3RST_Pos) /*!< 0x00008000 */
#define RCC_APB2RSTR_ADC3RST                 RCC_APB2RSTR_ADC3RST_Msk          /*!< ADC3 interface reset */



/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define RCC_APB1RSTR_TIM2RST_Pos             (0U)
#define RCC_APB1RSTR_TIM2RST_Msk             (0x1UL << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                 RCC_APB1RSTR_TIM2RST_Msk          /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST_Pos             (1U)
#define RCC_APB1RSTR_TIM3RST_Msk             (0x1UL << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST                 RCC_APB1RSTR_TIM3RST_Msk          /*!< Timer 3 reset */
#define RCC_APB1RSTR_WWDGRST_Pos             (11U)
#define RCC_APB1RSTR_WWDGRST_Msk             (0x1UL << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                 RCC_APB1RSTR_WWDGRST_Msk          /*!< Window Watchdog reset */
#define RCC_APB1RSTR_USART2RST_Pos           (17U)
#define RCC_APB1RSTR_USART2RST_Msk           (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST               RCC_APB1RSTR_USART2RST_Msk        /*!< USART 2 reset */
#define RCC_APB1RSTR_I2C1RST_Pos             (21U)
#define RCC_APB1RSTR_I2C1RST_Msk             (0x1UL << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                 RCC_APB1RSTR_I2C1RST_Msk          /*!< I2C 1 reset */

#define RCC_APB1RSTR_CAN1RST_Pos             (25U)
#define RCC_APB1RSTR_CAN1RST_Msk             (0x1UL << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST                 RCC_APB1RSTR_CAN1RST_Msk          /*!< CAN1 reset */

#define RCC_APB1RSTR_BKPRST_Pos              (27U)
#define RCC_APB1RSTR_BKPRST_Msk              (0x1UL << RCC_APB1RSTR_BKPRST_Pos) /*!< 0x08000000 */
#define RCC_APB1RSTR_BKPRST                  RCC_APB1RSTR_BKPRST_Msk           /*!< Backup interface reset */
#define RCC_APB1RSTR_PWRRST_Pos              (28U)
#define RCC_APB1RSTR_PWRRST_Msk              (0x1UL << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                  RCC_APB1RSTR_PWRRST_Msk           /*!< Power interface reset */

#define RCC_APB1RSTR_TIM4RST_Pos             (2U)
#define RCC_APB1RSTR_TIM4RST_Msk             (0x1UL << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST                 RCC_APB1RSTR_TIM4RST_Msk          /*!< Timer 4 reset */
#define RCC_APB1RSTR_SPI2RST_Pos             (14U)
#define RCC_APB1RSTR_SPI2RST_Msk             (0x1UL << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST                 RCC_APB1RSTR_SPI2RST_Msk          /*!< SPI 2 reset */
#define RCC_APB1RSTR_USART3RST_Pos           (18U)
#define RCC_APB1RSTR_USART3RST_Msk           (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST               RCC_APB1RSTR_USART3RST_Msk        /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C2RST_Pos             (22U)
#define RCC_APB1RSTR_I2C2RST_Msk             (0x1UL << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST                 RCC_APB1RSTR_I2C2RST_Msk          /*!< I2C 2 reset */

#define RCC_APB1RSTR_USBRST_Pos              (23U)
#define RCC_APB1RSTR_USBRST_Msk              (0x1UL << RCC_APB1RSTR_USBRST_Pos) /*!< 0x00800000 */
#define RCC_APB1RSTR_USBRST                  RCC_APB1RSTR_USBRST_Msk           /*!< USB Device reset */

#define RCC_APB1RSTR_TIM5RST_Pos             (3U)
#define RCC_APB1RSTR_TIM5RST_Msk             (0x1UL << RCC_APB1RSTR_TIM5RST_Pos) /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST                 RCC_APB1RSTR_TIM5RST_Msk          /*!< Timer 5 reset */
#define RCC_APB1RSTR_TIM6RST_Pos             (4U)
#define RCC_APB1RSTR_TIM6RST_Msk             (0x1UL << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST                 RCC_APB1RSTR_TIM6RST_Msk          /*!< Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST_Pos             (5U)
#define RCC_APB1RSTR_TIM7RST_Msk             (0x1UL << RCC_APB1RSTR_TIM7RST_Pos) /*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST                 RCC_APB1RSTR_TIM7RST_Msk          /*!< Timer 7 reset */
#define RCC_APB1RSTR_SPI3RST_Pos             (15U)
#define RCC_APB1RSTR_SPI3RST_Msk             (0x1UL << RCC_APB1RSTR_SPI3RST_Pos) /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST                 RCC_APB1RSTR_SPI3RST_Msk          /*!< SPI 3 reset */
#define RCC_APB1RSTR_UART4RST_Pos            (19U)
#define RCC_APB1RSTR_UART4RST_Msk            (0x1UL << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
#define RCC_APB1RSTR_UART4RST                RCC_APB1RSTR_UART4RST_Msk         /*!< UART 4 reset */
#define RCC_APB1RSTR_UART5RST_Pos            (20U)
#define RCC_APB1RSTR_UART5RST_Msk            (0x1UL << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
#define RCC_APB1RSTR_UART5RST                RCC_APB1RSTR_UART5RST_Msk         /*!< UART 5 reset */




#define RCC_APB1RSTR_DACRST_Pos              (29U)
#define RCC_APB1RSTR_DACRST_Msk              (0x1UL << RCC_APB1RSTR_DACRST_Pos) /*!< 0x20000000 */
#define RCC_APB1RSTR_DACRST                  RCC_APB1RSTR_DACRST_Msk           /*!< DAC interface reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define RCC_AHBENR_DMA1EN_Pos                (0U)
#define RCC_AHBENR_DMA1EN_Msk                (0x1UL << RCC_AHBENR_DMA1EN_Pos)   /*!< 0x00000001 */
#define RCC_AHBENR_DMA1EN                    RCC_AHBENR_DMA1EN_Msk             /*!< DMA1 clock enable */
#define RCC_AHBENR_SRAMEN_Pos                (2U)
#define RCC_AHBENR_SRAMEN_Msk                (0x1UL << RCC_AHBENR_SRAMEN_Pos)   /*!< 0x00000004 */
#define RCC_AHBENR_SRAMEN                    RCC_AHBENR_SRAMEN_Msk             /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos               (4U)
#define RCC_AHBENR_FLITFEN_Msk               (0x1UL << RCC_AHBENR_FLITFEN_Pos)  /*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN                   RCC_AHBENR_FLITFEN_Msk            /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos                 (6U)
#define RCC_AHBENR_CRCEN_Msk                 (0x1UL << RCC_AHBENR_CRCEN_Pos)    /*!< 0x00000040 */
#define RCC_AHBENR_CRCEN                     RCC_AHBENR_CRCEN_Msk              /*!< CRC clock enable */

#define RCC_AHBENR_DMA2EN_Pos                (1U)
#define RCC_AHBENR_DMA2EN_Msk                (0x1UL << RCC_AHBENR_DMA2EN_Pos)   /*!< 0x00000002 */
#define RCC_AHBENR_DMA2EN                    RCC_AHBENR_DMA2EN_Msk             /*!< DMA2 clock enable */

#define RCC_AHBENR_FSMCEN_Pos                (8U)
#define RCC_AHBENR_FSMCEN_Msk                (0x1UL << RCC_AHBENR_FSMCEN_Pos)   /*!< 0x00000100 */
#define RCC_AHBENR_FSMCEN                    RCC_AHBENR_FSMCEN_Msk             /*!< FSMC clock enable */
#define RCC_AHBENR_SDIOEN_Pos                (10U)
#define RCC_AHBENR_SDIOEN_Msk                (0x1UL << RCC_AHBENR_SDIOEN_Pos)   /*!< 0x00000400 */
#define RCC_AHBENR_SDIOEN                    RCC_AHBENR_SDIOEN_Msk             /*!< SDIO clock enable */


/******************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_AFIOEN_Pos               (0U)
#define RCC_APB2ENR_AFIOEN_Msk               (0x1UL << RCC_APB2ENR_AFIOEN_Pos)  /*!< 0x00000001 */
#define RCC_APB2ENR_AFIOEN                   RCC_APB2ENR_AFIOEN_Msk            /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN_Pos               (2U)
#define RCC_APB2ENR_IOPAEN_Msk               (0x1UL << RCC_APB2ENR_IOPAEN_Pos)  /*!< 0x00000004 */
#define RCC_APB2ENR_IOPAEN                   RCC_APB2ENR_IOPAEN_Msk            /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN_Pos               (3U)
#define RCC_APB2ENR_IOPBEN_Msk               (0x1UL << RCC_APB2ENR_IOPBEN_Pos)  /*!< 0x00000008 */
#define RCC_APB2ENR_IOPBEN                   RCC_APB2ENR_IOPBEN_Msk            /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN_Pos               (4U)
#define RCC_APB2ENR_IOPCEN_Msk               (0x1UL << RCC_APB2ENR_IOPCEN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_IOPCEN                   RCC_APB2ENR_IOPCEN_Msk            /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN_Pos               (5U)
#define RCC_APB2ENR_IOPDEN_Msk               (0x1UL << RCC_APB2ENR_IOPDEN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_IOPDEN                   RCC_APB2ENR_IOPDEN_Msk            /*!< I/O port D clock enable */
#define RCC_APB2ENR_ADC1EN_Pos               (9U)
#define RCC_APB2ENR_ADC1EN_Msk               (0x1UL << RCC_APB2ENR_ADC1EN_Pos)  /*!< 0x00000200 */
#define RCC_APB2ENR_ADC1EN                   RCC_APB2ENR_ADC1EN_Msk            /*!< ADC 1 interface clock enable */

#define RCC_APB2ENR_ADC2EN_Pos               (10U)
#define RCC_APB2ENR_ADC2EN_Msk               (0x1UL << RCC_APB2ENR_ADC2EN_Pos)  /*!< 0x00000400 */
#define RCC_APB2ENR_ADC2EN                   RCC_APB2ENR_ADC2EN_Msk            /*!< ADC 2 interface clock enable */

#define RCC_APB2ENR_TIM1EN_Pos               (11U)
#define RCC_APB2ENR_TIM1EN_Msk               (0x1UL << RCC_APB2ENR_TIM1EN_Pos)  /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN                   RCC_APB2ENR_TIM1EN_Msk            /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN_Pos               (12U)
#define RCC_APB2ENR_SPI1EN_Msk               (0x1UL << RCC_APB2ENR_SPI1EN_Pos)  /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                   RCC_APB2ENR_SPI1EN_Msk            /*!< SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos             (14U)
#define RCC_APB2ENR_USART1EN_Msk             (0x1UL << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                 RCC_APB2ENR_USART1EN_Msk          /*!< USART1 clock enable */


#define RCC_APB2ENR_IOPEEN_Pos               (6U)
#define RCC_APB2ENR_IOPEEN_Msk               (0x1UL << RCC_APB2ENR_IOPEEN_Pos)  /*!< 0x00000040 */
#define RCC_APB2ENR_IOPEEN                   RCC_APB2ENR_IOPEEN_Msk            /*!< I/O port E clock enable */

#define RCC_APB2ENR_IOPFEN_Pos               (7U)
#define RCC_APB2ENR_IOPFEN_Msk               (0x1UL << RCC_APB2ENR_IOPFEN_Pos)  /*!< 0x00000080 */
#define RCC_APB2ENR_IOPFEN                   RCC_APB2ENR_IOPFEN_Msk            /*!< I/O port F clock enable */
#define RCC_APB2ENR_IOPGEN_Pos               (8U)
#define RCC_APB2ENR_IOPGEN_Msk               (0x1UL << RCC_APB2ENR_IOPGEN_Pos)  /*!< 0x00000100 */
#define RCC_APB2ENR_IOPGEN                   RCC_APB2ENR_IOPGEN_Msk            /*!< I/O port G clock enable */
#define RCC_APB2ENR_TIM8EN_Pos               (13U)
#define RCC_APB2ENR_TIM8EN_Msk               (0x1UL << RCC_APB2ENR_TIM8EN_Pos)  /*!< 0x00002000 */
#define RCC_APB2ENR_TIM8EN                   RCC_APB2ENR_TIM8EN_Msk            /*!< TIM8 Timer clock enable */
#define RCC_APB2ENR_ADC3EN_Pos               (15U)
#define RCC_APB2ENR_ADC3EN_Msk               (0x1UL << RCC_APB2ENR_ADC3EN_Pos)  /*!< 0x00008000 */
#define RCC_APB2ENR_ADC3EN                   RCC_APB2ENR_ADC3EN_Msk            /*!< DMA1 clock enable */



/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define RCC_APB1ENR_TIM2EN_Pos               (0U)
#define RCC_APB1ENR_TIM2EN_Msk               (0x1UL << RCC_APB1ENR_TIM2EN_Pos)  /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                   RCC_APB1ENR_TIM2EN_Msk            /*!< Timer 2 clock enabled*/
#define RCC_APB1ENR_TIM3EN_Pos               (1U)
#define RCC_APB1ENR_TIM3EN_Msk               (0x1UL << RCC_APB1ENR_TIM3EN_Pos)  /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                   RCC_APB1ENR_TIM3EN_Msk            /*!< Timer 3 clock enable */
#define RCC_APB1ENR_WWDGEN_Pos               (11U)
#define RCC_APB1ENR_WWDGEN_Msk               (0x1UL << RCC_APB1ENR_WWDGEN_Pos)  /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                   RCC_APB1ENR_WWDGEN_Msk            /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_USART2EN_Pos             (17U)
#define RCC_APB1ENR_USART2EN_Msk             (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN                 RCC_APB1ENR_USART2EN_Msk          /*!< USART 2 clock enable */
#define RCC_APB1ENR_I2C1EN_Pos               (21U)
#define RCC_APB1ENR_I2C1EN_Msk               (0x1UL << RCC_APB1ENR_I2C1EN_Pos)  /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                   RCC_APB1ENR_I2C1EN_Msk            /*!< I2C 1 clock enable */

#define RCC_APB1ENR_CAN1EN_Pos               (25U)
#define RCC_APB1ENR_CAN1EN_Msk               (0x1UL << RCC_APB1ENR_CAN1EN_Pos)  /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN                   RCC_APB1ENR_CAN1EN_Msk            /*!< CAN1 clock enable */

#define RCC_APB1ENR_BKPEN_Pos                (27U)
#define RCC_APB1ENR_BKPEN_Msk                (0x1UL << RCC_APB1ENR_BKPEN_Pos)   /*!< 0x08000000 */
#define RCC_APB1ENR_BKPEN                    RCC_APB1ENR_BKPEN_Msk             /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN_Pos                (28U)
#define RCC_APB1ENR_PWREN_Msk                (0x1UL << RCC_APB1ENR_PWREN_Pos)   /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                    RCC_APB1ENR_PWREN_Msk             /*!< Power interface clock enable */

#define RCC_APB1ENR_TIM4EN_Pos               (2U)
#define RCC_APB1ENR_TIM4EN_Msk               (0x1UL << RCC_APB1ENR_TIM4EN_Pos)  /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                   RCC_APB1ENR_TIM4EN_Msk            /*!< Timer 4 clock enable */
#define RCC_APB1ENR_SPI2EN_Pos               (14U)
#define RCC_APB1ENR_SPI2EN_Msk               (0x1UL << RCC_APB1ENR_SPI2EN_Pos)  /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                   RCC_APB1ENR_SPI2EN_Msk            /*!< SPI 2 clock enable */
#define RCC_APB1ENR_USART3EN_Pos             (18U)
#define RCC_APB1ENR_USART3EN_Msk             (0x1UL << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN                 RCC_APB1ENR_USART3EN_Msk          /*!< USART 3 clock enable */
#define RCC_APB1ENR_I2C2EN_Pos               (22U)
#define RCC_APB1ENR_I2C2EN_Msk               (0x1UL << RCC_APB1ENR_I2C2EN_Pos)  /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                   RCC_APB1ENR_I2C2EN_Msk            /*!< I2C 2 clock enable */

#define RCC_APB1ENR_USBEN_Pos                (23U)
#define RCC_APB1ENR_USBEN_Msk                (0x1UL << RCC_APB1ENR_USBEN_Pos)   /*!< 0x00800000 */
#define RCC_APB1ENR_USBEN                    RCC_APB1ENR_USBEN_Msk             /*!< USB Device clock enable */

#define RCC_APB1ENR_TIM5EN_Pos               (3U)
#define RCC_APB1ENR_TIM5EN_Msk               (0x1UL << RCC_APB1ENR_TIM5EN_Pos)  /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN                   RCC_APB1ENR_TIM5EN_Msk            /*!< Timer 5 clock enable */
#define RCC_APB1ENR_TIM6EN_Pos               (4U)
#define RCC_APB1ENR_TIM6EN_Msk               (0x1UL << RCC_APB1ENR_TIM6EN_Pos)  /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                   RCC_APB1ENR_TIM6EN_Msk            /*!< Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN_Pos               (5U)
#define RCC_APB1ENR_TIM7EN_Msk               (0x1UL << RCC_APB1ENR_TIM7EN_Pos)  /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN                   RCC_APB1ENR_TIM7EN_Msk            /*!< Timer 7 clock enable */
#define RCC_APB1ENR_SPI3EN_Pos               (15U)
#define RCC_APB1ENR_SPI3EN_Msk               (0x1UL << RCC_APB1ENR_SPI3EN_Pos)  /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                   RCC_APB1ENR_SPI3EN_Msk            /*!< SPI 3 clock enable */
#define RCC_APB1ENR_UART4EN_Pos              (19U)
#define RCC_APB1ENR_UART4EN_Msk              (0x1UL << RCC_APB1ENR_UART4EN_Pos) /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN                  RCC_APB1ENR_UART4EN_Msk           /*!< UART 4 clock enable */
#define RCC_APB1ENR_UART5EN_Pos              (20U)
#define RCC_APB1ENR_UART5EN_Msk              (0x1UL << RCC_APB1ENR_UART5EN_Pos) /*!< 0x00100000 */
#define RCC_APB1ENR_UART5EN                  RCC_APB1ENR_UART5EN_Msk           /*!< UART 5 clock enable */




#define RCC_APB1ENR_DACEN_Pos                (29U)
#define RCC_APB1ENR_DACEN_Msk                (0x1UL << RCC_APB1ENR_DACEN_Pos)   /*!< 0x20000000 */
#define RCC_APB1ENR_DACEN                    RCC_APB1ENR_DACEN_Msk             /*!< DAC interface clock enable */

/*******************  Bit definition for RCC_BDCR register  *******************/
#define RCC_BDCR_LSEON_Pos                   (0U)
#define RCC_BDCR_LSEON_Msk                   (0x1UL << RCC_BDCR_LSEON_Pos)      /*!< 0x00000001 */
#define RCC_BDCR_LSEON                       RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos                  (1U)
#define RCC_BDCR_LSERDY_Msk                  (0x1UL << RCC_BDCR_LSERDY_Pos)     /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                      RCC_BDCR_LSERDY_Msk               /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos                  (2U)
#define RCC_BDCR_LSEBYP_Msk                  (0x1UL << RCC_BDCR_LSEBYP_Pos)     /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                      RCC_BDCR_LSEBYP_Msk               /*!< External Low Speed oscillator Bypass */

#define RCC_BDCR_RTCSEL_Pos                  (8U)
#define RCC_BDCR_RTCSEL_Msk                  (0x3UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                      RCC_BDCR_RTCSEL_Msk               /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_RTCSEL_0                    (0x1UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                    (0x2UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000200 */

/*!< RTC congiguration */
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000U                       /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE                  0x00000100U                       /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI                  0x00000200U                       /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE                  0x00000300U                       /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN_Pos                   (15U)
#define RCC_BDCR_RTCEN_Msk                   (0x1UL << RCC_BDCR_RTCEN_Pos)      /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                       RCC_BDCR_RTCEN_Msk                /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos                   (16U)
#define RCC_BDCR_BDRST_Msk                   (0x1UL << RCC_BDCR_BDRST_Pos)      /*!< 0x00010000 */
#define RCC_BDCR_BDRST                       RCC_BDCR_BDRST_Msk                /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/
#define RCC_CSR_LSION_Pos                    (0U)
#define RCC_CSR_LSION_Msk                    (0x1UL << RCC_CSR_LSION_Pos)       /*!< 0x00000001 */
#define RCC_CSR_LSION                        RCC_CSR_LSION_Msk                 /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos                   (1U)
#define RCC_CSR_LSIRDY_Msk                   (0x1UL << RCC_CSR_LSIRDY_Pos)      /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                       RCC_CSR_LSIRDY_Msk                /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF_Pos                     (24U)
#define RCC_CSR_RMVF_Msk                     (0x1UL << RCC_CSR_RMVF_Pos)        /*!< 0x01000000 */
#define RCC_CSR_RMVF                         RCC_CSR_RMVF_Msk                  /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos                  (26U)
#define RCC_CSR_PINRSTF_Msk                  (0x1UL << RCC_CSR_PINRSTF_Pos)     /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                      RCC_CSR_PINRSTF_Msk               /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                  (27U)
#define RCC_CSR_PORRSTF_Msk                  (0x1UL << RCC_CSR_PORRSTF_Pos)     /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                      RCC_CSR_PORRSTF_Msk               /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                  (28U)
#define RCC_CSR_SFTRSTF_Msk                  (0x1UL << RCC_CSR_SFTRSTF_Pos)     /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                      RCC_CSR_SFTRSTF_Msk               /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                 (29U)
#define RCC_CSR_IWDGRSTF_Msk                 (0x1UL << RCC_CSR_IWDGRSTF_Pos)    /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                     RCC_CSR_IWDGRSTF_Msk              /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                 (30U)
#define RCC_CSR_WWDGRSTF_Msk                 (0x1UL << RCC_CSR_WWDGRSTF_Pos)    /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                     RCC_CSR_WWDGRSTF_Msk              /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos                 (31U)
#define RCC_CSR_LPWRRSTF_Msk                 (0x1UL << RCC_CSR_LPWRRSTF_Pos)    /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                     RCC_CSR_LPWRRSTF_Msk              /*!< Low-Power reset flag */



/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define GPIO_CRL_MODE_Pos                    (0U)
#define GPIO_CRL_MODE_Msk                    (0x33333333UL << GPIO_CRL_MODE_Pos) /*!< 0x33333333 */
#define GPIO_CRL_MODE                        GPIO_CRL_MODE_Msk                 /*!< Port x mode bits */

#define GPIO_CRL_MODE0_Pos                   (0U)
#define GPIO_CRL_MODE0_Msk                   (0x3UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000003 */
#define GPIO_CRL_MODE0                       GPIO_CRL_MODE0_Msk                /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CRL_MODE0_0                     (0x1UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000001 */
#define GPIO_CRL_MODE0_1                     (0x2UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000002 */

#define GPIO_CRL_MODE1_Pos                   (4U)
#define GPIO_CRL_MODE1_Msk                   (0x3UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000030 */
#define GPIO_CRL_MODE1                       GPIO_CRL_MODE1_Msk                /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CRL_MODE1_0                     (0x1UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000010 */
#define GPIO_CRL_MODE1_1                     (0x2UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000020 */

#define GPIO_CRL_MODE2_Pos                   (8U)
#define GPIO_CRL_MODE2_Msk                   (0x3UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000300 */
#define GPIO_CRL_MODE2                       GPIO_CRL_MODE2_Msk                /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define GPIO_CRL_MODE2_0                     (0x1UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000100 */
#define GPIO_CRL_MODE2_1                     (0x2UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000200 */

#define GPIO_CRL_MODE3_Pos                   (12U)
#define GPIO_CRL_MODE3_Msk                   (0x3UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00003000 */
#define GPIO_CRL_MODE3                       GPIO_CRL_MODE3_Msk                /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define GPIO_CRL_MODE3_0                     (0x1UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00001000 */
#define GPIO_CRL_MODE3_1                     (0x2UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00002000 */

#define GPIO_CRL_MODE4_Pos                   (16U)
#define GPIO_CRL_MODE4_Msk                   (0x3UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00030000 */
#define GPIO_CRL_MODE4                       GPIO_CRL_MODE4_Msk                /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CRL_MODE4_0                     (0x1UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00010000 */
#define GPIO_CRL_MODE4_1                     (0x2UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00020000 */

#define GPIO_CRL_MODE5_Pos                   (20U)
#define GPIO_CRL_MODE5_Msk                   (0x3UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00300000 */
#define GPIO_CRL_MODE5                       GPIO_CRL_MODE5_Msk                /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CRL_MODE5_0                     (0x1UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00100000 */
#define GPIO_CRL_MODE5_1                     (0x2UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00200000 */

#define GPIO_CRL_MODE6_Pos                   (24U)
#define GPIO_CRL_MODE6_Msk                   (0x3UL << GPIO_CRL_MODE6_Pos)      /*!< 0x03000000 */
#define GPIO_CRL_MODE6                       GPIO_CRL_MODE6_Msk                /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CRL_MODE6_0                     (0x1UL << GPIO_CRL_MODE6_Pos)      /*!< 0x01000000 */
#define GPIO_CRL_MODE6_1                     (0x2UL << GPIO_CRL_MODE6_Pos)      /*!< 0x02000000 */

#define GPIO_CRL_MODE7_Pos                   (28U)
#define GPIO_CRL_MODE7_Msk                   (0x3UL << GPIO_CRL_MODE7_Pos)      /*!< 0x30000000 */
#define GPIO_CRL_MODE7                       GPIO_CRL_MODE7_Msk                /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CRL_MODE7_0                     (0x1UL << GPIO_CRL_MODE7_Pos)      /*!< 0x10000000 */
#define GPIO_CRL_MODE7_1                     (0x2UL << GPIO_CRL_MODE7_Pos)      /*!< 0x20000000 */

#define GPIO_CRL_CNF_Pos                     (2U)
#define GPIO_CRL_CNF_Msk                     (0x33333333UL << GPIO_CRL_CNF_Pos) /*!< 0xCCCCCCCC */
#define GPIO_CRL_CNF                         GPIO_CRL_CNF_Msk                  /*!< Port x configuration bits */

#define GPIO_CRL_CNF0_Pos                    (2U)
#define GPIO_CRL_CNF0_Msk                    (0x3UL << GPIO_CRL_CNF0_Pos)       /*!< 0x0000000C */
#define GPIO_CRL_CNF0                        GPIO_CRL_CNF0_Msk                 /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CRL_CNF0_0                      (0x1UL << GPIO_CRL_CNF0_Pos)       /*!< 0x00000004 */
#define GPIO_CRL_CNF0_1                      (0x2UL << GPIO_CRL_CNF0_Pos)       /*!< 0x00000008 */

#define GPIO_CRL_CNF1_Pos                    (6U)
#define GPIO_CRL_CNF1_Msk                    (0x3UL << GPIO_CRL_CNF1_Pos)       /*!< 0x000000C0 */
#define GPIO_CRL_CNF1                        GPIO_CRL_CNF1_Msk                 /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CRL_CNF1_0                      (0x1UL << GPIO_CRL_CNF1_Pos)       /*!< 0x00000040 */
#define GPIO_CRL_CNF1_1                      (0x2UL << GPIO_CRL_CNF1_Pos)       /*!< 0x00000080 */

#define GPIO_CRL_CNF2_Pos                    (10U)
#define GPIO_CRL_CNF2_Msk                    (0x3UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000C00 */
#define GPIO_CRL_CNF2                        GPIO_CRL_CNF2_Msk                 /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define GPIO_CRL_CNF2_0                      (0x1UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000400 */
#define GPIO_CRL_CNF2_1                      (0x2UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000800 */

#define GPIO_CRL_CNF3_Pos                    (14U)
#define GPIO_CRL_CNF3_Msk                    (0x3UL << GPIO_CRL_CNF3_Pos)       /*!< 0x0000C000 */
#define GPIO_CRL_CNF3                        GPIO_CRL_CNF3_Msk                 /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define GPIO_CRL_CNF3_0                      (0x1UL << GPIO_CRL_CNF3_Pos)       /*!< 0x00004000 */
#define GPIO_CRL_CNF3_1                      (0x2UL << GPIO_CRL_CNF3_Pos)       /*!< 0x00008000 */

#define GPIO_CRL_CNF4_Pos                    (18U)
#define GPIO_CRL_CNF4_Msk                    (0x3UL << GPIO_CRL_CNF4_Pos)       /*!< 0x000C0000 */
#define GPIO_CRL_CNF4                        GPIO_CRL_CNF4_Msk                 /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CRL_CNF4_0                      (0x1UL << GPIO_CRL_CNF4_Pos)       /*!< 0x00040000 */
#define GPIO_CRL_CNF4_1                      (0x2UL << GPIO_CRL_CNF4_Pos)       /*!< 0x00080000 */

#define GPIO_CRL_CNF5_Pos                    (22U)
#define GPIO_CRL_CNF5_Msk                    (0x3UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00C00000 */
#define GPIO_CRL_CNF5                        GPIO_CRL_CNF5_Msk                 /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CRL_CNF5_0                      (0x1UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00400000 */
#define GPIO_CRL_CNF5_1                      (0x2UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00800000 */

#define GPIO_CRL_CNF6_Pos                    (26U)
#define GPIO_CRL_CNF6_Msk                    (0x3UL << GPIO_CRL_CNF6_Pos)       /*!< 0x0C000000 */
#define GPIO_CRL_CNF6                        GPIO_CRL_CNF6_Msk                 /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CRL_CNF6_0                      (0x1UL << GPIO_CRL_CNF6_Pos)       /*!< 0x04000000 */
#define GPIO_CRL_CNF6_1                      (0x2UL << GPIO_CRL_CNF6_Pos)       /*!< 0x08000000 */

#define GPIO_CRL_CNF7_Pos                    (30U)
#define GPIO_CRL_CNF7_Msk                    (0x3UL << GPIO_CRL_CNF7_Pos)       /*!< 0xC0000000 */
#define GPIO_CRL_CNF7                        GPIO_CRL_CNF7_Msk                 /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CRL_CNF7_0                      (0x1UL << GPIO_CRL_CNF7_Pos)       /*!< 0x40000000 */
#define GPIO_CRL_CNF7_1                      (0x2UL << GPIO_CRL_CNF7_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_CRH register  *******************/
#define GPIO_CRH_MODE_Pos                    (0U)
#define GPIO_CRH_MODE_Msk                    (0x33333333UL << GPIO_CRH_MODE_Pos) /*!< 0x33333333 */
#define GPIO_CRH_MODE                        GPIO_CRH_MODE_Msk                 /*!< Port x mode bits */

#define GPIO_CRH_MODE8_Pos                   (0U)
#define GPIO_CRH_MODE8_Msk                   (0x3UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000003 */
#define GPIO_CRH_MODE8                       GPIO_CRH_MODE8_Msk                /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define GPIO_CRH_MODE8_0                     (0x1UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000001 */
#define GPIO_CRH_MODE8_1                     (0x2UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000002 */

#define GPIO_CRH_MODE9_Pos                   (4U)
#define GPIO_CRH_MODE9_Msk                   (0x3UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000030 */
#define GPIO_CRH_MODE9                       GPIO_CRH_MODE9_Msk                /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define GPIO_CRH_MODE9_0                     (0x1UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000010 */
#define GPIO_CRH_MODE9_1                     (0x2UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000020 */

#define GPIO_CRH_MODE10_Pos                  (8U)
#define GPIO_CRH_MODE10_Msk                  (0x3UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000300 */
#define GPIO_CRH_MODE10                      GPIO_CRH_MODE10_Msk               /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define GPIO_CRH_MODE10_0                    (0x1UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000100 */
#define GPIO_CRH_MODE10_1                    (0x2UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000200 */

#define GPIO_CRH_MODE11_Pos                  (12U)
#define GPIO_CRH_MODE11_Msk                  (0x3UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00003000 */
#define GPIO_CRH_MODE11                      GPIO_CRH_MODE11_Msk               /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define GPIO_CRH_MODE11_0                    (0x1UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00001000 */
#define GPIO_CRH_MODE11_1                    (0x2UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00002000 */

#define GPIO_CRH_MODE12_Pos                  (16U)
#define GPIO_CRH_MODE12_Msk                  (0x3UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00030000 */
#define GPIO_CRH_MODE12                      GPIO_CRH_MODE12_Msk               /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define GPIO_CRH_MODE12_0                    (0x1UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00010000 */
#define GPIO_CRH_MODE12_1                    (0x2UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00020000 */

#define GPIO_CRH_MODE13_Pos                  (20U)
#define GPIO_CRH_MODE13_Msk                  (0x3UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00300000 */
#define GPIO_CRH_MODE13                      GPIO_CRH_MODE13_Msk               /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define GPIO_CRH_MODE13_0                    (0x1UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00100000 */
#define GPIO_CRH_MODE13_1                    (0x2UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00200000 */

#define GPIO_CRH_MODE14_Pos                  (24U)
#define GPIO_CRH_MODE14_Msk                  (0x3UL << GPIO_CRH_MODE14_Pos)     /*!< 0x03000000 */
#define GPIO_CRH_MODE14                      GPIO_CRH_MODE14_Msk               /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define GPIO_CRH_MODE14_0                    (0x1UL << GPIO_CRH_MODE14_Pos)     /*!< 0x01000000 */
#define GPIO_CRH_MODE14_1                    (0x2UL << GPIO_CRH_MODE14_Pos)     /*!< 0x02000000 */

#define GPIO_CRH_MODE15_Pos                  (28U)
#define GPIO_CRH_MODE15_Msk                  (0x3UL << GPIO_CRH_MODE15_Pos)     /*!< 0x30000000 */
#define GPIO_CRH_MODE15                      GPIO_CRH_MODE15_Msk               /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define GPIO_CRH_MODE15_0                    (0x1UL << GPIO_CRH_MODE15_Pos)     /*!< 0x10000000 */
#define GPIO_CRH_MODE15_1                    (0x2UL << GPIO_CRH_MODE15_Pos)     /*!< 0x20000000 */

#define GPIO_CRH_CNF_Pos                     (2U)
#define GPIO_CRH_CNF_Msk                     (0x33333333UL << GPIO_CRH_CNF_Pos) /*!< 0xCCCCCCCC */
#define GPIO_CRH_CNF                         GPIO_CRH_CNF_Msk                  /*!< Port x configuration bits */

#define GPIO_CRH_CNF8_Pos                    (2U)
#define GPIO_CRH_CNF8_Msk                    (0x3UL << GPIO_CRH_CNF8_Pos)       /*!< 0x0000000C */
#define GPIO_CRH_CNF8                        GPIO_CRH_CNF8_Msk                 /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define GPIO_CRH_CNF8_0                      (0x1UL << GPIO_CRH_CNF8_Pos)       /*!< 0x00000004 */
#define GPIO_CRH_CNF8_1                      (0x2UL << GPIO_CRH_CNF8_Pos)       /*!< 0x00000008 */

#define GPIO_CRH_CNF9_Pos                    (6U)
#define GPIO_CRH_CNF9_Msk                    (0x3UL << GPIO_CRH_CNF9_Pos)       /*!< 0x000000C0 */
#define GPIO_CRH_CNF9                        GPIO_CRH_CNF9_Msk                 /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define GPIO_CRH_CNF9_0                      (0x1UL << GPIO_CRH_CNF9_Pos)       /*!< 0x00000040 */
#define GPIO_CRH_CNF9_1                      (0x2UL << GPIO_CRH_CNF9_Pos)       /*!< 0x00000080 */

#define GPIO_CRH_CNF10_Pos                   (10U)
#define GPIO_CRH_CNF10_Msk                   (0x3UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000C00 */
#define GPIO_CRH_CNF10                       GPIO_CRH_CNF10_Msk                /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define GPIO_CRH_CNF10_0                     (0x1UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000400 */
#define GPIO_CRH_CNF10_1                     (0x2UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000800 */

#define GPIO_CRH_CNF11_Pos                   (14U)
#define GPIO_CRH_CNF11_Msk                   (0x3UL << GPIO_CRH_CNF11_Pos)      /*!< 0x0000C000 */
#define GPIO_CRH_CNF11                       GPIO_CRH_CNF11_Msk                /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define GPIO_CRH_CNF11_0                     (0x1UL << GPIO_CRH_CNF11_Pos)      /*!< 0x00004000 */
#define GPIO_CRH_CNF11_1                     (0x2UL << GPIO_CRH_CNF11_Pos)      /*!< 0x00008000 */

#define GPIO_CRH_CNF12_Pos                   (18U)
#define GPIO_CRH_CNF12_Msk                   (0x3UL << GPIO_CRH_CNF12_Pos)      /*!< 0x000C0000 */
#define GPIO_CRH_CNF12                       GPIO_CRH_CNF12_Msk                /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define GPIO_CRH_CNF12_0                     (0x1UL << GPIO_CRH_CNF12_Pos)      /*!< 0x00040000 */
#define GPIO_CRH_CNF12_1                     (0x2UL << GPIO_CRH_CNF12_Pos)      /*!< 0x00080000 */

#define GPIO_CRH_CNF13_Pos                   (22U)
#define GPIO_CRH_CNF13_Msk                   (0x3UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00C00000 */
#define GPIO_CRH_CNF13                       GPIO_CRH_CNF13_Msk                /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define GPIO_CRH_CNF13_0                     (0x1UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00400000 */
#define GPIO_CRH_CNF13_1                     (0x2UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00800000 */

#define GPIO_CRH_CNF14_Pos                   (26U)
#define GPIO_CRH_CNF14_Msk                   (0x3UL << GPIO_CRH_CNF14_Pos)      /*!< 0x0C000000 */
#define GPIO_CRH_CNF14                       GPIO_CRH_CNF14_Msk                /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define GPIO_CRH_CNF14_0                     (0x1UL << GPIO_CRH_CNF14_Pos)      /*!< 0x04000000 */
#define GPIO_CRH_CNF14_1                     (0x2UL << GPIO_CRH_CNF14_Pos)      /*!< 0x08000000 */

#define GPIO_CRH_CNF15_Pos                   (30U)
#define GPIO_CRH_CNF15_Msk                   (0x3UL << GPIO_CRH_CNF15_Pos)      /*!< 0xC0000000 */
#define GPIO_CRH_CNF15                       GPIO_CRH_CNF15_Msk                /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define GPIO_CRH_CNF15_0                     (0x1UL << GPIO_CRH_CNF15_Pos)      /*!< 0x40000000 */
#define GPIO_CRH_CNF15_1                     (0x2UL << GPIO_CRH_CNF15_Pos)      /*!< 0x80000000 */

/*!<******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0_Pos                    (0U)
#define GPIO_IDR_IDR0_Msk                    (0x1UL << GPIO_IDR_IDR0_Pos)       /*!< 0x00000001 */
#define GPIO_IDR_IDR0                        GPIO_IDR_IDR0_Msk                 /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1_Pos                    (1U)
#define GPIO_IDR_IDR1_Msk                    (0x1UL << GPIO_IDR_IDR1_Pos)       /*!< 0x00000002 */
#define GPIO_IDR_IDR1                        GPIO_IDR_IDR1_Msk                 /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2_Pos                    (2U)
#define GPIO_IDR_IDR2_Msk                    (0x1UL << GPIO_IDR_IDR2_Pos)       /*!< 0x00000004 */
#define GPIO_IDR_IDR2                        GPIO_IDR_IDR2_Msk                 /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3_Pos                    (3U)
#define GPIO_IDR_IDR3_Msk                    (0x1UL << GPIO_IDR_IDR3_Pos)       /*!< 0x00000008 */
#define GPIO_IDR_IDR3                        GPIO_IDR_IDR3_Msk                 /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4_Pos                    (4U)
#define GPIO_IDR_IDR4_Msk                    (0x1UL << GPIO_IDR_IDR4_Pos)       /*!< 0x00000010 */
#define GPIO_IDR_IDR4                        GPIO_IDR_IDR4_Msk                 /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5_Pos                    (5U)
#define GPIO_IDR_IDR5_Msk                    (0x1UL << GPIO_IDR_IDR5_Pos)       /*!< 0x00000020 */
#define GPIO_IDR_IDR5                        GPIO_IDR_IDR5_Msk                 /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6_Pos                    (6U)
#define GPIO_IDR_IDR6_Msk                    (0x1UL << GPIO_IDR_IDR6_Pos)       /*!< 0x00000040 */
#define GPIO_IDR_IDR6                        GPIO_IDR_IDR6_Msk                 /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7_Pos                    (7U)
#define GPIO_IDR_IDR7_Msk                    (0x1UL << GPIO_IDR_IDR7_Pos)       /*!< 0x00000080 */
#define GPIO_IDR_IDR7                        GPIO_IDR_IDR7_Msk                 /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8_Pos                    (8U)
#define GPIO_IDR_IDR8_Msk                    (0x1UL << GPIO_IDR_IDR8_Pos)       /*!< 0x00000100 */
#define GPIO_IDR_IDR8                        GPIO_IDR_IDR8_Msk                 /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9_Pos                    (9U)
#define GPIO_IDR_IDR9_Msk                    (0x1UL << GPIO_IDR_IDR9_Pos)       /*!< 0x00000200 */
#define GPIO_IDR_IDR9                        GPIO_IDR_IDR9_Msk                 /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10_Pos                   (10U)
#define GPIO_IDR_IDR10_Msk                   (0x1UL << GPIO_IDR_IDR10_Pos)      /*!< 0x00000400 */
#define GPIO_IDR_IDR10                       GPIO_IDR_IDR10_Msk                /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11_Pos                   (11U)
#define GPIO_IDR_IDR11_Msk                   (0x1UL << GPIO_IDR_IDR11_Pos)      /*!< 0x00000800 */
#define GPIO_IDR_IDR11                       GPIO_IDR_IDR11_Msk                /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12_Pos                   (12U)
#define GPIO_IDR_IDR12_Msk                   (0x1UL << GPIO_IDR_IDR12_Pos)      /*!< 0x00001000 */
#define GPIO_IDR_IDR12                       GPIO_IDR_IDR12_Msk                /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13_Pos                   (13U)
#define GPIO_IDR_IDR13_Msk                   (0x1UL << GPIO_IDR_IDR13_Pos)      /*!< 0x00002000 */
#define GPIO_IDR_IDR13                       GPIO_IDR_IDR13_Msk                /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14_Pos                   (14U)
#define GPIO_IDR_IDR14_Msk                   (0x1UL << GPIO_IDR_IDR14_Pos)      /*!< 0x00004000 */
#define GPIO_IDR_IDR14                       GPIO_IDR_IDR14_Msk                /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15_Pos                   (15U)
#define GPIO_IDR_IDR15_Msk                   (0x1UL << GPIO_IDR_IDR15_Pos)      /*!< 0x00008000 */
#define GPIO_IDR_IDR15                       GPIO_IDR_IDR15_Msk                /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0_Pos                    (0U)
#define GPIO_ODR_ODR0_Msk                    (0x1UL << GPIO_ODR_ODR0_Pos)       /*!< 0x00000001 */
#define GPIO_ODR_ODR0                        GPIO_ODR_ODR0_Msk                 /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1_Pos                    (1U)
#define GPIO_ODR_ODR1_Msk                    (0x1UL << GPIO_ODR_ODR1_Pos)       /*!< 0x00000002 */
#define GPIO_ODR_ODR1                        GPIO_ODR_ODR1_Msk                 /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2_Pos                    (2U)
#define GPIO_ODR_ODR2_Msk                    (0x1UL << GPIO_ODR_ODR2_Pos)       /*!< 0x00000004 */
#define GPIO_ODR_ODR2                        GPIO_ODR_ODR2_Msk                 /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3_Pos                    (3U)
#define GPIO_ODR_ODR3_Msk                    (0x1UL << GPIO_ODR_ODR3_Pos)       /*!< 0x00000008 */
#define GPIO_ODR_ODR3                        GPIO_ODR_ODR3_Msk                 /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4_Pos                    (4U)
#define GPIO_ODR_ODR4_Msk                    (0x1UL << GPIO_ODR_ODR4_Pos)       /*!< 0x00000010 */
#define GPIO_ODR_ODR4                        GPIO_ODR_ODR4_Msk                 /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5_Pos                    (5U)
#define GPIO_ODR_ODR5_Msk                    (0x1UL << GPIO_ODR_ODR5_Pos)       /*!< 0x00000020 */
#define GPIO_ODR_ODR5                        GPIO_ODR_ODR5_Msk                 /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6_Pos                    (6U)
#define GPIO_ODR_ODR6_Msk                    (0x1UL << GPIO_ODR_ODR6_Pos)       /*!< 0x00000040 */
#define GPIO_ODR_ODR6                        GPIO_ODR_ODR6_Msk                 /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7_Pos                    (7U)
#define GPIO_ODR_ODR7_Msk                    (0x1UL << GPIO_ODR_ODR7_Pos)       /*!< 0x00000080 */
#define GPIO_ODR_ODR7                        GPIO_ODR_ODR7_Msk                 /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8_Pos                    (8U)
#define GPIO_ODR_ODR8_Msk                    (0x1UL << GPIO_ODR_ODR8_Pos)       /*!< 0x00000100 */
#define GPIO_ODR_ODR8                        GPIO_ODR_ODR8_Msk                 /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9_Pos                    (9U)
#define GPIO_ODR_ODR9_Msk                    (0x1UL << GPIO_ODR_ODR9_Pos)       /*!< 0x00000200 */
#define GPIO_ODR_ODR9                        GPIO_ODR_ODR9_Msk                 /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10_Pos                   (10U)
#define GPIO_ODR_ODR10_Msk                   (0x1UL << GPIO_ODR_ODR10_Pos)      /*!< 0x00000400 */
#define GPIO_ODR_ODR10                       GPIO_ODR_ODR10_Msk                /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11_Pos                   (11U)
#define GPIO_ODR_ODR11_Msk                   (0x1UL << GPIO_ODR_ODR11_Pos)      /*!< 0x00000800 */
#define GPIO_ODR_ODR11                       GPIO_ODR_ODR11_Msk                /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12_Pos                   (12U)
#define GPIO_ODR_ODR12_Msk                   (0x1UL << GPIO_ODR_ODR12_Pos)      /*!< 0x00001000 */
#define GPIO_ODR_ODR12                       GPIO_ODR_ODR12_Msk                /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13_Pos                   (13U)
#define GPIO_ODR_ODR13_Msk                   (0x1UL << GPIO_ODR_ODR13_Pos)      /*!< 0x00002000 */
#define GPIO_ODR_ODR13                       GPIO_ODR_ODR13_Msk                /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14_Pos                   (14U)
#define GPIO_ODR_ODR14_Msk                   (0x1UL << GPIO_ODR_ODR14_Pos)      /*!< 0x00004000 */
#define GPIO_ODR_ODR14                       GPIO_ODR_ODR14_Msk                /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15_Pos                   (15U)
#define GPIO_ODR_ODR15_Msk                   (0x1UL << GPIO_ODR_ODR15_Pos)      /*!< 0x00008000 */
#define GPIO_ODR_ODR15                       GPIO_ODR_ODR15_Msk                /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0_Pos                    (0U)
#define GPIO_BSRR_BS0_Msk                    (0x1UL << GPIO_BSRR_BS0_Pos)       /*!< 0x00000001 */
#define GPIO_BSRR_BS0                        GPIO_BSRR_BS0_Msk                 /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1_Pos                    (1U)
#define GPIO_BSRR_BS1_Msk                    (0x1UL << GPIO_BSRR_BS1_Pos)       /*!< 0x00000002 */
#define GPIO_BSRR_BS1                        GPIO_BSRR_BS1_Msk                 /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2_Pos                    (2U)
#define GPIO_BSRR_BS2_Msk                    (0x1UL << GPIO_BSRR_BS2_Pos)       /*!< 0x00000004 */
#define GPIO_BSRR_BS2                        GPIO_BSRR_BS2_Msk                 /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3_Pos                    (3U)
#define GPIO_BSRR_BS3_Msk                    (0x1UL << GPIO_BSRR_BS3_Pos)       /*!< 0x00000008 */
#define GPIO_BSRR_BS3                        GPIO_BSRR_BS3_Msk                 /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4_Pos                    (4U)
#define GPIO_BSRR_BS4_Msk                    (0x1UL << GPIO_BSRR_BS4_Pos)       /*!< 0x00000010 */
#define GPIO_BSRR_BS4                        GPIO_BSRR_BS4_Msk                 /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5_Pos                    (5U)
#define GPIO_BSRR_BS5_Msk                    (0x1UL << GPIO_BSRR_BS5_Pos)       /*!< 0x00000020 */
#define GPIO_BSRR_BS5                        GPIO_BSRR_BS5_Msk                 /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6_Pos                    (6U)
#define GPIO_BSRR_BS6_Msk                    (0x1UL << GPIO_BSRR_BS6_Pos)       /*!< 0x00000040 */
#define GPIO_BSRR_BS6                        GPIO_BSRR_BS6_Msk                 /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7_Pos                    (7U)
#define GPIO_BSRR_BS7_Msk                    (0x1UL << GPIO_BSRR_BS7_Pos)       /*!< 0x00000080 */
#define GPIO_BSRR_BS7                        GPIO_BSRR_BS7_Msk                 /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8_Pos                    (8U)
#define GPIO_BSRR_BS8_Msk                    (0x1UL << GPIO_BSRR_BS8_Pos)       /*!< 0x00000100 */
#define GPIO_BSRR_BS8                        GPIO_BSRR_BS8_Msk                 /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9_Pos                    (9U)
#define GPIO_BSRR_BS9_Msk                    (0x1UL << GPIO_BSRR_BS9_Pos)       /*!< 0x00000200 */
#define GPIO_BSRR_BS9                        GPIO_BSRR_BS9_Msk                 /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10_Pos                   (10U)
#define GPIO_BSRR_BS10_Msk                   (0x1UL << GPIO_BSRR_BS10_Pos)      /*!< 0x00000400 */
#define GPIO_BSRR_BS10                       GPIO_BSRR_BS10_Msk                /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11_Pos                   (11U)
#define GPIO_BSRR_BS11_Msk                   (0x1UL << GPIO_BSRR_BS11_Pos)      /*!< 0x00000800 */
#define GPIO_BSRR_BS11                       GPIO_BSRR_BS11_Msk                /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12_Pos                   (12U)
#define GPIO_BSRR_BS12_Msk                   (0x1UL << GPIO_BSRR_BS12_Pos)      /*!< 0x00001000 */
#define GPIO_BSRR_BS12                       GPIO_BSRR_BS12_Msk                /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13_Pos                   (13U)
#define GPIO_BSRR_BS13_Msk                   (0x1UL << GPIO_BSRR_BS13_Pos)      /*!< 0x00002000 */
#define GPIO_BSRR_BS13                       GPIO_BSRR_BS13_Msk                /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14_Pos                   (14U)
#define GPIO_BSRR_BS14_Msk                   (0x1UL << GPIO_BSRR_BS14_Pos)      /*!< 0x00004000 */
#define GPIO_BSRR_BS14                       GPIO_BSRR_BS14_Msk                /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15_Pos                   (15U)
#define GPIO_BSRR_BS15_Msk                   (0x1UL << GPIO_BSRR_BS15_Pos)      /*!< 0x00008000 */
#define GPIO_BSRR_BS15                       GPIO_BSRR_BS15_Msk                /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0_Pos                    (16U)
#define GPIO_BSRR_BR0_Msk                    (0x1UL << GPIO_BSRR_BR0_Pos)       /*!< 0x00010000 */
#define GPIO_BSRR_BR0                        GPIO_BSRR_BR0_Msk                 /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1_Pos                    (17U)
#define GPIO_BSRR_BR1_Msk                    (0x1UL << GPIO_BSRR_BR1_Pos)       /*!< 0x00020000 */
#define GPIO_BSRR_BR1                        GPIO_BSRR_BR1_Msk                 /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2_Pos                    (18U)
#define GPIO_BSRR_BR2_Msk                    (0x1UL << GPIO_BSRR_BR2_Pos)       /*!< 0x00040000 */
#define GPIO_BSRR_BR2                        GPIO_BSRR_BR2_Msk                 /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3_Pos                    (19U)
#define GPIO_BSRR_BR3_Msk                    (0x1UL << GPIO_BSRR_BR3_Pos)       /*!< 0x00080000 */
#define GPIO_BSRR_BR3                        GPIO_BSRR_BR3_Msk                 /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4_Pos                    (20U)
#define GPIO_BSRR_BR4_Msk                    (0x1UL << GPIO_BSRR_BR4_Pos)       /*!< 0x00100000 */
#define GPIO_BSRR_BR4                        GPIO_BSRR_BR4_Msk                 /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5_Pos                    (21U)
#define GPIO_BSRR_BR5_Msk                    (0x1UL << GPIO_BSRR_BR5_Pos)       /*!< 0x00200000 */
#define GPIO_BSRR_BR5                        GPIO_BSRR_BR5_Msk                 /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6_Pos                    (22U)
#define GPIO_BSRR_BR6_Msk                    (0x1UL << GPIO_BSRR_BR6_Pos)       /*!< 0x00400000 */
#define GPIO_BSRR_BR6                        GPIO_BSRR_BR6_Msk                 /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7_Pos                    (23U)
#define GPIO_BSRR_BR7_Msk                    (0x1UL << GPIO_BSRR_BR7_Pos)       /*!< 0x00800000 */
#define GPIO_BSRR_BR7                        GPIO_BSRR_BR7_Msk                 /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8_Pos                    (24U)
#define GPIO_BSRR_BR8_Msk                    (0x1UL << GPIO_BSRR_BR8_Pos)       /*!< 0x01000000 */
#define GPIO_BSRR_BR8                        GPIO_BSRR_BR8_Msk                 /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9_Pos                    (25U)
#define GPIO_BSRR_BR9_Msk                    (0x1UL << GPIO_BSRR_BR9_Pos)       /*!< 0x02000000 */
#define GPIO_BSRR_BR9                        GPIO_BSRR_BR9_Msk                 /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10_Pos                   (26U)
#define GPIO_BSRR_BR10_Msk                   (0x1UL << GPIO_BSRR_BR10_Pos)      /*!< 0x04000000 */
#define GPIO_BSRR_BR10                       GPIO_BSRR_BR10_Msk                /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11_Pos                   (27U)
#define GPIO_BSRR_BR11_Msk                   (0x1UL << GPIO_BSRR_BR11_Pos)      /*!< 0x08000000 */
#define GPIO_BSRR_BR11                       GPIO_BSRR_BR11_Msk                /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12_Pos                   (28U)
#define GPIO_BSRR_BR12_Msk                   (0x1UL << GPIO_BSRR_BR12_Pos)      /*!< 0x10000000 */
#define GPIO_BSRR_BR12                       GPIO_BSRR_BR12_Msk                /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13_Pos                   (29U)
#define GPIO_BSRR_BR13_Msk                   (0x1UL << GPIO_BSRR_BR13_Pos)      /*!< 0x20000000 */
#define GPIO_BSRR_BR13                       GPIO_BSRR_BR13_Msk                /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14_Pos                   (30U)
#define GPIO_BSRR_BR14_Msk                   (0x1UL << GPIO_BSRR_BR14_Pos)      /*!< 0x40000000 */
#define GPIO_BSRR_BR14                       GPIO_BSRR_BR14_Msk                /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15_Pos                   (31U)
#define GPIO_BSRR_BR15_Msk                   (0x1UL << GPIO_BSRR_BR15_Pos)      /*!< 0x80000000 */
#define GPIO_BSRR_BR15                       GPIO_BSRR_BR15_Msk                /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0_Pos                     (0U)
#define GPIO_BRR_BR0_Msk                     (0x1UL << GPIO_BRR_BR0_Pos)        /*!< 0x00000001 */
#define GPIO_BRR_BR0                         GPIO_BRR_BR0_Msk                  /*!< Port x Reset bit 0 */
#define GPIO_BRR_BR1_Pos                     (1U)
#define GPIO_BRR_BR1_Msk                     (0x1UL << GPIO_BRR_BR1_Pos)        /*!< 0x00000002 */
#define GPIO_BRR_BR1                         GPIO_BRR_BR1_Msk                  /*!< Port x Reset bit 1 */
#define GPIO_BRR_BR2_Pos                     (2U)
#define GPIO_BRR_BR2_Msk                     (0x1UL << GPIO_BRR_BR2_Pos)        /*!< 0x00000004 */
#define GPIO_BRR_BR2                         GPIO_BRR_BR2_Msk                  /*!< Port x Reset bit 2 */
#define GPIO_BRR_BR3_Pos                     (3U)
#define GPIO_BRR_BR3_Msk                     (0x1UL << GPIO_BRR_BR3_Pos)        /*!< 0x00000008 */
#define GPIO_BRR_BR3                         GPIO_BRR_BR3_Msk                  /*!< Port x Reset bit 3 */
#define GPIO_BRR_BR4_Pos                     (4U)
#define GPIO_BRR_BR4_Msk                     (0x1UL << GPIO_BRR_BR4_Pos)        /*!< 0x00000010 */
#define GPIO_BRR_BR4                         GPIO_BRR_BR4_Msk                  /*!< Port x Reset bit 4 */
#define GPIO_BRR_BR5_Pos                     (5U)
#define GPIO_BRR_BR5_Msk                     (0x1UL << GPIO_BRR_BR5_Pos)        /*!< 0x00000020 */
#define GPIO_BRR_BR5                         GPIO_BRR_BR5_Msk                  /*!< Port x Reset bit 5 */
#define GPIO_BRR_BR6_Pos                     (6U)
#define GPIO_BRR_BR6_Msk                     (0x1UL << GPIO_BRR_BR6_Pos)        /*!< 0x00000040 */
#define GPIO_BRR_BR6                         GPIO_BRR_BR6_Msk                  /*!< Port x Reset bit 6 */
#define GPIO_BRR_BR7_Pos                     (7U)
#define GPIO_BRR_BR7_Msk                     (0x1UL << GPIO_BRR_BR7_Pos)        /*!< 0x00000080 */
#define GPIO_BRR_BR7                         GPIO_BRR_BR7_Msk                  /*!< Port x Reset bit 7 */
#define GPIO_BRR_BR8_Pos                     (8U)
#define GPIO_BRR_BR8_Msk                     (0x1UL << GPIO_BRR_BR8_Pos)        /*!< 0x00000100 */
#define GPIO_BRR_BR8                         GPIO_BRR_BR8_Msk                  /*!< Port x Reset bit 8 */
#define GPIO_BRR_BR9_Pos                     (9U)
#define GPIO_BRR_BR9_Msk                     (0x1UL << GPIO_BRR_BR9_Pos)        /*!< 0x00000200 */
#define GPIO_BRR_BR9                         GPIO_BRR_BR9_Msk                  /*!< Port x Reset bit 9 */
#define GPIO_BRR_BR10_Pos                    (10U)
#define GPIO_BRR_BR10_Msk                    (0x1UL << GPIO_BRR_BR10_Pos)       /*!< 0x00000400 */
#define GPIO_BRR_BR10                        GPIO_BRR_BR10_Msk                 /*!< Port x Reset bit 10 */
#define GPIO_BRR_BR11_Pos                    (11U)
#define GPIO_BRR_BR11_Msk                    (0x1UL << GPIO_BRR_BR11_Pos)       /*!< 0x00000800 */
#define GPIO_BRR_BR11                        GPIO_BRR_BR11_Msk                 /*!< Port x Reset bit 11 */
#define GPIO_BRR_BR12_Pos                    (12U)
#define GPIO_BRR_BR12_Msk                    (0x1UL << GPIO_BRR_BR12_Pos)       /*!< 0x00001000 */
#define GPIO_BRR_BR12                        GPIO_BRR_BR12_Msk                 /*!< Port x Reset bit 12 */
#define GPIO_BRR_BR13_Pos                    (13U)
#define GPIO_BRR_BR13_Msk                    (0x1UL << GPIO_BRR_BR13_Pos)       /*!< 0x00002000 */
#define GPIO_BRR_BR13                        GPIO_BRR_BR13_Msk                 /*!< Port x Reset bit 13 */
#define GPIO_BRR_BR14_Pos                    (14U)
#define GPIO_BRR_BR14_Msk                    (0x1UL << GPIO_BRR_BR14_Pos)       /*!< 0x00004000 */
#define GPIO_BRR_BR14                        GPIO_BRR_BR14_Msk                 /*!< Port x Reset bit 14 */
#define GPIO_BRR_BR15_Pos                    (15U)
#define GPIO_BRR_BR15_Msk                    (0x1UL << GPIO_BRR_BR15_Pos)       /*!< 0x00008000 */
#define GPIO_BRR_BR15                        GPIO_BRR_BR15_Msk                 /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0_Pos                   (0U)
#define GPIO_LCKR_LCK0_Msk                   (0x1UL << GPIO_LCKR_LCK0_Pos)      /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                       GPIO_LCKR_LCK0_Msk                /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1_Pos                   (1U)
#define GPIO_LCKR_LCK1_Msk                   (0x1UL << GPIO_LCKR_LCK1_Pos)      /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                       GPIO_LCKR_LCK1_Msk                /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2_Pos                   (2U)
#define GPIO_LCKR_LCK2_Msk                   (0x1UL << GPIO_LCKR_LCK2_Pos)      /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                       GPIO_LCKR_LCK2_Msk                /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3_Pos                   (3U)
#define GPIO_LCKR_LCK3_Msk                   (0x1UL << GPIO_LCKR_LCK3_Pos)      /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                       GPIO_LCKR_LCK3_Msk                /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4_Pos                   (4U)
#define GPIO_LCKR_LCK4_Msk                   (0x1UL << GPIO_LCKR_LCK4_Pos)      /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                       GPIO_LCKR_LCK4_Msk                /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5_Pos                   (5U)
#define GPIO_LCKR_LCK5_Msk                   (0x1UL << GPIO_LCKR_LCK5_Pos)      /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                       GPIO_LCKR_LCK5_Msk                /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6_Pos                   (6U)
#define GPIO_LCKR_LCK6_Msk                   (0x1UL << GPIO_LCKR_LCK6_Pos)      /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                       GPIO_LCKR_LCK6_Msk                /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7_Pos                   (7U)
#define GPIO_LCKR_LCK7_Msk                   (0x1UL << GPIO_LCKR_LCK7_Pos)      /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                       GPIO_LCKR_LCK7_Msk                /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8_Pos                   (8U)
#define GPIO_LCKR_LCK8_Msk                   (0x1UL << GPIO_LCKR_LCK8_Pos)      /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                       GPIO_LCKR_LCK8_Msk                /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9_Pos                   (9U)
#define GPIO_LCKR_LCK9_Msk                   (0x1UL << GPIO_LCKR_LCK9_Pos)      /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                       GPIO_LCKR_LCK9_Msk                /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10_Pos                  (10U)
#define GPIO_LCKR_LCK10_Msk                  (0x1UL << GPIO_LCKR_LCK10_Pos)     /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                      GPIO_LCKR_LCK10_Msk               /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11_Pos                  (11U)
#define GPIO_LCKR_LCK11_Msk                  (0x1UL << GPIO_LCKR_LCK11_Pos)     /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                      GPIO_LCKR_LCK11_Msk               /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12_Pos                  (12U)
#define GPIO_LCKR_LCK12_Msk                  (0x1UL << GPIO_LCKR_LCK12_Pos)     /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                      GPIO_LCKR_LCK12_Msk               /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13_Pos                  (13U)
#define GPIO_LCKR_LCK13_Msk                  (0x1UL << GPIO_LCKR_LCK13_Pos)     /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                      GPIO_LCKR_LCK13_Msk               /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14_Pos                  (14U)
#define GPIO_LCKR_LCK14_Msk                  (0x1UL << GPIO_LCKR_LCK14_Pos)     /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                      GPIO_LCKR_LCK14_Msk               /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15_Pos                  (15U)
#define GPIO_LCKR_LCK15_Msk                  (0x1UL << GPIO_LCKR_LCK15_Pos)     /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                      GPIO_LCKR_LCK15_Msk               /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK_Pos                   (16U)
#define GPIO_LCKR_LCKK_Msk                   (0x1UL << GPIO_LCKR_LCKK_Pos)      /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                       GPIO_LCKR_LCKK_Msk                /*!< Lock key */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN_Pos                    (0U)
#define AFIO_EVCR_PIN_Msk                    (0xFUL << AFIO_EVCR_PIN_Pos)       /*!< 0x0000000F */
#define AFIO_EVCR_PIN                        AFIO_EVCR_PIN_Msk                 /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      (0x1UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000001 */
#define AFIO_EVCR_PIN_1                      (0x2UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000002 */
#define AFIO_EVCR_PIN_2                      (0x4UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000004 */
#define AFIO_EVCR_PIN_3                      (0x8UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000008 */

/*!< PIN configuration */
#define AFIO_EVCR_PIN_PX0                    0x00000000U                       /*!< Pin 0 selected */
#define AFIO_EVCR_PIN_PX1_Pos                (0U)
#define AFIO_EVCR_PIN_PX1_Msk                (0x1UL << AFIO_EVCR_PIN_PX1_Pos)   /*!< 0x00000001 */
#define AFIO_EVCR_PIN_PX1                    AFIO_EVCR_PIN_PX1_Msk             /*!< Pin 1 selected */
#define AFIO_EVCR_PIN_PX2_Pos                (1U)
#define AFIO_EVCR_PIN_PX2_Msk                (0x1UL << AFIO_EVCR_PIN_PX2_Pos)   /*!< 0x00000002 */
#define AFIO_EVCR_PIN_PX2                    AFIO_EVCR_PIN_PX2_Msk             /*!< Pin 2 selected */
#define AFIO_EVCR_PIN_PX3_Pos                (0U)
#define AFIO_EVCR_PIN_PX3_Msk                (0x3UL << AFIO_EVCR_PIN_PX3_Pos)   /*!< 0x00000003 */
#define AFIO_EVCR_PIN_PX3                    AFIO_EVCR_PIN_PX3_Msk             /*!< Pin 3 selected */
#define AFIO_EVCR_PIN_PX4_Pos                (2U)
#define AFIO_EVCR_PIN_PX4_Msk                (0x1UL << AFIO_EVCR_PIN_PX4_Pos)   /*!< 0x00000004 */
#define AFIO_EVCR_PIN_PX4                    AFIO_EVCR_PIN_PX4_Msk             /*!< Pin 4 selected */
#define AFIO_EVCR_PIN_PX5_Pos                (0U)
#define AFIO_EVCR_PIN_PX5_Msk                (0x5UL << AFIO_EVCR_PIN_PX5_Pos)   /*!< 0x00000005 */
#define AFIO_EVCR_PIN_PX5                    AFIO_EVCR_PIN_PX5_Msk             /*!< Pin 5 selected */
#define AFIO_EVCR_PIN_PX6_Pos                (1U)
#define AFIO_EVCR_PIN_PX6_Msk                (0x3UL << AFIO_EVCR_PIN_PX6_Pos)   /*!< 0x00000006 */
#define AFIO_EVCR_PIN_PX6                    AFIO_EVCR_PIN_PX6_Msk             /*!< Pin 6 selected */
#define AFIO_EVCR_PIN_PX7_Pos                (0U)
#define AFIO_EVCR_PIN_PX7_Msk                (0x7UL << AFIO_EVCR_PIN_PX7_Pos)   /*!< 0x00000007 */
#define AFIO_EVCR_PIN_PX7                    AFIO_EVCR_PIN_PX7_Msk             /*!< Pin 7 selected */
#define AFIO_EVCR_PIN_PX8_Pos                (3U)
#define AFIO_EVCR_PIN_PX8_Msk                (0x1UL << AFIO_EVCR_PIN_PX8_Pos)   /*!< 0x00000008 */
#define AFIO_EVCR_PIN_PX8                    AFIO_EVCR_PIN_PX8_Msk             /*!< Pin 8 selected */
#define AFIO_EVCR_PIN_PX9_Pos                (0U)
#define AFIO_EVCR_PIN_PX9_Msk                (0x9UL << AFIO_EVCR_PIN_PX9_Pos)   /*!< 0x00000009 */
#define AFIO_EVCR_PIN_PX9                    AFIO_EVCR_PIN_PX9_Msk             /*!< Pin 9 selected */
#define AFIO_EVCR_PIN_PX10_Pos               (1U)
#define AFIO_EVCR_PIN_PX10_Msk               (0x5UL << AFIO_EVCR_PIN_PX10_Pos)  /*!< 0x0000000A */
#define AFIO_EVCR_PIN_PX10                   AFIO_EVCR_PIN_PX10_Msk            /*!< Pin 10 selected */
#define AFIO_EVCR_PIN_PX11_Pos               (0U)
#define AFIO_EVCR_PIN_PX11_Msk               (0xBUL << AFIO_EVCR_PIN_PX11_Pos)  /*!< 0x0000000B */
#define AFIO_EVCR_PIN_PX11                   AFIO_EVCR_PIN_PX11_Msk            /*!< Pin 11 selected */
#define AFIO_EVCR_PIN_PX12_Pos               (2U)
#define AFIO_EVCR_PIN_PX12_Msk               (0x3UL << AFIO_EVCR_PIN_PX12_Pos)  /*!< 0x0000000C */
#define AFIO_EVCR_PIN_PX12                   AFIO_EVCR_PIN_PX12_Msk            /*!< Pin 12 selected */
#define AFIO_EVCR_PIN_PX13_Pos               (0U)
#define AFIO_EVCR_PIN_PX13_Msk               (0xDUL << AFIO_EVCR_PIN_PX13_Pos)  /*!< 0x0000000D */
#define AFIO_EVCR_PIN_PX13                   AFIO_EVCR_PIN_PX13_Msk            /*!< Pin 13 selected */
#define AFIO_EVCR_PIN_PX14_Pos               (1U)
#define AFIO_EVCR_PIN_PX14_Msk               (0x7UL << AFIO_EVCR_PIN_PX14_Pos)  /*!< 0x0000000E */
#define AFIO_EVCR_PIN_PX14                   AFIO_EVCR_PIN_PX14_Msk            /*!< Pin 14 selected */
#define AFIO_EVCR_PIN_PX15_Pos               (0U)
#define AFIO_EVCR_PIN_PX15_Msk               (0xFUL << AFIO_EVCR_PIN_PX15_Pos)  /*!< 0x0000000F */
#define AFIO_EVCR_PIN_PX15                   AFIO_EVCR_PIN_PX15_Msk            /*!< Pin 15 selected */

#define AFIO_EVCR_PORT_Pos                   (4U)
#define AFIO_EVCR_PORT_Msk                   (0x7UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000070 */
#define AFIO_EVCR_PORT                       AFIO_EVCR_PORT_Msk                /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     (0x1UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000010 */
#define AFIO_EVCR_PORT_1                     (0x2UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000020 */
#define AFIO_EVCR_PORT_2                     (0x4UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000040 */

/*!< PORT configuration */
#define AFIO_EVCR_PORT_PA                    0x00000000                        /*!< Port A selected */
#define AFIO_EVCR_PORT_PB_Pos                (4U)
#define AFIO_EVCR_PORT_PB_Msk                (0x1UL << AFIO_EVCR_PORT_PB_Pos)   /*!< 0x00000010 */
#define AFIO_EVCR_PORT_PB                    AFIO_EVCR_PORT_PB_Msk             /*!< Port B selected */
#define AFIO_EVCR_PORT_PC_Pos                (5U)
#define AFIO_EVCR_PORT_PC_Msk                (0x1UL << AFIO_EVCR_PORT_PC_Pos)   /*!< 0x00000020 */
#define AFIO_EVCR_PORT_PC                    AFIO_EVCR_PORT_PC_Msk             /*!< Port C selected */
#define AFIO_EVCR_PORT_PD_Pos                (4U)
#define AFIO_EVCR_PORT_PD_Msk                (0x3UL << AFIO_EVCR_PORT_PD_Pos)   /*!< 0x00000030 */
#define AFIO_EVCR_PORT_PD                    AFIO_EVCR_PORT_PD_Msk             /*!< Port D selected */
#define AFIO_EVCR_PORT_PE_Pos                (6U)
#define AFIO_EVCR_PORT_PE_Msk                (0x1UL << AFIO_EVCR_PORT_PE_Pos)   /*!< 0x00000040 */
#define AFIO_EVCR_PORT_PE                    AFIO_EVCR_PORT_PE_Msk             /*!< Port E selected */

#define AFIO_EVCR_EVOE_Pos                   (7U)
#define AFIO_EVCR_EVOE_Msk                   (0x1UL << AFIO_EVCR_EVOE_Pos)      /*!< 0x00000080 */
#define AFIO_EVCR_EVOE                       AFIO_EVCR_EVOE_Msk                /*!< Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP_Pos             (0U)
#define AFIO_MAPR_SPI1_REMAP_Msk             (0x1UL << AFIO_MAPR_SPI1_REMAP_Pos) /*!< 0x00000001 */
#define AFIO_MAPR_SPI1_REMAP                 AFIO_MAPR_SPI1_REMAP_Msk          /*!< SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP_Pos             (1U)
#define AFIO_MAPR_I2C1_REMAP_Msk             (0x1UL << AFIO_MAPR_I2C1_REMAP_Pos) /*!< 0x00000002 */
#define AFIO_MAPR_I2C1_REMAP                 AFIO_MAPR_I2C1_REMAP_Msk          /*!< I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP_Pos           (2U)
#define AFIO_MAPR_USART1_REMAP_Msk           (0x1UL << AFIO_MAPR_USART1_REMAP_Pos) /*!< 0x00000004 */
#define AFIO_MAPR_USART1_REMAP               AFIO_MAPR_USART1_REMAP_Msk        /*!< USART1 remapping */
#define AFIO_MAPR_USART2_REMAP_Pos           (3U)
#define AFIO_MAPR_USART2_REMAP_Msk           (0x1UL << AFIO_MAPR_USART2_REMAP_Pos) /*!< 0x00000008 */
#define AFIO_MAPR_USART2_REMAP               AFIO_MAPR_USART2_REMAP_Msk        /*!< USART2 remapping */

#define AFIO_MAPR_USART3_REMAP_Pos           (4U)
#define AFIO_MAPR_USART3_REMAP_Msk           (0x3UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000030 */
#define AFIO_MAPR_USART3_REMAP               AFIO_MAPR_USART3_REMAP_Msk        /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             (0x1UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000010 */
#define AFIO_MAPR_USART3_REMAP_1             (0x2UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000020 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       0x00000000U                          /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Pos (4U)
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Pos) /*!< 0x00000010 */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Msk /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP_Pos (4U)
#define AFIO_MAPR_USART3_REMAP_FULLREMAP_Msk (0x3UL << AFIO_MAPR_USART3_REMAP_FULLREMAP_Pos) /*!< 0x00000030 */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     AFIO_MAPR_USART3_REMAP_FULLREMAP_Msk /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP_Pos             (6U)
#define AFIO_MAPR_TIM1_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x000000C0 */
#define AFIO_MAPR_TIM1_REMAP                 AFIO_MAPR_TIM1_REMAP_Msk          /*!< TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               (0x1UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x00000040 */
#define AFIO_MAPR_TIM1_REMAP_1               (0x2UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x00000080 */

/*!< TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         0x00000000U                          /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Pos (6U)
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Pos) /*!< 0x00000040 */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Msk /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP_Pos   (6U)
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM1_REMAP_FULLREMAP_Pos) /*!< 0x000000C0 */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       AFIO_MAPR_TIM1_REMAP_FULLREMAP_Msk /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP_Pos             (8U)
#define AFIO_MAPR_TIM2_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000300 */
#define AFIO_MAPR_TIM2_REMAP                 AFIO_MAPR_TIM2_REMAP_Msk          /*!< TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               (0x1UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000100 */
#define AFIO_MAPR_TIM2_REMAP_1               (0x2UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000200 */

/*!< TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         0x00000000U                          /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Pos (8U)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Msk (0x1UL << AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Pos) /*!< 0x00000100 */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Msk /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Pos (9U)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Msk (0x1UL << AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Pos) /*!< 0x00000200 */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Msk /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP_Pos   (8U)
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM2_REMAP_FULLREMAP_Pos) /*!< 0x00000300 */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       AFIO_MAPR_TIM2_REMAP_FULLREMAP_Msk /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP_Pos             (10U)
#define AFIO_MAPR_TIM3_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000C00 */
#define AFIO_MAPR_TIM3_REMAP                 AFIO_MAPR_TIM3_REMAP_Msk          /*!< TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               (0x1UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000400 */
#define AFIO_MAPR_TIM3_REMAP_1               (0x2UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000800 */

/*!< TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         0x00000000U                          /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Pos (11U)
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Pos) /*!< 0x00000800 */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Msk /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP_Pos   (10U)
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM3_REMAP_FULLREMAP_Pos) /*!< 0x00000C00 */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       AFIO_MAPR_TIM3_REMAP_FULLREMAP_Msk /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP_Pos             (12U)
#define AFIO_MAPR_TIM4_REMAP_Msk             (0x1UL << AFIO_MAPR_TIM4_REMAP_Pos) /*!< 0x00001000 */
#define AFIO_MAPR_TIM4_REMAP                 AFIO_MAPR_TIM4_REMAP_Msk          /*!< TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP_Pos              (13U)
#define AFIO_MAPR_CAN_REMAP_Msk              (0x3UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00006000 */
#define AFIO_MAPR_CAN_REMAP                  AFIO_MAPR_CAN_REMAP_Msk           /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                (0x1UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00002000 */
#define AFIO_MAPR_CAN_REMAP_1                (0x2UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00004000 */

/*!< CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           0x00000000U                          /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2_Pos       (14U)
#define AFIO_MAPR_CAN_REMAP_REMAP2_Msk       (0x1UL << AFIO_MAPR_CAN_REMAP_REMAP2_Pos) /*!< 0x00004000 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           AFIO_MAPR_CAN_REMAP_REMAP2_Msk    /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3_Pos       (13U)
#define AFIO_MAPR_CAN_REMAP_REMAP3_Msk       (0x3UL << AFIO_MAPR_CAN_REMAP_REMAP3_Pos) /*!< 0x00006000 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           AFIO_MAPR_CAN_REMAP_REMAP3_Msk    /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP_Pos             (15U)
#define AFIO_MAPR_PD01_REMAP_Msk             (0x1UL << AFIO_MAPR_PD01_REMAP_Pos) /*!< 0x00008000 */
#define AFIO_MAPR_PD01_REMAP                 AFIO_MAPR_PD01_REMAP_Msk          /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP_Pos         (16U)
#define AFIO_MAPR_TIM5CH4_IREMAP_Msk         (0x1UL << AFIO_MAPR_TIM5CH4_IREMAP_Pos) /*!< 0x00010000 */
#define AFIO_MAPR_TIM5CH4_IREMAP             AFIO_MAPR_TIM5CH4_IREMAP_Msk      /*!< TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP_Pos     (17U)
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP_Msk     (0x1UL << AFIO_MAPR_ADC1_ETRGINJ_REMAP_Pos) /*!< 0x00020000 */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP         AFIO_MAPR_ADC1_ETRGINJ_REMAP_Msk  /*!< ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP_Pos     (18U)
#define AFIO_MAPR_ADC1_ETRGREG_REMAP_Msk     (0x1UL << AFIO_MAPR_ADC1_ETRGREG_REMAP_Pos) /*!< 0x00040000 */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP         AFIO_MAPR_ADC1_ETRGREG_REMAP_Msk  /*!< ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP_Pos     (19U)
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP_Msk     (0x1UL << AFIO_MAPR_ADC2_ETRGINJ_REMAP_Pos) /*!< 0x00080000 */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP         AFIO_MAPR_ADC2_ETRGINJ_REMAP_Msk  /*!< ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP_Pos     (20U)
#define AFIO_MAPR_ADC2_ETRGREG_REMAP_Msk     (0x1UL << AFIO_MAPR_ADC2_ETRGREG_REMAP_Pos) /*!< 0x00100000 */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP         AFIO_MAPR_ADC2_ETRGREG_REMAP_Msk  /*!< ADC 2 External Trigger Regular Conversion remapping */

/*!< SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG_Pos                (24U)
#define AFIO_MAPR_SWJ_CFG_Msk                (0x7UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x07000000 */
#define AFIO_MAPR_SWJ_CFG                    AFIO_MAPR_SWJ_CFG_Msk             /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  (0x1UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x01000000 */
#define AFIO_MAPR_SWJ_CFG_1                  (0x2UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x02000000 */
#define AFIO_MAPR_SWJ_CFG_2                  (0x4UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x04000000 */

#define AFIO_MAPR_SWJ_CFG_RESET              0x00000000U                          /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST_Pos       (24U)
#define AFIO_MAPR_SWJ_CFG_NOJNTRST_Msk       (0x1UL << AFIO_MAPR_SWJ_CFG_NOJNTRST_Pos) /*!< 0x01000000 */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           AFIO_MAPR_SWJ_CFG_NOJNTRST_Msk    /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Pos    (25U)
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Msk    (0x1UL << AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Pos) /*!< 0x02000000 */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Msk /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE_Pos        (26U)
#define AFIO_MAPR_SWJ_CFG_DISABLE_Msk        (0x1UL << AFIO_MAPR_SWJ_CFG_DISABLE_Pos) /*!< 0x04000000 */
#define AFIO_MAPR_SWJ_CFG_DISABLE            AFIO_MAPR_SWJ_CFG_DISABLE_Msk     /*!< JTAG-DP Disabled and SW-DP Disabled */


/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0_Pos               (0U)
#define AFIO_EXTICR1_EXTI0_Msk               (0xFUL << AFIO_EXTICR1_EXTI0_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR1_EXTI0                   AFIO_EXTICR1_EXTI0_Msk            /*!< EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1_Pos               (4U)
#define AFIO_EXTICR1_EXTI1_Msk               (0xFUL << AFIO_EXTICR1_EXTI1_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR1_EXTI1                   AFIO_EXTICR1_EXTI1_Msk            /*!< EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2_Pos               (8U)
#define AFIO_EXTICR1_EXTI2_Msk               (0xFUL << AFIO_EXTICR1_EXTI2_Pos)  /*!< 0x00000F00 */
#define AFIO_EXTICR1_EXTI2                   AFIO_EXTICR1_EXTI2_Msk            /*!< EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3_Pos               (12U)
#define AFIO_EXTICR1_EXTI3_Msk               (0xFUL << AFIO_EXTICR1_EXTI3_Pos)  /*!< 0x0000F000 */
#define AFIO_EXTICR1_EXTI3                   AFIO_EXTICR1_EXTI3_Msk            /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                0x00000000U                          /*!< PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR1_EXTI0_PB                AFIO_EXTICR1_EXTI0_PB_Msk         /*!< PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC_Pos            (1U)
#define AFIO_EXTICR1_EXTI0_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR1_EXTI0_PC                AFIO_EXTICR1_EXTI0_PC_Msk         /*!< PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI0_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR1_EXTI0_PD                AFIO_EXTICR1_EXTI0_PD_Msk         /*!< PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE_Pos            (2U)
#define AFIO_EXTICR1_EXTI0_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR1_EXTI0_PE                AFIO_EXTICR1_EXTI0_PE_Msk         /*!< PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI0_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR1_EXTI0_PF                AFIO_EXTICR1_EXTI0_PF_Msk         /*!< PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG_Pos            (1U)
#define AFIO_EXTICR1_EXTI0_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI0_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR1_EXTI0_PG                AFIO_EXTICR1_EXTI0_PG_Msk         /*!< PG[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                0x00000000U                          /*!< PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR1_EXTI1_PB                AFIO_EXTICR1_EXTI1_PB_Msk         /*!< PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC_Pos            (5U)
#define AFIO_EXTICR1_EXTI1_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR1_EXTI1_PC                AFIO_EXTICR1_EXTI1_PC_Msk         /*!< PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI1_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR1_EXTI1_PD                AFIO_EXTICR1_EXTI1_PD_Msk         /*!< PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE_Pos            (6U)
#define AFIO_EXTICR1_EXTI1_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR1_EXTI1_PE                AFIO_EXTICR1_EXTI1_PE_Msk         /*!< PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI1_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR1_EXTI1_PF                AFIO_EXTICR1_EXTI1_PF_Msk         /*!< PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG_Pos            (5U)
#define AFIO_EXTICR1_EXTI1_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI1_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR1_EXTI1_PG                AFIO_EXTICR1_EXTI1_PG_Msk         /*!< PG[1] pin */

/*!< EXTI2 configuration */
#define AFIO_EXTICR1_EXTI2_PA                0x00000000U                          /*!< PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR1_EXTI2_PB                AFIO_EXTICR1_EXTI2_PB_Msk         /*!< PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC_Pos            (9U)
#define AFIO_EXTICR1_EXTI2_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR1_EXTI2_PC                AFIO_EXTICR1_EXTI2_PC_Msk         /*!< PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI2_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR1_EXTI2_PD                AFIO_EXTICR1_EXTI2_PD_Msk         /*!< PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE_Pos            (10U)
#define AFIO_EXTICR1_EXTI2_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR1_EXTI2_PE                AFIO_EXTICR1_EXTI2_PE_Msk         /*!< PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI2_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR1_EXTI2_PF                AFIO_EXTICR1_EXTI2_PF_Msk         /*!< PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG_Pos            (9U)
#define AFIO_EXTICR1_EXTI2_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI2_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR1_EXTI2_PG                AFIO_EXTICR1_EXTI2_PG_Msk         /*!< PG[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                0x00000000U                          /*!< PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR1_EXTI3_PB                AFIO_EXTICR1_EXTI3_PB_Msk         /*!< PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC_Pos            (13U)
#define AFIO_EXTICR1_EXTI3_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR1_EXTI3_PC                AFIO_EXTICR1_EXTI3_PC_Msk         /*!< PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI3_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR1_EXTI3_PD                AFIO_EXTICR1_EXTI3_PD_Msk         /*!< PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE_Pos            (14U)
#define AFIO_EXTICR1_EXTI3_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR1_EXTI3_PE                AFIO_EXTICR1_EXTI3_PE_Msk         /*!< PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI3_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR1_EXTI3_PF                AFIO_EXTICR1_EXTI3_PF_Msk         /*!< PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG_Pos            (13U)
#define AFIO_EXTICR1_EXTI3_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI3_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR1_EXTI3_PG                AFIO_EXTICR1_EXTI3_PG_Msk         /*!< PG[3] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4_Pos               (0U)
#define AFIO_EXTICR2_EXTI4_Msk               (0xFUL << AFIO_EXTICR2_EXTI4_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR2_EXTI4                   AFIO_EXTICR2_EXTI4_Msk            /*!< EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5_Pos               (4U)
#define AFIO_EXTICR2_EXTI5_Msk               (0xFUL << AFIO_EXTICR2_EXTI5_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR2_EXTI5                   AFIO_EXTICR2_EXTI5_Msk            /*!< EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6_Pos               (8U)
#define AFIO_EXTICR2_EXTI6_Msk               (0xFUL << AFIO_EXTICR2_EXTI6_Pos)  /*!< 0x00000F00 */
#define AFIO_EXTICR2_EXTI6                   AFIO_EXTICR2_EXTI6_Msk            /*!< EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7_Pos               (12U)
#define AFIO_EXTICR2_EXTI7_Msk               (0xFUL << AFIO_EXTICR2_EXTI7_Pos)  /*!< 0x0000F000 */
#define AFIO_EXTICR2_EXTI7                   AFIO_EXTICR2_EXTI7_Msk            /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                0x00000000U                          /*!< PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR2_EXTI4_PB                AFIO_EXTICR2_EXTI4_PB_Msk         /*!< PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC_Pos            (1U)
#define AFIO_EXTICR2_EXTI4_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR2_EXTI4_PC                AFIO_EXTICR2_EXTI4_PC_Msk         /*!< PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI4_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR2_EXTI4_PD                AFIO_EXTICR2_EXTI4_PD_Msk         /*!< PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE_Pos            (2U)
#define AFIO_EXTICR2_EXTI4_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR2_EXTI4_PE                AFIO_EXTICR2_EXTI4_PE_Msk         /*!< PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI4_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR2_EXTI4_PF                AFIO_EXTICR2_EXTI4_PF_Msk         /*!< PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG_Pos            (1U)
#define AFIO_EXTICR2_EXTI4_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI4_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR2_EXTI4_PG                AFIO_EXTICR2_EXTI4_PG_Msk         /*!< PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                0x00000000U                          /*!< PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR2_EXTI5_PB                AFIO_EXTICR2_EXTI5_PB_Msk         /*!< PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC_Pos            (5U)
#define AFIO_EXTICR2_EXTI5_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR2_EXTI5_PC                AFIO_EXTICR2_EXTI5_PC_Msk         /*!< PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI5_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR2_EXTI5_PD                AFIO_EXTICR2_EXTI5_PD_Msk         /*!< PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE_Pos            (6U)
#define AFIO_EXTICR2_EXTI5_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR2_EXTI5_PE                AFIO_EXTICR2_EXTI5_PE_Msk         /*!< PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI5_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR2_EXTI5_PF                AFIO_EXTICR2_EXTI5_PF_Msk         /*!< PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG_Pos            (5U)
#define AFIO_EXTICR2_EXTI5_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI5_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR2_EXTI5_PG                AFIO_EXTICR2_EXTI5_PG_Msk         /*!< PG[5] pin */

/*!< EXTI6 configuration */
#define AFIO_EXTICR2_EXTI6_PA                0x00000000U                          /*!< PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR2_EXTI6_PB                AFIO_EXTICR2_EXTI6_PB_Msk         /*!< PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC_Pos            (9U)
#define AFIO_EXTICR2_EXTI6_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR2_EXTI6_PC                AFIO_EXTICR2_EXTI6_PC_Msk         /*!< PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI6_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR2_EXTI6_PD                AFIO_EXTICR2_EXTI6_PD_Msk         /*!< PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE_Pos            (10U)
#define AFIO_EXTICR2_EXTI6_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR2_EXTI6_PE                AFIO_EXTICR2_EXTI6_PE_Msk         /*!< PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI6_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR2_EXTI6_PF                AFIO_EXTICR2_EXTI6_PF_Msk         /*!< PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG_Pos            (9U)
#define AFIO_EXTICR2_EXTI6_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI6_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR2_EXTI6_PG                AFIO_EXTICR2_EXTI6_PG_Msk         /*!< PG[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                0x00000000U                          /*!< PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR2_EXTI7_PB                AFIO_EXTICR2_EXTI7_PB_Msk         /*!< PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC_Pos            (13U)
#define AFIO_EXTICR2_EXTI7_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR2_EXTI7_PC                AFIO_EXTICR2_EXTI7_PC_Msk         /*!< PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI7_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR2_EXTI7_PD                AFIO_EXTICR2_EXTI7_PD_Msk         /*!< PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE_Pos            (14U)
#define AFIO_EXTICR2_EXTI7_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR2_EXTI7_PE                AFIO_EXTICR2_EXTI7_PE_Msk         /*!< PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI7_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR2_EXTI7_PF                AFIO_EXTICR2_EXTI7_PF_Msk         /*!< PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG_Pos            (13U)
#define AFIO_EXTICR2_EXTI7_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI7_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR2_EXTI7_PG                AFIO_EXTICR2_EXTI7_PG_Msk         /*!< PG[7] pin */

/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8_Pos               (0U)
#define AFIO_EXTICR3_EXTI8_Msk               (0xFUL << AFIO_EXTICR3_EXTI8_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR3_EXTI8                   AFIO_EXTICR3_EXTI8_Msk            /*!< EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9_Pos               (4U)
#define AFIO_EXTICR3_EXTI9_Msk               (0xFUL << AFIO_EXTICR3_EXTI9_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR3_EXTI9                   AFIO_EXTICR3_EXTI9_Msk            /*!< EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10_Pos              (8U)
#define AFIO_EXTICR3_EXTI10_Msk              (0xFUL << AFIO_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define AFIO_EXTICR3_EXTI10                  AFIO_EXTICR3_EXTI10_Msk           /*!< EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11_Pos              (12U)
#define AFIO_EXTICR3_EXTI11_Msk              (0xFUL << AFIO_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define AFIO_EXTICR3_EXTI11                  AFIO_EXTICR3_EXTI11_Msk           /*!< EXTI 11 configuration */

/*!< EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                0x00000000U                          /*!< PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PB_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR3_EXTI8_PB                AFIO_EXTICR3_EXTI8_PB_Msk         /*!< PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC_Pos            (1U)
#define AFIO_EXTICR3_EXTI8_PC_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR3_EXTI8_PC                AFIO_EXTICR3_EXTI8_PC_Msk         /*!< PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PD_Msk            (0x3UL << AFIO_EXTICR3_EXTI8_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR3_EXTI8_PD                AFIO_EXTICR3_EXTI8_PD_Msk         /*!< PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE_Pos            (2U)
#define AFIO_EXTICR3_EXTI8_PE_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR3_EXTI8_PE                AFIO_EXTICR3_EXTI8_PE_Msk         /*!< PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PF_Msk            (0x5UL << AFIO_EXTICR3_EXTI8_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR3_EXTI8_PF                AFIO_EXTICR3_EXTI8_PF_Msk         /*!< PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG_Pos            (1U)
#define AFIO_EXTICR3_EXTI8_PG_Msk            (0x3UL << AFIO_EXTICR3_EXTI8_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR3_EXTI8_PG                AFIO_EXTICR3_EXTI8_PG_Msk         /*!< PG[8] pin */

/*!< EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                0x00000000U                          /*!< PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PB_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR3_EXTI9_PB                AFIO_EXTICR3_EXTI9_PB_Msk         /*!< PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC_Pos            (5U)
#define AFIO_EXTICR3_EXTI9_PC_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR3_EXTI9_PC                AFIO_EXTICR3_EXTI9_PC_Msk         /*!< PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PD_Msk            (0x3UL << AFIO_EXTICR3_EXTI9_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR3_EXTI9_PD                AFIO_EXTICR3_EXTI9_PD_Msk         /*!< PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE_Pos            (6U)
#define AFIO_EXTICR3_EXTI9_PE_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR3_EXTI9_PE                AFIO_EXTICR3_EXTI9_PE_Msk         /*!< PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PF_Msk            (0x5UL << AFIO_EXTICR3_EXTI9_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR3_EXTI9_PF                AFIO_EXTICR3_EXTI9_PF_Msk         /*!< PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG_Pos            (5U)
#define AFIO_EXTICR3_EXTI9_PG_Msk            (0x3UL << AFIO_EXTICR3_EXTI9_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR3_EXTI9_PG                AFIO_EXTICR3_EXTI9_PG_Msk         /*!< PG[9] pin */

/*!< EXTI10 configuration */
#define AFIO_EXTICR3_EXTI10_PA               0x00000000U                          /*!< PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PB_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR3_EXTI10_PB               AFIO_EXTICR3_EXTI10_PB_Msk        /*!< PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC_Pos           (9U)
#define AFIO_EXTICR3_EXTI10_PC_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR3_EXTI10_PC               AFIO_EXTICR3_EXTI10_PC_Msk        /*!< PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PD_Msk           (0x3UL << AFIO_EXTICR3_EXTI10_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR3_EXTI10_PD               AFIO_EXTICR3_EXTI10_PD_Msk        /*!< PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE_Pos           (10U)
#define AFIO_EXTICR3_EXTI10_PE_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR3_EXTI10_PE               AFIO_EXTICR3_EXTI10_PE_Msk        /*!< PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PF_Msk           (0x5UL << AFIO_EXTICR3_EXTI10_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR3_EXTI10_PF               AFIO_EXTICR3_EXTI10_PF_Msk        /*!< PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG_Pos           (9U)
#define AFIO_EXTICR3_EXTI10_PG_Msk           (0x3UL << AFIO_EXTICR3_EXTI10_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR3_EXTI10_PG               AFIO_EXTICR3_EXTI10_PG_Msk        /*!< PG[10] pin */

/*!< EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               0x00000000U                          /*!< PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PB_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR3_EXTI11_PB               AFIO_EXTICR3_EXTI11_PB_Msk        /*!< PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC_Pos           (13U)
#define AFIO_EXTICR3_EXTI11_PC_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR3_EXTI11_PC               AFIO_EXTICR3_EXTI11_PC_Msk        /*!< PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PD_Msk           (0x3UL << AFIO_EXTICR3_EXTI11_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR3_EXTI11_PD               AFIO_EXTICR3_EXTI11_PD_Msk        /*!< PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE_Pos           (14U)
#define AFIO_EXTICR3_EXTI11_PE_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR3_EXTI11_PE               AFIO_EXTICR3_EXTI11_PE_Msk        /*!< PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PF_Msk           (0x5UL << AFIO_EXTICR3_EXTI11_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR3_EXTI11_PF               AFIO_EXTICR3_EXTI11_PF_Msk        /*!< PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG_Pos           (13U)
#define AFIO_EXTICR3_EXTI11_PG_Msk           (0x3UL << AFIO_EXTICR3_EXTI11_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR3_EXTI11_PG               AFIO_EXTICR3_EXTI11_PG_Msk        /*!< PG[11] pin */

/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12_Pos              (0U)
#define AFIO_EXTICR4_EXTI12_Msk              (0xFUL << AFIO_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define AFIO_EXTICR4_EXTI12                  AFIO_EXTICR4_EXTI12_Msk           /*!< EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13_Pos              (4U)
#define AFIO_EXTICR4_EXTI13_Msk              (0xFUL << AFIO_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define AFIO_EXTICR4_EXTI13                  AFIO_EXTICR4_EXTI13_Msk           /*!< EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14_Pos              (8U)
#define AFIO_EXTICR4_EXTI14_Msk              (0xFUL << AFIO_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define AFIO_EXTICR4_EXTI14                  AFIO_EXTICR4_EXTI14_Msk           /*!< EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15_Pos              (12U)
#define AFIO_EXTICR4_EXTI15_Msk              (0xFUL << AFIO_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define AFIO_EXTICR4_EXTI15                  AFIO_EXTICR4_EXTI15_Msk           /*!< EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               0x00000000U                          /*!< PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR4_EXTI12_PB               AFIO_EXTICR4_EXTI12_PB_Msk        /*!< PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC_Pos           (1U)
#define AFIO_EXTICR4_EXTI12_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR4_EXTI12_PC               AFIO_EXTICR4_EXTI12_PC_Msk        /*!< PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI12_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR4_EXTI12_PD               AFIO_EXTICR4_EXTI12_PD_Msk        /*!< PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE_Pos           (2U)
#define AFIO_EXTICR4_EXTI12_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR4_EXTI12_PE               AFIO_EXTICR4_EXTI12_PE_Msk        /*!< PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI12_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR4_EXTI12_PF               AFIO_EXTICR4_EXTI12_PF_Msk        /*!< PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG_Pos           (1U)
#define AFIO_EXTICR4_EXTI12_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI12_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR4_EXTI12_PG               AFIO_EXTICR4_EXTI12_PG_Msk        /*!< PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               0x00000000U                          /*!< PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR4_EXTI13_PB               AFIO_EXTICR4_EXTI13_PB_Msk        /*!< PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC_Pos           (5U)
#define AFIO_EXTICR4_EXTI13_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR4_EXTI13_PC               AFIO_EXTICR4_EXTI13_PC_Msk        /*!< PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI13_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR4_EXTI13_PD               AFIO_EXTICR4_EXTI13_PD_Msk        /*!< PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE_Pos           (6U)
#define AFIO_EXTICR4_EXTI13_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR4_EXTI13_PE               AFIO_EXTICR4_EXTI13_PE_Msk        /*!< PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI13_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR4_EXTI13_PF               AFIO_EXTICR4_EXTI13_PF_Msk        /*!< PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG_Pos           (5U)
#define AFIO_EXTICR4_EXTI13_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI13_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR4_EXTI13_PG               AFIO_EXTICR4_EXTI13_PG_Msk        /*!< PG[13] pin */

/*!< EXTI14 configuration */
#define AFIO_EXTICR4_EXTI14_PA               0x00000000U                          /*!< PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR4_EXTI14_PB               AFIO_EXTICR4_EXTI14_PB_Msk        /*!< PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC_Pos           (9U)
#define AFIO_EXTICR4_EXTI14_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR4_EXTI14_PC               AFIO_EXTICR4_EXTI14_PC_Msk        /*!< PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI14_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR4_EXTI14_PD               AFIO_EXTICR4_EXTI14_PD_Msk        /*!< PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE_Pos           (10U)
#define AFIO_EXTICR4_EXTI14_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR4_EXTI14_PE               AFIO_EXTICR4_EXTI14_PE_Msk        /*!< PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI14_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR4_EXTI14_PF               AFIO_EXTICR4_EXTI14_PF_Msk        /*!< PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG_Pos           (9U)
#define AFIO_EXTICR4_EXTI14_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI14_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR4_EXTI14_PG               AFIO_EXTICR4_EXTI14_PG_Msk        /*!< PG[14] pin */

/*!< EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               0x00000000U                          /*!< PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR4_EXTI15_PB               AFIO_EXTICR4_EXTI15_PB_Msk        /*!< PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC_Pos           (13U)
#define AFIO_EXTICR4_EXTI15_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR4_EXTI15_PC               AFIO_EXTICR4_EXTI15_PC_Msk        /*!< PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI15_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR4_EXTI15_PD               AFIO_EXTICR4_EXTI15_PD_Msk        /*!< PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE_Pos           (14U)
#define AFIO_EXTICR4_EXTI15_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR4_EXTI15_PE               AFIO_EXTICR4_EXTI15_PE_Msk        /*!< PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI15_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR4_EXTI15_PF               AFIO_EXTICR4_EXTI15_PF_Msk        /*!< PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG_Pos           (13U)
#define AFIO_EXTICR4_EXTI15_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI15_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR4_EXTI15_PG               AFIO_EXTICR4_EXTI15_PG_Msk        /*!< PG[15] pin */

/******************  Bit definition for AFIO_MAPR2 register  ******************/


#define AFIO_MAPR2_FSMC_NADV_REMAP_Pos       (10U)
#define AFIO_MAPR2_FSMC_NADV_REMAP_Msk       (0x1UL << AFIO_MAPR2_FSMC_NADV_REMAP_Pos) /*!< 0x00000400 */
#define AFIO_MAPR2_FSMC_NADV_REMAP           AFIO_MAPR2_FSMC_NADV_REMAP_Msk    /*!< FSMC NADV remapping */




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


/**************************************************************************************************************
 * 																											  *
 * 											 User Macro Function				 							  *
 * 																											  *
 **************************************************************************************************************/

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WAIT_FLAG_SET(REG, FLAG)		do{  while( !((REG) & (FLAG)) ); }while(0)

#define WAIT_FLAG_CLEAR(REG, FLAG)		do{  while( ((REG) & (FLAG)) ); }while(0)


/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Header file					 						  *
 * 																											  *
 **************************************************************************************************************/

#include "stm32f103xx_rcc_driver.h"
#include "stm32f103xx_gpio_driver.h"
#include "core.h"
#include "it.h"

/**************************************************************************************************************
 * 																											  *
 * 										Peripheral Macro defifnitions				 						  *
 * 						(which are too few to include entire Peripheral Header file) 						  *
 * 																											  *
 **************************************************************************************************************/

/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define FLASH_LATENCY_0            0x00000000U               /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1            FLASH_ACR_LATENCY_0       /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2            FLASH_ACR_LATENCY_1       /*!< FLASH Two Latency cycles */





#endif /* STM32F103XX_H_ */
