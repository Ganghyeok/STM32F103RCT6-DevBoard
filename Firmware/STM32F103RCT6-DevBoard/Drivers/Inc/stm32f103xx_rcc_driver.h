/*
 * stm32f103xx_rcc_driver.h
 *
 *  Created on: Dec 2, 2020
 *      Author: Ganghyeok Lim
 */

#ifndef STM32F103XX_RCC_DRIVER_H_
#define STM32F103XX_RCC_DRIVER_H_

#include "stm32f103xx.h"


/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;      /*!< PLLState: The new state of the PLL.
                              This parameter can be a value of @ref RCC_PLL_Config */

  uint32_t PLLSource;     /*!< PLLSource: PLL entry clock source.
                              This parameter must be a value of @ref RCC_PLL_Clock_Source */

  uint32_t PLLMUL;        /*!< PLLMUL: Multiplication factor for PLL VCO input clock
                              This parameter must be a value of @ref RCCEx_PLL_Multiplication_Factor */

} RCC_PLLInitTypeDef;



/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                       This parameter can be a value of @ref RCC_Oscillator_Type */

  uint32_t HSEState;              /*!< The new state of the HSE.
                                       This parameter can be a value of @ref RCC_HSE_Config */

  uint32_t HSEPredivValue;       /*!<  The Prediv1 factor value (named PREDIV1 or PLLXTPRE in RM)
                                       This parameter can be a value of @ref RCCEx_Prediv1_Factor */

  uint32_t LSEState;              /*!<  The new state of the LSE.
                                        This parameter can be a value of @ref RCC_LSE_Config */

  uint32_t HSIState;              /*!< The new state of the HSI.
                                       This parameter can be a value of @ref RCC_HSI_Config */

  uint32_t HSICalibrationValue;   /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;              /*!<  The new state of the LSI.
                                        This parameter can be a value of @ref RCC_LSI_Config */

  RCC_PLLInitTypeDef PLL;         /*!< PLL structure parameters */


} RCC_OscInitTypeDef;



/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
} RCC_ClkInitTypeDef;



/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_Exported_Constants RCC Exported Constants
  * @{
  */

/** @defgroup RCCEx_PLL_Multiplication_Factor PLL Multiplication Factor
  * @{
  */
#define RCC_PLL_MUL2                    RCC_CFGR_PLLMULL2
#define RCC_PLL_MUL3                    RCC_CFGR_PLLMULL3
#define RCC_PLL_MUL4                    RCC_CFGR_PLLMULL4
#define RCC_PLL_MUL5                    RCC_CFGR_PLLMULL5
#define RCC_PLL_MUL6                    RCC_CFGR_PLLMULL6
#define RCC_PLL_MUL7                    RCC_CFGR_PLLMULL7
#define RCC_PLL_MUL8                    RCC_CFGR_PLLMULL8
#define RCC_PLL_MUL9                    RCC_CFGR_PLLMULL9
#define RCC_PLL_MUL10                   RCC_CFGR_PLLMULL10
#define RCC_PLL_MUL11                   RCC_CFGR_PLLMULL11
#define RCC_PLL_MUL12                   RCC_CFGR_PLLMULL12
#define RCC_PLL_MUL13                   RCC_CFGR_PLLMULL13
#define RCC_PLL_MUL14                   RCC_CFGR_PLLMULL14
#define RCC_PLL_MUL15                   RCC_CFGR_PLLMULL15
#define RCC_PLL_MUL16                   RCC_CFGR_PLLMULL16


/** @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */
#define RCC_PLLSOURCE_HSI_DIV2      0x00000000U     /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC            /*!< HSE clock selected as PLL entry clock source */


/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U


/** @defgroup RCC_HSE_Config HSE Config
  * @{
  */
#define RCC_HSE_OFF                      0x00000000U                                /*!< HSE clock deactivation */
#define RCC_HSE_ON                       RCC_CR_HSEON                               /*!< HSE clock activation */
#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON)) /*!< External clock source for HSE clock */


/** @defgroup RCCEx_Prediv1_Factor HSE Prediv1 Factor
  * @{
  */
#define RCC_HSE_PREDIV_DIV1              0x00000000U
#define RCC_HSE_PREDIV_DIV2              RCC_CFGR_PLLXTPRE

/** @defgroup RCC_LSE_Config LSE Config
  * @{
  */
#define RCC_LSE_OFF                      0x00000000U                                    /*!< LSE clock deactivation */
#define RCC_LSE_ON                       RCC_BDCR_LSEON                                 /*!< LSE clock activation */
#define RCC_LSE_BYPASS                   ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)) /*!< External clock source for LSE clock */


/** @defgroup RCC_HSI_Config HSI Config
  * @{
  */
#define RCC_HSI_OFF                      0x00000000U                      /*!< HSI clock deactivation */
#define RCC_HSI_ON                       RCC_CR_HSION                     /*!< HSI clock activation */

#define RCC_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */


/** @defgroup RCC_LSI_Config LSI Config
  * @{
  */
#define RCC_LSI_OFF                      0x00000000U              /*!< LSI clock deactivation */
#define RCC_LSI_ON                       RCC_CSR_LSION            /*!< LSI clock activation */


/** @defgroup RCC_PLL_Config PLL Config
  * @{
  */
#define RCC_PLL_NONE                      0x00000000U  /*!< PLL is not configured */
#define RCC_PLL_OFF                       0x00000001U  /*!< PLL deactivation */
#define RCC_PLL_ON                        0x00000002U  /*!< PLL activation */


/** @defgroup RCC_System_Clock_Type System Clock Type
  * @{
  */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */


/** @defgroup RCC_System_Clock_Source System Clock Source
  * @{
  */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI /*!< HSI selected as system clock */
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */


/** @defgroup RCC_System_Clock_Source_Status System Clock Source Status
  * @{
  */
#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */


/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */


/** @defgroup RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
  * @{
  */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */


/** @defgroup RCC_MCO_Index MCO Index
  * @{
  */
#define RCC_MCO1                         0x00000000U
#define RCC_MCO                          RCC_MCO1               /*!< MCO1 to be compliant with other families with 2 MCOs*/


/** @defgroup RCC_MCOx_Clock_Prescaler MCO Clock Prescaler
  * @{
  */
#define RCC_MCODIV_1                    0x00000000U




/** @defgroup RCC_Flag Flags
  *        Elements values convention: XXXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - XXX  : Register index
  *                 - 001: CR register
  *                 - 010: BDCR register
  *                 - 011: CSR register
  * @{
  */
/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos))   /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_PINRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_PINRSTF_Pos))  /*!< PIN reset flag */
#define RCC_FLAG_PORRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_PORRSTF_Pos))  /*!< POR/PDR reset flag */
#define RCC_FLAG_SFTRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_SFTRSTF_Pos))  /*!< Software Reset flag */
#define RCC_FLAG_IWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_IWDGRSTF_Pos)) /*!< Independent Watchdog reset flag */
#define RCC_FLAG_WWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_WWDGRSTF_Pos)) /*!< Window watchdog reset flag */
#define RCC_FLAG_LPWRRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LPWRRSTF_Pos)) /*!< Low-Power reset flag */

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */



/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/

/*
 *	System Clock options
 */
#define SYSCLK_FREQ_16MHZ				16
#define SYSCLK_FREQ_24MHZ				24
#define SYSCLK_FREQ_32MHZ				32
#define SYSCLK_FREQ_40MHZ				40
#define SYSCLK_FREQ_48MHZ				48
#define SYSCLK_FREQ_56MHZ				56
#define SYSCLK_FREQ_64MHZ				64
#define SYSCLK_FREQ_72MHZ				72

/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

/* APB2 peripheral clock enable function */
#define RCC_AFIO_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN) )
#define RCC_GPIOA_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN) )
#define RCC_GPIOB_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN) )
#define RCC_GPIOC_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN) )
#define RCC_GPIOD_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN) )
#define RCC_GPIOE_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPEEN) )
#define RCC_GPIOF_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPFEN) )
#define RCC_GPIOG_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPGEN) )
#define RCC_ADC1_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN) )
#define RCC_ADC2_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC2EN) )
#define RCC_TIM1_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN) )
#define RCC_SPI1_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN) )
#define RCC_TIM8_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN) )
#define RCC_USART1_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN) )
#define RCC_ADC3_CLK_ENABLE()			( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC3EN) )


/* APB1 peripheral clock enable function */
#define RCC_TIM2_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN) )
#define RCC_TIM3_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN) )
#define RCC_TIM4_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN) )
#define RCC_TIM5_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN) )
#define RCC_TIM6_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN) )
#define RCC_TIM7_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN) )
#define RCC_WWDG_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN) )
#define RCC_SPI2_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN) )
#define RCC_SPI3_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN) )
#define RCC_USART2_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN) )
#define RCC_USART3_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN) )
#define RCC_UART4_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN) )
#define RCC_UART5_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN) )
#define RCC_I2C1_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) )
#define RCC_I2C2_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN) )
#define RCC_CAN1_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN) )
#define RCC_PWR_CLK_ENABLE()			( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) )


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
void SystemClock_Config(uint8_t clock_freq);
void Delay_us(uint32_t time_us);
void Delay_ms(uint32_t time_ms);
void Delay_us_new(uint32_t time_us);




#endif /* STM32F103XX_RCC_DRIVER_H_ */
