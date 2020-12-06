/*
 * core.h
 *
 *  Created on: 2020. 12. 6.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_CORE_H_
#define INC_CORE_H_

#include "stm32f103xx.h"



/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IO uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
       uint32_t RESERVED0[24U];
  __IO uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
       uint32_t RSERVED1[24U];
  __IO uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
       uint32_t RESERVED2[24U];
  __IO uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
       uint32_t RESERVED3[24U];
  __IO uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
       uint32_t RESERVED4[56U];
  __IO uint8_t  IPR[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
       uint32_t RESERVED5[644U];
  __IO uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;



/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IO uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IO uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IO uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IO  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;



/* Peripheral Declarations of Core Hardware */
#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct */
#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */
#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */



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

/**************************************************************************************************************
 * 																											  *
 * 											 User Macro Function				 							  *
 * 																											  *
 **************************************************************************************************************/




/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di);





#endif /* INC_CORE_H_ */
