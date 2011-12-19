/**
  ******************************************************************************
  * @file    stm32_eval.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   Header file for stm32_eval.c module.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_EVAL_H
#define __STM32_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup Utilities
  * @{
  */ 
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @defgroup STM32_EVAL_Abstraction_Layer
  * @{
  */
  
/** @defgroup STM32_EVAL_HARDWARE_RESOURCES
  * @{
  */

/**
@code  
 The table below gives an overview of the hardware resources supported by each 
 STM32 EVAL board.
     - LCD: TFT Color LCD (Parallel (FSMC) and Serial (SPI))
     - IOE: IO Expander on I2C
     - sFLASH: serial SPI FLASH (M25Pxxx)
     - sEE: serial I2C EEPROM (M24C08, M24C32, M24C64)
     - TSENSOR: Temperature Sensor (LM75)
     - SD: SD Card memory (SPI and SDIO (SD Card MODE)) 
  =================================================================================================================+
    STM32 EVAL     | LED | Buttons  | Com Ports |    LCD    | IOE  | sFLASH | sEE | TSENSOR | SD (SPI) | SD(SDIO)  |
  =================================================================================================================+
   STM3210B-EVAL   |  4  |    8     |     2     | YES (SPI) | NO   |  YES   | NO  |   YES   |    YES   |    NO     |
  -----------------------------------------------------------------------------------------------------------------+
   STM3210E-EVAL   |  4  |    8     |     2     | YES (FSMC)| NO   |  YES   | NO  |   YES   |    NO    |    YES    |
  -----------------------------------------------------------------------------------------------------------------+
   STM3210C-EVAL   |  4  |    3     |     1     | YES (SPI) | YES  |  NO    | YES |   NO    |    YES   |    NO     |
  -----------------------------------------------------------------------------------------------------------------+
   STM32100B-EVAL  |  4  |    8     |     2     | YES (SPI) | NO   |  YES   | NO  |   YES   |    YES   |    NO     |
  -----------------------------------------------------------------------------------------------------------------+
   STM32L152-EVAL  |  4  |    8     |     2     | YES (SPI) | NO   |  NO    | NO  |   YES   |    YES   |    NO     |
  -----------------------------------------------------------------------------------------------------------------+
   STM32100E-EVAL  |  4  |    8     |     2     | YES (FSMC)| YES  |  YES   | YES |   YES   |    YES   |    NO     |
  =================================================================================================================+
@endcode
*/

/**
  * @}
  */
  
/** @defgroup STM32_EVAL_Exported_Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3,
  LED5 = 4
} Led_TypeDef;

typedef enum 
{  
  BUTTON_WAKEUP = 0,
  BUTTON_TAMPER = 1,
  BUTTON_USER1 = 2,
  BUTTON_USER2 = 3,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum 
{
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;   
/**
  * @}
  */ 
  
/** @defgroup STM32_EVAL_Exported_Constants
  * @{
  */

/** 
  * @brief  Uncomment the line corresponding to the STMicroelectronics evaluation
  *   board used in your application.
  *   
  *  Tip: To avoid modifying this file each time you need to switch between these
  *       boards, you can define the board in your toolchain compiler preprocessor.    
  */ 
#if !defined (USE_STM32100B_EVAL) && !defined (USE_STM3210B_EVAL) &&  !defined (USE_STM3210E_EVAL)\
   &&  !defined (USE_STM3210C_EVAL) &&  !defined (USE_STM32L152_EVAL) &&  !defined (USE_STM32100E_EVAL)
 //#define USE_STM32100B_EVAL
 //#define USE_STM3210B_EVAL
 //#define USE_STM3210E_EVAL
 //#define USE_STM3210C_EVAL
 //#define USE_STM32L152_EVAL
 //#define USE_STM32100E_EVAL
#endif

#ifdef USE_STM32100B_EVAL
 #include "stm32f10x.h"
 #include "stm32100b_eval/stm32100b_eval.h"
#elif defined USE_STM3210B_EVAL
 #include "stm32f10x.h"
 #include "stm3210b_eval/stm3210b_eval.h" 
#elif defined USE_STM3210E_EVAL
 #include "stm32f10x.h"
 #include "stm3210e_eval.h"
#elif defined USE_STM3210C_EVAL
 #include "stm32f10x.h"
 #include "stm3210c_eval/stm3210c_eval.h"
#elif defined USE_STM32L152_EVAL
 #include "stm32l1xx.h"
 #include "stm32l152_eval/stm32l152_eval.h" 
#elif defined USE_STM32100E_EVAL
 #include "stm32f10x.h"
 #include "stm32100e_eval/stm32100e_eval.h"
#else 
 #error "Please select first the STM32 EVAL board to be used (in stm32_eval.h)"
#endif                      

/**
  * @}
  */ 

/** @defgroup STM32_EVAL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32_EVAL_Exported_Functions
  * @{
  */ 
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif


#endif /* __STM32_EVAL_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */   

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
