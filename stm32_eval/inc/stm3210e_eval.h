/**
  ******************************************************************************
  * @file    stm3210e_eval.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file contains definitions for STM3210E_EVAL's Leds, push-buttons
  *          COM ports, sFLASH (on SPI) and Temperature Sensor LM75 (on I2C)
  *          hardware resources.  
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
#ifndef __STM3210E_EVAL_H
#define __STM3210E_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32_eval.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_EVAL
  * @{
  */  
  
/** @addtogroup STM3210E_EVAL
  * @{
  */ 

/** @addtogroup STM3210E_EVAL_LOW_LEVEL
  * @{
  */ 
  
/** @defgroup STM3210E_EVAL_LOW_LEVEL_Exported_Types
  * @{
  */
/**
  * @}
  */ 

/** @defgroup STM3210E_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */ 
/** @addtogroup STM3210E_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             5

#define LED1_PIN                         GPIO_Pin_6
#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK                    RCC_APB2Periph_GPIOF  
  
#define LED2_PIN                         GPIO_Pin_7
#define LED2_GPIO_PORT                   GPIOF
#define LED2_GPIO_CLK                    RCC_APB2Periph_GPIOF  

#define LED3_PIN                         GPIO_Pin_8  
#define LED3_GPIO_PORT                   GPIOF
#define LED3_GPIO_CLK                    RCC_APB2Periph_GPIOF  

#define LED4_PIN                         GPIO_Pin_9
#define LED4_GPIO_PORT                   GPIOF
#define LED4_GPIO_CLK                    RCC_APB2Periph_GPIOF

#define LED5_PIN                         GPIO_Pin_10
#define LED5_GPIO_PORT                   GPIOF
#define LED5_GPIO_CLK                    RCC_APB2Periph_GPIOF

/**
  * @}
  */
  
/** @addtogroup STM3210E_EVAL_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          4

/**
 * @brief Wakeup push-button
 */
#define WAKEUP_BUTTON_PIN                GPIO_Pin_0
#define WAKEUP_BUTTON_GPIO_PORT          GPIOA
#define WAKEUP_BUTTON_GPIO_CLK           RCC_APB2Periph_GPIOA
#define WAKEUP_BUTTON_EXTI_LINE          EXTI_Line0
#define WAKEUP_BUTTON_EXTI_PORT_SOURCE   GPIO_PortSourceGPIOA
#define WAKEUP_BUTTON_EXTI_PIN_SOURCE    GPIO_PinSource0
#define WAKEUP_BUTTON_EXTI_IRQn          EXTI0_IRQn 
/**
 * @brief Tamper push-button
 */
#define TAMPER_BUTTON_PIN                GPIO_Pin_13
#define TAMPER_BUTTON_GPIO_PORT          GPIOC
#define TAMPER_BUTTON_GPIO_CLK           RCC_APB2Periph_GPIOC
#define TAMPER_BUTTON_EXTI_LINE          EXTI_Line13
#define TAMPER_BUTTON_EXTI_PORT_SOURCE   GPIO_PortSourceGPIOC
#define TAMPER_BUTTON_EXTI_PIN_SOURCE    GPIO_PinSource13
#define TAMPER_BUTTON_EXTI_IRQn          EXTI15_10_IRQn 
/**
 * @brief User 1 push-button
 */
#define USER1_BUTTON_PIN                 GPIO_Pin_8
#define USER1_BUTTON_GPIO_PORT           GPIOA
#define USER1_BUTTON_GPIO_CLK            RCC_APB2Periph_GPIOA
#define USER1_BUTTON_EXTI_LINE           EXTI_Line8
#define USER1_BUTTON_EXTI_PORT_SOURCE    GPIO_PortSourceGPIOA
#define USER1_BUTTON_EXTI_PIN_SOURCE     GPIO_PinSource8
#define USER1_BUTTON_EXTI_IRQn           EXTI9_5_IRQn
/**
 * @brief User 2 push-button
 */
#define USER2_BUTTON_PIN                 GPIO_Pin_3
#define USER2_BUTTON_GPIO_PORT           GPIOD
#define USER2_BUTTON_GPIO_CLK            RCC_APB2Periph_GPIOD
#define USER2_BUTTON_EXTI_LINE           EXTI_Line3
#define USER2_BUTTON_EXTI_PORT_SOURCE    GPIO_PortSourceGPIOD
#define USER2_BUTTON_EXTI_PIN_SOURCE     GPIO_PinSource3
#define USER2_BUTTON_EXTI_IRQn           EXTI15_10_IRQn
      
/**
  * @}
  */ 

/** @addtogroup STM3210E_EVAL_LOW_LEVEL_COM
  * @{
  */
#define COMn                             2

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1
#define EVAL_COM1_TX_PIN                 GPIO_Pin_9
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_RX_PIN                 GPIO_Pin_10
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_IRQn                   USART1_IRQn

/**
 * @brief Definition for COM port2, connected to USART2
 */ 
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK                    RCC_APB1Periph_USART2
#define EVAL_COM2_TX_PIN                 GPIO_Pin_2
#define EVAL_COM2_TX_GPIO_PORT           GPIOA
#define EVAL_COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_RX_PIN                 GPIO_Pin_3
#define EVAL_COM2_RX_GPIO_PORT           GPIOA
#define EVAL_COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_IRQn                   USART2_IRQn

/**
  * @}
  */ 

/** @addtogroup STM3210E_EVAL_LOW_LEVEL_SD_FLASH
  * @{
  */
/**
  * @brief  SD FLASH SDIO Interface
  */ 

#define SD_DETECT_PIN                    GPIO_Pin_7                 /* PF.11 */
#define SD_DETECT_GPIO_PORT              GPIOC                       /* GPIOF */
#define SD_DETECT_GPIO_CLK               RCC_APB2Periph_GPIOC

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40018080)
/** 
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
#define SDIO_INIT_CLK_DIV                ((uint8_t)0xB2)
/** 
  * @brief  SDIO Data Transfer Frequency (25MHz max) 
  */
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x00) 

/**
  * @}
  */ 

 /**
   * @}
   */

 /** @addtogroup STM3210E_EVAL_LOW_LEVEL_M25P_FLASH_SPI
   * @{
   */
 /**
   * @brief  M25P FLASH SPI Interface pins
   */
 #define sFLASH_SPI                       SPI1
 #define sFLASH_SPI_CLK                   RCC_APB2Periph_SPI1
 #define sFLASH_SPI_SCK_PIN               GPIO_Pin_5                  /* PA.05 */
 #define sFLASH_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
 #define sFLASH_SPI_SCK_GPIO_CLK          RCC_APB2Periph_GPIOA
 #define sFLASH_SPI_MISO_PIN              GPIO_Pin_6                  /* PA.06 */
 #define sFLASH_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
 #define sFLASH_SPI_MISO_GPIO_CLK         RCC_APB2Periph_GPIOA
 #define sFLASH_SPI_MOSI_PIN              GPIO_Pin_7                  /* PA.07 */
 #define sFLASH_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
 #define sFLASH_SPI_MOSI_GPIO_CLK         RCC_APB2Periph_GPIOA
 #define sFLASH_CS_PIN                    GPIO_Pin_2                  /* PB.02 */
 #define sFLASH_CS_GPIO_PORT              GPIOB                       /* GPIOB */
 #define sFLASH_CS_GPIO_CLK               RCC_APB2Periph_GPIOB

 /**
   * @}
   */
  
 /** @addtogroup STM3210C_EVAL_LOW_LEVEL_I2C_EE
   * @{
   */
 /**
   * @brief  I2C EEPROM Interface pins
   */
 #define sEE_I2C                          I2C1
 #define sEE_I2C_CLK                      RCC_APB1Periph_I2C1
 #define sEE_I2C_SCL_PIN                  GPIO_Pin_6                  /* PB.06 */
 #define sEE_I2C_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
 #define sEE_I2C_SCL_GPIO_CLK             RCC_APB2Periph_GPIOB
 #define sEE_I2C_SDA_PIN                  GPIO_Pin_7                  /* PB.07 */
 #define sEE_I2C_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
 #define sEE_I2C_SDA_GPIO_CLK             RCC_APB2Periph_GPIOB
 #define sEE_M24C64_32

 #define sEE_I2C_DMA                      DMA1
 #define sEE_I2C_DMA_CHANNEL_TX           DMA1_Channel6
 #define sEE_I2C_DMA_CHANNEL_RX           DMA1_Channel7
 #define sEE_I2C_DMA_FLAG_TX_TC           DMA1_IT_TC6
 #define sEE_I2C_DMA_FLAG_TX_GL           DMA1_IT_GL6
 #define sEE_I2C_DMA_FLAG_RX_TC           DMA1_IT_TC7
 #define sEE_I2C_DMA_FLAG_RX_GL           DMA1_IT_GL7
 #define sEE_I2C_DMA_CLK                  RCC_AHBPeriph_DMA1
 #define sEE_I2C_DR_Address               ((uint32_t)0x40005410)
 #define sEE_USE_DMA

 #define sEE_I2C_DMA_TX_IRQn              DMA1_Channel6_IRQn
 #define sEE_I2C_DMA_RX_IRQn              DMA1_Channel7_IRQn
 #define sEE_I2C_DMA_TX_IRQHandler        DMA1_Channel6_IRQHandler
 #define sEE_I2C_DMA_RX_IRQHandler        DMA1_Channel7_IRQHandler
 #define sEE_I2C_DMA_PREPRIO              0
 #define sEE_I2C_DMA_SUBPRIO              0

 #define sEE_DIRECTION_TX                 0
 #define sEE_DIRECTION_RX                 1

 /* Time constant for the delay caclulation allowing to have a millisecond
    incrementing counter. This value should be equal to (System Clock / 1000).
    ie. if system clock = 72MHz then sEE_TIME_CONST should be 72. */
 #define sEE_TIME_CONST                   72

 /**
   * @}
   */
  
/**
  * @}
  */
  
/** @defgroup STM3210E_EVAL_LOW_LEVEL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM3210E_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */ 
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void); 
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
uint32_t SD_DMAEndOfTransferStatus(void);
void sFLASH_LowLevel_DeInit(void);
void sFLASH_LowLevel_Init(void);
void sEE_LowLevel_DeInit(void);
void sEE_LowLevel_Init(void);
void sEE_LowLevel_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction);

/**
  * @}
  */
#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210E_EVAL_H */
/**
  * @}
  */ 

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
