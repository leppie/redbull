/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_istr.h"
#include "stm32_eval_sdio_sd.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
}
*/
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

extern void HandleWakeupButtonDown(void) __attribute__ ((weak));
extern void HandleTamperButtonDown(void) __attribute__ ((weak));
extern void HandleUser1ButtonDown(void) __attribute__ ((weak));
extern void HandleUser2ButtonDown(void)__attribute__ ((weak));

extern void HandleWakeupButtonUp(void) __attribute__ ((weak));
extern void HandleTamperButtonUp(void) __attribute__ ((weak));
extern void HandleUser1ButtonUp(void) __attribute__ ((weak));
extern void HandleUser2ButtonUp(void)__attribute__ ((weak));

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == RESET)
    {
      if (HandleWakeupButtonDown)
        HandleWakeupButtonDown();
    }
    else
    {
      if (HandleWakeupButtonUp)
        HandleWakeupButtonUp();
    }

    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == RESET)
    {
      if (HandleUser2ButtonDown)
        HandleUser2ButtonDown();
    }
    else
    {
      if (HandleUser2ButtonUp)
        HandleUser2ButtonUp();
    }

    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}

void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == RESET)
    {
      if (HandleUser1ButtonDown)
        HandleUser1ButtonDown();
    }
    else
    {
      if (HandleUser1ButtonUp)
        HandleUser1ButtonUp();
    }

    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == RESET)
    {
      if(HandleTamperButtonDown)
        HandleTamperButtonDown();
    }
    else
    {
      if(HandleTamperButtonUp)
        HandleTamperButtonUp();
    }

    EXTI_ClearITPendingBit(EXTI_Line13);
  }
}

uint32_t HSV_to_RGB( float h, float s, float v);
void RGBLED_Update(uint8_t RED_Val, uint8_t GREEN_Val, uint8_t BLUE_Val);

static float h = 0, s = 1, v = 1;

void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
  {
#define HUE_MAX 6.0f
#define HUE_DELTA 0.001f

      h += HUE_DELTA;
      if (h > HUE_MAX)
      {
        h = 0;
      }

      uint32_t rgb = HSV_to_RGB(h,s,v);
      uint8_t* k = (uint8_t*) &rgb;

      RGBLED_Update(k[2],k[1],k[0]);

      TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  }
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
