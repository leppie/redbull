#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "stm32_eval.h"
#include "stm32_eval_sdio_sd.h"
#include "stm3210e_eval_fsmc_nand.h"
#include "stm3210e_eval_fsmc_nor.h"
#include "stm3210e_eval_fsmc_sram.h"
#include "delay.h"
#include "rtc.h"
#include "stm3210e_eval_lcd.h"
#include "usb_vcom.h"
#include "ff.h"


#define LED_PORT GPIOF

#define LED(x) (GPIO_Pin_6 << ((x) - 1))
#define LED_ALL (LED(1) | LED(2) | LED(3) | LED(4) | LED(5))

void LED_On(uint16_t leds)
{
  LED_PORT->ODR = ~leds;
}

void LED_Off(uint16_t leds)
{
  LED_PORT->ODR = leds;
}

void LED_Init(void)
{
  GPIO_InitTypeDef leds;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

  leds.GPIO_Pin = LED_ALL;
  leds.GPIO_Mode = GPIO_Mode_Out_PP;
  leds.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_PORT, &leds);

  LED_Off(LED_ALL);
}


void toggleLED(int pin, int prev)
{
  if (pin)
  {
    GPIO_ResetBits(GPIOF, pin);
    Delay(5);
  }
  if (prev)
  {
    GPIO_SetBits(GPIOF, prev);
    Delay(20);
  }
}


void Button_GPIO_Config(void)
{
  /* GPIOA Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  EXTI_InitTypeDef   EXTI_InitStructure;

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line3 | EXTI_Line8 | EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_Init(&NVIC_InitStructure);
}


void HandleWakeupButtonPress(void)
{
  LED_On(LED(1));
}

void HandleTamperButtonPress(void)
{
  LED_On(LED(2));
}

void HandleUser1ButtonPress(void)
{
  LED_On(LED(3));
}

void HandleUser2ButtonPress(void)
{
  LED_On(LED(4));
}


void printRight(int line, char* text)
{
  int l = strlen(text);
  int col = 40 - l;
  for (; *text; text++, col++)
  {
    int c = 319 - (col * LCD_GetFont()->Width);
    if (c > 0)
    {
      LCD_DisplayChar(LINE(line), c, *text);
    }
    else
    {
      break;
    }
  }
}


void USART_STDIO_Init(void);



void Redbull_Init()
{
  char buff[128] = { 0 };

  USART_STDIO_Init();
  Delay_Init();
  Button_GPIO_Config();

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  STM3210E_LCD_Init();
  LCD_SetFont(&Font8x12);
  LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);

  LCD_WriteRAM_Prepare();

  for (int i = 0; i < (320 * 240); i++)
  {
    LCD_WriteRAM(LCD_COLOR_WHITE);
  }
  for (int i = 0; i < (320 * 240); i++)
  {
    LCD_WriteRAM(LCD_COLOR_BLACK);
  }
  /*
   TP_Init();

   int x, y;
   TP_GetAdXY(&x, &y);

   while(1)
   {
   int irq = !GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_7);
   if (irq)
   {
   GPIO_InitTypeDef GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOG, &GPIO_InitStructure);

   GPIO_ResetBits(GPIOG, GPIO_Pin_7);
   TP_GetAdXY(&x, &y);
   printf("irq: %d x: %d y: %d\n", irq, x , y);

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOG, &GPIO_InitStructure);
   }


   }
   */
  LCD_DisplayStringLine(LINE(0), (uint8_t*) " initializing REDBULL");
  LCD_DisplayStringLine(LINE(1), (uint8_t*) " CPU ...............................");
  sprintf(buff, "ARM Cortex-M3 @ %dMHz", (int) SystemCoreClock / 1000000);
  printRight(1, buff);
  LCD_DisplayStringLine(LINE(2), (uint8_t*) " LCD ............................320x240");

  LCD_DisplayStringLine(LINE(3), (uint8_t*) " LED ..................................");

  LED_Init();
  toggleLED(LED1_PIN, 0);
  toggleLED(LED2_PIN, LED1_PIN);
  toggleLED(LED3_PIN, LED2_PIN);
  toggleLED(LED4_PIN, LED3_PIN);
  toggleLED(LED5_PIN, LED4_PIN);
  toggleLED(LED4_PIN, LED5_PIN);
  toggleLED(LED3_PIN, LED4_PIN);
  toggleLED(LED2_PIN, LED3_PIN);
  toggleLED(LED1_PIN, LED2_PIN);
  toggleLED(0, LED1_PIN);

  printRight(3, "5");

  LCD_DisplayStringLine(LINE(4), (uint8_t*) " RTC ................");
  RTC_Init();
  RTC_t rtc = { .year = 2011, .month = 12, .mday = 19, .hour = 21, .min = 00 };
  //RTC_SetTime(&rtc);
  RTC_GetTime(&rtc);
  sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", rtc.year, rtc.month, rtc.mday, rtc.hour, rtc.min, rtc.sec);
  printRight(4, buff);

  LCD_DisplayStringLine(LINE(5), (uint8_t*) " USB .................................");
  Set_USBClock();
  Set_System();
  USB_Interrupts_Config();
  USB_Init();
  printRight(5, "ok");

  //IS61LV25616 (512KB)
  LCD_DisplayStringLine(LINE(6), (uint8_t*) " SRAM ................................");
  SRAM_Init();
  uint32_t* RAM = (uint32_t*) Bank1_SRAM3_ADDR;
  uint8_t TESTOK = 1;
  for (uint32_t i = 0; i < (512 * 1024) / 4; i++)
  {
    RAM[i] = i;
  }
  for (uint32_t i = 0; i < (512 * 1024) / 4; i++)
  {
    if (RAM[i] != i)
    {
      TESTOK = 0;
    }
    RAM[i] = 0;
  }

  if (TESTOK)
  {
    printRight(6, "IS61LV25616 512KB");
  }
  else
  {
    printRight(6, "fail");
  }

  //M29W128F (2MB)
  LCD_DisplayStringLine(LINE(7), (uint8_t*) " NOR .................................");
  NOR_Init();
  NOR_IDTypeDef norid;
  NOR_ReadID(&norid);
  printRight(7, "MX29LV160D 2MB");

  //HY27UF081G2A (128MB)
  LCD_DisplayStringLine(LINE(8), (uint8_t*) " NAND ................................");
  NAND_Init();
  NAND_IDTypeDef nandid;
  NAND_ReadID(&nandid);
  printRight(8, "HY27UF081G2A 128MB");

  LCD_DisplayStringLine(LINE(9), (uint8_t*) " SDIO ................................");
  SD_Init();
  SD_CardInfo cardinfo;
  SD_GetCardInfo(&cardinfo);
  printRight(9, "ok");

}
