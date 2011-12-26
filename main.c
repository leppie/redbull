/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/main.c
 * @author  MCD Application Team
 * @version V3.3.0
 * @date    04/16/2010
 * @brief   Main program body
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
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32_eval.h"
#include "stm32_eval_sdio_sd.h"
#include "stm3210e_eval_fsmc_nand.h"
#include "stm3210e_eval_fsmc_nor.h"
#include "stm3210e_eval_fsmc_sram.h"
#include "delay.h"
#include "rtc.h"
#include "stm3210e_eval_lcd.h"
#include "libdcc/dcc_stdio.h"
#include "usb_vcom.h"
#include "ff.h"
#include "numbers.h"
#include "one-wire.h"
#include "ads7843drv.h"

void USART_STDIO_Init();

#define Bank1_SRAM3_ADDR    ((__IO uint32_t)0x68000000)

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

void OneWireInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	ow_reset();

	ow_write_byte(0xcc);
	ow_write_byte(0x4e);
	ow_write_byte(0);
	ow_write_byte(0);
	ow_write_byte(0x60); // 12 bit
	ow_reset();
}

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1);
}


typedef struct
{
	__IO uint16_t LCD_REG;
	__IO uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

void DMARead(void)
{
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) Bank1_SRAM3_ADDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 0xffff;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void Analog_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Analog_RCC_Config(void)
{
	/* ADCCLK = PCLK2/4 overclock ? FTW ! :o) */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	/* Enable peripheral clocks ------------------------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1, ADC2 and GPIOC clock */
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC,
			ENABLE);
}

void Analog_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC.01, PC.02 and PC.04 (ADC Channel11, Channel12 and Channel14)
	 as analog input ----------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Analog_Config(void)
{
	Analog_RCC_Config();
	Analog_GPIO_Config();
	Analog_NVIC_Config();

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_SlowInterl;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5);

	/* ADC2 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_SlowInterl;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC2 regular channels configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5);

	/* Enable ADC2 external trigger conversion */
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	/* Enable ADC1 DMA: it should be enabled in dual mode even if the DMA is not used */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	Delay(2);

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);

	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1))
		;

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);

	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1))
		;

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	Delay(2);

	/* Enable ADC2 reset calibaration register */
	ADC_ResetCalibration(ADC2);

	/* Check the end of ADC2 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC2))
		;

	/* Start ADC2 calibaration */
	ADC_StartCalibration(ADC2);

	/* Check the end of ADC2 calibration */
	while (ADC_GetCalibrationStatus(ADC2))
		;

}

/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420

/* Init Structure definition */
DAC_InitTypeDef DAC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t Sine12bit[32] =
{ 2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056, 3939, 3750, 3495,
		3185, 2831, 2447, 2047, 1647, 1263, 909, 599, 344, 155, 38, 0, 38, 155,
		344, 599, 909, 1263, 1647 };

uint32_t DualSine12bit[32];
uint8_t Idx = 0;

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void DAC_RCC_Configuration(void)
{
	/* DMA2 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void DAC_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	 connected to the DAC converter. In order to avoid parasitic consumption,
	 the GPIO pin should be configured in analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void DAC_Config(void)
{
	DAC_RCC_Configuration();

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	 connected to the DAC converter. In order to avoid parasitic consumption,
	 the GPIO pin should be configured in analog */
	DAC_GPIO_Configuration();

	/* TIM2 Configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0x1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude =
			DAC_TriangleAmplitude_1023;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DAC channel2 Configuration */
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	/* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_2, ENABLE);

	/* Set DAC dual channel DHR12RD register */
	DAC_SetDualChannelData(DAC_Align_12b_R, 0x00, 0x00);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
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

void printRight(int line, char* text)
{
	int l = strlen(text);
	int col = 40 - l;
	for (;*text; text++, col++ )
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

static FATFS fatfs;

const uint16_t STROBE[] =
{
	LED1_PIN,
	LED2_PIN,
	LED3_PIN,
	LED4_PIN,
	LED5_PIN,
	LED4_PIN,
	LED3_PIN,
	LED2_PIN,
};

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

	char buff[128] = {0};

	USART_STDIO_Init();

	Delay_Init();

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

	sprintf(buff, "ARM Cortex-M3 @ %dMHz", (int)SystemCoreClock/1000000);

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
	toggleLED(0		  , LED1_PIN);

	printRight(3, "5");

	LCD_DisplayStringLine(LINE(4), (uint8_t*) " RTC ................");
	RTC_Init();


	RTC_t rtc =
	{
	  .year = 2011,
	  .month = 12,
	  .mday = 19,
	  .hour = 21,
	  .min = 00
	};

	//RTC_SetTime(&rtc);

	RTC_GetTime(&rtc);
	sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", rtc.year, rtc.month, rtc.mday,  rtc.hour, rtc.min, rtc.sec);

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

	for (uint32_t i = 0; i < (512 * 1024)/4; i++)
	{
		RAM[i] = i;
	}

	for (uint32_t i = 0; i < (512 * 1024)/4; i++)
	{
	   if (RAM[i] != i)
	   {
		  TESTOK = 0;
	   }
	   RAM[i] = 0;
	}

	if (TESTOK)
	{
		printRight(6,"IS61LV25616 512KB");
	}
	else
	{
		printRight(6,"fail");
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

	LCD_DisplayStringLine(LINE(10), (uint8_t*) " Analog ..............................");
	Analog_Config();
	printRight(10, "ok");

	LCD_DisplayStringLine(LINE(11), (uint8_t*) " DAC .................................");
	DAC_Config();
	printRight(11, "ok");

	f_mount(0, &fatfs);

	LCD_DisplayStringLine(LINE(12), (uint8_t*) " TEMP ................................");
	OneWireInit();

	while (0)
	{
		uint32_t ms = millis();

		float c = Read_Temperature();

		RTC_GetTime(&rtc);

		sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", rtc.year, rtc.month, rtc.mday,  rtc.hour, rtc.min, rtc.sec);
		printRight(4, buff);

		sprintf(buff, "%.1f", c);
		printRight(12, buff);

		printf("%s\n", buff);

		Delay(1000 - (millis() - ms));
	}


	//static char path[1024] = "0:";

	//LCD_Clear(LCD_COLOR_BLUE);


	//LCD_DisplayStringLine(0, (uint8_t*) "Loading...");

	Delay(5000);

	//int i = 10;

	LCD_Clear(LCD_COLOR_BLACK);

	LCD_SetDisplayWindow(0, 0, 239, 319);
	LCD_WriteReg(3, 0x1000);
	LCD_WriteRAM_Prepare();

	DMARead();

	while (1)
	{
		while (DMA_GetFlagStatus(DMA1_FLAG_TE1) == SET)
		{
			printf("DMA error\n");
			Delay(100);
		}

		//uint32_t ms = millis();

		uint16_t rem = DMA_GetCurrDataCounter(DMA1_Channel1);
		uint16_t cnt = 0xffff - rem;

		if (cnt >= 80 && cnt < 160)
		{
			continue;
		}

		uint32_t buff[160] =
		{ 0 };

		uint16_t tillend = cnt > 160 ? 160 : cnt;
		uint16_t fromstart = 160 - tillend;

		uint16_t bcnt = cnt - 160;

		// before
		SRAM_ReadBuffer((uint16_t*) &buff[0], bcnt * 4, tillend * 2);
		// after
		if (fromstart)
		{
			SRAM_ReadBuffer((uint16_t*) &buff[tillend], 0, fromstart * 2);
		}

		int halt = 0;
		uint8_t prevsv = 0;

		for (uint16_t i = 0; i < 320; i++)
		{
			uint16_t val = 0;
			if (i & 1)
			{
				val = buff[i >> 1] & 0x0fff;
			}
			else
			{
				val = (buff[i >> 1] >> 16) & 0x0fff;
			}

#define MAXV (17 * 238)

			val = val > MAXV ? MAXV : val;

			uint8_t sv = val / 17;

			sv = sv > 238 ? 238 : sv;

			sv++;

			if (!prevsv)
			{
				prevsv = sv;
			}

			//halt |= (val > 1126);

			uint16_t pbuf[240];

			uint8_t minv = sv > prevsv ? prevsv : sv;
			uint8_t maxv = sv < prevsv ? prevsv : sv;

			for (uint8_t y = 0; y < 240; y++)
			{
				if (y >= minv && y <= maxv)
				{
					pbuf[y] = LCD_COLOR_YELLOW;
				}
				else
				{
					pbuf[y] = LCD_COLOR_BLACK;
				}
			}

			prevsv = sv;

			DMA_InitTypeDef DMA_InitStructure;

			DMA_DeInit(DMA2_Channel5);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) pbuf;
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &LCD->LCD_RAM; // FSMC
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			DMA_InitStructure.DMA_BufferSize = 240;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_HalfWord;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

			DMA_Init(DMA2_Channel5, &DMA_InitStructure);

			DMA_Cmd(DMA2_Channel5, ENABLE);

			while (!DMA_GetFlagStatus(DMA2_FLAG_TC5));

			DMA_ClearFlag(DMA2_FLAG_TC5);

			//Delay(100);
		}

		if (halt)
		{
			printf(
					"rem: %d cnt: %d tillend: %d fromstart: %d bcnt: %d halt: %d\n",
					rem, cnt, tillend, fromstart, bcnt, halt);
			//Delay(2000);
		}

	}
}

