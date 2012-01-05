#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm3210e_eval_fsmc_sram.h"
#include "stm3210e_eval_lcd.h"

#include "libdcc/dcc_stdio.h"
#include "usb_vcom.h"
#include "ff.h"
#include "numbers.h"
#include "one-wire.h"
#include "ads7843drv.h"
#include "rtc.h"
#include "arm_math.h"

void OneWireInit(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
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
  while (1)
    ;
}

typedef struct
{
  __IO uint16_t LCD_REG;
  __IO uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)

#define ADC1_DR_Address    ((uint32_t)0x4001244C)

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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC, ENABLE);
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

#define DAC_DHR12RD_Address      0x40007420

/* Init Structure definition */
DAC_InitTypeDef DAC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//const uint16_t Sine12bit[32] = { 2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056, 3939, 3750, 3495, 3185,
//    2831, 2447, 2047, 1647, 1263, 909, 599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647 };

const uint16_t Sine12bit[128] = { 2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3013, 3100, 3185, 3267,
    3346, 3423, 3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898, 3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093, 4095,
    4093, 4085, 4073, 4056, 4034, 4007, 3975, 3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565, 3495, 3423, 3346, 3267,
    3185, 3100, 3013, 2923, 2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148, 2048, 1947, 1847, 1747, 1648, 1550, 1453,
    1358, 1264, 1172, 1082, 995, 910, 828, 749, 672, 600, 530, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22,
    10, 2, 0, 2, 10, 22, 39, 61, 88, 120, 156, 197, 242, 291, 345, 403, 465, 530, 600, 672, 749, 828, 910, 995, 1082,
    1172, 1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947 };


uint16_t ModSine12bit[128];
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
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
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

  /* TIM6 Configuration */
  TIM_PrescalerConfig(TIM6, 0x3, TIM_PSCReloadMode_Update);
  TIM_SetAutoreload(TIM6, 0xF);
  /* TIM6 TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  for (int i = 0; i < 128; i++)
  {
    ModSine12bit[i] = Sine12bit[i] / 2 + 1023;
  }

  DMA_DeInit(DMA2_Channel3);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &DAC->DHR12R1;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ModSine12bit;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 128;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA2_Channel3, &DMA_InitStructure);
  /* Enable DMA2 Channel3 */
  DMA_Cmd(DMA2_Channel3, ENABLE);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
   automatically connected to the DAC converter. */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /* Enable DMA for DAC Channel1 */
  DAC_DMACmd(DAC_Channel_1, ENABLE);

  /* TIM6 enable counter */
  TIM_Cmd(TIM6, ENABLE);
}


void RGBLED_GPIO_Config(uint16_t PrescalerValue)
{
  /* GPIOA Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);


  /* -----------------------------------------------------------------------
   TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
   The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
   clock at 24 MHz the Prescaler is computed as following:
   - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
   SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   and Connectivity line devices and to 24 MHz for Low-Density Value line and
   Medium-Density Value line devices

   The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
   = 24 MHz / 666 = 36 KHz
   TIM1 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
   TIM1 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
   TIM1 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
   ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  //uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 255;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void RGBLED_Update(uint8_t RED_Val, uint8_t GREEN_Val, uint8_t BLUE_Val)
{
  //printf("r: %x g: %x b:%x\n", RED_Val, GREEN_Val, BLUE_Val);
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_OCStructInit(&TIM_OCInitStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = GREEN_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLUE_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = RED_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

uint32_t HSV_to_RGB( float h, float s, float v ) {
#define long(v) ((uint32_t)(v))
#define RGB(r,g,b) (long(r * 255 ) * 65536 + long(g * 255 ) * 256 + long(b * 255))
  /* modified from Alvy Ray Smith's site: http://www.alvyray.com/Papers/hsv2rgb.htm */
  // H is given on [0, 6]. S and V are given on [0, 1].
  // RGB is returned as a 24-bit long #rrggbb
  int i;
  float m, n, f;

  // not very elegant way of dealing with out of range: return black
  if ((s<0.0) || (s>1.0) || (v<1.0) || (v>1.0)) {
    return 0L;
  }

  if ((h < 0.0) || (h > 6)) {
    return RGB(v,v,v);
  }
  i = floorf(h);
  f = h - i;
  if ( !(i&1) ) {
    f = 1 - f; // if i is even
  }
  m = v * (1 - s);
  n = v * (1 - s * f);
  switch (i) {
    case 6:
    case 0:    return RGB(v,n,m);
    case 1:    return RGB(n,v,m);
    case 2:    return RGB(m,v,n);
    case 3:    return RGB(m,n,v);
    case 4:    return RGB(n,m,v);
    case 5:    return RGB(v,m,n);
  }
  return -1;
}

static FATFS fatfs;

void Redbull_Init(void);
void printRight(int, char*);

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  Redbull_Init();

  char buff[128] = { 0 };

  LCD_DisplayStringLine(LINE(10), (uint8_t*) " Analog ..............................");
  Analog_Config();
  printRight(10, "ok");

  LCD_DisplayStringLine(LINE(11), (uint8_t*) " DAC .................................");
  DAC_Config();
  printRight(11, "ok");

  f_mount(0, &fatfs);

  LCD_DisplayStringLine(LINE(12), (uint8_t*) " TEMP ................................");
  OneWireInit();
  float c = Read_Temperature();
  sprintf(buff, "%.1f", c);
  printRight(12, buff);
  printf("%s\n", buff);

  RGBLED_GPIO_Config(0xff);

  RGBLED_Update(0xff,0xff,0xff);
  Delay(200);
  RGBLED_Update(0xff,0,0);
  Delay(200);
  RGBLED_Update(0,0xff,0);
  Delay(200);
  RGBLED_Update(0,0,0xff);
  Delay(200);
  RGBLED_Update(0,0,0);

  //static char path[1024] = "0:";
  //LCD_Clear(LCD_COLOR_BLUE);
  //LCD_DisplayStringLine(0, (uint8_t*) "Loading...");
  Delay(1000);

  //int i = 10;

  LCD_Clear(LCD_COLOR_BLACK);

  LCD_SetDisplayWindow(0, 0, 239, 319);
  LCD_WriteReg(3, 0x1000);
  LCD_WriteRAM_Prepare();



  DMARead();

  float h = 0, s = 1, v = 1;

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

    if (cnt >= 80 && cnt < 512)
    {
      continue;
    }

    uint32_t buff[160] = { 0 };


    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == RESET)
    {
      // do fft
      uint16_t sbuf[1024] = {0};

      uint16_t tillend = cnt > 1024 ? 1024 : cnt;
      uint16_t fromstart = 1024 - tillend;

      uint16_t bcnt = cnt - 1024;

      // before
      SRAM_ReadBuffer(&sbuf[0], bcnt, tillend);
      // after
      if (fromstart)
      {
        SRAM_ReadBuffer(&sbuf[tillend], 0, fromstart);
      }

      float32_t input[2048], output[1024];

      for (int i = 0; i < 1024; i++)
      {
        input[i * 2] = (sbuf[i]/2048.0f) - 1;
        input[i * 2 + 1] = 0;
      }

      arm_cfft_radix4_instance_f32 S;
      arm_cfft_radix4_init_f32(&S, 1024, 0, 0);

      arm_cfft_radix4_f32(&S, input);

      arm_cmplx_mag_f32(input, output, 1024);

      for (int i = 0; i < 1024; i++)
      {
        sbuf[i] = (uint16_t) (output[i]);
      }

      for (int i = 0; i < 256; i++)
      {
        ((uint16_t*) buff)[i] = (sbuf[4 * i] + sbuf[4 * i + 1] + sbuf[4 * i + 2] + sbuf[4 * i + 3]) >> 2;
      }

      bcnt = 0;
/*
      for (int i = 0; i < 512; i++)
      {
        sbuf[i] = (uint16_t) ((output[i] + output[1023 - i]) * 16);
      }
      for (int i = 512; i < 1024 ; i++)
      {
        sbuf[i] = 0;
      }
*/
      // profit!
    }
    else
    {
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
    }

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
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
      DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
      DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
      DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

      DMA_Init(DMA2_Channel5, &DMA_InitStructure);

      DMA_Cmd(DMA2_Channel5, ENABLE);

      RTC_t rtc;
      RTC_GetTime(&rtc);

#define HUE_MAX 6.0f
#define HUE_DELTA 0.0001f

      h += HUE_DELTA;
      if (h > HUE_MAX)
      {
        h = 0;
      }

      uint32_t rgb = HSV_to_RGB(h,s,v);
      uint8_t* k = (uint8_t*) &rgb;

      RGBLED_Update(k[2],k[1],k[0]);

      //RGBLED_Update((rtc.min & 1) ? 0 : (rtc.hour * 10), ((rtc.sec >> 2) & 1) ? 0 : (rtc.min * 4), rtc.sec * 4);

      while (!DMA_GetFlagStatus(DMA2_FLAG_TC5))
        ;

      DMA_ClearFlag(DMA2_FLAG_TC5);

      while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == RESET)
        ;

    }
  }
}

