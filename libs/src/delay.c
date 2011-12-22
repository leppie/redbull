
#include "delay.h"
#include "misc.h"


volatile uint32_t ticks = 0;

void SysTick_Handler(void)
{
  ticks++;
}

void Delay(uint32_t ms)
{
  uint32_t t = ticks;
  uint32_t till = t + ms;
  if (till < t)
  {
    till = ms;
    ticks = 0;
  }
  while (ticks < till); // till we overflow and kaboom
}


uint32_t millis()
{
  return ticks;
}

void Delay_Init()
{
  if (SysTick_Config(SystemCoreClock/ 1000))
  {

    while (1)
      ;
  }
}

