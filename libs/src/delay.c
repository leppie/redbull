
#include "delay.h"
#include "misc.h"

#define STM32_DELAY_US_MULT         12

static inline void delay_us(uint32_t us) {
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

static inline void delay_ms( uint32_t ms ) {
    while (ms--) {
       delay_us( 1000 );
    }
}

volatile uint32_t ticks = 0;

void SysTick_Handler(void)
{
  ticks++;
}

void Delay(uint32_t ms)
{
//	uint32_t freq = (SystemCoreClock / 1000);
//    ticks = 0;
//	if (SysTick_Config(freq))
//	{
//		while(1);
//	}
//	while(ms--) { while (!ticks); ticks = 0; }
//
//	SysTick_Config(1);
	delay_ms(ms);
}


inline void MicroDelay(uint32_t us)
{
//	uint32_t freq = (SystemCoreClock / 1000000) * us ;
//    ticks = 0;
//	if (SysTick_Config(freq))
//	{
//		while(1);
//	}
//	if (us)	while(!ticks);
//	SysTick_Config(1);
	if (us) delay_us(us);
}

/*
uint32_t millis()
{
  return ticks;
}
*/

/*
void Delay_Init()
{
  if (SysTick_Config(SystemCoreClock/ 1000))
  {

    while (1)
      ;
  }
}
*/
