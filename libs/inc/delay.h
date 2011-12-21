

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x_conf.h"

//void Delay_Init(void);
void Delay(uint32_t);
void MicroDelay(uint32_t);
//uint32_t millis();


#ifdef __cplusplus
}
#endif

#endif
