#ifndef __TIMER_H__
#define __TIMER_H__

#include <string.h>
#include "BSP.h"
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc); 
void TIM4_Init(void);
int TIM4_capture(u16* captureTimePt);
void delayMs(uint16_t u16ms);
void delayUs(uint16_t u16us);



#endif

