#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "!Project_library.h"

void SysTick_Handler(void);
void SysTick_Init(void);
unsigned int SysTick_GetCurrentClock(void);
void SysTick_WaitTill(unsigned int val);


/* SIM800 */
void SysTick_SetSimTimeMs(unsigned int time);
unsigned int SysTick_GetSimTime(void);

#endif