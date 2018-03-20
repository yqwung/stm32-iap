#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H	 
#include <stm32f10x_conf.h>

#define toAppDelay   TO_APP_DELAY

void BSP_TIM3_Init(unsigned short arr, unsigned short psc);

		 				    
#endif