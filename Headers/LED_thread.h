////////////////////////////////////////////////////////////////////////////////
//	File Name					: LED_thread.h
//	Description				: Header file for LED thread
//	Author						: Harsh Aurora
//	Date							: Oct 28, 2016
////////////////////////////////////////////////////////////////////////////////

#ifndef LED_THREAD_H
#define LED_THREAD_H

#include "stm32f4xx_hal.h"

//		Exported Functios		//
void start_LED_thread(void *args);

void LED_tim_elapsed_handler(TIM_HandleTypeDef *htim);
void LED_thread_signal(void);

#endif
