////////////////////////////////////////////////////////////////////////////////
//	File Name					: mouse_thread.h
//	Description				: Header file for mouse thread
//	Author						: Harsh Aurora
//	Date							: Nov 8, 2016
////////////////////////////////////////////////////////////////////////////////

#ifndef _MOUSE_THREAD
#define _MOUSE_THREAD

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define NUMPACKETS_OUTOFORDER_THRESHOLD 10

//		Exported Functios		//
void start_mouse_thread(void *args);
void mouse_tim_elapsed_handler(TIM_HandleTypeDef *htim);

#endif
