/**  
  ******************************************************************************
  * @file    keypad_thread.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from keypad_thread.c
  */

#ifndef MOUSE_CONTROL_THREAD_H
#define MOUSE_CONTROL_THREAD_H

#include "stm32f4xx_hal.h"

#define KEYPAD_LEFTCLICK		1
#define KEYPAD_MIDDLECLICK 	2
#define KEYPAD_RIGHTCLICK		3
#define KEYPAD_SCROLLUP 		5
#define KEYPAD_SCROOLDOWN 	8

#define KEYPAD_LEFTCLICK_MASK		  0x01
#define KEYPAD_MIDDLECLICK_MASK 	0x04
#define KEYPAD_RIGHTCLICK_MASK		0x02

#define KEYPAD_SCROLLUP_VAL 			0x01
#define KEYPAD_SCROLLDOWN_VAL 		0xFF

#define CONTROL_ANGLE_THRESHOLD   5
#define CONTROL_ANGLE_ORIGIN 			90
#define CONTROL_MOUSE_SENSITIVITY 1 // 1 is default, ~3 is maximum

void start_mouse_control_thread(void *args);

void mouse_control_tim_elapsed_handler(TIM_HandleTypeDef *htim);

#endif /* MOUSE_CONTROL_THREAD_H */
