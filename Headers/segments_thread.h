/**  
  ******************************************************************************
  * @file    segments_thread.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from segments.c
  */

#ifndef SEGMENTS_THREAD_H
#define SEGMENTS_THREAD_H

#include "stm32f4xx_hal.h"

void start_Segments_thread(void *args);

void Segments_tim_elapsed_handler(TIM_HandleTypeDef *htim);

#endif
