/**  
  ******************************************************************************
  * @file    segments.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from segments.c
  */

#ifndef ACCEL_THREAD_H
#define ACCEL_THREAD_H

void start_Accel_thread(void *args);

void Accel_interrupt_handler(void);

#endif
