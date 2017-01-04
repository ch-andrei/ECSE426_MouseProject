/******************************************************************************
  * @file    segments.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   code to initialize and display numbers on the 7 segments display
  */

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
#include "data.h"
#include "math.h"
#include "main.h"
#include "accel_thread.h"
#include "accelerometer.h"
#include <cmsis_os.h>

void Accel_thread(void const *args);

osThreadId Accel_thread_ID;
osThreadDef(Accel_thread, osPriorityNormal, 1, 0);

float pitch_value, roll_value;

// start accelerometer thread
void start_Accel_thread(void *args) {
	Accel_thread_ID = osThreadCreate(osThread(Accel_thread), args);
}

// function performed by the acceleration thread
void Accel_thread(void const *args){
	while(1){
		osSignalWait(OS_THREAD_SIGNAL_1, osWaitForever);
		// read, calibrate, filter
		accelerometer_update();
		// compute angle values
		pitch_value = accelerometer_get_pitch_angle();
		roll_value = accelerometer_get_roll_angle();
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void Accel_interrupt_handler(void){
	if (Accel_thread_ID != NULL)
		osSignalSet(Accel_thread_ID, OS_THREAD_SIGNAL_1);
}
