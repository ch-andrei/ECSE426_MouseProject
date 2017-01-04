/******************************************************************************
  * @file    segments_thread.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   
  */

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
#include "data.h"
#include "main.h"
#include "segments_thread.h"
#include "segments.h"
#include <cmsis_os.h>

// operation type (set by main)
extern float pitch_value, roll_value;
float temperature_value = 0;

// opereration mode defined by main and set by keypad (which value to display)
int operation_mode = 0;

extern int temperature_alert_flag_g;

// current temperature displayed on segments
float current_value = 1;

// counter to update currently displayed temperature
int update_value_flag;

// current selector for segment to update (4 segments: s0, s1, s2, s3) 
int update_segment;

// value format X.XX, XX.X, XXX
int value_scale;

// thread definitions
void Segments_thread(void const *args);
osThreadId Segments_thread_ID;
osThreadDef(Segments_thread, osPriorityNormal, 1, 512);

// start the segments thread
void start_Segments_thread(void *args) {
	Segments_thread_ID = osThreadCreate(osThread(Segments_thread), args);
}

// function that is performed by the segments thread
void Segments_thread(void const *args){
	while(1){
		osSignalWait(OS_THREAD_SIGNAL_1, osWaitForever);
		// display a vaue depending on operation mode
		// TODO: OPTIONAL, draw stuff on segments
		// NEVER MIND, no big need to use segments in this project
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void Segments_tim_elapsed_handler(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		osSignalSet(Segments_thread_ID, OS_THREAD_SIGNAL_1);
	}
}
