/**  
  ******************************************************************************
  * @file    timer.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   Implementation of the hardware timer using TIM3
  */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "main.h"
#include <cmsis_os.h>
#include "segments_thread.h"
#include "LED_thread.h"
#include "segments.h"
#include "mouse_control_thread.h"
#include "mouse_thread.h"

TIM_HandleTypeDef TIM3_handle;

int tim_counter;

/**
   * @brief Configure the TIM3 hardware timer
   * @retval None
   */
void timer_configure(void){
	// set pointer to the flag
	// enable clock
	__TIM3_CLK_ENABLE();
	
	segments_configure();
	
	// initialize the timer 
	TIM3_handle.Init.Prescaler = 84;
	TIM3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM3_handle.Init.Period = 999;
	// Desired rate = TimerClockingFrequency / ((Period+1) x prescalar)
	// 84 Mhz / 84 / 1000 = 1000Hz
	
	TIM3_handle.Instance = TIM3;
	HAL_TIM_Base_Init(&TIM3_handle);     // Init timer
	HAL_TIM_Base_Start_IT(&TIM3_handle); // start timer interrupts
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		if (tim_counter % MOUSE_CONTROL_PERIOD == 0){
			mouse_control_tim_elapsed_handler(htim);
		}
		if (tim_counter % MOUSE_CONTROL_PERIOD == 0){
			mouse_tim_elapsed_handler(htim);
		}
		tim_counter++;
		TIM3_IRQHandler();
	}
}
