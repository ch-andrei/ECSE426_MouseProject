////////////////////////////////////////////////////////////////////////////////
//	File Name					: LED_thread.c
//	Description				: Example of an OS thread that toggles the board LED's
//											based on a 1 second interrupt from TIM3
//	Author						: Harsh Aurora
// 	Modified by 			: Andrei Chubarau
//	Date							: Oct 28, 2016
////////////////////////////////////////////////////////////////////////////////
	
//		Includes		//
#include <LED_thread.h> 
#include <stm32f4xx_hal.h>
#include "stm32f4xx_it.h"
#include <cmsis_os.h>
#include "main.h"
#include "led.h"

//		Function Declaration		//
void LED_thread(void const *args);

//		Globals 		//
osThreadId LED_thread_ID;
osThreadDef(LED_thread, osPriorityNormal, 1, 512);

uint8_t led_state = 1;

//Brief:		Starts the LED thread in the OS (from Inactive into the Lifecycle)
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void start_LED_thread(void *args) {
	LED_thread_ID = osThreadCreate(osThread(LED_thread), args);
}

/**
   * @brief update LEDs to be on/off as per the alert state
   * @retval None
   */
void alert_led_toggle(void){
	// if in alert state, toggle LEDs in order and change alert FSM state
	if (led_state == 1){ // toggle green
		led_toggle_led_off(LED_ORANGE);
		led_toggle_led_on(LED_GREEN);
		led_state = 2;
	} else if (led_state == 2){ // toggle red
		led_toggle_led_off(LED_GREEN);
		led_toggle_led_on(LED_BLUE);
		led_state = 3;
	} else if (led_state == 3){ // toggle orange
		led_toggle_led_off(LED_BLUE);
		led_toggle_led_on(LED_RED);
		led_state = 4;
	} else if (led_state == 4){ // toggle blue
		led_toggle_led_off(LED_RED);
		led_toggle_led_on(LED_ORANGE);
		led_state = 1;
	} 
}

void LED_thread(void const *args) {
	while(1) {
		osSignalWait(OS_THREAD_SIGNAL_1, osWaitForever);
		alert_led_toggle();
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void LED_tim_elapsed_handler(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		LED_thread_signal();
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void LED_thread_signal(void){
		osSignalSet(LED_thread_ID, OS_THREAD_SIGNAL_1);
}

