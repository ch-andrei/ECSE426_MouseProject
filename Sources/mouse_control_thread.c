/**  
  ******************************************************************************
  * @file    keypad_thread.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   
  */
	
#include "stm32f4xx_hal.h"
#include "main.h"
#include "mouse_control_thread.h"
#include "keypad.h"
#include <cmsis_os.h>

// pitch and roll angles computed by the accelerometer threads 
extern float pitch_value, roll_value;

// thread for capturing user input and encoding this into a 4-byte array
void mouse_control_thread(void const *args);
osThreadId mouse_control_thread_ID;
osThreadDef(mouse_control_thread, osPriorityNormal, 1, 512);

extern uint8_t transmision_message[5];
uint8_t leftFlag = 0;
uint8_t rightFlag = 0;
uint8_t middleFlag = 0;
uint8_t scrollUpFlag = 0;
uint8_t scrollDownFlag = 0;

// maps a float value angle input to its 1 byte representation in 2s compliment 
// @param input: angle in the a range of 0-255
// @output: unit8_t; 0-127 represents positive movement where 127 is maximum displacement; 128-255 represents negative movement, where 128 is maximum displacement.
uint8_t getNormalizedMovementFromAngle(float angle){
	uint8_t normalized = 0;
	if (angle < CONTROL_ANGLE_ORIGIN - CONTROL_ANGLE_THRESHOLD){
		// move down or left
		normalized = 0xFF;
		normalized -= (uint8_t)(((uint8_t)angle - CONTROL_ANGLE_ORIGIN + CONTROL_ANGLE_THRESHOLD ) * CONTROL_MOUSE_SENSITIVITY);
	}  else if (angle > CONTROL_ANGLE_ORIGIN + CONTROL_ANGLE_THRESHOLD){
		// move up or right
		normalized = 0x01;
		normalized += (uint8_t)((CONTROL_ANGLE_ORIGIN + CONTROL_ANGLE_THRESHOLD - (uint8_t)angle) * CONTROL_MOUSE_SENSITIVITY);
	} 
	return normalized;
}

// start mouse control thread
void start_mouse_control_thread(void *args) {
	mouse_control_thread_ID = osThreadCreate(osThread(mouse_control_thread), args);
}

// function passe to the mouse control thread
// acquires user input and incodes this to a 4-byte array
void mouse_control_thread(void const *args){
	uint8_t temp_key, mouse_control;
	while(1){
		osSignalWait(OS_THREAD_SIGNAL_1, osWaitForever);
		leftFlag = 0;
		rightFlag = 0;
		middleFlag = 0;
		scrollUpFlag = 0;
		scrollDownFlag = 0;
		if (keypad_read_debounce(&temp_key, DEBOUNCE_OFF) == 0){
			switch (temp_key){
				default:
					break;
				case KEYPAD_LEFTCLICK:
					leftFlag = 1;
					break;
				case KEYPAD_MIDDLECLICK:
					middleFlag = 1;
					break;
				case KEYPAD_RIGHTCLICK:
					rightFlag = 1;
					break;
				case KEYPAD_SCROLLUP:
					scrollUpFlag = 1;
					break;
				case KEYPAD_SCROOLDOWN:
					scrollDownFlag = 1;
					break;
			}
						
			// get clicks
			mouse_control = 0x00;
			if (leftFlag)
				mouse_control |= KEYPAD_LEFTCLICK_MASK;
			if (middleFlag)
				mouse_control |= KEYPAD_MIDDLECLICK_MASK;
			if (rightFlag)
				mouse_control |= KEYPAD_RIGHTCLICK_MASK;
			// store clicks
			transmision_message[0] = mouse_control;
			
			// get movement over X and Y from accelerometer
			mouse_control = getNormalizedMovementFromAngle(180 - roll_value);	// X: adjusted 180 - angle, otherwise wrong direction
			transmision_message[1] = mouse_control;	
			mouse_control = getNormalizedMovementFromAngle(pitch_value); // Y
			transmision_message[2] = mouse_control;

			// get scroll wheel up/down commands
			mouse_control = 0x00;
			if (scrollUpFlag)
				mouse_control = KEYPAD_SCROLLUP_VAL;
			else if (scrollDownFlag)
				mouse_control = KEYPAD_SCROLLDOWN_VAL;
			// store scroll wheel commands
			transmision_message[3] = mouse_control;	
		}
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void mouse_control_tim_elapsed_handler(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		osSignalSet(mouse_control_thread_ID, OS_THREAD_SIGNAL_1);
	}
}
