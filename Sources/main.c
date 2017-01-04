////////////////////////////////////////////////////////////////////////////////
//	File Name					: main.c
//	Description				: program entry
//	Author						: Harsh Aurora
// 	Modified by 			: Andrei Chubarau
//	Date							: Oct 28, 2016
////////////////////////////////////////////////////////////////////////////////
	
//		Includes		//
#include <stm32f4xx_hal.h>
#include <supporting_functions.h>
#include <sysclk_config.h>
#include <arm_math.h>
#include <LED_thread.h>
#include <cmsis_os.h>
#include "lis3dsh.h"
#include "led.h"
#include "accelerometer.h"
#include "accel_thread.h"
#include "segments_thread.h"
#include "segments.h"
#include "timer.h"
#include "main.h"
#include "keypad.h"
#include "mouse_control_thread.h"
#include "CC2500.h"
#include <mouse_thread.h>
#include <rl_usb.h>                     // Keil.MDK-Pro::USB:CORE

//Brief:	main program
//				
//Params:	None
//Return:	None
int main(void) {
  //		MCU Configuration		//
  //	Reset of all peripherals, Initializes the Flash interface and the Systick	//
	osKernelInitialize();  
  HAL_Init();
	
  //	Configure the system clock	//
  SystemClock_Config();
	
	// -------- Configure peripherals -------- //
	led_configure();
	
	#ifdef TRANSMITTER
		accelerometer_config();
		keypad_configure();
	#endif
	
	CC2500_configure();
	
	// configure USB only for the RECEIEVER
	#ifdef RECEIVER
		USBD_Initialize(0);               /* USB Device 0 Initialization */
		USBD_Connect(0);
	#endif
	
	// -------- Start threads -------- //
	
	start_LED_thread(NULL);
	
	// start accelerometer and mouse control threads only for TRANSMITTER
	#ifdef TRANSMITTER
		start_Accel_thread(NULL);
		start_mouse_control_thread(NULL);
	#endif
	
	start_mouse_thread(NULL);
	
	// configure and start TIM3 hardware timer
	timer_configure();
	
	osKernelStart();
	osDelay(osWaitForever);
	return 0;
}
