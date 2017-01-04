////////////////////////////////////////////////////////////////////////////////
//	File Name					: main.h
//	Description				: header file for main.c
//	Author						: Harsh Aurora; Modified by Andrei Chubarau
//	Date							: Oct 1st, 2016
////////////////////////////////////////////////////////////////////////////////

#ifndef _MAIN
#define _MAIN

#define TRANSMITTER
//#define RECEIVER

#define TEMPERATURE_OPERATION 			1
#define ROLL_OPERATION 							2
#define PITCH_OPERATION 						3

#define OS_THREAD_SIGNAL_1					0x00000001

#define LED_PERIOD									1000 // 1000ms period -> 1Hz
#define MOUSE_CONTROL_PERIOD 				20 // 25ms period -> 40Hz, if timer is running at 1000Hz

#endif
