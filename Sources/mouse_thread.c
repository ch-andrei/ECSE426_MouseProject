////////////////////////////////////////////////////////////////////////////////
//	File Name					: mouse.c
//	Description				: Example of an OS thread controlling the mouse USB 
//											interface via the on-board pushbutton
//	Author						: Harsh Aurora
//	Date							: Nov 8, 2016
////////////////////////////////////////////////////////////////////////////////
	
//		Includes		//
#include <mouse_thread.h> 
#include <cmsis_os.h>
#include <rl_usb.h>
#include "main.h"
#include "CC2500.h"
#include "LED_thread.h"

//		Function Declaration		//
void mouse_thread(void const *args);

//		Globals 		//
osThreadId mouse_thread_ID;
osThreadDef(mouse_thread, osPriorityAboveNormal, 1,0);

/*
represents the encoded mouse control message sent by the TRANSMITTER board.
formatted closely to what is expected by the HID descriptor for a mouse.
syntax:
transmision_message[0] = mouse click commands
transmision_message[1] = x movement
transmision_message[2] = y movement
transmision_message[3] = scroll wheel movement
transmision_message[4] = transmission_num
*/
uint8_t transmision_message[5] = {0,0,0,0,0};

// number to append to transmision_message (transmision_message[4])
uint8_t transmission_num = 0;

uint8_t outOfOrderPackets = 0;

//Brief:		Starts the mouse thread in the OS (from Inactive into the Lifecycle)
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void start_mouse_thread(void *args) {
	mouse_thread_ID = osThreadCreate(osThread(mouse_thread), args);
}

// Wait for transmission to start.
// initial content of the transmission_message is 0
void receiver_waitForSignal(void){
	while (transmision_message[4] == 0){
		CC2500_ReceivePacket(transmision_message);
		osDelay(MOUSE_CONTROL_PERIOD);
	}
}

void sendMouseControlsOverUSB(void){
	// toggle led lights to indicate receiving in-order packets
	LED_thread_signal();
	USBD_HID_GetReportTrigger(0, 0, transmision_message, 4);
}

//Brief:		Updates the mouse controls
// 					For TRANSMITTER, sends packet with mouse controls. Appends transmission_num to the transmitted packet.
//					For RECEIVER, receives packet and sends through USB. Handles connection issues via checking transmission_num.
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void update_mouse_state(void){
	#ifdef TRANSMITTER
		// append transmission num to the packet
		transmision_message[4] = transmission_num;
		CC2500_SendPacket(transmision_message);
		// update transmission num
		transmission_num = (transmission_num + 1) & 0xff;
		// toggle led lights to indicate sending something
		LED_thread_signal();
	#endif
		
	#ifdef RECEIVER
		receiver_waitForSignal();
		CC2500_ReceivePacket(transmision_message);
		// check transmission num; update only if (in pseudocode) {new transmission num == old transmission num + 1}
		if (((transmission_num + 1) & 0xff) == (transmision_message[4]) || transmision_message[4] == 0){
			outOfOrderPackets = 0;
			sendMouseControlsOverUSB();
		} else {
			outOfOrderPackets++;
			if (outOfOrderPackets < NUMPACKETS_OUTOFORDER_THRESHOLD)
				sendMouseControlsOverUSB();
		}
		// update transmission num
		transmission_num = transmision_message[4];
	#endif
}

//Brief:		The mouse thread function in the OS
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void mouse_thread(void const *args) {
	while(1) {
		osSignalWait(OS_THREAD_SIGNAL_1, osWaitForever);
		update_mouse_state();
	}
}

/**
   * @brief hardware interrupt when the TIM3 timer has elapsed
   * @retval None
   */
void mouse_tim_elapsed_handler(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		osSignalSet(mouse_thread_ID, OS_THREAD_SIGNAL_1);
	}
}
