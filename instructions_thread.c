////////////////////////////////////////////////////////////////////////////////
//	File Name					: LED_thread.c
//	Description				: Example of an OS thread that toggles the board LED's
//											based on a 1 second interrupt from TIM3
//	Author						: Harsh Aurora
//	Date							: Oct 28, 2016
////////////////////////////////////////////////////////////////////////////////
	
//		Includes		//
#include <instructions_thread.h> 
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

//		Function Declaration		//
void instructions_thread(void const *args);

//		Globals 		//
osThreadId instructions_thread_ID;
osThreadDef(instructions_thread, osPriorityNormal, 1,0);
TIM_HandleTypeDef TIM3_handle;





GPIO_PinState PinC1;
GPIO_PinState PinC2;
GPIO_PinState PinC3;
GPIO_PinState PinC4;
GPIO_PinState PinR1;
GPIO_PinState PinR2;
GPIO_PinState PinR3;
GPIO_PinState PinR4;
uint8_t temp;
uint8_t data;

int leftFlag = 0;
int rightFlag = 0;
int middleFlag = 0;

//Brief:		Initializes the GPIO and TIM periphs used in this example
//					GPGIOs : D12, D13, D14, D15 as output (LED GPIOs)
//					TIM:	TIM3 to provide 1 second interrupts
//Params:		None
//Return:		None
void instructions_thread_periph_init(void) {


	GPIO_InitTypeDef GPIO_InitE;
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitE.Pin = Col1 | Col2 | Col3 | Row1 | Row2 | Row3 | Row4;
	GPIO_InitE.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitE.Pull = GPIO_PULLUP;
	GPIO_InitE.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitE);

	
	__HAL_RCC_TIM3_CLK_ENABLE();
	TIM3_handle.Instance = TIM3;
	TIM3_handle.Init.Prescaler					= 20999;        
	TIM3_handle.Init.CounterMode				= TIM_COUNTERMODE_DOWN;     
	TIM3_handle.Init.Period							= 3999;           
	TIM3_handle.Init.ClockDivision			= TIM_CLOCKDIVISION_DIV1;    
	TIM3_handle.Init.RepetitionCounter	= 0;
	
	HAL_TIM_Base_Init(&TIM3_handle);
	HAL_TIM_Base_Start_IT(&TIM3_handle);
	
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
}



void Keypad_Columninput(void){
	HAL_GPIO_WritePin(GPIOE, Row1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Row2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Row3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Row4, GPIO_PIN_RESET);
}

/*Brief: Set up each Row for the keypad
**Params: None
**Return: None
*/

void Keypad_Rowinput(void){	
	HAL_GPIO_WritePin(GPIOE, Col1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Col3, GPIO_PIN_RESET);
}

/*Brief: Read the number of the Keypad
**Params: None
**Return: Input number as int
*/
int Keypad_Read(void){
	Keypad_Columninput();
	PinC1 = HAL_GPIO_ReadPin(GPIOE, Col1);
	PinC2 = HAL_GPIO_ReadPin(GPIOE, Col2);
	PinC3 = HAL_GPIO_ReadPin(GPIOE, Col3);
	

	Keypad_Rowinput();
	PinR1 = HAL_GPIO_ReadPin(GPIOE, Row1);
	PinR2 = HAL_GPIO_ReadPin(GPIOE, Row2);
	PinR3 = HAL_GPIO_ReadPin(GPIOE, Row3);
	PinR4 = HAL_GPIO_ReadPin(GPIOE, Row4);
	
	if(PinR1 && PinC1 == 1) return 1;
	if(PinR1 && PinC2 == 1) return 2;
	if(PinR1 && PinC3 == 1) return 3;
	if(PinR2 && PinC1 == 1) return 4;
	if(PinR2 && PinC2 == 1) return 5;
	if(PinR2 && PinC3 == 1) return 6;
	if(PinR3 && PinC1 == 1) return 7;
	if(PinR3 && PinC2 == 1) return 8;
	if(PinR3 && PinC3 == 1) return 9;
	
	if(PinR4 && PinC1 == 1) return 10;
	if(PinR4 && PinC2 == 1) return 0;
	if(PinR4 && PinC3 == 1) return 10;
	
	return -1;
}




//Brief:		Starts the LED thread in the OS (from Inactive into the Lifecycle)
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void start_instructions_thread(void *args) {
	instructions_thread_ID = osThreadCreate(osThread(instructions_thread), args);
}

//Brief:		The LED thread function in the OS
//					Waits for a signal from the TIM3 interrupt handler and then 
//					toggles the on board LEDs
//Params:		A void pointer to initial arguments, NULL if unused
//Return:		None
void instructions_thread(void const *args) {
	
	while(1) {
		osDelay(10);
		if(Keypad_Read() == 1){
			leftFlag = 1;
		}
		//if press 2, display roll angle
		else if(Keypad_Read() == 2) rightFlag = 1;
		//if press 3, display pitch angle
		else if(Keypad_Read() == 3) middleFlag = 1;
		//if press 4, reset the display.
		else {
			leftFlag = 0;
			rightFlag = 0;
			middleFlag = 0;
		}
	}
}

//Brief:		The TIM interrupt callback. Sends a signal to the LED_thread
//					if the interrupt was recived from TIM3
//Params:		Pointer to the TIM handle that caused the interrupt
//Return:		None
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	
	if(htim->Instance == TIM3)
		osSignalSet(instructions_thread_ID, 0x00000001);
}
