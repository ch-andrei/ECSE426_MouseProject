/**  
  ******************************************************************************
  * @file    keypad.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   Implementation of the 4x4 8-pin keypad; MODIFIED for 4x3 Keypad
  */

#include "supporting_functions.h"
#include "data.h"
#include "keypad.h"

// Pin state declarations
GPIO_PinState PinCol1;
GPIO_PinState PinCol2;
GPIO_PinState PinCol3;
GPIO_PinState PinCol4;
GPIO_PinState PinRow1;
GPIO_PinState PinRow2;
GPIO_PinState PinRow3;
GPIO_PinState PinRow4;

/**
   * @brief Configures GPIO for keypad
   * @retval None
   */
void keypad_configure(void){
	GPIO_InitTypeDef GPIO_InitE;
	KEYPAD_GPIO_CLK_ENABLE();
	GPIO_InitE.Pin = Col1 | Col2 | Col3 | Row1 | Row2 | Row3 | Row4; // removed Col4
	GPIO_InitE.Speed = GPIO_SPEED_HIGH;
	GPIO_InitE.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitE.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(KEYPAD_GPIOX, &GPIO_InitE);
}

/**
   * @brief Read the number from the keypad and account for key bouncing
   * @retval the number entered by using keypad
   */
uint8_t keypad_read_debounce(uint8_t *var, uint8_t debounce){
	uint8_t temp_key = keypad_read();
	if (temp_key == keypad_read()){
		*var = temp_key;
		if (debounce)
			while (keypad_read() >= 0);
		return 0;
	}
	return -1;
}

/**
   * @brief Setup each column of the keypad
   * @retval None
   */
void keypad_poll_column(void){
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Row1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Row2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Row3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Row4, GPIO_PIN_RESET);
}

/**
   * @brief Setup each row of the keypad
   * @retval None
   */
void keypad_poll_row(void){	
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Col1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Col2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIOX, Col3, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(KEYPAD_GPIOX, Col4, GPIO_PIN_RESET);
}

/**
   * @brief Read the number from the keypad
   * @retval the number entered by using keypad
   */
uint8_t keypad_read(void){
	// get column data
	uint8_t temp, data;
	
	keypad_poll_column();
	PinCol1 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Col1);
	PinCol2 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Col2);
	PinCol3 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Col3);
	PinCol4 = GPIO_PIN_RESET; // hardcoded
	temp = (((PinCol4 == GPIO_PIN_RESET) << 3) +((PinCol3 == GPIO_PIN_RESET) << 2) + ((PinCol2 == GPIO_PIN_RESET) << 1) + ((PinCol1 == GPIO_PIN_RESET)));
	data = temp<<4;
	
	// get row data
	keypad_poll_row();
	PinRow1 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Row1);
	PinRow2 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Row2);
	PinRow3 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Row3);
	PinRow4 = HAL_GPIO_ReadPin(KEYPAD_GPIOX,Row4);
	temp = (((PinRow4 == GPIO_PIN_RESET) << 3) +((PinRow3 == GPIO_PIN_RESET) << 2) + ((PinRow2 == GPIO_PIN_RESET) << 1) + ((PinRow1 == GPIO_PIN_RESET)));
	data |= temp;
	
	if(data != 0xFF){
		switch(data){
			// match key with return code
			default:
				return -2;
			case 0xD7:
				return 0;
			case 0xEE:
				return 1;
			case 0xDE:
				return 2;
			case 0xBE:
				return 3;
			case 0xED:
				return 4;
			case 0xDD:
				return 5;
			case 0xBD:
				return 6;
			case 0xEB:
				return 7;
			case 0xDB:
				return 8;
			case 0xBB:
				return 9;
			case 0x7E: // A
				return 10;
			case 0x7D: // B
				return 11;
			case 0x7B: // C
				return 12;
			case 0x77: // D
				return 13;
			case 0xE7: // *
				return 14;
			case 0xB7: // #
				return 15;
		}
	}
	else{
		return -1;
	}
}
