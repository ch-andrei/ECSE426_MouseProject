/******************************************************************************
  * @file    segments.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   code to initialize and display numbers on the 7 segments display
  */

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
#include "data.h"
#include "main.h"
#include "segments.h"
#include <cmsis_os.h>

// global temperature alert flag
int temperature_alert_flag_g = 0;

/* LOCAL VARIABLES */

// temperature alert counter and flag
int temperature_alert_flag_l;
int temperature_alert_counter;

// current temperature displayed on segments
extern float current_value;

// counter to update currently displayed temperature
extern int update_value_flag;

// current selector for segment to update (4 segments: s0, s1, s2, s3) 
extern int update_segment;

// value format X.XX, XX.X, XXX
extern int value_scale;

/**
   * @brief Configures GPIO for 7-segment display
   * @retval None
   */
void segments_configure(void){
	GPIO_InitTypeDef GPIO_InitC;
	GPIO_InitTypeDef GPIO_InitE;
	
	// turn on clocks for C and E GPIO
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// init pints //
	GPIO_InitE.Pin = A | B | C | D | E | F | G | DP;
	GPIO_InitE.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitE.Pull = GPIO_PULLUP;
	GPIO_InitE.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitE);
	
	GPIO_InitC.Pin = TENS | ONES | DECIMAL | DEGREE;
	GPIO_InitC.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitC.Pull = GPIO_PULLUP;
	GPIO_InitC.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitC);
	return;
}

/**
   * @brief Numerical digit Conventions (0 - 9) for 7-segment display
   * @param  number: one-digit number to display
   * @retval None
   */
void draw_number(int number){
	switch(number){
		case 0:		/* SET: ABCDEF */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_RESET);
			break;
		case 1:		/* SET: BC */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_RESET);
			break;
		case 2:		/* SET: ABDEG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 3:		/* SET: ABCDG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 4:		/* SET: BCFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 5:		/* SET: ACDFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 6:		/* SET: ACDEFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 7:		/* SET: ABC */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_RESET);
			break;
		case 8:		/* SET:	ABCDEFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 9:		/* SET: ABCDFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		case 'o':
			/* SET: ABFG */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, G, GPIO_PIN_SET);
			break;
		default:	/* Reset all pins */
			HAL_GPIO_WritePin(GPIOE, A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, C, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, D, GPIO_PIN_RESET);
			break;
	}
	return;
}

/**
   * @brief Select which segment will display a number
   * @param  digit: selector for segment #
   * @retval None
   */
void dispay_segment_select(int select){
	switch(select){
		case 1:
			HAL_GPIO_WritePin(GPIOC, TENS, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, ONES, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DECIMAL, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DEGREE, GPIO_PIN_RESET);
			if (value_scale == 0)
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_SET);
			else 
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_RESET);
			break;
		case 2:	/* both units digit and decimal point are on */
			HAL_GPIO_WritePin(GPIOC, TENS, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, ONES, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, DECIMAL, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DEGREE, GPIO_PIN_RESET);
			if (value_scale == 1)
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOC, TENS, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, ONES, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DECIMAL, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, DEGREE, GPIO_PIN_RESET);
			if (value_scale == 2)
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_SET);
			else 
				HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_RESET);
			break;
		case 4:	/* set degree symbol as rightmost digit */
			HAL_GPIO_WritePin(GPIOC, TENS, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, ONES, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DECIMAL, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DEGREE, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_RESET);
			break;
		default:
			HAL_GPIO_WritePin(GPIOC, TENS, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, ONES, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, DP, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DECIMAL, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, DEGREE, GPIO_PIN_RESET);
		break;
	}
	return;
}

// displays a digit to a segment
void display_digit(int digit, int selector){
	if ((digit < 0 || digit >= 10) && digit != 'o') 
		digit = 0;
	if (temperature_alert_flag_l){
		selector = -1;
		digit = -1;
	}
	dispay_segment_select(selector);
	draw_number(digit);
	return;
}

// update temperature reading to be displayed (only updates variable, not the actual physically displayed value)
void segments_update_value(float value){
	if (value >= 100)
		value_scale = 2;
	else if (value >= 10)
		value_scale = 1;
	else
		value_scale = 0;
	if (update_value_flag == 0)
		current_value = value; 
	update_value_flag = (update_value_flag + 1) % SEGMENTS_VALUE_UPDATE_PERIOD_DEFAULT;
	return;
}

// force displayed output to this value
void segments_force_display(float value){
	update_value_flag = 0;
	segments_update_display(value);
	return;
}

// update temperature alert state
void temperature_alert_update(void){
	if (temperature_alert_flag_g == TEMPERATURE_ALERT_ON){
		if (temperature_alert_flag_l == TEMPERATURE_ALERT_ON){
			if (temperature_alert_counter > TEMPERATURE_ALERT_COUNTER_ALERT_ON){
				temperature_alert_flag_l = TEMPERATURE_ALERT_OFF;
			}
		} else if (temperature_alert_flag_l == TEMPERATURE_ALERT_OFF){
			if (temperature_alert_counter < TEMPERATURE_ALERT_COUNTER_ALERT_ON){
				temperature_alert_flag_l = TEMPERATURE_ALERT_ON;
			}
		}
	} else {
		temperature_alert_flag_l = TEMPERATURE_ALERT_OFF;
	}
	temperature_alert_counter = (temperature_alert_counter + 1) % TEMPERATURE_ALERT_COUNTER_MODULO;
}

// update segment display one by one; must be called 4 times for full update
void segments_update_display(float value){		
	segments_update_value(value);
	temperature_alert_update();
	int tens, ones, decimal;
	// which segment t update
	float multiplier = 1;
	switch (value_scale){
		default:
			break;
		case 0:
			multiplier = 10;
			break;
		case 1:
			multiplier = 1;
			break;
		case 2:
			multiplier = 0.1;
			break;
	}
	float adjusted_temp = multiplier * current_value;
	switch (update_segment){
		case 0:
			tens = adjusted_temp / 10;
			display_digit(tens, 1);
			break;
		case 1:
		ones = (int)adjusted_temp % 10;
			display_digit(ones, 2);
			break;
		case 2:
			decimal = (int)(adjusted_temp * 10) % 10;	
			display_digit(decimal, 3);
			break;
		case 3:
			display_digit('o', 4);
			break;
		default:
			break;
	}
	update_segment = (update_segment + 1) % NUMBER_OF_SEGMENTS;
	return;
}
